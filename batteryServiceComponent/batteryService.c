/**
 *
 * @file batteryService.c
 *
 * This file provides control and monitoring of the power supply battery via the @ref c_battery and
 * the Data Hub.
 *
 * <hr>
 *
 * Copyright (C) Sierra Wireless Inc.
 */

#include "legato.h"
#include "interfaces.h"
#include "batteryUtils.h"

#define BATTERY_SAMPLE_INTERVAL_IN_MILLISECONDS 10000

static const char ChargerStr[] = "/sys/bus/i2c/devices/%d-006b/power_supply/bq24190-charger/%s";
static const char HealthStr[]  = "health";
static const char BatteryStr[] = "/sys/bus/i2c/devices/%d-006b/power_supply/bq24190-battery/%s";
static const char StatusStr[]  = "status";
static const char MonitorStr[] = "/sys/bus/i2c/devices/%d-0064/power_supply/LTC2942/%s";
static const char VoltageStr[] = "voltage_now";
static const char TempStr[]    = "temp";
static const char ChargeNowStr[] = "charge_now";
static const char PresenceStr[]  = "charge_counter";

static le_mem_PoolRef_t LevelAlarmPool;
static le_ref_MapRef_t LevelAlarmRefMap;

static le_mem_PoolRef_t ChargingStatusRegPool;
static le_ref_MapRef_t ChargingStatusRegRefMap;

static le_mem_PoolRef_t HealthStatusRegPool;
static le_ref_MapRef_t HealthStatusRegRefMap;


#define RES_PATH_TECH        "tech"
#define RES_PATH_CAPACITY    "capacity"
#define RES_PATH_NOM_VOLTAGE "nominalVoltage"
#define RES_PATH_VOLTAGE     "voltage"
#define RES_PATH_HEALTH      "health"
#define RES_PATH_PERCENT     "percent"
#define RES_PATH_ENERGY      "energy"
#define RES_PATH_CHARGING    "charging"
#define RES_PATH_TEMPERATURE "temperature"


typedef enum
{
    LEVEL_HIGH,
    LEVEL_LOW,
    LEVEL_NONE,
}
LevelAlarmType_t;

typedef struct
{
    uint8_t percentageHigh;
    uint8_t percentageLow;
    LevelAlarmType_t lastAlarmType;

    ma_battery_LevelPercentageHandlerFunc_t handler;
    void *clientContext;
    le_msg_SessionRef_t clientSessionRef;
}
LevelAlarmReg_t;

typedef struct
{
    ma_battery_ChargingStatusHandlerFunc_t handler;
    void *clientContext;
    le_msg_SessionRef_t clientSessionRef;
}
ChargingStatusReg_t;

typedef struct
{
    ma_battery_HealthHandlerFunc_t handler;
    void *clientContext;
    le_msg_SessionRef_t clientSessionRef;
}
HealthStatusReg_t;


//--------------------------------------------------------------------------------------------------
/**
 * Register a callback function to be called when the Percentage Level changes as follows:
 *
 * If the level goes above levelHigh raise an alarm, if the level goes below levelLow raise
 * an alarm.
 */
//--------------------------------------------------------------------------------------------------
ma_battery_LevelPercentageHandlerRef_t ma_battery_AddLevelPercentageHandler
(
    uint8_t percentageLow,  ///< Low percentage trigger threshold (0 = no low level alarm)
    uint8_t percentageHigh, ///< High percentage trigger threshold (>=100 means no high alm)
    ma_battery_LevelPercentageHandlerFunc_t handler,
    void *context
)
{
    if (percentageHigh > 100)
    {
        LE_ERROR("High percentage can't be higher than 100");
        return NULL;
    }
    else if (percentageHigh < percentageLow)
    {
        LE_ERROR("High percentage can't be less than low percentage");
        return NULL;
    }

    LevelAlarmReg_t *reg = le_mem_ForceAlloc(LevelAlarmPool);
    reg->percentageLow                 = percentageLow;
    reg->percentageHigh                = percentageHigh;
    reg->lastAlarmType                 = LEVEL_NONE;
    reg->handler                       = handler;
    reg->clientContext                 = context;
    reg->clientSessionRef              = ma_battery_GetClientSessionRef();

    return le_ref_CreateRef(LevelAlarmRefMap, reg);
}


//--------------------------------------------------------------------------------------------------
/**
 * Deregister a callback function registered using ma_battery_AddLevelPercentageHandler().
 */
//--------------------------------------------------------------------------------------------------
void ma_battery_RemoveLevelPercentageHandler
(
    ma_battery_LevelPercentageHandlerRef_t handlerRef
)
{
    LevelAlarmReg_t *reg = le_ref_Lookup(LevelAlarmRefMap, handlerRef);
    if (reg == NULL)
    {
        LE_ERROR("Failed to lookup event based on handle");
    }
    else
    {
        if (reg->clientSessionRef == ma_battery_GetClientSessionRef())
        {
            le_ref_DeleteRef(LevelAlarmRefMap, handlerRef);
            le_mem_Release(reg);
        }
        else
        {
            LE_ERROR("Remove invalid event handleRef");
        }
    }
}


static void RemoveAllLevelAlarmHandlersOwnedByClient
(
    le_msg_SessionRef_t owner
)
{
    le_ref_IterRef_t it = le_ref_GetIterator(LevelAlarmRefMap);

    bool finished = le_ref_NextNode(it) != LE_OK;
    while (!finished)
    {
        LevelAlarmReg_t *reg = le_ref_GetValue(it);
        LE_ASSERT(reg != NULL);
        // In order to prevent invalidating the iterator, we store the reference of the device we
        // want to close and advance the iterator before calling le_spi_Close which will remove the
        // reference from the hashmap.
        void *ref = (void *)le_ref_GetSafeRef(it);
        finished  = le_ref_NextNode(it) != LE_OK;
        if (reg->clientSessionRef == ref)
        {
            le_ref_DeleteRef(LevelAlarmRefMap, ref);
            le_mem_Release(reg);
        }
    }
}


//--------------------------------------------------------------------------------------------------
/**
 * Checks the list of registered battery level alarm thresholds to see if an alarm event should be
 * reported.  If so, reports them.
 */
//--------------------------------------------------------------------------------------------------
static void CheckBatteryLevelAlarm
(
    unsigned int batteryPercentage
)
{
    le_ref_IterRef_t it = le_ref_GetIterator(LevelAlarmRefMap);
    bool finished = (le_ref_NextNode(it) != LE_OK);
    while (!finished)
    {
        LevelAlarmReg_t *reg = le_ref_GetValue(it);
        LE_ASSERT(reg != NULL);
        if ((batteryPercentage > reg->percentageHigh) && (reg->lastAlarmType != LEVEL_HIGH))
        {
            reg->handler(batteryPercentage, reg->percentageHigh, true, reg->clientContext);
            reg->lastAlarmType = LEVEL_HIGH;
        }
        else if ((batteryPercentage < reg->percentageLow) && (reg->lastAlarmType != LEVEL_LOW))
        {
            reg->handler(batteryPercentage, reg->percentageLow, false, reg->clientContext);
            reg->lastAlarmType = LEVEL_LOW;
        }

        finished = (le_ref_NextNode(it) != LE_OK);
    }
}


ma_battery_ChargingStatusChangeHandlerRef_t ma_battery_AddChargingStatusChangeHandler
(
    ma_battery_ChargingStatusHandlerFunc_t handler,
    void *context
)
{
    ChargingStatusReg_t *reg = le_mem_ForceAlloc(ChargingStatusRegPool);

    reg->handler                        = handler;
    reg->clientContext                  = context;
    reg->clientSessionRef               = ma_battery_GetClientSessionRef();

    return le_ref_CreateRef(ChargingStatusRegRefMap, reg);
}


void ma_battery_RemoveChargingStatusChangeHandler
(
    ma_battery_ChargingStatusChangeHandlerRef_t handlerRef
)
{
    ChargingStatusReg_t *reg = le_ref_Lookup(ChargingStatusRegRefMap, handlerRef);
    if (reg == NULL)
    {
        LE_ERROR("Failed to lookup event based on handle %p", handlerRef);
    }
    else
    {
        if (reg->clientSessionRef == ma_battery_GetClientSessionRef())
        {
            le_ref_DeleteRef(ChargingStatusRegRefMap, handlerRef);
            le_mem_Release(reg);
        }
        else
        {
            LE_ERROR("Attempt to remove another client's Alarm Health event handleRef %p",
                     handlerRef);
        }
    }
}


static void RemoveAllChargeAlarmHandlersOwnedByClient
(
    le_msg_SessionRef_t owner
)
{
    le_ref_IterRef_t it = le_ref_GetIterator(ChargingStatusRegRefMap);

    bool finished = le_ref_NextNode(it) != LE_OK;
    while (!finished)
    {
        ChargingStatusReg_t *reg = le_ref_GetValue(it);
        LE_ASSERT(reg != NULL);
        // In order to prevent invalidating the iterator, we store the reference of the device we
        // want to close and advance the iterator before calling le_spi_Close which will remove the
        // reference from the hashmap.
        void *ref = (void *)le_ref_GetSafeRef(it);
        finished  = le_ref_NextNode(it) != LE_OK;
        if (reg->clientSessionRef == ref)
        {
            le_ref_DeleteRef(ChargingStatusRegRefMap, ref);
            le_mem_Release(reg);
        }
    }
}


//--------------------------------------------------------------------------------------------------
/**
 * Report a change in the charging status to any registered charging status handlers.
 */
//--------------------------------------------------------------------------------------------------
static void ReportChargingStatusChange
(
    ma_battery_ChargingStatus_t status
)
{
    le_ref_IterRef_t it = le_ref_GetIterator(ChargingStatusRegRefMap);
    bool finished       = le_ref_NextNode(it) != LE_OK;
    while (!finished)
    {
        ChargingStatusReg_t *reg = le_ref_GetValue(it);
        LE_ASSERT(reg != NULL);

        reg->handler(status, reg->clientContext);
        finished = (le_ref_NextNode(it) != LE_OK);
    }
}


ma_battery_HealthChangeHandlerRef_t ma_battery_AddHealthChangeHandler
(
    ma_battery_HealthHandlerFunc_t handler,
    void *context
)
{
    HealthStatusReg_t *reg = le_mem_ForceAlloc(HealthStatusRegPool);
    reg->handler                        = handler;
    reg->clientContext                  = context;
    reg->clientSessionRef               = ma_battery_GetClientSessionRef();

    return le_ref_CreateRef(HealthStatusRegRefMap, reg);
}


void ma_battery_RemoveHealthChangeHandler
(
    ma_battery_HealthChangeHandlerRef_t handlerRef
)
{
    HealthStatusReg_t *reg = le_ref_Lookup(HealthStatusRegRefMap, handlerRef);
    if (reg == NULL)
    {
        LE_ERROR("Failed to lookup event based on handle %p", handlerRef);
    }
    else
    {
        if (reg->clientSessionRef == ma_battery_GetClientSessionRef())
        {
            le_ref_DeleteRef(HealthStatusRegRefMap, handlerRef);
            le_mem_Release(reg);
        }
        else
        {
            LE_ERROR("Attempt to remove another client's Alarm Health event handleRef %p",
                     handlerRef);
        }
    }
}


static void RemoveAllHealthAlarmHandlersOwnedByClient
(
    le_msg_SessionRef_t owner
)
{
    le_ref_IterRef_t it = le_ref_GetIterator(HealthStatusRegRefMap);

    bool finished = le_ref_NextNode(it) != LE_OK;
    while (!finished)
    {
        HealthStatusReg_t *reg = le_ref_GetValue(it);
        LE_ASSERT(reg != NULL);
        // In order to prevent invalidating the iterator, we store the reference of the device we
        // want to close and advance the iterator before calling le_spi_Close which will remove the
        // reference from the hashmap.
        void *ref = (void *)le_ref_GetSafeRef(it);
        finished  = le_ref_NextNode(it) != LE_OK;
        if (reg->clientSessionRef == ref)
        {
            le_ref_DeleteRef(HealthStatusRegRefMap, ref);
            le_mem_Release(reg);
        }
    }
}


//--------------------------------------------------------------------------------------------------
/**
 * Reports a change in the health status to any registered health status change event handlers.
 */
//--------------------------------------------------------------------------------------------------
static void ReportHealthStatusChange
(
    ma_battery_HealthStatus_t status
)
{
    le_ref_IterRef_t it = le_ref_GetIterator(HealthStatusRegRefMap);
    bool finished       = le_ref_NextNode(it) != LE_OK;
    while (!finished)
    {
        HealthStatusReg_t *reg = le_ref_GetValue(it);
        LE_ASSERT(reg != NULL);
        reg->handler(status, reg->clientContext);
        finished = le_ref_NextNode(it) != LE_OK;
    }
}


//--------------------------------------------------------------------------------------------------
/**
 * A handler for client disconnects which frees all resources associated with the client.
 */
//--------------------------------------------------------------------------------------------------
static void ClientSessionClosedHandler
(
    le_msg_SessionRef_t clientSession,
    void *context
)
{
    RemoveAllLevelAlarmHandlersOwnedByClient(clientSession);
    RemoveAllChargeAlarmHandlersOwnedByClient(clientSession);
    RemoveAllHealthAlarmHandlersOwnedByClient(clientSession);
}


//--------------------------------------------------------------------------------------------------
/**
 * Set the battery technology as set by the battery manufacturer
 *
 * @note this function sets the battery parameters and although is optional but is good to have
 */
//--------------------------------------------------------------------------------------------------
void ma_adminbattery_SetTechnology
(
    const char *batteryType,
    uint32_t mAh,
    uint32_t milliVolts
)
{
    LE_DEBUG(" Create battery configuration");

    // Create a write transaction so we can update the tree
    le_cfg_IteratorRef_t iteratorRef = le_cfg_CreateWriteTxn("batteryInfo");

    // Set the integer for the battery as it is an enum
    le_cfg_SetString(iteratorRef, "type", batteryType);

    // Set the battery capacity as set by the manufacturer
    le_cfg_SetInt(iteratorRef, "capacity", mAh);

    // Set the voltage rating as set by the manufacturer in milliVolts
    le_cfg_SetInt(iteratorRef, "voltage", milliVolts);

    // Commit the transaction to make sure new settings are written to config tree
    le_cfg_CommitTxn(iteratorRef);

    // Update this info in the Data Hub.
    dhubIO_SetStringDefault(RES_PATH_TECH, batteryType);
    dhubIO_SetNumericDefault(RES_PATH_NOM_VOLTAGE, ((double)milliVolts) / 1000.0);
    dhubIO_SetNumericDefault(RES_PATH_CAPACITY, mAh);
}


//--------------------------------------------------------------------------------------------------
/**
 * Get the battery technology as set by the battery manufacturer
 *
 * @note this function gets the battery parameters and although is optional but is good to have
 */
//--------------------------------------------------------------------------------------------------
le_result_t ma_battery_GetTechnology
(
    char *batteryType,
    size_t batteryTypeSize, ///< Size of buffer pointed to by batteryType (bytes).
    uint16_t *capacityPtr,
    uint16_t *voltagePtr
)
{
    // Create a read transaction
    le_cfg_IteratorRef_t iteratorRef = le_cfg_CreateReadTxn("batteryInfo");

    // Get the name for the battery type
    le_result_t result = le_cfg_GetString(iteratorRef, "type", batteryType, batteryTypeSize, "");
    if (result != LE_OK)
    {
        LE_ERROR("Cannot get battery type (%s)", LE_RESULT_TXT(result));
        goto cleanup;
    }

    // Get the battery capacity in mAh (or -1 if not found)
    int32_t capacity = le_cfg_GetInt(iteratorRef, "capacity", -1);
    if (capacity < 0)
    {
        LE_ERROR("Cannot get battery capacity");
        result = LE_NOT_FOUND;
        goto cleanup;
    }
    *capacityPtr = (uint16_t)capacity;

    // Get the battery voltage in mV (or -1 if not found)
    int32_t voltage = le_cfg_GetInt(iteratorRef, "voltage", -1);
    if (voltage < 0)
    {
        LE_ERROR("Cannot get battery voltage");
        result = LE_NOT_FOUND;
        goto cleanup;
    }
    *voltagePtr = (uint16_t)voltage;

cleanup:

    le_cfg_CancelTxn(iteratorRef);
    return result;
}


//--------------------------------------------------------------------------------------------------
/**
 * Provides battery health status
 *
 * @return Health status code.
 */
//--------------------------------------------------------------------------------------------------
ma_battery_HealthStatus_t ma_battery_GetHealthStatus(void)
{
    char path[256];
    int pathLen = snprintf(path, sizeof(path), ChargerStr, MANGOH_I2C_BUS_BATTERY, HealthStr);
    LE_ASSERT(pathLen < sizeof(path));

    char healthValue[512];
    le_result_t r = ReadStringFromFile(path, healthValue, sizeof(healthValue));
    LE_DEBUG(" health = %s", healthValue);
    if (r == LE_OK)
    {
        if (strcmp(healthValue, "Good") == 0)
        {
            return MA_BATTERY_GOOD;
        }
        else if (strcmp(healthValue, "Overvoltage") == 0)
        {
            return MA_BATTERY_OVERVOLTAGE;
        }
        else if (strcmp(healthValue, "Cold") == 0)
        {
            return MA_BATTERY_COLD;
        }

        else if (strcmp(healthValue, "Overheat") == 0)
        {
            return MA_BATTERY_HOT;
        }
        else
        {
            return MA_BATTERY_HEALTHUNDEFINED;
        }
    }
    else
    {
        return MA_BATTERY_HEALTHERROR;
    }
}


//--------------------------------------------------------------------------------------------------
/**
 * Provides battery charging status
 *
 * @return Charging status code.
 */
//--------------------------------------------------------------------------------------------------
ma_battery_ChargingStatus_t ma_battery_GetChargingStatus(void)
{
    char path[256];
    int pathLen = snprintf(path, sizeof(path), BatteryStr, MANGOH_I2C_BUS_BATTERY, StatusStr);
    LE_ASSERT(pathLen < sizeof(path));

    char chargingStatus[512];
    le_result_t r = ReadStringFromFile(path, chargingStatus, sizeof(chargingStatus));

    if (r == LE_OK)
    {
        if (strcmp(chargingStatus, "Discharging") == 0)
        {
            return MA_BATTERY_DISCHARGING;
        }
        else if (strcmp(chargingStatus, "Charging") == 0)
        {
            return MA_BATTERY_CHARGING;
        }
        else if (strcmp(chargingStatus, "Full") == 0)
        {
            return MA_BATTERY_FULL;
        }
        else
        {
            LE_ERROR("Unrecognized charging status '%s'.", chargingStatus);

            return MA_BATTERY_HEALTHUNDEFINED;
        }
    }
    else
    {
        LE_DEBUG("failed to read the charging status (%s).", LE_RESULT_TXT(r));

        return MA_BATTERY_CHARGEERROR;
    }
}


//--------------------------------------------------------------------------------------------------
/**
 * Get battery voltage (in Volts)
 *
 * @return
 *      - LE_OK on success.
 */
//--------------------------------------------------------------------------------------------------
le_result_t ma_battery_GetVoltage
(
    double *volt
)
{
    char path[256];

    int pathLen = snprintf(path, sizeof(path), MonitorStr, MANGOH_I2C_BUS_BATTERY, VoltageStr);
    LE_ASSERT(pathLen < sizeof(path));

    int32_t mV;
    le_result_t r = ReadIntFromFile(path, &mV);
    if (r == LE_OK)
    {
        *volt = ((double)mV) / 1000.0;
    }

    return r;
}


//--------------------------------------------------------------------------------------------------
/**
 * Get battery temperature in degrees Celcius
 *
 * @return
 *      - LE_OK on success.
 */
//--------------------------------------------------------------------------------------------------
le_result_t ma_battery_GetTemp
(
    double *temp    ///< degrees C
)
{
    char path[256];

    int pathLen = snprintf(path, sizeof(path), MonitorStr, MANGOH_I2C_BUS_BATTERY, TempStr);
    LE_ASSERT(pathLen < sizeof(path));

    int32_t tempcalc;  // In centidegrees Celcius.
    le_result_t r = ReadIntFromFile(path, &tempcalc);
    if (r == LE_OK)
    {
        *temp = ((double)tempcalc) / 100.0;
    }

    return r;
}


//--------------------------------------------------------------------------------------------------
/**
 * Get Charge Remaining in mAh
 *
 * @return
 *      - LE_OK
 *      - LE_IO_ERROR
 */
//--------------------------------------------------------------------------------------------------
le_result_t ma_battery_GetChargeRemaining
(
    uint16_t *charge    ///< mAh
)
{
    char path[256];

    int pathLen = snprintf(path, sizeof(path), MonitorStr, MANGOH_I2C_BUS_BATTERY, ChargeNowStr);
    LE_ASSERT(pathLen < sizeof(path));

    int32_t uAh;
    le_result_t r = ReadIntFromFile(path, &uAh);
    if (r == LE_OK)
    {
        *charge = uAh / 1000;
    }

    return r;
}


//--------------------------------------------------------------------------------------------------
/**
 * Get charge remaining, in percentage
 *
 * @return
 *      - LE_OK
 *      - LE_IO_ERROR
 *      - LE_NOT_FOUND
 */
//--------------------------------------------------------------------------------------------------
le_result_t ma_battery_GetPercentRemaining
(
    uint16_t *percentage
)
{
    int32_t capacity = le_cfg_QuickGetInt("batteryInfo/capacity", -1);

    if (capacity < 0)
    {
        LE_WARN("Battery capacity not configured");
        return LE_NOT_FOUND;
    }
    else
    {
        uint16_t remaining;
        le_result_t r = ma_battery_GetChargeRemaining(&remaining);
        if (r == LE_OK)
        {
            *percentage = (uint16_t)(100UL * remaining / capacity);
        }

        return r;
    }
}


//--------------------------------------------------------------------------------------------------
/**
 * Inform user of battery presence
 *
 * @return
 *      - true  if battery is present
 *      - false if no battery
 */
//--------------------------------------------------------------------------------------------------
bool ma_battery_Present(void)
{
    char path[256];

    int pathLen = snprintf(path, sizeof(path), MonitorStr, MANGOH_I2C_BUS_BATTERY, PresenceStr);
    LE_ASSERT(pathLen < sizeof(path));

    int32_t present;  // 0 = not present, 1 = present.
    le_result_t r = ReadIntFromFile(path, &present);
    if (r == LE_OK)
    {
        LE_DEBUG("value %d", present);

        if (present == 0)
        {
            LE_DEBUG("Battery not present");
            return false;
        }
        else
        {
            LE_DEBUG("Battery present");
            return true;
        }
    }
    else
    {
        LE_ERROR("Battery presence is indeterminate");
        return false;
    }
}


//--------------------------------------------------------------------------------------------------
/**
 * Write the present charge level to the battery monitoring driver.
 *
 * This is only done to correct the monitoring driver's idea of how much charge is presently stored
 * in the battery.  Normally the driver updates this itself as the battery drains and charges.
 */
//--------------------------------------------------------------------------------------------------
static void UpdateChargeLevel
(
    int mAh
)
{
    if (mAh > 0)
    {
        int uAh = mAh * 1000;

        LE_DEBUG("battery %d", uAh);

        char path[256];
        int len = snprintf(path, sizeof(path), MonitorStr, MANGOH_I2C_BUS_BATTERY, ChargeNowStr);
        LE_ASSERT(len < sizeof(path));
        WriteIntToFile(path, uAh);
    }
    else
    {
        LE_ERROR("Charge level invalid. (%d mAh)", mAh);
    }
}


//--------------------------------------------------------------------------------------------------
/**
 * Initialize the battery monitor state.
 */
//--------------------------------------------------------------------------------------------------
static void InitMonitoringState(void)
{
    // Read the battery technology configuration settings from the Config Tree.
    char type[MA_BATTERY_MAX_BATT_TYPE_STR_LEN + 1];
    uint16_t capacity; // mAh
    uint16_t voltage; // mV
    le_result_t result = ma_battery_GetTechnology(type, sizeof(type), &capacity, &voltage);
    if (result != LE_OK)
    {
        LE_ERROR("Battery monitor calibration will have to wait until configuration is performed.");
        return;
    }

    dhubIO_SetStringDefault(RES_PATH_TECH, type);
    dhubIO_SetNumericDefault(RES_PATH_NOM_VOLTAGE, ((double)voltage) / 1000);
    dhubIO_SetNumericDefault(RES_PATH_CAPACITY, capacity);

    // Read the present charge condition of the battery.
    ma_battery_ChargingStatus_t chargingStatus = ma_battery_GetChargingStatus();

    // If the battery is full,
    if (chargingStatus == MA_BATTERY_FULL)
    {
        LE_DEBUG("Battery is full");

        dhubIO_PushNumeric(RES_PATH_PERCENT, 0, 100.0);

        // Tell the battery monitoring driver that battery's present charge level is
        // equal to the maximum configured capacity.
        UpdateChargeLevel(capacity);

        // Update the Data Hub.
        dhubIO_PushNumeric(RES_PATH_ENERGY, 0, (double)capacity);
    }
    // But, if the battery is not full,
    else
    {
        LE_DEBUG("Battery not full");

        // Since we have no way of knowing what the actual charge level of the battery
        // is, tell the battery monitoring driver the battery's present charge is half its
        // maximum capacity.  When the battery charger later signals a "full" condition,
        // we'll update this again.  Otherwise, we let the battery monitoring driver update
        // it as the battery charges and drains.
        UpdateChargeLevel(capacity / 2);
    }
}


//--------------------------------------------------------------------------------------------------
/**
 * Get a printable string describing a health status code.
 *
 * @return Ptr to the null-terminated string.
 */
//--------------------------------------------------------------------------------------------------
static const char* GetHealthStr
(
    ma_battery_HealthStatus_t healthCode
)
{
    switch (healthCode)
    {
        case MA_BATTERY_OVERVOLTAGE:        return "overvoltage";
        case MA_BATTERY_GOOD:               return "good";
        case MA_BATTERY_COLD:               return "cold";
        case MA_BATTERY_HOT:                return "hot";
        case MA_BATTERY_HEALTHUNDEFINED:    return "undefined";
        case MA_BATTERY_HEALTHERROR:        return "error";
    }

    LE_CRIT("Unexpected health code %d.", healthCode);

    return "unknown";
}


//--------------------------------------------------------------------------------------------------
/**
 * Timer handler will monitor information on the battery charge status
 * If indication is that battery is full, then it will update the LTC charge register to
 * maximum battery charge capacity in mAh
 */
//--------------------------------------------------------------------------------------------------
static void batteryTimer
(
    le_timer_Ref_t batteryTimerRef
)
{
    static ma_battery_ChargingStatus_t oldChargingStatus = MA_BATTERY_CHARGEUNDEFINED;
    static ma_battery_HealthStatus_t oldHealthStatus = MA_BATTERY_HEALTHUNDEFINED;

    ma_battery_ChargingStatus_t chargingStatus = ma_battery_GetChargingStatus();
    if (chargingStatus != oldChargingStatus)
    {
        ReportChargingStatusChange(chargingStatus);
        oldChargingStatus = chargingStatus;

        dhubIO_PushBoolean(RES_PATH_CHARGING, 0, (chargingStatus == MA_BATTERY_CHARGING));
    }

    ma_battery_HealthStatus_t healthStatus = ma_battery_GetHealthStatus();
    if (healthStatus != oldHealthStatus)
    {
        ReportHealthStatusChange(healthStatus);
        oldHealthStatus = healthStatus;

        dhubIO_PushString(RES_PATH_HEALTH, 0, GetHealthStr(healthStatus));
    }

    int32_t capacity = le_cfg_QuickGetInt("batteryInfo/capacity", -1);
    if (capacity < 0)
    {
        LE_WARN("Battery capacity configuration not found");
        return;
    }

    uint16_t chargeRemaining;

    if (chargingStatus == MA_BATTERY_FULL)
    {
        // The battery is full, so the charge remaining must be the full capacity.
        chargeRemaining = capacity;

        // Auto-calibrate the battery current monitor by telling it that the battery now has the
        // configured maximum charge.
        UpdateChargeLevel(capacity);
    }
    else
    {
        // Ask the battery current monitor how much charge it thinks is left.
        if (ma_battery_GetChargeRemaining(&chargeRemaining) != LE_OK)
        {
            LE_WARN("Couldn't read battery level");

            return;
        }
    }

    double percentage = 100 * ((double)chargeRemaining / (double)capacity);

    dhubIO_PushNumeric(RES_PATH_ENERGY, 0, (double)chargeRemaining);
    dhubIO_PushNumeric(RES_PATH_PERCENT, 0, percentage);

    CheckBatteryLevelAlarm((unsigned int)round(percentage));

    double voltage;
    le_result_t r = ma_battery_GetVoltage(&voltage);
    if (r == LE_OK)
    {
        dhubIO_PushNumeric(RES_PATH_VOLTAGE, 0, voltage);
    }

    double degC;
    r = ma_battery_GetTemp(&degC);
    if (r == LE_OK)
    {
        dhubIO_PushNumeric(RES_PATH_TEMPERATURE, 0, degC);
    }
}


COMPONENT_INIT
{
    // type/tech = a string describing the battery technology.
    LE_ASSERT(LE_OK == dhubIO_CreateOutput(RES_PATH_TECH, DHUBIO_DATA_TYPE_STRING, ""));

    // type/voltage = nominal voltage of the battery when charged.
    LE_ASSERT(LE_OK == dhubIO_CreateOutput(RES_PATH_NOM_VOLTAGE, DHUBIO_DATA_TYPE_NUMERIC, "V"));

    // type/capacity = amount of charge the battery can store (mAh).
    LE_ASSERT(LE_OK == dhubIO_CreateOutput(RES_PATH_CAPACITY, DHUBIO_DATA_TYPE_NUMERIC, "mAh"));

    // health = string describing the health of the battery.
    LE_ASSERT(LE_OK == dhubIO_CreateInput(RES_PATH_HEALTH, DHUBIO_DATA_TYPE_STRING, ""));

    // charge/percent = percentage of total charge.
    LE_ASSERT(LE_OK == dhubIO_CreateInput(RES_PATH_PERCENT, DHUBIO_DATA_TYPE_NUMERIC, "%EL"));

    // charge/mAh = charge remaining (mAh).
    LE_ASSERT(LE_OK == dhubIO_CreateInput(RES_PATH_ENERGY, DHUBIO_DATA_TYPE_NUMERIC, "mAh"));

    // charging = a boolean indicating whether the battery is charging or not.
    LE_ASSERT(LE_OK == dhubIO_CreateInput(RES_PATH_CHARGING, DHUBIO_DATA_TYPE_BOOLEAN, ""));

    // voltage = the voltage at present.
    LE_ASSERT(LE_OK == dhubIO_CreateInput(RES_PATH_VOLTAGE, DHUBIO_DATA_TYPE_NUMERIC, "V"));

    // temperature = the temperature of the battery.
    LE_ASSERT(LE_OK == dhubIO_CreateInput(RES_PATH_TEMPERATURE, DHUBIO_DATA_TYPE_NUMERIC, "degC"));

    le_msg_AddServiceCloseHandler(ma_battery_GetServiceRef(), ClientSessionClosedHandler, NULL);

    LevelAlarmPool   = le_mem_CreatePool("batt_events", sizeof(LevelAlarmReg_t));
    LevelAlarmRefMap = le_ref_CreateMap("batt_events", 4);

    ChargingStatusRegPool   = le_mem_CreatePool("charge_events", sizeof(ChargingStatusReg_t));
    ChargingStatusRegRefMap = le_ref_CreateMap("charge_events", 4);

    HealthStatusRegPool   = le_mem_CreatePool("health_events", sizeof(HealthStatusReg_t));
    HealthStatusRegRefMap = le_ref_CreateMap("health_events", 4);

    InitMonitoringState();

    le_timer_Ref_t batteryTimerRef = le_timer_Create("Battery Service Timer");
    le_timer_SetMsInterval(batteryTimerRef, BATTERY_SAMPLE_INTERVAL_IN_MILLISECONDS);
    le_timer_SetRepeat(batteryTimerRef, 0);
    le_timer_SetHandler(batteryTimerRef, batteryTimer);
    le_timer_Start(batteryTimerRef);

    LE_INFO("---------------------- Battery Service started");
}
