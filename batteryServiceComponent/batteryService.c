/**
 *
 * This file provides the implementation of @ref c_battery
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
static uint16_t ChargeRemaining;

static le_mem_PoolRef_t LevelEventPool;
static le_ref_MapRef_t LevelEventRefMap;

static le_mem_PoolRef_t AlarmChargePool;
static le_ref_MapRef_t AlarmChargeRefMap;

static le_mem_PoolRef_t AlarmHealthPool;
static le_ref_MapRef_t AlarmHealthRefMap;

enum LevelEvent
{
    LEVEL_HIGH,
    LEVEL_LOW,
    LEVEL_NONE,
};

struct LevelEventRegistration
{
    uint8_t percentageHigh;
    uint8_t percentageLow;
    enum LevelEvent lastEventType;

    ma_battery_LevelPercentageHandlerFunc_t handler;
    void *clientContext;
    le_msg_SessionRef_t clientSessionRef;
};

struct AlarmChargeRegistration
{
    ma_battery_AlarmChargeHandlerFunc_t handler;
    void *clientContext;
    le_msg_SessionRef_t clientSessionRef;
};

struct AlarmHealthRegistration
{
    ma_battery_AlarmHealthHandlerFunc_t handler;
    void *clientContext;
    le_msg_SessionRef_t clientSessionRef;
};

ma_battery_LevelPercentageHandlerRef_t ma_battery_AddLevelPercentageHandler
(
    uint8_t percentageLow,
    uint8_t percentageHigh,
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

    struct LevelEventRegistration *reg = le_mem_ForceAlloc(LevelEventPool);
    reg->percentageLow                 = percentageLow;
    reg->percentageHigh                = percentageHigh;
    reg->lastEventType                 = LEVEL_NONE;
    reg->handler                       = handler;
    reg->clientContext                 = context;
    reg->clientSessionRef              = ma_battery_GetClientSessionRef();

    return le_ref_CreateRef(LevelEventRefMap, reg);
}

void ma_battery_RemoveLevelPercentageHandler
(
    ma_battery_LevelPercentageHandlerRef_t handlerRef
)
{
    struct LevelEventRegistration *reg = le_ref_Lookup(LevelEventRefMap, handlerRef);
    if (reg == NULL)
    {
        LE_ERROR("Failed to lookup event based on handle");
    }
    else
    {
        if (reg->clientSessionRef == ma_battery_GetClientSessionRef())
        {
            le_ref_DeleteRef(LevelEventRefMap, handlerRef);
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
    le_ref_IterRef_t it = le_ref_GetIterator(LevelEventRefMap);

    bool finished = le_ref_NextNode(it) != LE_OK;
    while (!finished)
    {
        struct LevelEventRegistration *reg = le_ref_GetValue(it);
        LE_ASSERT(reg != NULL);
        // In order to prevent invalidating the iterator, we store the reference of the device we
        // want to close and advance the iterator before calling le_spi_Close which will remove the
        // reference from the hashmap.
        void *ref = (void *)le_ref_GetSafeRef(it);
        finished  = le_ref_NextNode(it) != LE_OK;
        if (reg->clientSessionRef == ref)
        {
            le_ref_DeleteRef(LevelEventRefMap, ref);
            le_mem_Release(reg);
        }
    }
}

static void CheckBatteryLevelEvent
(
    unsigned int batteryPercentage
)
{
    le_ref_IterRef_t it = le_ref_GetIterator(LevelEventRefMap);
    bool finished       = le_ref_NextNode(it) != LE_OK;
    while (!finished)
    {
        struct LevelEventRegistration *reg = le_ref_GetValue(it);
        LE_ASSERT(reg != NULL);
        if (batteryPercentage > reg->percentageHigh && reg->lastEventType != LEVEL_HIGH)
        {
            reg->handler(batteryPercentage, reg->percentageHigh, true, reg->clientContext);
            reg->lastEventType = LEVEL_HIGH;
        }
        else if (batteryPercentage < reg->percentageLow && reg->lastEventType != LEVEL_LOW)
        {
            reg->handler(batteryPercentage, reg->percentageLow, false, reg->clientContext);
            reg->lastEventType = LEVEL_LOW;
        }

        finished = le_ref_NextNode(it) != LE_OK;
    }
}

ma_battery_AlarmChargeHandlerRef_t ma_battery_AddAlarmChargeHandler
(
    ma_battery_AlarmChargeHandlerFunc_t handler,
    void *context
)
{
    struct AlarmChargeRegistration *reg = le_mem_ForceAlloc(AlarmChargePool);
    reg->handler                        = handler;
    reg->clientContext                  = context;
    reg->clientSessionRef               = ma_battery_GetClientSessionRef();
    return le_ref_CreateRef(AlarmChargeRefMap, reg);
}

void ma_battery_RemoveAlarmChargeHandler
(
    ma_battery_AlarmChargeHandlerRef_t handlerRef
)
{
    struct AlarmChargeRegistration *reg = le_ref_Lookup(AlarmChargeRefMap, handlerRef);
    if (reg == NULL)
    {
        LE_ERROR("Failed to lookup event based on handle");
    }
    else
    {
        if (reg->clientSessionRef == ma_battery_GetClientSessionRef())
        {
            le_ref_DeleteRef(AlarmChargeRefMap, handlerRef);
            le_mem_Release(reg);
        }
        else
        {
            LE_ERROR("Remove invalid Alarm Health event handleRef");
        }
    }
}

static void RemoveAllChargeAlarmHandlersOwnedByClient
(
    le_msg_SessionRef_t owner
)
{
    le_ref_IterRef_t it = le_ref_GetIterator(AlarmChargeRefMap);

    bool finished = le_ref_NextNode(it) != LE_OK;
    while (!finished)
    {
        struct AlarmChargeRegistration *reg = le_ref_GetValue(it);
        LE_ASSERT(reg != NULL);
        // In order to prevent invalidating the iterator, we store the reference of the device we
        // want to close and advance the iterator before calling le_spi_Close which will remove the
        // reference from the hashmap.
        void *ref = (void *)le_ref_GetSafeRef(it);
        finished  = le_ref_NextNode(it) != LE_OK;
        if (reg->clientSessionRef == ref)
        {
            le_ref_DeleteRef(AlarmChargeRefMap, ref);
            le_mem_Release(reg);
        }
    }
}

static void CheckAlarmChargeEvent
(
    ma_battery_ChargeCondition_t chargecondition
)
{
    le_ref_IterRef_t it = le_ref_GetIterator(AlarmChargeRefMap);
    bool finished       = le_ref_NextNode(it) != LE_OK;
    while (!finished)
    {
        struct AlarmChargeRegistration *reg = le_ref_GetValue(it);
        LE_ASSERT(reg != NULL);

        reg->handler(chargecondition, reg->clientContext);
        finished = le_ref_NextNode(it) != LE_OK;
    }
}

ma_battery_AlarmHealthHandlerRef_t ma_battery_AddAlarmHealthHandler
(
    ma_battery_AlarmHealthHandlerFunc_t handler,
    void *context
)
{
    struct AlarmHealthRegistration *reg = le_mem_ForceAlloc(AlarmChargePool);
    reg->handler                        = handler;
    reg->clientContext                  = context;
    reg->clientSessionRef               = ma_battery_GetClientSessionRef();

    return le_ref_CreateRef(AlarmHealthRefMap, reg);
}

void ma_battery_RemoveAlarmHealthHandler
(
    ma_battery_AlarmHealthHandlerRef_t handlerRef
)
{
    struct AlarmHealthRegistration *reg = le_ref_Lookup(AlarmHealthRefMap, handlerRef);
    if (reg == NULL)
    {
        LE_ERROR("Failed to lookup event based on handle");
    }
    else
    {
        if (reg->clientSessionRef == ma_battery_GetClientSessionRef())
        {
            le_ref_DeleteRef(AlarmHealthRefMap, handlerRef);
            le_mem_Release(reg);
        }
        else
        {
            LE_ERROR("Remove invalid Alarm Health event handleRef");
        }
    }
}

static void RemoveAllHealthAlarmHandlersOwnedByClient
(
    le_msg_SessionRef_t owner
)
{
    le_ref_IterRef_t it = le_ref_GetIterator(AlarmHealthRefMap);

    bool finished = le_ref_NextNode(it) != LE_OK;
    while (!finished)
    {
        struct AlarmHealthRegistration *reg = le_ref_GetValue(it);
        LE_ASSERT(reg != NULL);
        // In order to prevent invalidating the iterator, we store the reference of the device we
        // want to close and advance the iterator before calling le_spi_Close which will remove the
        // reference from the hashmap.
        void *ref = (void *)le_ref_GetSafeRef(it);
        finished  = le_ref_NextNode(it) != LE_OK;
        if (reg->clientSessionRef == ref)
        {
            le_ref_DeleteRef(AlarmHealthRefMap, ref);
            le_mem_Release(reg);
        }
    }
}

static void CheckAlarmHealthEvent
(
    ma_battery_HealthCondition_t healthcondition
)
{
    le_ref_IterRef_t it = le_ref_GetIterator(AlarmHealthRefMap);
    bool finished       = le_ref_NextNode(it) != LE_OK;
    while (!finished)
    {
        struct AlarmHealthRegistration *reg = le_ref_GetValue(it);
        LE_ASSERT(reg != NULL);
        reg->handler(healthcondition, reg->clientContext);
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
    uint32_t maH,
    uint32_t voltage
)
{
    // Create a write transaction so we can update the tree

    LE_DEBUG(" Create battery configuration");
    le_cfg_IteratorRef_t iteratorRef = le_cfg_CreateWriteTxn("batteryInfo");

    // Set the integer  for the battery as it is an enum
    LE_DEBUG(" Enter the battery type");
    le_cfg_SetString(iteratorRef, "type", batteryType);


    // Set the battery capacity as set by the manufacturer
    LE_DEBUG("Enter the battery capacity in maH");
    le_cfg_SetInt(iteratorRef, "capacity", maH);

    // Set the voltage rating as set by the manufacturer in milliVolts
    LE_DEBUG("Enter the voltage rating");
    le_cfg_SetInt(iteratorRef, "voltage", voltage);

    int32_t energy = (int32_t)(((double)maH) * ((double)voltage) / 1000.0);

    // Set the voltage rating as set by the manufacturer
    LE_DEBUG("Enter the energy in mWH");
    le_cfg_SetInt(iteratorRef, "energy", energy);

    // Commit the transaction to make sure new settings are written to config tree
    le_cfg_CommitTxn(iteratorRef);
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
    size_t lengthofname,
    uint16_t *maH,
    uint16_t *voltage,
    uint16_t *energy
)
{
    // Create a read transaction so we can update the tree
    le_cfg_IteratorRef_t iteratorRef = le_cfg_CreateReadTxn("batteryInfo");

    if (le_cfg_NodeExists(iteratorRef, "") == false)
    {
        LE_WARN("Configuration not found");
        le_cfg_CancelTxn(iteratorRef);
        return LE_NOT_FOUND;
    }

    // Get the name for the battery type
    le_result_t result = le_cfg_GetString(iteratorRef, "type", batteryType, lengthofname, "");

    if (result != LE_OK)
    {
        le_cfg_CancelTxn(iteratorRef);
        LE_ERROR("Cannot get battery type with failure == %s", LE_RESULT_TXT(result));
        return result;
    }

    // Get the maH  for the battery
    int32_t batteryConfigCapacity = le_cfg_GetInt(iteratorRef, "capacity", -1);

    if (batteryConfigCapacity < 0)
    {
        le_cfg_CancelTxn(iteratorRef);
        LE_ERROR("Cannot get battery capcity");
        return LE_NOT_FOUND;
    }
    *maH = (uint16_t)batteryConfigCapacity;
    // Get the voltage  for the battery
    int32_t batteryConfigVoltage = le_cfg_GetInt(iteratorRef, "voltage", -1);

    if (batteryConfigCapacity < 0)
    {
        le_cfg_CancelTxn(iteratorRef);
        LE_ERROR("Cannot get battery voltage");
        return LE_NOT_FOUND;
    }

    *voltage = (uint16_t)batteryConfigVoltage;

    // Get the mWH  for the battery
    int32_t batteryConfigEnergy = le_cfg_GetInt(iteratorRef, "energy", -1);

    if (batteryConfigEnergy < 0)
    {
        le_cfg_CancelTxn(iteratorRef);
        LE_ERROR("Cannot get battery energy");
        return LE_NOT_FOUND;
    }

    *energy = (uint32_t)batteryConfigEnergy;
    le_cfg_CancelTxn(iteratorRef);
    return LE_OK;
}

//--------------------------------------------------------------------------------------------------
/**
 * Get the battery health
 *
 * @return
 *      - GOOD
 *      - OVERVOLTAGE
 *      - COLD
 *      - HOT
 *      - HEALTHUNDEFINED
 *      - HEALTHERROR
 */
//--------------------------------------------------------------------------------------------------
ma_battery_HealthCondition_t ma_battery_GetHealthStatus(void)
{
    le_result_t r;
    char path[256], healthValue[512];

    int pathLen = snprintf(path, sizeof(path), ChargerStr, MANGOH_I2C_BUS_BATTERY, HealthStr);
    LE_ASSERT(pathLen < sizeof(path));

    r = ReadStringFromFile(path, healthValue, sizeof(healthValue));
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
 * Get Charge Status
 *
 * @return
 *      - NOTCHARGING
 *      - CHARGING
 *      - FULL
 *      - CHARGEUNDEFINED
 *      - CHARGEERROR
 */
//--------------------------------------------------------------------------------------------------
ma_battery_ChargeCondition_t ma_battery_GetChargeStatus(void)
{
    le_result_t r;
    char path[256], chargeCondition[32];

    int pathLen = snprintf(path, sizeof(path), BatteryStr, MANGOH_I2C_BUS_BATTERY, StatusStr);
    LE_ASSERT(pathLen < sizeof(path));

    r = ReadStringFromFile(path, chargeCondition, sizeof(chargeCondition));
    if (r == LE_OK)
    {
        if (strcmp(chargeCondition, "Discharging") == 0)
        {
            return MA_BATTERY_DISCHARGING;
        }
        else if (strcmp(chargeCondition, "Charging") == 0)
        {
            return MA_BATTERY_CHARGING;
        }
        else if (strcmp(chargeCondition, "Full") == 0)
        {
            return MA_BATTERY_FULL;
        }
        else
        {
            return MA_BATTERY_HEALTHUNDEFINED;
        }
    }
    else
    {
        return MA_BATTERY_CHARGEERROR;
    }
}

//--------------------------------------------------------------------------------------------------
/**
 * Get Voltage Status
 *
 * @return
 *      - LE_OK
 *      - LE_IO_ERROR
 */
//--------------------------------------------------------------------------------------------------
le_result_t ma_battery_GetVoltage
(
    double *volt
)
{
    le_result_t r;
    int32_t voltcalc;
    char path[256];

    int pathLen = snprintf(path, sizeof(path), MonitorStr, MANGOH_I2C_BUS_BATTERY, VoltageStr);
    LE_ASSERT(pathLen < sizeof(path));

    r = ReadIntFromFile(path, &voltcalc);
    if (r == LE_OK)
    {
        *volt = ((double)voltcalc) / 1000.0;
    }

    return r;
}

//--------------------------------------------------------------------------------------------------
/**
 * Get Temperature Status
 *
 * @return
 *      - LE_OK
 *      - LE_IO_ERROR
 */
//--------------------------------------------------------------------------------------------------
le_result_t ma_battery_GetTemp
(
    double *temp
)
{
    le_result_t r;
    int32_t tempcalc;
    char path[256];

    int pathLen = snprintf(path, sizeof(path), MonitorStr, MANGOH_I2C_BUS_BATTERY, TempStr);
    LE_ASSERT(pathLen < sizeof(path));

    r = ReadIntFromFile(path, &tempcalc);
    if (r == LE_OK)
    {
        *temp = ((double)tempcalc) / 100.0;
    }

    return r;
}

//--------------------------------------------------------------------------------------------------
/**
 * Get  Charge Remaining in mAh
 * @return
 *      - LE_OK
 *      - LE_IO_ERROR
 */
//--------------------------------------------------------------------------------------------------
le_result_t ma_battery_GetChargeRemaining
(
    uint16_t *charge
)
{
    le_result_t r;
    int32_t chargeNow;
    char path[256];

    int pathLen = snprintf(path, sizeof(path), MonitorStr, MANGOH_I2C_BUS_BATTERY, ChargeNowStr);
    LE_ASSERT(pathLen < sizeof(path));

    r = ReadIntFromFile(path, &chargeNow);
    if (r == LE_OK)
    {
        *charge = ((chargeNow) / 1000);
    }

    return r;
}

//--------------------------------------------------------------------------------------------------
/**
 * Get  Energy Remaining in percentage
 *
 * @return
 *      - LE_OK
 *      - LE_IO_ERROR
 */
//--------------------------------------------------------------------------------------------------
le_result_t ma_battery_GetPercentRemaining
(
    uint16_t *percentage
)
{
    le_result_t r;
    int32_t chargeNow;
    char path[256];

    int pathLen = snprintf(path, sizeof(path), MonitorStr, MANGOH_I2C_BUS_BATTERY, ChargeNowStr);
    LE_ASSERT(pathLen < sizeof(path));

    le_cfg_IteratorRef_t iteratorRef = le_cfg_CreateReadTxn("batteryInfo");

    if (le_cfg_NodeExists(iteratorRef, "") == false)
    {
        LE_WARN("Configuration not found");
        r = LE_NOT_FOUND;
        goto done;
    }

    int32_t batteryConfigCapacity = le_cfg_GetInt(iteratorRef, "capacity", -1);
    if (batteryConfigCapacity < 0)
    {
        LE_WARN("Cannot get battery capcity yet");
        r = LE_CLOSED;
        goto done;
    }

    r = ReadIntFromFile(path, &chargeNow);
    if (r != LE_OK)
    {
        goto done;
    }

    *percentage = round(100 * (((double)chargeNow) / 1000) / (double)batteryConfigCapacity);

done:
    le_cfg_CancelTxn(iteratorRef);
    return r;
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
    le_result_t r;
    int32_t chargecalc;
    char path[256];

    int pathLen = snprintf(path, sizeof(path), MonitorStr, MANGOH_I2C_BUS_BATTERY, PresenceStr);
    LE_ASSERT(pathLen < sizeof(path));

    r = ReadIntFromFile(path, &chargecalc);
    if (r != LE_OK)
    {
        LE_ERROR("Battery presence is indeterminate");
        return false;
    }

    return chargecalc != 0;
}

//--------------------------------------------------------------------------------------------------
/**
 * Initialize the battery charge remaining
 */
//--------------------------------------------------------------------------------------------------
static void InitializeBatteryCharge(void)
{
    char path[256], chargeCondition[32], setCharge[256];
    int32_t battCapacityMah = -1;
    int pathLen;
    int setChargeLen;

    le_cfg_IteratorRef_t iteratorRef = le_cfg_CreateReadTxn("batteryInfo");
    if (!le_cfg_NodeExists(iteratorRef, ""))
    {
        LE_WARN("Battery configuration not found");
    }
    else
    {
        // Get the maH  for the battery
        const int32_t battCapacityMah = le_cfg_GetInt(iteratorRef, "capacity", -1);
        if (battCapacityMah < 0)
        {
            LE_WARN("Cannot get battery capacity");
        }
    }
    le_cfg_CancelTxn(iteratorRef);

    if (battCapacityMah < 0)
    {
        // Can't do anything else without knowing the battery capacity
        return;
    }

    pathLen = snprintf(path, sizeof(path), BatteryStr, MANGOH_I2C_BUS_BATTERY, StatusStr);
    LE_ASSERT(pathLen < sizeof(path));
    ReadStringFromFile(path, chargeCondition, sizeof(chargeCondition));

    // If the battery is not fully charged, initialize to 50% of capacity as a best guess
    ChargeRemaining =
        (strcmp(chargeCondition, "Full") == 0) ? battCapacityMah : (battCapacityMah / 2);

    setChargeLen = snprintf(
        setCharge, sizeof(setCharge), MonitorStr, MANGOH_I2C_BUS_BATTERY, ChargeNowStr);
    LE_ASSERT(setChargeLen < sizeof(setCharge));
    WriteIntToFile(setCharge, ChargeRemaining * 1000);
    LE_INFO("Initialized battery charge to %d mAh", ChargeRemaining);
}

//--------------------------------------------------------------------------------------------------
/**
 * Timer handler will monitor information on the battery charge status
 *
 * If indication is that battery is full, then it will update the LTC charge register to
 * maximum battery charge capacity in maH
 */
//--------------------------------------------------------------------------------------------------
static void BatteryTimerHandler
(
    le_timer_Ref_t batteryTimerRef
)
{
    static ma_battery_ChargeCondition_t oldChargeStatus = MA_BATTERY_CHARGEUNDEFINED;
    static ma_battery_HealthCondition_t oldHealthStatus = MA_BATTERY_HEALTHUNDEFINED;

    le_result_t r;
    char capacityPath[256];

    int capacityPathLen = snprintf(
        capacityPath, sizeof(capacityPath), MonitorStr, MANGOH_I2C_BUS_BATTERY, ChargeNowStr);
    LE_ASSERT(capacityPathLen < sizeof(capacityPath));

    int32_t battCapacityMah;
    le_cfg_IteratorRef_t iteratorRef = le_cfg_CreateReadTxn("batteryInfo");
    if (!le_cfg_NodeExists(iteratorRef, ""))
    {
        LE_WARN("Battery configuration not found");
        le_cfg_CancelTxn(iteratorRef);
        return;
    }

    battCapacityMah = le_cfg_GetInt(iteratorRef, "capacity", -1);
    le_cfg_CancelTxn(iteratorRef);
    if (battCapacityMah < 0)
    {
        LE_WARN("No capacity has been set");
        return;
    }

    ma_battery_ChargeCondition_t chargeStatus = ma_battery_GetChargeStatus();
    if (chargeStatus != oldChargeStatus)
    {
        CheckAlarmChargeEvent(chargeStatus);
        oldChargeStatus = chargeStatus;
    }

    ma_battery_HealthCondition_t healthStatus = ma_battery_GetHealthStatus();
    if (healthStatus != oldHealthStatus)
    {
        CheckAlarmHealthEvent(healthStatus);
        oldHealthStatus = healthStatus;
    }

    if (chargeStatus == MA_BATTERY_FULL)
    {
        WriteIntToFile(capacityPath, battCapacityMah * 1000);
        ChargeRemaining = battCapacityMah;
    }
    else
    {
        int32_t chargeNow;
        r = ReadIntFromFile(capacityPath, &chargeNow);
        if (r == LE_OK)
        {
            ChargeRemaining = chargeNow / 1000;
        }
        else
        {
            LE_WARN("Couldn't read battery level");
            return;
        }
    }

    int percentage = round(100 * (double)ChargeRemaining / (double)battCapacityMah);
    CheckBatteryLevelEvent(percentage);
}

COMPONENT_INIT
{
    le_msg_AddServiceCloseHandler(ma_battery_GetServiceRef(), ClientSessionClosedHandler, NULL);

    LevelEventPool   = le_mem_CreatePool("batt_events", sizeof(struct LevelEventRegistration));
    LevelEventRefMap = le_ref_CreateMap("batt_events", 4);

    AlarmChargePool   = le_mem_CreatePool("charge_events", sizeof(struct AlarmChargeRegistration));
    AlarmChargeRefMap = le_ref_CreateMap("charge_events", 4);

    AlarmHealthPool   = le_mem_CreatePool("health_events", sizeof(struct AlarmHealthRegistration));
    AlarmHealthRefMap = le_ref_CreateMap("health_events", 4);

    InitializeBatteryCharge();
    le_timer_Ref_t batteryTimerRef = le_timer_Create("Battery Service Timer");
    le_timer_SetMsInterval(batteryTimerRef, BATTERY_SAMPLE_INTERVAL_IN_MILLISECONDS);
    le_timer_SetRepeat(batteryTimerRef, 0);
    le_timer_SetHandler(batteryTimerRef, BatteryTimerHandler);
    le_timer_Start(batteryTimerRef);
}
