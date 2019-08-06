/**
 *
 * @file battery.c
 *
 * This file provides control and monitoring of the power supply battery via the @ref c_battery and
 * the Data Hub.
 *
 * This module operates as a state machine.  See the State variable definition.
 *
 * Data Hub integration uses the Data Hub's psensor component.
 *
 * <hr>
 *
 * Copyright (C) Sierra Wireless Inc.
 */

#include "legato.h"
#include "interfaces.h"
// This is necessary because we can't alias a types-only API due to a bug in Legato.
#define dhubIO_DataType_t io_DataType_t
#include "periodicSensor.h"
#include "batteryUtils.h"

/// Example JSON value
#define JSON_EXAMPLE "{\"health\":\"good\",\"percent\":100,\"mAh\":2200,"\
                      "\"charging\":true,\"mA\":2.838,\"V\":3.7,\"degC\":32.1}"

#define WORST_CASE_ALARM_LAG_MS 5000

// Sysfs file paths used to interface with the battery charger and fuel gauge kernel drivers.
static const char HealthFilePath[]  = "/sys/class/power_supply/bq25601-battery/health";
static const char StatusFilePath[]  = "/sys/class/power_supply/bq25601-battery/status";
#define MONITOR_DIR_PATH "/sys/class/power_supply/BQ27246"
static const char VoltageFilePath[] = MONITOR_DIR_PATH "/voltage_now";
static const char TempFilePath[]    = MONITOR_DIR_PATH "/temp";
static const char ChargeNowFilePath[] = MONITOR_DIR_PATH "/charge_now";
static const char CurrentNowFilePath[]  = MONITOR_DIR_PATH "/current_now";
static const char PresentFilePath[] = MONITOR_DIR_PATH "/present";
static const char ChargeMaxFilePath[] = MONITOR_DIR_PATH "/charge_full";

static le_mem_PoolRef_t LevelAlarmPool;
static le_ref_MapRef_t LevelAlarmRefMap;

static le_mem_PoolRef_t ChargingStatusRegPool;
static le_ref_MapRef_t ChargingStatusRegRefMap;

static le_mem_PoolRef_t HealthStatusRegPool;
static le_ref_MapRef_t HealthStatusRegRefMap;

/// Timer used to ensure that alarms and other notifications requested via the Battery API don't
/// get sampled slower than WORST_CASE_ALARM_LAG_MS.
static le_timer_Ref_t ApiCallbackCheckTimer;

/// Enumeration of possible types of alarm.
typedef enum
{
    LEVEL_HIGH, ///< Level was higher than high alarm threshold.
    LEVEL_LOW,  ///< Level was lower that low alarm threshold.
    LEVEL_NONE, ///< No alarm.
}
LevelAlarmType_t;

/// Holds percentage level alarm call-back registration information.
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

/// Holds charging status change notification call-back registration information.
typedef struct
{
    ma_battery_ChargingStatusHandlerFunc_t handler;
    void *clientContext;
    le_msg_SessionRef_t clientSessionRef;
}
ChargingStatusReg_t;

/// Holds health status change notification call-back registration information.
typedef struct
{
    ma_battery_HealthHandlerFunc_t handler;
    void *clientContext;
    le_msg_SessionRef_t clientSessionRef;
}
HealthStatusReg_t;


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
        case MA_BATTERY_DISCONNECTED:       return "disconnected";
        case MA_BATTERY_HEALTH_UNKNOWN:     return "unknown";
        case MA_BATTERY_HEALTH_ERROR:       return "error";
    }

    LE_CRIT("Unexpected health code %d.", healthCode);

    return "unknown";
}


//--------------------------------------------------------------------------------------------------
/**
 * Stop the API Callback Check Timer if no callbacks are currently registered.
 */
//--------------------------------------------------------------------------------------------------
static void StopTimerIfNoCallbacksRegistered
(
    void
)
{
    if (   (le_ref_NextNode(le_ref_GetIterator(HealthStatusRegRefMap)) != LE_OK)
        && (le_ref_NextNode(le_ref_GetIterator(LevelAlarmRefMap)) != LE_OK)
        && (le_ref_NextNode(le_ref_GetIterator(ChargingStatusRegRefMap)) != LE_OK)  )
    {
        le_timer_Stop(ApiCallbackCheckTimer);
    }
}


//--------------------------------------------------------------------------------------------------
/**
 * Register a callback function to be called when the Percentage Level changes as follows:
 *
 * Notify the client via call-back if the level goes above percentageHigh or below percentageLow.
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

    void* safeRef = le_ref_CreateRef(LevelAlarmRefMap, reg);

    // Start the API callback check timer if it isn't already running.
    (void)le_timer_Start(ApiCallbackCheckTimer);

    return safeRef;
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

            StopTimerIfNoCallbacksRegistered();
        }
        else
        {
            LE_ERROR("Remove invalid event handleRef");
        }
    }
}


//--------------------------------------------------------------------------------------------------
/**
 * Checks the list of registered battery level alarm thresholds to see if an alarm event should be
 * reported.  If so, reports them.
 */
//--------------------------------------------------------------------------------------------------
static void ReportBatteryLevelAlarms
(
    uint8_t percentage
)
{
    le_ref_IterRef_t it = le_ref_GetIterator(LevelAlarmRefMap);
    bool finished = (le_ref_NextNode(it) != LE_OK);
    while (!finished)
    {
        LevelAlarmReg_t *reg = le_ref_GetValue(it);
        LE_ASSERT(reg != NULL);
        if ((percentage > reg->percentageHigh) && (reg->lastAlarmType != LEVEL_HIGH))
        {
            reg->handler(percentage, reg->percentageHigh, true, reg->clientContext);
            reg->lastAlarmType = LEVEL_HIGH;
        }
        else if ((percentage < reg->percentageLow) && (reg->lastAlarmType != LEVEL_LOW))
        {
            reg->handler(percentage, reg->percentageLow, false, reg->clientContext);
            reg->lastAlarmType = LEVEL_LOW;
        }

        finished = (le_ref_NextNode(it) != LE_OK);
    }
}


//--------------------------------------------------------------------------------------------------
/**
 * Add handler function for EVENT 'ma_battery_ChargingStatusChange'
 *
 * Register a callback function to be called when there is a change in battery charging status.
 */
//--------------------------------------------------------------------------------------------------
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

    void* safeRef = le_ref_CreateRef(ChargingStatusRegRefMap, reg);

    // Start the API callback check timer if it isn't already running.
    (void)le_timer_Start(ApiCallbackCheckTimer);

    return safeRef;
}


//--------------------------------------------------------------------------------------------------
/**
 * Remove handler function for EVENT 'ma_battery_ChargingStatusChange'
 */
//--------------------------------------------------------------------------------------------------
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

            StopTimerIfNoCallbacksRegistered();
        }
        else
        {
            LE_ERROR("Attempt to remove another client's Alarm Health event handleRef %p",
                     handlerRef);
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
    static ma_battery_ChargingStatus_t oldStatus = MA_BATTERY_CHARGING_UNKNOWN;

    if (oldStatus != status)
    {
        oldStatus = status;

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
}


//--------------------------------------------------------------------------------------------------
/**
 * Add handler function for EVENT 'ma_battery_HealthChange'
 *
 * Register a callback function to be called when there is a change in battery health status.
 */
//--------------------------------------------------------------------------------------------------
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

    void* safeRef = le_ref_CreateRef(HealthStatusRegRefMap, reg);

    // Start the API callback check timer if it isn't already running.
    (void)le_timer_Start(ApiCallbackCheckTimer);

    return safeRef;
}


//--------------------------------------------------------------------------------------------------
/**
 * Remove handler function for EVENT 'ma_battery_HealthChange'
 */
//--------------------------------------------------------------------------------------------------
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

            StopTimerIfNoCallbacksRegistered();
        }
        else
        {
            LE_ERROR("Attempt to remove another client's Alarm Health event handleRef %p",
                     handlerRef);
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
    ma_battery_HealthStatus_t healthStatus
)
{
    static ma_battery_HealthStatus_t oldStatus = MA_BATTERY_HEALTH_UNKNOWN;

    if (oldStatus != healthStatus)
    {
        oldStatus = healthStatus;

        le_ref_IterRef_t it = le_ref_GetIterator(HealthStatusRegRefMap);
        while (le_ref_NextNode(it) == LE_OK)
        {
            HealthStatusReg_t *reg = le_ref_GetValue(it);
            LE_ASSERT(reg != NULL);
            reg->handler(healthStatus, reg->clientContext);
        }
    }
}


//--------------------------------------------------------------------------------------------------
/**
 * Compute the percentage of battery charge given the energy charge level and the capacity.
 *
 * @return The percentage.
 */
//--------------------------------------------------------------------------------------------------
static uint ComputePercentage
(
    uint charge,
    uint capacity
)
{
    uint percentage = 0;

    // Clamp at 100%
    if (charge > capacity)
    {
        LE_ERROR("Battery monitor reports available charge (%u mAh) higher than maximum of %u mAh.",
                 charge,
                 capacity);
        percentage = 100;
    }
    else if (capacity == 0) // Catch a zero capacity to avoid a divide by zero.
    {
        percentage = 0;
    }
    else
    {
        // Compute the battery charge percentage, rounding up from half a percent or higher.
        uint percentTimesTen = 1000 * charge / capacity;
        percentage = (percentTimesTen / 10);
        if ((percentTimesTen % 10) >= 5)
        {
            percentage += 1;
        }
    }

    return percentage;
}


//--------------------------------------------------------------------------------------------------
/**
 * Detect the battery presence using the bin pin on the BQ27426
 *
 * @return true if a battery is detected, false if not.
 */
//--------------------------------------------------------------------------------------------------
static bool BatteryPresent
(
    void
)
{
    int present;
    le_result_t result = util_ReadIntFromFile(PresentFilePath, &present);
    if (result != LE_OK)
    {
        LE_FATAL("Failed to read file '%s' (%s).", PresentFilePath, LE_RESULT_TXT(result));
    }

    LE_DEBUG("Battery presence= %d.", present);

    return (present > 0);
}


//--------------------------------------------------------------------------------------------------
/**
 * Reads the battery charging status from the driver.
 *
 * @return the charging status code.
 */
//--------------------------------------------------------------------------------------------------
static ma_battery_ChargingStatus_t ReadChargingStatus
(
    void
)
{
    char chargingStatus[512];
    le_result_t r = util_ReadStringFromFile(StatusFilePath, chargingStatus, sizeof(chargingStatus));

    if (r == LE_OK)
    {
        LE_DEBUG("Charging status = '%s'.", chargingStatus);

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
        else if (strcmp(chargingStatus, "Not charging") == 0)
        {
            return MA_BATTERY_NOT_CHARGING;
        }
        else if (strcmp(chargingStatus, "Unknown") == 0)
        {
            return MA_BATTERY_CHARGING_UNKNOWN;
        }
        else
        {
            LE_ERROR("Unrecognized charging status '%s'.", chargingStatus);

            return MA_BATTERY_CHARGING_ERROR;
        }
    }
    else
    {
        LE_ERROR("failed to read the charging status (%s).", LE_RESULT_TXT(r));

        return MA_BATTERY_CHARGING_ERROR;
    }
}


//--------------------------------------------------------------------------------------------------
/**
 * Read what the battery monitor thinks is "full charge" (i.e., the estimated capacity).
 *
 * @return Estimated full charge in mAh.
 */
//--------------------------------------------------------------------------------------------------
static uint ReadCapacity
(
    void
)
{
    int uAhCapacity;
    le_result_t result = util_ReadIntFromFile(ChargeMaxFilePath, &uAhCapacity);

    if (result != LE_OK)
    {
        LE_FATAL("Failed to read file '%s' (%s).", ChargeMaxFilePath, LE_RESULT_TXT(result));
    }

    if (uAhCapacity < 0)
    {
        LE_ERROR("Estimate of battery capacity is negative? (%d uAh)", uAhCapacity);
        uAhCapacity = 0;
    }

    uint mAh = (uint)uAhCapacity / 1000;

    LE_DEBUG("Charge capacity = %d mAh.", mAh);

    return mAh;
}


//--------------------------------------------------------------------------------------------------
/**
 * Read the battery health status from the driver.
 *
 * @return the battery health status code.
 */
//--------------------------------------------------------------------------------------------------
static ma_battery_HealthStatus_t ReadHealthStatus
(
    void
)
{
    char healthValue[32];
    le_result_t r = util_ReadStringFromFile(HealthFilePath, healthValue, sizeof(healthValue));

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
            LE_ERROR("Unrecognized health string from driver: '%s'.", healthValue);
            return MA_BATTERY_HEALTH_ERROR;
        }
    }
    else
    {
        return MA_BATTERY_HEALTH_ERROR;
    }
}


//--------------------------------------------------------------------------------------------------
/**
 * Provides battery health status
 *
 * @return Health status code.
 */
//--------------------------------------------------------------------------------------------------
ma_battery_HealthStatus_t ma_battery_GetHealthStatus
(
    void
)
{
    return ReadHealthStatus();
}


//--------------------------------------------------------------------------------------------------
/**
 * Provides battery charging status
 *
 * @return Charging status code.
 */
//--------------------------------------------------------------------------------------------------
ma_battery_ChargingStatus_t ma_battery_GetChargingStatus
(
    void
)
{
    ma_battery_ChargingStatus_t status = MA_BATTERY_CHARGING_UNKNOWN;

    if (BatteryPresent())
    {
        status = ReadChargingStatus();
    }

    return status;
}


//--------------------------------------------------------------------------------------------------
/**
 * Read the battery voltage from the driver.
 *
 * @return the battery voltage (in Volts).
 */
//--------------------------------------------------------------------------------------------------
static double ReadVoltage
(
    void
)
{
    int uV;
    le_result_t result = util_ReadIntFromFile(VoltageFilePath, &uV);
    if (result != LE_OK)
    {
        LE_FATAL("Failed to read file '%s' (%s).", VoltageFilePath, LE_RESULT_TXT(result));
    }

    return ((double)uV) / 1000000.0;
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
    double *volt    ///< [out] The battery voltage, in V, if LE_OK is returned.
)
{
    if (BatteryPresent())
    {
        *volt = ReadVoltage();

        return LE_OK;
    }
    else
    {
        return LE_NOT_FOUND;
    }
}


//--------------------------------------------------------------------------------------------------
/**
 * Read from the driver the electrical current flow in/out of the battery at this time.
 *
 * @return the battery current in mA.
 */
//--------------------------------------------------------------------------------------------------
static double ReadCurrent
(
    void
)
{
    int uACurrent;
    le_result_t result = util_ReadIntFromFile(CurrentNowFilePath, &uACurrent);
    if (result != LE_OK)
    {
        LE_FATAL("Failed to read file '%s' (%s).", CurrentNowFilePath, LE_RESULT_TXT(result));
    }

    double mA = ((double)uACurrent) / 1000.0;

    LE_DEBUG("Battery current = %lf mA.", mA);

    return mA;
}


//--------------------------------------------------------------------------------------------------
/**
 * Get current now (in mA)
 *
 * @return
 *      - LE_OK on success.
 */
//--------------------------------------------------------------------------------------------------
le_result_t ma_battery_GetCurrent
(
    double *current ///< [out] The current, in mA, if LE_OK is returned.
)
{
    if (BatteryPresent())
    {
        *current = ReadCurrent();
        return LE_OK;
    }
    else
    {
        return LE_NOT_FOUND;
    }
}


//--------------------------------------------------------------------------------------------------
/**
 * Read the battery temperature from the driver.
 *
 * @return Battery temperature in degrees Celcius.
 */
//--------------------------------------------------------------------------------------------------
static double ReadTemperature
(
    void
)
{
    int deciDegs;  // Tenths of a degree Celcius.

    le_result_t r = util_ReadIntFromFile(TempFilePath, &deciDegs);

    LE_FATAL_IF(r != LE_OK, "Unable to read from file (%s): %s", TempFilePath, LE_RESULT_TXT(r));

    return ((double)deciDegs) / 10.0;
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
    double *temp    ///< [out] The battery temperature, in degrees C, if LE_OK is returned.
)
{
    if (BatteryPresent())
    {
        *temp = ReadTemperature();

        return LE_OK;
    }
    else
    {
        return LE_NOT_FOUND;
    }
}


//--------------------------------------------------------------------------------------------------
/**
 * Read the charge remaining in the battery from the driver.
 *
 * @return the estimated charge remaining, in mAh.
 */
//--------------------------------------------------------------------------------------------------
static uint ReadChargeRemaining
(
    void
)
{
    int uAh;

    le_result_t r = util_ReadIntFromFile(ChargeNowFilePath, &uAh);
    if (r != LE_OK)
    {
        LE_FATAL("Failed (%s) to read file (%s).", LE_RESULT_TXT(r), ChargeNowFilePath);
    }

    if (uAh < 0)
    {
        LE_ERROR("Driver reported negative charge remaining (%d)", uAh);
        uAh = 0;
    }

    uint mAh = (uint)uAh / 1000;

    LE_DEBUG("Charge remaining = %d mAh.", mAh);

    return mAh;
}


//--------------------------------------------------------------------------------------------------
/**
 * Get Charge Remaining in mAh
 *
 * @return
 *      - LE_OK if successful.
 */
//--------------------------------------------------------------------------------------------------
le_result_t ma_battery_GetChargeRemaining
(
    uint16_t *charge    ///< The battery charge remaining, in mAh, if LE_OK is returned.
)
{
    if (BatteryPresent())
    {
        *charge = ReadChargeRemaining();
        return LE_OK;
    }
    else
    {
        return LE_NOT_FOUND;
    }
}


//--------------------------------------------------------------------------------------------------
/**
 * Get charge remaining, in percentage
 *
 * @return
 *      - LE_OK if successful
 *      - LE_IO_ERROR
 *      - LE_NOT_FOUND
 */
//--------------------------------------------------------------------------------------------------
le_result_t ma_battery_GetPercentRemaining
(
    uint16_t *percentage    ///< [out] The percent of charge remaining, if LE_OK is returned.
)
{
    le_result_t result = LE_NOT_FOUND;

    if (BatteryPresent())
    {
        uint capacity = ReadCapacity();

        // If the capacity is not known, then
        if (capacity == 0)
        {
            LE_WARN("Battery capacity unknown.");
        }
        else
        {
            uint remaining = ReadChargeRemaining();

            *percentage = ComputePercentage(remaining, capacity);

            result = LE_OK;
        }
    }

    return result;
}


//--------------------------------------------------------------------------------------------------
/**
 * Push an update to the value resource in the Data Hub.
 */
//--------------------------------------------------------------------------------------------------
static void PushToDataHub
(
    psensor_Ref_t psensorRef,
    void *context
)
//--------------------------------------------------------------------------------------------------
{
    ma_battery_HealthStatus_t healthStatus = MA_BATTERY_DISCONNECTED;
    ma_battery_ChargingStatus_t chargingStatus = MA_BATTERY_CHARGING_UNKNOWN;
    bool isCharging = false;
    uint charge = 0;
    uint percentage = 0;
    double voltage = 0.0;
    double current = 0.0;
    double temperature = 0.0;

    if (BatteryPresent())
    {
        healthStatus = ReadHealthStatus();
        chargingStatus = ReadChargingStatus();
        // Note: The battery monitor shows FULL only when on external power.
        isCharging = (   (chargingStatus == MA_BATTERY_CHARGING)
                      || (chargingStatus == MA_BATTERY_FULL)  );
        charge = ReadChargeRemaining();
        percentage = ComputePercentage(charge, ReadCapacity());
        voltage = ReadVoltage();
        current = ReadCurrent();
        temperature = ReadTemperature();
    }

    // Generate a JSON value.
    char value[IO_MAX_STRING_VALUE_LEN + 1];
    int len = snprintf(value,
                       sizeof(value),
                       "{\"health\":\"%s\","
                       "\"percent\":%u,"
                       "\"mAh\":%u,"
                       "\"charging\":%s,"
                       "\"mA\": %.3lf,"
                       "\"V\":%.2lf,"
                       "\"degC\":%.2lf}",
                       GetHealthStr(healthStatus),
                       percentage,
                       charge,
                       isCharging ? "true" : "false",
                       current,
                       voltage,
                       temperature);
    LE_DEBUG("'%s'", value);
    if (len >= sizeof(value))
    {
        LE_ERROR("JSON value too big for Data Hub (%d characters).", len);
    }
    else
    {
        psensor_PushJson(psensorRef, IO_NOW, value);
    }

    // Report alarms and status changes that API clients have registered to receive.
    ReportHealthStatusChange(healthStatus);
    ReportChargingStatusChange(chargingStatus);
    ReportBatteryLevelAlarms(percentage);

    // Restart the timer that is used to ensure a minimum polling frequency for the
    // alarms and status change reports for API clients.
    le_timer_Restart(ApiCallbackCheckTimer);
}


//--------------------------------------------------------------------------------------------------
/**
 * Timer expiry handler for the API notification check timer.  This timer only expires when
 * someone has registered for notification callbacks for battery condition changes, level alarms,
 * etc., and the data hub is receiving periodic updates slower than the minimum amount of time
 * we consider acceptable for these notifications (i.e., if more than WORST_CASE_ALARM_LAG_MS
 * passes before PushToDataHub() is called, then this timer will expire).
 */
//--------------------------------------------------------------------------------------------------
static void AlarmCheckTimerExpiryHandler
(
    le_timer_Ref_t batteryTimerRef
)
{
    ma_battery_HealthStatus_t healthStatus = MA_BATTERY_DISCONNECTED;
    ma_battery_ChargingStatus_t chargingStatus = MA_BATTERY_CHARGING_UNKNOWN;
    uint percentage = 0;

    if (BatteryPresent())
    {
        healthStatus = ReadHealthStatus();
        chargingStatus = ReadChargingStatus();
        uint charge = ReadChargeRemaining();
        percentage = ComputePercentage(charge, ReadCapacity());
    }

    ReportHealthStatusChange(healthStatus);
    ReportChargingStatusChange(chargingStatus);
    ReportBatteryLevelAlarms(percentage);

    // NOTE: We don't need to restart the timer, because the timer is a repeating timer.
}


COMPONENT_INIT
{
    LevelAlarmPool   = le_mem_CreatePool("batt_events", sizeof(LevelAlarmReg_t));
    LevelAlarmRefMap = le_ref_CreateMap("batt_events", 4);

    ChargingStatusRegPool   = le_mem_CreatePool("charge_events", sizeof(ChargingStatusReg_t));
    ChargingStatusRegRefMap = le_ref_CreateMap("charge_events", 4);

    HealthStatusRegPool   = le_mem_CreatePool("health_events", sizeof(HealthStatusReg_t));
    HealthStatusRegRefMap = le_ref_CreateMap("health_events", 4);

    psensor_CreateJson("", JSON_EXAMPLE, PushToDataHub, NULL);

    // Create a timer for checking if a client of the battery API has asked for notification
    // callbacks. But don't run it until someone registers a callback.
    ApiCallbackCheckTimer = le_timer_Create("NotifyTimer");
    le_timer_SetMsInterval(ApiCallbackCheckTimer, WORST_CASE_ALARM_LAG_MS);
    le_timer_SetRepeat(ApiCallbackCheckTimer, 0 /* repeat forever */);
    le_timer_SetHandler(ApiCallbackCheckTimer, AlarmCheckTimerExpiryHandler);

    LE_INFO("---------------------- Battery Service started");
}
