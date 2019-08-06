/**
 * @file batteryService.c
 *
 * This file provides control and monitoring of the power supply battery via the @ref c_battery and
 * the Data Hub.
 *
 * Beware that the battery charger will report "Good" health and "Full" charge status when the
 * battery is disconnected and the system is running on external power.  Therefore, a presence
 * check must be performed before reporting battery health.
 *
 * The presence is checked by examining taking 2 samples from the charge counter over a period of
 * time. If the charge counter is changing, then a battery is connected.  Counting up means that
 * the battery is charging. Counting down means that the battery is discharging.  If the charge
 * counter is not changing over time, then either there's no battery connected or the battery
 * is full.  Unfortunately, a "Full" status will be intermittently reported even when the battery
 * is disconnected.  But, if we ever see "Charging" while the charge counter is not changing,
 * then we know that there isn't a battery connected.
 *
 * This module operates as a state machine.  See the State variable definition.
 *
 * Configuration settings are stored in the Config Tree.  In addition, when a battery is known to
 * exist and the calibration procedure has completed, the battery percent level is periodically
 * stored in the Config Tree so that we don't have to run the calibration procedure again after
 * a reboot.
 *
 * <hr>
 *
 * Copyright (C) Sierra Wireless Inc.
 */

#include "legato.h"
#include "interfaces.h"
#include "batteryUtils.h"

#define DEFAULT_BATTERY_SAMPLE_INTERVAL_MS 10000
#define STABILIZATION_TIME_MS 5000

static const char HealthFilePath[]  = "/sys/class/power_supply/bq24190-charger/health";
static const char StatusFilePath[]  = "/sys/class/power_supply/bq24190-battery/status";
static const char MonitorDirPath[] = "/sys/class/power_supply/LTC2942";
static const char VoltageFileName[] = "voltage_now";
static const char TempFileName[]    = "temp";
static const char ChargeNowFileName[] = "charge_now";
static const char CounterFileName[]  = "charge_counter";

static le_mem_PoolRef_t LevelAlarmPool;
static le_ref_MapRef_t LevelAlarmRefMap;

static le_mem_PoolRef_t ChargingStatusRegPool;
static le_ref_MapRef_t ChargingStatusRegRefMap;

static le_mem_PoolRef_t HealthStatusRegPool;
static le_ref_MapRef_t HealthStatusRegRefMap;

// Output resources (configuration settings).
#define RES_PATH_TECH        "tech"     ///< String name of the battery technology (e.g., "LiPo")
#define RES_PATH_CAPACITY    "capacity" ///< Capacity of the battery in mAh
#define RES_PATH_NOM_VOLTAGE "nominalVoltage"   ///< Nominal battery voltage in Volts
#define RES_PATH_PERIOD      "period"   ///< Sampling period in seconds

/// Input resource path
#define RES_PATH_VALUE       "value"

/// Example JSON value
#define JSON_EXAMPLE "{\"health\":\"good\",\"%EL\":100,\"mAh\":2200,\"charging\":true,"\
                      "\"mA\":2.838,\"V\":3.7,\"degC\":32.1}"
/// Not a number
#ifndef NAN
    #define NAN  (0.0 / 0.0)
#endif

/// The timer used to trigger polling of the battery monitor.
static le_timer_Ref_t Timer = NULL;

/// The normal polling period in ms.
static uint32_t PollingPeriod = DEFAULT_BATTERY_SAMPLE_INTERVAL_MS;

/// Battery capacity (mAh), or -1 if not configured.
static int32_t Capacity = -1;

/// The charging status of the battery.
static ma_battery_ChargingStatus_t ChargingStatus = MA_BATTERY_CHARGING_UNKNOWN;

/// The last read value of the Charge Counter.  If counting up or down, a battery is connected.
static int32_t ChargeCounter = 0;

/// The value read before the last read value of the Charge Counter.
static int32_t OldChargeCounter = 0;

/// The current flowing into or out of the battery (mA).
static double CurrentFlow = 0;


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


/// Enumerates all states that the battery service can be in.
static enum
{
    STATE_UNCONFIGURED,     ///< The required configuration settings have not been provided.
    STATE_STABILIZING,      ///< The capacity has been changed, waiting a few seconds to stabilize.
    STATE_DETECTING_PRESENCE, ///< Running the battery detection algorithm.
    STATE_DISCONNECTED,     ///< No battery connected.
    STATE_CALIBRATING,      ///< A battery is present but the charge level is not yet known.
    STATE_NOMINAL           ///< The state of the battery is known.
}
State = STATE_UNCONFIGURED;


/// Enumerates all the events that are significant to the operation of the state machine.
typedef enum
{
    EVENT_TIMER_EXPIRED,
    EVENT_CAPACITY_CHANGED,
}
Event_t;


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


//--------------------------------------------------------------------------------------------------
/**
 * Report a change in the charging status to any registered charging status handlers.
 */
//--------------------------------------------------------------------------------------------------
static void ReportChargingStatusChange
(
    void
)
{
    static ma_battery_ChargingStatus_t oldStatus = MA_BATTERY_CHARGING_UNKNOWN;

    ma_battery_ChargingStatus_t chargingStatus = ma_battery_GetChargingStatus();

    if (oldStatus != chargingStatus)
    {
        oldStatus = chargingStatus;

        le_ref_IterRef_t it = le_ref_GetIterator(ChargingStatusRegRefMap);
        bool finished       = le_ref_NextNode(it) != LE_OK;
        while (!finished)
        {
            ChargingStatusReg_t *reg = le_ref_GetValue(it);
            LE_ASSERT(reg != NULL);

            reg->handler(chargingStatus, reg->clientContext);
            finished = (le_ref_NextNode(it) != LE_OK);
        }
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

//--------------------------------------------------------------------------------------------------
/**
 * A handler for client disconnects which frees all resources associated with the client.
 */
//--------------------------------------------------------------------------------------------------
/*static void ClientSessionClosedHandler
(
    le_msg_SessionRef_t clientSession,
    void *context
)
{
    RemoveAllLevelAlarmHandlersOwnedByClient(clientSession);
    RemoveAllChargeAlarmHandlersOwnedByClient(clientSession);
    RemoveAllHealthAlarmHandlersOwnedByClient(clientSession);
}
*/

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
        bool finished       = le_ref_NextNode(it) != LE_OK;
        while (!finished)
        {
            HealthStatusReg_t *reg = le_ref_GetValue(it);
            LE_ASSERT(reg != NULL);
            reg->handler(healthStatus, reg->clientContext);
            finished = le_ref_NextNode(it) != LE_OK;
        }
    }
}


//--------------------------------------------------------------------------------------------------
/**
 * @return true if charging, false if not or if unknown.
 */
//--------------------------------------------------------------------------------------------------
static bool IsCharging
(
    void
)
{
    ma_battery_ChargingStatus_t status = ma_battery_GetChargingStatus();

    // Note: The battery monitor shows FULL only when on external power.
    return ((status == MA_BATTERY_CHARGING) || (status == MA_BATTERY_FULL));
}

//--------------------------------------------------------------------------------------------------
/**
 * Compute the percentage of battery charge given the energy charge level.
 *
 * @return The percentage.
 */
//--------------------------------------------------------------------------------------------------
static unsigned int ComputePercentage
(
    unsigned int mAh
)
{
    // Compute the battery charge percentage, rounding up from half a percent or higher.
    uint32_t percentTimesTen = 1000UL * mAh / Capacity;
    unsigned int percentage = (percentTimesTen / 10);
    if ((percentTimesTen % 10) >= 5)
    {
        percentage += 1;
    }

    // Clamp at 100%
    if (percentage > 100)
    {
        LE_WARN("Battery monitor reports available charge (%u mAh) higher than maximum of %u mAh.",
                mAh,
                (unsigned int)Capacity);
        percentage = 100;
    }

    return percentage;
}


//--------------------------------------------------------------------------------------------------
/**
 * Save the percentage level.
 */
//--------------------------------------------------------------------------------------------------
static void SavePercentage
(
    unsigned int percentage
)
{
    le_cfg_QuickSetInt("batteryInfo/percent", percentage);
}


//--------------------------------------------------------------------------------------------------
/**
 * Delete the saved percentage level.
 */
//--------------------------------------------------------------------------------------------------
static void DeletePercentage
(
    void
)
{
    le_cfg_QuickDeleteNode("batteryInfo/percent");
}


//--------------------------------------------------------------------------------------------------
/**
 * Load the saved percentage level.
 *
 * @return The percentage, or -1 if not found.
 */
//--------------------------------------------------------------------------------------------------
static int LoadPercentage
(
    void
)
{
    return le_cfg_QuickGetInt("batteryInfo/percent", -1);
}


//--------------------------------------------------------------------------------------------------
/**
 * Push an update to the value resource in the Data Hub.
 */
//--------------------------------------------------------------------------------------------------
static void PushToDataHub
(
    ma_battery_HealthStatus_t healthStatus,
    unsigned int percentage,
    unsigned int mAh
)
//--------------------------------------------------------------------------------------------------

{
    // Get the battery voltage.
    double voltage;
    le_result_t r = ma_battery_GetVoltage(&voltage);
    if (r != LE_OK)
    {
        LE_FATAL("Failed to read battery voltage (%s).", LE_RESULT_TXT(r));
    }

    // Get the temperature reading.
    double temperature;
    r = ma_battery_GetTemp(&temperature);
    if (r != LE_OK)
    {
        LE_FATAL("Failed to read temperature (%s).", LE_RESULT_TXT(r));
    }

    // If the health is not known, or the battery is definitely disconnected, then the
    // charge levels are meaningless and should be zeroed.
    if (   (healthStatus == MA_BATTERY_DISCONNECTED)
        || (healthStatus == MA_BATTERY_HEALTH_ERROR)
        || (healthStatus == MA_BATTERY_HEALTH_UNKNOWN)  )
    {
        mAh = 0;
        percentage = 0;
    }

    // Generate a JSON value.
    char value[DHUBIO_MAX_STRING_VALUE_LEN + 1];
    int len = snprintf(value,
                       sizeof(value),
                       "{\"health\":\"%s\","
                       "\"%%EL\":%u,"
                       "\"mAh\":%u,"
                       "\"charging\":%s,"
                       "\"mA\": %.3lf,"
                       "\"V\":%.2lf,"
                       "\"degC\":%.2lf}",
                       GetHealthStr(healthStatus),
                       percentage,
                       mAh,
                       IsCharging() ? "true" : "false",
                       CurrentFlow,
                       voltage,
                       temperature);
    if (len >= sizeof(value))
    {
        LE_ERROR("JSON value too big for Data Hub (%d characters).", len);
    }
    else
    {
        LE_DEBUG("'%s'", value);
        dhubIO_PushJson(RES_PATH_VALUE, DHUBIO_NOW, value);
    }
}

//--------------------------------------------------------------------------------------------------
/**
 * Report all types of alarms and status updates.
 */
//--------------------------------------------------------------------------------------------------
static void ReportAll
(
    void
)
{
    // Get the Energy Level.
    uint16_t mAh = 0;
    if (State != STATE_DISCONNECTED)
    {
        le_result_t r = ma_battery_GetChargeRemaining(&mAh);
        if (r != LE_OK)
        {
            LE_FATAL("Failed to read battery charge level (%s).", LE_RESULT_TXT(r));
        }
    }

    unsigned int percentage = ComputePercentage(mAh);

    // In the NOMINAL state, whenever the percentage changes, save it in the Config Tree
    // so we don't have to re-calibrate whenever there's a reboot.
    static int oldPercentage = -1;
    if (State == STATE_NOMINAL)
    {
        if (percentage != oldPercentage)
        {
            SavePercentage(percentage);

            oldPercentage = percentage;
        }
    }
    else
    {
        oldPercentage = -1;
    }

    // Get the health status.
    ma_battery_HealthStatus_t healthStatus = ma_battery_GetHealthStatus();

    ReportBatteryLevelAlarms((uint8_t)percentage);
    ReportChargingStatusChange();
    ReportHealthStatusChange(healthStatus);
    PushToDataHub(healthStatus, percentage, mAh);
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
    LE_DEBUG("Charge level = %d mAh.", mAh);

    if (mAh > 0)
    {
        int uAh = mAh * 1000;

        LE_DEBUG("battery %d", uAh);

        char path[256];
        int len = snprintf(path, sizeof(path), "%s/%s", MonitorDirPath, ChargeNowFileName);
        LE_ASSERT(len < sizeof(path));
        util_WriteIntToFile(path, uAh);
    }
    else
    {
        LE_ERROR("Charge level invalid. (%d mAh)", mAh);
    }
}


//--------------------------------------------------------------------------------------------------
/**
 * Reads the battery charging status and updates the ChargingStatus variable.
 */
//--------------------------------------------------------------------------------------------------
static void ReadChargingStatus
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
            ChargingStatus = MA_BATTERY_DISCHARGING;
        }
        else if (strcmp(chargingStatus, "Charging") == 0)
        {
            ChargingStatus = MA_BATTERY_CHARGING;
        }
        else if (strcmp(chargingStatus, "Full") == 0)
        {
            ChargingStatus = MA_BATTERY_FULL;
        }
        else if (strcmp(chargingStatus, "Not charging") == 0)
        {
            ChargingStatus = MA_BATTERY_NOT_CHARGING;
        }
        else if (strcmp(chargingStatus, "Unknown") == 0)
        {
            ChargingStatus = MA_BATTERY_CHARGING_UNKNOWN;
        }
        else
        {
            LE_ERROR("Unrecognized charging status '%s'.", chargingStatus);

            ChargingStatus = MA_BATTERY_CHARGING_ERROR;
        }
    }
    else
    {
        LE_ERROR("failed to read the charging status (%s).", LE_RESULT_TXT(r));

        ChargingStatus = MA_BATTERY_CHARGING_ERROR;
    }
}


//--------------------------------------------------------------------------------------------------
/**
 * Read the value of the battery current monitor's charge counter and update the
 * ChargeCounter and OldChargeCounter variables.
 */
//--------------------------------------------------------------------------------------------------
static void ReadChargeCounter
(
    void
)
{
    char path[PATH_MAX];

    int pathLen = snprintf(path, sizeof(path), "%s/%s", MonitorDirPath, CounterFileName);
    LE_ASSERT(pathLen < sizeof(path));

    int32_t counter;
    le_result_t result = util_ReadIntFromFile(path, &counter);

    if (result != LE_OK)
    {
        LE_FATAL("Failed to read file '%s' (%s).", path, LE_RESULT_TXT(result));
    }

    LE_DEBUG("Charge counter = %d.", counter);

    OldChargeCounter = ChargeCounter;
    ChargeCounter = counter;
}


//--------------------------------------------------------------------------------------------------
/**
 * Start the stabilization period.  After configuration is changed, we have to wait a few seconds
 * for the battery monitor to settle-down.
 *
 * @warning Make sure Capacity is set before calling this function.
 */
//--------------------------------------------------------------------------------------------------
static void StartStabilization
(
    void
)
//--------------------------------------------------------------------------------------------------
{
    State = STATE_STABILIZING;

    le_timer_Stop(Timer);
    le_timer_SetMsInterval(Timer, STABILIZATION_TIME_MS);
    le_timer_Start(Timer);
}


//--------------------------------------------------------------------------------------------------
/**
 * Start calibration.
 *
 * @warning Make sure Capacity is set before calling this function.
 */
//--------------------------------------------------------------------------------------------------
static void StartCalibration
(
    void
)
//--------------------------------------------------------------------------------------------------
{
    // If the battery is full,
    if (ChargingStatus == MA_BATTERY_FULL)
    {
        LE_DEBUG("Battery is full");

        // Tell the battery monitoring driver that battery's present charge level is
        // equal to the maximum configured capacity.
        UpdateChargeLevel(Capacity);

        State = STATE_NOMINAL;
    }
    // But, if the battery is not full,
    else
    {
        // Since we have no way of knowing what the actual charge level of the battery
        // is, tell the battery monitoring driver the battery's present charge is half its
        // maximum capacity.  When the battery charger later signals a "full" condition,
        // we'll update this again.  Otherwise, we let the battery monitoring driver update
        // it as the battery charges and drains.
        LE_WARN("Battery level unknown. Assuming 50%% for now. Please fully charge to calibrate.");

        UpdateChargeLevel(Capacity / 2);

        State = STATE_CALIBRATING;  // Battery is known to exist but charge level is unknown.
    }

    // Reset the timer to run at the normal polling frequency.
    le_timer_Stop(Timer);
    le_timer_SetMsInterval(Timer, PollingPeriod);
    le_timer_Start(Timer);
}


//--------------------------------------------------------------------------------------------------
/**
 * Event handler function for the UNCONFIGURED state.
 */
//--------------------------------------------------------------------------------------------------
static void UnconfiguredState
(
    Event_t event
)
{
    switch (event)
    {
        case EVENT_TIMER_EXPIRED:

            // In the unconfigured state, we are missing information required to properly function.
            LE_CRIT("Timer expired in UNCONFIGURED state.");
            le_timer_Stop(Timer);
            break;

        case EVENT_CAPACITY_CHANGED:

            // We transition to the STABILIZING state and set the timer to tell us when
            // the stabilization period is over.
            StartStabilization();
            break;
    }
}


//--------------------------------------------------------------------------------------------------
/**
 * Event handler function for the STABILIZING state.
 */
//--------------------------------------------------------------------------------------------------
static void StabilizingState
(
    Event_t event
)
{
    switch (event)
    {
        case EVENT_TIMER_EXPIRED:

            // This tells us we are done stabilizing.
            // We only enter the stabilizing state after the capacity setting has been changed,
            // so we know we are configured.

            // Enter the DETECTING_PRESENCE state, starting the timer to tell us when we should
            // check the flow counter and charging status again to see if we have a battery.
            State = STATE_DETECTING_PRESENCE;
            le_timer_Stop(Timer);
            le_timer_SetMsInterval(Timer, PollingPeriod);
            le_timer_Start(Timer);
            break;

        case EVENT_CAPACITY_CHANGED:

            // Restart the stabilization period.
            le_timer_Stop(Timer);
            le_timer_Start(Timer);
            break;
    }
}


//--------------------------------------------------------------------------------------------------
/**
 * Event handler function for the DETECTING_PRESENCE state.
 */
//--------------------------------------------------------------------------------------------------
static void DetectingPresenceState
(
    Event_t event
)
{
    switch (event)
    {
        case EVENT_TIMER_EXPIRED:
        {
            // If the charge counter has changed, then we know there's a battery connected.
            if (ChargeCounter != OldChargeCounter)
            {
                // Start battery level calibration.
                State = STATE_CALIBRATING;
                StartCalibration();
            }
            // If the charge counter has not changed, and we've seen "Charging" (instead of "Full"),
            // then we know that a battery is NOT connected.
            else if (ChargingStatus == MA_BATTERY_CHARGING)
            {
                State = STATE_DISCONNECTED;
            }

            break;
        }

        case EVENT_CAPACITY_CHANGED:

            // Start the stabilization period.
            StartStabilization();
            break;
    }
}


//--------------------------------------------------------------------------------------------------
/**
 * Event handler function for the DISCONNECTED state.
 */
//--------------------------------------------------------------------------------------------------
static void DisconnectedState
(
    Event_t event
)
{
    switch (event)
    {
        case EVENT_TIMER_EXPIRED:
        {
            // If the charge counter has changed, then we know there's a battery connected.
            if (ChargeCounter != OldChargeCounter)
            {
                // Start battery level calibration.
                State = STATE_CALIBRATING;
                StartCalibration();
            }

            break;
        }

        case EVENT_CAPACITY_CHANGED:

            // Start the stabilization period.
            StartStabilization();
            break;
    }
}


//--------------------------------------------------------------------------------------------------
/**
 * Event handler function for the CALIBRATING state.
 */
//--------------------------------------------------------------------------------------------------
static void CalibratingState
(
    Event_t event
)
{
    switch (event)
    {
        case EVENT_TIMER_EXPIRED:
        {
            // If the charging status is "Full", we're done calibrating.  We know the level is
            // 100%.  Update the battery monitor and switch to the NOMINAL state.
            if (ChargingStatus == MA_BATTERY_FULL)
            {
                UpdateChargeLevel(Capacity);

                State = STATE_NOMINAL;
            }
            // Otherwise, if the charge counter has not changed, but the hardware still thinks
            // it's charging, then the battery must have been disconnected.
            else if ((ChargeCounter == OldChargeCounter) && (ChargingStatus == MA_BATTERY_CHARGING))
            {
                State = STATE_DISCONNECTED;

                // Forget the old percent level, if it's stored in the Config Tree.
                DeletePercentage();
            }

            break;
        }

        case EVENT_CAPACITY_CHANGED:

            // Start the stabilization period.
            StartStabilization();
            break;
    }
}


//--------------------------------------------------------------------------------------------------
/**
 * Event handler function for the NOMINAL state.
 */
//--------------------------------------------------------------------------------------------------
static void NominalState
(
    Event_t event
)
{
    switch (event)
    {
        case EVENT_TIMER_EXPIRED:
        {
            // If the charge counter has not changed, but the hardware still thinks
            // it's charging, then the battery must have been disconnected.
            if ((ChargeCounter == OldChargeCounter) && (ChargingStatus == MA_BATTERY_CHARGING))
            {
                State = STATE_DISCONNECTED;

                // Forget the old percent level, if it's stored in the Config Tree.
                DeletePercentage();
            }
            // Else, if the charger is reporting that the battery is full,
            // re-calibrate the charge monitor to 100%.
            else if (ChargingStatus == MA_BATTERY_FULL)
            {
                UpdateChargeLevel(Capacity);
            }

            break;
        }

        case EVENT_CAPACITY_CHANGED:

            // Start the stabilization period.
            StartStabilization();
            break;
    }
}


//--------------------------------------------------------------------------------------------------
/**
 * Run the state machine, given an event.
 */
//--------------------------------------------------------------------------------------------------
static void RunStateMachine
(
    Event_t event
)
{
    switch (State)
    {
        case STATE_UNCONFIGURED:

            UnconfiguredState(event);
            break;

        case STATE_STABILIZING:

            StabilizingState(event);
            break;

        case STATE_DETECTING_PRESENCE:

            DetectingPresenceState(event);
            break;

        case STATE_DISCONNECTED:

            DisconnectedState(event);
            break;

        case STATE_CALIBRATING:

            CalibratingState(event);
            break;

        case STATE_NOMINAL:

            NominalState(event);
            break;
    }

    ReportAll();
}


//--------------------------------------------------------------------------------------------------
/**
 * Set the battery technology.
 */
//--------------------------------------------------------------------------------------------------

static void SetTechnology
(
    double timestamp,
    const char* tech,
    void* contextPtr ///< unused
)
//--------------------------------------------------------------------------------------------------
{
    le_cfg_QuickSetString("batteryInfo/type", tech);
}


//--------------------------------------------------------------------------------------------------
/**
 * Set the capacity.
 */
//--------------------------------------------------------------------------------------------------

static void SetCapacity
(
    double timestamp,
    double capacity,  ///< mAh
    void* contextPtr ///< unused
)
//--------------------------------------------------------------------------------------------------
{
    if (capacity < 0)
    {
        LE_ERROR("Capacity of %lf mAh is out of range.", capacity);
    }
    else if ((uint32_t)capacity != Capacity)
    {
        Capacity = capacity;

        le_cfg_QuickSetInt("batteryInfo/capacity", (int32_t)capacity);

        // Forget the old percent level, if it's stored in the Config Tree.
        DeletePercentage();

        // Notify the state machine that the capacity setting changed.
        RunStateMachine(EVENT_CAPACITY_CHANGED);
    }
}


//--------------------------------------------------------------------------------------------------
/**
 * Set the nominal voltage of the battery.
 */
//--------------------------------------------------------------------------------------------------

static void SetNominalVoltage
(
    double timestamp,
    double voltage,  ///< V
    void* contextPtr ///< unused
)
//--------------------------------------------------------------------------------------------------
{
    if (voltage < 0)
    {
        LE_ERROR("Voltage of %lf V is out of range.", voltage);
    }
    else
    {
        le_cfg_QuickSetInt("batteryInfo/voltage", (uint32_t)(voltage * 1000)); // stored as mV
    }
}

//--------------------------------------------------------------------------------------------------
/**
 * Set the timer period.
 */
//--------------------------------------------------------------------------------------------------

static void SetPeriod
(
    double timestamp,
    double period,  ///< seconds
    void* contextPtr ///< unused
)
//--------------------------------------------------------------------------------------------------
{
    if (period <= 0)
    {
        LE_ERROR("Period of %lf seconds is out of range.", period);
    }
    else
    {
        le_timer_SetMsInterval(Timer, (uint32_t)(period * 1000));
    }
}

//--------------------------------------------------------------------------------------------------
/**
 * Set the battery technology as set by the battery manufacturer
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

    // Set the battery technology.
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

    // Notify the state machine if the capacity setting changed.
    if (Capacity != mAh)
    {
        Capacity = mAh;

        // Forget the old percent level, if it's stored in the Config Tree.
        DeletePercentage();

        RunStateMachine(EVENT_CAPACITY_CHANGED);
    }
}


//--------------------------------------------------------------------------------------------------
/**
 * Get the battery technology as set by the battery manufacturer
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
        if (batteryTypeSize > 0)
        {
            batteryType[0] = '\0';
        }
    }
    if ((batteryTypeSize > 0) && (batteryType[0] == '\0'))
    {
        LE_WARN("Battery type not configured.");
    }

    // Get the battery voltage in mV (or -1 if not found)
    int32_t voltage = le_cfg_GetInt(iteratorRef, "voltage", -1);
    if (voltage < 0)
    {
        LE_WARN("Battery nominal voltage not configured.");
        voltage = 0;
    }
    *voltagePtr = (uint16_t)voltage;

    // Get the battery capacity in mAh (or -1 if not found)
    // NOTE: This is the only one that really matters.  Everything else is informational.
    int capacity = le_cfg_GetInt(iteratorRef, "capacity", -1);
    if (capacity < 0)
    {
        LE_ERROR("Battery capacity not configured.  Battery Service cannot function without it.");
        LE_ERROR("Please configure battery capacity via Battery API or Data Hub.");
        result = LE_NOT_FOUND;
    }
    else
    {
        *capacityPtr = (uint16_t)capacity;
        result = LE_OK;
    }

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
ma_battery_HealthStatus_t ma_battery_GetHealthStatus
(
    void
)
{
    if (State == STATE_DISCONNECTED)
    {
        return MA_BATTERY_DISCONNECTED;
    }

    char healthValue[32];
    le_result_t r = util_ReadStringFromFile(HealthFilePath, healthValue, sizeof(healthValue));

    if (r == LE_OK)
    {
        if (strcmp(healthValue, "Good") == 0)
        {
            if ((State != STATE_CALIBRATING) && (State != STATE_NOMINAL))
            {
                return MA_BATTERY_HEALTH_UNKNOWN;
            }
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
            return MA_BATTERY_HEALTH_UNKNOWN;
        }
    }
    else
    {
        return MA_BATTERY_HEALTH_ERROR;
    }
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
    switch (State)
    {
        case STATE_UNCONFIGURED:
        case STATE_STABILIZING:
        case STATE_DETECTING_PRESENCE:
        case STATE_DISCONNECTED:

            return MA_BATTERY_CHARGING_UNKNOWN;

        case STATE_CALIBRATING:
        case STATE_NOMINAL:

            return ChargingStatus;
    }

    LE_FATAL("Invalid state %d.", State);
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

    int pathLen = snprintf(path, sizeof(path), "%s/%s", MonitorDirPath, VoltageFileName);
    LE_ASSERT(pathLen < sizeof(path));

    int32_t uV;
    le_result_t r = util_ReadIntFromFile(path, &uV);
    if (r == LE_OK)
    {
        *volt = ((double)uV) / 1000000.0;
    }

    return r;
}


//--------------------------------------------------------------------------------------------------
/**
 * Get battery current (in mA)
 *
 * @return
 *      - LE_OK on success.
 */
//--------------------------------------------------------------------------------------------------
le_result_t ma_battery_GetCurrent
(
    double *current ///< Battery current in mA, if LE_OK is returned.
)
{
    return LE_NOT_IMPLEMENTED;
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

    int pathLen = snprintf(path, sizeof(path), "%s/%s", MonitorDirPath, TempFileName);
    LE_ASSERT(pathLen < sizeof(path));

    int32_t tempcalc;  // In centidegrees Celcius.
    le_result_t r = util_ReadIntFromFile(path, &tempcalc);
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

    int pathLen = snprintf(path, sizeof(path), "%s/%s", MonitorDirPath, ChargeNowFileName);
    LE_ASSERT(pathLen < sizeof(path));

    int32_t uAh;
    le_result_t r = util_ReadIntFromFile(path, &uAh);
    if (r == LE_OK)
    {
        *charge = uAh / 1000;
    }

    LE_DEBUG("Charge level = %uh mAh.", *charge);

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
    if (Capacity < 0)
    {
        LE_WARN("Battery capacity not configured");
        return LE_NOT_FOUND;
    }
    else if (   (State == STATE_DISCONNECTED)
             || (State == STATE_STABILIZING)
             || (State == STATE_DETECTING_PRESENCE))
    {
        return LE_NOT_FOUND;
    }
    else
    {
        uint16_t remaining;
        le_result_t r = ma_battery_GetChargeRemaining(&remaining);
        if (r == LE_OK)
        {
            *percentage = ComputePercentage(remaining);
        }

        return r;
    }
}


//--------------------------------------------------------------------------------------------------
/**
 * Timer handler will monitor information on the battery charge status
 * If indication is that battery is full, then it will update the LTC charge register to
 * maximum battery charge capacity in mAh
 */
//--------------------------------------------------------------------------------------------------
static void BatteryTimerExpiryHandler
(
    le_timer_Ref_t batteryTimerRef  ///< not used (MAY BE NULL)
)
{
    // Update the charge flow counter.
    // Note: The charge counters must only be updated on a timer tick so that we can accurately
    //       derive the current flow over time.
    ReadChargeCounter();

    // Compute the current flow.
    // ChargeCounter counts uAh. Counting upward = charging, downward = draining.
    #define MS_PER_HOUR (1000 * 60 * 60)
    double mAh = (double)(ChargeCounter - OldChargeCounter) / 1000;
    double h = (double)le_timer_GetMsInterval(batteryTimerRef) / MS_PER_HOUR;
    CurrentFlow = ( mAh / h );

    // Update the charging status.
    ReadChargingStatus();

    RunStateMachine(EVENT_TIMER_EXPIRED);
}


COMPONENT_INIT
{
   // le_msg_AddServiceCloseHandler(ma_battery_GetServiceRef(), ClientSessionClosedHandler, NULL);

    // String describing the battery technology.
    LE_ASSERT(LE_OK == dhubIO_CreateOutput(RES_PATH_TECH, DHUBIO_DATA_TYPE_STRING, ""));
    dhubIO_AddStringPushHandler(RES_PATH_TECH, SetTechnology, NULL);
    dhubIO_MarkOptional(RES_PATH_TECH);

    // Nominal voltage of the battery when charged.
    LE_ASSERT(LE_OK == dhubIO_CreateOutput(RES_PATH_NOM_VOLTAGE, DHUBIO_DATA_TYPE_NUMERIC, "V"));
    dhubIO_AddNumericPushHandler(RES_PATH_NOM_VOLTAGE, SetNominalVoltage, NULL);
    dhubIO_MarkOptional(RES_PATH_NOM_VOLTAGE);

    // Amount of charge the battery can store (mAh).
    LE_ASSERT(LE_OK == dhubIO_CreateOutput(RES_PATH_CAPACITY, DHUBIO_DATA_TYPE_NUMERIC, "mAh"));
    dhubIO_AddNumericPushHandler(RES_PATH_CAPACITY, SetCapacity, NULL);

    // Sample period (seconds).
    LE_ASSERT(LE_OK == dhubIO_CreateOutput(RES_PATH_PERIOD, DHUBIO_DATA_TYPE_NUMERIC, "s"));
    dhubIO_AddNumericPushHandler(RES_PATH_PERIOD, SetPeriod, NULL);
    dhubIO_SetNumericDefault(RES_PATH_PERIOD, ((double)DEFAULT_BATTERY_SAMPLE_INTERVAL_MS) / 1000);

    // Sensor data flowing into the Data Hub as a JSON structure.
    LE_ASSERT(LE_OK == dhubIO_CreateInput(RES_PATH_VALUE, DHUBIO_DATA_TYPE_JSON, ""));
    dhubIO_SetJsonExample(RES_PATH_VALUE, JSON_EXAMPLE);

    LevelAlarmPool   = le_mem_CreatePool("batt_events", sizeof(LevelAlarmReg_t));
    LevelAlarmRefMap = le_ref_CreateMap("batt_events", 4);

    ChargingStatusRegPool   = le_mem_CreatePool("charge_events", sizeof(ChargingStatusReg_t));
    ChargingStatusRegRefMap = le_ref_CreateMap("charge_events", 4);

    HealthStatusRegPool   = le_mem_CreatePool("health_events", sizeof(HealthStatusReg_t));
    HealthStatusRegRefMap = le_ref_CreateMap("health_events", 4);

    // Set up the timer, but don't start it until we know we are configured.
    Timer = le_timer_Create("Battery Service Timer");
    le_timer_SetMsInterval(Timer, DEFAULT_BATTERY_SAMPLE_INTERVAL_MS);
    le_timer_SetRepeat(Timer, 0);
    le_timer_SetHandler(Timer, BatteryTimerExpiryHandler);

    // Read the battery technology configuration settings from the Config Tree.
    char type[MA_BATTERY_MAX_BATT_TYPE_STR_LEN + 1];
    uint16_t mAh; // mAh
    uint16_t mV; // mV
    le_result_t result = ma_battery_GetTechnology(type, sizeof(type), &mAh, &mV);
    if (result != LE_OK)
    {
        LE_ERROR("Battery monitor is not configured.");

        // Remain in the UNCONFIGURED state without the timer running.
    }
    else
    {
        Capacity = mAh;

        // Read the charge counter and remember the value to be compared against later.
        ReadChargeCounter();
        OldChargeCounter = ChargeCounter;   // To prevent wild mA measurements on subsequent reads.

        // Set the default values of the configuration settings resources in the Data Hub.
        if (type[0] != '\0')
        {
            dhubIO_SetStringDefault(RES_PATH_TECH, type);
        }
        if (mV > 0)
        {
            dhubIO_SetNumericDefault(RES_PATH_NOM_VOLTAGE, ((double)mV) / 1000);
        }
        dhubIO_SetNumericDefault(RES_PATH_CAPACITY, Capacity);

        // Read the charge level percentage from the config tree.
        int percent = LoadPercentage();
        if (percent >= 0)
        {
            // Tell the battery monitor what level we think the battery is at.
            UpdateChargeLevel(Capacity * percent / 100);

            // Enter the NOMINAL state.
            State = STATE_NOMINAL;
        }
        else
        {
            // The charge level percentage wasn't saved, so it must not have been calibrated,
            // and we don't even know if we have a battery connected, so go into the
            // DETECTING_PRESENCE state.
            State = STATE_DETECTING_PRESENCE;
        }

        // Start the update timer.
        le_timer_Start(Timer);
    }

    LE_INFO("---------------------- Battery Service started");
}
