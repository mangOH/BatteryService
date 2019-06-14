//--------------------------------------------------------------------------------------------------
/**
 * @file batteryUtils.h
 *
 * File access utilities used by the Battery Service.
 */
//--------------------------------------------------------------------------------------------------

#ifndef BATTERY_UTILS_H
#define BATTERY_UTILS_H

#include "legato.h"

LE_SHARED le_result_t util_ReadIntFromFile(const char *filePath, int *value);
LE_SHARED le_result_t util_ReadDoubleFromFile(const char *filePath, double *value);
LE_SHARED le_result_t util_ReadStringFromFile(const char *filePath, char *value, size_t valueSize);
LE_SHARED le_result_t util_WriteIntToFile(const char *filepath, int value);

#endif // BATTERY_UTILS_H
