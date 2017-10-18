#ifndef BATTERY_UTILS_H
#define BATTERY_UTILS_H

#include "legato.h"

le_result_t ReadIntFromFile(const char *filePath, int *value);
le_result_t ReadDoubleFromFile(const char *filePath, double *value);
le_result_t ReadStringFromFile(const char *filePath, char *value, size_t valueSize);
le_result_t WriteIntToFile(const char *filepath, int value);
#endif // BATTERY_UTILS_H
