//--------------------------------------------------------------------------------------------------
/**
 * @file batteryUtils.c
 *
 * File access utilities used by the Battery Service.
 */
//--------------------------------------------------------------------------------------------------

#include "legato.h"
#include "batteryUtils.h"

le_result_t util_ReadStringFromFile
(
    const char *filePath,
    char *value,
    size_t valueSize
)
{
    le_result_t r = LE_OK;
    FILE *f       = fopen(filePath, "r");

    if (f == NULL)
    {
        LE_WARN("Couldn't open '%s' - %m", filePath);
        r = LE_IO_ERROR;
        goto done;
    }

    bool eofReached = false;
    size_t numRead  = 0;
    while (numRead < valueSize - 1)
    {
        size_t freadRes = fread(&value[numRead], 1, valueSize - 1 - numRead, f);
        numRead += freadRes;
        if (feof(f))
        {
            if (numRead > 0 && value[numRead - 1] == '\n')
            {
                value[numRead - 1] = '\0';
            }
            else
            {
                value[numRead] = '\0';
            }
            eofReached = true;
            break;
        }
        if (ferror(f))
        {
            r = LE_IO_ERROR;
            goto cleanup;
        }
    }

    if (!eofReached)
    {
        r = LE_OVERFLOW;
        goto cleanup;
    }

cleanup:
    fclose(f);
done:
    return r;
}

le_result_t util_ReadIntFromFile
(
    const char *filePath,
    int *value
)
{
    char buffer[16];
    le_result_t r = util_ReadStringFromFile(filePath, buffer, sizeof(buffer));
    if (r == LE_OK)
    {
        int charsScanned = 0;
        int sscanfRes    = sscanf(buffer, "%d%n", value, &charsScanned);
        if (sscanfRes == 1)
        {
            if (charsScanned == strlen(buffer))
            {
                r = LE_OK;
            }
            else
            {
                r = LE_FORMAT_ERROR;
            }
        }
        else
        {
            r = LE_FORMAT_ERROR;
        }
    }
    return r;
}


le_result_t util_ReadDoubleFromFile
(
    const char *filePath,
    double *value
)
{
    char buffer[32];
    le_result_t r = util_ReadStringFromFile(filePath, buffer, sizeof(buffer));
    if (r == LE_OK)
    {
        int charsScanned = 0;
        int sscanfRes    = sscanf(buffer, "%lf%n", value, &charsScanned);
        if (sscanfRes == 1)
        {
            if (charsScanned == strlen(buffer))
            {
                r = LE_OK;
            }
            else
            {
                r = LE_FORMAT_ERROR;
            }
        }
        else
        {
            r = LE_FORMAT_ERROR;
        }
    }
    return r;
}


le_result_t util_WriteIntToFile
(
    const char *filePath,
    int value
)
{
    char intStr[12];
    le_result_t r = LE_OK;
    FILE *f       = fopen(filePath, "w");
    if (f == NULL)
    {
        LE_WARN("Couldn't open '%s' - %m", filePath);
        r = LE_IO_ERROR;
        goto done;
    }

    int bytesRequired = snprintf(intStr, sizeof(intStr), "%d", value);
    LE_ASSERT(bytesRequired < sizeof(intStr));

    const size_t numWritten = fwrite(intStr, 1, bytesRequired, f);
    if (numWritten != bytesRequired)
    {
        r = LE_IO_ERROR;
    }

    fclose(f);
done:
    return r;
}


COMPONENT_INIT
{
}
