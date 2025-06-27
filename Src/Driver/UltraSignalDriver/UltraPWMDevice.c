/*
 * UltraPwmDevice.c
 *
 *  Created on: 2019年8月16日
 *      Author: Administrator
 */

#include "UltraPWMDevice.h"
#include "Tracer/Trace.h"
#include "Driver/System.h"

void UltraPwmDevice_Init(UltraPwmDevice *device)
{
    device->status = UltraPwmDevice_IDLE;
    UltraPwmDeviceDriver_Init(&device->deviceDriver);
}

UltraPwmDeviceStatus UltraPwmDevice_GetStatus(UltraPwmDevice *device)
{
    return device->status;
}

Bool UltraPwmDevice_SetOutput(UltraPwmDevice *device, float level)
{
    Bool ret = FALSE;
    if (level <= 1 && level >= 0)
    {
        if (0 == level)
        {
            device->status = UltraPwmDevice_IDLE;
        }
        else
        {
            device->status = UltraPwmDevice_BUSY;
            level = device->maxDutyCycle * level;
        }
        ret = device->setOutputWayFunc(&device->deviceDriver, level);
    }
    else
    {
        TRACE_ERROR("\n SetOutput out of range, level: ");
        System_PrintfFloat(TRACE_LEVEL_ERROR, level, 3);
    }
    return ret;
}

float UltraPwmDevice_GetMaxDutyCycle(UltraPwmDevice *device)
{
    TRACE_INFO("\n GetMaxDutyCycle maxDutyCycle: ");
    System_PrintfFloat(TRACE_LEVEL_INFO, device->maxDutyCycle, 3);
    return device->maxDutyCycle;
}

Bool UltraPwmDevice_SetMaxDutyCycle(UltraPwmDevice *device, float value)
{
    if (value <= 1 && value >= 0)
    {
        TRACE_INFO("\n SetMaxDutyCycle maxDutyCycle: ");
        System_PrintfFloat(TRACE_LEVEL_INFO, value, 3);
        device->maxDutyCycle = value;
        return TRUE;
    }
    else
    {
        TRACE_ERROR("\n SetMaxDutyCycle out of range, value: ");
        System_PrintfFloat(TRACE_LEVEL_ERROR, value, 3);
        return FALSE;
    }
}

