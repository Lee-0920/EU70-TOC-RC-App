/*
 * UltraPwmDevice.h
 *
 *  Created on: 2019年8月16日
 *      Author: Administrator
 */

#ifndef SRC_DRIVER_ULTRAPWMDEVICE_H_
#define SRC_DRIVER_ULTRAPWMDEVICE_H_

#include <UltraSignalDriver/UltraPWMDeviceDriver.h>

typedef enum
{
    UltraPwmDevice_IDLE = 0,
    UltraPwmDevice_BUSY = 1,
}UltraPwmDeviceStatus;

typedef Bool (*SetOutputWayFunc)(UltraPwmDeviceDriver *deviceDriver, float level);

typedef struct
{
    UltraPwmDeviceStatus status;
    float maxDutyCycle;
    UltraPwmDeviceDriver deviceDriver;
    SetOutputWayFunc setOutputWayFunc;
}UltraPwmDevice;

void UltraPwmDevice_Init(UltraPwmDevice *device);
UltraPwmDeviceStatus UltraPwmDevice_GetStatus(UltraPwmDevice *device);
Bool UltraPwmDevice_SetOutput(UltraPwmDevice *device, float level);
float UltraPwmDevice_GetMaxDutyCycle(UltraPwmDevice *device);
Bool UltraPwmDevice_SetMaxDutyCycle(UltraPwmDevice *device, float value);

#endif /* SRC_DRIVER_TEMPDRIVER_UltraPwmDevice_H_ */
