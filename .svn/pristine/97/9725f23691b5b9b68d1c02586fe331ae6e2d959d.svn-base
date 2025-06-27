/*
 * PwmDriver.h
 *
 *  Created on: 2020年6月23日
 *      Author: Administrator
 */

#ifndef SRC_DRIVER_TEMPDRIVER_PWMDRIVER_H_
#define SRC_DRIVER_TEMPDRIVER_PWMDRIVER_H_

#include "stm32f4xx.h"
#include "Common/Types.h"

typedef void (*PwmDriverTimer_Handle)(void*, Bool);
typedef void (*ExPwmDriverTimer_Handle)(void*, Bool);

void PwmDriver_Init(void);
Bool PwmDriver_SetPwm(float level);
float PwmDriver_GetPwm(void);
void PwmDriver_RegisterDeviceHandle(void* device, PwmDriverTimer_Handle handle);
void ExPwmDriver_Init(void);
Bool ExPwmDriver_SetPwm(float level);
float ExPwmDriver_GetPwm(void);
void ExPwmDriver_RegisterDeviceHandle(void* device, PwmDriverTimer_Handle handle);

#endif /* SRC_DRIVER_TEMPDRIVER_PWMDRIVER_H_ */
