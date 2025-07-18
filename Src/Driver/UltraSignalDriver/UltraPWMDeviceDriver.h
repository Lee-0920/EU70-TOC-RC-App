/*
 * UltraPwmDeviceDriver.h
 *
 *  Created on: 2019年8月16日
 *      Author: Administrator
 */

#ifndef SRC_DRIVER_ULTRAUltraPwmDeviceDriver_H_
#define SRC_DRIVER_ULTRAUltraPwmDeviceDriver_H_

#include "stm32f4xx.h"
#include "Common/Types.h"

typedef enum
{
    UltraPwmDeviceDriver_PWM = 0,
    UltraPwmDeviceDriver_IO = 1,
    UltraPwmDeviceDriver_VIRTUAL_PWM = 2,
}UltraPwmDeviceDriverMode;

typedef void(*TimerRccInitFunction)(uint32_t RCC_Periph, FunctionalState NewState);

typedef struct
{
    uint8_t pinSource;
    uint8_t goipAF;
    TimerRccInitFunction timerRccInitFunction;
    uint32_t timerRcc;
    uint16_t timerPrescaler;
    uint32_t timerPeriod;
    uint16_t timerChannel;
    TIM_TypeDef* timer;
    uint16_t timerOCPolarity;
    uint16_t timerOCMode;
}PWMModeConfig;

typedef struct
{
    BitAction open;
    BitAction close;
}IOModeConfig;

typedef union
{
    PWMModeConfig PWMConfig;
    IOModeConfig IOConfig;
}ModeConfig;

typedef struct
{
    UltraPwmDeviceDriverMode mode;
    GPIO_TypeDef * port;
    uint32_t pin;
    uint32_t gpioRcc;
    ModeConfig modeConfig;
    Bool isExtra;
}UltraPwmDeviceDriver;

void UltraPwmDeviceDriver_Init(UltraPwmDeviceDriver *deviceDriver);
Bool UltraPwmDeviceDriver_SetOutput(UltraPwmDeviceDriver *deviceDriver, float level);
void UltraPwmDeviceDriver_PwmFunc(void *device, Bool state);

#endif /* SRC_DRIVER_TEMPDRIVER_UltraPwmDeviceDriver_H_ */
