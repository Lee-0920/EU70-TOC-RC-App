/*
 * UltraPwmDeviceMap.c
 *
 *  Created on: 2019年8月16日
 *      Author: Administrator
 */

/*
 * UltraPwmDeviceMap.c
 *
 *  Created on: 2017年11月16日
 *      Author: LIANG
 */

#include "UltraPWMDeviceMap.h"
#include "Driver/System.h"
#include <string.h>
#include "Tracer/Trace.h"
#include "UltraPWMDeviceDriver.h"
#include "UltraPWMDevice.h"

//加热丝输出
static Bool UltraPwmDeviceMap_SetOutputWay1(UltraPwmDeviceDriver *deviceDriver, float level);

//风扇输出
static Bool UltraPwmDeviceMap_SetOutputWay2(UltraPwmDeviceDriver *deviceDriver, float level);

//电炉输出
static Bool UltraPwmDeviceMap_SetOutputWay3(UltraPwmDeviceDriver *deviceDriver, float level);

void UltraPwmDeviceMap_Init(UltraPwmDevice* device)
{
//	device[0].maxDutyCycle = 1;
//	device[0].setOutputWayFunc = UltraPwmDeviceMap_SetOutputWay1;
//	device[0].deviceDriver.mode = UltraPwmDeviceDriver_PWM;
//	device[0].deviceDriver.port = GPIOB;
//	device[0].deviceDriver.pin = GPIO_Pin_5;
//	device[0].deviceDriver.modeConfig.PWMConfig.pinSource = GPIO_PinSource5;
//	device[0].deviceDriver.modeConfig.PWMConfig.goipAF = GPIO_AF_TIM3;
//	device[0].deviceDriver.modeConfig.PWMConfig.timerRccInitFunction = RCC_APB1PeriphClockCmd;
//	device[0].deviceDriver.modeConfig.PWMConfig.timerRcc = RCC_APB1Periph_TIM3;
//	device[0].deviceDriver.modeConfig.PWMConfig.timerPrescaler = 44;
//	device[0].deviceDriver.modeConfig.PWMConfig.timerPeriod = 9999;
//	device[0].deviceDriver.modeConfig.PWMConfig.timerChannel = 1;
//	device[0].deviceDriver.modeConfig.PWMConfig.timer = TIM3;
//	device[0].deviceDriver.modeConfig.PWMConfig.timerOCPolarity = TIM_OCPolarity_Low;
//	device[0].deviceDriver.modeConfig.PWMConfig.timerOCMode = TIM_OCMode_PWM2;//在向上计数模式下，TIMx_CNT < TIMx_CCR1时，通道1为无效电平
//	UltraPwmDevice_Init(&device[0]);
//
//	//制冷片
//	device[1].maxDutyCycle = 1;
//	device[1].setOutputWayFunc = UltraPwmDeviceMap_SetOutputWay1;
//	device[1].deviceDriver.mode = UltraPwmDeviceDriver_PWM;
//	device[1].deviceDriver.port = GPIOB;
//	device[1].deviceDriver.pin = GPIO_Pin_6;
//	device[1].deviceDriver.modeConfig.PWMConfig.pinSource = GPIO_PinSource6;
//	device[1].deviceDriver.modeConfig.PWMConfig.goipAF = GPIO_AF_TIM4;
//	device[1].deviceDriver.modeConfig.PWMConfig.timerRccInitFunction = RCC_APB1PeriphClockCmd;
//	device[1].deviceDriver.modeConfig.PWMConfig.timerRcc = RCC_APB1Periph_TIM4;
//	device[1].deviceDriver.modeConfig.PWMConfig.timerPrescaler = 44;
//	device[1].deviceDriver.modeConfig.PWMConfig.timerPeriod = 9999;
//	device[1].deviceDriver.modeConfig.PWMConfig.timerChannel = 2;
//	device[1].deviceDriver.modeConfig.PWMConfig.timer = TIM4;
//	device[1].deviceDriver.modeConfig.PWMConfig.timerOCPolarity = TIM_OCPolarity_Low;
//	device[1].deviceDriver.modeConfig.PWMConfig.timerOCMode = TIM_OCMode_PWM2;//在向上计数模式下，TIMx_CNT < TIMx_CCR1时，通道1为无效电平
//	UltraPwmDevice_Init(&device[1]);
//
//	device[2].maxDutyCycle = 1;
//	device[2].setOutputWayFunc = UltraPwmDeviceMap_SetOutputWay1;
//	device[2].deviceDriver.mode = UltraPwmDeviceDriver_PWM;
//	device[2].deviceDriver.port = GPIOC;
//	device[2].deviceDriver.pin = GPIO_Pin_12;
//	device[2].deviceDriver.modeConfig.PWMConfig.pinSource = GPIO_PinSource12;
//	device[2].deviceDriver.modeConfig.PWMConfig.goipAF = GPIO_AF_TIM3;
//	device[2].deviceDriver.modeConfig.PWMConfig.timerRccInitFunction = RCC_APB1PeriphClockCmd;
//	device[2].deviceDriver.modeConfig.PWMConfig.timerRcc = RCC_APB1Periph_TIM3;
//	device[2].deviceDriver.modeConfig.PWMConfig.timerPrescaler = 44;
//	device[1].deviceDriver.modeConfig.PWMConfig.timerPeriod = 49999;
//	device[2].deviceDriver.modeConfig.PWMConfig.timerChannel = 2;
//	device[2].deviceDriver.modeConfig.PWMConfig.timer = TIM3;
//	device[2].deviceDriver.modeConfig.PWMConfig.timerOCPolarity = TIM_OCPolarity_Low;
//	device[2].deviceDriver.modeConfig.PWMConfig.timerOCMode = TIM_OCMode_PWM2;//在向上计数模式下，TIMx_CNT < TIMx_CCR1时，通道1为无效电平
//	UltraPwmDevice_Init(&device[2]);
//
//	device[3].maxDutyCycle = 1;
//	device[3].setOutputWayFunc = UltraPwmDeviceMap_SetOutputWay1;
//	device[3].deviceDriver.mode = UltraPwmDeviceDriver_PWM;
//	device[3].deviceDriver.port = GPIOC;
//	device[3].deviceDriver.pin = GPIO_Pin_7;
//	device[3].deviceDriver.modeConfig.PWMConfig.pinSource = GPIO_PinSource7;
//	device[3].deviceDriver.modeConfig.PWMConfig.goipAF = GPIO_AF_TIM3;
//	device[3].deviceDriver.modeConfig.PWMConfig.timerRccInitFunction = RCC_APB1PeriphClockCmd;
//	device[3].deviceDriver.modeConfig.PWMConfig.timerRcc = RCC_APB1Periph_TIM3;
//	device[3].deviceDriver.modeConfig.PWMConfig.timerPrescaler = 44;
//	device[1].deviceDriver.modeConfig.PWMConfig.timerPeriod = 49999;
//	device[3].deviceDriver.modeConfig.PWMConfig.timerChannel = 2;
//	device[3].deviceDriver.modeConfig.PWMConfig.timer = TIM3;
//	device[3].deviceDriver.modeConfig.PWMConfig.timerOCPolarity = TIM_OCPolarity_Low;
//	device[3].deviceDriver.modeConfig.PWMConfig.timerOCMode = TIM_OCMode_PWM2;//在向上计数模式下，TIMx_CNT < TIMx_CCR1时，通道1为无效电平
//	UltraPwmDevice_Init(&device[3]);

    TRACE_INFO("\n Ultra Device init over.");
}

static Bool UltraPwmDeviceMap_SetOutputWay1(UltraPwmDeviceDriver *deviceDriver, float level)
{
    TRACE_CODE("\n Output way 1");
    return UltraPwmDeviceDriver_SetOutput(deviceDriver, level);
}

static Bool UltraPwmDeviceMap_SetOutputWay2(UltraPwmDeviceDriver *deviceDriver, float level)
{
    TRACE_CODE("\n Output way 2");
    if (0 != level)
    {
    	TRACE_INFO("\n Output way 2");
        level = 0.5 * level + 0.5;
        if (level < 0.75)
        {
            UltraPwmDeviceDriver_SetOutput(deviceDriver, 1);
            System_Delay(200);
        }
    }
    return UltraPwmDeviceDriver_SetOutput(deviceDriver, level);
}

static Bool UltraPwmDeviceMap_SetOutputWay3(UltraPwmDeviceDriver *deviceDriver, float level)
{
    TRACE_CODE("\n Output way 3");
    if (0 != level)
    {
        if (level < 0.05)
        {
        	level = 0.05;
        }
        else if(level > 0.7)
        {
			level = 0.7;
        }
    }
    return UltraPwmDeviceDriver_SetOutput(deviceDriver, level);
}

char* UltraPwmDeviceMap_GetName(Uint8 index)
{
    static char name[5] = "Null";
    return name;
}
