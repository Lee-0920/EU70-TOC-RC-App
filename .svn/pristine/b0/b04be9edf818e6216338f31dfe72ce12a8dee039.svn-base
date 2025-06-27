/*
 * ValveMap.c
 *
 *  Created on: 2016年6月7日
 *      Author: Administrator
 */

#include <LiquidDriver/ValveMap.h>
#include "stm32f4xx.h"
#include "SolenoidValve/ValveManager.h"

void ValveMap_Init(Valve *valve)
{
    Uint8 i;

    valve[0].pin = GPIO_Pin_13;
    valve[0].port = GPIOE;
    valve[0].rcc = RCC_AHB1Periph_GPIOE;
    ValveDriver_Init(&valve[0]);

    valve[1].pin = GPIO_Pin_14;
    valve[1].port = GPIOE;
    valve[1].rcc = RCC_AHB1Periph_GPIOE;
    ValveDriver_Init(&valve[1]);

    for(i = 0; i < SOLENOIDVALVECONF_TOTALVAlVES; i++)
    {
        ValveDriver_Control(&valve[i], VAlVE_CLOSE);
    }

}


