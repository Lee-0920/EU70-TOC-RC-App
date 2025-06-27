/*
 * HardwareFlag.c
 *
 *  Created on: 2020年4月10日
 *      Author: Administrator
 */

#include "stm32f4xx.h"
#include <HardwareType.h>

#define GPIO_ADDRESS_PIN         GPIO_Pin_0
#define GPIO_ADDRESS_PORT        GPIOB
#define ADDRESS 			     GPIO_ReadInputDataBit(GPIO_ADDRESS_PORT, GPIO_ADDRESS_PIN)

/**
 * @brief 硬件板卡类型标记初始化
 */
void HardwareType_Init(void)
{
    GPIO_InitTypeDef GPIO_InitStructure;
    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB, ENABLE);

    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_2;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN;
    GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_DOWN;
    GPIO_Init(GPIO_ADDRESS_PORT, &GPIO_InitStructure);
}

/**
 * @brief 读取硬件版本标记
 * @return 标记值
 */
Uint8 HardwareType_GetValue(void)
{
    Uint8 value = 0;

    if(ADDRESS)
    {
    	value = 1;
    }

    return value;
}

