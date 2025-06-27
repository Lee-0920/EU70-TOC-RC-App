/*
 * PumpMap.c
 *
 *  Created on: 2016年5月30日
 *      Author: Administrator
 */
#include "stm32f4xx.h"
#include "PumpDriver.h"
#include "PumpMap.h"
#include "HardwareType.h"

void PumpMap_Init(Pump *pump)
{
//	pump[0].driver.pinClock = GPIO_Pin_11;
//	pump[0].driver.portClock = GPIOD;
//	pump[0].driver.rccClock = RCC_AHB1Periph_GPIOD;
//
//	pump[0].driver.pinDir = GPIO_Pin_10;
//	pump[0].driver.portDir = GPIOD;
//	pump[0].driver.rccDir = RCC_AHB1Periph_GPIOD;
//
//	pump[0].driver.pinDiag = GPIO_Pin_12;
//	pump[0].driver.portDiag = GPIOD;
//	pump[0].driver.rccDiag = RCC_AHB1Periph_GPIOD;
//
////	pump[0].driver.pinReset = GPIO_Pin_13;
////	pump[0].driver.portReset = GPIOB;
////	pump[0].driver.rccReset = RCC_AHB1Periph_GPIOB;
//
//    PumpDriver_Init(&pump[0].driver);
//    PumpDriver_PullLow(&pump[0].driver);
//    PumpDriver_SetForwardLevel(&pump[0].driver,Bit_RESET);
//
//	pump[1].driver.pinClock = GPIO_Pin_14;
//	pump[1].driver.portClock = GPIOD;
//	pump[1].driver.rccClock = RCC_AHB1Periph_GPIOD;
//
//	pump[1].driver.pinDir = GPIO_Pin_13;
//	pump[1].driver.portDir = GPIOD;
//	pump[1].driver.rccDir = RCC_AHB1Periph_GPIOD;
//
//	pump[1].driver.pinDiag = GPIO_Pin_12;
//	pump[1].driver.portDiag = GPIOD;
//	pump[1].driver.rccDiag = RCC_AHB1Periph_GPIOD;
//
////	pump[1].driver.pinReset = GPIO_Pin_15;
////	pump[1].driver.portReset = GPIOA;
////	pump[1].driver.rccReset = RCC_AHB1Periph_GPIOA;
//
//    PumpDriver_Init(&pump[1].driver);
//    PumpDriver_PullLow(&pump[1].driver);
//    PumpDriver_SetForwardLevel(&pump[1].driver,Bit_RESET);
//
//	pump[2].driver.pinClock = GPIO_Pin_6;
//	pump[2].driver.portClock = GPIOC;
//	pump[2].driver.rccClock = RCC_AHB1Periph_GPIOC;
//
//	pump[2].driver.pinDir = GPIO_Pin_15;
//	pump[2].driver.portDir = GPIOD;
//	pump[2].driver.rccDir = RCC_AHB1Periph_GPIOD;
//
//	pump[2].driver.pinDiag = GPIO_Pin_12;
//	pump[2].driver.portDiag = GPIOD;
//	pump[2].driver.rccDiag = RCC_AHB1Periph_GPIOD;
//
////	pump[2].driver.pinReset = GPIO_Pin_10;
////	pump[2].driver.portReset = GPIOC;
////	pump[2].driver.rccReset = RCC_AHB1Periph_GPIOC;
//
//    PumpDriver_Init(&pump[2].driver);
//    PumpDriver_PullLow(&pump[2].driver);
//    PumpDriver_SetForwardLevel(&pump[2].driver,Bit_RESET);
//
//    pump[3].driver.pinClock = GPIO_Pin_8;
//	pump[3].driver.portClock = GPIOC;
//	pump[3].driver.rccClock = RCC_AHB1Periph_GPIOC;
//
//	pump[3].driver.pinDir = GPIO_Pin_7;
//	pump[3].driver.portDir = GPIOC;
//	pump[3].driver.rccDir = RCC_AHB1Periph_GPIOC;
//
//	pump[3].driver.pinDiag = GPIO_Pin_12;
//	pump[3].driver.portDiag = GPIOD;
//	pump[3].driver.rccDiag = RCC_AHB1Periph_GPIOD;
//
////	pump[3].driver.pinReset = GPIO_Pin_10;
////	pump[3].driver.portReset = GPIOC;
////	pump[3].driver.rccReset = RCC_AHB1Periph_GPIOC;
//
//	PumpDriver_Init(&pump[3].driver);
//	PumpDriver_PullLow(&pump[3].driver);
//	PumpDriver_SetForwardLevel(&pump[3].driver,Bit_RESET);
//
//	pump[4].driver.pinClock = GPIO_Pin_11;
//	pump[4].driver.portClock = GPIOA;
//	pump[4].driver.rccClock = RCC_AHB1Periph_GPIOA;
//
//	pump[4].driver.pinDir = GPIO_Pin_12;
//	pump[4].driver.portDir = GPIOA;
//	pump[4].driver.rccDir = RCC_AHB1Periph_GPIOA;
//
//	pump[4].driver.pinDiag = GPIO_Pin_5;
//	pump[4].driver.portDiag = GPIOD;
//	pump[4].driver.rccDiag = RCC_AHB1Periph_GPIOD;
//
////	pump[4].driver.pinReset = GPIO_Pin_13;
////	pump[4].driver.portReset = GPIOB;
////	pump[4].driver.rccReset = RCC_AHB1Periph_GPIOB;
//
//	PumpDriver_Init(&pump[4].driver);
//	PumpDriver_PullLow(&pump[4].driver);
//	PumpDriver_SetForwardLevel(&pump[4].driver,Bit_RESET);
//
//	pump[5].driver.pinClock = GPIO_Pin_1;
//	pump[5].driver.portClock = GPIOD;
//	pump[5].driver.rccClock = RCC_AHB1Periph_GPIOD;
//
//	pump[5].driver.pinDir = GPIO_Pin_3;
//	pump[5].driver.portDir = GPIOD;
//	pump[5].driver.rccDir = RCC_AHB1Periph_GPIOD;
//
//	pump[5].driver.pinDiag = GPIO_Pin_5;
//	pump[5].driver.portDiag = GPIOD;
//	pump[5].driver.rccDiag = RCC_AHB1Periph_GPIOD;
//
////	pump[5].driver.pinReset = GPIO_Pin_15;
////	pump[5].driver.portReset = GPIOA;
////	pump[5].driver.rccReset = RCC_AHB1Periph_GPIOA;
//
//	PumpDriver_Init(&pump[5].driver);
//	PumpDriver_PullLow(&pump[5].driver);
//	PumpDriver_SetForwardLevel(&pump[5].driver,Bit_RESET);
//
//	pump[6].driver.pinClock = GPIO_Pin_7;
//	pump[6].driver.portClock = GPIOD;
//	pump[6].driver.rccClock = RCC_AHB1Periph_GPIOD;
//
//	pump[6].driver.pinDir = GPIO_Pin_6;
//	pump[6].driver.portDir = GPIOD;
//	pump[6].driver.rccDir = RCC_AHB1Periph_GPIOD;
//
//	pump[6].driver.pinDiag = GPIO_Pin_5;
//	pump[6].driver.portDiag = GPIOD;
//	pump[6].driver.rccDiag = RCC_AHB1Periph_GPIOD;
//
////	pump[6].driver.pinReset = GPIO_Pin_15;
////	pump[6].driver.portReset = GPIOA;
////	pump[6].driver.rccReset = RCC_AHB1Periph_GPIOA;
//
//	PumpDriver_Init(&pump[6].driver);
//	PumpDriver_PullLow(&pump[6].driver);
//	PumpDriver_SetForwardLevel(&pump[6].driver,Bit_RESET);
//
//	pump[7].driver.pinClock = GPIO_Pin_4;
//	pump[7].driver.portClock = GPIOB;
//	pump[7].driver.rccClock = RCC_AHB1Periph_GPIOB;
//
//	pump[7].driver.pinDir = GPIO_Pin_3;
//	pump[7].driver.portDir = GPIOB;
//	pump[7].driver.rccDir = RCC_AHB1Periph_GPIOB;
//
//	pump[7].driver.pinDiag = GPIO_Pin_5;
//	pump[7].driver.portDiag = GPIOD;
//	pump[7].driver.rccDiag = RCC_AHB1Periph_GPIOD;
//
////	pump[7].driver.pinReset = GPIO_Pin_10;
////	pump[7].driver.portReset = GPIOC;
////	pump[7].driver.rccReset = RCC_AHB1Periph_GPIOC;
//
//	PumpDriver_Init(&pump[7].driver);
//	PumpDriver_PullLow(&pump[7].driver);
//	PumpDriver_SetForwardLevel(&pump[7].driver,Bit_RESET);
//
//	pump[8].driver.pinClock = GPIO_Pin_6;
//	pump[8].driver.portClock = GPIOB;
//	pump[8].driver.rccClock = RCC_AHB1Periph_GPIOB;
//
//	pump[8].driver.pinDir = GPIO_Pin_5;
//	pump[8].driver.portDir = GPIOB;
//	pump[8].driver.rccDir = RCC_AHB1Periph_GPIOB;
//
//	pump[8].driver.pinDiag = GPIO_Pin_6;
//	pump[8].driver.portDiag = GPIOE;
//	pump[8].driver.rccDiag = RCC_AHB1Periph_GPIOE;
//
////	pump[8].driver.pinReset = GPIO_Pin_10;
////	pump[8].driver.portReset = GPIOC;
////	pump[8].driver.rccReset = RCC_AHB1Periph_GPIOC;
//
//	PumpDriver_Init(&pump[8].driver);
//	PumpDriver_PullLow(&pump[8].driver);
//	PumpDriver_SetForwardLevel(&pump[8].driver,Bit_RESET);
//
//	pump[9].driver.pinClock = GPIO_Pin_8;
//	pump[9].driver.portClock = GPIOB;
//	pump[9].driver.rccClock = RCC_AHB1Periph_GPIOB;
//
//	pump[9].driver.pinDir = GPIO_Pin_7;
//	pump[9].driver.portDir = GPIOB;
//	pump[9].driver.rccDir = RCC_AHB1Periph_GPIOB;
//
//	pump[9].driver.pinDiag = GPIO_Pin_6;
//	pump[9].driver.portDiag = GPIOE;
//	pump[9].driver.rccDiag = RCC_AHB1Periph_GPIOE;
//
////	pump[9].driver.pinReset = GPIO_Pin_10;
////	pump[9].driver.portReset = GPIOC;
////	pump[9].driver.rccReset = RCC_AHB1Periph_GPIOC;
//
//	PumpDriver_Init(&pump[9].driver);
//	PumpDriver_PullLow(&pump[9].driver);
//	PumpDriver_SetForwardLevel(&pump[9].driver,Bit_RESET);
//
//	pump[10].driver.pinClock = GPIO_Pin_3;
//	pump[10].driver.portClock = GPIOE;
//	pump[10].driver.rccClock = RCC_AHB1Periph_GPIOE;
//
//	pump[10].driver.pinDir = GPIO_Pin_2;
//	pump[10].driver.portDir = GPIOE;
//	pump[10].driver.rccDir = RCC_AHB1Periph_GPIOE;
//
//	pump[10].driver.pinDiag = GPIO_Pin_6;
//	pump[10].driver.portDiag = GPIOE;
//	pump[10].driver.rccDiag = RCC_AHB1Periph_GPIOE;
//
//	//	pump[10].driver.pinReset = GPIO_Pin_10;
//	//	pump[10].driver.portReset = GPIOC;
//	//	pump[10].driver.rccReset = RCC_AHB1Periph_GPIOC;
//
//	PumpDriver_Init(&pump[10].driver);
//	PumpDriver_PullLow(&pump[10].driver);
//	PumpDriver_SetForwardLevel(&pump[10].driver,Bit_RESET);
//
//	pump[11].driver.pinClock = GPIO_Pin_5;
//	pump[11].driver.portClock = GPIOE;
//	pump[11].driver.rccClock = RCC_AHB1Periph_GPIOE;
//
//	pump[11].driver.pinDir = GPIO_Pin_4;
//	pump[11].driver.portDir = GPIOE;
//	pump[11].driver.rccDir = RCC_AHB1Periph_GPIOE;
//
//	pump[11].driver.pinDiag = GPIO_Pin_6;
//	pump[11].driver.portDiag = GPIOE;
//	pump[11].driver.rccDiag = RCC_AHB1Periph_GPIOE;
//
//	//	pump[11].driver.pinReset = GPIO_Pin_10;
//	//	pump[11].driver.portReset = GPIOC;
//	//	pump[11].driver.rccReset = RCC_AHB1Periph_GPIOC;
//
//	PumpDriver_Init(&pump[11].driver);
//	PumpDriver_PullLow(&pump[11].driver);
//	PumpDriver_SetForwardLevel(&pump[11].driver,Bit_RESET);
}

