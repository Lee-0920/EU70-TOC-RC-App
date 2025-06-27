/*
 * ADS1259Driver.h
 *
 *  Created on: 2022年2月11日
 *      Author: lwq
 */

#ifndef SRC_DRIVER_OPTICALDRIVER_ADS1259DRIVER_H_
#define SRC_DRIVER_OPTICALDRIVER_ADS1259DRIVER_H_

#include "stm32f4xx.h"
#include "Common/Types.h"

#ifdef __cplusplus
extern "C" {
#endif

void ADS1259Driver_Init(void);
Uint32 ADS1259Driver_GetAD(void);
Uint32 ADS1259Driver_GetADWithSync(void);
Bool ADS1259Driver_GetSigelChannelAD(Uint32 *channel1Buff, Uint8 len, Uint32 sampleTimeOut);
Bool ADS1259Driver_GetDoubleChannelAD(Uint16 *channel1Buff, Uint8 channel1, Uint16 *channel2Buff, Uint8 channel2, Uint8 len, Uint32 sampleTimeOut);
float ADS1259Driver_GetData(Int32 tempData);
float ADS1259Driver_GetResult(Int32 tempData);


#endif /* SRC_DRIVER_OPTICALDRIVER_ADS1259DRIVER_H_ */
