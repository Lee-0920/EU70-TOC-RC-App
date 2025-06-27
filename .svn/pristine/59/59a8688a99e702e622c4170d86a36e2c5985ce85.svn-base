/*
 * ADS1220Driver.h
 *
 *  Created on: 2022年2月11日
 *      Author: lwq
 */

#ifndef SRC_DRIVER_OPTICALDRIVER_ADS1220DRIVER_H_
#define SRC_DRIVER_OPTICALDRIVER_ADS1220DRIVER_H_

#include "stm32f4xx.h"
#include "Common/Types.h"

#ifdef __cplusplus
extern "C" {
#endif

void ADS1220Driver_Init(void);
Uint32 ADS1220Driver_GetAD(void);
Uint32 ADS1220Driver_GetADWithSync(void);
Bool ADS1220Driver_GetSigelChannelAD(Uint32 *channel1Buff, Uint8 len, Uint32 sampleTimeOut);
Bool ADS1220Driver_GetDoubleChannelAD(Uint16 *channel1Buff, Uint8 channel1, Uint16 *channel2Buff, Uint8 channel2, Uint8 len, Uint32 sampleTimeOut);
float ADS1220Driver_GetData(Int32 tempData);
float ADS1220Driver_GetResult(Int32 tempData);
void ADS1220Driver_SetSPS(Uint16 value);
void ADS1220Driver_RegList(void);

#endif /* SRC_DRIVER_OPTICALDRIVER_ADS1220Driver_H_ */
