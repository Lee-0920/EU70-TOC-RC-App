/*
 * ValveManager.h
 *
 *  Created on: 2016年6月7日
 *      Author: Administrator
 */

#ifndef SRC_SOLENOIDVALVE_VALVEMANAGER_H_
#define SRC_SOLENOIDVALVE_VALVEMANAGER_H_

#include "Common/Types.h"

#define SOLENOIDVALVECONF_TOTALVAlVES         	2		//最大阀数
#define SOLENOIDVALVE_MAX_MAP                 0xF

void ValveManager_Init(void);
Uint16 ValveManager_GetTotalValves(void);
Bool ValveManager_SetValvesMap(Uint32 map);
Uint32 ValveManager_GetValvesMap(void);
Bool ValveManager_SetValvesMapNormalOpen(Uint32 map);
#endif /* SRC_SOLENOIDVALVE_VALVEMANAGER_H_ */
