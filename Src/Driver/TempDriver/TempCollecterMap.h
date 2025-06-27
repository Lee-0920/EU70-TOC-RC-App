/*
 * TempCollecterMap.h
 *
 *  Created on: 2019年8月16日
 *      Author: Administrator
 */

#ifndef SRC_DRIVER_TEMPDRIVER_TEMPCOLLECTERMAP_H_
#define SRC_DRIVER_TEMPDRIVER_TEMPCOLLECTERMAP_H_

#include <TempDriver/TempCollecter.h>

#define MEASUREMODULE1_TEMP       0
#define MEASUREMODULE2_TEMP       1
#define MEASUREMODULE3_TEMP       2
#define MEASUREMODULE4_TEMP       3

#define TOTAL_TEMP                4

void TempCollecterMap_Init(TempCollecter *tempCollecter);
char* TempCollecterMap_GetName(Uint8 index);

#endif /* SRC_DRIVER_TEMPDRIVER_TEMPCOLLECTERMAP_H_ */
