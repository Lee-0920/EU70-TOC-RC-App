/**
 * @file
 * @brief 消解温度采集驱动头文件。
 * @details 提供采集消解温度功能接口。
 * @version 1.1.0
 * @author kim.xiejinqiang
 * @date 2015-06-04
 */

#ifndef SRC_DRIVER_TEMPDRIVER_TEMPADCOLLECT_H_
#define SRC_DRIVER_TEMPDRIVER_TEMPADCOLLECT_H_

/* Includes ------------------------------------------------------------------*/
#include "stm32f4xx.h"

#ifdef __cplusplus
extern "C" {
#endif

#define TEMP_INDEX_0      	 0     // 加热炉温度AD获取索引
#define TEMP_INDEX_1      	 1     // 制冷温度AD获取索引
#define TEMP_INDEX_2      	 2     // 加热炉温度AD获取索引
#define TEMP_INDEX_3      	 3     // 制冷温度AD获取索引

void TempADCollect_Init(void);
uint16_t TempADCollect_GetAD(uint8_t index);
#ifdef __cplusplus
}
#endif

#endif /* SRC_DRIVER_TEMPDRIVER_TEMPADCOLLECT_H_ */
