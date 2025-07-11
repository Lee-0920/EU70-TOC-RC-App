/**
 * @file
 * @brief 光学信号AD采集驱动头文件。
 * @details 提供光学信号AD采集功能接口。
 * @version 1.1.0
 * @author kim.xiejinqiang
 * @date 2015-06-04
 */
#ifndef SRC_DRIVER_OPTICALDRIVER_AD7791COLLECT_H_
#define SRC_DRIVER_OPTICALDRIVER_AD7791COLLECT_H_


#include "stm32f4xx.h"
#include "Common/Types.h"
#ifdef __cplusplus
extern "C" {
#endif

// 极性
#define AD7791_MODE_UNIPOLAR   (1 << 2)     // 单极性
#define AD7791_MODE_BIPOLAR    0x00         // 双极性

typedef struct
{
	GPIO_TypeDef *portCS;
	Uint16 pinCS;
	uint32_t rccCS;
    GPIO_TypeDef *portSCK;
    Uint16 pinSCK;
    uint32_t rccSCK;
    GPIO_TypeDef *portDIN;
    Uint16 pinDIN;
    uint32_t rccDIN;
    GPIO_TypeDef *portDOUT;
	Uint16 pinDOUT;
	uint32_t rccDOUT;
}AD7791Driver;

void AD7791Collect_Init(AD7791Driver* ad7791);
uint32_t AD7791Collect_GetAD(AD7791Driver* ad7791);
void AD7791Collect_ChangePolar(unsigned char polar);
float AD7791Collect_GetResult(Int32 tempData);
void AD7791Collect_Test(AD7791Driver* ad7791);
#ifdef __cplusplus
}
#endif


#endif /* SRC_DRIVER_OPTICALDRIVER_AD7791COLLECT_H_ */
