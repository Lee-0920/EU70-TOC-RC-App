
#ifndef _LTC2400DRIVER_H
#define _LTC2400DRIVER_H

#include "stdint.h"
#include "Common/Types.h"

/* 片选枚举体 */
typedef enum LTC2400CS {
  LTC2400CS_Enable,
  LTC2400CS_Disable
}LTC2400CSType;

/* 时钟枚举体 */
typedef enum LTC2400Clock {
  INTERNAL_CLOCK50Hz,
  INTERNAL_CLOCK60Hz,
  EXTERNAL_CLOCK
}LTC2400ClockType;

/* LTC2400初始化结构体 */
typedef struct Ltc2400Object {
  LTC2400ClockType clock;       			 //时钟选择
  uint32_t dataCode;            			 //数据编码
  void (*Receive)(uint8_t *rData);     		 //接受数据
  void (*ChipSelect)(LTC2400CSType cs);		 //实现片选
  void (*Delayms)(volatile uint32_t nTime);  //实现延时
}Ltc2400ObjectType;

/* 数据转换函数 */
float GetLtc2400Data(Ltc2400ObjectType *ltc);

/* LTC2400初始化函数 */
void LTC2400Initialization(Ltc2400ObjectType *ltc, LTC2400ClockType clock);

#endif
