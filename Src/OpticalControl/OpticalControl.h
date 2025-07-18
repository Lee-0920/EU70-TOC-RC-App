/**
 * @file
 * @brief 光学采集控制接口实现头文件
 * @details
 * @version 1.0.0
 * @author lemon.xiaoxun
 * @date 2016-5-27
 */

#ifndef SRC_DNCPSTACK_OPTICALCONTROL_H_
#define SRC_DNCPSTACK_OPTICALCONTROL_H_


#ifdef __cplusplus
extern "C" {
#endif

#include "stm32f4xx.h"
#include "Common/Types.h"

typedef enum
{
    OPTICALCONTROL_IDLE = 0,
    OPTICALCONTROL_BUSY = 1,
    OPTICALCONTROL_COLLECT = 2,
}OpticalControlStatus;

typedef enum
{
    ONLY_AD7791 = 0,
    ONLY_ADS1146 = 1,
    BOTH_AD7791_ADS1146 = 2,
}OpticalWorkMode;

typedef struct
{
    Uint32 reference;        // 参考端AD。
    Uint32 measure;          // 测量端AD。
}OpticalSignalAD;

void OpticalControl_Init(void);
void OpticalControl_ADCChangeInit(Uint8 type);
void OpticalControl_ResetWorkMode(OpticalWorkMode mode);
void OpticalControl_WorkModeRestore(void);
OpticalWorkMode OpticalControl_GetWorkMode(void);
void OpticalControl_Restore(void);
void OpticalControl_TurnOnLed(Bool status);
Bool OpticalControl_StartAcquirer(Uint8 index, float adacquiretime);
Bool OpticalControl_StopAcquirer();
void OpticalControl_SendEventOpen(void);
void OpticalControl_SendEventClose(void);
void OpticalControl_SetSignalADNotifyPeriod(float period);
void OpticalControl_PrintfInfo(void);
Uint16 OpticalControl_FilterData(Uint16 *inputData, Uint16 count, Uint16 filterHigh, Uint16 filterLow);
//void OpticalControl_TurnOnXonen(Bool status);
//Bool OpticalControl_GetAD7791Data(OpticalSignalAD *sampleData);
Bool OpticalControl_GetADS1146Data(OpticalSignalAD *sampleData);
OpticalControlStatus OpticalControl_GetCurrentStatus(void);
Bool OpticalControl_EnterCollectStatus(void);
Bool OpticalControl_EnterIdleStatus(void);
//void OpticalControl_GlitterXonen(Bool status);
void OpticalControl_CollectTestFun(Bool isOpenTestMode, Uint8 channel);
void OpticalControl_AD5175Write(Uint8 index, Uint16 value);
Uint16 OpticalControl_AD5175Read(Uint8 index);
void OpticalControl_AD5175ReadWithAddr(Uint8 index, Uint8 addr);

//*****下位机测试使用*****

#ifdef __cplusplus
}
#endif

#endif /* SRC_DNCPSTACK_OPTICALCONTROL_H_ */
