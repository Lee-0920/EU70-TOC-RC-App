
/**
 * @file
 * @brief 温度采集器接口头文件
 * @details 提供温度采集、上报功能接口。
 * @version 1.1.0
 * @author kim.xiejinqiang
 * @date 2012-10-30
 */

#ifndef SRC_DRIVER_TEMPDRIVER_TEMPCOLLECTER_H_
#define SRC_DRIVER_TEMPDRIVER_TEMPCOLLECTER_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include "Common/Types.h"

// 温度计算参数
typedef struct
{										    //热电偶温度系数
    float negativeInput; //负输入分压 V			//两点校准-ad差值与温度差值之比
    float vref;          //参考电压 VREF V		//参考AD
    float vcal;          //校正电压			//参考温度
} TempCalibrateParam;

// 热电偶温度计算参数
typedef struct
{
    float scale;            //两点校准-ad差值与温度差值之比
    float refad; 			//参考AD
    float reftemp;          //参考温度
} TempCalculateParam;

typedef double (*GetResistanceValue)(TempCalibrateParam *tempCalibrateParam, Uint16 ad);

typedef struct
{
    Uint8 number;
    TempCalibrateParam tempCalbrateFactor;
    GetResistanceValue getResistanceValueFunc;
}TempCollecter;



void TempCollecter_Init(TempCollecter *tempCollecter, TempCalibrateParam kTempCalculateParam);
void TempCollecter_SetNumber(TempCollecter *tempCollecter, Uint8 number);
TempCalibrateParam TempCollecter_GetCalibrateFactor(TempCollecter *tempCollecter);
void TempCollecter_SetCalibrateFactor(TempCollecter *tempCollecter, TempCalibrateParam tempCalbrateFactor);
float TempCollecter_GetTemperature (TempCollecter *tempCollecter);
void TempCollecter_SetDebugMode(Uint8 offset);

#ifdef __cplusplus
}
#endif

#endif /* SRC_DRIVER_TEMPDRIVER_TEMPCOLLECTER_H_ */
