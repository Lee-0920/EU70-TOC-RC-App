/*
 * TempCollecterMap.c
 *
 *  Created on: 2019年8月16日
 *      Author: Administrator
 */

#include "TempCollecterMap.h"
#include "Tracer/Trace.h"
#include "Driver/System.h"
#include <string.h>

#define TEMPADCOLLECT_AD_MAX  (4095)    // 24位AD
#define TEMPADCOLLECT_V_REF  (2.481)       // 单位(V)

#define K_TEMPFACTOR 		 (175)       // k型热电偶AD值放大倍数

//上机箱温度系统初始化参数
const static TempCalibrateParam s_kFanUpTempCalculateParam =
{ .negativeInput = 1.48, .vref = 2.48, .vcal = 0, };
//下机箱温度系统初始化参数
const static TempCalibrateParam s_kFanDownTempCalculateParam =
{ .negativeInput = 1.48, .vref = 2.48, .vcal = 0, };

static double TempCollecterMap_GetResistanceValue(TempCalibrateParam *tempCalibrateParam, Uint16 ad);
static double TempCollecterMap_GetExtraResistanceValue(TempCalibrateParam *tempCalibrateParam, Uint16 ad);
static double TempCollecterMap_GetHighTempResistanceValue(TempCalibrateParam *tempCalibrateParam, Uint16 ad);

void TempCollecterMap_Init(TempCollecter *tempCollecter)
{
    tempCollecter[MEASUREMODULE1_TEMP].getResistanceValueFunc = TempCollecterMap_GetResistanceValue;
    TempCollecter_SetNumber(&tempCollecter[MEASUREMODULE1_TEMP], MEASUREMODULE1_TEMP);
    TempCollecter_Init(&tempCollecter[MEASUREMODULE1_TEMP], s_kFanUpTempCalculateParam);

    tempCollecter[MEASUREMODULE2_TEMP].getResistanceValueFunc = TempCollecterMap_GetResistanceValue;
    TempCollecter_SetNumber(&tempCollecter[MEASUREMODULE2_TEMP], MEASUREMODULE2_TEMP);
    TempCollecter_Init(&tempCollecter[MEASUREMODULE2_TEMP], s_kFanDownTempCalculateParam);

    tempCollecter[MEASUREMODULE3_TEMP].getResistanceValueFunc = TempCollecterMap_GetResistanceValue;
	TempCollecter_SetNumber(&tempCollecter[MEASUREMODULE3_TEMP], MEASUREMODULE3_TEMP);
	TempCollecter_Init(&tempCollecter[MEASUREMODULE3_TEMP], s_kFanUpTempCalculateParam);

	tempCollecter[MEASUREMODULE4_TEMP].getResistanceValueFunc = TempCollecterMap_GetResistanceValue;
	TempCollecter_SetNumber(&tempCollecter[MEASUREMODULE4_TEMP], MEASUREMODULE4_TEMP);
	TempCollecter_Init(&tempCollecter[MEASUREMODULE4_TEMP], s_kFanDownTempCalculateParam);
}

static double TempCollecterMap_GetResistanceValue(TempCalibrateParam *tempCalibrateParam, Uint16 ad)
{
    double realV = ad * TEMPADCOLLECT_V_REF / TEMPADCOLLECT_AD_MAX; //根据AD值计算得到电压值
    double rt;
//    TRACE_INFO("\n realV :%f ,ad: %d \n",realV,ad);
    if (realV >= 2.5)//超出PT1000计算范围
    {
        return -3000;
    }
    rt = (tempCalibrateParam->vref - realV / 3) * 10000 / (realV / 3);

//    TRACE_CODE("\n way 1 ad: %d ; realV: %f; Rt: %f,; Temp:", ad, realV, rt);
//    TRACE_INFO("\n way 1 ad: %d ; realV: %f; Rt: %f,:", ad, realV, rt);
    return rt;
}

static double TempCollecterMap_GetExtraResistanceValue(TempCalibrateParam *tempCalibrateParam, Uint16 ad)
{
    double realV = ad * TEMPADCOLLECT_V_REF / TEMPADCOLLECT_AD_MAX; //根据AD值计算得到电压值
    double rt;
//    TRACE_INFO("\n realV :%f ,ad: %d \n",realV,ad);
    if (realV >= 2.5)//超出PT1000计算范围
    {
        return 2000;
    }
    rt = ((tempCalibrateParam->vref - tempCalibrateParam->negativeInput)
            / (tempCalibrateParam->negativeInput - realV / 6)) * 1000;
//    rt = ((2.495 - 1.495)
//            / (1.495 - realV / 4)) * 1000;

//    TRACE_CODE("\n way 1 ad: %d ; realV: %f; Rt: %f,; Temp:", ad, realV, rt);
//    TRACE_INFO("\n way 1 ad: %d ; realV: %f; Rt: %f,; Temp:", ad, realV, rt);
    return rt;
}

static double TempCollecterMap_GetHighTempResistanceValue(TempCalibrateParam *tempCalibrateParam, Uint16 ad)
{
    double realV = ad * tempCalibrateParam->vref / TEMPADCOLLECT_AD_MAX; //根据AD值计算得到电压值
    double rt;
    if (realV >= 3.3)//超出PT1000计算范围
    {
        return 2000;
    }
	rt = realV / (K_TEMPFACTOR) * 1000;

//	  TRACE_INFO("\n high temp rt :%f ,ad: %d \n",rt,ad);

//    TRACE_CODE("\n way 1 ad: %d ; realV: %f; Rt: %f,; Temp:", ad, realV, rt);
//    TRACE_INFO("\n way 1 ad: %d ; realV: %f; Rt: %f,; Temp:", ad, realV, rt);
    return rt;
}

char* TempCollecterMap_GetName(Uint8 index)
{
    static char name[20] = "";
    memset(name, 0, sizeof(name));
    switch(index)
    {
    case MEASUREMODULE1_TEMP:
        strcpy(name, "MeasureModule1-Stove temperature");
        break;
    case MEASUREMODULE2_TEMP:
        strcpy(name, "MeasureModule2-Cooler temperature");
        break;
    default:
        strcpy(name, "NULL");
        break;
    }
    return name;
}

