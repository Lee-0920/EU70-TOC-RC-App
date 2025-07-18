/**
 * @file
 * @brief 温度采集器。
 * @details 提供温度采集、上报功能接口。
 * @version 1.1.0
 * @author kim.xiejinqiang
 * @date 2012-10-30
 */

#include "Driver/System.h"
#include "Driver/McuFlash.h"
#include "Tracer/Trace.h"
#include <string.h>
#include "TempADCollect.h"
#include <TempDriver/TempADCollect.h>
#include "SystemConfig.h"
#include "PT1000.h"
#include "TempCollecter.h"
#include "KThermocouple.h"
#include "TempCollecterMap.h"
#include "math.h"
#include "ITS90.h"
#include "NTC100K.h"
#include "NTC1K.h"

static float TempCollecter_CalcPt1000Temperature(TempCollecter *tempCollecter, uint16_t ad);
static float TempCollecter_CalcITS90Temperature(TempCollecter *tempCollecter, uint16_t ad);
Uint8 s_debugAD = 0;

/**
 * @brief 温度采集器初始化
 * @param
 */
void TempCollecter_Init(TempCollecter *tempCollecter, TempCalibrateParam kTempCalculateParam)
{
    Uint8 buffer[TEMPERATURE_FACTORY_SIGN_FLASH_LEN] = { 0 };
    Uint32 flashFactorySign = 0;

    if(tempCollecter->number > 0)  //扩展的温度计
    {
        McuFlash_Read(EXTTEMPERATURE_FACTORY_SIGN_FLASH_BASE_ADDR + TEMPERATURE_FACTORY_SIGN_FLASH_LEN * (tempCollecter->number - 1),
        TEMPERATURE_FACTORY_SIGN_FLASH_LEN, buffer);               //读取出厂标志位
    }
    else
    {
        McuFlash_Read(TEMPERATURE_FACTORY_SIGN_FLASH_BASE_ADDR + TEMPERATURE_FACTORY_SIGN_FLASH_LEN * tempCollecter->number,
        TEMPERATURE_FACTORY_SIGN_FLASH_LEN, buffer);               //读取出厂标志位
    }
    memcpy(&flashFactorySign, buffer, TEMPERATURE_FACTORY_SIGN_FLASH_LEN);

    if (FLASH_FACTORY_SIGN == flashFactorySign)               //表示已经过出厂设置
    {
        tempCollecter->tempCalbrateFactor = TempCollecter_GetCalibrateFactor(tempCollecter);
    }
    else               //未设置,使用默认值，并写入出厂标志
    {
        TempCollecter_SetCalibrateFactor(tempCollecter, kTempCalculateParam);

        flashFactorySign = FLASH_FACTORY_SIGN;
        memcpy(buffer, &flashFactorySign, TEMPERATURE_FACTORY_SIGN_FLASH_LEN);
        if(tempCollecter->number > 0)  //扩展的温度计
        {
            McuFlash_Write(
            EXTTEMPERATURE_FACTORY_SIGN_FLASH_BASE_ADDR + TEMPERATURE_FACTORY_SIGN_FLASH_LEN * (tempCollecter->number-1),
            TEMPERATURE_FACTORY_SIGN_FLASH_LEN, buffer);
        }
        else
        {
            McuFlash_Write(
            TEMPERATURE_FACTORY_SIGN_FLASH_BASE_ADDR + TEMPERATURE_FACTORY_SIGN_FLASH_LEN * tempCollecter->number,
            TEMPERATURE_FACTORY_SIGN_FLASH_LEN, buffer);
        }
    }
}

void TempCollecter_SetNumber(TempCollecter *tempCollecter, Uint8 number)
{
    tempCollecter->number = number;
}
/**
 * @brief 读温度传感器的校准系数
 * @param
 */
TempCalibrateParam TempCollecter_GetCalibrateFactor(TempCollecter *tempCollecter)
{
    Uint8 readData[TEMPERATURE_CALIBRATE_FACTOR_LEN] = { 0 };
    TempCalibrateParam calibratefactor;

    if(tempCollecter->number > 0)  //扩展的温度计
    {
        McuFlash_Read(EXTTEMPERATURE_CALIBRATE_FACTOR_ADDRESS +TEMPERATURE_CALIBRATE_FACTOR_LEN * (tempCollecter->number-1),
                TEMPERATURE_CALIBRATE_FACTOR_LEN, readData);
    }
    else
    {
        McuFlash_Read(TEMPERATURE_CALIBRATE_FACTOR_ADDRESS +TEMPERATURE_CALIBRATE_FACTOR_LEN * tempCollecter->number,
                TEMPERATURE_CALIBRATE_FACTOR_LEN, readData);
    }
    memcpy(&calibratefactor, readData, sizeof(TempCalibrateParam));
    return calibratefactor;
}

/**
 * @brief 写温度传感器的校准系数
 * @param data
 */
void TempCollecter_SetCalibrateFactor(TempCollecter *tempCollecter, TempCalibrateParam calibratefactor)
{
    Uint8 writeData[TEMPERATURE_CALIBRATE_FACTOR_LEN] = { 0 };

    memcpy(writeData, &calibratefactor, sizeof(TempCalibrateParam));

    if(tempCollecter->number > 0)  //扩展的温度计
    {
        McuFlash_Write(EXTTEMPERATURE_CALIBRATE_FACTOR_ADDRESS + TEMPERATURE_CALIBRATE_FACTOR_LEN * (tempCollecter->number-1),
                TEMPERATURE_CALIBRATE_FACTOR_LEN, writeData);
    }
    else
    {
        McuFlash_Write(TEMPERATURE_CALIBRATE_FACTOR_ADDRESS + TEMPERATURE_CALIBRATE_FACTOR_LEN * tempCollecter->number,
                TEMPERATURE_CALIBRATE_FACTOR_LEN, writeData);
    }

    tempCollecter->tempCalbrateFactor = calibratefactor;

//    if(MEASUREMODULE1_TEMP == tempCollecter->number)
//	{
//    	TRACE_INFO("\n No %d Temp Set scale = ", tempCollecter->number);
//		System_PrintfFloat(1, calibratefactor.negativeInput, 8);
//		Printf("\n refAD =");
//		System_PrintfFloat(1, calibratefactor.vref, 8);
//		Printf("\n refTemp =");
//		System_PrintfFloat(1, calibratefactor.vcal, 8);
//	}
//	else
	{
		TRACE_INFO("\n No %d Temp Set negativeInput = ", tempCollecter->number);
		System_PrintfFloat(1, calibratefactor.negativeInput, 8);
		Printf("\n vref =");
		System_PrintfFloat(1, calibratefactor.vref, 8);
		Printf("\n vcal =");
		System_PrintfFloat(1, calibratefactor.vcal, 8);
	}

}

/**
 * @brief 温度计算
 * @param
 */
float TempCollecter_CalcPt1000Temperature(TempCollecter *tempCollecter, uint16_t ad)
{
    double rt;        //PT1000阻值
    float TempRet = -3000;
    Uint16 line, lineStart, lineStop;
    Int16 column;

    rt = tempCollecter->getResistanceValueFunc(&tempCollecter->tempCalbrateFactor, ad);
    TRACE_CODE("\n rt:%f , ad: %d\n", rt, ad);

    if (rt >= 1000)
    {
        //根据计算得到的rt值缩小查找范围
        if (rt >= g_PT1000TempTable[199][9]) //150℃以上进行查找 Rt>=1573.251
        {
            lineStart = 200;
            lineStop = 250;
        }
        else if (rt >= g_PT1000TempTable[149][9]) //100~150℃进行查找 Rt>=1385.055
        {
            lineStart = 150;
            lineStop = 199;
        }
        else if (rt >= g_PT1000TempTable[99][9]) //50~99.9℃进行查找 Rt>=1193.971
        {
            lineStart = 100;
            lineStop = 149;
        }
        else //0~49.9℃进行查找 Rt>=1000
        {
        	lineStart = 50;
			lineStop = 99;
        }

        for (line = lineStart; line <= lineStop; line++)
        {
            for (column = 0; column < 10; column++)
            {
                if (g_PT1000TempTable[line][column] < rt)
                {
                    continue;
                }
                else
                {
                    TempRet = (line - 50) + column / 10.0;
                    System_PrintfFloat(TRACE_LEVEL_CODE, TempRet, 1);

                    return TempRet;
                }
            }
        }
    }
    else if (rt < 1000 && rt > g_PT1000TempTable[49][0])
    {
        float tempTable[10] = {1000, 999.609, 999.218, 998.827, 998.437,
                        998.046,997.655, 997.264, 996.873, 996.482};
        for (int i = 9; i >= 0; i--)
        {
            if (tempTable[i] < rt)
            {
                continue;
            }
            else
            {
                TempRet = -1 * i / 10.0;
                System_PrintfFloat(TRACE_LEVEL_CODE, TempRet, 1);
                return TempRet;
            }
        }
    }
    else //负温度范围
    {
    	for (line = 0; line < 2; line++)
		{
			for (column = 11; column >= 0; column--)
			{
				if (g_PT1000TempTable[line][column] < rt)
				{
					continue;
				}
				else
				{
					TempRet = (line - 2) - column * 10.0;
					System_PrintfFloat(TRACE_LEVEL_CODE, TempRet, 1);
					return TempRet;
				}
			}
		}

		if(rt >= g_PT1000TempTable[0][0] && TempRet < -1000)
		{
			TempRet =  -(1 - (rt - g_PT1000TempTable[49][0])/(g_PT1000TempTable[50][0] - g_PT1000TempTable[49][0]));
			System_PrintfFloat(TRACE_LEVEL_CODE, TempRet, 1);
			return TempRet;
		}
    }
    return TempRet;
}

/**
 * @brief ITS90S型热电偶温度计算
 * @param
 */
float TempCollecter_CalcITS90Temperature(TempCollecter *tempCollecter, uint16_t ad)
{
    double rt;        //PT1000阻值
    float TempRet = -3000;
    Uint16 line, lineStart, lineStop;
    Int16 column;

    rt = tempCollecter->getResistanceValueFunc(&tempCollecter->tempCalbrateFactor, ad);
    if(s_debugAD & 1<< MEASUREMODULE1_TEMP)
    {
    	    TRACE_INFO("\n rt:%f , ad: %d\n", rt, ad);
    }

    if (rt >= 0 && rt < 18)
    {
        //根据计算得到的rt值缩小查找范围
        if (rt >= g_ITS90TempTable[150][9]) //1500℃以上进行查找 Rt > 15.7
        {
            lineStart = 150;
            lineStop = 176;
        }
        else if (rt >= g_ITS90TempTable[99][9]) //1000~1500℃进行查找 Rt 9.7~15.7
        {
            lineStart = 100;
            lineStop = 149;
        }
        else if (rt >= g_ITS90TempTable[49][9]) //500~1000℃进行查找 Rt 4.223~9.7
        {
            lineStart = 50;
            lineStop = 99;
        }
        else //0~500℃进行查找 Rt 0~4.223
        {
            lineStart = 0;
            lineStop = 49;
        }

        for (line = lineStart; line <= lineStop; line++)
        {
            for (column = 0; column < 10; column++)
            {
                if (g_ITS90TempTable[line][column] < rt)
                {
                    continue;
                }
                else
                {
                    TempRet = (line)*10 + column;

                    System_PrintfFloat(TRACE_LEVEL_CODE, TempRet, 1);

                    return TempRet;
                }
            }
        }
    }
    else if (rt < 1000 && rt > g_ITS90TempTable[176][0])
    {
    }
    else //负温度范围
    {
    }
    return TempRet;
}

/**
 * @brief 热电偶温度计算
 * 通过两组已知的温度及AD值可以计算出任意AD值所对应的温度
 * @param
 */
float TempCollecter_CalThermocoupleTemperature(uint16_t ad, TempCalibrateParam tempCalbrateFactor)
{
	float temperature = 0;
	int absad = 0;
//	absad = fabs((int)ad - (int)calParam.refad);
//	TRACE_CODE("\nabsad %d, reftemp %f, scale %f", absad, calParam.reftemp, calParam.scale);
//	if(ad > calParam.refad)
//	{
//		temperature = (calParam.reftemp + absad / calParam.scale);
//	}
//	else
//	{
//		temperature = (calParam.reftemp - absad / calParam.scale);
//	}
	absad = fabs((int)ad - (int)tempCalbrateFactor.vref);
	TRACE_CODE("\nabsad %d, reftemp %f, scale %f, refad: %f, ", absad, tempCalbrateFactor.vcal, tempCalbrateFactor.negativeInput, tempCalbrateFactor.vref);
	if(ad > tempCalbrateFactor.vref)
	{
		temperature = (tempCalbrateFactor.vcal + absad / tempCalbrateFactor.negativeInput);
	}
	else
	{
		temperature = (tempCalbrateFactor.vcal - absad / tempCalbrateFactor.negativeInput);
	}
	return temperature;
}

/*
**
 * @brief 温度计算
 * @param
 */
float TempCollecter_CalcNtc100KTemperature(TempCollecter *tempCollecter, uint16_t ad)
{
    double rt;        //NTC100K阻值
    float TempRet = -3000;
    float offsetTemperature = 41.0;
    Uint8 line;
    rt = tempCollecter->getResistanceValueFunc(&tempCollecter->tempCalbrateFactor, ad) / 1000.0;
    TRACE_CODE("\n rt:%f KOhm, ad: %d\n", rt, ad);
    if(rt > g_NTC100KTempTable[15]) //低于-25摄氏度判断为异常
    {
    	 return TempRet;
    }

    if (rt <= g_NTC100KTempTable[40] && rt >= g_NTC100KTempTable[100]) //0-60°
    {
    	for (line = 40; line <= 100; line++)
    	{
    		 if (rt > g_NTC100KTempTable[line])
    		 {
    			 TempRet = (float)(line - offsetTemperature + (g_NTC100KTempTable[line-1] - rt)/ (g_NTC100KTempTable[line-1] - g_NTC100KTempTable[line]));
    			 return TempRet;
    		 }
    	}
    }
    else if (rt <= g_NTC100KTempTable[100] && rt >= g_NTC100KTempTable[150]) //60-100°
    {
    	for (line = 100; line <= 150; line++)
    	{
    		 if (rt > g_NTC100KTempTable[line])
    		 {
    			 TempRet = (float)(line - offsetTemperature + (g_NTC100KTempTable[line-1] - rt)/ (g_NTC100KTempTable[line-1] - g_NTC100KTempTable[line]));
    			 return TempRet;
    		 }
    	}
    }
    else if (rt <= g_NTC100KTempTable[150] && rt >= g_NTC100KTempTable[200]) //100-150°
	{
		for (line = 150; line <= 200; line++)
		{
			 if (rt > g_NTC100KTempTable[line])
			 {
				 TempRet = (float)(line - offsetTemperature + (g_NTC100KTempTable[line-1] - rt)/ (g_NTC100KTempTable[line-1] - g_NTC100KTempTable[line]));
				 return TempRet;
			 }
		}
	}
    else //负温度范围
    {
    	for (line = 0; line < 40; line++)
		{
			 if (rt > g_NTC100KTempTable[line])
			 {
				 TempRet = (float)(line + (g_NTC100KTempTable[line-1] - rt)/ (g_NTC100KTempTable[line-1] - g_NTC100KTempTable[line])) - offsetTemperature ;
				 return TempRet;
			 }
		}
    }
    return TempRet;
}


/*
**
 * @brief 温度计算
 * @param
 */
float TempCollecter_CalcNtc1KTemperature(TempCollecter *tempCollecter, uint16_t ad)
{
    double rt;        //NTC1K阻值
    float TempRet = -3000;
    float offsetTemperature = 41.0;
    Uint8 line;
    rt = tempCollecter->getResistanceValueFunc(&tempCollecter->tempCalbrateFactor, ad) / 1000.0;
    TRACE_CODE("\n rt:%f KOhm, ad: %d\n", rt, ad);

    if (rt <= g_NTC1KTempTable[40] && rt >= g_NTC1KTempTable[100]) //0-50°
    {
    	for (line = 40; line <= 100; line++)
    	{
    		 if (rt > g_NTC1KTempTable[line])
    		 {
    			 TempRet = (float)(line - offsetTemperature + (g_NTC1KTempTable[line-1] - rt)/ (g_NTC1KTempTable[line-1] - g_NTC1KTempTable[line]));
    			 TRACE_CODE("\n ##%d", line);
    			 return TempRet;
    		 }
    	}
    }
    else if (rt <= g_NTC1KTempTable[100] && rt >= g_NTC1KTempTable[150]) //50-110°
    {
    	for (line = 100; line <= 150; line++)
    	{
    		 if (rt > g_NTC1KTempTable[line])
    		 {
    			 TempRet = (float)(line - offsetTemperature + (g_NTC1KTempTable[line-1] - rt)/ (g_NTC1KTempTable[line-1] - g_NTC1KTempTable[line]));
    			 return TempRet;
    		 }
    	}
    }
    else if (rt <= g_NTC1KTempTable[150] && rt >= g_NTC1KTempTable[200]) //110-160°
	{
		for (line = 150; line <= 200; line++)
		{
			 if (rt > g_NTC1KTempTable[line])
			 {
				 TempRet = (float)(line - offsetTemperature + (g_NTC1KTempTable[line-1] - rt)/ (g_NTC1KTempTable[line-1] - g_NTC1KTempTable[line]));
				 return TempRet;
			 }
		}
	}
    else //负温度范围
    {
    	for (line = 0; line < 40; line++)
		{
			 if (rt > g_NTC1KTempTable[line])
			 {
				 TempRet = (float)(line + (g_NTC1KTempTable[line-1] - rt)/ (g_NTC1KTempTable[line-1] - g_NTC1KTempTable[line])) - offsetTemperature ;
				 return TempRet;
			 }
		}
    }
    return TempRet;
}

/**
 * @brief 电炉温度计算
 * @param
 */
float TempCollecter_CalcHighTemperature(TempCollecter *tempCollecter, uint16_t ad)
{
	 float TempRet = -2000;

//	 TempRet = TempCollecter_CalThermocoupleTemperature(ad, tempCollecter->tempCalbrateFactor);
	 TempRet = TempCollecter_CalcITS90Temperature(tempCollecter, ad);

	return TempRet;
}

/**
 * @brief 查询温度
 * @param
 */
float TempCollecter_GetTemperature(TempCollecter *tempCollecter)
{
    float temperature = 0;
    uint16_t ad = 0;
//    TRACE_INFO("\ntemprature number: %d", tempCollecter->number);
    if(MEASUREMODULE1_TEMP == tempCollecter->number)
    {
    	ad = TempADCollect_GetAD(TEMP_INDEX_0);
    	temperature = TempCollecter_CalcNtc100KTemperature(tempCollecter, ad);
    	if(s_debugAD & 1<< MEASUREMODULE1_TEMP)
    	{
    	   	TRACE_INFO("\n--------TEMP_INDEX_0 number: %d, temp: %f,  ad: %d", tempCollecter->number, temperature, ad);
    	}
    }
    else if(MEASUREMODULE2_TEMP == tempCollecter->number)
    {
    	ad = TempADCollect_GetAD(TEMP_INDEX_1);
    	temperature = TempCollecter_CalcNtc100KTemperature(tempCollecter, ad);
    	if(s_debugAD & 1<< MEASUREMODULE2_TEMP)
    	{
			TRACE_INFO("\n--------TEMP_INDEX_1 number: %d, temp: %f, ad: %d", tempCollecter->number, temperature, ad);
    	}
    }
    else  if(MEASUREMODULE3_TEMP == tempCollecter->number)
    {
    	ad = TempADCollect_GetAD(TEMP_INDEX_2);
    	temperature = TempCollecter_CalcNtc100KTemperature(tempCollecter, ad);
    	if(s_debugAD & 1<< MEASUREMODULE3_TEMP)
    	{
    	   	TRACE_INFO("\n--------TEMP_INDEX_2 number: %d, temp: %f,  ad: %d", tempCollecter->number, temperature, ad);
    	}
    }
    else if(MEASUREMODULE4_TEMP == tempCollecter->number)
    {
    	ad = TempADCollect_GetAD(TEMP_INDEX_3);
    	temperature = TempCollecter_CalcNtc100KTemperature(tempCollecter, ad);
    	if(s_debugAD & 1<< MEASUREMODULE4_TEMP)
    	{
			TRACE_INFO("\n--------TEMP_INDEX_3 number: %d, temp: %f, ad: %d", tempCollecter->number, temperature, ad);
    	}
    }
	System_Delay(10);

    return temperature;
}

/**
 * @brief AD调试信息
 * @param
 */
void TempCollecter_SetDebugMode(Uint8 offset)
{
	s_debugAD = offset;
	Printf("\n Set Debug Mode %d", s_debugAD);
}

