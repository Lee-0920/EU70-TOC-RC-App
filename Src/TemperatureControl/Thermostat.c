
/**
 * @file
 * @brief 恒温器接口
 * @details 提供温度控制功能接口。
 * @version 1.1.0
 * @author kim.xiejinqiang
 * @date 2012-10-30
 */

#include "FreeRTOS.h"
#include "timers.h"
#include "task.h"
#include "Driver/System.h"
#include "Driver/McuFlash.h"
#include "DncpStack/DncpStack.h"
#include "Tracer/Trace.h"
#include <string.h>
#include "SystemConfig.h"
#include "DNCP/App/DscpSysDefine.h"
#include "TempCollecterManager.h"
#include "ThermostatDeviceManager.h"
#include "Thermostat.h"
#include "ThermostatManager.h"
#include "DncpStack/DeviceStatus.h"
#include <math.h>

static void Thermostat_Handle(void *argument);
//static void ThermostatProtect_Handle(void *argument);
static void Thermostat_InitParam(Thermostat *thermostat, ThermostatParam kDefaultThermostatParam);

#define TEMP_WARNING_VALUE  1300 //温度监控的警戒值
#define TEMP_UNUSUAL_VALUE  -50 //温度传感器异常值
#define TEMP_OVER_VALUE  	50 //温控异常值

#define TEMP_REPORT_CNT 5
static float s_stoveReportArry[TEMP_REPORT_CNT] = {0};
static float s_stoveReportTemp = 0;
static uint8_t s_tempReportCnt = 0;

/**
 * @brief    恒温初始化
 * @details
 */
void Thermostat_Init(Thermostat *thermostat, ThermostatParam kDefaultThermostatParam)
{
    thermostat->mode = THERMOSTAT_MODE_AUTO;
    thermostat->targetTemp = 0;
    thermostat->toleranceTemp = 0;
    thermostat->timeout = 0;
    thermostat->alreadyTime = 0;
    thermostat->status = THERMOSTAT_IDLE;
    thermostat->reportResult.result = THERMOSTAT_RESULT_REACHED;
    thermostat->reportResult.temperature = 0;
    thermostat->isSendEvent = FALSE;
    thermostat->isThermostatReached = FALSE;
    thermostat->isGreaterThanObjTemp = FALSE;
    thermostat->isRequestStop = FALSE;

    Thermostat_InitParam(thermostat, kDefaultThermostatParam);

    xTaskCreate(Thermostat_Handle, "Thermostat_Handle", THERMOSTAT_STK_SIZE, (void*)thermostat,
            THERMOSTAT_TASK_PRIO, &thermostat->taskHandle);

    thermostat->tempIndex = TEMP_WITHOUT;
    thermostat->heaterIndex = NULL;
    thermostat->heaterTotal = 0;
    thermostat->refrigeratorIndex = NULL;
    thermostat->refrigeratorTotal = 0;
    thermostat->sumError = 0;
    thermostat->lastError = 0;
    thermostat->isTempWaring = FALSE;

    memset(thermostat->checkTemp, 0, sizeof(thermostat->checkTemp));
    thermostat->errTimes = 0;
    thermostat->checkIndex = 0;

    TRACE_DEBUG("\n No %d Thermostat Init Over ", thermostat->number);
}

void Thermostat_SetNumber(Thermostat *thermostat, Uint8 number)
{
    thermostat->number = number;
}

void Thermostat_SetTemp(Thermostat *thermostat, int index)
{
    thermostat->tempIndex = index;
}

void Thermostat_SetHeater(Thermostat *thermostat, Uint8 heaterTotal, Uint8* index)
{
    thermostat->heaterTotal = heaterTotal;
    thermostat->heaterIndex = pvPortMalloc(heaterTotal);
    memcpy(thermostat->heaterIndex, index, heaterTotal);
}

void Thermostat_SetRefrigerator(Thermostat *thermostat, Uint8 refrigeratorTotal, Uint8* index)
{
    thermostat->refrigeratorTotal = refrigeratorTotal;
    thermostat->refrigeratorIndex = pvPortMalloc(refrigeratorTotal);
    memcpy(thermostat->refrigeratorIndex, index, refrigeratorTotal);
}
/**
 * @brief 获取恒温参数
 * @param
 */
ThermostatParam Thermostat_GetPIDParam(Thermostat *thermostat)
{
    Uint8 readData[THERMOSTAT_PARAM_LEN] = { 0 };
    ThermostatParam thermostatparam;

    if(thermostat->number > 0)  //扩展的恒温器
    {
        McuFlash_Read(EXTTHERMOSTAT_PARAM_ADDRESS + THERMOSTAT_PARAM_LEN * (thermostat->number-1),
                THERMOSTAT_PARAM_LEN, readData);
    }
    else
    {
        McuFlash_Read(THERMOSTAT_PARAM_ADDRESS + THERMOSTAT_PARAM_LEN * thermostat->number,
                THERMOSTAT_PARAM_LEN, readData);
    }
    memcpy(&thermostatparam, readData, sizeof(ThermostatParam));
    return thermostatparam;
}

/**
 * @brief 设置恒温参数
 * @param
 */
void Thermostat_SetPIDParam(Thermostat *thermostat, ThermostatParam thermostatparam)
{
    Uint8 writeData[THERMOSTAT_PARAM_LEN] =
    { 0 };

    memcpy(writeData, &thermostatparam, sizeof(ThermostatParam));

    if(thermostat->number > 0)  //扩展的恒温器
    {
        McuFlash_Write(EXTTHERMOSTAT_PARAM_ADDRESS + THERMOSTAT_PARAM_LEN * (thermostat->number-1),
                THERMOSTAT_PARAM_LEN, writeData);
    }
    else
    {
        McuFlash_Write(THERMOSTAT_PARAM_ADDRESS + THERMOSTAT_PARAM_LEN * thermostat->number,
                THERMOSTAT_PARAM_LEN, writeData);
    }

    thermostat->theromstatparam = thermostatparam;
    TRACE_INFO("\n No %d Thermostat Set p = ", thermostat->number);
    System_PrintfFloat(TRACE_LEVEL_INFO, thermostatparam.proportion, 3);
    TRACE_INFO("\n i =");
    System_PrintfFloat(TRACE_LEVEL_INFO, thermostatparam.integration, 3);
    TRACE_INFO("\n d =");
    System_PrintfFloat(TRACE_LEVEL_INFO, thermostatparam.differential, 3);
}

/**
 * @brief 初始化参数
 * @param
 */
static void Thermostat_InitParam(Thermostat *thermostat, ThermostatParam kDefaultThermostatParam)
{
    Uint8 buffer[TEMPERATURE_FACTORY_SIGN_FLASH_LEN] =
    { 0 };
    Uint32 flashFactorySign = 0;

    if(thermostat->number > 0)  //扩展的恒温器
    {
        McuFlash_Read(EXTTHERMOSTAT_FACTORY_SIGN_FLASH_BASE_ADDR + THERMOSTAT_FACTORY_SIGN_FLASH_LEN * (thermostat->number-1),
        THERMOSTAT_FACTORY_SIGN_FLASH_LEN, buffer);                //读取出厂标志位
    }
    else
    {
        McuFlash_Read(THERMOSTAT_FACTORY_SIGN_FLASH_BASE_ADDR + THERMOSTAT_FACTORY_SIGN_FLASH_LEN * thermostat->number,
        THERMOSTAT_FACTORY_SIGN_FLASH_LEN, buffer);                //读取出厂标志位
    }
    memcpy(&flashFactorySign, buffer, THERMOSTAT_FACTORY_SIGN_FLASH_LEN);

    if (FLASH_FACTORY_SIGN == flashFactorySign)                      //表示已经过出厂设置
    {
        thermostat->theromstatparam = Thermostat_GetPIDParam(thermostat);
    }
    else                       //未设置,使用默认值，并写入出厂标志
    {
        Thermostat_SetPIDParam(thermostat, kDefaultThermostatParam);

        flashFactorySign = FLASH_FACTORY_SIGN;
        memcpy(buffer, &flashFactorySign, THERMOSTAT_FACTORY_SIGN_FLASH_LEN);
        if(thermostat->number > 0)  //扩展的恒温器
        {
            McuFlash_Write(EXTTHERMOSTAT_FACTORY_SIGN_FLASH_BASE_ADDR + THERMOSTAT_FACTORY_SIGN_FLASH_LEN * (thermostat->number-1),
            THERMOSTAT_FACTORY_SIGN_FLASH_LEN, buffer);
        }
        else
        {
            McuFlash_Write(THERMOSTAT_FACTORY_SIGN_FLASH_BASE_ADDR + THERMOSTAT_FACTORY_SIGN_FLASH_LEN * thermostat->number,
            THERMOSTAT_FACTORY_SIGN_FLASH_LEN, buffer);
        }
    }
}

/**
 * @brief 获取恒温状态
 * @param
 */
ThermostatStatus Thermostat_GetStatus(Thermostat *thermostat)
{
    return (thermostat->status);
}

float Thermostat_GetCurrentTemp(Thermostat *thermostat)
{
    float temp = 0;
    if (thermostat->tempIndex != TEMP_WITHOUT)
    {
    	temp = TempCollecterManager_GetTemp(thermostat->tempIndex);
    }
    return temp;
}

static Bool Thermostat_IsSupportHeater(Thermostat *thermostat)
{
    if (thermostat->heaterTotal > 0)
    {
        return TRUE;
    }
    else
    {
        return FALSE;
    }
}

static Bool Thermostat_IsSupportRefrigerator(Thermostat *thermostat)
{
    if (thermostat->refrigeratorTotal > 0)
    {
        return TRUE;
    }
    else
    {
        return FALSE;
    }
}

static void Thermostat_SetHeaterOutput(Thermostat *thermostat, float level)
{
    for (Uint8 i = 0; i < thermostat->heaterTotal; i++)
    {
        ThermostatDeviceManager_SetOutput(thermostat->heaterIndex[i], level);
    }
}

Bool Thermostat_SetSingleRefrigeratorOutput(Thermostat *thermostat, Uint8 fanIndex, float level)
{
    Bool ret = FALSE;
    if (fanIndex < thermostat->refrigeratorTotal)
    {
        ret = ThermostatDeviceManager_SetOutput(thermostat->refrigeratorIndex[fanIndex], level);
    }
    else
    {
        TRACE_ERROR("\n SetOutput  No. %d Refrigerator.", fanIndex);
    }
    return ret;
}

void Thermostat_SetRefrigeratorOutput(Thermostat *thermostat, float level)
{
    for (Uint8 i = 0; i < thermostat->refrigeratorTotal; i++)
    {
        ThermostatDeviceManager_SetOutput(thermostat->refrigeratorIndex[i], level);
    }
}

/**
 * @brief 开始恒温
 * @param
 */
int Thermostat_Start(Thermostat *thermostat, ThermostatMode mode, float targetTemp,
        float toleranceTemp, float timeout)
{
    if(THERMOSTAT_BUSY == Thermostat_GetStatus(thermostat))
    {
        TRACE_ERROR("\n Thermostat busy error");
        return DSCP_BUSY;
    }
    if( Thermostat_GetCurrentTemp(thermostat) <= TEMP_UNUSUAL_VALUE
            || Thermostat_GetCurrentTemp(thermostat) >= TEMP_WARNING_VALUE)
    {
        TRACE_ERROR("\n Thermostat temperature unusual error");
        return DSCP_ERROR;
    }
    if ((mode == THERMOSTAT_MODE_AUTO || mode == THERMOSTAT_MODE_HEATER)
            && FALSE == Thermostat_IsSupportHeater(thermostat))
    {
        TRACE_ERROR("\n Thermostat start parametric error, without heater");
        return DSCP_ERROR_PARAM;
    }
    if ((mode == THERMOSTAT_MODE_AUTO || mode == THERMOSTAT_MODE_REFRIGERATE)
            && FALSE == Thermostat_IsSupportRefrigerator(thermostat))
    {
        TRACE_ERROR("\n Thermostat start parametric error, without refrigerator");
        return DSCP_ERROR_PARAM;
    }
    if (targetTemp > TEMP_UNUSUAL_VALUE && targetTemp < TEMP_WARNING_VALUE && toleranceTemp >= 0 && timeout > 0)
    {
        thermostat->sumError = 0;
        thermostat->lastError = 0;

        thermostat->isThermostatReached = FALSE;
        thermostat->isRequestStop = FALSE;
        thermostat->mode = mode;
        thermostat->status = THERMOSTAT_BUSY;
        thermostat->targetTemp = targetTemp;
        thermostat->toleranceTemp = toleranceTemp;
        thermostat->timeout = timeout;
        thermostat->alreadyTime = 0;
        thermostat->isTempFault = FALSE;
        thermostat->tempFaultCnt = 0;
        thermostat->isNDIRReached = FALSE;
        thermostat->isHeatReached = FALSE;

        memset(thermostat->checkTemp, 0, sizeof(thermostat->checkTemp));
        thermostat->errTimes = 0;
        thermostat->checkIndex = 0;

        if(Thermostat_GetCurrentTemp(thermostat) >= thermostat->targetTemp)
        {
            thermostat->isGreaterThanObjTemp = TRUE;
        }
        else
        {
            thermostat->isGreaterThanObjTemp = FALSE;
        }

        DncpStack_ClearBufferedEvent();

        switch (thermostat->mode)
        {
        case THERMOSTAT_MODE_AUTO:
            Thermostat_SetHeaterOutput(thermostat, 0);
            Thermostat_SetRefrigeratorOutput(thermostat, 0);
            TRACE_INFO("\n mode: auto ,targetTemp:");
            break;

        case THERMOSTAT_MODE_HEATER:
            Thermostat_SetHeaterOutput(thermostat, 0);
            TRACE_INFO("\n mode: heater ,targetTemp:");
            break;

        case THERMOSTAT_MODE_REFRIGERATE:
            Thermostat_SetRefrigeratorOutput(thermostat, 0);
            TRACE_INFO("\n mode: refrigerate ,targetTemp:");
            break;
        }

//        HEATER2_POWER_ON();								//开启电源，防止电炉异常终止后无法供电

        vTaskResume(thermostat->taskHandle);

        System_PrintfFloat(TRACE_LEVEL_INFO, targetTemp, 2);
        TRACE_INFO("  ,toleranceTemp:");
        System_PrintfFloat(TRACE_LEVEL_INFO, toleranceTemp, 2);
        TRACE_INFO("  ,timeout:");
        System_PrintfFloat(TRACE_LEVEL_INFO, timeout,3);
        return DSCP_OK;
    }
    else
    {
        TRACE_ERROR("Thermostat start parametric error\n");
        return DSCP_ERROR_PARAM;
    }
}
/**
 * @brief 打开停止发送事件功能
 */
void Thermostat_SendEventOpen(Thermostat *thermostat)
{
    thermostat->isSendEvent = TRUE;
}

/**
 * @brief 关闭停止发送事件功能
 */
void Thermostat_SendEventClose(Thermostat *thermostat)
{
    thermostat->isSendEvent = FALSE;
}

static void Thermostat_SendEvent(Thermostat *thermostat)
{
    if(TRUE == thermostat->isSendEvent)
    {
        Uint8 data[6] = {0};
        data[0] = thermostat->reportResult.result;
        memcpy(&data[1], &thermostat->reportResult.temperature, sizeof(thermostat->reportResult.temperature));
        data[5] = thermostat->number;
        DncpStack_SendEvent(DSCP_EVENT_TCI_THERMOSTAT_RESULT, (void *)data, sizeof(data));
        DncpStack_BufferEvent(DSCP_EVENT_TCI_THERMOSTAT_RESULT, (void *)data, sizeof(data));
    }
}

Bool Thermostat_RequestStop(Thermostat *thermostat)
{
    if (THERMOSTAT_BUSY == thermostat->status)
    {
        thermostat->isRequestStop = TRUE;
        TRACE_INFO("\n thermostat is request stop.");
        return TRUE;
    }
    else
    {
        TRACE_ERROR("\n Failed to stop the thermostat, thermostat for idle.");
        return FALSE;
    }
}

static void Thermostat_Stop(Thermostat *thermostat)
{
    switch (thermostat->mode)
     {
         case THERMOSTAT_MODE_AUTO:
             Thermostat_SetHeaterOutput(thermostat, 0);
             Thermostat_SetRefrigeratorOutput(thermostat, 0);
             break;

         case THERMOSTAT_MODE_HEATER:
             Thermostat_SetHeaterOutput(thermostat, 0);
             break;

         case THERMOSTAT_MODE_REFRIGERATE:
             Thermostat_SetRefrigeratorOutput(thermostat, 0);
             break;
     }
    if (FALSE == thermostat->isTempFault)
    {
        thermostat->reportResult.result = THERMOSTAT_RESULT_STOPPED;
        Thermostat_SendEvent(thermostat);
    }
    else
    {
        TRACE_INFO("\n No stop event is sent");
        thermostat->isTempFault = FALSE;
    }
    TRACE_INFO("\n Stop Thermostat %d", thermostat->number);
    thermostat->status = THERMOSTAT_IDLE;
//    HEATER2_POWER_OFF();		//恒温停止则停止电炉总供电

    vTaskSuspend(thermostat->taskHandle);
}

/**
 * @brief 恒温计算  位置式PID
 * @param
 */
static float Thermostat_LocPIDCalc(Thermostat *thermostat, float temperature)
{
    float nowerror, derror;
    float pidout;

    nowerror = thermostat->targetTemp - temperature;  //当前控制误差

    if (nowerror > 15.0)
    {
        pidout = 1000;
    }
    else if (nowerror < -15.0)
    {
        pidout = -1000;
    }
    else
    {
        thermostat->sumError += nowerror;                       //计算当前误差的积分值
        derror = nowerror - thermostat->lastError;            //计算当前误差的微分值
        thermostat->lastError = nowerror;                       //保存误差值

        pidout = (thermostat->theromstatparam.proportion * nowerror        //比例项
        + thermostat->theromstatparam.integration * thermostat->sumError             //积分项
        + thermostat->theromstatparam.differential * derror);            //微分项

        if (pidout > 1000)
            pidout = 1000;
        else if (pidout < -1000)
            pidout = -1000;
        TRACE_DEBUG("\n pidout:");
        System_PrintfFloat(TRACE_LEVEL_DEBUG, pidout, 4);
    }
    return pidout;
}


/**
 * @brief 清空pid参数
 * @param
 */
static void Thermostat_ClearPID(Thermostat *thermostat)
{
	thermostat->sumError = 0;                       //清空当前误差的积分值
	thermostat->lastError = 0;                       //清空误差值
}

/**
 * @brief 恒温控制
 * @param
 */
static void Thermostat_ControlTemp(Thermostat *thermostat)
{
    float leve;
    thermostat->reportResult.temperature = Thermostat_GetCurrentTemp(thermostat);
    TRACE_DEBUG("\n targetTemp:");
    System_PrintfFloat(TRACE_LEVEL_DEBUG, thermostat->targetTemp, 1);
    TRACE_DEBUG("  CurrentTemp:");
    System_PrintfFloat(TRACE_LEVEL_DEBUG, thermostat->reportResult.temperature, 1);

    switch (thermostat->mode)
    {
    case THERMOSTAT_MODE_AUTO:
        leve = Thermostat_LocPIDCalc(thermostat, thermostat->reportResult.temperature)
                / 1000;
        if (leve < 0)
        {
            Thermostat_SetHeaterOutput(thermostat, 0);
            Thermostat_SetRefrigeratorOutput(thermostat, -leve);
        }
        else
        {
            Thermostat_SetHeaterOutput(thermostat, leve);
            Thermostat_SetRefrigeratorOutput(thermostat, 0);
        }
        break;

    case THERMOSTAT_MODE_HEATER:
//    	if(thermostat->number >= THERMOSTAT_NDIR_LONG)  //NDIR未到达温度前不进行PID值计算
    	{
    		if(thermostat->reportResult.temperature < 0)
    		{
    			return ;
    		}

    		if(thermostat->isHeatReached == FALSE) //加热过程
			{
				leve = 1; //100%
			}
			else
			{
				leve = 0;
			}

			if(thermostat->isNDIRReached == FALSE && thermostat->reportResult.temperature > thermostat->targetTemp + 0.5) //降温
			{
				leve = 0; //100%
				thermostat->isHeatReached = TRUE;
				TRACE_DEBUG("NDIR heat reached %f, leve %f",thermostat->reportResult.temperature, leve);
			}

			if(thermostat->isNDIRReached == FALSE &&
					thermostat->isHeatReached == TRUE && thermostat->reportResult.temperature < thermostat->targetTemp + 0.5)
			{
				thermostat->isNDIRReached = TRUE;
				thermostat->alreadyTime = 0;
				thermostat->time  = 0;
				thermostat->lastTemp = thermostat->reportResult.temperature;
				thermostat->NDIRlevel =  0.05*1.66; //10%
				leve = thermostat->NDIRlevel;
				TRACE_DEBUG("NDIRReached %f, leve %f",thermostat->reportResult.temperature, thermostat->NDIRlevel);
			}

			if(thermostat->isNDIRReached == TRUE
					&& (thermostat->reportResult.temperature - thermostat->targetTemp) < -1
					&& thermostat->alreadyTime > 15)//升温过程温控
			{
				if(thermostat->reportResult.temperature - thermostat->lastTemp >= 0.2) //当前温度比10分钟高0.2
				{

				}
				else //温度变化较小或无变化，增加功率
				{
					thermostat->NDIRlevel += 0.005*1.66;
				}
				TRACE_INFO("\n *NDIR* lastTemp %f, curTemp %f, leve %f",thermostat->lastTemp, thermostat->reportResult.temperature, thermostat->NDIRlevel);
				thermostat->lastTemp = thermostat->reportResult.temperature;
				leve = thermostat->NDIRlevel;
				thermostat->time  = 0;
				thermostat->alreadyTime = 0;
				TRACE_MARK("\n *NDIR* lastTemp %f, curTemp %f, leve %f",thermostat->lastTemp, thermostat->reportResult.temperature, thermostat->NDIRlevel);
//    			thermostat->lastTemp = thermostat->reportResult.temperature;
			}
			else if(thermostat->isNDIRReached == TRUE
					&& (thermostat->reportResult.temperature - thermostat->targetTemp) > 1
					&& thermostat->alreadyTime > 40) //降温阶段温控
			{
				if(thermostat->reportResult.temperature - thermostat->lastTemp <= -0.2) //当前温度比10分钟高0.2
				{

				}
				else //温度变化较小或无变化，降低功率
				{
					thermostat->NDIRlevel -= 0.005*1.66;
				}
				TRACE_INFO("\n *NDIR* lastTemp %f, curTemp %f, leve %f",thermostat->lastTemp, thermostat->reportResult.temperature, thermostat->NDIRlevel);
				thermostat->lastTemp = thermostat->reportResult.temperature;
				leve = thermostat->NDIRlevel;
				thermostat->time  = 0;
				thermostat->alreadyTime = 0;
			}
			else if(thermostat->isNDIRReached == TRUE
					&& (fabs)(thermostat->reportResult.temperature - thermostat->targetTemp) < 1) //恒温阶段温控
			{
				if(thermostat->reportResult.temperature - thermostat->lastTemp <= -0.2) //当前温度比10分钟低0.2
				{
					thermostat->NDIRlevel += 0.005*1.66;
					TRACE_INFO("\n *NDIR* lastTemp %f, curTemp %f, leve %f",thermostat->lastTemp, thermostat->reportResult.temperature, thermostat->NDIRlevel);
					thermostat->lastTemp = thermostat->reportResult.temperature;
					leve = thermostat->NDIRlevel;
				}
				else if(thermostat->reportResult.temperature - thermostat->lastTemp >= 0.2) //当前温度比10分钟高0.2
				{
					thermostat->NDIRlevel -= 0.005*1.66;
					TRACE_INFO("\n *NDIR* lastTemp %f, curTemp %f, leve %f",thermostat->lastTemp, thermostat->reportResult.temperature, thermostat->NDIRlevel);
					thermostat->lastTemp = thermostat->reportResult.temperature;
					leve = thermostat->NDIRlevel;
				}
				else
				{
//    				thermostat->lastTemp = thermostat->reportResult.temperature;
					leve = thermostat->NDIRlevel;
				}
			}
			else if(thermostat->isNDIRReached == TRUE)
			{
				leve = thermostat->NDIRlevel;
				thermostat->alreadyTime += 0.5;
				thermostat->time += 0.5;
			}
			TRACE_DEBUG("\n NDIR temperature %f, NDIRlevel %f, leve %f, time %f, sectime %f",thermostat->reportResult.temperature
					, thermostat->NDIRlevel, leve, thermostat->alreadyTime, thermostat->time);
    	}
//    	else
//    	{
//    		leve = Thermostat_LocPIDCalc(thermostat, thermostat->reportResult.temperature)
//    						/ 1000;
//    	}
		if (leve >= 0)
		{
			Thermostat_SetHeaterOutput(thermostat, leve);
		}
		else
		{
			Thermostat_SetHeaterOutput(thermostat, 0);
		}

        break;

    case THERMOSTAT_MODE_REFRIGERATE:
    	leve = Thermostat_LocPIDCalc(thermostat, thermostat->reportResult.temperature)
    	                / 1000;
		if (leve <= 0)
		{
			Thermostat_SetRefrigeratorOutput(thermostat, -leve);
		}
		else
		{
			Thermostat_SetRefrigeratorOutput(thermostat, 0);
		}
//		TRACE_INFO("\n REFRIGERATE targetTemp:%f, CurrentTemp:%f",thermostat->targetTemp, thermostat->reportResult.temperature);
        break;
    default:
        break;
    }
}

/**
 * @brief 计算偏差
 */
static float CalculateMaxDeviation(float* data, int len)
{
    float min = data[0];
    float max = data[0];
    float deviation = 0;

    for(int i= 0; i < len; i++)
    {
        if(data[i] < min)
        {
            min = data[i];
        }
        if(data[i] > max)
        {
            max = data[i];
        }
    }
    deviation = max - min;

    TRACE_MARK("\nArray: ");
    for(int i = 0; i < len; i++)
    {
    	TRACE_MARK("%f, ", data[i]);
    }
    TRACE_MARK("\nArray: Min = %f, Max = %f, Deviation = %f", min, max, deviation);

    return deviation;
}

/**
 * @brief 计算标准差
 */
static float CalculateStdDeviation(float* data, int len)
{
    float sum,mean,temp,std = 0;

    for(int i = 0; i < len; i++)
    {
        sum += data[i];
    }
    mean = sum/len;

    for(int i = 0; i < len; i++)
    {
        temp += pow((data[i] - mean), 2);
    }
    std = sqrt(temp/len);

    TRACE_MARK("\nArray: ");
    for(int i = 0; i < len; i++)
    {
    	TRACE_MARK("%f, ", data[i]);
    }
    TRACE_MARK("\nArray: Mean = %f, Std = %f", mean, std);

    return std;
}

/**
 * @brief 恒温异常检查
 */
static Bool Thermostat_TempCheck(Thermostat * thermostat)
{
    float deviation = 0;
    float curTemp = Thermostat_GetCurrentTemp(thermostat);

    if(thermostat->mode == THERMOSTAT_MODE_AUTO || thermostat->mode == THERMOSTAT_MODE_HEATER)
    {
        if((thermostat->targetTemp - curTemp) > 15)
        {
            thermostat->checkTemp[thermostat->checkIndex++] = curTemp;

            if(thermostat->checkIndex >= 40)
            {
                thermostat->checkIndex= 0;
                deviation = CalculateMaxDeviation(thermostat->checkTemp, 40);
                if(deviation < 0.5)
                {
                    thermostat->errTimes++;
                    TRACE_WARN("\nThermost check temp exception. max deviation = %f, errTimes = %d", deviation, thermostat->errTimes);
                }
                else
                {
                    TRACE_DEBUG("\nThermost check temp array: max deviation = %f", deviation);
                }
            }
        }
        else if(((curTemp - thermostat->targetTemp) > TEMP_OVER_VALUE) && (thermostat->mode == THERMOSTAT_MODE_HEATER))
        {
        	 thermostat->isRequestStop = FALSE;
			 Thermostat_Stop(thermostat);
        	 TRACE_ERROR("\nThermost forced stop!");
			 return FALSE;
        }
    }

    if(thermostat->errTimes >= 15)
    {
    	thermostat->errTimes = 0;
		TRACE_ERROR("\nThermost NDIR check temp error!");
		return FALSE;
    }
    else
    {
        return TRUE;
    }
}

/**
 // * @brief 恒温控制任务处理
 // * @param
 // */
static void Thermostat_Handle(void *argument)
{
    Thermostat *thermostat;
    thermostat = (Thermostat *)argument;
    vTaskSuspend(thermostat->taskHandle);
    while(1)
    {
        vTaskDelay(400 / portTICK_RATE_MS);
//        TRACE_ERROR("\n -----1------");
        float currentThermostatTemp = Thermostat_GetCurrentTemp(thermostat);
        if(thermostat->mode == THERMOSTAT_MODE_REFRIGERATE)
        {
        	currentThermostatTemp = ThermostatManager_GetCurrentTemp(1);
        }
        thermostat->reportResult.temperature = currentThermostatTemp;

        if(currentThermostatTemp <= TEMP_UNUSUAL_VALUE
                    || currentThermostatTemp >= TEMP_WARNING_VALUE)// 温度异常判断
        {
            thermostat->tempFaultCnt++;
            if (thermostat->tempFaultCnt >= 10)
            {
                thermostat->tempFaultCnt = 0;
                thermostat->reportResult.result = THERMOSTAT_RESULT_FAILED;   // 温度传感器温度异常。
                TRACE_ERROR("\n THERMOSTAT_RESULT_FAILED");
                Thermostat_SendEvent(thermostat);
                thermostat->isRequestStop = TRUE;
                thermostat->isTempFault = TRUE;
            }
        }
        else if(FALSE == Thermostat_TempCheck(thermostat))  //温度检查异常
        {
            thermostat->reportResult.result = THERMOSTAT_RESULT_FAILED;   // 温度传感器温度异常。
            TRACE_ERROR("\n THERMOSTAT_RESULT_FAILED");
            Thermostat_SendEvent(thermostat);
            thermostat->isRequestStop = TRUE;
            thermostat->isTempFault = TRUE;
        }
        else    // 温度正常
        {
            thermostat->tempFaultCnt = 0;
            if (FALSE == thermostat->isThermostatReached)
            {
                thermostat->alreadyTime += 0.5;
                if ((TRUE == thermostat->isGreaterThanObjTemp &&
                        thermostat->reportResult.temperature <= (thermostat->targetTemp + thermostat->toleranceTemp))
                        || (FALSE == thermostat->isGreaterThanObjTemp &&
                                thermostat->reportResult.temperature >= (thermostat->targetTemp - thermostat->toleranceTemp)))
                {
                    thermostat->isThermostatReached = TRUE;
                    thermostat->reportResult.result = THERMOSTAT_RESULT_REACHED;
                    TRACE_ERROR("\n THERMOSTAT_RESULT_REACHED");
                    Thermostat_SendEvent(thermostat);
//                    if(thermostat->number == THERMOSTAT_STOVE)
//                    {
//                    	Thermostat_ClearPID(thermostat);
//                    }
                }
                else if (thermostat->alreadyTime >= thermostat->timeout)
                {
                    thermostat->isThermostatReached = TRUE;
                    thermostat->reportResult.result = THERMOSTAT_RESULT_TIMEOUT;
                    TRACE_ERROR("\n THERMOSTAT_RESULT_TIMEOUT");
                    Thermostat_SendEvent(thermostat);
                }
            }

            Thermostat_ControlTemp(thermostat);   // 恒温控制执行
            if (FALSE == thermostat->isThermostatReached)
            {
                // 如果当前温度小于目标温度 ，并且模式是制冷，直接返回reached
                if(THERMOSTAT_MODE_REFRIGERATE == thermostat->mode)
                {
                    if((thermostat->targetTemp - currentThermostatTemp) >= 0.000001)
                    {
                        thermostat->isThermostatReached = TRUE;
                        thermostat->reportResult.result = THERMOSTAT_RESULT_REACHED;
                        TRACE_ERROR("\n THERMOSTAT_RESULT_REACHED");
                        Thermostat_SendEvent(thermostat);
                    }
                }
                // 如果当前温度大于目标温度，并且模式是加热， 直接返回reached
                if(THERMOSTAT_MODE_HEATER == thermostat->mode)
                {
                    if((currentThermostatTemp - thermostat->targetTemp) >= 0.000001)
                    {
                        thermostat->isThermostatReached = TRUE;
                        thermostat->reportResult.result = THERMOSTAT_RESULT_REACHED;
                        TRACE_ERROR("\n THERMOSTAT_RESULT_REACHED");
                        Thermostat_SendEvent(thermostat);
                    }
                }
            }
        }

        if(TRUE == thermostat->isRequestStop)
        {
            thermostat->isRequestStop = FALSE;
            Thermostat_Stop(thermostat);
        }
    }
}

ThermostatParam Thermostat_GetCurrentPIDParam(Thermostat *thermostat)
{
    return thermostat->theromstatparam;
}

void Thermostat_SetCurrentPIDParam(Thermostat *thermostat, ThermostatParam thermostatparam)
{
    thermostat->theromstatparam = thermostatparam;

    TRACE_INFO("\n SetCurrentPIDParam p =");
    System_PrintfFloat(TRACE_LEVEL_INFO, thermostatparam.proportion, 3);
    TRACE_INFO("\n i =");
    System_PrintfFloat(TRACE_LEVEL_INFO, thermostatparam.integration, 3);
    TRACE_INFO("\n d =");
    System_PrintfFloat(TRACE_LEVEL_INFO, thermostatparam.differential, 3);
}

void Thermostat_TempMonitor(Thermostat *thermostat)
{
//	TRACE_INFO("\n -----2------");
    if(Thermostat_GetCurrentTemp(thermostat) >= TEMP_WARNING_VALUE)
    {
        thermostat->isTempWaring = TRUE;
        if (THERMOSTAT_BUSY == Thermostat_GetStatus(thermostat))
        {
            Thermostat_RequestStop(thermostat);
            while (THERMOSTAT_IDLE != Thermostat_GetStatus(thermostat))
            {
                System_Delay(5);
            }
//            HEATER2_POWER_OFF();
            TRACE_ERROR("\n Error temperature: %f", Thermostat_GetCurrentTemp(thermostat));
            TRACE_ERROR("\nThermost has forced power off! ");
        }
        TRACE_ERROR("\n No %d Thermostat Heater close", thermostat->number);
        for (Uint8 i = 0; i < thermostat->heaterTotal; i++)
        {
            if(TRUE == ThermostatDeviceManager_IsOpen(thermostat->heaterIndex[i]))
            {
                ThermostatDeviceManager_SetOutput(thermostat->heaterIndex[i], 0);
            }
        }
        TRACE_ERROR("\n No %d Thermostat Fan open", thermostat->number);
        for (Uint8 i = 0; i < thermostat->refrigeratorTotal; i++)
        {
            if(FALSE == ThermostatDeviceManager_IsOpen(thermostat->refrigeratorIndex[i]))
            {
                ThermostatDeviceManager_SetOutput(thermostat->refrigeratorIndex[i], 1);
            }
        }
    }
    if(Thermostat_GetCurrentTemp(thermostat) < TEMP_WARNING_VALUE && TRUE == thermostat->isTempWaring)
    {
        thermostat->isTempWaring = FALSE;
//        TRACE_ERROR("\n No %d Thermostat Fan close", thermostat->number);
        for (Uint8 i = 0; i < thermostat->refrigeratorTotal; i++)
        {
            ThermostatDeviceManager_SetOutput(thermostat->refrigeratorIndex[i], 0);
        }
    }

    if(Thermostat_GetCurrentTemp(thermostat) < TEMP_UNUSUAL_VALUE)
    {
        if (THERMOSTAT_BUSY == Thermostat_GetStatus(thermostat))
        {
//            Thermostat_RequestStop(thermostat);
            while (THERMOSTAT_IDLE != Thermostat_GetStatus(thermostat))
            {
                System_Delay(5);
            }
        }
    }
    if((Thermostat_GetCurrentTemp(thermostat) - thermostat->targetTemp >= 50) && THERMOSTAT_MODE_HEATER ==thermostat->mode)
    {
		ThermostatDeviceManager_SetOutput(thermostat->heaterIndex[0], 1);
		thermostat->isRequestStop = FALSE;
		Thermostat_Stop(thermostat);
		TRACE_ERROR("\n Thermostat temperature warnning");
    }

//    if(thermostat->number == THERMOSTAT_STOVE)
//    {
//
//    	if(TEMP_REPORT_CNT > s_tempReportCnt)	//装填数组
//    	{
//    		s_stoveReportArry[s_tempReportCnt] = Thermostat_GetCurrentTemp(thermostat);
//    		s_stoveReportTemp = s_stoveReportArry[s_tempReportCnt];
//    	}
//    	else
//    	{
//    		//进入临界区 防止温度未计算完成时被抢占
//    		taskENTER_CRITICAL();
//    		s_stoveReportTemp = 0;
//    		for(uint8_t i = 0;i<TEMP_REPORT_CNT - 1;i++) //数据连续左移，保证每次取连续的五个点的平均值作为温度显示，左移前4位，再末尾追加
//			{
//    			s_stoveReportArry[i] = s_stoveReportArry[i + 1];
//    			s_stoveReportTemp += s_stoveReportArry[i];
//    			TRACE_DEBUG("\n Temp:%f", s_stoveReportArry[i]);
//			}
//    		s_stoveReportArry[TEMP_REPORT_CNT-1] = Thermostat_GetCurrentTemp(thermostat);
//    		TRACE_DEBUG("\n Temp:%f", s_stoveReportArry[TEMP_REPORT_CNT-1]);
//    		s_stoveReportTemp = (s_stoveReportTemp + s_stoveReportArry[TEMP_REPORT_CNT-1])/TEMP_REPORT_CNT;
//    		s_tempReportCnt = TEMP_REPORT_CNT;
//    		TRACE_DEBUG("\n Report Temp:%f", s_stoveReportTemp);
//    		//退出临界区
//    		taskEXIT_CRITICAL();
//    	}
//    	s_tempReportCnt++;
//    }

}

float Thermostat_GetStoveAveragerTemp(void)
{
	return s_stoveReportTemp;
}

void Thermostat_RelayControl(Bool status)
{
	if(status)
	{
//		HEATER2_POWER_ON();
		TRACE_INFO("\n relay on");
	}
	else
	{
//		HEATER2_POWER_OFF();
		TRACE_INFO("\n relay off");
	}

}

