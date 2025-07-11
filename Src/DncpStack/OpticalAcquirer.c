/**
 #include <OpticalAcquirer.h>
 * @file
 * @brief 光学采集接口实现
 * @details
 * @version 1.0.0
 * @author lemon.xiaoxun
 * @date 2016-5-27
 */

#include <string.h>
#include "Common/Utils.h"
#include "DNCP/App/DscpSysDefine.h"
#include "Tracer/Trace.h"
#include "Driver/System.h"
#include "DncpStack/DncpStack.h"
#include "LuipApi/OpticalAcquireInterface.h"
#include "OpticalControl/OpticalControl.h"
#include "OpticalAcquirer.h"
#include <math.h>
#include "Driver/UltraSignalDriver/ChannelDriver.h"


void OpticalAcquirer_TurnOnLed(DscpDevice* dscp, Byte* data, Uint16 len)
{
    unsigned short ret = DSCP_OK;
//    StaticADControl_ResetMeaLedParam(FALSE);    //重设对应LED的数字电位器参数
//    LEDController_TurnOnLed();
    OpticalLed_TurnOn();
    DscpDevice_SendStatus(dscp, ret);
}
/**
 * @brief 设置信号AD上报周期。
 * @param dscp
 * @param data
 * @param len
 */
void OpticalAcquirer_SetSignalADNotifyPeriod(DscpDevice* dscp, Byte* data,
        Uint16 len)
{
    float period;
    int size = 0;
    Uint16 ret = DSCP_OK;

    size = sizeof(float);
    if ((len > size))
    {
        ret = DSCP_ERROR;
    }
    else
    {
        memcpy(&period, data, sizeof(float));
        OpticalControl_SetSignalADNotifyPeriod(period);
    }
    DscpDevice_SendStatus(dscp, ret);
}

/**
 * @brief 启动采集过程
 * @param dscp
 * @param data
 * @param len
 */
void OpticalAcquirer_StartAcquirer(DscpDevice* dscp, Byte* data, Uint16 len)
{
	unsigned short ret = DSCP_OK;
	float adacquiretime;
	Uint8 adcType;
	Uint16 size = sizeof(float) + sizeof(Uint8);

	TRACE_INFO("OpticalAcquirer_StartAcquirer");

	if (len > size)
	{
		ret = DSCP_ERROR;
		TRACE_ERROR("Parame Len Error\n");
		TRACE_ERROR("%d \n", size);
	}
	else if(len < size)  //兼容旧版本命令
	{
		memcpy(&adacquiretime, data, sizeof(adacquiretime));
		TRACE_DEBUG("\n Time %d ms", (Uint32 )(adacquiretime * 1000));
		OpticalControl_SendEventOpen();
		if (FALSE == OpticalControl_StartAcquirer(1, adacquiretime))
		{
			ret = DSCP_ERROR;
		}
	}
	else
	{
		memcpy(&adacquiretime, data, sizeof(adacquiretime));
		memcpy(&adcType, data + sizeof(adacquiretime), sizeof(adcType));
		TRACE_DEBUG("\n Time %d ms", (Uint32 )(adacquiretime * 1000));
		OpticalControl_SendEventOpen();
		if (FALSE == OpticalControl_StartAcquirer(adcType, adacquiretime))
		{
			ret = DSCP_ERROR;
		}
	}
	DscpDevice_SendStatus(dscp, ret);
}
/**
 * @brief 停止采集过程
 * @param dscp
 * @param data
 * @param len
 */
void OpticalAcquirer_StopAcquirer(DscpDevice* dscp, Byte* data, Uint16 len)
{
    unsigned short ret = DSCP_OK;
    Bool result;
    result = OpticalControl_StopAcquirer();
    if (result == FALSE)
    {
        ret = DSCP_ERROR;
    }
    OpticalControl_SendEventOpen();
    DscpDevice_SendStatus(dscp, ret);
}

void OpticalAcquirer_StartLEDController(DscpDevice* dscp, Byte* data,
        Uint16 len)
{
    unsigned short ret = DSCP_OK;
    DscpDevice_SendStatus(dscp, ret);
}

void OpticalAcquirer_StopLEDController(DscpDevice* dscp, Byte* data, Uint16 len)
{
    unsigned short ret = DSCP_OK;
    DscpDevice_SendStatus(dscp, ret);
}

void OpticalAcquirer_GetLEDControllerTarget(DscpDevice* dscp, Byte* data,
        Uint16 len)
{
    Uint32 target;

    DscpDevice_SendResp(dscp, &target, sizeof(Uint32));
}

void OpticalAcquirer_SetLEDControllerTarget(DscpDevice* dscp, Byte* data,
        Uint16 len)
{
    Uint16 ret = DSCP_OK;
    Uint32 target;
    Uint16 size = 0;

    size = sizeof(Uint32);
    if ((len > size))
    {
        ret = DSCP_ERROR;
        TRACE_ERROR("Parame Len Error\n");
        TRACE_ERROR("%d \n", size);
    }
    else
    {
        memcpy(&target, data, sizeof(Uint32));
    }
    DscpDevice_SendStatus(dscp, ret);
}

void OpticalAcquirer_GetLEDControllerParam(DscpDevice* dscp, Byte* data,
        Uint16 len)
{
//    LEDControllerParam ledControllerParam;
//
//    ledControllerParam = LEDController_GetParam(0);
//    TRACE_DEBUG("\n p =");
//    System_PrintfFloat(TRACE_LEVEL_DEBUG, ledControllerParam.proportion, 5);
//    TRACE_DEBUG("\n i =");
//    System_PrintfFloat(TRACE_LEVEL_DEBUG, ledControllerParam.integration, 5);
//    TRACE_DEBUG("\n d =");
//    System_PrintfFloat(TRACE_LEVEL_DEBUG, ledControllerParam.differential, 5);
//
//    DscpDevice_SendResp(dscp, &ledControllerParam, sizeof(LEDControllerParam));
}

void OpticalAcquirer_SetLEDControllerParam(DscpDevice* dscp, Byte* data,
        Uint16 len)
{
//    LEDControllerParam ledControllerParam;
    Uint16 ret = DSCP_OK;
//    Uint16 size = 0;
//
//    size = sizeof(LEDControllerParam);
//    if ((len > size))
//    {
//        ret = DSCP_ERROR;
//        TRACE_ERROR("Parame Len Error\n");
//        TRACE_ERROR("%d \n", size);
//    }
//    else
//    {
//        memcpy(&ledControllerParam, data, sizeof(LEDControllerParam));
//        LEDController_SetParam(0, ledControllerParam);
//    }
    DscpDevice_SendStatus(dscp, ret);
}

void OpticalAcquirer_StartLEDAdjust(DscpDevice* dscp, Byte* data,
        Uint16 len)
{
    unsigned short ret = DSCP_OK;
    float targetAD;
    float tolerance;
    Uint32 timeout;

    int size = 0;
    //设置数据正确性判断
    size =  sizeof(Uint32) * 3;
    if ((len > size))
    {
        ret = DSCP_ERROR;
        TRACE_ERROR("Parame Len Error\n");
        TRACE_ERROR("%d \n", size);
    }
    else
    {
    	 ret = DSCP_ERROR;
    }

    DscpDevice_SendStatus(dscp, ret);
}

void OpticalAcquirer_StopLEDAdjust(DscpDevice* dscp, Byte* data, Uint16 len)
{
    unsigned short ret = DSCP_OK;
    DscpDevice_SendStatus(dscp, ret);
}

void OpticalAcquirer_StartStaticADControl(DscpDevice* dscp, Byte* data, Uint16 len)
{
    unsigned short ret = DSCP_OK;
    Uint8 index;
    Uint32 targetAD;

    int size = 0;
    //设置数据正确性判断
    size =  sizeof(Uint8)  + sizeof(Uint32);
    if ((len > size))
    {
        ret = DSCP_ERROR;
        TRACE_ERROR("Parame Len Error\n");
        TRACE_ERROR("%d \n", size);
    }
    else
    {
    	ret = DSCP_ERROR;
    }

    DscpDevice_SendStatus(dscp, ret);
}

void OpticalAcquirer_StopStaticADControl(DscpDevice* dscp, Byte* data, Uint16 len)
{
    unsigned short ret = DSCP_OK;
    DscpDevice_SendStatus(dscp, ret);
}

void OpticalAcquirer_GetStaticADControlParam(DscpDevice* dscp, Byte* data, Uint16 len)
{
    Uint8 index;
    Uint16 value = DSCP_ERROR;

    int size = 0;
    //设置数据正确性判断
    size =  sizeof(Uint8);
    if ((len > size))
    {
        TRACE_ERROR("param error len : %d > %d\n", len, size);
    }
    else
    {
    	memcpy(&index, data, sizeof(Uint8));

		if(index < 2)
		{
			value = OpticalControl_AD5175Read(index);
		}
		else
		{
			TRACE_ERROR("param index error \n");
		}
    	TRACE_ERROR("GetStaticADControlParam\n");
    }

    DscpDevice_SendResp(dscp, &value, sizeof(Uint16));
}

void OpticalAcquirer_SetStaticADControlParam(DscpDevice* dscp, Byte* data, Uint16 len)
{
    Uint16 ret = DSCP_OK;
    Uint16 size = 0;
    Uint8 index;
    Uint16 value;

    size = sizeof(Uint8) + sizeof(Uint16);
    if ((len > size))
    {
        ret = DSCP_ERROR;
        TRACE_ERROR("param error len : %d > %d\n", len, size);
    }
    else
    {
    	memcpy(&index, data, sizeof(Uint8));
    	memcpy(&value, data + 1, sizeof(Uint16));
    	OpticalControl_AD5175Write(index, value);
    	TRACE_ERROR("SetStaticADControlParam\n");
    }
    DscpDevice_SendStatus(dscp, ret);
}

void OpticalAcquire_IsADControlValid(DscpDevice* dscp, Byte* data, Uint16 len)
{
    Uint16 ret = DSCP_ERROR;
    DscpDevice_SendStatus(dscp, ret);
}

void OpticalAcquirer_GetLEDDefaultValue(DscpDevice* dscp, Byte* data, Uint16 len)
{
    float value;
    value = (float)ChannelDriver_GetGainMap();
    TRACE_DEBUG("\n GainMap value: ");
    System_PrintfFloat(TRACE_LEVEL_DEBUG, value, 3);
    DscpDevice_SendResp(dscp, &value, sizeof(float));
}

void OpticalAcquirer_SetLEDDefaultValue(DscpDevice* dscp, Byte* data, Uint16 len)
{
    Uint16 ret = DSCP_OK;
    float value;
    Uint16 size = 0;

    size = sizeof(float);
    if ((len > size))
    {
        ret = DSCP_ERROR;
        TRACE_ERROR("Parame Len Error %d\n", size);
    }
    else
    {
        memcpy(&value, data, sizeof(float));
        Printf("\nGainMap Rec");
        float* flArr = &value;
        Uint8 param = (Uint8)round(flArr[0]);
        Printf("\nGainMap %d", param);
        ChannelDriver_SetGainMap(param);
        ret = DSCP_OK;
    }
    DscpDevice_SendStatus(dscp, ret);
}

/**
 * @brief 设置测量值上报周期。
 * @details 系统将根据设定的周期，定时向上发出上报事件。
 * @param period Float32，测量值上报周期，单位为秒。0表示不需要上报，默认为1。
 * @see
 * @note 所设置的上报周期将在下一次启动时丢失，默认为2。
 */
void OpticalAcquirer_SetNotifyPeriod(DscpDevice* dscp, Byte* data, Uint16 len)
{
    Float32 period;
    int size = 0;
    Uint16 ret = DSCP_OK;
    //设置数据正确性判断
    size = sizeof(period);
    if ((len > size))
    {
        ret = DSCP_ERROR;
        TRACE_ERROR("Parame Len Error\n");
        TRACE_ERROR("%d \n", size);
    }
    //修改并保存
    if (DSCP_OK == ret)
    {
        //修改并保存
        memcpy(&period, data, sizeof(period));
    }
    DscpDevice_SendStatus(dscp, ret);
}

/**
 * @brief 设置漏液上报周期。
 * @details 系统将根据设定的周期，定时向上发出漏液上报事件。
 * @param period Float32，漏液上报周期，单位为秒。0表示不需要上报，默认为5。
 * @see DSCP_EVENT_DCI_CHECK_LEAKING_NOTICE
 * @note 所设置的上报周期将在下一次启动时丢失，默认为0，不上报。
 */
void OpticalAcquirer_SetCheckLeakingReportPeriod(DscpDevice* dscp, Byte* data, Uint16 len)
{
    Float32 period;
    int size = 0;
    Uint16 ret = DSCP_OK;
    //设置数据正确性判断
    size = sizeof(period);
    if ((len > size))
    {
        ret = DSCP_ERROR;
        TRACE_ERROR("Parame Len Error\n");
        TRACE_ERROR("%d \n", size);
    }
    //修改并保存
    if (DSCP_OK == ret)
    {
        //修改并保存
        memcpy(&period, data, sizeof(period));
//        CheckLeaking_SetCheckLeakingReportPeriod(period);
    }
    DscpDevice_SendStatus(dscp, ret);
}

