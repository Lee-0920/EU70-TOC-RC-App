/**
 #include <OpticalADCollect.h>
 * @file
 * @brief 光学采集控制接口实现
 * @details
 * @version 1.0.0
 * @author lemon.xiaoxun
 * @date 2016-5-27
 */

#include "Common/Types.h"
#include "DncpStack/DncpStack.h"
#include "FreeRTOS.h"
#include "task.h"
#include "semphr.h"
#include "timers.h"
#include "Tracer/Trace.h"
#include "System.h"
#include "DNCP/Lai/LaiRS485Handler.h"
//#include "Driver/OpticalDriver/OpticalXonen.h"
#include "Driver/OpticalDriver/ADS1146Collect.h"
#include "Driver/OpticalDriver/ADS131M02Driver.h"
#include "OpticalControl.h"
#include "SystemConfig.h"
#include <string.h>
//#include "Driver/OpticalDriver/ADS1259Driver.h"
//#include "Driver/OpticalDriver/LTC2400Driver.h"
//#include "Driver/OpticalDriver/ADS1220Driver.h"
#include "Driver/UltraSignalDriver/ChannelDriver.h"
#include "Driver/UltraSignalDriver/UltraPWMDeviceMap.h"
#include "Driver/OpticalDriver/AD7791Collect.h"
#include "Driver/OpticalDriver/AD5175Driver.h"
#include "LuipApi/OpticalAcquireInterface.h"
#include "TemperatureControl/TempCollecterManager.h"

typedef enum
{
    COLLECT_IDLE, COLLECT_BUSY
} CollectStatus;

#define MODE_SWITCH_TIMEOUT    1000     //双ADC模式切换时间

#define SAMPLE_DATA_QUEUE_SIZE           (5)

#define SIGNAL_SAMPLE_TIME       (45)                 // 信号采样时间  单位:ms
#define SIGNAL_SAMPLE_COUNT      (1)                  // 信号采样个数
#define DATA_FILTER_NUM(x)       (x/4)                // 数据过滤比例
#define DATA_MAX_LENGHT          (25)               // 数据最大长度

#define ADS1146_DATA_MAX_LEN  1024
#define ADS1146_DATA_FILTER_NUM(x)       (x/15)                // 数据过滤比例

typedef struct
{
    Uint8 ret;
    OpticalSignalAD ad;
}OpticalAcquireResult;

typedef struct
{
    Bool setModeEvent;
    OpticalWorkMode workMode;
    OpticalControlStatus status;
    Bool isRequestStop;
    Bool isSendEvent;
    QueueHandle_t ad7791DataQueue;//采集处理任务运行时，用于接收采集任务发送的数据
    QueueHandle_t ads1146DataQueue;//采集处理任务运行时，用于接收采集任务发送的数据
    OpticalSignalAD curAD7791Data;    //当前采样数据
    OpticalSignalAD curADS1146Data;    //当前采样数据
    Bool isNewSampleData;
    Bool isNewAD7791Data;
    Bool isNewADS1146Data;
    Uint8 currentAcquireType;
    OpticalAcquireResult result;
    TaskHandle_t adCollectHandle;
    TaskHandle_t opticalCollectHandle;
    float acquireADTime;
    float acquireADStartTime;
    TimerHandle_t signalADNotifyPeriodTimer;
    Bool isOpenTestMode;
    Uint8 testChannel;
    float ad7791RefData;
    float ad7791MeaData;
    Uint32 referenceAD;
    Uint32 measureAD;
}OpticalControl;

static OpticalControl s_opticalControl;

static void OpticalControl_ADHandleTask(void *argument);
static void OpticalControl_ADCollectTask(void *argument);
static void OpticalControl_SignalADPeriodHandle(TimerHandle_t argument);

static AD7791Driver s_ad7791[2];
static AD5175Driver s_ad5175[1];

void AD5175_Map(AD5175Driver* ad5175)
{
	ad5175[0].pinSCL = GPIO_Pin_5;
	ad5175[0].portSCL = GPIOD;
	ad5175[0].rccSCL  = RCC_AHB1Periph_GPIOD;
	ad5175[0].pinSDA = GPIO_Pin_4;
	ad5175[0].portSDA = GPIOD;
	ad5175[0].rccSDA  = RCC_AHB1Periph_GPIOD;
    AD5175_Init(&ad5175[0]);
}

void AD7791_Map(AD7791Driver* ad7791)
{

	/*
	 * 参考端配置
	 * */
	ad7791[0].pinCS = GPIO_Pin_4;
	ad7791[0].portCS = GPIOE;
	ad7791[0].rccCS = RCC_AHB1Periph_GPIOE;
	ad7791[0].pinSCK = GPIO_Pin_2;
	ad7791[0].portSCK = GPIOE;
	ad7791[0].rccSCK = RCC_AHB1Periph_GPIOE;
	ad7791[0].pinDIN = GPIO_Pin_6;
	ad7791[0].portDIN = GPIOE;
	ad7791[0].rccDIN = RCC_AHB1Periph_GPIOE;
	ad7791[0].pinDOUT = GPIO_Pin_5;
	ad7791[0].portDOUT = GPIOE;
	ad7791[0].rccDOUT = RCC_AHB1Periph_GPIOE;
	AD7791Collect_Init(&ad7791[0]);

	/*
	 * 测量端配置
	 * */
	ad7791[1].pinCS = GPIO_Pin_12;
	ad7791[1].portCS = GPIOD;
	ad7791[1].rccCS = RCC_AHB1Periph_GPIOD;
	ad7791[1].pinSCK = GPIO_Pin_10;
	ad7791[1].portSCK = GPIOD;
	ad7791[1].rccSCK = RCC_AHB1Periph_GPIOD;
	ad7791[1].pinDIN = GPIO_Pin_9;
	ad7791[1].portDIN = GPIOD;
	ad7791[1].rccDIN = RCC_AHB1Periph_GPIOD;
	ad7791[1].pinDOUT = GPIO_Pin_11;
	ad7791[1].portDOUT = GPIOD;
	ad7791[1].rccDOUT = RCC_AHB1Periph_GPIOD;
	AD7791Collect_Init(&ad7791[1]);

}

/**
 * @brief 光学信号采集模块初始化
 */
void OpticalControl_Init(void)
{
    s_opticalControl.setModeEvent = FALSE;
    s_opticalControl.workMode = ONLY_ADS1146;
    s_opticalControl.status = OPTICALCONTROL_IDLE;
    s_opticalControl.isRequestStop = FALSE;
    s_opticalControl.isSendEvent = FALSE;
    s_opticalControl.ad7791DataQueue = xQueueCreate(SAMPLE_DATA_QUEUE_SIZE, sizeof(OpticalSignalAD));
    s_opticalControl.ads1146DataQueue = xQueueCreate(SAMPLE_DATA_QUEUE_SIZE, sizeof(OpticalSignalAD));
    s_opticalControl.curAD7791Data.reference = 0;
    s_opticalControl.curAD7791Data.measure = 0;
    s_opticalControl.curADS1146Data.reference = 0;
    s_opticalControl.curADS1146Data.measure = 0;
    s_opticalControl.acquireADTime = 0;
    s_opticalControl.acquireADStartTime = 0;
    s_opticalControl.currentAcquireType = 0;
    s_opticalControl.referenceAD = 0;
    s_opticalControl.measureAD = 0;

    ChannelDriver_Init();

	memset(s_ad5175, 0 ,sizeof(s_ad5175));
	AD5175_Map(s_ad5175);

	memset(s_ad7791, 0 ,sizeof(s_ad7791));
	AD7791_Map(s_ad7791);

    xTaskCreate(OpticalControl_ADCollectTask, "OpticalADCollect",
            256, (void *)&s_opticalControl,
            5, &s_opticalControl.adCollectHandle);

    xTaskCreate(OpticalControl_ADHandleTask, "OpticalADHandle",
            256, (void *)&s_opticalControl,
            5, &s_opticalControl.opticalCollectHandle);

    s_opticalControl.signalADNotifyPeriodTimer = xTimerCreate("signalADPeriod",
            (uint32_t) (200 / portTICK_RATE_MS), pdTRUE, (void *) 5,
            OpticalControl_SignalADPeriodHandle);
    xTimerStart(s_opticalControl.signalADNotifyPeriodTimer, 0);


}

void OpticalControl_ADCChangeInit(Uint8 type)
{
    s_opticalControl.currentAcquireType = type;
//    ADS1146Collect_Init();
//    ADS131M02Driver_Init();
//    ADS1259Driver_Init();

//    switch(type)
//    {
//        case AD7791:
//            AD7791Collect_Init();
//            break;
//
//        case ADS1146:
//            ADS1146Collect_Init();
//            break;
//
//        default:
//            break;
//    }
}

/**
 * @brief 重设信号采集模块工作模式
 */
void OpticalControl_ResetWorkMode(OpticalWorkMode mode)
{
    TRACE_INFO("\n Optical control mode reset %d", (Uint8)mode);

    taskENTER_CRITICAL();

    s_opticalControl.workMode = mode;
    s_opticalControl.setModeEvent = TRUE;  //重设模式

    taskEXIT_CRITICAL();
}

/**
 * @brief 光学采集恢复初始化
 */
void OpticalControl_Restore(void)
{

}

/**
 * @brief 光学采集模式初始化
 */
void OpticalControl_WorkModeRestore(void)
{
    OpticalControl_ResetWorkMode(s_opticalControl.workMode);

}

/**
 * @brief 获取工作模式
 */
OpticalWorkMode OpticalControl_GetWorkMode(void)
{
    return s_opticalControl.workMode;
}

/**
 * @brief 更新AD7791缓冲数据
 */
static void OpticalControl_SetAD7791Data(OpticalSignalAD sampleData)
{
//    s_opticalControl.curAD7791Data = sampleData;
//    s_opticalControl.isNewAD7791Data = TRUE;
}

/**
 * @brief 读取ADS1146缓冲数据
 */
Bool OpticalControl_GetAD7791Data(OpticalSignalAD *sampleData)
{
    Bool ret = TRUE;

    if (NULL == sampleData)
    {
        ret = FALSE;
    }

    if (TRUE == s_opticalControl.isNewAD7791Data)
    {
        *sampleData = s_opticalControl.curAD7791Data;
        s_opticalControl.isNewAD7791Data = FALSE;
    }
    else
    {
        *sampleData = s_opticalControl.curAD7791Data;
        ret = FALSE;
    }

    return ret;
}

/**
 * @brief 更新ADS1146缓冲数据
 */
static void OpticalControl_SetADS1146Data(OpticalSignalAD sampleData)
{
    s_opticalControl.curADS1146Data = sampleData;
    s_opticalControl.isNewADS1146Data = TRUE;
}

/**
 * @brief 读取ADS1146缓冲数据
 */
Bool OpticalControl_GetADS1146Data(OpticalSignalAD *sampleData)
{
    Bool ret = TRUE;

    if (NULL == sampleData)
    {
        ret = FALSE;
    }

    if (TRUE == s_opticalControl.isNewADS1146Data)
    {
        *sampleData = s_opticalControl.curADS1146Data;
        s_opticalControl.isNewADS1146Data = FALSE;
    }
    else
    {
        *sampleData = s_opticalControl.curADS1146Data;
        ret = FALSE;
    }

    return ret;
}

/**
 * @brief 启动AD采集过程
 * @param type ADCType, ADC类型
 * @param acquireADTime float, 采集时间
 */
Bool OpticalControl_StartAcquirer(Uint8 index, float acquireADTime)
{
    Bool ret = TRUE;
    return ret;
}

/**
 * @brief 停止AD采集过程
 */
Bool OpticalControl_StopAcquirer()
{
    Bool retValue = TRUE;
    if (OPTICALCONTROL_BUSY == s_opticalControl.status)              // 处于采集过程
    {
//        OpticalControl_WorkModeRestore();
        s_opticalControl.status = OPTICALCONTROL_IDLE;
        s_opticalControl.isRequestStop = TRUE;
        TRACE_INFO("\n Request stop acquirer.");
    }
    else                                    // 采集处于空闲
    {
        TRACE_ERROR("\n collect is idle");
        retValue =  FALSE;
    }
    return retValue;
}

void OpticalControl_SendEventOpen(void)
{
    s_opticalControl.isSendEvent = TRUE;
}

void OpticalControl_SendEventClose(void)
{
    s_opticalControl.isSendEvent = FALSE;
}

/**
 * @brief 实时获取AD7791数据-本地任务采集时使用
 */
OpticalSignalAD OpticalControl_GetFilterAD7791AD(OpticalControl *opticalControl)
{
    OpticalSignalAD resultAD = { 0 };
    uint64_t sumRefData = 0;
    uint64_t sumMeaData = 0;
    uint16_t count = 0;

    OpticalSignalAD sampleData;
    TickType_t timeout = (TickType_t) (opticalControl->acquireADTime * 1000) + xTaskGetTickCount();

    opticalControl->acquireADStartTime = xTaskGetTickCount();

    // 采集
    while (1)
    {
        do
        {
            System_Delay(2);
            if (TRUE == opticalControl->isRequestStop)
            {
                return resultAD;
            }
        } while (FALSE == xQueueReceive(opticalControl->ad7791DataQueue, &sampleData, 0));

        if ((sumRefData + sampleData.reference) >= 0xFFFFFFFFFFFFFFFF
                || (sumMeaData + sampleData.measure) >= 0xFFFFFFFFFFFFFFFF)
        {

            TRACE_ERROR(
                    "The intermediate variable of the measuring end or the reference end is out of range.");
            break;
        }

        sumRefData += sampleData.reference; //获取Ref端AD值
        sumMeaData += sampleData.measure; //获取Mea端AD值
        count++;
        TRACE_DEBUG("\n ad7791 ref = %d, mea = %d, count = %d", sampleData.reference, sampleData.measure, count);


        if (TRUE == opticalControl->isRequestStop)
        {
            resultAD.reference = 0;
            resultAD.measure = 0;
            return resultAD;
        }

        if (timeout <  (xTaskGetTickCount() + 100))       // 采样超时检查，剩余时间不足以再进行一次采样
        {
            break;
        }
    }

    resultAD.reference = sumRefData / count;
    resultAD.measure = sumMeaData / count;
    TRACE_MARK("\n count %d", count);
    return resultAD;
}

/**
 * @brief 实时获取ADS1146数据-本地任务采集时使用
 */
OpticalSignalAD OpticalControl_GetFilterADS1146AD(OpticalControl *opticalControl)
{
    OpticalSignalAD resultAD = { 0 };

    uint16_t count = 0;
    static Uint16 refData[ADS1146_DATA_MAX_LEN] = {0};               //275
    static Uint16 meaData[ADS1146_DATA_MAX_LEN] = {0};           //220

    memset(refData, 0, sizeof(refData));
    memset(meaData, 0, sizeof(meaData));


    OpticalSignalAD sampleData;
    TickType_t timeout = (TickType_t) (opticalControl->acquireADTime * 1000) + xTaskGetTickCount();

    opticalControl->acquireADStartTime = xTaskGetTickCount();

    // 采集
    while (1)
    {
        do
        {
            System_Delay(2);
            if (TRUE == opticalControl->isRequestStop)
            {
                return resultAD;
            }
        } while (FALSE == xQueueReceive(opticalControl->ads1146DataQueue, &sampleData, 0));

        float temp = s_opticalControl.ad7791RefData;
		refData[count] = (Uint32)(temp*1000);
		meaData[count] = (Uint32)(temp*1000);
		count++;
		TRACE_DEBUG("\n ads1146 ref %d, mea %d, count = %d",  refData[count-1],  meaData[count-1], count);

        if (TRUE == opticalControl->isRequestStop)
        {
            resultAD.reference = 0;
            resultAD.measure = 0;
            return resultAD;
        }

        if (timeout <  (xTaskGetTickCount() + 100))       // 采样超时检查，剩余时间不足以再进行一次采样
        {
            break;
        }
    }
    TRACE_INFO("\n ads1146 data collect count %d", count);

    if ((0 != count) && (FALSE == opticalControl->isRequestStop))
    {
        // 对数据进行滤波处理
        resultAD.reference = OpticalControl_FilterData(refData, count, ADS1146_DATA_FILTER_NUM(count), ADS1146_DATA_FILTER_NUM(count));
        resultAD.measure   = OpticalControl_FilterData(meaData, count, ADS1146_DATA_FILTER_NUM(count), ADS1146_DATA_FILTER_NUM(count));
        TRACE_INFO("\n ads1146 collect data ref: %d  mea: %d", sampleData.reference, sampleData.measure);
    }
    else
    {
        resultAD.reference = 0;
        resultAD.measure   = 0;
        TRACE_INFO("\n ads1146 collect data 0");
    }

    return resultAD;
}

/**
 * @brief 信号采集命令处理任务
 */
static void OpticalControl_ADHandleTask(void *argument)
{
    OpticalSignalAD resultAD = { 0 };
    OpticalControl *opticalControl;
    opticalControl = (OpticalControl *)argument;
    vTaskSuspend(NULL);
    while (1)
    {
        switch (opticalControl->status)
        {
            case OPTICALCONTROL_IDLE:
                vTaskSuspend(NULL);
                break;
            case OPTICALCONTROL_BUSY:

                break;
            case OPTICALCONTROL_COLLECT:
                vTaskSuspend(NULL);
                break;
        }
    }
}

/**
 * @brief AD信号采集任务
 */
static void OpticalControl_ADCollectTask(void *argument)
{

    OpticalControl *opticalControl;
    opticalControl = (OpticalControl *)argument;
//   vTaskSuspend(NULL);
    while (1)
    {
        if (TRUE == opticalControl->isOpenTestMode)
        {
        	Uint32 ad = AD7791Collect_GetAD(&s_ad7791[0]);
			float temp = AD7791Collect_GetResult(ad);
        }
        else
        {
        	s_opticalControl.referenceAD = AD7791Collect_GetAD(&s_ad7791[0]);
			s_opticalControl.measureAD = AD7791Collect_GetAD(&s_ad7791[1]);
//        	AD7791Collect_Test(&s_ad7791[1]);
        	s_opticalControl.ad7791RefData = AD7791Collect_GetResult(s_opticalControl.referenceAD);
        	s_opticalControl.ad7791MeaData = AD7791Collect_GetResult(s_opticalControl.measureAD);
        	vTaskDelay(50);   //LED 频率固定1Hz，调度时间影响采样个数：1000 / vTaskDelay
        }
    }
}

/**
 * @brief 设置信号上报周期
 */
void OpticalControl_SetSignalADNotifyPeriod(float period)
{
    TRACE_INFO("\n OpticalControl ad period:");
    System_PrintfFloat(TRACE_LEVEL_INFO, period, 3);
    TRACE_INFO(" s");
    if (period > 0)
    {
        xTimerChangePeriod(s_opticalControl.signalADNotifyPeriodTimer, (uint32_t)((period * 1000) / portTICK_RATE_MS),  0);
        xTimerStart(s_opticalControl.signalADNotifyPeriodTimer, 0);
    }
    else
    {
        xTimerStop(s_opticalControl.signalADNotifyPeriodTimer, 0);
    }
}

/**
 * @brief 信号周期上报任务
 */
static void OpticalControl_SignalADPeriodHandle(TimerHandle_t argument)
{
	if (TRUE == LaiRS485_GetHostStatus())
	{
		OpticalSignalAD adData[2];
		Uint8 data[8] = {0};
		memset(adData, 0, sizeof(adData));

		if(s_opticalControl.referenceAD > 0 || s_opticalControl.measureAD > 0)
		{
			memcpy(&data[0],&s_opticalControl.ad7791RefData, sizeof(s_opticalControl.ad7791RefData));
			memcpy(&data[4],&s_opticalControl.ad7791MeaData, sizeof(s_opticalControl.ad7791MeaData));
		}
		DncpStack_SendEvent(DSCP_EVENT_OAI_SIGNAL_AD_NOTICE, &data[0], sizeof(data));
	}
}

/**
 * @brief 打印调试信息
 */
void OpticalControl_PrintfInfo(void)
{
	Uint32 adRef = s_opticalControl.referenceAD;
	Uint32 adMea = s_opticalControl.measureAD;
	float tempRef = AD7791Collect_GetResult(adRef);
	float tempMea = AD7791Collect_GetResult(adMea);
	float ntc = TempCollecterManager_GetTemp(1);
	Printf("\n AD7791 [RefV] %f V, [MeaV] %f, [RefAD] %d, [MeaAD] %d,Temperature: %f", tempRef, tempMea, adRef, adMea, ntc);
}

OpticalSignalAD OpticalControl_GetNowSignalAD(void)
{
    OpticalSignalAD ad = {0};
    return ad;
}

/**
 * 对数据进行排序(冒泡法)
 * @param 数据
 * @param 数据个数
 */
static void OpticalControl_BubbleSort(Uint16 *dataBuff, Uint16 count)
{
    Uint16 i    = 0;
    Uint16 j    = 0;
    Uint16 temp = 0;

    for(i = 1; i < count; i++)
    {
        for(j = count - 1; j >= i; j--)
        {
            if(dataBuff[j] < dataBuff[j-1])
            {
                temp = dataBuff[j-1];
                dataBuff[j-1] = dataBuff[j];
                dataBuff[j] = temp;
            }
        }
    }
}

/**
 * @brief 对数据进行滤波处理，去掉最大的和最小的数据
 * @param  要进行滤波处理的数据
 * @param  数据个数
 * @param  去掉高端数据个数
 * @param  去掉低端数据个数
 * @return 滤波之后的数据
 */
Uint16 OpticalControl_FilterData(Uint16 *inputData, Uint16 count, Uint16 filterHigh, Uint16 filterLow)
{
    Uint16 i         = 0;
    uint16_t avgData = 0;
    uint32_t sumData = 0;

    // 冒泡
    OpticalControl_BubbleSort(inputData, count);

    // 过滤
    memcpy(inputData, (inputData + filterLow), (count - filterLow) * sizeof(Uint16));
    count -= (filterHigh + filterLow);

    // 取平均值
    for (i = 0; i < count; i++)
    {
        sumData += inputData[i];
    }

    avgData = sumData / count;

    return avgData;
}

/**
 * @brief LED灯控制
 * @param  0 关闭     1 打开
 */
void OpticalControl_TurnOnLed(Bool status)
{

}

/**
 * @brief 氙灯控制
 * @param  0 关闭     1 打开
 */
void OpticalControl_TurnOnXonen(Bool status)
{

}

/**
 * 氙灯定频闪烁
 * @param  0 关闭     1 打开
 */
void OpticalControl_GlitterXonen(Bool status)
{

}


OpticalControlStatus OpticalControl_GetCurrentStatus(void)
{
    return s_opticalControl.status;
}

Bool OpticalControl_EnterCollectStatus(void)
{
    if(s_opticalControl.status != OPTICALCONTROL_BUSY)
    {
        s_opticalControl.status = OPTICALCONTROL_COLLECT;
        return TRUE;
    }
    return FALSE;
}

Bool OpticalControl_EnterIdleStatus(void)
{
    if(s_opticalControl.status != OPTICALCONTROL_BUSY)
    {
        s_opticalControl.status = OPTICALCONTROL_IDLE;
        return TRUE;
    }
    return FALSE;
}

/**
 * @brief 此函数用于测试单独某个通道信号采集，测量端和参考端的采集的相关功能将无效
 * @param isOpenTestMode 是否打开测试功能，TRUE为打开
 * @param channel 需要单独测试的通道
 */
void OpticalControl_CollectTestFun(Bool isOpenTestMode, Uint8 channel)
{
    s_opticalControl.isOpenTestMode = isOpenTestMode;
    s_opticalControl.testChannel = channel;
}

//***********供MCP4651下位机测试**********
 void OpticalControl_AD5175Write(Uint8 index, Uint16 value)
 {
 	Uint8 offset = index*2;
 	Uint8 param[2] = {0};
 	Uint8 addr = AD5175_ADDR_0;
	 if(index > 0)
	 {
		 addr = AD5175_ADDR_1;
	 }
 	if(AD5175_WriteRDAC(s_ad5175, addr, value))
 	{
// 		g_staticADController.adController[index].defaultValue = value;
// 		memcpy(&param, &value, sizeof(Uint16));
// 		McuFlash_Write(MCP4651_CONTROL_PARAM_FLASH_BASE_ADDR + offset , MCP4651_CONTROL_PARAM_FLASH_LEN, param);
 		TRACE_INFO("\n AD5175 write flash done \n.");
 	}
 	else
 	{
 		TRACE_ERROR("\n AD5175 write flash failed \n.");
 	}

 }

 Uint16 OpticalControl_AD5175Read(Uint8 index)
 {
	 Uint8 addr = AD5175_ADDR_0;
	 if(index > 0)
	 {
		 addr = AD5175_ADDR_1;
	 }
	 AD5175_ReadRDAC(s_ad5175, addr);
 	TRACE_ERROR("\n AD5175 index : %d, receive addr: %x \n.", index, addr);
 }

 void OpticalControl_AD5175ReadWithAddr(Uint8 index, Uint8 addr)
 {
	 AD5175_ReadRDAC(s_ad5175, addr);
 }
