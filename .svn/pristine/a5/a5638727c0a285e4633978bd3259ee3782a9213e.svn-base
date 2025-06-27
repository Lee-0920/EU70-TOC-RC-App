/*
 * ADS1220Driver.c
 *
 *  Created on: 2024年3月6日
 *      Author: Lee
 */
#include <OpticalDriver/ADS1220Driver.h>
#include "Tracer/Trace.h"
#include "Driver/System.h"
#include "FreeRTOS.h"
#include "task.h"
#include "string.h"
#include "math.h"

// **************** 命令 ****************
#define ADS1220_CMD_RESET       0x06        //
#define ADS1220_CMD_START       0x08        //
#define ADS1220_CMD_POWER_DOWN  0x02        //
#define ADS1220_CMD_NOP         0xFF        //
#define ADS1220_CMD_RDATA       0x10        //
#define ADS1220_CMD_RREG        0x20        //
#define ADS1220_CMD_WREG        0x40        //

#define ADS1220_DATA_NOP        0x00        //

#define ADS1220_REG_LEN  		(0x03)  //写入字节数

#define ADS1220_REG_ADDR_CONFIG0  (0x00 << 2) //增益配置、输入多路复用配置
#define ADS1220_REG_ADDR_CONFIG1  (0x01 << 2) //采样速率、工作模式、转换模式、温度传感器
#define ADS1220_REG_ADDR_CONFIG2  (0x02 << 2) //基准电压选择、滤波器配置、低测电源开关、IDAC电流配置
#define ADS1220_REG_ADDR_CONFIG3  (0x03 << 2)

//********ADS1220_REG_ADDR_CONFIG0*********
// ######## 增益配置 GAIN ########
#define GAIN_1                  0x00
#define GAIN_2                  (0x01 << 1)
#define GAIN_4                  (0x20 << 1)
#define GAIN_8                  (0x30 << 1)
#define GAIN_16                 (0x40 << 1)
#define GAIN_32                 (0x50 << 1)
#define GAIN_64                 (0x60 << 1)
#define GAIN_128                (0x70 << 1)

//*********ADS1220_REG_ADDR_CONFIG1**********
// ######## 采样速率 RATE ######## 正常模式 ## TURBO模式下翻倍 ## 占空比模式下1/4
#define RATE_20                 (0x00 << 5)
#define RATE_45                 (0x01 << 5)
#define RATE_90                 (0x02 << 5)
#define RATE_175                (0x03 << 5)
#define RATE_330                (0x04 << 5)
#define RATE_600                (0x05 << 5)
#define RATE_1000               (0x06 << 5)

// 工作模式
#define WORK_MODE_NORMAL 	    (0x00 << 3) //正常模式(256kHz)
#define WORK_MODE_PWM			(0x01 << 3) //占空比模式(内部占空比 1：4)
#define WORK_MODE_TURBO			(0x03 << 3) //Turbo模式(512kHz)

//转换模式
#define CONVER_MODE_SINGLE		(0x00 << 2)
#define CONVER_MODE_CONTINUE	(0x01 << 2)

//温度传感器模式
#define TEMPERATOR_DISABLE		(0x00 << 1)
#define TEMPERATOR_ENABLE	    (0x01 << 1)

//*********ADS1220_REG_ADDR_CONFIG2*********
//基准电压
#define VREF_INTERL             (0x00 << 6)
#define VREF_REFP0_REFN0        (0x01 << 6)
#define VERF_AIN0_REFP1         (0x02 << 6)
#define VREF_AVDD_AVSS          (0x03 << 6)
//滤波配置
#define FIR_NONE                (0x00 << 4)
#define FIR_50HZ_60HZ           (0x01 << 4)
#define FIR_50HZ                (0x02 << 4)
#define FIR_60HZ                (0x03 << 4)
//IDAC电流设置
#define IDAC_OFF                0x00
#define IDAC_10UA               0x01
#define IDAC_50UA               0x02
#define IDAC_100UA              0x03
#define IDAC_250UA              0x04
#define IDAC_500UA              0x05
#define IDAC_1000UA             0x06
#define IDAC_1500UA             0x07

#define ADS1220_COMMUNICATION_TIMEOUT_MS  500             // 等待转换完成超时时间

#define SAMPLE_SYNC_TIMEOUT_MS            20           // 等待采样同步信号超时时间

// CS
#define ADS1220_CS_PIN           GPIO_Pin_12
#define ADS1220_CS_GPIO          GPIOB
#define ADS1220_CS_RCC           RCC_AHB1Periph_GPIOB
#define ADS1220_CS_HIGH()        GPIO_SetBits(ADS1220_CS_GPIO, ADS1220_CS_PIN)
#define ADS1220_CS_LOW()         GPIO_ResetBits(ADS1220_CS_GPIO, ADS1220_CS_PIN)

// SCLK
#define ADS1220_SCLK_PIN         GPIO_Pin_13
#define ADS1220_SCLK_GPIO        GPIOB
#define ADS1220_SCLK_RCC         RCC_AHB1Periph_GPIOB
#define ADS1220_SCLK_HIGH()      GPIO_SetBits(ADS1220_SCLK_GPIO, ADS1220_SCLK_PIN)
#define ADS1220_SCLK_LOW()       GPIO_ResetBits(ADS1220_SCLK_GPIO, ADS1220_SCLK_PIN)

// DIN
#define ADS1220_DIN_PIN          GPIO_Pin_15
#define ADS1220_DIN_GPIO         GPIOB
#define ADS1220_DIN_RCC          RCC_AHB1Periph_GPIOB
#define ADS1220_DIN_HIGH()       GPIO_SetBits(ADS1220_DIN_GPIO, ADS1220_DIN_PIN)
#define ADS1220_DIN_LOW()        GPIO_ResetBits(ADS1220_DIN_GPIO, ADS1220_DIN_PIN)

// DOUT
#define ADS1220_DOUT_PIN         GPIO_Pin_14
#define ADS1220_DOUT_GPIO        GPIOB
#define ADS1220_DOUT_RCC         RCC_AHB1Periph_GPIOB
#define ADS1220_DOUT_READ()      GPIO_ReadInputDataBit(ADS1220_DOUT_GPIO, ADS1220_DOUT_PIN)

// DRDY
#define ADS1220_DRDY_PIN         GPIO_Pin_8
#define ADS1220_DRDY_GPIO        GPIOD
#define ADS1220_DRDY_RCC         RCC_AHB1Periph_GPIOD
#define ADS1220_DRDY_READ()      GPIO_ReadInputDataBit(ADS1220_DRDY_GPIO, ADS1220_DRDY_PIN)

/**
 * @brief ADS1220发送字节
 * @param
 */
static uint8_t ADS1220Driver_ADS1220SendByte(uint8_t senddata)
{
    uint8_t i = 0;
    uint8_t receivedata = 0;

    for (i = 0; i < 8; i++)
    {
        // 发送
        ADS1220_SCLK_HIGH();
        if (senddata & 0x80)
        {
            ADS1220_DIN_HIGH();
//            Printf("\n@");
        }
        else
        {
            ADS1220_DIN_LOW();
//            Printf("\n*");
        }
        senddata <<= 1;

        ADS1220_SCLK_LOW();

        // 接收
        receivedata <<= 1;
        if (ADS1220_DOUT_READ())
		{
			receivedata |= 0x01;
		}
    }
//    Printf("\n**********%d",receivedata);

    return receivedata;
}

/**
 * @brief 光学采集驱动初始化
 * @param
 */
void ADS1220Driver_Init(void)
{
    GPIO_InitTypeDef GPIO_InitStructure;

    //时钟配置
    RCC_AHB1PeriphClockCmd(
            ADS1220_SCLK_RCC |
            ADS1220_DIN_RCC |
            ADS1220_DOUT_RCC |
            ADS1220_CS_RCC |
            ADS1220_DRDY_RCC , ENABLE);

    //IO配置
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
    GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;

    // CS
    GPIO_InitStructure.GPIO_Pin = ADS1220_CS_PIN;
    GPIO_Init(ADS1220_CS_GPIO, &GPIO_InitStructure);

    // SCLK
    GPIO_InitStructure.GPIO_Pin = ADS1220_SCLK_PIN;
    GPIO_Init(ADS1220_SCLK_GPIO, &GPIO_InitStructure);

    // DIN
    GPIO_InitStructure.GPIO_Pin = ADS1220_DIN_PIN;
    GPIO_Init(ADS1220_DIN_GPIO, &GPIO_InitStructure);

    // DOUT
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN;
    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
    GPIO_InitStructure.GPIO_Pin = ADS1220_DOUT_PIN;
    GPIO_Init(ADS1220_DOUT_GPIO, &GPIO_InitStructure);

    // DRDY
    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
    GPIO_InitStructure.GPIO_Pin = ADS1220_DRDY_PIN;
    GPIO_Init(ADS1220_DRDY_GPIO, &GPIO_InitStructure);

    //片选
    ADS1220_CS_HIGH();

    // 初始引脚
    ADS1220_SCLK_HIGH();
    ADS1220_DIN_HIGH();

    // SPI接口初始化
    ADS1220_CS_HIGH();       // 失能
    System_NonOSDelay(1);
    ADS1220_CS_LOW();        // 使能
    System_NonOSDelay(1);//1ms
    // 复位
    ADS1220Driver_ADS1220SendByte(ADS1220_CMD_RESET);
    System_NonOSDelay(1);//1ms

    // 等待 至少0.6ms
    System_NonOSDelay(1);//1ms

    //寄存器配置 写命令 | 起始寄存器地址 | 字节长度-1
	ADS1220Driver_ADS1220SendByte(ADS1220_CMD_WREG | ADS1220_REG_ADDR_CONFIG0 | ((3-1)&ADS1220_REG_LEN));
	// 设置增益
	ADS1220Driver_ADS1220SendByte(GAIN_1);
	// 设置采样率-工作模式-转换模式 必须启动温度传感器
	ADS1220Driver_ADS1220SendByte(RATE_1000 | WORK_MODE_NORMAL | CONVER_MODE_CONTINUE | TEMPERATOR_ENABLE);
    // 设置基准电压-滤波-IDAC
    ADS1220Driver_ADS1220SendByte(VREF_INTERL | FIR_NONE | IDAC_OFF);//VREF_INTERL | FIR_NONE | IDAC_OFF

    //启动转换
//    ADS1220Driver_ADS1220SendByte(ADS1220_CMD_START);
    System_NonOSDelay(1);//1ms

    ADS1220_CS_HIGH();       // 失能

    TRACE_INFO("\n ADS1220 Collect Init Over");
}
/**
 * @brief 检查ADS1220是否转换完成
 * @param
 */
static Bool ADS1220Driver_ADS1220Ready(uint32_t timeoutms)
{
    uint32_t loopms = 0;
    uint32_t looptimes = 0;
    loopms = 1;
    looptimes = timeoutms / loopms;     // 循环查询次数

    while(ADS1220_DRDY_READ())         // 等待DOUT由期间拉低
    {
        System_Delay(loopms);
        if (0 == looptimes)
        {
            return FALSE;
        }
        looptimes--;
    }
    if(looptimes < 5)
    {
    	 TRACE_ERROR("Ready error %d", looptimes);
    }
    TRACE_MARK("Ready %d", looptimes);

    return TRUE;
}

/**
 * @brief 读取ADS1220 16位数据
 * @param
 */
static uint32_t ADS1220Driver_ADS1220Read24bitData()
{
    uint32_t read_data = 0;

    // 读数据
    read_data = 0;
    read_data |= ADS1220Driver_ADS1220SendByte(ADS1220_CMD_NOP)<< 16;
    read_data |= ADS1220Driver_ADS1220SendByte(ADS1220_CMD_NOP)<< 8;
    read_data |= ADS1220Driver_ADS1220SendByte(ADS1220_CMD_NOP);
    TRACE_MARK("\n %x \t %x \t %x", read_data >> 16, read_data & 0xFF00, read_data & 0xFF);

    return (read_data & 0xFFFFFF);
}

Uint32 ADS1220Driver_GetAD(void)
{
    Uint32 data = 0;

    ADS1220_CS_LOW();    // 使能
    //启动转换
//   ADS1220Driver_ADS1220SendByte(ADS1220_CMD_RDATA);
    ADS1220Driver_ADS1220SendByte(ADS1220_CMD_START); //转换指令 连续模式直接读DRDY状态

    if (FALSE == ADS1220Driver_ADS1220Ready(ADS1220_COMMUNICATION_TIMEOUT_MS))
    {
        TRACE_ERROR("\n ADS1220 collect time over");
        return 0;   // 如果超时
    }
    // 读取数据
    data = ADS1220Driver_ADS1220Read24bitData();
    TRACE_MARK("\n ADS1220 data %d", data);
    ADS1220_CS_HIGH();    // 失能
    return data;
}

float ADS1220Driver_GetData(Int32 tempData)
{
	Int32 tData = 0;
	float temp = 0;

	tData = tempData;
	if(tData >= 0x800000)
	{
		tData -= 0x800000;
		float data = (float)tData;
		float a = data/(8388607.0);
		temp = a * 2.5 - 2.5;
	}
	else
	{
		temp = (float)tData * 2.048 / (8388607.0);
	}

	return temp;
}

float ADS1220Driver_GetResult(Int32 tempData)
{
	Int32 tData = 0;
	float temp = 0;

	tData = tempData;
	if(tData >= 0x800000)
	{
		tData -= 0x800000;
		float data = (float)tData;
		float a = data/(8388607.0);
		temp = a * 2.5 - 2.5;
	}
	else
	{
		temp = (float)tData * 2.048 / (8388607.0);
	}

	return temp;
}

/**
 * @brief 检查采样同步信号是否到来
 * @param
 */
static Bool ADS1220Driver_IsGetADAble(uint32_t timeoutms)
{
    uint32_t loopms = 0;
    uint32_t looptimes = 0;

    loopms = 1;
    looptimes = timeoutms / loopms;             // 循环查询次数

//    while(FALSE == collectSyncFlag)
//    {
//        System_Delay(loopms);
//        if (0 == looptimes)
//        {
//            return FALSE;
//        }
//        looptimes--;
//    }
    return TRUE;
}

Uint32 ADS1220Driver_GetADWithSync(void)
{
    Uint16 result;
    result = ADS1220Driver_GetAD();
    return result;
}

Bool ADS1220Driver_GetSigelChannelAD(Uint32 *channel1Buff, Uint8 len, Uint32 sampleTimeOut)
{
    TickType_t sampleTimeout, sampleInitTime;
    Uint16 data   = 0;
    Uint8 dataLen = 0;
    Uint8 i       = 0;
    Uint32 *dataBuff = channel1Buff;

//****读取数据完成后自行修改，无需等待同步标志位
//    if (FALSE == ADS1220Driver_IsGetADAble(SAMPLE_SYNC_TIMEOUT_MS))
//    {
//       TRACE_ERROR("\n sample sync time over ");
//       return FALSE;
//    }


        sampleInitTime = xTaskGetTickCount();
        sampleTimeout = sampleTimeOut  + sampleInitTime;
        for (i = 0; i < len; i++)
        {
           data = ADS1220Driver_GetAD();

           dataBuff[i] = data;

           if (xTaskGetTickCount() >= sampleTimeout)
           {
//               TRACE_INFO("\ncur %d  timeout %d init %d", xTaskGetTickCount(), sampleTimeout, sampleInitTime);
               dataLen = 0;
               TRACE_ERROR("\n channel sample time over");
               break;
           }
           dataLen = i + 1;
        }
//        TRACE_INFO("\n %d channel sample data length = %d sample Time = %dms ", channel[j], dataLen, xTaskGetTickCount() - sampleInitTime);

        if (dataLen < len)
        {
            TRACE_ERROR("\n channel sample len = %d, curlen = %d", len, dataLen);
            return FALSE;
        }
    return TRUE;
}

void ADS1220Driver_SetSPS(Uint16 value)
{
	Uint8 rate = RATE_20;
	if(20 == value)
	{
		rate = RATE_20;
	}
	else if(45 == value)
	{
		rate = RATE_45;
	}
	else if(90 == value)
	{
		rate = RATE_90;
	}
	else if(175 == value)
	{
		rate = RATE_175;
	}
	else if(330 == value)
	{
		rate = RATE_330;
	}
	else if(600 == value)
	{
		rate = RATE_600;
	}
	else if(1000 == value)
	{
		rate = RATE_1000;
	}
	else
	{
		TRACE_ERROR("\n Invalid Rate");
		return ;
	}
	TRACE_INFO("\n ADS1220 Set SPS : %d, code %x", value, rate);

	ADS1220_CS_LOW();        // 使能
	System_NonOSDelay(1);
	ADS1220Driver_ADS1220SendByte(ADS1220_CMD_WREG | ADS1220_REG_ADDR_CONFIG1); //ADS1220_REG_LEN
	ADS1220Driver_ADS1220SendByte(rate | WORK_MODE_NORMAL | CONVER_MODE_SINGLE);//CONVER_MODE_SINGLE CONVER_MODE_CONTINUE
	ADS1220_CS_HIGH();       // 失能
}

void ADS1220Driver_RegList(void)
{
	ADS1220_CS_LOW();        // 使能
	System_NonOSDelay(1);
	ADS1220Driver_ADS1220SendByte(ADS1220_CMD_RREG | ADS1220_REG_ADDR_CONFIG0); //ADS1220_REG_LEN
	Uint8 read_data = ADS1220Driver_ADS1220SendByte(ADS1220_CMD_NOP);
	TRACE_INFO("\n ADS1220 ADS1220_REG_ADDR_CONFIG0 0x%x,", read_data);
	 ADS1220Driver_ADS1220SendByte(ADS1220_CMD_RREG | ADS1220_REG_ADDR_CONFIG1); //ADS1220_REG_LEN
	read_data = ADS1220Driver_ADS1220SendByte(ADS1220_CMD_NOP);
	TRACE_INFO("\n ADS1220 ADS1220_REG_ADDR_CONFIG1 0x%x,", read_data);
	ADS1220Driver_ADS1220SendByte(ADS1220_CMD_RREG | ADS1220_REG_ADDR_CONFIG2); //ADS1220_REG_LEN
	read_data = ADS1220Driver_ADS1220SendByte(ADS1220_CMD_NOP);
	TRACE_INFO("\n ADS1220 ADS1220_REG_ADDR_CONFIG2 0x%x,", read_data);
	ADS1220_CS_HIGH();       // 失能
}
