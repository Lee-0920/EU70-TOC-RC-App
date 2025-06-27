/**
 * @file
 * @brief AD7791信号采集驱动。
 * @details 提供光学信号AD采集功能接口。
 * @version 1.1.0
 * @author kim.xiejinqiang
 * @date 2015-06-04
 */


#include <OpticalDriver/AD7791Collect.h>
#include "FreeRTOS.h"
#include "task.h"
#include "Tracer/Trace.h"
#include "Driver/System.h"

// **************** 命令 ****************
#define AD7791_CMD_READ_STATUS      0x08    // 读状态
#define AD7791_CMD_SETTING_MODE     0x10    // 设置模式
#define AD7791_CMD_SETTING_FILTER   0x20    // 设置频率(频率与滤波相关)
#define AD7791_CMD_READ_SINGLE      0x38    // 单次读取数据
#define AD7791_CMD_READ_CONTINUOUS  0x3C    // 连续读取数据

// **************** 参数 ****************


// 缓冲
#define AD7791_MODE_BUFFER     (1 << 1)     // 缓存模式
#define AD7791_MODE_UNBUFFER   0x00         // 非缓存模式

// 转换方式
#define AD7791_MODE_CONV_SINGLE      0x80   // 单次转换
#define AD7791_MODE_CONV_CONTINUOUS  0x00   // 连续转换

// 虚位
#define AD7791_DUMMY_BYTE 0xFF

#define DRDY_MODE               (1 << 3)
//#define AD7791_COMMUNICATION_TIMEOUT_MS  25
#define AD7791_COMMUNICATION_TIMEOUT_MS  125
// 频率
#define AD7791_UPDATE_RATES_120    0x00     // 120Hz，25 dB @ 60 Hz
#define AD7791_UPDATE_RATES_100    0x01     // 100Hz，25 dB @ 50 Hz
#define AD7791_UPDATE_RATES_33_3   0x02     // 33.3Hz
#define AD7791_UPDATE_RATES_20     0x03     // 20Hz，80 dB @ 60 Hz
#define AD7791_UPDATE_RATES_16_6   0x04     // 16.6Hz，65 dB @ 50 Hz/60 Hz (Default Setting)
#define AD7791_UPDATE_RATES_16_7   0x05     // 16.7Hz，80 dB @ 50 Hz
#define AD7791_UPDATE_RATES_13_3   0x06     // 13.3Hz
#define AD7791_UPDATE_RATES_9_5    0x07     // 9.5Hz，67 dB @ 50/60 Hz

static unsigned char s_AD7791PolarMode = AD7791_MODE_UNIPOLAR;

void AD7791_SCLK_High(AD7791Driver* ad7791)
{
    GPIO_SetBits(ad7791->portSCK, ad7791->pinSCK);
}

void AD7791_SCLK_Low(AD7791Driver* ad7791)
{
    GPIO_ResetBits(ad7791->portSCK, ad7791->pinSCK);
}

void AD7791_CS_High(AD7791Driver* ad7791)
{
    GPIO_SetBits(ad7791->portCS, ad7791->pinCS);
}

void AD7791_CS_Low(AD7791Driver* ad7791)
{
    GPIO_ResetBits(ad7791->portCS, ad7791->pinCS);
}

void AD7791_DIN_High(AD7791Driver* ad7791)
{
    GPIO_SetBits(ad7791->portDIN, ad7791->pinDIN);
}

void AD7791_DIN_Low(AD7791Driver* ad7791)
{
    GPIO_ResetBits(ad7791->portDIN, ad7791->pinDIN);
}

Uint8 AD7791_DOUT_Read(AD7791Driver* ad7791)
{
    return GPIO_ReadInputDataBit(ad7791->portDOUT, ad7791->pinDOUT);
}

/**
 * @brief AD7791发送字节
 * @param
 */
static uint8_t AD7791Collect_AD7791SendByte(AD7791Driver* ad7791, uint8_t senddata)
{

    uint8_t i = 0;
    uint8_t receivedata = 0;
    for (i = 0; i < 8; i++)
    {
        // 发送
        AD7791_SCLK_Low(ad7791);
        if (senddata & 0x80)
        {
            AD7791_DIN_High(ad7791);
        }
        else
        {
            AD7791_DIN_Low(ad7791);
        }
        senddata <<= 1;
        AD7791_SCLK_High(ad7791);

        // 接收
        receivedata <<= 1;
        if (AD7791_DOUT_Read(ad7791))
        {
            receivedata |= 0x01;
        }
    }
    return receivedata;
}

/**
 * @brief 检查AD7791是否转换完成
 * @param
 */
static Bool AD7791Collect_AD7791Ready(AD7791Driver* ad7791, uint32_t timeoutms)
{
    uint32_t loopms = 0;
    uint32_t looptimes = 0;
    loopms = 2;
    looptimes = timeoutms / loopms;     // 循环查询次数
    while (AD7791_DOUT_Read(ad7791))       // 等待DOUT由期间拉低
    {
        System_Delay(loopms);
        if (0 == looptimes)
        {
            return 0;
        }
        looptimes--;
    }
    return 1;
}

/**
 * @brief 读取AD7791 24位数据
 * @param
 */
static uint32_t AD7791Collect_AD7791Read24bitData(AD7791Driver* ad7791)
{
    uint32_t read_data = 0;
    // 读数据
    read_data = 0;
    read_data |= AD7791Collect_AD7791SendByte(ad7791, AD7791_DUMMY_BYTE) << 16;
    read_data |= AD7791Collect_AD7791SendByte(ad7791, AD7791_DUMMY_BYTE) << 8;
    read_data |= AD7791Collect_AD7791SendByte(ad7791, AD7791_DUMMY_BYTE);
    return read_data;
}

/**
 * @brief 光学采集驱动初始化
 * @param
 */
void AD7791Collect_Init(AD7791Driver* ad7791)
{
    GPIO_InitTypeDef GPIO_InitStructure;

    //时钟配置
    RCC_AHB1PeriphClockCmd(ad7791->rccCS |
	ad7791->rccSCK |
	ad7791->rccDIN |
	ad7791->rccDOUT , ENABLE);

    //IO配置
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
    GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;

    // CS
    GPIO_InitStructure.GPIO_Pin = ad7791->pinCS;
    GPIO_Init(ad7791->portCS, &GPIO_InitStructure);

    // SCLK
    GPIO_InitStructure.GPIO_Pin = ad7791->pinSCK;
    GPIO_Init(ad7791->portSCK, &GPIO_InitStructure);

    // DIN
    GPIO_InitStructure.GPIO_Pin = ad7791->pinDIN;
    GPIO_Init(ad7791->portDIN, &GPIO_InitStructure);

    // DOUT
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN;
    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
    GPIO_InitStructure.GPIO_Pin = ad7791->pinDOUT;
    GPIO_Init(ad7791->portDOUT, &GPIO_InitStructure);

    // 初始引脚
    AD7791_CS_High(ad7791);
    AD7791_SCLK_High(ad7791);
    AD7791_DIN_High(ad7791);

    // 复位
    AD7791_CS_Low(ad7791);    // 使能
    AD7791Collect_AD7791SendByte(ad7791, 0xff);       // 复位芯片，最少32个时钟的 DIN 高电平
    AD7791Collect_AD7791SendByte(ad7791, 0xff);
    AD7791Collect_AD7791SendByte(ad7791, 0xff);
    AD7791Collect_AD7791SendByte(ad7791, 0xff);
    AD7791Collect_AD7791SendByte(ad7791, 0xff);
    AD7791Collect_AD7791SendByte(ad7791, 0xff);
    AD7791_CS_High(ad7791);   // 失能

    // 设置采样频率
    AD7791_CS_Low(ad7791);    // 使能
    AD7791Collect_AD7791SendByte(ad7791, AD7791_CMD_SETTING_FILTER);
    AD7791Collect_AD7791SendByte(ad7791, AD7791_UPDATE_RATES_33_3 & 0x07);

    AD7791_CS_High(ad7791);   // 失能

    TRACE_DEBUG("\n AD7791 Collect Init Over");
}

uint32_t AD7791Collect_GetAD(AD7791Driver* ad7791)
{
    uint32_t data = 0;
//    Printf("\n pin %x, port %x", ad7791->pinCS, ad7791->portCS);
    System_Delay(20);
    AD7791_CS_Low(ad7791);    // 使能
    AD7791Collect_AD7791SendByte(ad7791, AD7791_CMD_SETTING_MODE);
    AD7791Collect_AD7791SendByte(ad7791,
            AD7791_MODE_CONV_SINGLE | AD7791_MODE_BUFFER | s_AD7791PolarMode);
    if (0 == AD7791Collect_AD7791Ready(ad7791, AD7791_COMMUNICATION_TIMEOUT_MS)) // 等待转换完成
    {
        TRACE_ERROR("\n AD7791 collect time over");
        AD7791_CS_High(ad7791);   // 失能
        return 0;   // 如果超时
    }
    // 读取数据
    AD7791Collect_AD7791SendByte(ad7791, AD7791_CMD_READ_SINGLE);
    data = AD7791Collect_AD7791Read24bitData(ad7791);
    AD7791_CS_High(ad7791);   // 失能
    return (data & 0xFFFFFF);  // 24bit
}

void AD7791Collect_ChangePolar(unsigned char polar)
{
    s_AD7791PolarMode = polar;
}


float AD7791Collect_GetResult(Int32 tempData)
{
	Int32 tData = 0;
	float temp = 0;

	tData = tempData;
//	if(tData >= 0x800000)
//	{
//		tData -= 0x800000;
////		float data = (float)tData;
////		float a = data/(8388607.0);
////		temp = a * 2.5 - 2.5;
//		temp = (float)tData * 2.5 / 8388607 + 2.5;
//	}
//	else
//	{
////		temp = (float)tData * 2.048 / (8388607.0);
//		tData = 0x800000 - tData;
//		temp = (float)tData * 2.5 / 8388607 + 2.5;
//	}

	temp = (float)tData * 2.481 / 0xFFFFFF;
//	System_PrintfFloat(TRACE_LEVEL_INFO, valve * 2.5 / 8388607 + 2.5, 3);
//	TRACE_INFO(" V");

	return temp;
}

void AD7791Collect_Test(AD7791Driver* ad7791)
{
	AD7791_CS_Low(ad7791);    // 使能
	AD7791_DIN_Low(ad7791);
	AD7791_SCLK_Low(ad7791);
	System_Delay(20);
	AD7791_CS_High(ad7791);    // 使能
	AD7791_DIN_High(ad7791);
	AD7791_SCLK_High(ad7791);
}

