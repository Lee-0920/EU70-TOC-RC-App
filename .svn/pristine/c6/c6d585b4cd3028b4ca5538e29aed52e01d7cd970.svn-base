/**
 * @file
 * @brief 光学信号AD采集驱动。
 * @details 提供光学信号AD采集功能接口。
 * @version 1.1.0
 * @author kim.xiejinqiang
 * @date 2015-06-04
 */


#include <OpticalDriver/OpticalADCollect.h>
#include "FreeRTOS.h"
#include "task.h"
#include "Tracer/Trace.h"
#include "Driver/System.h"

// **************** 命令 ****************
#define ADS131_CMD_READ_STATUS      0x08    // 读状态
#define ADS131_CMD_SETTING_MODE     0x10    // 设置模式
#define ADS131_CMD_SETTING_FILTER   0x20    // 设置频率(频率与滤波相关)
#define ADS131_CMD_READ_SINGLE      0x38    // 单次读取数据
#define ADS131_CMD_READ_CONTINUOUS  0x3C    // 连续读取数据

// **************** 参数 ****************


// 缓冲
#define ADS131_MODE_BUFFER     (1 << 1)     // 缓存模式
#define ADS131_MODE_UNBUFFER   0x00         // 非缓存模式

// 转换方式
#define ADS131_MODE_CONV_SINGLE      0x80   // 单次转换
#define ADS131_MODE_CONV_CONTINUOUS  0x00   // 连续转换

// 虚位
#define ADS131_DUMMY_BYTE 0xFF

#define DRDY_MODE               (1 << 3)
//#define ADS131_COMMUNICATION_TIMEOUT_MS  25
#define ADS131_COMMUNICATION_TIMEOUT_MS  125
// 频率
#define ADS131_UPDATE_RATES_120    0x00     // 120Hz，25 dB @ 60 Hz
#define ADS131_UPDATE_RATES_100    0x01     // 100Hz，25 dB @ 50 Hz
#define ADS131_UPDATE_RATES_33_3   0x02     // 33.3Hz
#define ADS131_UPDATE_RATES_20     0x03     // 20Hz，80 dB @ 60 Hz
#define ADS131_UPDATE_RATES_16_6   0x04     // 16.6Hz，65 dB @ 50 Hz/60 Hz (Default Setting)
#define ADS131_UPDATE_RATES_16_7   0x05     // 16.7Hz，80 dB @ 50 Hz
#define ADS131_UPDATE_RATES_13_3   0x06     // 13.3Hz
#define ADS131_UPDATE_RATES_9_5    0x07     // 9.5Hz，67 dB @ 50/60 Hz

// CS
#define ADS131_CS_PIN           GPIO_Pin_4
#define ADS131_CS_GPIO          GPIOE
#define ADS131_CS_RCC           RCC_AHB1Periph_GPIOE
#define ADS131_CS_HIGH()        GPIO_SetBits(ADS131_CS_GPIO, ADS131_CS_PIN)
#define ADS131_CS_LOW()         GPIO_ResetBits(ADS131_CS_GPIO, ADS131_CS_PIN)

//RESET
#define ADS131_RESET_PIN           GPIO_Pin_4
#define ADS131_RESET_GPIO          GPIOE
#define ADS131_RESET_RCC           RCC_AHB1Periph_GPIOE
#define ADS131_RESET_HIGH()        GPIO_SetBits(ADS131_RESET_GPIO, ADS131_RESET_PIN)
#define ADS131_RESET_LOW()         GPIO_ResetBits(ADS131_RESET_GPIO, ADS131_RESET_PIN)

// SCLK
#define ADS131_SCLK_PIN         GPIO_Pin_2
#define ADS131_SCLK_GPIO        GPIOE
#define ADS131_SCLK_RCC         RCC_AHB1Periph_GPIOE
#define ADS131_SCLK_HIGH()      GPIO_SetBits(ADS131_SCLK_GPIO, ADS131_SCLK_PIN)
#define ADS131_SCLK_LOW()       GPIO_ResetBits(ADS131_SCLK_GPIO, ADS131_SCLK_PIN)

// DIN
#define ADS131_DIN_PIN          GPIO_Pin_6
#define ADS131_DIN_GPIO         GPIOE
#define ADS131_DIN_RCC          RCC_AHB1Periph_GPIOE
#define ADS131_DIN_HIGH()       GPIO_SetBits(ADS131_DIN_GPIO, ADS131_DIN_PIN)
#define ADS131_DIN_LOW()        GPIO_ResetBits(ADS131_DIN_GPIO, ADS131_DIN_PIN)

// DOUT
#define ADS131_DOUT_PIN         GPIO_Pin_5
#define ADS131_DOUT_GPIO        GPIOE
#define ADS131_DOUT_RCC         RCC_AHB1Periph_GPIOE
#define ADS131_DOUT_READ()      GPIO_ReadInputDataBit(ADS131_DOUT_GPIO, ADS131_DOUT_PIN)

static unsigned char s_ADS131PolarMode = ADS131_MODE_BIPOLAR;
/**
 * @brief ADS131发送字节
 * @param
 */
static uint8_t OpticalADCollect_ADS131SendByte(uint8_t senddata)
{
    uint8_t i = 0;
    uint8_t receivedata = 0;
    for (i = 0; i < 8; i++)
    {
        // 发送
        ADS131_SCLK_LOW();
        if (senddata & 0x80)
        {
            ADS131_DIN_HIGH();
        }
        else
        {
            ADS131_DIN_LOW();
        }
        senddata <<= 1;
        ADS131_SCLK_HIGH();

        // 接收
        receivedata <<= 1;
        if (ADS131_DOUT_READ())
        {
            receivedata |= 0x01;
        }
    }
    return receivedata;
}

/**
 * @brief 检查ADS131是否转换完成
 * @param
 */
static Bool OpticalADCollect_ADS131Ready(uint32_t timeoutms)
{
    uint32_t loopms = 0;
    uint32_t looptimes = 0;
    loopms = 2;
    looptimes = timeoutms / loopms;     // 循环查询次数
    while (ADS131_DOUT_READ())       // 等待DOUT由期间拉低
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
 * @brief 读取ADS131 24位数据
 * @param
 */
static uint32_t OpticalADCollect_ADS131Read24bitData()
{
    uint32_t read_data = 0;
    // 读数据
    read_data = 0;
    read_data |= OpticalADCollect_ADS131SendByte(ADS131_DUMMY_BYTE) << 16;
    read_data |= OpticalADCollect_ADS131SendByte(ADS131_DUMMY_BYTE) << 8;
    read_data |= OpticalADCollect_ADS131SendByte(ADS131_DUMMY_BYTE);
    return read_data;
}

/**
 * @brief 光学采集驱动初始化
 * @param
 */
void OpticalADCollect_Init(void)
{
    GPIO_InitTypeDef GPIO_InitStructure;

    //时钟配置
    RCC_AHB1PeriphClockCmd(ADS131_CS_RCC |
    ADS131_SCLK_RCC |
    ADS131_DIN_RCC |
    ADS131_DOUT_RCC |
	ADS131_RESET_RCC, ENABLE);

    //IO配置
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
    GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;

    // CS
    GPIO_InitStructure.GPIO_Pin = ADS131_CS_PIN;
    GPIO_Init(ADS131_CS_GPIO, &GPIO_InitStructure);
    // RESET
	GPIO_InitStructure.GPIO_Pin = ADS131_RESET_PIN;
	GPIO_Init(ADS131_RESET_GPIO, &GPIO_InitStructure);
    // SCLK
    GPIO_InitStructure.GPIO_Pin = ADS131_SCLK_PIN;
    GPIO_Init(ADS131_SCLK_GPIO, &GPIO_InitStructure);
    // DIN
    GPIO_InitStructure.GPIO_Pin = ADS131_DIN_PIN;
    GPIO_Init(ADS131_DIN_GPIO, &GPIO_InitStructure);

    // DOUT
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN;
    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
    GPIO_InitStructure.GPIO_Pin = ADS131_DOUT_PIN;
    GPIO_Init(ADS131_DOUT_GPIO, &GPIO_InitStructure);

    // 初始引脚
    ADS131_CS_HIGH();
    ADS131_SCLK_HIGH();
    ADS131_DIN_HIGH();

    // 复位
    ADS131_CS_LOW();    // 使能
    OpticalADCollect_ADS131SendByte(0xff);       // 复位芯片，最少32个时钟的 DIN 高电平
    OpticalADCollect_ADS131SendByte(0xff);
    OpticalADCollect_ADS131SendByte(0xff);
    OpticalADCollect_ADS131SendByte(0xff);
    OpticalADCollect_ADS131SendByte(0xff);
    OpticalADCollect_ADS131SendByte(0xff);
    ADS131_CS_HIGH();   // 失能

    // 设置采样频率
    ADS131_CS_LOW();    // 使能
    OpticalADCollect_ADS131SendByte(ADS131_CMD_SETTING_FILTER);
    OpticalADCollect_ADS131SendByte(ADS131_UPDATE_RATES_33_3 & 0x07);

    ADS131_CS_HIGH();   // 失能

    OpticalChannel_Init();
}

uint32_t OpticalADCollect_GetAD(Uint8 channel)
{
    uint32_t data = 0;

    OpticalChannel_Select(channel);
    System_Delay(20);
    ADS131_CS_LOW();    // 使能
    OpticalADCollect_ADS131SendByte(ADS131_CMD_SETTING_MODE);
    OpticalADCollect_ADS131SendByte(
            ADS131_MODE_CONV_SINGLE | ADS131_MODE_BUFFER | s_ADS131PolarMode);
    if (0 == OpticalADCollect_ADS131Ready(ADS131_COMMUNICATION_TIMEOUT_MS)) // 等待转换完成
    {
        TRACE_ERROR("\n mea time over");
        return 0;   // 如果超时
    }
    // 读取数据
    OpticalADCollect_ADS131SendByte(ADS131_CMD_READ_SINGLE);
    data = OpticalADCollect_ADS131Read24bitData();
    ADS131_CS_HIGH();   // 失能
    return (data & 0xFFFFFF);  // 24bit
}

void OpticalADCollect_ChangePolar(unsigned char polar)
{
    s_ADS131PolarMode = polar;
}
