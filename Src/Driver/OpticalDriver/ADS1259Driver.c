/*
 * ADS1259Driver.c
 *
 *  Created on: 2022年2月11日
 *      Author: lwq
 */
#include <OpticalDriver/ADS1259Driver.h>
#include "Tracer/Trace.h"
#include "Driver/System.h"
#include "FreeRTOS.h"
#include "task.h"
#include "string.h"
#include "math.h"

// **************** 命令 ****************
#define ADS1259_CMD_WAKEUP      0x03        //
#define ADS1259_CMD_SLEEP       0x05        //
//#define ADS1259_CMD_SYNC        0x05        //
#define ADS1259_CMD_RESET       0x07        //
#define ADS1259_CMD_NOP         0xFF        //
#define ADS1259_CMD_RDATA       0x12        //
#define ADS1259_CMD_RDATAC      0x10        //
#define ADS1259_CMD_SDATAC      0x11        //
#define ADS1259_CMD_RREG        0x20        //
#define ADS1259_CMD_WREG        0x40        //
//#define ADS1259_CMD_SYSOCAL     0x60        //
//#define ADS1259_CMD_SYSGCAL     0x61        //
#define ADS1259_CMD_SELFOCAL    0x18        //
#define ADS1259_CMD_RESTRICTED  0xF1        //
#define ADS1259_CMD_GANCAL    	0x19        //

// **************** 地址 ****************
//#define ADS1259_REG_ADDR_BCS    0x00
//#define ADS1259_REG_ADDR_VBIAS  0x01
//#define ADS1259_REG_ADDR_MUX1   0x02
//#define ADS1259_REG_ADDR_SYS0   0x03
//#define ADS1259_REG_ADDR_OFC0   0x04
//#define ADS1259_REG_ADDR_OFC1   0x05
//#define ADS1259_REG_ADDR_OFC2   0x06
//#define ADS1259_REG_ADDR_FSC0   0x07
//#define ADS1259_REG_ADDR_FSC1   0x08
//#define ADS1259_REG_ADDR_FSC2   0x09
//#define ADS1259_REG_ADDR_ID     0x0A

#define ADS1259_REG_ADDR_CONFIG0  0x00
#define ADS1259_REG_ADDR_CONFIG1  0x01
#define ADS1259_REG_ADDR_CONFIG2  0x02

// **************** BSC ****************
#define BCS0                    (1 << 6)
#define BCS1                    (1 << 7)

// **************** VBIAS ****************
#define VBIAS0                  (1 << 0)
#define VBIAS1                  (1 << 1)

// **************** MUXCAL ****************
#define DEFAULT_CAL             0x00
#define OFFSET_CAL              0x01
#define GAIN_CAL                0x02
#define TEMP_CAL                0x03

// **************** SYS0 ****************
// ######## GAIN ########
#define GAIN_1                  0x00
#define GAIN_2                  0x10
#define GAIN_4                  0x20
#define GAIN_8                  0x30
#define GAIN_16                 0x40
#define GAIN_32                 0x50
#define GAIN_64                 0x60
#define GAIN_128                0x70
// ######## RATE ########
#define RATE_10                 0x00
#define RATE_16                 0x01
#define RATE_50                 0x02
#define RATE_60                 0x03
#define RATE_400                0x04
#define RATE_1200               0x05
#define RATE_3600               0x06
#define RATE_14400              0x07

#define REF_INTERNAL            0x04
#define RBIAS_DISENABLE         0x85

#define ADS1259_COMMUNICATION_TIMEOUT_MS  8             // 等待转换完成超时时间

#define SAMPLE_SYNC_TIMEOUT_MS            20           // 等待采样同步信号超时时间

// CS_MUTEX
//#define ADS1259_MUTEX_PIN           GPIO_Pin_2
//#define ADS1259_MUTEX_GPIO          GPIOE
//#define ADS1259_MUTEX_RCC           RCC_AHB1Periph_GPIOE
//#define ADS1259_MUTEX_HIGH()        GPIO_SetBits(ADS1259_MUTEX_GPIO, ADS1259_MUTEX_PIN)
//#define ADS1259_MUTEX_LOW()         GPIO_ResetBits(ADS1259_MUTEX_GPIO, ADS1259_MUTEX_PIN)

 //RESET
#define ADS1259_RESET_PIN           GPIO_Pin_12
#define ADS1259_RESET_GPIO          GPIOE
#define ADS1259_RESET_RCC           RCC_AHB1Periph_GPIOE
#define ADS1259_RESET_HIGH()        GPIO_SetBits(ADS1259_RESET_GPIO, ADS1259_RESET_PIN)
#define ADS1259_RESET_LOW()         GPIO_ResetBits(ADS1259_RESET_GPIO, ADS1259_RESET_PIN)

// SCLK
#define ADS1259_SCLK_PIN         GPIO_Pin_2
#define ADS1259_SCLK_GPIO        GPIOE
#define ADS1259_SCLK_RCC         RCC_AHB1Periph_GPIOE
#define ADS1259_SCLK_HIGH()      GPIO_SetBits(ADS1259_SCLK_GPIO, ADS1259_SCLK_PIN)
#define ADS1259_SCLK_LOW()       GPIO_ResetBits(ADS1259_SCLK_GPIO, ADS1259_SCLK_PIN)

// DIN
#define ADS1259_DIN_PIN          GPIO_Pin_6
#define ADS1259_DIN_GPIO         GPIOE
#define ADS1259_DIN_RCC          RCC_AHB1Periph_GPIOE
#define ADS1259_DIN_HIGH()       GPIO_SetBits(ADS1259_DIN_GPIO, ADS1259_DIN_PIN)
#define ADS1259_DIN_LOW()        GPIO_ResetBits(ADS1259_DIN_GPIO, ADS1259_DIN_PIN)

// DOUT
#define ADS1259_DOUT_PIN         GPIO_Pin_5
#define ADS1259_DOUT_GPIO        GPIOE
#define ADS1259_DOUT_RCC         RCC_AHB1Periph_GPIOE
#define ADS1259_DOUT_READ()      GPIO_ReadInputDataBit(ADS1259_DOUT_GPIO, ADS1259_DOUT_PIN)

// START
#define ADS1259_START_PIN        GPIO_Pin_8
#define ADS1259_START_GPIO       GPIOC
#define ADS1259_START_RCC        RCC_AHB1Periph_GPIOC
#define ADS1259_START_HIGH()     GPIO_SetBits(ADS1259_START_GPIO, ADS1259_START_PIN)
#define ADS1259_START_LOW()      GPIO_ResetBits(ADS1259_START_GPIO, ADS1259_START_PIN)

// DRDY
#define ADS1259_DRDY_PIN         GPIO_Pin_4
#define ADS1259_DRDY_GPIO        GPIOE
#define ADS1259_DRDY_RCC         RCC_AHB1Periph_GPIOE
#define ADS1259_DRDY_READ()      GPIO_ReadInputDataBit(ADS1259_DRDY_GPIO, ADS1259_DRDY_PIN)

/**
 * @brief ADS1259发送字节
 * @param
 */
static uint8_t ADS1259Driver_ADS1259SendByte(uint8_t senddata)
{
    uint8_t i = 0;
    uint8_t receivedata = 0;

    for (i = 0; i < 8; i++)
    {
        // 发送
        ADS1259_SCLK_HIGH();
        if (senddata & 0x80)
        {
            ADS1259_DIN_HIGH();
//            Printf("\n@");
        }
        else
        {
            ADS1259_DIN_LOW();
//            Printf("\n*");
        }
        senddata <<= 1;

        ADS1259_SCLK_LOW();

        // 接收
        receivedata <<= 1;
        if (ADS1259_DOUT_READ())
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
void ADS1259Driver_Init(void)
{
    GPIO_InitTypeDef GPIO_InitStructure;

    //时钟配置
    RCC_AHB1PeriphClockCmd(
            ADS1259_SCLK_RCC |
            ADS1259_DIN_RCC |
            ADS1259_DOUT_RCC |
            ADS1259_START_RCC |
            ADS1259_DRDY_RCC , ENABLE);

    //IO配置
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
    GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;

    // CS_MUTEX
//    GPIO_InitStructure.GPIO_Pin = ADS1259_MUTEX_PIN;
//    GPIO_Init(ADS1259_MUTEX_GPIO, &GPIO_InitStructure);

    // RESET
    GPIO_InitStructure.GPIO_Pin = ADS1259_RESET_PIN;
    GPIO_Init(ADS1259_RESET_GPIO, &GPIO_InitStructure);

    // SCLK
    GPIO_InitStructure.GPIO_Pin = ADS1259_SCLK_PIN;
    GPIO_Init(ADS1259_SCLK_GPIO, &GPIO_InitStructure);

    // DIN
    GPIO_InitStructure.GPIO_Pin = ADS1259_DIN_PIN;
    GPIO_Init(ADS1259_DIN_GPIO, &GPIO_InitStructure);
    // START
    GPIO_InitStructure.GPIO_Pin = ADS1259_START_PIN;
    GPIO_Init(ADS1259_START_GPIO, &GPIO_InitStructure);

    // DOUT
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN;
    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
    GPIO_InitStructure.GPIO_Pin = ADS1259_DOUT_PIN;
    GPIO_Init(ADS1259_DOUT_GPIO, &GPIO_InitStructure);

    // DRDY
    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
    GPIO_InitStructure.GPIO_Pin = ADS1259_DRDY_PIN;
    GPIO_Init(ADS1259_DRDY_GPIO, &GPIO_InitStructure);

    //禁用冲突片选
//    ADS1259_MUTEX_HIGH();

    // 初始引脚
    ADS1259_RESET_LOW();
    ADS1259_SCLK_HIGH();
    ADS1259_DIN_HIGH();
    ADS1259_START_LOW();

    // SPI接口初始化
//    ADS1259_CS_HIGH();       // 失能
    System_NonOSDelay(1);
//    ADS1259_CS_LOW();        // 使能

    ADS1259_START_HIGH();

    // 复位
    ADS1259Driver_ADS1259SendByte(ADS1259_CMD_RESET);
    System_NonOSDelay(1);//1ms

    ADS1259_RESET_HIGH();

    // 等待 至少0.6ms
    System_NonOSDelay(1);//1ms

    // 设置增益和转换频率
    ADS1259Driver_ADS1259SendByte(ADS1259_CMD_WREG | ADS1259_REG_ADDR_CONFIG2); //
    ADS1259Driver_ADS1259SendByte(0x00);
    ADS1259Driver_ADS1259SendByte(RATE_400);

    ADS1259Driver_ADS1259SendByte(ADS1259_CMD_WREG | ADS1259_REG_ADDR_CONFIG1); //
    ADS1259Driver_ADS1259SendByte(0x00);
    ADS1259Driver_ADS1259SendByte(REF_INTERNAL);

    ADS1259Driver_ADS1259SendByte(ADS1259_CMD_WREG | ADS1259_REG_ADDR_CONFIG0); //
    ADS1259Driver_ADS1259SendByte(0x00);
    ADS1259Driver_ADS1259SendByte(RBIAS_DISENABLE);

    // 自动校准
//    ADS1259Driver_ADS1259SendByte(ADS1259_CMD_SELFOCAL);
    System_NonOSDelay(20);//1ms
//    ADS1259Driver_ADS1259SendByte(ADS1259_CMD_GANCAL);
//    System_Delay(20);//1ms


    ADS1259_START_LOW();

//    ADS1259_CS_HIGH();       // 失能

    TRACE_DEBUG("\n ADS1259 Collect Init Over");
}
/**
 * @brief 检查ADS1259是否转换完成
 * @param
 */
static Bool ADS1259Driver_ADS1259Ready(uint32_t timeoutms)
{
    uint32_t loopms = 0;
    uint32_t looptimes = 0;
    loopms = 1;
    looptimes = timeoutms / loopms;     // 循环查询次数

    while (ADS1259_DRDY_READ())         // 等待DOUT由期间拉低
    {
    	if(FALSE == OpticalLed_GetledSyncFlag())  //灯关闭则不再进行采集
    	{
    		 return FALSE;
    	}
        System_Delay(loopms);
//    	 System_DelayUs(900);

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
 * @brief 读取ADS1259 16位数据
 * @param
 */
static uint32_t ADS1259Driver_ADS1259Read24bitData()
{
    uint32_t read_data = 0;

    // 读数据
    read_data = 0;
    read_data |= ADS1259Driver_ADS1259SendByte(ADS1259_CMD_NOP)<< 16;
    read_data |= ADS1259Driver_ADS1259SendByte(ADS1259_CMD_NOP)<< 8;
    read_data |= ADS1259Driver_ADS1259SendByte(ADS1259_CMD_NOP);


    return (read_data & 0xFFFFFF);
}

Uint32 ADS1259Driver_GetAD(void)
{
    Uint32 data = 0;

//    ADS1259_CS_LOW();    // 使能
    ADS1259_START_HIGH();
//    System_Delay(1);
    System_DelayUs(10);   //至少三个时钟周期
    ADS1259_START_LOW();        // 停止转换

    if (FALSE == ADS1259Driver_ADS1259Ready(ADS1259_COMMUNICATION_TIMEOUT_MS))
    {
        TRACE_ERROR("\n ADS1259 collect time over");
        return 0;   // 如果超时
    }
    // 读取数据
    ADS1259Driver_ADS1259SendByte(ADS1259_CMD_RDATA);
    data = ADS1259Driver_ADS1259Read24bitData();
//    TRACE_INFO("\n ADS1259 data %d", data);

    return data;
}

float ADS1259Driver_GetData(Int32 tempData)
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
		temp = (float)tData * 2.5 / 0x7FFFFF;
	}

	return temp;
}

float ADS1259Driver_GetResult(Int32 tempData)
{
	Int32 tData = 0;
	float temp = 0;

	tData = tempData;
	if(tData >= 0x800000)
	{
		tData -= 0x800000;
		temp = -(float)tData/1000000;
	}
	else
	{
		temp = (float)tData/1000000;
	}

	return temp;
}

/**
 * @brief 检查采样同步信号是否到来
 * @param
 */
static Bool ADS1259Driver_IsGetADAble(uint32_t timeoutms)
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

Uint32 ADS1259Driver_GetADWithSync(void)
{
    Uint16 result;
    if (FALSE == ADS1259Driver_IsGetADAble(SAMPLE_SYNC_TIMEOUT_MS))
    {
       TRACE_ERROR("\n sample sync time over ");
       return FALSE;
    }
    result = ADS1259Driver_GetAD();
//    collectSyncFlag = FALSE;
    return result;
}

Bool ADS1259Driver_GetSigelChannelAD(Uint32 *channel1Buff, Uint8 len, Uint32 sampleTimeOut)
{
    TickType_t sampleTimeout, sampleInitTime;
    Uint16 data   = 0;
    Uint8 dataLen = 0;
    Uint8 i       = 0;
    Uint32 *dataBuff = channel1Buff;

//****读取数据完成后自行修改，无需等待同步标志位
//    if (FALSE == ADS1259Driver_IsGetADAble(SAMPLE_SYNC_TIMEOUT_MS))
//    {
//       TRACE_ERROR("\n sample sync time over ");
//       return FALSE;
//    }


        sampleInitTime = xTaskGetTickCount();
        sampleTimeout = sampleTimeOut  + sampleInitTime;
        for (i = 0; i < len; i++)
        {
           data = ADS1259Driver_GetAD();

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


