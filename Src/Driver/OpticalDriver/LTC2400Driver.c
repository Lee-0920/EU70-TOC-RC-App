/**
 * @file
 * @brief LTC2400 AD采集驱动。
 * @details 提供AD采集功能接口。
 * @version 1.1.0
 * @author Lee
 * @date 2024-03-06
 */

#include "stddef.h"
#include "LTC2400Driver.h"
#include "Tracer/Trace.h"
#include "System.h"

//CS
#define Ltc2400_CS_PIN           GPIO_Pin_9
#define Ltc2400_CS_GPIO          GPIOC
#define Ltc2400_CS_RCC           RCC_AHB1Periph_GPIOC
#define Ltc2400_CS_HIGH()        GPIO_SetBits(Ltc2400_CS_GPIO, Ltc2400_CS_PIN)
#define Ltc2400_CS_LOW()         GPIO_ResetBits(Ltc2400_CS_GPIO, Ltc2400_CS_PIN)

// SCLK
#define Ltc2400_SCLK_PIN         GPIO_Pin_7
#define Ltc2400_SCLK_GPIO        GPIOC
#define Ltc2400_SCLK_RCC         RCC_AHB1Periph_GPIOC
#define Ltc2400_SCLK_HIGH()      GPIO_SetBits(Ltc2400_SCLK_GPIO, Ltc2400_SCLK_PIN)
#define Ltc2400_SCLK_LOW()       GPIO_ResetBits(Ltc2400_SCLK_GPIO, Ltc2400_SCLK_PIN)

// DOUT
#define Ltc2400_DOUT_PIN         GPIO_Pin_8
#define Ltc2400_DOUT_GPIO        GPIOC
#define Ltc2400_DOUT_RCC         RCC_AHB1Periph_GPIOC
#define Ltc2400_DOUT_READ()      GPIO_ReadInputDataBit(Ltc2400_DOUT_GPIO, Ltc2400_DOUT_PIN)

// F0 晶振选择引脚
#define Ltc2400_OS_PIN           GPIO_Pin_6
#define Ltc2400_OS_GPIO          GPIOC
#define Ltc2400_OS_RCC           RCC_AHB1Periph_GPIOC
#define Ltc2400_OS_HIGH()        GPIO_SetBits(Ltc2400_OS_GPIO, Ltc2400_OS_PIN)
#define Ltc2400_OS_LOW()         GPIO_ResetBits(Ltc2400_OS_GPIO, Ltc2400_OS_PIN)

#define LTC2400_OUTPUT_SIGNBIT 0x20000000
#define LTC2400_OUTPUT_EXRNBIT 0x10000000
#define LTC2400_OUTPUT_DATABITS 0xFFFFFF0
#define LTC2400_DIGITAL_RANGE 0xFFFFFF
#define LTC2400_DIGITAL_ZERO 0x0

///*定义接收数据函数指针类型*/
//typedef void (*LTC2400Receive)(uint8_t *rData);
///*定义片选函数指针类型*/
//typedef void (*LTC2400ChipSelect)(LTC2400CSType cs);
///*定义延时函数指针类型*/
//typedef void (*LTC2400Delay)(volatile uint32_t nTime);

/*定义接收数据函数*/
static void LTC2400Receive(uint8_t *rData);
/*定义片选函数*/
static void LTC2400ChipSelect(LTC2400CSType cs);
/*定义延时函数*/
static void LTC2400Delay(volatile uint32_t nTime);

/*LTC2400浮点数换算*/
static float CompoundLTC2400Data(Ltc2400ObjectType *ltc,uint8_t *dataCode);

void Ltc2400_Delay(uint8_t time)
{
	while(time--);
}
/**
 * @brief Ltc2400发送字节
 * @param
 */
static uint8_t Ltc2400ReadByte(void)
{
    uint8_t i = 0;
    uint8_t receivedata = 0;
    for (i = 0; i < 8; i++)
    {

        Ltc2400_SCLK_LOW();
        // 发送
        Ltc2400_Delay(10);
		Ltc2400_SCLK_HIGH();

        // 接收
		receivedata <<= 1;
		if (Ltc2400_DOUT_READ())
		{
			receivedata |= 0x01;
		}
    }
//    Printf("\n**********%d",receivedata);

    return receivedata;
}

static void LTC2400Receive(uint8_t *rData)
{
    // 读32位数据
	// 发送
	rData[0] |= Ltc2400ReadByte();
	rData[1] |= Ltc2400ReadByte();
	rData[2] |= Ltc2400ReadByte();
	rData[3] |= Ltc2400ReadByte();
//	Ltc2400_SCLK_HIGH();
}

void Ltc2400ClockSelect(LTC2400ClockType clock)
{
	GPIO_InitTypeDef GPIO_InitStructure;

	//时钟配置
	RCC_AHB1PeriphClockCmd(Ltc2400_OS_RCC, ENABLE);

	//IO配置
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;

	// OS
	GPIO_InitStructure.GPIO_Pin = Ltc2400_OS_PIN;
	GPIO_Init(Ltc2400_OS_GPIO, &GPIO_InitStructure);

	if(INTERNAL_CLOCK50Hz == clock)
	{
		Ltc2400_OS_HIGH();
		TRACE_INFO("\n LTC2400 Select: Internal 50Hz");
	}
	else if(INTERNAL_CLOCK60Hz == clock)
	{
		Ltc2400_OS_LOW();
		TRACE_INFO("\n LTC2400 Select: Internal 60Hz");
	}
	else
	{
		TRACE_INFO("\n LTC2400 Select: Invalid Param");
	}
}

/**
 * @brief 光学采集驱动初始化
 * @param
 */
void Ltc2400Driver_Init(void)
{
    GPIO_InitTypeDef GPIO_InitStructure;

    //时钟配置
    RCC_AHB1PeriphClockCmd(Ltc2400_CS_RCC | Ltc2400_SCLK_RCC | Ltc2400_DOUT_RCC, ENABLE);

    //IO配置
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
    GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;

    // CS
    GPIO_InitStructure.GPIO_Pin = Ltc2400_CS_PIN;
    GPIO_Init(Ltc2400_CS_GPIO, &GPIO_InitStructure);

    // SCLK
    GPIO_InitStructure.GPIO_Pin = Ltc2400_SCLK_PIN;
    GPIO_Init(Ltc2400_SCLK_GPIO, &GPIO_InitStructure);

    // DOUT
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN;
    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
    GPIO_InitStructure.GPIO_Pin = Ltc2400_DOUT_PIN;
    GPIO_Init(Ltc2400_DOUT_GPIO, &GPIO_InitStructure);

    // 初始引脚
    Ltc2400_SCLK_HIGH();
    Ltc2400_CS_HIGH();

    TRACE_INFO("\n Ltc2400 Collect Init Over");
}

static Bool Ltc2400Driver_TestEOC(uint32_t timeoutus)
{
    uint32_t loopms = 0;
    uint32_t looptimes = 0;
    loopms = 1;
    looptimes = timeoutus / loopms;     // 循环查询次数

    while(Ltc2400_DOUT_READ())         // 等待DOUT由期间拉低
    {
    	System_DelayUs(loopms);
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

/* 数据采集 */
float GetLtc2400Data(Ltc2400ObjectType *ltc)
{
  uint8_t rData[4] = 0;
  
  ltc->ChipSelect(LTC2400CS_Enable);
  ltc->Delayms(1);
  
//  //EOC检测
//  if(!Ltc2400Driver_TestEOC(20))
//  {
//	  return 0;
//  }
//  ltc->ChipSelect(LTC2400CS_Disable);
//  System_DelayUs(1);
//  ltc->ChipSelect(LTC2400CS_Enable);

  ltc->Receive(rData);
  
  ltc->Delayms(1);
  ltc->ChipSelect(LTC2400CS_Disable);
  
  return CompoundLTC2400Data(ltc,rData);
}

/*LTC2400浮点数换算*/
static float CompoundLTC2400Data(Ltc2400ObjectType *ltc,uint8_t *dataCode)
{
  uint32_t temp = 0;
  float result = 0.0;
  
  temp = (dataCode[0]<<24)+(dataCode[1]<<16)+(dataCode[2]<<8)+dataCode[3];
  
  ltc->dataCode = temp;

  result = (float)(((((temp&LTC2400_OUTPUT_DATABITS)>>4)-LTC2400_DIGITAL_ZERO) * 2.483 / 16777215.0));
//  TRACE_INFO("\n ltc2400 0x%x, %d, %f", temp, (temp&LTC2400_OUTPUT_DATABITS)>>4, result);
//  if((temp&LTC2400_OUTPUT_SIGNBIT)!=LTC2400_OUTPUT_SIGNBIT)//数据正常
//  {
////	  Printf("\n ");
////    result=result-1.0;
//  }
//  else
//  {
//    if((temp&LTC2400_OUTPUT_EXRNBIT)!=LTC2400_OUTPUT_EXRNBIT)
//    {
//      Printf("\n Out of Range");
//      result = result + 0.1;
//    }
//  }
  
  return result;
}

/* LTC2400初始化*/
void LTC2400Initialization(Ltc2400ObjectType *ltc, LTC2400ClockType clock)
{
//  if((ltc==NULL)||(receive==NULL)||(msDelay==NULL))
//  {
//    return;
//  }
	
  ltc->dataCode = 0;
  ltc->clock = clock;
  ltc->ChipSelect = LTC2400ChipSelect;
  ltc->Receive = LTC2400Receive;
  ltc->Delayms = LTC2400Delay;
  Ltc2400ClockSelect(INTERNAL_CLOCK50Hz);
  Ltc2400Driver_Init();
}

void LTC2400ChipSelect(LTC2400CSType cs)
{
	if(LTC2400CS_Enable == cs)
	{
		 Ltc2400_CS_LOW();
	}
	else
	{
		 Ltc2400_CS_HIGH();
	}
}

void LTC2400Delay(volatile uint32_t nTime)
{
	System_Delay(nTime);
}

