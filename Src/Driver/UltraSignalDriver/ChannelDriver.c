
/*
 * ChannelDriver.c
 *
 *  Created on: 2016年11月09日
 *      Author: Liang
 */
#include <UltraSignalDriver/ChannelDriver.h>
#include "Tracer/Trace.h"
#include "Driver/System.h"
#include "SystemConfig.h"
#include <TempDriver/ThermostatDeviceDriver.h>

#define ADG708_EN_PIN                         GPIO_Pin_3
#define ADG708_EN_PORT                        GPIOA
#define ADG708_EN_RCC                  		  RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE)
#define ADG708_EN_HIGH()					  GPIO_SetBits(ADG708_EN_PORT, ADG708_EN_PIN);
#define ADG708_EN_LOW()						  GPIO_ResetBits(ADG708_EN_PORT, ADG708_EN_PIN);

#define ADG708_A0_PIN                         GPIO_Pin_1
#define ADG708_A0_PORT                        GPIOD
#define ADG708_A0_RCC                	      RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOD, ENABLE)
#define ADG708_A0_HIGH()					  GPIO_SetBits(ADG708_A0_PORT, ADG708_A0_PIN);
#define ADG708_A0_LOW()					      GPIO_ResetBits(ADG708_A0_PORT, ADG708_A0_PIN);

#define ADG708_A1_PIN                         GPIO_Pin_2
#define ADG708_A1_PORT                     	  GPIOD
#define ADG708_A1_RCC              	          RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOD, ENABLE)
#define ADG708_A1_HIGH()					  GPIO_SetBits(ADG708_A1_PORT, ADG708_A1_PIN);
#define ADG708_A1_LOW()				 		  GPIO_ResetBits(ADG708_A1_PORT, ADG708_A1_PIN);

#define ADG708_A2_PIN                         GPIO_Pin_3
#define ADG708_A2_PORT                     	  GPIOD
#define ADG708_A2_RCC              	          RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOD, ENABLE)
#define ADG708_A2_HIGH()					  GPIO_SetBits(ADG708_A2_PORT, ADG708_A2_PIN);
#define ADG708_A2_LOW()				 		  GPIO_ResetBits(ADG708_A2_PORT, ADG708_A2_PIN);

#define GAIN1_PIN                             GPIO_Pin_6
#define GAIN1_PORT                     	      GPIOD
#define GAIN1_RCC              	              RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOD, ENABLE)
#define GAIN1_HIGH()					      GPIO_SetBits(GAIN1_PORT, GAIN1_PIN);
#define GAIN1_LOW()				 		      GPIO_ResetBits(GAIN1_PORT, GAIN1_PIN);

#define GAIN2_PIN                             GPIO_Pin_7
#define GAIN2_PORT                     	      GPIOD
#define GAIN2_RCC              	              RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOD, ENABLE)
#define GAIN2_HIGH()					      GPIO_SetBits(GAIN2_PORT, GAIN2_PIN);
#define GAIN2_LOW()				 		      GPIO_ResetBits(GAIN2_PORT, GAIN2_PIN);

#define GAIN3_PIN                             GPIO_Pin_11
#define GAIN3_PORT                     	      GPIOC
#define GAIN3_RCC              	              RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOC, ENABLE)
#define GAIN3_HIGH()					      GPIO_SetBits(GAIN3_PORT, GAIN3_PIN);
#define GAIN3_LOW()				 		      GPIO_ResetBits(GAIN3_PORT, GAIN3_PIN);

#define GAIN4_PIN                             GPIO_Pin_12
#define GAIN4_PORT                     	      GPIOC
#define GAIN4_RCC              	              RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOC, ENABLE)
#define GAIN4_HIGH()					      GPIO_SetBits(GAIN4_PORT, GAIN4_PIN);
#define GAIN4_LOW()				 		      GPIO_ResetBits(GAIN4_PORT, GAIN4_PIN);

#define MEA_CALIBRATION_PIN                   GPIO_Pin_15
#define MEA_CALIBRATION_PORT                  GPIOD
#define MEA_CALIBRATION_RCC              	  RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOD, ENABLE)
#define MEA_CALIBRATION_HIGH()				  GPIO_SetBits(MEA_CALIBRATION_PORT, MEA_CALIBRATION_PIN);
#define MEA_CALIBRATION_LOW()				  GPIO_ResetBits(MEA_CALIBRATION_PORT, MEA_CALIBRATION_PIN);

#define TIMER_4KHZ  (9-1)
#define TIMER_2KHZ  (18-1)

typedef enum
{
	TIMER_IDLE,
	TIMER_ON,
	TIMER_OFF,
}TimerState;

typedef struct
{
    GPIO_TypeDef * port;
    uint32_t pin;
    uint32_t gpioRcc;
    ModeConfig modeConfig;
    uint16_t compareValue;
}ChannelDeviceDriver;

void ChannelDeviceDriver_Init(ChannelDeviceDriver *deviceDriver)
{
	GPIO_InitTypeDef GPIO_InitStructure;
	RCC_AHB1PeriphClockCmd(deviceDriver->gpioRcc, ENABLE);
	GPIO_InitStructure.GPIO_Pin = deviceDriver->pin;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
	GPIO_Init(deviceDriver->port, &GPIO_InitStructure);

	GPIO_PinAFConfig(deviceDriver->port, deviceDriver->modeConfig.PWMConfig.pinSource, deviceDriver->modeConfig.PWMConfig.goipAF);

//	if (FALSE == ChannelDeviceDriver_IsAlreadyInitTimer(deviceDriver->modeConfig.PWMConfig.timer))
	{
		TIM_TimeBaseInitTypeDef TIM_TimeBaseStructure;

		deviceDriver->modeConfig.PWMConfig.timerRccInitFunction(deviceDriver->modeConfig.PWMConfig.timerRcc, ENABLE);

		TIM_TimeBaseStructure.TIM_ClockDivision = TIM_CKD_DIV1;
		TIM_TimeBaseStructure.TIM_Period = deviceDriver->modeConfig.PWMConfig.timerPeriod;
		TIM_TimeBaseStructure.TIM_Prescaler = deviceDriver->modeConfig.PWMConfig.timerPrescaler;
		TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;
		TIM_TimeBaseInit(deviceDriver->modeConfig.PWMConfig.timer, &TIM_TimeBaseStructure);
	}
	TIM_OCInitTypeDef TIM_OCInitStructure;
	TIM_OCInitStructure.TIM_OCMode = deviceDriver->modeConfig.PWMConfig.timerOCMode;
	TIM_OCInitStructure.TIM_OCPolarity = deviceDriver->modeConfig.PWMConfig.timerOCPolarity;
	TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
	///TIM_OCInitStructure.TIM_OutputNState = TIM_OutputNState_Enable;
	TIM_OCInitStructure.TIM_Pulse = 0;

	switch(deviceDriver->modeConfig.PWMConfig.timerChannel)
	{
		case 1:
			TIM_OC1Init(deviceDriver->modeConfig.PWMConfig.timer, &TIM_OCInitStructure);
			TIM_OC1PreloadConfig(deviceDriver->modeConfig.PWMConfig.timer, TIM_OCPreload_Enable);
			break;
		case 2:
			TIM_OC2Init(deviceDriver->modeConfig.PWMConfig.timer, &TIM_OCInitStructure);
			TIM_OC2PreloadConfig(deviceDriver->modeConfig.PWMConfig.timer, TIM_OCPreload_Enable);
			break;
		case 3:
			TIM_OC3Init(deviceDriver->modeConfig.PWMConfig.timer, &TIM_OCInitStructure);
			TIM_OC3PreloadConfig(deviceDriver->modeConfig.PWMConfig.timer, TIM_OCPreload_Enable);
			break;
		case 4:
			TIM_OC4Init(deviceDriver->modeConfig.PWMConfig.timer, &TIM_OCInitStructure);
			TIM_OC4PreloadConfig(deviceDriver->modeConfig.PWMConfig.timer, TIM_OCPreload_Enable);
			break;
	}
	TIM_CCxCmd(deviceDriver->modeConfig.PWMConfig.timer, deviceDriver->modeConfig.PWMConfig.timerChannel, ENABLE);

//	if (FALSE == ChannelDeviceDriver_IsAlreadyInitTimer(deviceDriver->modeConfig.PWMConfig.timer))
	{
//		ChannelDeviceDriver_AddTimer(deviceDriver->modeConfig.PWMConfig.timer);
//		TIM_ARRPreloadConfig(deviceDriver->modeConfig.PWMConfig.timer, ENABLE);
//		TIM_Cmd(deviceDriver->modeConfig.PWMConfig.timer, ENABLE);  //默认不启动
		TIM_CtrlPWMOutputs(deviceDriver->modeConfig.PWMConfig.timer, ENABLE);
		 switch(deviceDriver->modeConfig.PWMConfig.timerChannel)
		{
			case 1:
				TIM_SetCompare1(deviceDriver->modeConfig.PWMConfig.timer, (uint16_t) ((deviceDriver->compareValue)));
				break;
			case 2:
				TIM_SetCompare2(deviceDriver->modeConfig.PWMConfig.timer, (uint16_t) ((deviceDriver->compareValue)));
				break;
			case 3:
				TIM_SetCompare3(deviceDriver->modeConfig.PWMConfig.timer, (uint16_t) ((deviceDriver->compareValue)));
				break;
			case 4:
				TIM_SetCompare4(deviceDriver->modeConfig.PWMConfig.timer, (uint16_t) ((deviceDriver->compareValue)));
				break;
		}
	}
}

void ChannelDriver_MapInit(void)
{
	//TIM4参考 2KHz(参考、测量极性相同)
	ChannelDeviceDriver deviceDriver;
	deviceDriver.port = GPIOB;
	deviceDriver.pin = GPIO_Pin_6;
	deviceDriver.modeConfig.PWMConfig.pinSource = GPIO_PinSource6;
	deviceDriver.modeConfig.PWMConfig.goipAF = GPIO_AF_TIM4;
	deviceDriver.modeConfig.PWMConfig.timerRccInitFunction = RCC_APB1PeriphClockCmd;
	deviceDriver.modeConfig.PWMConfig.timerRcc = RCC_APB1Periph_TIM4;
	deviceDriver.modeConfig.PWMConfig.timerPrescaler = TIMER_4KHZ;
	deviceDriver.modeConfig.PWMConfig.timerPeriod = 4999;
	deviceDriver.modeConfig.PWMConfig.timerChannel = 1;
	deviceDriver.modeConfig.PWMConfig.timer = TIM4;
	deviceDriver.modeConfig.PWMConfig.timerOCPolarity = TIM_OCPolarity_Low;
	deviceDriver.modeConfig.PWMConfig.timerOCMode = TIM_OCMode_PWM1;//在向上计数模式下，TIMx_CNT < TIMx_CCR1时，通道1为无效电平
	deviceDriver.compareValue = 2500-1;
	ChannelDeviceDriver_Init(&deviceDriver);

	//TIM4测量 2KHz(参考、测量极性相同)
	deviceDriver.port = GPIOB;
	deviceDriver.pin = GPIO_Pin_7;
	deviceDriver.modeConfig.PWMConfig.pinSource = GPIO_PinSource7;
	deviceDriver.modeConfig.PWMConfig.goipAF = GPIO_AF_TIM4;
	deviceDriver.modeConfig.PWMConfig.timerRccInitFunction = RCC_APB1PeriphClockCmd;
	deviceDriver.modeConfig.PWMConfig.timerRcc = RCC_APB1Periph_TIM4;
	deviceDriver.modeConfig.PWMConfig.timerPrescaler = TIMER_4KHZ;
	deviceDriver.modeConfig.PWMConfig.timerPeriod = 4999;
	deviceDriver.modeConfig.PWMConfig.timerChannel = 2;
	deviceDriver.modeConfig.PWMConfig.timer = TIM4;
	deviceDriver.modeConfig.PWMConfig.timerOCPolarity = TIM_OCPolarity_Low;
	deviceDriver.modeConfig.PWMConfig.timerOCMode = TIM_OCMode_PWM1;//在向上计数模式下，TIMx_CNT < TIMx_CCR1时，通道1为无效电平
	deviceDriver.compareValue = 2500-1;
	ChannelDeviceDriver_Init(&deviceDriver);

	//TIM3 2KHz,极性与参考测量相反
	deviceDriver.port = GPIOB;
	deviceDriver.pin = GPIO_Pin_1;
	deviceDriver.modeConfig.PWMConfig.pinSource = GPIO_PinSource1;
	deviceDriver.modeConfig.PWMConfig.goipAF = GPIO_AF_TIM3;
	deviceDriver.modeConfig.PWMConfig.timerRccInitFunction = RCC_APB1PeriphClockCmd;
	deviceDriver.modeConfig.PWMConfig.timerRcc = RCC_APB1Periph_TIM3;
	deviceDriver.modeConfig.PWMConfig.timerPrescaler = TIMER_4KHZ;
	deviceDriver.modeConfig.PWMConfig.timerPeriod = 4999;
	deviceDriver.modeConfig.PWMConfig.timerChannel = 4;
	deviceDriver.modeConfig.PWMConfig.timer = TIM3;
	deviceDriver.modeConfig.PWMConfig.timerOCPolarity = TIM_OCPolarity_Low;
	deviceDriver.modeConfig.PWMConfig.timerOCMode = TIM_OCMode_PWM2;//在向上计数模式下，TIMx_CNT < TIMx_CCR1时，通道1为无效电平
	deviceDriver.compareValue = 2500-1;
	ChannelDeviceDriver_Init(&deviceDriver);

	//TIM3 2KHz,极性与参考测量相反
	deviceDriver.port = GPIOB;
	deviceDriver.pin = GPIO_Pin_0;
	deviceDriver.modeConfig.PWMConfig.pinSource = GPIO_PinSource0;
	deviceDriver.modeConfig.PWMConfig.goipAF = GPIO_AF_TIM3;
	deviceDriver.modeConfig.PWMConfig.timerRccInitFunction = RCC_APB1PeriphClockCmd;
	deviceDriver.modeConfig.PWMConfig.timerRcc = RCC_APB1Periph_TIM3;
	deviceDriver.modeConfig.PWMConfig.timerPrescaler = TIMER_4KHZ;
	deviceDriver.modeConfig.PWMConfig.timerPeriod = 4999;
	deviceDriver.modeConfig.PWMConfig.timerChannel = 3;
	deviceDriver.modeConfig.PWMConfig.timer = TIM3;
	deviceDriver.modeConfig.PWMConfig.timerOCPolarity = TIM_OCPolarity_Low;
	deviceDriver.modeConfig.PWMConfig.timerOCMode = TIM_OCMode_PWM2;//在向上计数模式下，TIMx_CNT < TIMx_CCR1时，通道1为无效电平
	deviceDriver.compareValue = 2500-1;
	ChannelDeviceDriver_Init(&deviceDriver);

	//TIM2 4KHz, PA1/2/3/5,
	//PA3 C4/C1极性PWM1, C2/C3 PWM2
	//C1 compare值1400, C2/3/4 compare值900
	deviceDriver.port = GPIOA;
	deviceDriver.pin = GPIO_Pin_1;
	deviceDriver.modeConfig.PWMConfig.pinSource = GPIO_PinSource1;
	deviceDriver.modeConfig.PWMConfig.goipAF = GPIO_AF_TIM2;
	deviceDriver.modeConfig.PWMConfig.timerRccInitFunction = RCC_APB1PeriphClockCmd;
	deviceDriver.modeConfig.PWMConfig.timerRcc = RCC_APB1Periph_TIM2;
	deviceDriver.modeConfig.PWMConfig.timerPrescaler = TIMER_4KHZ;
	deviceDriver.modeConfig.PWMConfig.timerPeriod = 2499;
	deviceDriver.modeConfig.PWMConfig.timerChannel = 2;
	deviceDriver.modeConfig.PWMConfig.timer = TIM2;
	deviceDriver.modeConfig.PWMConfig.timerOCPolarity = TIM_OCPolarity_Low;
	deviceDriver.modeConfig.PWMConfig.timerOCMode = TIM_OCMode_PWM2;//在向上计数模式下，TIMx_CNT < TIMx_CCR1时，通道1为无效电平
	deviceDriver.compareValue = 900-1;
	ChannelDeviceDriver_Init(&deviceDriver);

	deviceDriver.port = GPIOA;
	deviceDriver.pin = GPIO_Pin_2;
	deviceDriver.modeConfig.PWMConfig.pinSource = GPIO_PinSource2;
	deviceDriver.modeConfig.PWMConfig.goipAF = GPIO_AF_TIM2;
	deviceDriver.modeConfig.PWMConfig.timerRccInitFunction = RCC_APB1PeriphClockCmd;
	deviceDriver.modeConfig.PWMConfig.timerRcc = RCC_APB1Periph_TIM2;
	deviceDriver.modeConfig.PWMConfig.timerPrescaler = TIMER_4KHZ;
	deviceDriver.modeConfig.PWMConfig.timerPeriod = 2499;
	deviceDriver.modeConfig.PWMConfig.timerChannel = 3;
	deviceDriver.modeConfig.PWMConfig.timer = TIM2;
	deviceDriver.modeConfig.PWMConfig.timerOCPolarity = TIM_OCPolarity_Low;
	deviceDriver.modeConfig.PWMConfig.timerOCMode = TIM_OCMode_PWM2;//在向上计数模式下，TIMx_CNT < TIMx_CCR1时，通道1为无效电平
	deviceDriver.compareValue = 900-1;
	ChannelDeviceDriver_Init(&deviceDriver);

	//EN Pin, PWM1 compare 1400
	deviceDriver.port = GPIOA;
	deviceDriver.pin = GPIO_Pin_3;
	deviceDriver.modeConfig.PWMConfig.pinSource = GPIO_PinSource3;
	deviceDriver.modeConfig.PWMConfig.goipAF = GPIO_AF_TIM2;
	deviceDriver.modeConfig.PWMConfig.timerRccInitFunction = RCC_APB1PeriphClockCmd;
	deviceDriver.modeConfig.PWMConfig.timerRcc = RCC_APB1Periph_TIM2;
	deviceDriver.modeConfig.PWMConfig.timerPrescaler = TIMER_4KHZ;
	deviceDriver.modeConfig.PWMConfig.timerPeriod = 2499;
	deviceDriver.modeConfig.PWMConfig.timerChannel = 4;
	deviceDriver.modeConfig.PWMConfig.timer = TIM2;
	deviceDriver.modeConfig.PWMConfig.timerOCPolarity = TIM_OCPolarity_Low;
	deviceDriver.modeConfig.PWMConfig.timerOCMode = TIM_OCMode_PWM1;//在向上计数模式下，TIMx_CNT < TIMx_CCR1时，通道1为无效电平
	deviceDriver.compareValue = 900-1;
	ChannelDeviceDriver_Init(&deviceDriver);

	deviceDriver.port = GPIOA;
	deviceDriver.pin = GPIO_Pin_5;
	deviceDriver.modeConfig.PWMConfig.pinSource = GPIO_PinSource5;
	deviceDriver.modeConfig.PWMConfig.goipAF = GPIO_AF_TIM2;
	deviceDriver.modeConfig.PWMConfig.timerRccInitFunction = RCC_APB1PeriphClockCmd;
	deviceDriver.modeConfig.PWMConfig.timerRcc = RCC_APB1Periph_TIM2;
	deviceDriver.modeConfig.PWMConfig.timerPrescaler = TIMER_4KHZ;
	deviceDriver.modeConfig.PWMConfig.timerPeriod = 2499;
	deviceDriver.modeConfig.PWMConfig.timerChannel = 1;
	deviceDriver.modeConfig.PWMConfig.timer = TIM2;
	deviceDriver.modeConfig.PWMConfig.timerOCPolarity = TIM_OCPolarity_Low;
	deviceDriver.modeConfig.PWMConfig.timerOCMode = TIM_OCMode_PWM1;//在向上计数模式下，TIMx_CNT < TIMx_CCR1时，通道1为无效电平
	deviceDriver.compareValue = 1400-1;
	ChannelDeviceDriver_Init(&deviceDriver);

	/*
	 * 参考引脚
	 * */
	//EN Pin, PWM1 compare 1400
	deviceDriver.port = GPIOC;
	deviceDriver.pin = GPIO_Pin_6;
	deviceDriver.modeConfig.PWMConfig.pinSource = GPIO_PinSource6;
	deviceDriver.modeConfig.PWMConfig.goipAF = GPIO_AF_TIM8;
	deviceDriver.modeConfig.PWMConfig.timerRccInitFunction = RCC_APB2PeriphClockCmd;
	deviceDriver.modeConfig.PWMConfig.timerRcc = RCC_APB2Periph_TIM8;
	deviceDriver.modeConfig.PWMConfig.timerPrescaler = TIMER_2KHZ;
	deviceDriver.modeConfig.PWMConfig.timerPeriod = 2499;
	deviceDriver.modeConfig.PWMConfig.timerChannel = 1;
	deviceDriver.modeConfig.PWMConfig.timer = TIM8;
	deviceDriver.modeConfig.PWMConfig.timerOCPolarity = TIM_OCPolarity_Low;
	deviceDriver.modeConfig.PWMConfig.timerOCMode = TIM_OCMode_PWM1;//在向上计数模式下，TIMx_CNT < TIMx_CCR1时，通道1为无效电平
	deviceDriver.compareValue = 900-1;
	ChannelDeviceDriver_Init(&deviceDriver);

	deviceDriver.port = GPIOC;
	deviceDriver.pin = GPIO_Pin_7;
	deviceDriver.modeConfig.PWMConfig.pinSource = GPIO_PinSource7;
	deviceDriver.modeConfig.PWMConfig.goipAF = GPIO_AF_TIM8;
	deviceDriver.modeConfig.PWMConfig.timerRccInitFunction = RCC_APB2PeriphClockCmd;
	deviceDriver.modeConfig.PWMConfig.timerRcc = RCC_APB2Periph_TIM8;
	deviceDriver.modeConfig.PWMConfig.timerPrescaler = TIMER_2KHZ;
	deviceDriver.modeConfig.PWMConfig.timerPeriod = 2499;
	deviceDriver.modeConfig.PWMConfig.timerChannel = 2;
	deviceDriver.modeConfig.PWMConfig.timer = TIM8;
	deviceDriver.modeConfig.PWMConfig.timerOCPolarity = TIM_OCPolarity_Low;
	deviceDriver.modeConfig.PWMConfig.timerOCMode = TIM_OCMode_PWM1;//在向上计数模式下，TIMx_CNT < TIMx_CCR1时，通道1为无效电平
	deviceDriver.compareValue = 1400-1;
	ChannelDeviceDriver_Init(&deviceDriver);

	//*Timer Start*//
	TIM_Cmd(TIM2, ENABLE);//4K
	TIM_Cmd(TIM8, ENABLE);//4K
	System_DelayUs(15);
	TIM_Cmd(TIM3, ENABLE);//2K
	TIM_Cmd(TIM4, ENABLE);//2K

}


/**
 *
void ChannelPWM_Init(void)
{
	//REF_MOS_C MEA_MOS_C
	//参考和测量，频率均为2KHz,但极性相反
	GPIO_InitTypeDef GPIO_InitStructure;
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB,ENABLE);
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_6;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
	GPIO_Init(GPIOB, &GPIO_InitStructure);

	GPIO_PinAFConfig(GPIOB, GPIO_PinSource6, GPIO_AF_TIM4);

	TIM_TimeBaseInitTypeDef TIM_TimeBaseStructure;

	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM4, ENABLE);

	TIM_TimeBaseStructure.TIM_ClockDivision = TIM_CKD_DIV1;
	TIM_TimeBaseStructure.TIM_Period = 4999;
	TIM_TimeBaseStructure.TIM_Prescaler = 9-1;
	TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;
	TIM_TimeBaseInit(TIM4, &TIM_TimeBaseStructure);

	TIM_OCInitTypeDef TIM_OCInitStructure;
	TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1;
	TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_Low;
	TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
	TIM_OCInitStructure.TIM_Pulse = 0;

	TIM_OC1Init(TIM4, &TIM_OCInitStructure);
	TIM_OC1PreloadConfig(TIM4, TIM_OCPreload_Enable);
	TIM_CCxCmd(TIM4, 1, ENABLE);
	TIM_SetCompare1(TIM4, 2500-1);

	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB,ENABLE);
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_7;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
	GPIO_Init(GPIOB, &GPIO_InitStructure);

	GPIO_PinAFConfig(GPIOB, GPIO_PinSource7, GPIO_AF_TIM4);

	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM4, ENABLE);

	TIM_TimeBaseStructure.TIM_ClockDivision = TIM_CKD_DIV1;
	TIM_TimeBaseStructure.TIM_Period = 4999;
	TIM_TimeBaseStructure.TIM_Prescaler = 9-1;
	TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;
	TIM_TimeBaseInit(TIM4, &TIM_TimeBaseStructure);

	TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1;
	TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_Low;
	TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
	TIM_OCInitStructure.TIM_Pulse = 0;

	TIM_OC2Init(TIM4, &TIM_OCInitStructure);
	TIM_OC2PreloadConfig(TIM4, TIM_OCPreload_Enable);
	TIM_CCxCmd(TIM4, 2, ENABLE);
	TIM_SetCompare2(TIM4, 2500-1);
	//REF_MOS_C MEA_MOS_C  End
	///////2KHz TIM4 C2 C1 end//////////

	///////2k TIM3 C2 start
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB,ENABLE);
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_5;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
	GPIO_Init(GPIOB, &GPIO_InitStructure);

	GPIO_PinAFConfig(GPIOB, GPIO_PinSource5, GPIO_AF_TIM3);

	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM3, ENABLE);

	TIM_TimeBaseStructure.TIM_ClockDivision = TIM_CKD_DIV1;
	TIM_TimeBaseStructure.TIM_Period = 4999;
	TIM_TimeBaseStructure.TIM_Prescaler = 9-1;
	TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;
	TIM_TimeBaseInit(TIM3, &TIM_TimeBaseStructure);

	TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM2;
	TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_Low;
	TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
	TIM_OCInitStructure.TIM_Pulse = 0;

	TIM_OC2Init(TIM3, &TIM_OCInitStructure);
	TIM_OC2PreloadConfig(TIM3, TIM_OCPreload_Enable);
	TIM_CCxCmd(TIM3, 2, ENABLE);
	TIM_SetCompare2(TIM3, 2500-1);
	///////2KHz TIM3 C2 end//////////

	/////2k TIM3-C3 start
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB,ENABLE);
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
	GPIO_Init(GPIOB, &GPIO_InitStructure);

	GPIO_PinAFConfig(GPIOB, GPIO_PinSource0, GPIO_AF_TIM3);

	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM3, ENABLE);

	TIM_TimeBaseStructure.TIM_ClockDivision = TIM_CKD_DIV1;
	TIM_TimeBaseStructure.TIM_Period = 4999;
	TIM_TimeBaseStructure.TIM_Prescaler = 9-1;
	TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;
	TIM_TimeBaseInit(TIM3, &TIM_TimeBaseStructure);

	TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM2;
	TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_Low;
	TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
	TIM_OCInitStructure.TIM_Pulse = 0;

	TIM_OC3Init(TIM3, &TIM_OCInitStructure);
	TIM_OC3PreloadConfig(TIM3, TIM_OCPreload_Enable);
	TIM_CCxCmd(TIM3, 3, ENABLE);
	TIM_SetCompare3(TIM3, 2500-1);
	///////2KHz TIM3 end//////////

	///4k cond
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA,ENABLE);
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_1 | GPIO_Pin_2 | GPIO_Pin_3 | GPIO_Pin_5;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
	GPIO_Init(GPIOA, &GPIO_InitStructure);

	GPIO_PinAFConfig(GPIOA, GPIO_PinSource1, GPIO_AF_TIM2);
	GPIO_PinAFConfig(GPIOA, GPIO_PinSource2, GPIO_AF_TIM2);
	GPIO_PinAFConfig(GPIOA, GPIO_PinSource3, GPIO_AF_TIM2);
	GPIO_PinAFConfig(GPIOA, GPIO_PinSource5, GPIO_AF_TIM2);

	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2, ENABLE);

	TIM_TimeBaseStructure.TIM_ClockDivision = TIM_CKD_DIV1;
	TIM_TimeBaseStructure.TIM_Period = 2499;
	TIM_TimeBaseStructure.TIM_Prescaler = 9-1;
	TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;
	TIM_TimeBaseInit(TIM2, &TIM_TimeBaseStructure);

	TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM2;
	TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_Low;
	TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
	TIM_OCInitStructure.TIM_Pulse = 0;

	TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM2;
	TIM_OC2Init(TIM2, &TIM_OCInitStructure);
	TIM_OC2PreloadConfig(TIM2, TIM_OCPreload_Enable);
	TIM_CCxCmd(TIM2, 2, ENABLE);
	TIM_SetCompare2(TIM2, 900-1);

	TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM2;
	TIM_OC3Init(TIM2, &TIM_OCInitStructure);
	TIM_OC3PreloadConfig(TIM2, TIM_OCPreload_Enable);
	TIM_CCxCmd(TIM2, 3, ENABLE);
	TIM_SetCompare3(TIM2, 900-1);

	TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1;
	TIM_OC4Init(TIM2, &TIM_OCInitStructure);
	TIM_OC4PreloadConfig(TIM2, TIM_OCPreload_Enable);
	TIM_CCxCmd(TIM2, 4, ENABLE);
	TIM_SetCompare4(TIM2, 900-1);

	TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1;
	TIM_OC1Init(TIM2, &TIM_OCInitStructure);
	TIM_OC1PreloadConfig(TIM2, TIM_OCPreload_Enable);
	TIM_CCxCmd(TIM2, 1, ENABLE);
	TIM_SetCompare1(TIM2, 1400-1);

	////4k cond end

	//Timer Start
	TIM_Cmd(TIM2, ENABLE);//4K
	System_DelayUs(15);
	TIM_Cmd(TIM3, ENABLE);//2K
	TIM_Cmd(TIM4, ENABLE);//2K

}
  */

void ChannelDriver_Init(void)
{
    GPIO_InitTypeDef GPIO_InitStructure;

    ADG708_EN_RCC;
    ADG708_A0_RCC;
    ADG708_A1_RCC;
    ADG708_A2_RCC;
    GAIN1_RCC;
    GAIN2_RCC;
    GAIN3_RCC;
    GAIN4_RCC;
    MEA_CALIBRATION_RCC;

    GPIO_InitStructure.GPIO_Pin=ADG708_EN_PIN;
    GPIO_InitStructure.GPIO_Mode=GPIO_Mode_OUT;
    GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_DOWN;
    GPIO_InitStructure.GPIO_Speed=GPIO_Speed_50MHz;
    GPIO_Init(ADG708_EN_PORT,&GPIO_InitStructure);
    ADG708_EN_HIGH();

    GPIO_InitStructure.GPIO_Pin=ADG708_A0_PIN;
	GPIO_Init(ADG708_A0_PORT,&GPIO_InitStructure);
	ADG708_A0_LOW();

	GPIO_InitStructure.GPIO_Pin=ADG708_A1_PIN;
	GPIO_Init(ADG708_A1_PORT,&GPIO_InitStructure);
	ADG708_A1_HIGH();

	GPIO_InitStructure.GPIO_Pin=ADG708_A2_PIN;
	GPIO_Init(ADG708_A2_PORT,&GPIO_InitStructure);
	ADG708_A2_LOW();

	GPIO_InitStructure.GPIO_Pin=GAIN1_PIN;
	GPIO_Init(GAIN1_PORT,&GPIO_InitStructure);
	GAIN1_HIGH();

	GPIO_InitStructure.GPIO_Pin=GAIN2_PIN;
	GPIO_Init(GAIN2_PORT,&GPIO_InitStructure);
	GAIN2_LOW();

	GPIO_InitStructure.GPIO_Pin=GAIN3_PIN;
	GPIO_Init(GAIN3_PORT,&GPIO_InitStructure);
	GAIN3_LOW();

	GPIO_InitStructure.GPIO_Pin=GAIN4_PIN;
	GPIO_Init(GAIN4_PORT,&GPIO_InitStructure);
	GAIN4_LOW();

	GPIO_InitStructure.GPIO_Pin=MEA_CALIBRATION_PIN;
	GPIO_Init(MEA_CALIBRATION_PORT,&GPIO_InitStructure);
	MEA_CALIBRATION_HIGH();

//	ChannelPWM_Init();
	ChannelDriver_SetGainMap(5);
	ChannelDriver_MapInit();
}


/*
 * 参考/测量的通道切换，EN引脚为4K方波，A2引脚NA，此处均可忽略
 * 真值表	  A0   |  A1   |  A2   |  EN
 * 0(NA)	  0		  0       0       0
 * 11(Ref)	  1		  1       0       1
 * 10(Mea)	  0		  1       0       1
 * */
void ChannelDriver_SetSwitchMap(uint8_t map)
{
	TRACE_INFO("\n A0 %d, A1 %d, A2 %d , EN %d", (map & 0x01), (map & 0x02), (map & 0x04), (map & 0x08));
	if(map & 0x01)
	{
		ADG708_A0_HIGH();
	}
	else
	{
		ADG708_A0_LOW();
	}

	if(map & 0x02)
	{
		ADG708_A1_HIGH();
	}
	else
	{
		ADG708_A1_LOW();
	}

	if(map & 0x04)
	{
		ADG708_A2_HIGH();
	}
	else
	{
		ADG708_A2_LOW();
	}

	if(map & 0x08)
	{
		ADG708_EN_HIGH();
	}
	else
	{
		ADG708_EN_LOW();
	}
}

/*
 * 设置信号增益倍数，要求GAIN1|3保持同步，GAIN2|4保持同步，以下为合理值(0,5,10,15)对应的真值表
 * 真值表	 GAIN1 | GAIN2 | GAIN3 | GAIN4
 * 0(1倍)	  0		  0       0       0
 * 5(0.5倍)	  1		  0       1       0
 * 10(0.01倍) 0		  1       0       1
 * 15(3倍)	  1		  1       1       1
 * */
void ChannelDriver_SetGainMap(uint8_t gainMap)
{
	TRACE_INFO("\n GAIN1 %d, GAIN2 %d, GAIN3 %d, GAIN4 %d ", (gainMap & 0x01), (gainMap & 0x02), (gainMap & 0x04),(gainMap & 0x08));
	if(gainMap & 0x01)
	{
		GAIN1_HIGH();
	}
	else
	{
		GAIN1_LOW();
	}

	if(gainMap & 0x02)
	{
		GAIN2_HIGH();
	}
	else
	{
		GAIN2_LOW();
	}

	if(gainMap & 0x04)
	{
		GAIN3_HIGH();
	}
	else
	{
		GAIN3_LOW();
	}

	if(gainMap & 0x08)
	{
		GAIN4_HIGH();
	}
	else
	{
		GAIN4_LOW();
	}
}

uint8_t ChannelDriver_GetGainMap(void)
{
	uint8_t gainMap = 0;
	if(GPIO_ReadOutputDataBit(GAIN1_PORT,GAIN1_PIN))
	{
		gainMap |= 0x01;
	}

	if(GPIO_ReadOutputDataBit(GAIN2_PORT,GAIN2_PIN))
	{
		gainMap |= 0x02;
	}

	if(GPIO_ReadOutputDataBit(GAIN3_PORT,GAIN3_PIN))
	{
		gainMap |= 0x04;
	}

	if(GPIO_ReadOutputDataBit(GAIN4_PORT,GAIN4_PIN))
	{
		gainMap |= 0x08;
	}

	TRACE_INFO("\n Read GAIN1 %d, GAIN2 %d, GAIN3 %d, GAIN4 %d ", (gainMap & 0x01), (gainMap & 0x02), (gainMap & 0x04),(gainMap & 0x08));
	return gainMap;
}
