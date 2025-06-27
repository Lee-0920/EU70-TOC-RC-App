
/*
 * BoxFanDriver.c
 *
 *  Created on: 2016年11月09日
 *      Author: Liang
 */
#include <TempDriver/BoxFanDriver.h>
#include "Tracer/Trace.h"
#include "Driver/System.h"
#include "SystemConfig.h"

// 机箱风扇
#define BOXFAN_PIN_UP                         GPIO_Pin_12
#define BOXFAN_PORT_UP                        GPIOE
#define BOXFAN_RCC_CONFIG_UP                  RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOE, ENABLE)
#define BOXFAN_UP_HIGH()					  GPIO_SetBits(BOXFAN_PORT_UP, BOXFAN_PIN_UP);
#define BOXFAN_UP_LOW()						  GPIO_ResetBits(BOXFAN_PORT_UP, BOXFAN_PIN_UP);

#define BOXFAN_PIN_DOWN                       GPIO_Pin_12
#define BOXFAN_PORT_DOWN                      GPIOE
#define BOXFAN_RCC_CONFIG_DOWN                RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOE, ENABLE)
#define BOXFAN_DOWN_HIGH()					  GPIO_SetBits(BOXFAN_PORT_DOWN, BOXFAN_PIN_DOWN);
#define BOXFAN_DOWN_LOW()					  GPIO_ResetBits(BOXFAN_PORT_DOWN, BOXFAN_PIN_DOWN);

//进样风扇
#define INSIDEFAN_PIN                         GPIO_Pin_12
#define INSIDEFAN_PORT                     	  GPIOE
#define INSIDEFAN_RCC_CONFIG              	  RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOE, ENABLE)
#define INSIDEFAN_HIGH()					  GPIO_SetBits(INSIDEFAN_PORT, INSIDEFAN_PIN);
#define INSIDEFAN_LOW()				 		  GPIO_ResetBits(INSIDEFAN_PORT, INSIDEFAN_PIN);

#define UP_TIMER_RCC                		  RCC_APB1Periph_TIM12
#define UP_TIMER_RCC_CONFIG         		  RCC_APB1PeriphClockCmd(UP_TIMER_RCC,ENABLE)
#define UP_TIMER_IRQn               		  TIM8_BRK_TIM12_IRQn
#define UP_IRQHANDLER         				  TIM8_BRK_TIM12_IRQHandler
#define UP_TIMER                 		      TIM12

#define DOWN_TIMER_RCC                		  RCC_APB1Periph_TIM13
#define DOWN_TIMER_RCC_CONFIG        		  RCC_APB1PeriphClockCmd(DOWN_TIMER_RCC,ENABLE)
#define DOWN_TIMER_IRQn             		  TIM8_UP_TIM13_IRQn
#define DOWN_IRQHANDLER         			  TIM8_UP_TIM13_IRQHandler
#define DOWN_TIMER                		      TIM13

typedef enum
{
	BOXFAN_IDLE,
	BOXFAN_ON,
	BOXFAN_OFF,
}BoxFanState;

// 定时器
#define BOXFAN_TIMER_PERIOD                  (4000-1)    // 重载计数
#define BOXFAN_TIMER_PRESCALER               (90-1)     // 预分频
#define BOXFAN_FAN_OFF_PERIOD               (500-1)     // 预分频
static uint16_t s_upFan = BOXFAN_TIMER_PERIOD;          // 上风扇中断周期

static uint16_t s_downFan = BOXFAN_TIMER_PERIOD;        // 下风扇中断周期

static Bool s_isBoxFanUpOpen = FALSE;
static Bool s_isBoxFanDownOpen = FALSE;

static BoxFanState s_upFanState = BOXFAN_OFF;
static BoxFanState s_downFanState = BOXFAN_OFF;

void BoxFanDriver_Init(void)
{
    GPIO_InitTypeDef GPIO_InitStructure;

    BOXFAN_RCC_CONFIG_UP;
//    BOXFAN_RCC_CONFIG_DOWN;
    UP_TIMER_RCC_CONFIG;
//    DOWN_TIMER_RCC_CONFIG;
//    INSIDEFAN_RCC_CONFIG;

    GPIO_InitStructure.GPIO_Pin=BOXFAN_PIN_UP;
    GPIO_InitStructure.GPIO_Mode=GPIO_Mode_OUT;
    GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_DOWN;
    GPIO_InitStructure.GPIO_Speed=GPIO_Speed_50MHz;
    GPIO_Init(BOXFAN_PORT_UP,&GPIO_InitStructure);
    GPIO_ResetBits(BOXFAN_PORT_UP, BOXFAN_PIN_UP);

//    GPIO_InitStructure.GPIO_Pin=BOXFAN_PIN_DOWN;
//	GPIO_InitStructure.GPIO_Mode=GPIO_Mode_OUT;
//	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
//	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_DOWN;
//	GPIO_InitStructure.GPIO_Speed=GPIO_Speed_50MHz;
//	GPIO_Init(BOXFAN_PORT_DOWN,&GPIO_InitStructure);
//	GPIO_ResetBits(BOXFAN_PORT_DOWN, BOXFAN_PIN_DOWN);
//
//	GPIO_InitStructure.GPIO_Pin=INSIDEFAN_PIN;
//	GPIO_Init(INSIDEFAN_PORT,&GPIO_InitStructure);
//	GPIO_ResetBits(INSIDEFAN_PORT, INSIDEFAN_PIN);

	NVIC_InitTypeDef NVIC_InitStructure;

	// 变量初始化
	memset(&NVIC_InitStructure, 0, sizeof(NVIC_InitStructure));

	NVIC_InitStructure.NVIC_IRQChannel                   = UP_TIMER_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = BOXFAN_UP_IRQ_PRIORITY;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
	NVIC_InitStructure.NVIC_IRQChannelCmd                = ENABLE;
	NVIC_Init(&NVIC_InitStructure);

	// 变量初始化
   memset(&NVIC_InitStructure, 0, sizeof(NVIC_InitStructure));

	NVIC_InitStructure.NVIC_IRQChannel                   = DOWN_TIMER_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = BOXFAN_DOWN_IRQ_PRIORITY;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
	NVIC_InitStructure.NVIC_IRQChannelCmd                = ENABLE;
	NVIC_Init(&NVIC_InitStructure);


    TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;

    // 变量初始化
    memset(&TIM_TimeBaseStructure, 0, sizeof(TIM_TimeBaseStructure));

    TIM_TimeBaseStructure.TIM_Period        = BOXFAN_TIMER_PERIOD;
    TIM_TimeBaseStructure.TIM_Prescaler     = BOXFAN_TIMER_PRESCALER;
    TIM_TimeBaseStructure.TIM_ClockDivision = 0;
    TIM_TimeBaseStructure.TIM_CounterMode   = TIM_CounterMode_Up;
    TIM_TimeBaseInit(UP_TIMER, &TIM_TimeBaseStructure);
    TIM_ClearFlag(UP_TIMER, TIM_IT_Update);
    TIM_ITConfig(UP_TIMER, TIM_IT_Update, ENABLE);  // 使能计数中断
    TIM_Cmd(UP_TIMER, DISABLE);

    memset(&TIM_TimeBaseStructure, 0, sizeof(TIM_TimeBaseStructure));

	TIM_TimeBaseStructure.TIM_Period        = BOXFAN_TIMER_PERIOD;
	TIM_TimeBaseStructure.TIM_Prescaler     = BOXFAN_TIMER_PRESCALER;
	TIM_TimeBaseStructure.TIM_ClockDivision = 0;
	TIM_TimeBaseStructure.TIM_CounterMode   = TIM_CounterMode_Up;
	TIM_TimeBaseInit(DOWN_TIMER, &TIM_TimeBaseStructure);
	TIM_ClearFlag(DOWN_TIMER, TIM_IT_Update);
	TIM_ITConfig(DOWN_TIMER, TIM_IT_Update, ENABLE);  // 使能计数中断
	TIM_Cmd(DOWN_TIMER, DISABLE);
}


static void BoxFanDriver_SetLevel(uint8_t index, float level)
{
	if(level > 0)
	{
		if(0 == index)
		{
			BOXFAN_UP_HIGH();
			s_upFanState = BOXFAN_ON;
			s_upFan = (uint16_t)(level * BOXFAN_TIMER_PERIOD);
			TIM_ITConfig(UP_TIMER, TIM_IT_Update, ENABLE);  // 使能计数中断
			TIM_Cmd(UP_TIMER, ENABLE);
			TRACE_DEBUG("up boxfan %d", s_upFan);
			s_isBoxFanUpOpen = TRUE;
		}
		else if(2 == index)
		{
			INSIDEFAN_HIGH();
			TRACE_DEBUG("inside boxfan");
		}
		else
		{
			BOXFAN_DOWN_HIGH();
			s_downFanState = BOXFAN_ON;
			s_downFan = (uint16_t)(level * BOXFAN_TIMER_PERIOD);
			TIM_ITConfig(DOWN_TIMER, TIM_IT_Update, ENABLE);  // 使能计数中断
			TIM_Cmd(DOWN_TIMER, ENABLE);
			TRACE_DEBUG("down boxfan %d", s_downFan);
			s_isBoxFanDownOpen = TRUE;
		}
	}
}

/**
 * @brief not use
 * @param
 */
static void BoxFanDriver_Close(uint8_t index)
{
    if(0 == index)
    {
    	s_upFan = 0;
		s_upFanState = BOXFAN_OFF;
		BOXFAN_UP_LOW();
		TIM_Cmd(UP_TIMER, DISABLE);
		s_isBoxFanUpOpen = FALSE;
    }
    else if(2 == index)
	{
    	INSIDEFAN_LOW();
	}
    else
    {
    	s_downFan = 0;
    	s_downFanState = BOXFAN_OFF;
    	BOXFAN_DOWN_LOW();		//预留风扇高电平为关闭
		TIM_Cmd(DOWN_TIMER, DISABLE);
		s_isBoxFanDownOpen = FALSE;
    }

}
/**
 * @brief 设置风扇转速
 * @param
 */
void BoxFanDriver_SetOutput(uint8_t index, float level)
{

    if (level <= 1 && level >= 0)
    {
        if(level != 0)
        {
        	BoxFanDriver_SetLevel(index, level);
        }
        else
        {
            BoxFanDriver_Close(index);
        }
        TRACE_DEBUG("\n outsidefan %d level %d %%", index, (uint32_t )(level * 100));
    }
    else
    {
        TRACE_ERROR("\n OutSideFan level error!");
    }
}

Bool BoxFanDriver_IsOpen(Uint8 index)
{
	if(index == 0)
	{
		return s_isBoxFanUpOpen;
	}
	else
	{
		return s_isBoxFanDownOpen;
	}
}

void UP_IRQHANDLER(void)
{
    if(TIM_GetITStatus(UP_TIMER, TIM_IT_Update) != RESET)
    {
        uint16_t cnt = 0;
        switch(s_upFanState)
        {
		case BOXFAN_ON:
			BOXFAN_UP_LOW();
			cnt = BOXFAN_FAN_OFF_PERIOD;
			s_upFanState = BOXFAN_OFF;
			break;
		case BOXFAN_OFF:
			BOXFAN_UP_HIGH();
			cnt = s_upFan;
			s_upFanState = BOXFAN_ON;
			break;
        }
        // 装载计数值
		TIM_SetCounter(UP_TIMER, 0);
		TIM_SetAutoreload(UP_TIMER, cnt);
    }
    TIM_ClearITPendingBit(UP_TIMER, TIM_IT_Update);
}

void DOWN_IRQHANDLER(void)
{
    if(TIM_GetITStatus(DOWN_TIMER, TIM_IT_Update) != RESET)
    {
        uint16_t cnt = 0;
        switch(s_downFanState)
		{
		case BOXFAN_ON:
			BOXFAN_DOWN_LOW();
			s_downFanState = BOXFAN_OFF;
			cnt = BOXFAN_FAN_OFF_PERIOD;
			break;
		case BOXFAN_OFF:
			BOXFAN_DOWN_HIGH();
			cnt = s_downFan;
			s_downFanState = BOXFAN_ON;
			break;
		}
		// 装载计数值
		TIM_SetCounter(DOWN_TIMER, 0);
		TIM_SetAutoreload(DOWN_TIMER, cnt);
    }
    TIM_ClearITPendingBit(DOWN_TIMER, TIM_IT_Update);
}



