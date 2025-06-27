/*
 * SystemConfig.h
 *
 *  Created on: 2016年6月7日
 *      Author: Administrator
 */

#ifndef SRC_SYSTEMCONFIG_H_
#define SRC_SYSTEMCONFIG_H_
#include "PeristalticPump/PumpManager.h"

//外设中断抢占式优先级配置=============================================================================
#define    WATCHDOG_IRQ_PRIORITY                    3
#define    LAIRS485ADAPTER_IRQ_PRIORITY             7
#define    CONSOLEUART_IRQ_PRIORITY                 7
#define    PUMP_TIMERA_PRIORITY                     9
#define    PUMP_TIMERB_PRIORITY                     9
#define    PUMP_TIMERC_PRIORITY                     9
#define    PUMP_TIMERD_PRIORITY                     9
#define    PWM_TIMER_PRIORITY                     9
#define    METER_ADCDMA_PRIORITY                    8
#define    TEMP_ADCDMA_PRIORITY                    8
#define    SYSTEM_STATE_TIMER_PRIORITY              5
#define    TMC_UART_IRQ_PRIORITY                    6
#define    ADS131_IRQ_PRIORITY                 		6
#define    BOXFAN_UP_IRQ_PRIORITY                   6
#define    BOXFAN_DOWN_IRQ_PRIORITY                 6
//DNCP系统使能配置===================================================================
#define USE_DNCP_SYSTEM     0
//本地系统使能配置===================================================================
#define USE_LOCAL_SYSTEM   1
//统一使用蠕动泵DNCP接口配置==========================================================================
#define    DNCP_PUMP_TOTAL                          4
//freeRTOS 任务优先级和堆栈大小配置===================================================================
//基本任务
#define CONSOLESCHEDULER_TASK_PRIO                  5   //控制台命令处理任务
#define CONSOLESCHEDULER_STK_SIZE                   256

#define LAIRS485COMMITTOUPPER_TASK_PRIO             6   //LAI485接收任务
#define LAIRS485COMMITTOUPPER_STK_SIZE              512

#define LAIRS485SENDREQUEST_TASK_PRIO               6   //LAI485发送任务
#define LAIRS485SENDREQUEST_STK_SIZE                512

#define LAIRS485MONITORHOST_TASK_PRIO               6   //LAI485监控任务
#define LAIRS485MONITORHOST_STK_SIZE                128

#define DNCPSTACKDSCPCMD_TASK_PRIO                  6   //DNCP命令处理任务
#define DNCPSTACKDSCPCMD_STK_SIZE                   256

#define DEVICEINDICATOR_LED_TASK_PRIO               4   //运行指示灯任务
#define DEVICEINDICATOR_LED_STK_SIZE                128

#define UPDATER_ERASE_TASK_PRIO                     4   //升级擦除任务
#define UPDATER_ERASE_STK_SIZE                      128

//控制台循环输出信息任务
#define CMDCLINE_INFOOUTPUT_TASK_PRIO               4   //命令台常规信息输出
#define CMDCLINE_INFOOUTPUT_STK_SIZE                256

// 接受电机动作完成事件
#define PUMP_EVENT_TASK_PRIO                        6  //接受电机事件任务
#define PUMP_EVENT_STK_SIZE                         128

//TMC控制
#define TMC_MONITOR_TASK_PRIO					5  //TMC任务
#define TMC_MONITOR_STK_SIZE   				    128

//温控功能
#define TEMP_MONITOR_TASK_PRIO                      4   //温度监控任务
#define TEMP_MONITOR_STK_SIZE                       256

#define THERMOSTAT_TASK_PRIO                        5   //恒温器任务
#define THERMOSTAT_STK_SIZE                         256

#define TEMP_REPORT_TASK_PRIO                       3   //温度上报任务
#define TEMP_REPORT_STK_SIZE                        256

//用户FLASH地址和大小配置==============================================================================
#define UPDATE_FLASH_START                          0x08000000//UPDATE程序空间48K
#define UPDATE_FLASH_END                            0x0800BFFF
#define UPDATE_DATA_FLASH_START                     0x0800C000//UPDATE数据空间16K
#define UPDATE_DATA_FLASH_END                       0x0800FFFF
#define APP_DATA_FLASH_START                        0x08010000//APP数据空间64K
#define APP_DATA_FLASH_END                          0x0801FFFF
#define APP_FLASH_START                             0x08020000 
#define APP_FLASH_END                               0x081FFFFF


#define FLASH_FACTORY_SIGN                          0xAA55AA55
#define FLASH_USE_BASE                              APP_DATA_FLASH_START
//板卡信息：共96byte
#define DEVICE_INFO_SIGN_FLASH_BASE_ADDR            FLASH_USE_BASE
#define DEVICE_INFO_SIGN_FLASH_LEN                  4

#define DEVICE_INFO_TYPE_ADDRESS                    (DEVICE_INFO_SIGN_FLASH_BASE_ADDR + DEVICE_INFO_SIGN_FLASH_LEN)
#define DEVICE_INFO_TYPE_LEN                        16
#define DEVICE_INFO_MODEL_ADDRESS                   (DEVICE_INFO_TYPE_ADDRESS + DEVICE_INFO_TYPE_LEN)
#define DEVICE_INFO_MODEL_LEN                       24
#define DEVICE_INFO_SN_ADDRESS                      (DEVICE_INFO_MODEL_ADDRESS + DEVICE_INFO_MODEL_LEN)
#define DEVICE_INFO_SN_LEN                          32
#define DEVICE_INFO_MANUF_ADDRESS                   (DEVICE_INFO_SN_ADDRESS +  DEVICE_INFO_SN_LEN)
#define DEVICE_INFO_MANUF_LEN                       20
#define DEVICE_INFO_DATE_ADDRESS                    (DEVICE_INFO_MANUF_ADDRESS + DEVICE_INFO_MANUF_LEN)
#define DEVICE_INFO_DATE_LEN                        4

//泵参数：共32*4=128yte   //原先预留了4个泵的参数位置
#define PUMP_SIGN_FLASH_BASE_ADDR                   (DEVICE_INFO_DATE_ADDRESS + DEVICE_INFO_DATE_LEN)
#define PUMP_SIGN_FLASH_LEN                         4
#define PUMP_SIGN_FLASH_LEN_TOTAL           		(PUMP_SIGN_FLASH_LEN*4)

#define PUMP_MOTIONPARAM_FLASH_BASE_ADDR            (PUMP_SIGN_FLASH_BASE_ADDR + PUMP_SIGN_FLASH_LEN_TOTAL)
#define PUMP_MOTIONPARAM_FLASH_LEN                  8
#define PUMP_MOTIONPARAM_FLASH_LEN_TOTAL     		(PUMP_MOTIONPARAM_FLASH_LEN*4)

#define PUMP_FACTOR_FLASH_BASE_ADDR                 (PUMP_MOTIONPARAM_FLASH_BASE_ADDR + PUMP_MOTIONPARAM_FLASH_LEN_TOTAL)
#define PUMP_FACTOR_FLASH_LEN                       4
#define PUMP_FACTOR_FLASH_LEN_TOTAL          	    (PUMP_FACTOR_FLASH_LEN*4)

//温控功能参数：共24byte 2个温度校准参数
#define TEMPERATURE_BASE                                 (PUMP_FACTOR_FLASH_BASE_ADDR + PUMP_FACTOR_FLASH_LEN_TOTAL)
#define TEMPERATURE_CALIBRATE_FACTOR_ADDRESS             (TEMPERATURE_BASE + 0)
#define TEMPERATURE_CALIBRATE_FACTOR_LEN                 12
#define TEMPERATURE_CALIBRATE_FACTOR_LEN_TOTAL     		 (TEMPERATURE_CALIBRATE_FACTOR_LEN * 2 )  //实际只用了前12个字节
#define TEMPERATURE_FACTORY_SIGN_FLASH_BASE_ADDR         (TEMPERATURE_BASE + TEMPERATURE_CALIBRATE_FACTOR_LEN_TOTAL)
#define TEMPERATURE_FACTORY_SIGN_FLASH_LEN               4
#define TEMPERATURE_FACTORY_SIGN_FLASH_LEN_TOTAL         (4 * 2)

#define THERMOSTAT_FACTORY_SIGN_FLASH_BASE_ADDR          (TEMPERATURE_FACTORY_SIGN_FLASH_BASE_ADDR + TEMPERATURE_FACTORY_SIGN_FLASH_LEN_TOTAL)
#define THERMOSTAT_FACTORY_SIGN_FLASH_LEN                4
#define THERMOSTAT_PARAM_ADDRESS                         (THERMOSTAT_FACTORY_SIGN_FLASH_BASE_ADDR + THERMOSTAT_FACTORY_SIGN_FLASH_LEN)
#define THERMOSTAT_PARAM_LEN                             12


//扩展泵参数：共32 * 5 = 160 byte 额外预留8个泵的参数位置
#define EXTPUMP_SIGN_FLASH_BASE_ADDR                   (THERMOSTAT_PARAM_ADDRESS + THERMOSTAT_PARAM_LEN)
#define EXTPUMP_SIGN_FLASH_LEN                         PUMP_SIGN_FLASH_LEN
#define EXTPUMP_SIGN_FLASH_LEN_TOTAL           		   (PUMP_SIGN_FLASH_LEN*8)

#define EXTPUMP_MOTIONPARAM_FLASH_BASE_ADDR            (EXTPUMP_SIGN_FLASH_BASE_ADDR + EXTPUMP_SIGN_FLASH_LEN_TOTAL)
#define EXTPUMP_MOTIONPARAM_FLASH_LEN                  PUMP_MOTIONPARAM_FLASH_LEN
#define EXTPUMP_MOTIONPARAM_FLASH_LEN_TOTAL     	   (PUMP_MOTIONPARAM_FLASH_LEN*8)

#define EXTPUMP_FACTOR_FLASH_BASE_ADDR                 (EXTPUMP_MOTIONPARAM_FLASH_BASE_ADDR + EXTPUMP_MOTIONPARAM_FLASH_LEN_TOTAL)
#define EXTPUMP_FACTOR_FLASH_LEN                       PUMP_FACTOR_FLASH_LEN
#define EXTPUMP_FACTOR_FLASH_LEN_TOTAL         		   (PUMP_FACTOR_FLASH_LEN*8)

//扩展的温控功能参数：共16 * 6 = 96 byte 预留2个温度计
#define EXTTEMPERATURE_FACTORY_SIGN_FLASH_BASE_ADDR         (EXTPUMP_FACTOR_FLASH_BASE_ADDR + EXTPUMP_FACTOR_FLASH_LEN_TOTAL)
#define EXTTEMPERATURE_FACTORY_SIGN_FLASH_LEN               4
#define EXTTEMPERATURE_FACTORY_SIGN_FLASH_LEN_TOTAL         (EXTTEMPERATURE_FACTORY_SIGN_FLASH_LEN*2)
#define EXTTEMPERATURE_CALIBRATE_FACTOR_ADDRESS             (EXTTEMPERATURE_FACTORY_SIGN_FLASH_BASE_ADDR + EXTTEMPERATURE_FACTORY_SIGN_FLASH_LEN_TOTAL)
#define EXTTEMPERATURE_CALIBRATE_FACTOR_LEN                 12
#define EXTTEMPERATURE_CALIBRATE_FACTOR_LEN_TOTAL           (EXTTEMPERATURE_CALIBRATE_FACTOR_LEN*2)

//扩展的温控功能参数：共16 * 4 = 64 byte 预留4个恒温器参数
#define EXTTHERMOSTAT_FACTORY_SIGN_FLASH_BASE_ADDR          (EXTTEMPERATURE_CALIBRATE_FACTOR_ADDRESS + EXTTEMPERATURE_CALIBRATE_FACTOR_LEN_TOTAL)
#define EXTTHERMOSTAT_FACTORY_SIGN_FLASH_LEN                4
#define EXTTHERMOSTAT_FACTORY_SIGN_FLASH_LEN_TOTAL          (EXTTHERMOSTAT_FACTORY_SIGN_FLASH_LEN*4)
#define EXTTHERMOSTAT_PARAM_ADDRESS                         (EXTTHERMOSTAT_FACTORY_SIGN_FLASH_BASE_ADDR + EXTTHERMOSTAT_FACTORY_SIGN_FLASH_LEN_TOTAL)
#define EXTTHERMOSTAT_PARAM_LEN                             12
#define EXTTHERMOSTAT_PARAM_LEN_TOTAL                		(EXTTHERMOSTAT_PARAM_LEN*4)

//所有使用的FLASH 669
#define FLASH_USE_SIZE                              ((u16)400)

#endif /* SRC_SYSTEMCONFIG_H_ */
