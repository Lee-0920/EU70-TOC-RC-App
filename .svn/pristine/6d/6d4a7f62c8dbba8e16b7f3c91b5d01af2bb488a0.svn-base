/*
 * TMCConfig.c
 *
 *  Created on: 2020年1月7日
 *      Author: Administrator
 */


#include "TMCConfig.h"
#include "SystemConfig.h"
#include "Driver/System.h"
#include "Tracer/Trace.h"
#include "PeristalticPump/TMCConfig.h"
#include "Driver/LiquidDriver/TMCConfigDriver.h"
#include "Driver/LiquidDriver/ExTMCConfigDriver.h"
#include "Driver/LiquidDriver/ReTMCConfigDriver.h"
//#include "PeristalticPump/DisplacementMotorManager.h"
//#include "PeristalticPump/SyringeManager.h"
//#include "PeristalticPump/PeristalticPumpManager.h"
#include "FreeRTOS.h"
#include "task.h"
#include <math.h>
#include "Common/Types.h"

/**
 *  Example:
 *  GCONF             0x00     0x000000C0   //软件设置细分 开启串口功能
 *  IHOLD_IRUN   0x10     0x00070A03   //电机负载驱动电流设置
 *  TCOOLTHRS    0x14     0x000003E8   //开启电机堵转功能
 *  SGTHRS           0x40     0x00000050   //堵转检测灵敏度阈值
 *  CHOPCONF     0x6C     0x14000053   //配置细分为16
 *  PWMCONF     0x70     0xC10D0024   //默认值
 */

#define  TMC_REG_GCONF   0x00               ///<全局配置寄存器 RW  b8-多步滤波(默认1推荐0) b7-微步寄存器使能 b6-UART功能 b3-电机反转(默认0) b0-默认1推荐0
#define  TMC_REG_GSTAT    0x01              ///<全局状态寄存器 R  b2-欠压 b1-高温错误 b0-复位
#define  TMC_REG_IFCNT    0x02             ///<写指令计数器  R
#define TMC_REG_IHOLD_IRUN  0x10    ///<(只写)电机保持运行电流配置 b19-b16:降电流延时 b12-b8:运行电流 b4-b0:静止电流  0=1/32 31=32/32
#define TMC_REG_TCOOLTHRS   0x14    ///<电机堵转检测
#define TMC_REG_SGTHRS   0x40           ///<堵转灵敏度
#define  TMC_REG_CHOPCONF    0x6C     ///<斩波配置寄存器 RW //微步细分设置 b27-b24:%0000-%1000 -> 256-1 (%0100 = 16)
#define  TMC_REG_DRV_STATUS   0x6F    ///<驱动状态寄存器 R
#define  TMC_REG_PWMCONF    0x70    ///<PWM斩波配置 RW
#define  TMC_SLAVER_ADDRESS_LIMIT    0x04    ///<串口地址上限

//#define GPIO_TMCC_DIAG1_PIN         GPIO_Pin_10
//#define GPIO_TMCC_DIAG1_PORT        GPIOD
//#define DIAG1 					    GPIO_ReadInputDataBit(GPIO_TMCC_DIAG1_PORT, GPIO_TMCC_DIAG1_PIN)
//
//#define GPIO_TMCC_DIAG2_PIN         GPIO_Pin_15
//#define GPIO_TMCC_DIAG2_PORT        GPIOA
//#define DIAG2 					    GPIO_ReadInputDataBit(GPIO_TMCC_DIAG2_PORT, GPIO_TMCC_DIAG2_PIN)

typedef enum
{
  TMC_INIT  = 0,  //!< TMC驱动初始化
  TMC_IDLE  =  1, //!< TMC正常
  TMC_CHECK = 2,     //!< TMC驱动自检
}TMC_MONITOR_STATUS;

/* ----------------------- Task ----------------------------------------*/
xTaskHandle s_tmcMonitorTask;
static void TMCMonitor_Handle(TaskHandle_t argument);

static TMC_MONITOR_STATUS s_tmcMonitorStatus = TMC_INIT;

Bool s_initOver = TRUE;

void TMCConfig_Init(void)
{
    TMCConfigDriver_UARTInit();//步进电机1-4
    ExTMCConfigDriver_UARTInit();//步进电机5-8
    ReTMCConfigDriver_UARTInit();//步进电机9-12
    xTaskCreate(TMCMonitor_Handle, "TMCMonitor", TMC_MONITOR_STK_SIZE, NULL, TMC_MONITOR_TASK_PRIO, &s_tmcMonitorTask);

    vTaskResume(s_tmcMonitorTask);
}

void TMCConfig_Reinit(void)
{
    TRACE_WARN("\nTMC Driver ReInit");
    s_tmcMonitorStatus = TMC_INIT;
    vTaskResume(s_tmcMonitorTask);
}

DriverGroup TMCConfig_GroupSelect(Uint8 index)
{
	DriverGroup gp = None;
	switch(index)
	{
	case 0:
		gp = Group1;
		break;
	case 1:
		gp = Group1;
		break;
	case 2:
		gp = Group1;
		break;
	case 3:
		gp = Group1;
		break;
	case 4:
		gp = Group2;
		break;
	case 5:
		gp = Group2;
		break;
	case 6:
		gp = Group2;
		break;
	case 7:
		gp = Group2;
		break;
	case 8:
		gp = Group3;
		break;
	case 9:
		gp = Group3;
		break;
	case 10:
		gp = Group3;
		break;
	case 11:
		gp = Group3;
		break;
	}
	return gp;
}

DriverError TMCConfig_ReadDriveError(Uint8 slaveAddr)
{
    DriverError error = NoError;
    Uint32 gstat = 0;
    Uint32 drvStatus = 0;

    TMCConfig_ReadData(slaveAddr, TMC_REG_GSTAT, &gstat);
    System_NonOSDelay(1);
    if(gstat & 0x2)  //高温/驱动出错
    {
        TMCConfig_ReadData(slaveAddr, TMC_REG_DRV_STATUS, &drvStatus);
        if(drvStatus & 0xFC)
        {
            error = ShortCircuit;
            TRACE_ERROR("\nTMC Driver Status = 0x%08X, Diagnostic : ShortCircuit", drvStatus);
        }
        else if(drvStatus & 0x03)
        {
            error = OverTemperature;
            TRACE_ERROR("\nTMC Driver Status = 0x%08X, Diagnostic : OverTemperature", drvStatus);
        }
    }
    else if(gstat & 0x4)  //欠压
    {
        error = UnderVoltage;
        TRACE_ERROR("\nTMC Driver GSTAT = 0x%08X, Diagnostic : UnderVoltage", gstat);
    }
    else
    {
        TRACE_INFO("\nTMC Driver GSTAT = 0x%08X, Diagnostic : NoError", gstat);
    }

    return error;
}

Bool TMCConfig_EnableMicrostepReg(Uint8 slaveAddr)
{
    Uint32 gconf = 0x000000C0;  //GCONF b7=1 b6=1

    System_NonOSDelay(1);

   TMCConfig_WriteData(slaveAddr, TMC_REG_GCONF, gconf);

   return TRUE;
}

Uint32 TMCConfig_ReadSubdivision(Uint8 slaveAddr)
{
    Uint32 subdivision = 0;
    Uint32 chopConf = 0;
    if(TRUE == TMCConfig_ReadData(slaveAddr, TMC_REG_CHOPCONF, &chopConf))
    {
        subdivision = (Uint32)(pow(2, 8 - ((chopConf >> 24) & 0xF)));
    }
    TRACE_INFO("\nTMC read slave %d subdivision = %d", slaveAddr, subdivision);

    return subdivision;
}

Uint32 TMCConfig_CurrentSet(Uint8 slaveAddr, Uint8 ihold, Uint8 irun, Uint8 delay)
{
    Uint32 data = ((delay&0xF) << 16) + ((irun&0x1F) << 8) + (ihold&0x1F);
    TRACE_INFO("\nTMC current set ihold = 0x%x, irun = 0x%x, delay = 0x%x, data = 0x%08X", ihold, irun, delay, data);

    if(TRUE == TMCConfig_WriteData(slaveAddr, TMC_REG_IHOLD_IRUN, data))
    {
        return TRUE;
    }
    else
    {
        TRACE_ERROR("\nTMC set driver%d current failed", slaveAddr);
        return FALSE;
    }
}

Bool TMCConfig_WriteSubdivision(Uint8 slaveAddr, Uint32 subdivision)
{
    if(subdivision > 256)
    {
        TRACE_ERROR("\nTMC set subdivision = %d out of limit", subdivision);
        return FALSE;
    }

    Uint8 value = 8 - (Uint8)(log2(subdivision));

    Uint32 chopConf = 0;
    TMCConfig_ReadData(slaveAddr, TMC_REG_CHOPCONF, &chopConf);

    chopConf = (chopConf & 0xF0FFFFFF) | (value << 24);

    System_NonOSDelay(1);

    if(TRUE == TMCConfig_WriteData(slaveAddr, TMC_REG_CHOPCONF, chopConf))
    {
        return TRUE;
    }
    else
    {
        TRACE_ERROR("\nTMC set slave %d subdivision = %d failed", slaveAddr, subdivision);
        return FALSE;
    }
}

static Bool TMCConfig_MotorDriverInit(Pump* motor)
{
    Uint8 slaveAddr = motor->number;
    Uint32 subdivision = motor->subdivision;
    Uint8 ret = FALSE;

    TMCConfig_EnableMicrostepReg(slaveAddr);
    System_NonOSDelay(1);
    if(slaveAddr == 0)
    {
        TMCConfig_CurrentSet(slaveAddr, 0, 30, 4);               //定量电机 电流设置
    }
    else
    {
        TMCConfig_CurrentSet(slaveAddr, 0, 30, 4);              //电流设置
    }
    //TMCConfig_CurrentSet(slaveAddr, TMC_IHOLD, TMC_IRUN, TMC_IDELAY);               //电流初始化设置
//    System_NonOSDelay(1);
//    TMCConfig_WriteData(slaveAddr, TMC_REG_PWMCONF, 0xC10D0024);
    System_NonOSDelay(1);
    TMCConfig_WriteSubdivision(slaveAddr, subdivision);     //细分设置
    System_NonOSDelay(1);
    if(TMCConfig_ReadSubdivision(slaveAddr) == subdivision)
    {
        ret = TRUE;
    }
    else
    {
        System_NonOSDelay(1);
        TMCConfig_WriteSubdivision(slaveAddr, subdivision);
        System_NonOSDelay(1);
        if(TMCConfig_ReadSubdivision(slaveAddr) == subdivision)
        {
            ret = TRUE;
        }
        else
        {
            TRACE_ERROR("\nTMC Driver %d Subdivision Init Failed !", slaveAddr);
            ret = FALSE;
        }
    }

    System_NonOSDelay(1);
    TMCConfig_WriteData(slaveAddr, TMC_REG_GSTAT, 0x1);  //Write 1 to Clear Reset Bit

    return ret;
}

Uint8 TMCConfig_DriverCheck(Pump* motor)
{
    Uint8 slaveAddr = motor->number;
    Uint32 gstat = 0;
    TMCConfig_ReadData(slaveAddr, TMC_REG_GSTAT, &gstat);
    TRACE_INFO("\nTMC Reset Check addr %d, gstat = 0x%08x", slaveAddr, gstat);
    System_Delay(1);
    if(TRUE == (gstat & 0x1))
    {
        TRACE_WARN("\nTMC Driver Reset %d", slaveAddr);
        TMCConfig_MotorDriverInit(motor);
    }

    return (Uint8)gstat;
}

Bool TMCConfig_ReadData(Uint8 slaveAddr, Uint8 regAddr, Uint32* data)
{
    Uint8 sendData[4] = {0};
    TMC_Data resp;
    DriverGroup gp = TMCConfig_GroupSelect(slaveAddr);

    sendData[0] = 0x05;
    if(slaveAddr >= TMC_SLAVER_ADDRESS_LIMIT*2)
	{
    	sendData[1] = slaveAddr - TMC_SLAVER_ADDRESS_LIMIT*2;
//    	Printf("\n read9");
	}
    else if(slaveAddr >= TMC_SLAVER_ADDRESS_LIMIT)
    {
    	sendData[1] = slaveAddr - TMC_SLAVER_ADDRESS_LIMIT;
//    	Printf("\n read5-8");
    }
    else
    {
    	sendData[1] = slaveAddr;
//    	Printf("\n read1-4");
    }

    sendData[2] = regAddr;

    switch(gp)
    {
    case Group1:
    	TMCConfigDriver_CRC8(sendData, 4);

		TMCConfigDriver_WriteData(sendData, 4);

		if(TRUE == TMCConfigDriver_WaitRespData(&resp, 100))
		{
			if(regAddr == resp.regAddr)
			{
				*data = resp.data;
				TRACE_DEBUG("\nTMC read slave = %d, reg = 0x%02X, data = 0x%08X", slaveAddr, regAddr, resp.data);
			}
			else
			{
				TRACE_ERROR("\nWait TMC resp wrong addr");
				return FALSE;
			}
		}
		else
		{
			TRACE_ERROR("\nWait TMC resp timeout, slave = %d, reg = 0x%02X ", slaveAddr, regAddr);
			return FALSE;
		}
    	break;
    case Group2:
    	ExTMCConfigDriver_CRC8(sendData, 4);

		ExTMCConfigDriver_WriteData(sendData, 4);

		if(TRUE == ExTMCConfigDriver_WaitRespData(&resp, 100))
		{
			if(regAddr == resp.regAddr)
			{
				*data = resp.data;
				TRACE_DEBUG("\nExTMC read slave = %d, reg = 0x%02X, data = 0x%08X", slaveAddr, regAddr, resp.data);
			}
			else
			{
				TRACE_ERROR("\nWait ExTMC resp wrong addr");
				return FALSE;
			}
		}
		else
		{
			TRACE_ERROR("\nWait ExTMC resp timeout, slave = %d, reg = 0x%02X ", slaveAddr, regAddr);
			return FALSE;
		}
       	break;
    case Group3:
    	ReTMCConfigDriver_CRC8(sendData, 4);

		ReTMCConfigDriver_WriteData(sendData, 4);

		if(TRUE == ReTMCConfigDriver_WaitRespData(&resp, 100))
		{
			if(regAddr == resp.regAddr)
			{
				*data = resp.data;
				TRACE_DEBUG("\nReTMC read slave = %d, reg = 0x%02X, data = 0x%08X", slaveAddr, regAddr, resp.data);
			}
			else
			{
				TRACE_ERROR("\nWait ReTMC resp wrong addr");
				return FALSE;
			}
		}
		else
		{
			TRACE_ERROR("\nWait ReTMC resp timeout, slave = %d, reg = 0x%02X ", slaveAddr, regAddr);
			return FALSE;
		}
       	break;
    }

    return TRUE;
}

Bool TMCConfig_WriteData(Uint8 slaveAddr, Uint8 regAddr, Uint32 data)
{
    Uint8 sendData[8] = {0};
    DriverGroup gp = TMCConfig_GroupSelect(slaveAddr);
    sendData[0] = 0x05;
    if(slaveAddr >= TMC_SLAVER_ADDRESS_LIMIT*2)
	{
		sendData[1] = slaveAddr - TMC_SLAVER_ADDRESS_LIMIT*2;
	}
	else if(slaveAddr >= TMC_SLAVER_ADDRESS_LIMIT)
	{
		sendData[1] = slaveAddr - TMC_SLAVER_ADDRESS_LIMIT;
	}
	else
	{
		sendData[1] = slaveAddr;
	}
    sendData[2] = 0x80 + regAddr;
    sendData[3] = (data >> 24) & 0xFF;
    sendData[4] = (data >> 16) & 0xFF;
    sendData[5] = (data>> 8) & 0xFF;
    sendData[6] = data & 0xFF;

    switch(gp)
    {
    case Group1:
    	TRACE_DEBUG("\nTMC write slave = %d, register = 0x%02X, data = 0x%08X", slaveAddr, regAddr, data);

		TMCConfigDriver_CRC8(sendData, 8);

		TMCConfigDriver_WriteData(sendData, 8);
    	break;
    case Group2:
    	TRACE_DEBUG("\nExTMC write slave = %d, register = 0x%02X, data = 0x%08X", slaveAddr, regAddr, data);

		ExTMCConfigDriver_CRC8(sendData, 8);

		ExTMCConfigDriver_WriteData(sendData, 8);
		break;
    case Group3:
    	TRACE_DEBUG("\nReTMC write slave = %d, register = 0x%02X, data = 0x%08X", slaveAddr, regAddr, data);

		ReTMCConfigDriver_CRC8(sendData, 8);

		ReTMCConfigDriver_WriteData(sendData, 8);
		break;
    }

    return TRUE;
}

void TMCConfig_RegList(Uint8 slaveAddr)
{
    Uint32 data[6] = {0};
    TMCConfig_ReadData(slaveAddr, TMC_REG_GCONF, &data[0]);
	System_NonOSDelay(1);
	TMCConfig_ReadData(slaveAddr, TMC_REG_GSTAT, &data[1]);
	System_NonOSDelay(1);
	TMCConfig_ReadData(slaveAddr, TMC_REG_IFCNT, &data[2]);
	System_NonOSDelay(1);
	TMCConfig_ReadData(slaveAddr, TMC_REG_CHOPCONF, &data[3]);
	System_NonOSDelay(1);
	TMCConfig_ReadData(slaveAddr, TMC_REG_DRV_STATUS, &data[4]);
	System_NonOSDelay(1);
	TMCConfig_ReadData(slaveAddr, TMC_REG_PWMCONF, &data[5]);

	Printf("\n====TMC Driver %d Reg List====", slaveAddr);
	Printf("\nGCONF(%02X): 0x%08X", TMC_REG_GCONF, data[0]);
	Printf("\nGSTAT(%02X): 0x%08X", TMC_REG_GSTAT, data[1]);
	Printf("\nIFCNT(%02X): 0x%08X", TMC_REG_IFCNT, data[2]);
	Printf("\nCHOPCONF(%02X): 0x%08X", TMC_REG_CHOPCONF, data[3]);
	Printf("\nDRV_STATUS(%02X): 0x%08X", TMC_REG_DRV_STATUS, data[4]);
	Printf("\nPWMCONF(%02X): 0x%08X", TMC_REG_PWMCONF, data[5]);
}

char* TMCConfig_DriverName(Uint8 index)
{
    static char name[20] = "";
    memset(name, 0, sizeof(name));
    switch(index)
    {
        case 0:
            strcpy(name, "X-Motor");
            break;
        case 1:
            strcpy(name, "Z-Motor");
            break;
        case 2:
            strcpy(name, "Syringe");
            break;
        case 3:
            strcpy(name, "Pump");
            break;
        default:
            strcpy(name, "NULL");
            break;
    }
    return name;
}

/**
  * @brief 读取tmcc驱动错误输出反馈引脚状态，TOC三个泵输出反馈由一个三极管检测
  * @brief 检测脚已在步进电机初始化中完成，此处可直接使用
  */
Bool TMCCOnfig_DiagCheck()
{
//	if(DIAG1 || DIAG2)
//	{
		return TRUE;
//	}
//	else
//	{
//		TRACE_ERROR("\npump error");
//		return FALSE;
//	}
}


/**
  * @brief PMT测试
  */
void TMCMonitor_Handle(TaskHandle_t argument)
{
    while(1)
    {
        switch(s_tmcMonitorStatus)
        {
            case TMC_INIT:
                System_Delay(10);
                TMCConfig_MotorDriverInit(PumpManager_GetPump(0));	//Group1
                System_Delay(10);
                TMCConfig_MotorDriverInit(PumpManager_GetPump(1));
                System_Delay(10);
                TMCConfig_MotorDriverInit(PumpManager_GetPump(2));
                System_Delay(10);
                TMCConfig_MotorDriverInit(PumpManager_GetPump(3));
				System_Delay(10);
				TMCConfig_MotorDriverInit(PumpManager_GetPump(4));	//Group2
				System_Delay(10);
				TMCConfig_MotorDriverInit(PumpManager_GetPump(5));
				System_Delay(10);
				TMCConfig_MotorDriverInit(PumpManager_GetPump(6));
				System_Delay(10);
				TMCConfig_MotorDriverInit(PumpManager_GetPump(7));
				System_Delay(10);
				TMCConfig_MotorDriverInit(PumpManager_GetPump(8));	//Group3
				System_Delay(10);
				TMCConfig_MotorDriverInit(PumpManager_GetPump(9));
				System_Delay(10);
				TMCConfig_MotorDriverInit(PumpManager_GetPump(10));
				System_Delay(10);
				TMCConfig_MotorDriverInit(PumpManager_GetPump(11));
                 s_tmcMonitorStatus = TMC_IDLE;
                break;
            case TMC_IDLE:
                vTaskSuspend(s_tmcMonitorTask);
                break;
            default:
                break;
        }
        if(!TMCCOnfig_DiagCheck()) 		//此处只有初始化的时候会检测一次，非实时监测
        {
        	TMCConfig_DriverCheck(PumpManager_GetPump(0));
        	TMCConfig_DriverCheck(PumpManager_GetPump(1));
        	TMCConfig_DriverCheck(PumpManager_GetPump(2));
        	TMCConfig_DriverCheck(PumpManager_GetPump(3));
        	TMCConfig_DriverCheck(PumpManager_GetPump(4));
			TMCConfig_DriverCheck(PumpManager_GetPump(5));
			TMCConfig_DriverCheck(PumpManager_GetPump(6));
			TMCConfig_DriverCheck(PumpManager_GetPump(7));
			TMCConfig_DriverCheck(PumpManager_GetPump(8));
        }
        System_Delay(1000);
    }
}
