/**
 * @addtogroup module_CmdLine
 * @{
 */

/**
 * @file
 * @brief 应用接口：命令行实现。
 * @details 定义各应用命令及其处理函数。
 * @version 1.0.0
 * @author kim.xiejinqiang
 * @date 2012-5-21
 */


//#include <OpticalDriver/OpticalADCollect.h>
#include "FreeRTOS.h"
#include "task.h"
#include <string.h>
#include <stdlib.h>
#include "Tracer/Trace.h"
#include "Console/Console.h"
#include "Driver/System.h"
#include "CmdLine.h"
#include "SystemConfig.h"
#include "DeviceIndicatorLED.h"
#include "UpdateDriver.h"
#include "DeviceUpdate/UpdateHandle.h"
#include "McuFlash.h"
#include "Driver/HardwareType.h"
#include "Manufacture/VersionInfo.h"
#include "PeristalticPump/PumpManager.h"
#include "PeristalticPump/TMCConfig.h"
#include "Driver/LiquidDriver/TMCConfigDriver.h"
#include "SolenoidValve/ValveManager.h"
#include "Driver/LiquidDriver/ValveDriver.h"
#include "OpticalControl/OpticalControl.h"
#include "Driver/OpticalDriver/ADS1220Driver.h"
#include "Driver/UltraSignalDriver/ChannelDriver.h"

#include <TempDriver/TempADCollect.h>
#include "Driver/TempDriver/TempCollecterMap.h"
#include "TemperatureControl/TempCollecterManager.h"
#include "TemperatureControl/ThermostatManager.h"
#include "Driver/TempDriver/TempCollecterMap.h"
#include "Driver/TempDriver/ThermostatDeviceMap.h"
#include "TemperatureControl/ThermostatDeviceManager.h"
#include "TemperatureControl/ThermostatManager.h"
#include "TemperatureControl/TempCollecterManager.h"
#include "Driver/TempDriver/BoxFanDriver.h"

// 命令行版本定义，命令有变更时，需要相应更新本版本号
const CmdLineVersion g_kCmdLineVersion =
{
        1,      // 主版本号
        0,      // 次版本号
        0,      // 修订版本号
        0       // 编译版本号
};
static  Uint8 s_currPumpNum = 0;

// 命令处理函数声明列表
static int Cmd_help(int argc, char *argv[]);
static int Cmd_welcome(int argc, char *argv[]);
static int Cmd_version(int argc, char *argv[]);
static int Cmd_showparam(int argc, char *argv[]);
static int Cmd_reset(int argc, char *argv[]);
static int Cmd_trace(int argc, char *argv[]);
static int Cmd_demo(int argc, char *argv[]);

//系统命令
static int Cmd_flash(int argc, char *argv[]);
static int Cmd_RestoreInit(int argc, char *argv[]);
static int Cmd_IAP(int argc, char *argv[]);
static int Cmd_SetBlink(int argc, char *argv[]);
static int Cmd_TaskState(int argc, char *argv[]);
static int Cmd_Hardware(int argc, char *argv[]);

//泵命令函数
static int Cmd_PumpNum(int argc, char *argv[]);
static int Cmd_PumpStatus(int argc, char *argv[]);
static int Cmd_PumpParam(int argc, char *argv[]);
static int Cmd_PumpFactor(int argc, char *argv[]);
static int Cmd_TotalPumps(int argc, char *argv[]);
static int Cmd_PumpVolume(int argc, char *argv[]);
static int Cmd_Pump(int argc, char *argv[]);

//温控命令
static int Cmd_Temp(int argc, char *argv[]);
static int Cmd_TempFactor(int argc, char *argv[]);
static int Cmd_Therm(int argc, char *argv[]);
static int Cmd_ThermDevice(int argc, char *argv[]);
static int Cmd_BoxFan(int argc, char *argv[]);

//阀命令函数
static int Cmd_valve(int argc, char *argv[]);

static int Cmd_OAI(int argc, char *argv[]);

static int Cmd_AD(int argc, char *argv[]);

static void InfoOutput(void *argument);
/**
 * @brief 命令行命令表，保存命令关键字与其处理函数之间的对应关系。
 * @details 每条命令对应一个结点，结点包括：命令关键字、处理函数、简短描述文本。
 */
const CmdLineEntry g_kConsoleCmdTable[] =
{
    { "demo",       Cmd_demo,       "\t\t: Demo for cmd implementation and param parse" },
    { "trace",      Cmd_trace,      "\t\t: Trace level" },
    { "welcome",    Cmd_welcome,    "\t\t: Display welcome message" },
    { "version",    Cmd_version,    "\t\t: Display version infomation about this application" },
    { "reset",      Cmd_reset,      "\t\t: Reset system" },
    { "help",       Cmd_help,       "\t\t: Display list of commands. Short format: h or ?" },

    { "flash",      Cmd_flash,      "\t\t: Flash read write erase operation." },
    { "RI",         Cmd_RestoreInit, "\t\t:Restore system initial state." },
    { "iap",        Cmd_IAP,         "\t\t:Provides erase and jump operations." },
    { "blink",      Cmd_SetBlink,    "\t\t:Set the duration of equipment indicator, on time and off time.Uint milliseconds." },
    { "taskstate",  Cmd_TaskState,  "\t\t: Out put system task state." },
    { "hardware",   Cmd_Hardware,  "\t\t: Read Hardware Info." },

    { "valve",      Cmd_valve,      "\t\t: valve set read operation." },
//    { "pumpfactor", Cmd_PumpFactor, "\t\t: Setting and acquisition of pump calibration coefficient." },
//    { "pumpstatus", Cmd_PumpStatus, "\t\t: Get the running state of the pump." },
//    { "pumpparam",  Cmd_PumpParam,  "\t\t: Get the running state of the pump." },
//    { "pumptotal",  Cmd_TotalPumps, "\t\t: Get pump total." },
//    { "pumpvolume", Cmd_PumpVolume, "\t\t: Gets the volume of a pump on the current pump ." },
//    { "pump",       Cmd_Pump,       "\t\t: The start and stop of the pump. " },
//    { "pumpnum",    Cmd_PumpNum,    "\t\t: Sets the current pump number. " },

	{ "oai",        Cmd_OAI,        "\t\t: AD Signal acquisition operation ." },
	{ "tempfactor", Cmd_TempFactor, "\t\t: Temperature sensor coefficient operation." },
	{ "temp",       Cmd_Temp,       "\t\t: get temperature" },
	{ "thermdevice", Cmd_ThermDevice, "\t\t: " },
	{ "therm",      Cmd_Therm,      "\t\t: Thermostat temperature" },
	{ "boxfan",     Cmd_BoxFan,      "\t\t: boxfan control" },

	{ "ad",         Cmd_AD,   "\t\t: " },

    { "?",          Cmd_help,       0 },
    { "h",          Cmd_help,       0 },
    { "showparam",  Cmd_showparam,  0 },
    { 0, 0, 0 }
};


/**
 * @brief 判断第一个字串等于第二个字串。
 * @details 本函数与strcmp相比，预先做了有效性检查。
 * @param str1 要比较的字串1。
 * @param str2 要比较的字串2，不能为NULL。
 * @return 是否相等。
 */
Bool IsEqual(const char* str1, const char* str2)
{
    return (0 == strcmp(str1, str2)) ? TRUE : FALSE;
}

static xTaskHandle s_InfoOutputHandle;
static Uint16 s_getInfoTime = 0;
typedef enum
{
    LC,
    OA,
    TC,
    SYS,
}InfoOutputMode;

static InfoOutputMode s_InfoOutputMode = LC;

void CmdLine_Init(void)
{
    xTaskCreate(InfoOutput, "InfoOutput", CMDCLINE_INFOOUTPUT_STK_SIZE, NULL,
            CMDCLINE_INFOOUTPUT_TASK_PRIO, &s_InfoOutputHandle);
}

static void InfoOutput(void *argument)
{
    (void) argument;
    vTaskSuspend(NULL);
    while (1)
    {
        vTaskDelay(s_getInfoTime / portTICK_RATE_MS);
        switch(s_InfoOutputMode)
        {
        	case OA:
				OpticalControl_PrintfInfo();
				break;
        	case LC:
        		Printf("\n temperature: %f", TempCollecterManager_GetTemp(0));
				break;
            case SYS:
                System_TaskStatePrintf();
                break;
        }
    }
}

//*****************************************************************************
//
// 系统常规命令处理函数
//
//*****************************************************************************
#include "console/port/driver/ConsoleUart.h"

int Cmd_TaskState(int argc, char *argv[])
{
    if(IsEqual(argv[1], "start"))
    {
        if (argv[2] && atoi(argv[2]) >= 10)
        {
            s_getInfoTime = atoi(argv[2]);
            s_InfoOutputMode = SYS;
            vTaskResume(s_InfoOutputHandle);
        }
        else
        {
            Printf("Invalid param %s\n", argv[2]);
        }
    }
    else if(IsEqual(argv[1], "stop"))
    {
        vTaskSuspend(s_InfoOutputHandle);
    }
    else if (0 == argv[1] || IsEqual(argv[1], "help") ||
             IsEqual(argv[1], "?"))
    {
        Printf("====== taskstate commands ======\n");
        Printf(" taskstate start [TIME]:ms\n");
        Printf(" taskstate stop        :\n");
    }
    return (0);
}

int Cmd_Hardware(int argc, char *argv[])
{
    Uint8 type = 0;
    if (IsEqual(argv[1], "type"))
    {
        type = HardwareType_GetValue();
        Printf("Hardware Type: %d\n", type);
    }
    else if (0 == argv[1] || IsEqual(argv[1], "help") ||
             IsEqual(argv[1], "?"))
    {
        Printf("====== hardware commands ======\n");
        Printf(" type : get hardware type value\n");
    }

    return type;
}

int Cmd_SetBlink(int argc, char *argv[])
{
    if (argv[1])
    {
        if (argv[2] && argv[3])
        {
            DeviceIndicatorLED_SetBlink(atoi(argv[1]), atoi(argv[2]), atoi(argv[3]));
        }
        else
        {
            Printf("Invalid param\n");
        }
    }
    else if (0 == argv[1] || IsEqual(argv[1], "help") ||
             IsEqual(argv[1], "?"))
    {
        Printf("====== blink commands ======\n");
        Printf(" blink [DURATION] [ONTIME] [OFFTIME]:\n");
    }

    return (0);
}

int Cmd_IAP(int argc, char *argv[])
{
    if (IsEqual(argv[1], "erase"))
    {
        UpdateHandle_StartErase();
    }
    else if (IsEqual(argv[1], "write"))
    {
        if (argv[2] && argv[3] && argv[4])
        {
            UpdateHandle_WriteProgram((u8 *)argv[2], atoi(argv[3]), atoi(argv[4]));
        }
        else
        {
            Printf("Invalid param\n");
        }
    }
    else if (IsEqual(argv[1], "check"))
    {
        if (argv[2])
        {
            UpdateHandle_CheckIntegrity(atoi(argv[2]));
        }
        else
        {
            Printf("Invalid param\n");
        }
    }
    else if (IsEqual(argv[1], "read"))
    {
        if (argv[2] && argv[3])
        {
            uint8_t str[30]={""};
            UpdateDriver_Read(atoi(argv[2]),atoi(argv[3]),str);
            Printf("\n");
            for(int i = 0; i< atoi(argv[3]); i++)
            {
                Printf("0x%02x ",str[i]);
            }
            Printf("\n%s",str);
        }
        else
        {
            Printf("Invalid param\n");
        }
    }
    else if (IsEqual(argv[1], "getmax"))
    {
        Printf("%d\n", UpdateHandle_GetMaxFragmentSize());
    }
    else if (IsEqual(argv[1], "getmode"))
    {
        DeviceRunMode mode = UpdateHandle_GetRunMode();
        Printf("%d\n", mode);
    }
#ifdef _CS_APP
    else if (IsEqual(argv[1], "updater"))
    {
        UpdateHandle_EnterUpdater();
    }
#else
    else if (IsEqual(argv[1], "app"))
    {
        UpdateHandle_EnterApplication();
    }
#endif
    else if (0 == argv[1] || IsEqual(argv[1], "help") ||
             IsEqual(argv[1], "?"))
    {
        Printf("====== iap commands ======\n");
        Printf(" iap erase: \n");
        Printf(" iap write [TEXT]   [NUM]  [SEQ]: \n");
        Printf(" iap check [CRC16]              : \n");
        Printf(" iap read  [OFFSET] [NUM]       : \n");
#ifdef _CS_APP
        Printf(" iap updater                    : \n");
#else
        Printf(" iap app                        : \n");
#endif
        Printf(" iap getmax                     : \n");
        Printf(" iap getmode                    : \n");
    }
    else
    {
        Printf("Invalid param: %s\n", argv[1]);
    }
    return (0);
}
   
int Cmd_flash(int argc, char *argv[])
{
    if (IsEqual(argv[1], "deletewrite"))//不保留原始数据的写
    {
        if (argv[2] && argv[3] && argv[4])
        {
            McuFlash_DeleteWrite(atoi(argv[2]), atoi(argv[3]),(u8 *)argv[4]);
            Printf("\nWriteAddr 0x%x ",atoi(argv[2]));
        }
        else
        {
            Printf("Invalid param\n");
        }
    }
    else if (IsEqual(argv[1], "write"))//保留原始数据的写
    {
        if (argv[2] && argv[3] && argv[4])
        {
            McuFlash_Write(atoi(argv[2]), atoi(argv[3]),(u8 *)argv[4]);
            Printf("\nWriteAddr 0x%x ",atoi(argv[2]));
        }
        else
        {
            Printf("Invalid param\n");
        }
    }
    else if (IsEqual(argv[1], "read"))//读FLASH数据
    {
        if (argv[2] && argv[3])
        {
            uint8_t str[30]={""};
            McuFlash_Read(atoi(argv[2]),atoi(argv[3]),str);
            Printf("\nReadAddr 0x%x\n",atoi(argv[2]));
            for(int i = 0; i< atoi(argv[3]); i++)
            {
                Printf("0x%02x ",str[i]);
            }
            Printf("\n%s",str);
        }
        else
        {
            Printf("Invalid param\n");
        }
    }
    else if (IsEqual(argv[1], "erase"))//擦除
    {
        if (argv[2])
        {
           McuFlash_EraseSector(atoi(argv[2]));
           Printf("\nEraseAddr 0x%x", atoi(argv[2]));
        }
        else
        {
            Printf("Invalid param\n");
        }

    }
    else if (IsEqual(argv[1], "end"))//擦除
    {
    	Printf("\n End:%x", EXTPUMP_FACTOR_FLASH_BASE_ADDR);
    }
    else if (0 == argv[1] || IsEqual(argv[1], "help") ||
             IsEqual(argv[1], "?"))
    {
        Printf("====== flash commands ======\n");
        Printf(" flash deletewrite [ADDR] [NUM] [TEXT]: \n");
        Printf(" flash write       [ADDR] [NUM] [TEXT]: \n");
        Printf(" flash read        [ADDR] [NUM]       : \n");
        Printf(" flash erase       [ADDR]             : \n");
    }
    else
    {
        Printf("Invalid param: %s\n", argv[1]);
    }
    return (0);
}

int Cmd_RestoreInit(int argc, char *argv[])
{
    //以下操作可能会因为没有启动而控制台出现错误提醒。
    PumpManager_Restore();//依次关闭所有的泵
	ValveManager_SetValvesMap(0);//这只保证定量运行的时候不会关闭所有的阀。
    return 0;
}

//*****************************************************************************
//
// 阀命令处理函数
//
//*****************************************************************************
int Cmd_valve(int argc, char *argv[])
{
    if (IsEqual(argv[1], "map"))
    {
        if (argv[2] )
        {
            ValveManager_SetValvesMap(System_buildMap(argv[2]));
        }
        else
        {
            Printf("Invalid param");
        }
    }
    else if (IsEqual(argv[1], "open"))
    {
        if (argv[2] && atoi(argv[2]) > 0)
        {
        	Uint32 map = 1;
			Uint8 offset = (atoi(argv[2]) - 1);
			map = map << offset;
            ValveManager_SetValvesMap(map);
        }
        else
        {
            Printf("Invalid param");
        }
    }
    else if (IsEqual(argv[1], "closeall"))
    {
        ValveManager_SetValvesMap(0);
    }
    else if (IsEqual(argv[1], "get"))
    {
        Uint32 map = ValveManager_GetValvesMap();
        Printf("ValvesMap: 0x%6x\n", map);
    }
    else if (IsEqual(argv[1], "total"))
    {
        Printf("total: %d\n", ValveManager_GetTotalValves());
    }
    else if (IsEqual(argv[1], "no"))
	{
		if(argv[2])
		{
			ValveManager_SetValvesMapNormalOpen(atoi(argv[2]));
		}
	}
    else if (0 == argv[1] || IsEqual(argv[1], "help") ||
             IsEqual(argv[1], "?"))
    {
        Printf("====== valve commands ======\n");
        Printf(" valve map  [MAP]: Set the map of the valve. map must be 0x0 - 0x%x.\n\n", SOLENOIDVALVE_MAX_MAP);
        Printf(" valve open [NUM]: Open valve NUM.Num must be 1 - %d.\n", ValveManager_GetTotalValves());
        Printf(" valve closeall  : Close all valves. \n");
        Printf(" valve get       : Mapping map of the solenoid valve. \n");
        System_Delay(5);
        Printf(" valve total     : Total number of solenoid valves. \n");
        Printf(" valve ex  [0/1][true/false] : turn on/off extra device \n");
        Printf(" valve no  [map] : normal open turn on valve map 0x0 - 0x%x\n");
//        Printf(" valve off [index] : turn off dac valve. index 0-1 \n");
    }
    else
    {
        Printf("Invalid param: %s\n", argv[1]);
    }
    return (0);
}

//*****************************************************************************
//
// 泵命令处理函数
//
//*****************************************************************************
int Cmd_PumpNum(int argc, char *argv[])
{
    if (argv[1])
    {
        if (atoi(argv[1]) < PumpManager_GetTotalPumps())
        {
            s_currPumpNum = atoi(argv[1]);
        }
        else
        {
            Printf("NUM must to be 0 - %d\n", PumpManager_GetTotalPumps() - 1);
        }
    }
    else if (0 == argv[1] || IsEqual(argv[1], "help") || IsEqual(argv[1], "?"))
    {
        Printf("====== pumpnum commands ======\n");
        Printf(
                " pumpnum [NUM]: Set the pump for the current operation to NUM.NUM must to be 0 - %d\n",
                PumpManager_GetTotalPumps() - 1);
    }
    return (0);
}

int Cmd_PumpStatus(int argc, char *argv[])
{
    PumpStatus status = PumpManager_GetStatus(s_currPumpNum);
    switch(status)
    {
    case PUMP_IDLE:
        Printf("Pump: %d IDLE\n",s_currPumpNum);
        break;
    case PUMP_BUSY:
        Printf("Pump: %d BUSY\n",s_currPumpNum);
        break;
    }
    return (0);
}

int Cmd_PumpFactor(int argc, char *argv[])
{
    if (IsEqual(argv[1], "set"))
    {
        if (argv[2])
        {
        	PumpManager_SetFactor(s_currPumpNum, atof(argv[2]));
        }
        else
        {
            Printf("Invalid param\n");
        }
    }
    else if (IsEqual(argv[1], "get"))
    {
        float factor = PumpManager_GetFactor(s_currPumpNum);
        Printf("Pump %d Factor ", s_currPumpNum);
        System_PrintfFloat(1, factor, 8);
        Printf(" ml/step");


    }
    else if (0 == argv[1] || IsEqual(argv[1], "help") ||
             IsEqual(argv[1], "?"))
    {
        Printf("====== pumpfactor commands ======\n");
        Printf(" pumpfactor set [FACTOR]: Set calibration FACTOR for current pump.Unit is ml/step\n");
        Printf(" pumpfactor get         : Calibration parameters for reading the current pump.\n");
    }
    else if (IsEqual(argv[1], "test"))
    {
    	Uint8 buffer[PUMP_SIGN_FLASH_LEN] = { 0 };
	  Uint32 flashFactorySign = 0;
	  Uint32 baseAddr = PUMP_SIGN_FLASH_BASE_ADDR;
	  Uint8 index = atoi(argv[2]);
	  if(index >= 0)
	  {

		  if(index >= 2)  //扩展的泵
		  {
			  McuFlash_Read(EXTPUMP_SIGN_FLASH_BASE_ADDR + PUMP_SIGN_FLASH_LEN * (index-2), PUMP_SIGN_FLASH_LEN, buffer); //读取出厂标志位
		  }
		  else
		  {
			  McuFlash_Read(baseAddr + PUMP_SIGN_FLASH_LEN * index, PUMP_SIGN_FLASH_LEN, buffer); //读取出厂标志位
		  }

		  memcpy(&flashFactorySign, buffer, PUMP_SIGN_FLASH_LEN);
		  Printf("\n pump %d,  %x", index, flashFactorySign);
	  }



    }
    else
    {
        Printf("Invalid param: %s\n", argv[1]);
    }
    return (0);
}

int Cmd_PumpParam(int argc, char *argv[])
{
    if (IsEqual(argv[1], "set"))
    {
        if (argv[2] && argv[3])
        {
            PumpManager_SetMotionParam(s_currPumpNum,atof(argv[2]),atof(argv[3]), WriteFLASH);
        }
        else
        {
            Printf("Invalid param\n");
        }
    }
    else if (IsEqual(argv[1], "get"))
    {
        PumpParam param= PumpManager_GetMotionParam(s_currPumpNum);

        Printf("Pump %d acc:", s_currPumpNum);
        System_PrintfFloat(1, param.acceleration, 4);
        Printf(" ml/(s^2),maxSpeed:");
        System_PrintfFloat(1, param.maxSpeed, 4);
        Printf(" ml/s\n");
    }
    else if (IsEqual(argv[1], "setmoveing"))
    {
        if (argv[2] && argv[3])
        {
            PumpManager_SetMotionParam(s_currPumpNum,atof(argv[2]),atof(argv[3]), NoWriteFLASH);
        }
        else
        {
            Printf("Invalid param\n");
        }
    }
    else if (0 == argv[1] || IsEqual(argv[1], "help") ||
             IsEqual(argv[1], "?"))
    {
        Printf("========== pumpparam commands ==========\n");
        Printf(" pumpparam set         [ACC] [MAXSPEED]: Set the ACC(ml/(s^2)) and MAXSPEED(ml/s) of the current pump, the motion parameters are saved to flash. \n");
        Printf(" pumpparam get                         : Gets the motion parameters of the current pump.\n");
        Printf(" pumpparam setmoveing  [ACC] [MAXSPEED]: Set temporary acceleration and speed of the current pump.\n");
        Printf(" pumpparam accget                      : get the max acceleration of syring.\n");
        Printf(" pumpparam accset      [ACC]           : set the max acceleration of syring.\n");
    }
    else
    {
        Printf("Invalid param: %s\n", argv[1]);
    }
    return (0);
}

int Cmd_TotalPumps(int argc, char *argv[])
{
    Printf("TotalPumps: %d\n", PumpManager_GetTotalPumps());
    return (0);
}

int Cmd_PumpVolume(int argc, char *argv[])
{
   if (IsEqual(argv[1], "change"))
   {
       if (argv[2])
       {
           PumpManager_ChangeVolume(s_currPumpNum, atof(argv[2]));
       }
       else
       {
           Printf("Invalid param\n");
       }
   }
   else if (IsEqual(argv[1], "get"))
   {
       float volume = PumpManager_GetVolume(s_currPumpNum);
       Printf("Pump %d Volume", s_currPumpNum);
       System_PrintfFloat(1, volume, 4);
       Printf(" ml");
   }
   else if (0 == argv[1] || IsEqual(argv[1], "help") ||
            IsEqual(argv[1], "?"))
   {
       Printf("====== pumpvolume commands ======\n");
       Printf(" pumpvolume change [VOLUME]: Change the volume of the pump. Uint ml\n");
       Printf(" pumpvolume get            : drain VOLUME ml\n");
   }
   return (0);
}

int Cmd_Pump(int argc, char *argv[])
{

    Bool isParamOK = FALSE;
    Direction dir;

    if (IsEqual(argv[1], "extract") || IsEqual(argv[1], "e"))
    {
        isParamOK = TRUE;
        dir = COUNTERCLOCKWISE;
    }
    else if (IsEqual(argv[1], "drain") || IsEqual(argv[1], "d"))
    {
        isParamOK = TRUE;
        dir = CLOCKWISE;
    }
    if(TRUE == isParamOK)
    {
        if(argv[2])
        {
            if(argv[3] && argv[4])
            {
                PumpManager_SetMotionParam(s_currPumpNum, atof(argv[3]), atof(argv[4]),
                        NoWriteFLASH);
                PumpManager_Start(s_currPumpNum, dir, atof(argv[2]), NoReadFLASH);
            }
            else
            {
                PumpManager_Start(s_currPumpNum,dir,atof(argv[2]), ReadFLASH);
            }
        }
        else
        {
            Printf("Invalid param\n");
        }

    }
    else if (IsEqual(argv[1], "stop"))
    {
        PumpManager_Stop(s_currPumpNum);
    }
    else if (0 == argv[1] || IsEqual(argv[1], "help") ||
             IsEqual(argv[1], "?") )
    {
        Printf("====== pump commands ======\n");
        Printf(" pump extract(e) [VOLUME] [ACC] [SPEED]: Extract VOLUME ml.ACC and SPEED is optional.Uint ml/s.\n");
        Printf(" pump drain(d)   [VOLUME] [ACC] [SPEED]: drain VOLUME ml.ACC and SPEED is optional.Uint ml/s. \n");
        Printf(" pump stop                             : \n");
    }
    else
    {
        Printf("Invalid param: %s\n", argv[1]);
    }
    return (0);

}

 int Cmd_OAI(int argc, char *argv[])
 {
	 float temp;
     if (IsEqual(argv[1], "getad"))
     {
         if(IsEqual(argv[2], "start"))
         {
             if (argv[3] && atoi(argv[3]) >= 10)
             {
                 s_getInfoTime = atoi(argv[3]);
                 s_InfoOutputMode = OA;
                 vTaskResume(s_InfoOutputHandle);
             }
             else
             {
                 Printf("Invalid param\n");
             }
         }
         else if(IsEqual(argv[2], "stop"))
         {
             vTaskSuspend(s_InfoOutputHandle);
         }
         else
         {
             Printf("\n Invalid param");
         }
     }
     else if (IsEqual(argv[1], "start"))
     {

     }
     else if (IsEqual(argv[1], "stop"))
     {
     }
     else if (IsEqual(argv[1], "report"))
	{
		if (argv[2] && atof(argv[2]) >= 0)
		{
			OpticalControl_SetSignalADNotifyPeriod(atof(argv[2]));
		}

	}
//     else if (IsEqual(argv[1], "mea"))
//     {
//    	 ChannelDriver_SetSwitchMap(10);
//     }
//     else if (IsEqual(argv[1], "ref"))
//	{
//    	 ChannelDriver_SetSwitchMap(11);
//	}
    else if (IsEqual(argv[1], "gainmap"))
	{
    	ChannelDriver_SetGainMap(atoi(argv[2]));
	}
    else if (IsEqual(argv[1], "write"))
	{
    	if(atoi(argv[3]))
    	{
    		OpticalControl_AD5175Write(atoi(argv[2]), atoi(argv[3]));
    	}

	}
    else if (IsEqual(argv[1], "read"))
	{
		OpticalControl_AD5175Read(atoi(argv[2]));
//		for(Uint8 i = 0;i < 0x2F;i++)
//		{
//			OpticalControl_AD5175ReadWithAddr(0, i);
//			System_Delay(200);
//		}
	}
     else if (0 == argv[1] || IsEqual(argv[1], "help") ||
              IsEqual(argv[1], "?"))
     {
         Printf("====== oai commands ======\n");
         Printf(" oai getad  start  [TIME]:Ad value of the optical is continuously display.Display interval [TIME].Uint is s.\n");
         Printf(" oai getad  stop         :Stop continue to collect the ad value of the current channel.\n");
         Printf(" oai start  [TIME]       :Start collecte ad value of the mea and the ref,acquisition time TIME s.\n");
         Printf(" oai stop                : \n");
         Printf(" oai reprot [TIME]       :Start upload ad signal\n");
         System_Delay(10);
//         Printf(" oai switchmap  [map]  :Set SwitchMap, Free Combine Follow Switch Option.\n \t --->0x01-A0;0x02-A1;0x04-A2;0x08-EN\n");
//         Printf(" oai ref                : Select ref channel \n");
//         Printf(" oai mea                : Select mea channel \n");
         System_Delay(10);
         Printf(" oai gainmap  [map]  :Set GainMap, Free Combine Follow Gain Option.\n \t "
        		 "--->0x01-Gain1;0x02-Gain2;0x04-Gain3;0x08-Gain4\n"
        		 "\n \t --->[map-scale][0-1][5-0.5][10-0.01][15-3]\n");
         Printf(" oai write	[0-1][0-128]  :write to ad5175 \n");
         Printf(" oai read	[0-1] :read ad5175 \n");
     }
     else
     {
         Printf("Invalid param: %s\n", argv[1]);
     }
     return (0);
 }

 //*****************************************************************************
 //
 // 温控命令处理函数
 //
 //*****************************************************************************
 int Cmd_Therm(int argc, char *argv[])
 {
     if (IsEqual(argv[1], "auto"))
     {
         if(argv[2] && argv[3] && argv[4] && argv[5])
         {
             ThermostatManager_SendEventClose(atoi(argv[2]));
             ThermostatManager_Start(atoi(argv[2]),
                     THERMOSTAT_MODE_AUTO, atof(argv[3]), atof(argv[4]), atof(argv[5]));
         }
         else
         {
             Printf("Invalid param\n");
         }
     }
     else if (IsEqual(argv[1], "heater"))
     {
         if(argv[2] && argv[3] && argv[4] && argv[5])
         {
             ThermostatManager_SendEventClose(atoi(argv[2]));
             ThermostatManager_Start(atoi(argv[2]),
                     THERMOSTAT_MODE_HEATER, atof(argv[3]), atof(argv[4]), atof(argv[5]));
         }
         else
         {
             Printf("Invalid param\n");
         }
     }
     else if (IsEqual(argv[1], "refrigerate"))
     {
         if(argv[2] && argv[3] && argv[4] && argv[5])
         {
             ThermostatManager_SendEventClose(atoi(argv[2]));
             ThermostatManager_Start(atoi(argv[2]),
                     THERMOSTAT_MODE_REFRIGERATE, atof(argv[3]), atof(argv[4]), atof(argv[5]));
         }
         else
         {
             Printf("Invalid param\n");
         }
     }
     else if (IsEqual(argv[1], "natural"))
     {
         if(argv[2] && argv[3] && argv[4] && argv[5])
         {
             ThermostatManager_SendEventClose(atoi(argv[2]));
             ThermostatManager_Start(atoi(argv[2]),
                     THERMOSTAT_MODE_NATURAL, atof(argv[3]), atof(argv[4]), atof(argv[5]));
         }
         else
         {
             Printf("Invalid param\n");
         }
     }
     else if (IsEqual(argv[1], "stop"))
     {
         if(argv[2])
         {
             ThermostatManager_SendEventClose(atoi(argv[2]));
             ThermostatManager_RequestStop(atoi(argv[2]));
         }
         else
         {
             Printf("Invalid param\n");
         }
     }
     else if (IsEqual(argv[1], "status"))
     {
         if(argv[2])
         {
             ThermostatStatus result = ThermostatManager_GetStatus(atoi(argv[2]));
             if (THERMOSTAT_IDLE == result)
             {
                 Printf("%s Thermostat IDLE \n", ThermostatManager_GetName(atoi(argv[2])));
             }
             else
             {
                 Printf("%s Thermostat BUSY \n", ThermostatManager_GetName(atoi(argv[2])));
             }
         }
         else
         {
             Printf("Invalid param\n");
         }
     }
     else if (IsEqual(argv[1], "number"))
     {
         Printf("\n GetTotalThermostat %d", TOTAL_THERMOSTAT);
     }
     else if (0 == argv[1] || IsEqual(argv[1], "help") ||
              IsEqual(argv[1], "?"))
     {
         Printf("====== therm commands ======\n");
         Printf(" therm auto        [INDEX] [TEMP] [ALW] [TIME]: \n");
         Printf(" therm heater      [INDEX] [TEMP] [ALW] [TIME]: \n");
         Printf(" therm refrigerate [INDEX] [TEMP] [ALW] [TIME]: \n");
         System_Delay(2);
         Printf(" therm natural     [INDEX] [TEMP] [ALW] [TIME]: \n");
         Printf(" therm stop        [INDEX]                    : \n");
         Printf(" therm status      [INDEX]                    : \n");
         Printf(" therm number                                 : \n");
         System_Delay(10);
         for (Uint8 i = 0; i < TOTAL_THERMOSTAT; i++)
         {
             Printf(" No %d name %s\n", i, ThermostatManager_GetName(i));
         }
     }
     else
     {
         Printf("Invalid param: %s\n", argv[1]);
     }

     return (0);
 }


 int Cmd_ThermDevice(int argc, char *argv[])
{
    if (IsEqual(argv[1], "level"))
    {
        if(argv[2] && argv[3])
        {
            Printf("%s Device SetOutput :", ThermostatDeviceManager_GetName(atoi(argv[2])));
            System_PrintfFloat(1, atof(argv[3]) * 100, 3);
            Printf(" %%");
            ThermostatDeviceManager_SetOutput(atoi(argv[2]), atof(argv[3]));
        }
        else
        {
            Printf("Invalid param: %s\n", argv[2]);
        }
    }
    else if (IsEqual(argv[1], "stop"))
    {
        if(argv[2])
        {
            Printf("%s Device stop", ThermostatDeviceManager_GetName(atoi(argv[2])));
            ThermostatDeviceManager_SetOutput(atoi(argv[2]), 0);
        }
        else
        {
            Printf("Invalid param: %s\n", argv[2]);
        }
    }
    else if (IsEqual(argv[1], "isopen"))
    {
        if(argv[2])
        {
            Printf("\n %s ThermDevice ", ThermostatDeviceManager_GetName(atoi(argv[2])));
            ThermostatDeviceManager_IsOpen(atoi(argv[2]));
        }
        else
        {
            Printf("Invalid param: %s\n", argv[2]);
        }
    }
    else if (IsEqual(argv[1], "setduty"))
    {
        if(argv[2] && argv[3])
        {
            Printf("\n %s Device  SetHeaterMaxDutyCycle :", ThermostatDeviceManager_GetName(atoi(argv[2])));
            System_PrintfFloat(1, atof(argv[3]) * 100, 3);
            Printf(" %%");
            ThermostatDeviceManager_SetMaxDutyCycle(atoi(argv[2]), atof(argv[3]));
        }
        else
        {
            Printf("Invalid param: %s\n", argv[2]);
        }
    }
    else if (IsEqual(argv[1], "getduty"))
    {
        if(argv[2])
        {
            Printf("\n %s Device max duty cycle:", ThermostatDeviceManager_GetName(atoi(argv[2])));
            ThermostatDeviceManager_GetMaxDutyCycle(atoi(argv[2]));
        }
        else
        {
            Printf("Invalid param: %s\n", argv[2]);
        }
    }
    else if (0 == argv[1] || IsEqual(argv[1], "help") ||
            IsEqual(argv[1], "?"))
    {
        Printf("====== thermdevice commands ======\n");
//        Printf("\n0-heater1\t1-heater2\t2-coolfan\n3-stovefan\t4-cool\t5-resfan\n");
        Printf(" thermdevice level   [INDEX] [PER] : 0 - %d device output\n", TOTAL_THERMOSTATDEVICE);
        Printf(" thermdevice stop    [INDEX]       :\n");
        Printf(" thermdevice isopen  [INDEX]       :\n");
        Printf(" thermdevice setduty [INDEX] [DUTY]:\n");
        Printf(" thermdevice getduty [INDEX]       :\n");
        for (Uint8 i = 0; i < TOTAL_THERMOSTATDEVICE; i++)
        {
            Printf(" No %d name %s\n", i, ThermostatDeviceManager_GetName(i));
            System_Delay(1);
        }
    }
    else
    {
        Printf("Invalid param: %s\n", argv[1]);
    }
    return (0);
}

 int Cmd_Temp(int argc, char *argv[])
 {
 	float temp = 0;
     if (IsEqual(argv[1], "get"))
     {
         if(argv[2])
         {
             float temp = TempCollecterManager_GetTemp(atoi(argv[2]));
             Printf("\n %s Temp :  %f", TempCollecterManager_GetName(atoi(argv[2])), temp);
         }
         else
         {
             Printf("Invalid param: %s\n", argv[2]);
         }
     }
     else if (IsEqual(argv[1], "env"))
     {
         Printf("\n EnvironmentTemp :  %f", TempCollecterManager_GetEnvironmentTemp());
     }
     else if (IsEqual(argv[1], "all"))
     {

 //        Printf("\n EnvironmentTemp :  %f", TempCollecterManager_GetEnvironmentTemp());
         for (Uint8 i = 0; i < 2; i++)//TOTAL_THERMOSTAT
         {
         	float temp = 1;
             temp = TempCollecterManager_GetTemp(i);
             Printf("\n %s Thermostat Temp :  %f", TempCollecterManager_GetName(i), temp);
         }
     }
     else if (IsEqual(argv[1], "debug"))
    {
        TempCollecter_SetDebugMode(atoi(argv[2]));
    }
     else if (IsEqual(argv[1], "getad"))
     {
     	char name[10] = "";
     	for (Uint8 i = 0; i < 4; i++)
 		{
 			Uint16 temp = 0;
 			temp = TempADCollect_GetAD(i);
 			switch(i)
 			{
 			case 0:
 				strcpy(name, "MEA");
 				break;
 			case 1:
 				strcpy(name, "REF");
 				break;
 			}
 			Printf("\n ad %s channel %d:  %d", name, i + 1 , temp);
 			System_Delay(10);
 		}
     }
     else if (IsEqual(argv[1], "factor"))
     {
         TempCalibrateParam calibrateFactor;
         if (IsEqual(argv[2], "get"))
         {
             if (argv[3])
             {
                 calibrateFactor = TempCollecterManager_GetCalibrateFactor(atoi(argv[3]));
                 TRACE_INFO("\n %s Temp \n negativeInput = ", TempCollecterManager_GetName(atoi(argv[3])));
                 System_PrintfFloat(1, calibrateFactor.negativeInput, 8);
                 Printf("\n vref =");
                 System_PrintfFloat(1, calibrateFactor.vref, 8);
                 Printf("\n vcal =");
                 System_PrintfFloat(1, calibrateFactor.vcal, 8);
             }
             else
             {
                 Printf("Invalid param!\n");
             }
         }
         else if (IsEqual(argv[2], "set"))
         {
             if (argv[3] && argv[4] && argv[5] && argv[6])
             {
                 calibrateFactor.negativeInput = atof(argv[4]);
                 calibrateFactor.vref = atof(argv[5]);
                 calibrateFactor.vcal = atof(argv[6]);
                 TempCollecterManager_SetCalibrateFactor(atoi(argv[3]), calibrateFactor);
                 Printf("\n set ok");
             }
             else
             {
                 Printf("Invalid param!\n");
             }
         }
     }
     else if (0 == argv[1] || IsEqual(argv[1], "help") ||
              IsEqual(argv[1], "?"))
     {
         Printf("====== temp commands ======\n");
         Printf(" temp get [ID]                         :\n");
         Printf(" temp env                              :\n");
         Printf(" temp all                              :\n");
         Printf(" temp factor get [INDEX]               :\n");
         Printf(" temp factor set [INDEX] [NI] [VR] [VC]:\n");
         Printf(" temp debug                            :0-none,1-stove,2-cooler,4-ndir, 8-upbox, 16-downbox\n");
         Printf(" temp getad                            : printf all ad channel info from 0 - 7\n");

         for (Uint8 i = 0; i < TOTAL_TEMP; i++)
         {
             Printf(" No %d name %s\n", i, TempCollecterManager_GetName(i));
         }
     }
     else
     {
         Printf("Invalid param: %s\n", argv[1]);
     }
     return (0);
 }

 int Cmd_TempFactor(int argc, char *argv[])
 {
     TempCalibrateParam calibrateFactor;
     if (IsEqual(argv[1], "get"))
     {
         if (argv[2])
         {
			calibrateFactor = TempCollecterManager_GetCalibrateFactor(atoi(argv[2]));
			TRACE_INFO("\n %s Thermostat \n negativeInput = ", TempCollecterManager_GetName(atoi(argv[2])));
			System_PrintfFloat(1, calibrateFactor.negativeInput, 8);
			Printf("\n vref =");
			System_PrintfFloat(1, calibrateFactor.vref, 8);
			Printf("\n vcal =");
			System_PrintfFloat(1, calibrateFactor.vcal, 8);
         }
         else
         {
             Printf("Invalid param!\n");
         }
     }
     else if (IsEqual(argv[1], "set"))
     {
         if (argv[2] && argv[3] && argv[4] && argv[5])
         {
             calibrateFactor.negativeInput = atof(argv[3]);
             calibrateFactor.vref = atof(argv[4]);
             calibrateFactor.vcal = atof(argv[5]);
             TempCollecterManager_SetCalibrateFactor(atoi(argv[2]), calibrateFactor);
             Printf("\n set ok");
         }
         else
         {
             Printf("Invalid param!\n");
         }
     }
     else if (0 == argv[1] || IsEqual(argv[1], "help") ||
              IsEqual(argv[1], "?"))
     {
         Printf("====== tempfactor commands ======\n");
         Printf(" tempfactor get [INDEX]               :\n");
         Printf(" tempfactor set [INDEX] [NI] [VR] [VC]: for stove: [index][scale][refad][refTemp]\n");
         System_Delay(10);
         for (Uint8 i = 0; i < 2; i++)
         {
             Printf(" No %d name %s\n", i, TempCollecterManager_GetName(i));
         }
         Printf(" No %d name %s\n", 3, "BoxFanUp");
         Printf(" No %d name %s\n", 4, "BoxFanDown");
     }
     else
     {
         Printf("Invalid param: %s\n", argv[1]);
     }
     return (0);
 }


 int Cmd_BoxFan(int argc, char *argv[])
 {
     if (IsEqual(argv[1], "level"))
     {
         if(argv[2] && atof(argv[2]) >= 0 && atof(argv[2]) <= 1)
         {
        	 BoxFanDriver_SetOutput(0, atof(argv[2]));
         }
         else
         {
             Printf("Invalid param!level: 0 - 1 .\n");
         }
     }
     else if (IsEqual(argv[1], "stop"))
     {
     	 if(argv[2] && atoi(argv[2]) >= 0 && atoi(argv[2]) <= 1)
     	 {
     		 BoxFanDriver_SetOutput(0, atof(argv[2]));
 			 Printf("\n======Boxfan %d Stop======", atoi(argv[2]));
     	 }
     	 else
     	 {
     		 Printf("Invalid param index: 0 - 1 .\n");
     	 }
     }
     else if (IsEqual(argv[1], "status"))
     {
     	 if(argv[2] && atoi(argv[2]) >= 0 && atoi(argv[2]) <= 1)
 		 {
     		 Printf("\n  Boxfan Is Open %d", BoxFanDriver_IsOpen(atoi(argv[2])));
 		 }
 		 else
 		 {
 			 Printf("Invalid param index: 0 - 1 .\n");
 		 }
     }
     else if (0 == argv[1] || IsEqual(argv[1], "help") ||
              IsEqual(argv[1], "?"))
     {
         Printf("====== boxfan commands ======\n");
         Printf(" boxfan level [PER]: boxfan output level 0-1\n");
         Printf(" boxfan stop       :0-1\n");
         Printf(" boxfan status     : return boxfan status\n");
     }
     else
     {
         Printf("Invalid param: %s\n", argv[1]);
     }
     return (0);
 }


 // AD检测
 int Cmd_AD(int argc, char *argv[])
 {
 	 if (IsEqual(argv[1], "get"))
 	{
 	System_Delay(10);
 	for (Uint8 i = 0; i < 4; i++)
 	{
 		Uint16 temp = 0;
 		temp = TempADCollect_GetAD(i);
 		Printf("\n ad conver channel %d:  %d",i + 1 , temp);
 		System_Delay(10);
 	}
 	}
     else if (0 == argv[1] || IsEqual(argv[1], "help") ||
              IsEqual(argv[1], "?"))
     {
         Printf("====== AD ======\n");
         Printf(" ad get  : \n");
     }
     else
     {
         Printf("Invalid param: %s\n", argv[1]);
     }

     return (0);
 }


//*****************************************************************************
//
// 命令处理函数
//
//*****************************************************************************
// 显示帮助，简单显示命令列表
int Cmd_help(int argc, char *argv[])
{
    CmdLineEntry *cmdEntry;

    ConsoleOut("\nAvailable commands\n");
    ConsoleOut("------------------\n");

    cmdEntry = (CmdLineEntry *) &g_kConsoleCmdTable[0];

    // 遍历整个命令表，打印出有提示信息的命令
    while (cmdEntry->cmdKeyword)
    {
        // 延时一下，等待控制台缓冲区空出足够的空间
        System_Delay(10);

        if (cmdEntry->cmdHelp)
            ConsoleOut("%s%s\n", cmdEntry->cmdKeyword, cmdEntry->cmdHelp);

        cmdEntry++;
    }

    return (0);
}

int Cmd_version(int argc, char *argv[])
{
    ManufVersion softVer = VersionInfo_GetSoftwareVersion();
    ManufVersion hardVer = VersionInfo_GetHardwareVersion();
    ConsoleOut("Cmd Ver: %d.%d.%d.%d\n", g_kCmdLineVersion.major, g_kCmdLineVersion.minor, g_kCmdLineVersion.revision, g_kCmdLineVersion.build);
    ConsoleOut("Soft Ver: %d.%d.%d.%d\n", softVer.major, softVer.minor, softVer.revision, softVer.build);
    ConsoleOut("Pcb Ver: %d.%d.%d.%d\n", hardVer.major, hardVer.minor, hardVer.revision, hardVer.build);

    return(0);
}

int Cmd_welcome(int argc, char *argv[])
{
    Console_Welcome();
    return(0);
}


// 显示参数
int Cmd_showparam(int argc, char *argv[])
{
    int i = 0;

    ConsoleOut("The params is:\n");
    for (i = 1; i < argc; i++)
    {
        ConsoleOut("    Param %d: %s\n", i, argv[i]);
    }

    return(0);
}


int Cmd_reset(int argc, char *argv[])
{
    Printf("\n\n\n");
    System_Delay(10);
    System_Reset();
    return (0);
}

int Cmd_trace(int argc, char *argv[])
{
    if (argv[1])
    {
        Trace_SetLevel(atoi(argv[1]));
    }
    else
    {
        Printf("L: %d\n", Trace_GetLevel());
    }

    return (0);
}

/** @} */
// 命令处理函数示例
int Cmd_demo(int argc, char *argv[])
{
    if (IsEqual(argv[1], "subcmd1"))
    {
        if (IsEqual(argv[2], "param"))
        {
            // 调用相关功能函数
            Printf("Exc: subcmd1 param");
        }
    }
    else if (IsEqual(argv[1], "subcmd2"))
    {
        Printf("Exc: subcmd2");
    }
    else if (0 == argv[1] || IsEqual(argv[1], "help") ||
             IsEqual(argv[1], "?"))
    {
        Printf("====== Sub commands ======\n");
        Printf(" mycmd subcmd1 param : Sub command description\n");
        Printf(" mycmd subcmd2       : Sub command description\n");
    }
    else
    {
        Printf("Invalid param: %s\n", argv[1]);
    }

    return (0);
}
