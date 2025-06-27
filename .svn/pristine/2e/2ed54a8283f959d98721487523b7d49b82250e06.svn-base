/*
 * ThermostatDeviceMap.c
 *
 *  Created on: 2019年8月16日
 *      Author: Administrator
 */

/*
 * ThermostatDeviceMap.c
 *
 *  Created on: 2017年11月16日
 *      Author: LIANG
 */

#include "ThermostatDeviceMap.h"
#include "Driver/System.h"
#include <string.h>
#include "Tracer/Trace.h"

//加热丝输出
static Bool ThermostatDeviceMap_SetOutputWay1(ThermostatDeviceDriver *deviceDriver, float level);

//风扇输出
static Bool ThermostatDeviceMap_SetOutputWay2(ThermostatDeviceDriver *deviceDriver, float level);

//电炉输出
static Bool ThermostatDeviceMap_SetOutputWay3(ThermostatDeviceDriver *deviceDriver, float level);

void ThermostatDeviceMap_Init(ThermostatDevice* device)
{
	//电炉加热 //TEMP_CTRL
	device[0].maxDutyCycle = 1;
	device[0].setOutputWayFunc = ThermostatDeviceMap_SetOutputWay1;
	device[0].deviceDriver.isExtra = FALSE;
	device[0].deviceDriver.mode = THERMOSTATDEVICEDRIVER_VIRTUAL_PWM;
	device[0].deviceDriver.port = GPIOE;
	device[0].deviceDriver.pin = GPIO_Pin_10;
	device[0].deviceDriver.gpioRcc = RCC_AHB1Periph_GPIOE;
	device[0].deviceDriver.modeConfig.IOConfig.open = Bit_SET;
	device[0].deviceDriver.modeConfig.IOConfig.close = Bit_RESET;
	ThermostatDevice_Init(&device[0]);

    device[1].maxDutyCycle = 1;
    device[1].setOutputWayFunc = ThermostatDeviceMap_SetOutputWay1;
    device[1].deviceDriver.isExtra = TRUE;
    device[1].deviceDriver.mode = THERMOSTATDEVICEDRIVER_VIRTUAL_PWM;
    device[1].deviceDriver.port = GPIOE;
	device[1].deviceDriver.pin = GPIO_Pin_11;
	device[1].deviceDriver.gpioRcc = RCC_AHB1Periph_GPIOE;
	device[1].deviceDriver.modeConfig.IOConfig.open = Bit_SET;
	device[1].deviceDriver.modeConfig.IOConfig.close = Bit_RESET;
    ThermostatDevice_Init(&device[1]);
    TRACE_INFO("\n thermdevice init over.");
}

static Bool ThermostatDeviceMap_SetOutputWay1(ThermostatDeviceDriver *deviceDriver, float level)
{
    TRACE_CODE("\n Output way 1");
    return ThermostatDeviceDriver_SetOutput(deviceDriver, level);
}

static Bool ThermostatDeviceMap_SetOutputWay2(ThermostatDeviceDriver *deviceDriver, float level)
{
    TRACE_CODE("\n Output way 2");
    if (0 != level)
    {
    	TRACE_INFO("\n Output way 2");
        level = 0.5 * level + 0.5;
        if (level < 0.75)
        {
            ThermostatDeviceDriver_SetOutput(deviceDriver, 1);
            System_Delay(200);
        }
    }
    return ThermostatDeviceDriver_SetOutput(deviceDriver, level);
}

static Bool ThermostatDeviceMap_SetOutputWay3(ThermostatDeviceDriver *deviceDriver, float level)
{
    TRACE_CODE("\n Output way 3");
    if (0 != level)
    {
        if (level < 0.05)
        {
        	level = 0.05;
        }
        else if(level > 0.7)
        {
			level = 0.7;
        }
    }
    return ThermostatDeviceDriver_SetOutput(deviceDriver, level);
}

char* ThermostatDeviceMap_GetName(Uint8 index)
{
    static char name[35] = "";
    switch(index)
    {
    case MEASUREMODULE_HEATER1:
        strcpy(name, "MEASUREMODULE_HEATER1");
        break;
    case MEASUREMODULE_HEATER2:
		strcpy(name, "MEASUREMODULE_HEATER2");
		break;
    default:
        strcpy(name, "NULL");
        break;
    }
    return name;
}
