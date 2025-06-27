/**
 * @file SolenoidValveInterface.h
 * @brief 电磁阀控制执行文件。
 * @version 1.0.0
 * @author xingfan
 * @date 2016-05-27
 */
#include "Tracer/Trace.h"
#include "Dncp/App/DscpSysDefine.h"
#include <string.h>
#include "SolenoidValve/ValveManager.h"
#include "SolenoidValve.h"
#include "LiquidDriver/ValveDriver.h"
#include "Driver/UltraSignalDriver/ChannelDriver.h"

/**
 * @brief 查询系统支持的总电磁阀数目。
 * @param dscp
 * @param data
 * @param len
 */
void SolenoidVale_GetTotalValves(DscpDevice* dscp, Byte* data, Uint16 len)
{
    Uint16 TotalValves = ValveManager_GetTotalValves();
    DscpDevice_SendResp(dscp, &TotalValves, sizeof(TotalValves));
}
/**
 * @brief 查询当前开启的阀门映射图。
 * @param dscp
 * @param data
 * @param len
 */
void SolenoidVale_GetValvesMap(DscpDevice* dscp, Byte* data, Uint16 len)
{
    Uint32 map = ValveManager_GetValvesMap();
    TRACE_MARK("\nGetMap: 0x%x\n",map);
    DscpDevice_SendResp(dscp, &map, sizeof(Uint32));
}
/**
 * @brief 设置要开启的阀门映射图。
 * @param dscp
 * @param data
 * @param len
 */
void SolenoidVale_SetValvesMap(DscpDevice* dscp, Byte* data, Uint16 len)
{
    Uint32 map =  0;
    Uint16 ret = DSCP_OK;

    memcpy(&map, data, sizeof(Uint32));

    if (FALSE == ValveManager_SetValvesMap(map))
    {
        ret = DSCP_ERROR;
    }
    DscpDevice_SendStatus(dscp, ret);
}

void SolenoidVale_SetValvesMapNormalOpen(DscpDevice* dscp, Byte* data, Uint16 len)
{
    Uint32 map =  0;
    Uint16 ret = DSCP_OK;

    memcpy(&map, data, sizeof(Uint32));
    TRACE_INFO("\nNormalOpen: 0x%x\n",map);
    if (FALSE == ValveManager_SetValvesMapNormalOpen(map))
    {
        ret = DSCP_ERROR;
    }
    DscpDevice_SendStatus(dscp, ret);
}

/**
 * @brief 查询系统支持的总电磁阀数目。
 * @param dscp
 * @param data
 * @param len
 */
void SolenoidVale_SetGainMap(DscpDevice* dscp, Byte* data, Uint16 len)
{
	Uint8 map =  0;
	Uint16 ret = DSCP_OK;
	memcpy(&map, data, sizeof(Uint8));
	TRACE_INFO("\nSetGainMap: 0x%x\n",map);
    ChannelDriver_SetGainMap(map);
    DscpDevice_SendStatus(dscp, ret);
}
/**
 * @brief 查询当前开启的阀门映射图。
 * @param dscp
 * @param data
 * @param len
 */
void SolenoidVale_GetGainMap(DscpDevice* dscp, Byte* data, Uint16 len)
{
    Uint8 map = ChannelDriver_GetGainMap();
    TRACE_INFO("\nGetGainMap: 0x%x\n",map);
    DscpDevice_SendResp(dscp, &map, sizeof(Uint8));
}

