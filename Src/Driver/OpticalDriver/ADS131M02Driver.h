/*
 * ADS131M02.h
 *
 *  Created on: 2022年1月4日
 *      Author: lwq
 */

#ifndef SRC_DRIVER_OPTICALDRIVER_ADS131M02DRIVER_H_
#define SRC_DRIVER_OPTICALDRIVER_ADS131M02DRIVER_H_

#include "stm32f4xx.h"
#include "Common/Types.h"
#include "OpticalControl/OpticalControl.h"

#define NUM_ADS131M0X  1
#define ADS131M02_CONFIGURATION

#define CHANNEL_COUNT (2)   // ADS131M02 -> 2 Channels


// Select the desired MODE register settings...
// NOTE: These settings will be enforced and not modifiable during runtime!
#define ADS131M0X_BITRES        24
#define COMMAND_WLENGTH          1

	//#define WORD_LENGTH_16BIT_TRUNCATED
#define WORD_LENGTH_24BIT
	//#define WORD_LENGTH_32BIT_SIGN_EXTEND
	//#define WORD_LENGTH_32BIT_ZERO_PADDED

#define CRC_CCITT


#define DELAY_250US      250
#define DELAY_2048TCLK   2200	// Pulse duration for RESET  1TCLK = 1us, 2048TCLK ->> 2200us
#define DELAY_1TCLK      1   	// Pulse duration for SYNC   1ns
#define DELAY_5US        5      // Register default acquisition time
#define DELAY_500MS      500      // Waitting for DYDY go  high level



// Define WLENGTH_BYTES
#if defined WORD_LENGTH_16BIT_TRUNCATED
    #define WLENGTH_BYTES  2
    typedef int16_t external_adc_sample_t;

#elif defined WORD_LENGTH_24BIT
    #define WLENGTH_BYTES  3
    typedef int32_t external_adc_sample_t;

#elif defined WORD_LENGTH_32BIT_SIGN_EXTEND
    #define WLENGTH_BYTES  4
    typedef int32_t external_adc_sample_t;

#elif defined WORD_LENGTH_32BIT_ZERO_PADDED
    #define WLENGTH_BYTES  4
    typedef int32_t external_adc_sample_t;

#else
    #error Must define at least one WORD_LENGTH mode
#endif


#define NUM_REGISTERS                           ((uint8_t) 64)

#define OPCODE_NULL                             ((uint16_t) 0x0000)
#define OPCODE_RESET                            ((uint16_t) 0x0011)
#define OPCODE_RREG                             ((uint16_t) 0xA000)
#define OPCODE_WREG                             ((uint16_t) 0x6000)
#define OPCODE_STANDBY                          ((uint16_t) 0x0022)
#define OPCODE_WAKEUP                           ((uint16_t) 0x0033)
#define OPCODE_LOCK                             ((uint16_t) 0x0555)
#define OPCODE_UNLOCK                           ((uint16_t) 0x0655)

#define OPCODE_RESET_RESPONSE                   ((uint16_t) 0xFF22)

/* Register 0x00 (ID) definition - READ ONLY
 * -----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------
 * |   Bit 15   |   Bit 14   |   Bit 13   |   Bit 12   |   Bit 11   |   Bit 10   |    Bit 9   |    Bit 8   |    Bit 7   |    Bit 6   |    Bit 5   |    Bit 4   |    Bit 3   |    Bit 2   |    Bit 1   |    Bit 0   |
 * -----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------
 * |                    RESERVED[3:0]                  |                    CHANCNT[3:0]                   |                                               REVID[7:0]                                              |
 * -----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------
 */

/* ID register address & default value */
#define ID_ADDRESS                                                      ((uint8_t)  0x00)
#define ID_DEFAULT                                                      ((uint16_t) 0x2000 | (CHANNEL_COUNT << 8))  // NOTE: May change with future device revisions!

/* RESERVED field mask */
#define ID_RESERVED_MASK                                                ((uint16_t) 0xF000)

/* CHANCNT field mask & values */
#define ID_CHANCNT_MASK                                                 ((uint16_t) 0x0F00)
#define ID_CHANCNT_4CH                                                  ((uint16_t) 0x0004 << 8)
#define ID_CHANCNT_6CH                                                  ((uint16_t) 0x0006 << 8)
#define ID_CHANCNT_8CH                                                  ((uint16_t) 0x0008 << 8)

/* REVID field mask & values */
#define ID_REVID_MASK                                                   ((uint16_t) 0x00FF)
#define ID_REVID_REVA                                                   ((uint16_t) 0x0000 << 0)    // DEFAULT

/* Register 0x01 (STATUS) definition - READ ONLY
* -----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------
* |   Bit 15   |   Bit 14   |   Bit 13   |   Bit 12   |   Bit 11   |   Bit 10   |    Bit 9   |    Bit 8   |    Bit 7   |    Bit 6   |    Bit 5   |    Bit 4   |    Bit 3   |    Bit 2   |    Bit 1   |    Bit 0   |
* -----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------
* |    LOCK    |  F_RESYNC  |   REG_MAP  |   CRC_ERR  |  CRC_TYPE  |    RESET   |       WLENGTH[1:0]      |    DRDY7   |    DRDY6   |    DRDY5   |    DRDY4   |    DRDY3   |    DRDY2   |    DRDY1   |    DRDY0   |
* -----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------
*  NOTE 1: Bits 0 through 7 are hardware controlled. Reading these values multiple times may return different results.
*  NOTE 2: Bits 0 through 4 are RESERVED on the ADS131M04. These bits will always read 0.
*/

/* STATUS register address & default value */
#define STATUS_ADDRESS                                                  ((uint8_t)  0x01)
#define STATUS_DEFAULT                                                  ((uint16_t) 0x0500)

/* LOCK field mask & values */
#define STATUS_LOCK_MASK                                                ((uint16_t) 0x8000)
#define STATUS_LOCK_UNLOCKED                                            ((uint16_t) 0x0000 << 15)   // DEFAULT
#define STATUS_LOCK_LOCKED                                              ((uint16_t) 0x0001 << 15)

/* F_RESYNC field mask & values */
#define STATUS_F_RESYNC_MASK                                            ((uint16_t) 0x4000)
#define STATUS_F_RESYNC_NO_FAULT                                        ((uint16_t) 0x0000 << 14)   // DEFAULT
#define STATUS_F_RESYNC_FAULT                                           ((uint16_t) 0x0001 << 14)

/* REG_MAP field mask & values */
#define STATUS_REG_MAP_MASK                                             ((uint16_t) 0x2000)
#define STATUS_REG_MAP_NO_CHANGE_CRC                                    ((uint16_t) 0x0000 << 13)   // DEFAULT
#define STATUS_REG_MAP_CHANGED_CRC                                      ((uint16_t) 0x0001 << 13)

/* CRC_ERR field mask & values */
#define STATUS_CRC_ERR_MASK                                             ((uint16_t) 0x1000)
#define STATUS_CRC_ERR_NO_CRC_ERROR                                     ((uint16_t) 0x0000 << 12)   // DEFAULT
#define STATUS_CRC_ERR_INPUT_CRC_ERROR                                  ((uint16_t) 0x0001 << 12)

/* CRC_TYPE field mask & values */
#define STATUS_CRC_TYPE_MASK                                            ((uint16_t) 0x0800)
#define STATUS_CRC_TYPE_16BIT_CCITT                                     ((uint16_t) 0x0000 << 11)   // DEFAULT
#define STATUS_CRC_TYPE_16BIT_ANSI                                      ((uint16_t) 0x0001 << 11)

/* RESET field mask & values */
#define STATUS_RESET_MASK                                               ((uint16_t) 0x0400)
#define STATUS_RESET_NO_RESET                                           ((uint16_t) 0x0000 << 10)
#define STATUS_RESET_RESET_OCCURRED                                     ((uint16_t) 0x0001 << 10)   // DEFAULT

/* WLENGTH field mask & values */
#define STATUS_WLENGTH_MASK                                             ((uint16_t) 0x0300)
#define STATUS_WLENGTH_16BIT                                            ((uint16_t) 0x0000 << 8)
#define STATUS_WLENGTH_24BIT                                            ((uint16_t) 0x0001 << 8)    // DEFAULT
#define STATUS_WLENGTH_32BIT_LSB_ZEROES                                 ((uint16_t) 0x0002 << 8)
#define STATUS_WLENGTH_32BIT_MSB_SIGN_EXT                               ((uint16_t) 0x0003 << 8)

#if (CHANNEL_COUNT > 1)

/* DRDY1 field mask & values */
#define STATUS_DRDY1_MASK                                               ((uint16_t) 0x0002)
#define STATUS_DRDY1_NO_NEW_DATA                                        ((uint16_t) 0x0000 << 1)
#define STATUS_DRDY1_NEW_DATA                                           ((uint16_t) 0x0001 << 1)

#endif

/* DRDY0 field mask & values */
#define STATUS_DRDY0_MASK                                               ((uint16_t) 0x0001)
#define STATUS_DRDY0_NO_NEW_DATA                                        ((uint16_t) 0x0000 << 0)
#define STATUS_DRDY0_NEW_DATA                                           ((uint16_t) 0x0001 << 0)

/* Register 0x02 (MODE) definition
* -----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------
* |   Bit 15   |   Bit 14   |   Bit 13   |   Bit 12   |   Bit 11   |   Bit 10   |    Bit 9   |    Bit 8   |    Bit 7   |    Bit 6   |    Bit 5   |    Bit 4   |    Bit 3   |    Bit 2   |    Bit 1   |    Bit 0   |
* -----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------
* |      RESERVED0[1:0]     | REG_CRC_EN |  RX_CRC_EN |  CRC_TYPE  |    RESET   |       WLENGTH[1:0]      |             RESERVED1[2:0]           |   TIMEOUT  |       DRDY_SEL[1:0]     |  DRDY_HiZ  |  DRDY_FMT  |
* -----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------
*/

/* MODE register address & default value */
#define MODE_ADDRESS                                                    ((uint8_t)  0x02)
#define MODE_DEFAULT                                                    ((uint16_t) 0x0510)

/* RESERVED0 field mask */
#define MODE_RESERVED0_MASK                                             ((uint16_t) 0xC000)

/* REG_CRC_EN field mask & values */
#define MODE_REG_CRC_EN_MASK                                            ((uint16_t) 0x2000)
#define MODE_REG_CRC_EN_DISABLED                                        ((uint16_t) 0x0000 << 13)   // DEFAULT
#define MODE_REG_CRC_EN_ENABLED                                         ((uint16_t) 0x0001 << 13)

/* RX_CRC_EN field mask & values */
#define MODE_RX_CRC_EN_MASK                                             ((uint16_t) 0x1000)
#define MODE_RX_CRC_EN_DISABLED                                         ((uint16_t) 0x0000 << 12)   // DEFAULT
#define MODE_RX_CRC_EN_ENABLED                                          ((uint16_t) 0x0001 << 12)

/* CRC_TYPE field mask & values */
#define MODE_CRC_TYPE_MASK                                              ((uint16_t) 0x0800)
#define MODE_CRC_TYPE_16BIT_CCITT                                       ((uint16_t) 0x0000 << 11)   // DEFAULT
#define MODE_CRC_TYPE_16BIT_ANSI                                        ((uint16_t) 0x0001 << 11)

/* RESET field mask & values */
#define MODE_RESET_MASK                                                 ((uint16_t) 0x0400)
#define MODE_RESET_NO_RESET                                             ((uint16_t) 0x0000 << 10)
#define MODE_RESET_RESET_OCCURRED                                       ((uint16_t) 0x0001 << 10)   // DEFAULT

/* WLENGTH field mask & values */
#define MODE_WLENGTH_MASK                                               ((uint16_t) 0x0300)
#define MODE_WLENGTH_16BIT                                              ((uint16_t) 0x0000 << 8)
#define MODE_WLENGTH_24BIT                                              ((uint16_t) 0x0001 << 8)
#define MODE_WLENGTH_32BIT_LSB_ZEROES                                   ((uint16_t) 0x0002 << 8)
#define MODE_WLENGTH_32BIT_MSB_SIGN_EXT                                 ((uint16_t) 0x0003 << 8)

/* RESERVED1 field mask */
#define MODE_RESERVED1_MASK                                             ((uint16_t) 0x00E0)

/* TIMEOUT field mask & values */
#define MODE_TIMEOUT_MASK                                               ((uint16_t) 0x0010)
#define MODE_TIMEOUT_DISABLED                                           ((uint16_t) 0x0000 << 4)
#define MODE_TIMEOUT_ENABLED                                            ((uint16_t) 0x0001 << 4)    // DEFAULT

/* DRDY_SEL field mask & values */
#define MODE_DRDY_SEL_MASK                                              ((uint16_t) 0x000C)
#define MODE_DRDY_SEL_MOST_LAGGING                                      ((uint16_t) 0x0000 << 2)    // DEFAULT
#define MODE_DRDY_SEL_LOGIC_OR                                          ((uint16_t) 0x0001 << 2)
#define MODE_DRDY_SEL_MOST_LEADING                                      ((uint16_t) 0x0002 << 2)    // Alternative value: ((uint16_t) 0x0003 << 2)

/* DRDY_HiZ field mask & values */
#define MODE_DRDY_HiZ_MASK                                              ((uint16_t) 0x0002)
#define MODE_DRDY_HiZ_LOGIC_HIGH                                        ((uint16_t) 0x0000 << 1)    // DEFAULT
#define MODE_DRDY_HiZ_HIGH_IMPEDANCE                                    ((uint16_t) 0x0001 << 1)

/* DRDY_FMT field mask & values */
#define MODE_DRDY_FMT_MASK                                              ((uint16_t) 0x0001)
#define MODE_DRDY_FMT_LOGIC_LOW                                         ((uint16_t) 0x0000 << 0)    // DEFAULT
#define MODE_DRDY_FMT_NEG_PULSE_FIXED_WIDTH                             ((uint16_t) 0x0001 << 0)

/* Register 0x03 (CLOCK) definition
* -----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------
* |   Bit 15   |   Bit 14   |   Bit 13   |   Bit 12   |   Bit 11   |   Bit 10   |    Bit 9   |    Bit 8   |    Bit 7   |    Bit 6   |    Bit 5   |    Bit 4   |    Bit 3   |    Bit 2   |    Bit 1   |    Bit 0   |
* -----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------
* |   CH7_EN   |   CH6_EN   |   CH5_EN   |   CH4_EN   |   CH3_EN   |   CH2_EN   |   CH1_EN   |   CH0_EN   |  XTAL_DIS  |  EXTREF_EN |  RESERVED  |               OSR[2:0]               |         PWR[1:0]        |
* -----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------
*  NOTE 1: Bits 5 thorugh 7 and 12 through 15 are RESERVED on the ADS131M04.
*/

/* CLOCK register address & default value */
#define CLOCK_ADDRESS                                                   ((uint8_t)  0x03)

#if (CHANNEL_COUNT == 4)
#define CLOCK_DEFAULT                                                   ((uint16_t) 0x0F0E)
#endif
#if (CHANNEL_COUNT == 3)
#define CLOCK_DEFAULT                                                   ((uint16_t) 0x070E)
#endif
#if (CHANNEL_COUNT == 2)
#define CLOCK_DEFAULT                                                   ((uint16_t) 0x030E)
#endif
#if (CHANNEL_COUNT == 1)
#define CLOCK_DEFAULT                                                   ((uint16_t) 0x010E)
#endif

#if (CHANNEL_COUNT > 3)

    /* CH3_EN field mask & values */
    #define CLOCK_CH3_EN_MASK                                               ((uint16_t) 0x0800)
    #define CLOCK_CH3_EN_DISABLED                                           ((uint16_t) 0x0000 << 11)
    #define CLOCK_CH3_EN_ENABLED                                            ((uint16_t) 0x0001 << 11)

#endif
#if (CHANNEL_COUNT > 2)

    /* CH2_EN field mask & values */
    #define CLOCK_CH2_EN_MASK                                               ((uint16_t) 0x0400)
    #define CLOCK_CH2_EN_DISABLED                                           ((uint16_t) 0x0000 << 10)
    #define CLOCK_CH2_EN_ENABLED                                            ((uint16_t) 0x0001 << 10)

#endif
#if (CHANNEL_COUNT > 1)

/* CH1_EN field mask & values */
#define CLOCK_CH1_EN_MASK                                               ((uint16_t) 0x0200)
#define CLOCK_CH1_EN_DISABLED                                           ((uint16_t) 0x0000 << 9)
#define CLOCK_CH1_EN_ENABLED                                            ((uint16_t) 0x0001 << 9)

#endif

/* CH0_EN field mask & values */
#define CLOCK_CH0_EN_MASK                                               ((uint16_t) 0x0100)
#define CLOCK_CH0_EN_DISABLED                                           ((uint16_t) 0x0000 << 8)
#define CLOCK_CH0_EN_ENABLED                                            ((uint16_t) 0x0001 << 8)    // DEFAULT

/* RESERVED1 field mask */
#define CLOCK_RESERVED_MASK_M04                                         ((uint16_t) 0x00E0)
#define CLOCK_RESERVED_MASK_M08                                         ((uint16_t) 0x0020)

/* XTAL_DIS field mask & values (ADS131M08 Only) */
#define CLOCK_XTAL_DIS_MASK                                             ((uint16_t) 0x0080)
#define CLOCK_XTAL_DIS_ENABLED                                          ((uint16_t) 0x0000 << 7)    // DEFAULT
#define CLOCK_XTAL_DIS_DISABLED                                         ((uint16_t) 0x0001 << 7)

/* EXTREF_EN field mask & values (ADS131M08 Only) */
#define CLOCK_EXTREF_EN_MASK                                            ((uint16_t) 0x0040)
#define CLOCK_EXTREF_EN_DISABLED                                        ((uint16_t) 0x0000 << 6)    // DEFAULT
#define CLOCK_EXTREF_EN_ENABLED                                         ((uint16_t) 0x0001 << 6)

/* OSR field mask & values */
#define CLOCK_OSR_MASK                                                  ((uint16_t) 0x001C)
#define CLOCK_OSR_128                                                   ((uint16_t) 0x0000 << 2)
#define CLOCK_OSR_256                                                   ((uint16_t) 0x0001 << 2)
#define CLOCK_OSR_512                                                   ((uint16_t) 0x0002 << 2)
#define CLOCK_OSR_1024                                                  ((uint16_t) 0x0003 << 2)    // DEFAULT
#define CLOCK_OSR_2048                                                  ((uint16_t) 0x0004 << 2)
#define CLOCK_OSR_4096                                                  ((uint16_t) 0x0005 << 2)
#define CLOCK_OSR_8192                                                  ((uint16_t) 0x0006 << 2)
#define CLOCK_OSR_16384                                                 ((uint16_t) 0x0007 << 2)

/* PWR field mask & values */
#define CLOCK_PWR_MASK                                                  ((uint16_t) 0x0003)
#define CLOCK_PWR_VLP                                                   ((uint16_t) 0x0000 << 0)
#define CLOCK_PWR_LP                                                    ((uint16_t) 0x0001 << 0)
#define CLOCK_PWR_HR                                                    ((uint16_t) 0x0002 << 0)     // DEFAULT, Alternative value: ((uint16_t) 0x0003 << 2)

/* Register 0x04 (GAIN1) definition  it's only GAIN1 for ADS131M02
* -----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------
* |   Bit 15   |   Bit 14   |   Bit 13   |   Bit 12   |   Bit 11   |   Bit 10   |    Bit 9   |    Bit 8   |    Bit 7   |    Bit 6   |    Bit 5   |    Bit 4   |    Bit 3   |    Bit 2   |    Bit 1   |    Bit 0   |
* -----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------
* |  RESERVED0 |             PGAGAIN3[2:0]            |  RESERVED1 |             PGAGAIN2[2:0]            |  RESERVED2 |             PGAGAIN1[2:0]            |  RESERVED3 |             PGAGAIN0[2:0]            |
* -----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------
*/

/* GAIN1 register address & default value */
#define GAIN1_ADDRESS                                                   ((uint8_t)  0x04)
#define GAIN1_DEFAULT                                                   ((uint16_t) 0x0000)

/* RESERVED0 field mask & values */
#define GAIN1_RESERVED0_MASK                                            ((uint16_t) 0x8000)

/* PGAGAIN3 field mask & values */
#define GAIN1_PGAGAIN3_MASK                                             ((uint16_t) 0x7000)
#define GAIN1_PGAGAIN3_1                                                ((uint16_t) 0x0000 << 12)    // DEFAULT
#define GAIN1_PGAGAIN3_2                                                ((uint16_t) 0x0001 << 12)
#define GAIN1_PGAGAIN3_4                                                ((uint16_t) 0x0002 << 12)
#define GAIN1_PGAGAIN3_8                                                ((uint16_t) 0x0003 << 12)
#define GAIN1_PGAGAIN3_16                                               ((uint16_t) 0x0004 << 12)
#define GAIN1_PGAGAIN3_32                                               ((uint16_t) 0x0005 << 12)
#define GAIN1_PGAGAIN3_64                                               ((uint16_t) 0x0006 << 12)
#define GAIN1_PGAGAIN3_128                                              ((uint16_t) 0x0007 << 12)

/* RESERVED1 field mask & values */
#define GAIN1_RESERVED1_MASK                                            ((uint16_t) 0x0800)

/* PGAGAIN2 field mask & values */
#define GAIN1_PGAGAIN2_MASK                                             ((uint16_t) 0x0700)
#define GAIN1_PGAGAIN2_1                                                ((uint16_t) 0x0000 << 8)    // DEFAULT
#define GAIN1_PGAGAIN2_2                                                ((uint16_t) 0x0001 << 8)
#define GAIN1_PGAGAIN2_4                                                ((uint16_t) 0x0002 << 8)
#define GAIN1_PGAGAIN2_8                                                ((uint16_t) 0x0003 << 8)
#define GAIN1_PGAGAIN2_16                                               ((uint16_t) 0x0004 << 8)
#define GAIN1_PGAGAIN2_32                                               ((uint16_t) 0x0005 << 8)
#define GAIN1_PGAGAIN2_64                                               ((uint16_t) 0x0006 << 8)
#define GAIN1_PGAGAIN2_128                                              ((uint16_t) 0x0007 << 8)

/* RESERVED2 field mask & values */
#define GAIN1_RESERVED2_MASK                                            ((uint16_t) 0x0080)

/* PGAGAIN1 field mask & values */
#define GAIN1_PGAGAIN1_MASK                                             ((uint16_t) 0x0070)
#define GAIN1_PGAGAIN1_1                                                ((uint16_t) 0x0000 << 4)    // DEFAULT
#define GAIN1_PGAGAIN1_2                                                ((uint16_t) 0x0001 << 4)
#define GAIN1_PGAGAIN1_4                                                ((uint16_t) 0x0002 << 4)
#define GAIN1_PGAGAIN1_8                                                ((uint16_t) 0x0003 << 4)
#define GAIN1_PGAGAIN1_16                                               ((uint16_t) 0x0004 << 4)
#define GAIN1_PGAGAIN1_32                                               ((uint16_t) 0x0005 << 4)
#define GAIN1_PGAGAIN1_64                                               ((uint16_t) 0x0006 << 4)
#define GAIN1_PGAGAIN1_128                                              ((uint16_t) 0x0007 << 4)

/* RESERVED3 field mask & values */
#define GAIN1_RESERVED3_MASK                                            ((uint16_t) 0x0008)

/* PGAGAIN0 field mask & values */
#define GAIN1_PGAGAIN0_MASK                                             ((uint16_t) 0x0007)
#define GAIN1_PGAGAIN0_1                                                ((uint16_t) 0x0000 << 0)    // DEFAULT
#define GAIN1_PGAGAIN0_2                                                ((uint16_t) 0x0001 << 0)
#define GAIN1_PGAGAIN0_4                                                ((uint16_t) 0x0002 << 0)
#define GAIN1_PGAGAIN0_8                                                ((uint16_t) 0x0003 << 0)
#define GAIN1_PGAGAIN0_16                                               ((uint16_t) 0x0004 << 0)
#define GAIN1_PGAGAIN0_32                                               ((uint16_t) 0x0005 << 0)
#define GAIN1_PGAGAIN0_64                                               ((uint16_t) 0x0006 << 0)
#define GAIN1_PGAGAIN0_128                                              ((uint16_t) 0x0007 << 0)

/* Register 0x06 (CFG) definition
* -----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------
* |   Bit 15   |   Bit 14   |   Bit 13   |   Bit 12   |   Bit 11   |   Bit 10   |    Bit 9   |    Bit 8   |    Bit 7   |    Bit 6   |    Bit 5   |    Bit 4   |    Bit 3   |    Bit 2   |    Bit 1   |    Bit 0   |
* -----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------
* |            RESERVED0[2:0]            |                    GC_DLY[3:0]                    |    GC_EN   |  CD_ALLCH  |              CD_NUM[2:0]             |              CD_LEN[2:0]             |    CD_EN   |
* -----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------
*/

/* CFG register address & default value */
#define CFG_ADDRESS                                                     ((uint8_t)  0x06)
#define CFG_DEFAULT                                                     ((uint16_t) 0x0600)

/* RESERVED0 field mask & values */
#define CFG_RESERVED0_MASK                                              ((uint16_t) 0xE000)

/* GC_DLY field mask & values */
#define CFG_GC_DLY_MASK                                                 ((uint16_t) 0x1E00)
#define CFG_GC_DLY_2                                                    ((uint16_t) 0x0000 << 9)
#define CFG_GC_DLY_4                                                    ((uint16_t) 0x0001 << 9)
#define CFG_GC_DLY_8                                                    ((uint16_t) 0x0002 << 9)
#define CFG_GC_DLY_16                                                   ((uint16_t) 0x0003 << 9)    // DEFAULT
#define CFG_GC_DLY_32                                                   ((uint16_t) 0x0004 << 9)
#define CFG_GC_DLY_64                                                   ((uint16_t) 0x0005 << 9)
#define CFG_GC_DLY_128                                                  ((uint16_t) 0x0006 << 9)
#define CFG_GC_DLY_256                                                  ((uint16_t) 0x0007 << 9)
#define CFG_GC_DLY_512                                                  ((uint16_t) 0x0008 << 9)
#define CFG_GC_DLY_1024                                                 ((uint16_t) 0x0009 << 9)
#define CFG_GC_DLY_2048                                                 ((uint16_t) 0x000A << 9)
#define CFG_GC_DLY_4096                                                 ((uint16_t) 0x000B << 9)
#define CFG_GC_DLY_8192                                                 ((uint16_t) 0x000C << 9)
#define CFG_GC_DLY_16484                                                ((uint16_t) 0x000D << 9)
#define CFG_GC_DLY_32768                                                ((uint16_t) 0x000E << 9)
#define CFG_GC_DLY_65536                                                ((uint16_t) 0x000F << 9)

/* GC_EN field mask & values */
#define CFG_GC_EN_MASK                                                  ((uint16_t) 0x0100)
#define CFG_GC_EN_DISABLED                                              ((uint16_t) 0x0000 << 8)    // DEFAULT
#define CFG_GC_EN_ENABLED                                               ((uint16_t) 0x0001 << 8)

/* CD_ALLCH field mask & values */
#define CFG_CD_ALLCH_MASK                                               ((uint16_t) 0x0080)
#define CFG_CD_ALLCH_ANY_CHANNEL                                        ((uint16_t) 0x0000 << 7)    // DEFAULT
#define CFG_CD_ALLCH_ALL_CHANNELS                                       ((uint16_t) 0x0001 << 7)

/* CD_NUM field mask & values */
#define CFG_CD_NUM_MASK                                                 ((uint16_t) 0x0070)
#define CFG_CD_NUM_1                                                    ((uint16_t) 0x0000 << 4)    // DEFAULT
#define CFG_CD_NUM_2                                                    ((uint16_t) 0x0001 << 4)
#define CFG_CD_NUM_4                                                    ((uint16_t) 0x0002 << 4)
#define CFG_CD_NUM_8                                                    ((uint16_t) 0x0003 << 4)
#define CFG_CD_NUM_16                                                   ((uint16_t) 0x0004 << 4)
#define CFG_CD_NUM_32                                                   ((uint16_t) 0x0005 << 4)
#define CFG_CD_NUM_64                                                   ((uint16_t) 0x0006 << 4)
#define CFG_CD_NUM_128                                                  ((uint16_t) 0x0007 << 4)

/* CD_LEN field mask & values */
#define CFG_CD_LEN_MASK                                                 ((uint16_t) 0x000E)
#define CFG_CD_LEN_128                                                  ((uint16_t) 0x0000 << 1)    // DEFAULT
#define CFG_CD_LEN_256                                                  ((uint16_t) 0x0001 << 1)
#define CFG_CD_LEN_512                                                  ((uint16_t) 0x0002 << 1)
#define CFG_CD_LEN_768                                                  ((uint16_t) 0x0003 << 1)
#define CFG_CD_LEN_1280                                                 ((uint16_t) 0x0004 << 1)
#define CFG_CD_LEN_1792                                                 ((uint16_t) 0x0005 << 1)
#define CFG_CD_LEN_2560                                                 ((uint16_t) 0x0006 << 1)
#define CFG_CD_LEN_3584                                                 ((uint16_t) 0x0007 << 1)

/* CD_EN field mask & values */
#define CFG_CD_EN_MASK                                                  ((uint16_t) 0x0001)
#define CFG_CD_EN_DISABLED                                              ((uint16_t) 0x0000 << 0)    // DEFAULT
#define CFG_CD_EN_ENABLED                                               ((uint16_t) 0x0001 << 0)



/* Register 0x07 (THRSHLD_MSB) definition
* -----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------
* |   Bit 15   |   Bit 14   |   Bit 13   |   Bit 12   |   Bit 11   |   Bit 10   |    Bit 9   |    Bit 8   |    Bit 7   |    Bit 6   |    Bit 5   |    Bit 4   |    Bit 3   |    Bit 2   |    Bit 1   |    Bit 0   |
* -----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------
* |                                                                                                CD_TH_MSB[15:0]                                                                                                |
* -----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------
*/

/* THRSHLD_MSB register address & default value */
#define THRSHLD_MSB_ADDRESS                                             ((uint8_t)  0x07)
#define THRSHLD_MSB_DEFAULT                                             ((uint16_t) 0x0000)

/* CD_TH_MSB field mask & values */
#define THRSHLD_MSB_CD_TH_MSB_MASK                                      ((uint16_t) 0xFFFF)



/* Register 0x08 (THRSHLD_LSB) definition
* -----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------
* |   Bit 15   |   Bit 14   |   Bit 13   |   Bit 12   |   Bit 11   |   Bit 10   |    Bit 9   |    Bit 8   |    Bit 7   |    Bit 6   |    Bit 5   |    Bit 4   |    Bit 3   |    Bit 2   |    Bit 1   |    Bit 0   |
* -----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------
* |                                             CD_TH_LSB[7:0]                                            |                       RESERVED[7:4]               |                    DCBLOCK[3:0]                   |
* -----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------
*/

/* THRSHLD_LSB register address & default value */
#define THRSHLD_LSB_ADDRESS                                             ((uint8_t)  0x08)
#define THRSHLD_LSB_DEFAULT                                             ((uint16_t) 0x0000)

/* CD_TH_LSB field mask & values */
#define THRSHLD_LSB_CD_TH_LSB_MASK                                      ((uint16_t) 0xFF00)

/* RESERVED0 field mask & values */
#define THRSHLD_LSB_RESERVED0_MASK                                      ((uint16_t) 0x00F0)

/* DCBLOCK field mask & values */
#define THRSHLD_LSB_DCBLOCK_MASK                                        ((uint16_t) 0x000F)
#define THRSHLD_LSB_DCBLOCK_DISABLED                                    ((uint16_t) 0x0000 << 0)  // DC Block filter disabled
#define THRSHLD_LSB_DCBLOCK_181P0Hz                                     ((uint16_t) 0x0001 << 0)  // 181 Hz fc, 11.5 dB pass-band at 50 Hz, 10.1 dB pass-band at 60 Hz, 99% settled at 17 samples, fully settled at 88 samples
#define THRSHLD_LSB_DCBLOCK_84p8Hz                                      ((uint16_t) 0x0002 << 0)  // 84.8 Hz fc, 5.89 dB pass-band at 50 Hz, 4.77 dB pass-band at 60 Hz, 99% settled at 36 samples, fully settled at 187 samples
#define THRSHLD_LSB_DCBLOCK_41p1Hz                                      ((uint16_t) 0x0003 << 0)  // 41.1 Hz fc, 2.24 dB pass-band at 50 Hz, 1.67 dB pass-band at 60 Hz, 99% settled at 72 samples, fully settled at 387 samples
#define THRSHLD_LSB_DCBLOCK_20p2Hz                                      ((uint16_t) 0x0004 << 0)  // 20.2 Hz fc, 0.657 mdB pass-band at 50 Hz, 0.466dB pass-band at 60 Hz, 99% settled at 146 samples, fully settled at 786 samples
#define THRSHLD_LSB_DCBLOCK_10p0Hz                                      ((uint16_t) 0x0005 << 0)  // 10.0 Hz fc, 171 mdB pass-band at 50 Hz, 0.119dB pass-band at 60 Hz, 99% settled at 293 samples, fully settled at 1585 samples
#define THRSHLD_LSB_DCBLOCK_4p99Hz                                      ((uint16_t) 0x0006 << 0)  // 4.99 Hz fc, 43.1 mdB pass-band at 50 Hz, 29.9mdB pass-band at 60 Hz, 99% settled at 588 samples, fully settled at 3182 samples
#define THRSHLD_LSB_DCBLOCK_2p49Hz                                      ((uint16_t) 0x0007 << 0)  // 2.49 Hz fc, 10.8 mdB pass-band at 50 Hz, 7.47mdB pass-band at 60 Hz, 99% settled at 1178 samples, fully settled at 6376 samples
#define THRSHLD_LSB_DCBLOCK_1p24Hz                                      ((uint16_t) 0x0008 << 0)  // 1.24 Hz fc, 2.69 mdB pass-band at 50 Hz, 1.87mdB pass-band at 60 Hz, 99% settled at 2357 samples, fully settled at 12764 samples
#define THRSHLD_LSB_DCBLOCK_622mHz                                      ((uint16_t) 0x0009 << 0)  // 0.622 Hz fc, 671 udB pass-band at 50 Hz, 466udB pass-band at 60 Hz, 99% settled at 4714 samples, fully settled at 25540 samples
#define THRSHLD_LSB_DCBLOCK_311mHz                                      ((uint16_t) 0x000A << 0)  // 0.311 Hz fc, 168 udB pass-band at 50 Hz, 116 udB pass-band at 60 Hz, 99% settled at 9430 samples, fully settled at 51093 samples
#define THRSHLD_LSB_DCBLOCK_155mHz                                      ((uint16_t) 0x000B << 0)  // 0.155 Hz fc, 41.9 udB pass-band at 50 Hz, 29.1 udB pass-band at 60 Hz, 99% settled at 18861 samples, fully settled at 102202 samples
#define THRSHLD_LSB_DCBLOCK_77p7mHz                                     ((uint16_t) 0x000C << 0)  // 77.7 mHz fc, 10.5 udB pass-band at 50 Hz, 7.27 udB pass-band at 60 Hz, 99% settled at 37724 samples, fully settled at 204447 samples
#define THRSHLD_LSB_DCBLOCK_38p9mHz                                     ((uint16_t) 0x000D << 0)  // 38.9 mHz fc, 2.63 udB pass-band at 50 Hz, 1.82 udB pass-band at 60 Hz, 99% settled at 75450 samples, fully settled at 409156 samples
#define THRSHLD_LSB_DCBLOCK_19p4mHz                                     ((uint16_t) 0x000F << 0)  // 19.4 mHz fc, 655 ndB pass-band at 50 Hz, 455 ndB pass-band at 60 Hz, 99% settled at 150901 samples, fully settled at 820188 samples
#define THRSHLD_LSB_DCBLOCK_9p70mHz                                     ((uint16_t) 0x0007 << 0)  // 9.70 mHz fc, 164 ndB pass-band at 50 Hz, 114 ndB pass-band at 60 Hz, 99% settled at 301803 samples, fully settled at 1627730 samples



/* Register 0x09 (CH0_CFG) definition
* -----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------
* |   Bit 15   |   Bit 14   |   Bit 13   |   Bit 12   |   Bit 11   |   Bit 10   |    Bit 9   |    Bit 8   |    Bit 7   |    Bit 6   |    Bit 5   |    Bit 4   |    Bit 3   |    Bit 2   |    Bit 1   |    Bit 0   |
* -----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------
* |                                                           PHASE0[9:0]                                                           |                   RESERVED0[3:0]                  |        MUX0[1:0]        |
* -----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------
*/

/* CH0_CFG register address & default value */
#define CH0_CFG_ADDRESS                                                 ((uint8_t)  0x09)
#define CH0_CFG_DEFAULT                                                 ((uint16_t) 0x0000)

/* PHASE0 field mask & values */
#define CH0_CFG_PHASE0_MASK                                             ((uint16_t) 0xFFC0)
#define PHASEn_SHIFT                                                    (6)
#define SET_PHASE_REGISTER_VALUE(phase_shift)                           ((phase_shift )<<PHASEn_SHIFT)

/* RESERVED0 field mask & values */
#define CH0_CFG_RESERVED0_MASK                                          ((uint16_t) 0x003C)

/* MUX0 field mask & values */
#define CH0_CFG_MUX0_MASK                                               ((uint16_t) 0x0003)
#define CH0_CFG_MUX0_AIN0P_AIN0N                                        ((uint16_t) 0x0000 << 0)    // DEFAULT
#define CH0_CFG_MUX0_ADC_INPUT_SHORT                                    ((uint16_t) 0x0001 << 0)
#define CH0_CFG_MUX0_DC_DIAGNOSTIC                                      ((uint16_t) 0x0002 << 0)
#define CH0_CFG_MUX0_AC_DIAGNOSTIC                                      ((uint16_t) 0x0003 << 0)



/* Register 0x0A (CH0_OCAL_MSB) definition
* -----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------
* |   Bit 15   |   Bit 14   |   Bit 13   |   Bit 12   |   Bit 11   |   Bit 10   |    Bit 9   |    Bit 8   |    Bit 7   |    Bit 6   |    Bit 5   |    Bit 4   |    Bit 3   |    Bit 2   |    Bit 1   |    Bit 0   |
* -----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------
* |                                                                                                OCAL0_MSB[15:0]                                                                                                |
* -----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------
*/

/* CH0_OCAL_MSB register address & default value */
#define CH0_OCAL_MSB_ADDRESS                                            ((uint8_t)  0x0A)
#define CH0_OCAL_MSB_DEFAULT                                            ((uint16_t) 0x0000)

/* OCAL0_MSB field mask & values */
#define CH0_OCAL_MSB_OCAL0_MSB_MASK                                     ((uint16_t) 0xFFFF)



/* Register 0x0B (CH0_OCAL_LSB) definition
* -----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------
* |   Bit 15   |   Bit 14   |   Bit 13   |   Bit 12   |   Bit 11   |   Bit 10   |    Bit 9   |    Bit 8   |    Bit 7   |    Bit 6   |    Bit 5   |    Bit 4   |    Bit 3   |    Bit 2   |    Bit 1   |    Bit 0   |
* -----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------
* |                                             OCAL0_LSB[7:0]                                            |                                             RESERVED0[7:0]                                            |
* -----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------
*/

/* CH0_OCAL_LSB register address & default value */
#define CH0_OCAL_LSB_ADDRESS                                            ((uint8_t)  0x0B)
#define CH0_OCAL_LSB_DEFAULT                                            ((uint16_t) 0x0000)

/* OCAL0_LSB field mask & values */
#define CH0_OCAL_LSB_OCAL0_LSB_MASK                                     ((uint16_t) 0xFF00)

/* RESERVED0 field mask & values */
#define CH0_OCAL_LSB_RESERVED0_MASK                                     ((uint16_t) 0x00FF)



/* Register 0x0C (CH0_GCAL_MSB) definition
* -----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------
* |   Bit 15   |   Bit 14   |   Bit 13   |   Bit 12   |   Bit 11   |   Bit 10   |    Bit 9   |    Bit 8   |    Bit 7   |    Bit 6   |    Bit 5   |    Bit 4   |    Bit 3   |    Bit 2   |    Bit 1   |    Bit 0   |
* -----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------
* |                                                                                                GCAL0_MSB[15:0]                                                                                                |
* -----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------
*/

/* CH0_GCAL_MSB register address & default value */
#define CH0_GCAL_MSB_ADDRESS                                            ((uint8_t)  0x0C)
#define CH0_GCAL_MSB_DEFAULT                                            ((uint16_t) 0x8000)

/* GCAL0_MSB field mask & values */
#define CH0_GCAL_MSB_GCAL0_MSB_MASK                                     ((uint16_t) 0xFFFF)



/* Register 0x0D (CH0_GCAL_LSB) definition
* -----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------
* |   Bit 15   |   Bit 14   |   Bit 13   |   Bit 12   |   Bit 11   |   Bit 10   |    Bit 9   |    Bit 8   |    Bit 7   |    Bit 6   |    Bit 5   |    Bit 4   |    Bit 3   |    Bit 2   |    Bit 1   |    Bit 0   |
* -----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------
* |                                             GCAL0_LSB[7:0]                                            |                                             RESERVED0[7:0]                                            |
* -----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------
*/

/* CH0_GCAL_LSB register address & default value */
#define CH0_GCAL_LSB_ADDRESS                                            ((uint8_t)  0x0D)
#define CH0_GCAL_LSB_DEFAULT                                            ((uint16_t) 0x0000)

/* GCAL0_LSB field mask & values */
#define CH0_GCAL_LSB_GCAL0_LSB_MASK                                     ((uint16_t) 0xFF00)

/* RESERVED0 field mask & values */
#define CH0_GCAL_LSB_RESERVED0_MASK                                     ((uint16_t) 0x00FF)



#if (CHANNEL_COUNT > 1)

/* Register 0x0E (CH1_CFG) definition
* -----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------
* |   Bit 15   |   Bit 14   |   Bit 13   |   Bit 12   |   Bit 11   |   Bit 10   |    Bit 9   |    Bit 8   |    Bit 7   |    Bit 6   |    Bit 5   |    Bit 4   |    Bit 3   |    Bit 2   |    Bit 1   |    Bit 0   |
* -----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------
* |                                                           PHASE1[9:0]                                                           |                   RESERVED0[3:0]                  |        MUX1[1:0]        |
* -----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------
*/

/* CH1_CFG register address & default value */
#define CH1_CFG_ADDRESS                                                 ((uint8_t)  0x0E)
#define CH1_CFG_DEFAULT                                                 ((uint16_t) 0x0000)

/* PHASE1 field mask & values */
#define CH1_CFG_PHASE1_MASK                                             ((uint16_t) 0xFFC0)

/* RESERVED0 field mask & values */
#define CH1_CFG_RESERVED0_MASK                                          ((uint16_t) 0x003C)

/* MUX1 field mask & values */
#define CH1_CFG_MUX1_MASK                                               ((uint16_t) 0x0003)
#define CH1_CFG_MUX1_AIN1P_AIN1N                                        ((uint16_t) 0x0000 << 0)    // DEFAULT
#define CH1_CFG_MUX1_ADC_INPUT_SHORT                                    ((uint16_t) 0x0001 << 0)
#define CH1_CFG_MUX1_DC_DIAGNOSTIC                                      ((uint16_t) 0x0002 << 0)
#define CH1_CFG_MUX1_AC_DIAGNOSTIC                                      ((uint16_t) 0x0003 << 0)



/* Register 0x0F (CH1_OCAL_MSB) definition
* -----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------
* |   Bit 15   |   Bit 14   |   Bit 13   |   Bit 12   |   Bit 11   |   Bit 10   |    Bit 9   |    Bit 8   |    Bit 7   |    Bit 6   |    Bit 5   |    Bit 4   |    Bit 3   |    Bit 2   |    Bit 1   |    Bit 0   |
* -----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------
* |                                                                                                OCAL1_MSB[15:0]                                                                                                |
* -----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------
*/

/* CH1_OCAL_MSB register address & default value */
#define CH1_OCAL_MSB_ADDRESS                                            ((uint8_t)  0x0F)
#define CH1_OCAL_MSB_DEFAULT                                            ((uint16_t) 0x0000)

/* OCAL1_MSB field mask & values */
#define CH1_OCAL_MSB_OCAL1_MSB_MASK                                     ((uint16_t) 0xFFFF)



/* Register 0x10 (CH1_OCAL_LSB) definition
* -----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------
* |   Bit 15   |   Bit 14   |   Bit 13   |   Bit 12   |   Bit 11   |   Bit 10   |    Bit 9   |    Bit 8   |    Bit 7   |    Bit 6   |    Bit 5   |    Bit 4   |    Bit 3   |    Bit 2   |    Bit 1   |    Bit 0   |
* -----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------
* |                                             OCAL1_LSB[7:0]                                            |                                             RESERVED0[7:0]                                            |
* -----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------
*/

/* CH1_OCAL_LSB register address & default value */
#define CH1_OCAL_LSB_ADDRESS                                            ((uint8_t)  0x10)
#define CH1_OCAL_LSB_DEFAULT                                            ((uint16_t) 0x0000)

/* OCAL1_LSB field mask & values */
#define CH1_OCAL_LSB_OCAL1_LSB_MASK                                     ((uint16_t) 0xFF00)

/* RESERVED0 field mask & values */
#define CH1_OCAL_LSB_RESERVED0_MASK                                     ((uint16_t) 0x00FF)



/* Register 0x11 (CH1_GCAL_MSB) definition
* -----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------
* |   Bit 15   |   Bit 14   |   Bit 13   |   Bit 12   |   Bit 11   |   Bit 10   |    Bit 9   |    Bit 8   |    Bit 7   |    Bit 6   |    Bit 5   |    Bit 4   |    Bit 3   |    Bit 2   |    Bit 1   |    Bit 0   |
* -----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------
* |                                                                                                GCAL1_MSB[15:0]                                                                                                |
* -----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------
*/

/* CH1_GCAL_MSB register address & default value */
#define CH1_GCAL_MSB_ADDRESS                                            ((uint8_t)  0x11)
#define CH1_GCAL_MSB_DEFAULT                                            ((uint16_t) 0x8000)

/* GCAL1_MSB field mask & values */
#define CH1_GCAL_MSB_GCAL1_MSB_MASK                                     ((uint16_t) 0xFFFF)



/* Register 0x12 (CH1_GCAL_LSB) definition
* -----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------
* |   Bit 15   |   Bit 14   |   Bit 13   |   Bit 12   |   Bit 11   |   Bit 10   |    Bit 9   |    Bit 8   |    Bit 7   |    Bit 6   |    Bit 5   |    Bit 4   |    Bit 3   |    Bit 2   |    Bit 1   |    Bit 0   |
* -----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------
* |                                             GCAL1_LSB[7:0]                                            |                                             RESERVED0[7:0]                                            |
* -----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------
*/

/* CH1_GCAL_LSB register address & default value */
#define CH1_GCAL_LSB_ADDRESS                                            ((uint8_t)  0x12)
#define CH1_GCAL_LSB_DEFAULT                                            ((uint16_t) 0x0000)

/* GCAL1_LSB field mask & values */
#define CH1_GCAL_LSB_GCAL1_LSB_MASK                                     ((uint16_t) 0xFF00)

/* RESERVED0 field mask & values */
#define CH1_GCAL_LSB_RESERVED0_MASK                                     ((uint16_t) 0x00FF)

#endif


/* REGMAP_CRC register address & default value */
#define REGMAP_CRC_ADDRESS                                              ((uint8_t)  0x3E)
#define REGMAP_CRC_DEFAULT                                              ((uint16_t) 0x0000)

/* REG_CRC field mask & values */
#define REGMAP_CRC_REG_CRC_MASK                                         ((uint16_t) 0xFFFF)


//****************************************************************************
//
// Channel data structure
//
//****************************************************************************
typedef struct ADCStruct {
	uint16_t response;
	uint16_t crc;
	int32_t channel0;
#if (CHANNEL_COUNT > 1)
	int32_t channel1;
#endif
//	uint8_t rawBytes[(COMMAND_WLENGTH + CHANNEL_COUNT + 1) * WLENGTH_BYTES];

} adc_channel_data;

//****************************************************************************
//
// ADS connections
//
//****************************************************************************
typedef struct ADSConn {
    uint16_t CSport;
    uint16_t CSpin;
    uint16_t DRDYport;
    uint16_t DRDYpin;
} adc_connections;


void        ADS131M02Driver_Init(void);
void 		DRDYInterrupt_Init(void);
void 		DRDYInterrupt_Status(Bool ret);
Bool 		ADS131M02Driver_GetAD(OpticalSignalAD* data);
Bool        ADS131M02Driver_ADCStartup( void );
uint32_t    get_adc_gain_cal( uint8_t channel_number, uint8_t ads_index );
uint32_t    set_adc_gain_cal( uint8_t channel_number, uint32_t gain, uint8_t ads_index );
uint32_t    get_adc_offset_cal( uint8_t channel_number, uint8_t ads_index );
uint32_t    set_adc_offset_cal( uint8_t channel_number, uint32_t gain, uint8_t ads_index );
uint32_t    get_adc_phase( uint8_t channel_number, uint8_t ads_index );
uint32_t    set_adc_phase( uint8_t channel_number, uint32_t phase, uint8_t ads_index );
Bool        EnterCurrentDetectionMode( void );
Bool        ExitADClowPowerMode( void );
uint16_t    ADS131M02Driver_SendCommand( uint16_t op_code, uint8_t ads_index );
void        ADS131M02Driver_SendStandBy( void );
void        ADS131M02Driver_SendWakeup( void );
Bool        ADS131M02Driver_RequestData( adc_channel_data *ADCdata, uint8_t ads_index );
uint16_t    ADS131M02Driver_ReadSingleRegister( uint8_t address, uint8_t ads_index );
Bool        ADS131M02Driver_WriteSingleRegister( uint8_t address, uint16_t data, Bool readCheck, uint8_t ads_index );
Bool        ADS131M02Driver_LockRegisters( uint8_t ads_index );
Bool        ADS131M02Driver_UnLockRegisters( uint8_t ads_index );
void        ADS131M02Driver_ResetDevice( uint8_t ads_index );
void        ADS131M02Driver_RestoreRegisterDefaults( void );
uint16_t    calculateCRC( const uint8_t dataBytes[], uint8_t numberBytes, uint16_t initialValue );
Bool        verify_adc_CRC( adc_channel_data *ADCdata );

// Getter functions
uint16_t    getRegisterValue( uint8_t address, uint8_t ads_index );

// Helper functions
uint8_t     upperByte( uint16_t uint16_Word );
uint8_t     lowerByte( uint16_t uint16_Word );
uint16_t    combineBytes( uint8_t upperByte, uint8_t lowerByte );
int32_t     signExtend( uint8_t dataBytes[] );

float 		ADS131M02Driver_GetData(uint8_t index);



#endif /* SRC_DRIVER_OPTICALDRIVER_ADS131M02DRIVER_H_ */
