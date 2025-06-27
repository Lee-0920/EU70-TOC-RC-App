/*
 * ADS131M02.c
 *
 *  Created on: 2022年1月4日
 *      Author: lwq
 */

#include <OpticalDriver/ADS131M02Driver.h>
#include <OpticalDriver/OpticalADCollect.h>
#include "Tracer/Trace.h"
#include "Driver/System.h"
#include "FreeRTOS.h"
#include "task.h"
#include "SystemConfig.h"
#include "Driver/System.h"


// CS
#define ADS131_CS_PIN           GPIO_Pin_8
#define ADS131_CS_GPIO          GPIOC
#define ADS131_CS_RCC           RCC_AHB1Periph_GPIOC
#define ADS131_CS_HIGH()        GPIO_SetBits(ADS131_CS_GPIO, ADS131_CS_PIN)
#define ADS131_CS_LOW()         GPIO_ResetBits(ADS131_CS_GPIO, ADS131_CS_PIN)

//RESET
#define ADS131_RESET_PIN           GPIO_Pin_12
#define ADS131_RESET_GPIO          GPIOE
#define ADS131_RESET_RCC           RCC_AHB1Periph_GPIOE
#define ADS131_RESET_HIGH()        GPIO_SetBits(ADS131_RESET_GPIO, ADS131_RESET_PIN)
#define ADS131_RESET_LOW()         GPIO_ResetBits(ADS131_RESET_GPIO, ADS131_RESET_PIN)

// SCLK
#define ADS131_SCLK_PIN         GPIO_Pin_2
#define ADS131_SCLK_GPIO        GPIOE
#define ADS131_SCLK_PinSource   GPIO_PinSource2
#define ADS131_SCLK_RCC         RCC_AHB1Periph_GPIOE
#define ADS131_SCLK_HIGH()      GPIO_SetBits(ADS131_SCLK_GPIO, ADS131_SCLK_PIN)
#define ADS131_SCLK_LOW()       GPIO_ResetBits(ADS131_SCLK_GPIO, ADS131_SCLK_PIN)

// DIN
#define ADS131_DIN_PIN          GPIO_Pin_6
#define ADS131_DIN_GPIO         GPIOE
#define ADS131_DIN_RCC          RCC_AHB1Periph_GPIOE
#define ADS131_DIN_PinSource    GPIO_PinSource6
#define ADS131_DIN_HIGH()       GPIO_SetBits(ADS131_DIN_GPIO, ADS131_DIN_PIN)
#define ADS131_DIN_LOW()        GPIO_ResetBits(ADS131_DIN_GPIO, ADS131_DIN_PIN)

// DOUT
#define ADS131_DOUT_PIN         GPIO_Pin_5
#define ADS131_DOUT_GPIO        GPIOE
#define ADS131_DOUT_RCC         RCC_AHB1Periph_GPIOE
#define ADS131_DOUT_PinSource   GPIO_PinSource5
#define ADS131_DOUT_READ()      GPIO_ReadInputDataBit(ADS131_DOUT_GPIO, ADS131_DOUT_PIN)

//DRDY
#define ADS131_DRDY_PIN    		GPIO_Pin_4
#define ADS131_DRDY_GPIO   		GPIOE
#define ADS131_DRDY_RCC         RCC_AHB1Periph_GPIOE
#define ADS131_DRDY_READ()      GPIO_ReadInputDataBit(ADS131_DRDY_GPIO, ADS131_DRDY_PIN)
#define ADS131_DRDY_PortSource  EXTI_PortSourceGPIOE
#define ADS131_DRDY_PinSource   EXTI_PinSource4
#define ADS131_DRDY_Line        EXTI_Line4
#define ADS131_DRDY_IRQ         EXTI4_IRQn
#define ADS131_DRDY_IRQHandle   EXTI4_IRQHandler


#define ADS131_DUMMY_BYTE 		0x00

#define SPIx					SPI4
#define GPIO_AF_SPIx 			GPIO_AF_SPI4
#define ADS131_SPIx_RCC 		RCC_APB2Periph_SPI4



//****************************************************************************
//
// Internal variables
//
//****************************************************************************

static adc_channel_data s_ADCdata[NUM_ADS131M0X];
static Uint32 tempData[4] = {0};
adc_connections  ADCconn[NUM_ADS131M0X];

// Array used to recall device register map configurations */
static uint16_t        registerMap[NUM_REGISTERS][NUM_ADS131M0X];
external_adc_sample_t  channel_data[NUM_ADS131M0X][CHANNEL_COUNT];
extern const uint8_t SMCLKOFF[];

static float channelData[4] = {0};

#ifdef ENABLE_CRC_IN
    uint8_t dataTx[(COMMAND_WLENGTH + CRC_IN_WLENGTH + CHANNEL_COUNT + 1) * WLENGTH_BYTES] = { 0 };      // 2 words, up to 4 bytes each = 8 bytes maximum
    uint8_t dataRx[(COMMAND_WLENGTH + CRC_IN_WLENGTH + CHANNEL_COUNT + 1) * WLENGTH_BYTES] = { 0 };
#else
    uint8_t dataTx[(COMMAND_WLENGTH + CHANNEL_COUNT + 1) * WLENGTH_BYTES] = { 0 };      // 1 word, up to 4 bytes long = 4 bytes maximum
    uint8_t dataRx[(COMMAND_WLENGTH + CHANNEL_COUNT + 1) * WLENGTH_BYTES] = { 0 };
#endif

#define  numFrameWords  4
unsigned long DummyWord[numFrameWords] = { 0x00000000, 0x00000000, 0x00000000, 0x00000000,};


//****************************************************************************
//
// Internal function prototypes
//
//****************************************************************************

uint8_t     buildSPIarray( const uint16_t opcodeArray[], uint8_t numberOpcodes, uint8_t byteArray[] );
uint16_t    enforce_selected_device_modes( uint16_t data );



/************************************************************************************//**
 *
 * @brief getRESET()
 *          Returns the state of the MCU's ADC_RESET_SYNC GPIO pin
 *
 * @return Boolean level of /RESET pin (false = low, true = high)
 */
Bool getRESET( void )
{
    return (Bool) GPIO_ReadOutputDataBit( ADS131_RESET_GPIO, ADS131_RESET_PIN );
}

/************************************************************************************//**
 *
 * @brief setRESET()
 *            Sets the state of the MCU ADC_RESET_SYNC GPIO pin
 *
 * @param[in]   state   level of /RESET pin (false = low, true = high)
 *
 * @return      None
 */

void setRESET( Bool state )
{
    if ( state )
    	GPIO_SetBits(ADS131_RESET_GPIO, ADS131_RESET_PIN);
    else
    	GPIO_ResetBits(ADS131_RESET_GPIO, ADS131_RESET_PIN);
}

/************************************************************************************//**
 *
 * @brief toggleRESET()
 *            Pulses the /RESET GPIO pin low
 *
 * @return      None
 */
void toggleRESET( void )
{
    int ads_index;

    ADS131_RESET_LOW();
    // Minimum nRESET_SYNC width: 2048 tCLKs
#ifndef __ICCARM__  // CCS Project
    usleep( DELAY_2048TCLK );
#else               // IAR Project
    System_DelayUs( DELAY_2048TCLK );
#endif

    ADS131_RESET_HIGH();

    // NOTE: The ADS131M0x's next response word should be (0xFF20 | CHANCNT).  0xFF22 here
    // A different response may be an indication that the device did not reset.

    // Minimum wait of 5usec before communicating through SPI
#ifndef __ICCARM__  // CCS Project
    usleep( DELAY_5US );
#else               // IAR Project
    System_DelayUs( DELAY_5US );
#endif

    // Update register array
    ADS131M02Driver_RestoreRegisterDefaults();

//    for ( ads_index = 0; ads_index < NUM_ADS131M0X; ads_index++ ) {
//        // Write to MODE register to enforce mode settings
//        ADS131M02Driver_WriteSingleRegister(MODE_ADDRESS, (MODE_DEFAULT & ~MODE_RESET_RESET_OCCURRED), TRUE, ads_index );
//    }
}

/************************************************************************************//**
 *
 * @brief toggleSYNC()
 *            Pulses the /RESET GPIO for a sync pulse
 *
 * @return      None
 */
void toggleSYNC( void )
{
	ADS131_RESET_LOW();
    // Minimum Sync pulse between 1 and 2047 tCLKs, use 1 tCLKs
#ifndef __ICCARM__  // CCS Project
    usleep( DELAY_1TCLK );
#else               // IAR Project
    System_DelayUs( DELAY_1TCLK );
#endif

    ADS131_RESET_HIGH();
}

/************************************************************************************//**
 *
 * @brief ADS131M02Driver_SendWakeup()
 *            Sends Wakeup Command through SPI
 *
 * @return      None
 */
void ADS131M02Driver_SendWakeup( void )
{
    int ads_index;
    uint8_t dataTx = OPCODE_WAKEUP;

    for ( ads_index = 0; ads_index < NUM_ADS131M0X; ads_index++ ) {
        // Wakeup device
        ADS131M02Driver_SendCommand( dataTx, ads_index );
    }
}

/************************************************************************************//**
 *
 * @brief ADS131M02Driver_SendStandBy()
 *            Sends StandBy Command through SPI
 *
 * @return      None
 */
void ADS131M02Driver_SendStandBy( void )
{
    int ads_index;
    uint8_t dataTx = OPCODE_STANDBY;

    for ( ads_index = 0; ads_index < NUM_ADS131M0X; ads_index++ ) {
        // Wakeup device
        ADS131M02Driver_SendCommand( dataTx, ads_index );
    }
}


/**
 * @brief ADS1146发送字节
 * @param
 */
static uint8_t ADS131SendByte(uint8_t senddata)
{
    uint8_t i = 0;
    uint8_t receivedata = 0;

    for (i = 0; i < 8; i++)
    {
        // 发送

        ADS131_SCLK_HIGH();
        System_DelayNs(3); //100ns

        if (senddata & 0x80)
        {
            ADS131_DIN_HIGH();
//            TRACE_INFO("\n@");
        }
        else
        {
            ADS131_DIN_LOW();
//            TRACE_INFO("\n*");
        }
        senddata <<= 1;

        ADS131_SCLK_LOW();
//        System_DelayNs(10);

        // 接收
        receivedata <<= 1;
        if (ADS131_DOUT_READ())
		{
			receivedata |= 0x01;
		}
    }
//    TRACE_INFO("\n**********%d",receivedata);

    return receivedata;
}

static void ADS131SendReceiveByte(uint8_t* tx, uint8_t* rx, uint8_t numberByte)
{
	for(uint8_t i = 0; i < numberByte; i++)
	{
		rx[i] = ADS131SendByte(tx[i]);
	}
}

/**
 * @brief 读取ADS131 24位数据
 * @param
 */
static uint32_t ADS131Read24bitData()
{
    uint32_t read_data = 0;

    // 读数据
    read_data = 0;
    read_data |= ADS131SendByte(ADS131_DUMMY_BYTE) << 16;
    read_data |= ADS131SendByte(ADS131_DUMMY_BYTE) << 8;
    read_data |= ADS131SendByte(ADS131_DUMMY_BYTE);

    return (read_data & 0xFFFFFF);
}

//*****************************************************************************
//
//! Getter function to access registerMap array from outside of this module.
//!
//! @fn uint16_t getRegisterValue( uint8_t address, uint8_t ads_index )
//!
//! @param[in] address   is the ADS register address to read.
//! @param[in] ads_index is the ADS device to read.
//!
//! NOTE: The internal registerMap arrays stores the last know register value,
//! since the last read or write operation to that register. This function
//! does not communicate with the device to retrieve the current register value.
//! For the most up-to-date register data or retrieving the value of a hardware
//! controlled register, it is recommend to use ADS131M02Driver_ReadSingleRegister() to read the
//! current register value.
//!
//! @return unsigned 16-bit register value.
//
//*****************************************************************************
uint16_t getRegisterValue( uint8_t address, uint8_t ads_index )
{
//    assert(address < NUM_REGISTERS);
    return registerMap[address][ads_index];
}


void ADS131M02Driver_Init(void)
{
    GPIO_InitTypeDef GPIO_InitStructure;

    //时钟配置
    RCC_AHB1PeriphClockCmd(ADS131_CS_RCC |
    ADS131_SCLK_RCC |
    ADS131_DIN_RCC |
    ADS131_DOUT_RCC |
	ADS131_RESET_RCC |
	ADS131_DRDY_RCC |
	ADS131_SPIx_RCC, ENABLE);
//    RCC_AHB2PeriphClockCmd(ADS131_SPIx_RCC, ENABLE);

    //IO配置
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
    GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;

    // CS
    GPIO_InitStructure.GPIO_Pin = ADS131_CS_PIN;
    GPIO_Init(ADS131_CS_GPIO, &GPIO_InitStructure);

    // RESET
	GPIO_InitStructure.GPIO_Pin = ADS131_RESET_PIN;
	GPIO_Init(ADS131_RESET_GPIO, &GPIO_InitStructure);

    // SCLK
    GPIO_InitStructure.GPIO_Pin = ADS131_SCLK_PIN;
    GPIO_Init(ADS131_SCLK_GPIO, &GPIO_InitStructure);
//    GPIO_PinAFConfig(ADS131_SCLK_GPIO, ADS131_SCLK_PinSource, GPIO_AF_SPIx);

    // DIN
    GPIO_InitStructure.GPIO_Pin = ADS131_DIN_PIN;
    GPIO_Init(ADS131_DIN_GPIO, &GPIO_InitStructure);
//    GPIO_PinAFConfig(ADS131_DIN_GPIO, ADS131_DIN_PinSource, GPIO_AF_SPIx);

    // DOUT
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN;
    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
    GPIO_InitStructure.GPIO_Pin = ADS131_DOUT_PIN;
    GPIO_Init(ADS131_DOUT_GPIO, &GPIO_InitStructure);
//    GPIO_PinAFConfig(ADS131_DOUT_GPIO, ADS131_DOUT_PinSource, GPIO_AF_SPIx);

    //DRDY
    GPIO_InitStructure.GPIO_Pin = ADS131_DRDY_PIN;
	GPIO_Init(ADS131_DRDY_GPIO, &GPIO_InitStructure);



    // 初始引脚
    ADS131_CS_LOW();
    ADS131_SCLK_HIGH();
    ADS131_DIN_HIGH();
    ADS131_RESET_HIGH();

//    // 初始化SPI模块
//	SPI_InitTypeDef SPI_InitStructure;
//	SPI_InitStructure.SPI_Direction = SPI_Direction_2Lines_FullDuplex;       // SPI总线设置使用两条线，一条用于Rx，另一条用于Tx
//	SPI_InitStructure.SPI_Mode = SPI_Mode_Master;                                // STM32是主服务，tlc5940作为从服务
//	SPI_InitStructure.SPI_DataSize = SPI_DataSize_8b;                             // 使用8位数据传输
//	SPI_InitStructure.SPI_CPOL = SPI_CPOL_Low;                                    // TLC5940时钟空闲时低
//	SPI_InitStructure.SPI_CPHA = SPI_CPHA_1Edge;                                // TLC5940使用第一个时钟过渡作为“捕获边缘”
//	SPI_InitStructure.SPI_NSS = SPI_NSS_Soft;                                        // 软件slave-select操作
//	SPI_InitStructure.SPI_BaudRatePrescaler = SPI_BaudRatePrescaler_8; // 设置预定标器
//	SPI_InitStructure.SPI_FirstBit = SPI_FirstBit_MSB;                             // TLC5940数据先从MSB传输
//	SPI_InitStructure.SPI_CRCPolynomial = 0;                                          // 没有使用CRC
//
//	SPI_Init(SPIx, &SPI_InitStructure);                                                    // 初始化SPI2外围设备
//	SPI_SSOutputCmd(SPIx, ENABLE);                                                 // 将SS Pin设置为输出(主模式)
//	SPI_Cmd(SPIx, ENABLE);


    if(!ADS131M02Driver_ADCStartup())
    {
    	TRACE_INFO("\n ADS131 Init failed");
    }

    //DRDY IO中断初始化
	DRDYInterrupt_Init();

}

Bool ADS131M02Driver_ADCStartup(void)
{
    int adc_index;
    uint8_t loop = 20;
    int timeout = DELAY_500MS / loop;
	/* (OPTIONAL) Provide additional delay time for power supply settling */
#ifndef __ICCARM__  // CCS Project
    usleep( DELAY_5US );
#else               // IAR Project
   System_DelayUs(DELAY_250US);
#endif

   //等待DRDY拉高
   while(!ADS131_DRDY_READ())
   {
	   System_Delay(loop);
	   if(!timeout)
	   {
		   TRACE_INFO("\n DRDY not ready");
		   return FALSE;
	   }
	   timeout--;
   }

   ADS131_CS_LOW();

//   ADS131M02Driver_ResetDevice(0);

	/* (OPTIONAL) Toggle nRESET pin to ensure default register settings. */
	/* NOTE: This also ensures that the device registers are unlocked.	 */
	toggleRESET();

    /* (REQUIRED) Initialize internal 'registerMap' array with device default settings */
	ADS131M02Driver_RestoreRegisterDefaults();


    for ( adc_index = 0; adc_index < NUM_ADS131M0X; adc_index++ )
    {
	/* (OPTIONAL) Validate first response word when beginning SPI communication: (0xFF20 | CHANCNT) */
	uint16_t response = ADS131M02Driver_SendCommand( OPCODE_NULL, adc_index );
	TRACE_INFO("\n rs:%x", response);

	/* Disable ADC channels to send short frames for configuring ADC */
//	#if defined( ADS131M02_CONFIGURATION )
//		if ( !ADS131M02Driver_WriteSingleRegister( CLOCK_ADDRESS, CLOCK_CH1_EN_DISABLED | CLOCK_CH0_EN_DISABLED
//								| CLOCK_OSR_512 | CLOCK_PWR_VLP, TRUE, adc_index ) )
////			(CLOCK_DEFAULT & ~(CLOCK_CH1_EN_MASK | CLOCK_CH0_EN_MASK | CLOCK_OSR_MASK))| CLOCK_OSR_128 | CLOCK_PWR_VLP
////			return FALSE;
//			;
//    #endif

//    #ifdef CRC_CCITT
//        if ( !ADS131M02Driver_WriteSingleRegister( MODE_ADDRESS, MODE_CRC_TYPE_16BIT_CCITT |
//																 MODE_WLENGTH_24BIT | MODE_DRDY_SEL_MOST_LAGGING |
//																 MODE_DRDY_HiZ_LOGIC_HIGH | MODE_DRDY_FMT_LOGIC_LOW, TRUE, adc_index ) )
////            return FALSE;
//        	;
//    #else
//        if ( !ADS131M02Driver_WriteSingleRegister( MODE_ADDRESS, MODE_CRC_TYPE_16BIT_ANSI | MODE_WLENGTH_24BIT | MODE_DRDY_SEL_MOST_LAGGING | MODE_DRDY_HiZ_LOGIC_HIGH | MODE_DRDY_FMT_LOGIC_LOW, TRUE, adc_index ) )
//            return FALSE;
//    #endif
    }

////    #if defined( ADS131M02_CONFIGURATION )
////		// First ADS131M02
////		if ( !ADS131M02Driver_WriteSingleRegister( GAIN1_ADDRESS, GAIN1_PGAGAIN0_1 | GAIN1_PGAGAIN1_1, TRUE, 0 ) )
////			return FALSE;
////    #endif
//
//    System_DelayUs(DELAY_5US);  //DELAY_5US
//
//	// SAMPLE_RATES = 5859, 6000, 6400, 7812 or 8000
//	#if defined( ADS131M02_CONFIGURATION )
//		if ( !ADS131M02Driver_WriteSingleRegister( CLOCK_ADDRESS, CLOCK_CH1_EN_ENABLED | CLOCK_CH0_EN_ENABLED | CLOCK_OSR_512 | CLOCK_PWR_VLP, TRUE, 0 ) )
//			return FALSE;
//    #endif

//    if ( !writeSingleRegister( THRSHLD_MSB_ADDRESS, (CURRENT_DETECTION_THRESHOLD_VALUE & 0x00FFFF00) >> 8, TRUE, adc_index ) )
//    {
//        return FALSE;
//    }


	ADS131_CS_HIGH();

    return TRUE;

}

//*****************************************************************************
//
//! Reads the gain calibration of the specified channel
//!
//! @fn uint32_t get_adc_gain_cal( uint8_t channel_number )
//!
//! @param channel_number is the 8-bit channel to read.
//! @param ads_index is the ADS device to read.
//!
//! @return Returns the 32-bit gain calibration of the requested channel.
//
//*****************************************************************************
uint32_t get_adc_gain_cal( uint8_t channel_number, uint8_t ads_index )
{
    uint32_t gain_value;

    gain_value = (ADS131M02Driver_ReadSingleRegister( CH0_GCAL_MSB_ADDRESS +  channel_number * (CH1_GCAL_MSB_ADDRESS - CH0_GCAL_MSB_ADDRESS), ads_index )
                    & CH0_GCAL_MSB_GCAL0_MSB_MASK) << 8;
    gain_value |= (ADS131M02Driver_ReadSingleRegister( CH0_GCAL_LSB_ADDRESS +  channel_number * (CH1_GCAL_LSB_ADDRESS - CH0_GCAL_LSB_ADDRESS), ads_index )
                    & CH0_GCAL_LSB_GCAL0_LSB_MASK) >> 8;
    return gain_value;
}

//*****************************************************************************
//
//! Writes the gain calibration of the specified channel
//!
//! @fn uint32_t set_adc_gain_cal( uint8_t channel_number )
//!
//! @param channel_number is the 8-bit channel to read.
//!
//! @return Return TRUE, if succesful.
//
//*****************************************************************************
uint32_t set_adc_gain_cal( uint8_t channel_number, uint32_t gain, uint8_t ads_index )
{
    if ( !ADS131M02Driver_WriteSingleRegister( CH0_GCAL_MSB_ADDRESS +  channel_number * (CH1_GCAL_MSB_ADDRESS - CH0_GCAL_MSB_ADDRESS),
                                ((uint16_t) (gain >> 8)) & CH0_GCAL_MSB_GCAL0_MSB_MASK, TRUE, ads_index ) )
        return FALSE;
    if ( !ADS131M02Driver_WriteSingleRegister( CH0_GCAL_LSB_ADDRESS +  channel_number * (CH1_GCAL_LSB_ADDRESS - CH0_GCAL_LSB_ADDRESS),
                                (((uint16_t) gain) << 8) & CH0_GCAL_LSB_GCAL0_LSB_MASK, TRUE, ads_index ) )
        return FALSE;
    return TRUE;
}

//*****************************************************************************
//
//! Reads the offset calibration of the specified channel
//!
//! @fn uint32_t get_adc_offset_cal( uint8_t channel_number )
//!
//! @param channel_number is the 8-bit channel to read.
//!
//! @return Returns the 32-bit offset calibration of the requested channel.
//
//*****************************************************************************
uint32_t get_adc_offset_cal( uint8_t channel_number, uint8_t ads_index )
{
    uint32_t gain_value;

    gain_value = (ADS131M02Driver_ReadSingleRegister( CH0_OCAL_MSB_ADDRESS +  channel_number * (CH1_OCAL_MSB_ADDRESS - CH0_OCAL_MSB_ADDRESS), ads_index )
                    & CH0_OCAL_MSB_OCAL0_MSB_MASK) << 8;
    gain_value |= (ADS131M02Driver_ReadSingleRegister( CH0_OCAL_LSB_ADDRESS +  channel_number * (CH1_OCAL_LSB_ADDRESS - CH0_OCAL_LSB_ADDRESS), ads_index )
                    & CH0_OCAL_LSB_OCAL0_LSB_MASK) >> 8;
    return gain_value;
}

//*****************************************************************************
//
//! Writes the offset calibration of the specified channel
//!
//! @fn uint32_t set_adc_offset_cal( uint8_t channel_number )
//!
//! @param channel_number is the 8-bit channel to read.
//!
//! @return Return TRUE, if succesful.
//
//*****************************************************************************
uint32_t set_adc_offset_cal( uint8_t channel_number, uint32_t offset, uint8_t ads_index )
{
    if ( !ADS131M02Driver_WriteSingleRegister( CH0_OCAL_MSB_ADDRESS +  channel_number * (CH1_OCAL_MSB_ADDRESS - CH0_OCAL_MSB_ADDRESS),
                                ((uint16_t) (offset >> 8)) & CH0_OCAL_MSB_OCAL0_MSB_MASK, TRUE, ads_index ) )
        return FALSE;
    if ( !ADS131M02Driver_WriteSingleRegister( CH0_OCAL_LSB_ADDRESS +  channel_number * (CH1_OCAL_LSB_ADDRESS - CH0_OCAL_LSB_ADDRESS),
                                (((uint16_t) offset) << 8) & CH0_OCAL_LSB_OCAL0_LSB_MASK, TRUE, ads_index ) )
        return FALSE;
    return TRUE;
}


//*****************************************************************************
//
//! Reads the phase of the specified channel
//!
//! @fn uint32_t get_adc_phase( uint8_t channel_number )
//!
//! @param channel_number is the 8-bit channel to read.
//!
//! @return Returns the 32-bit phase of the requested channel.
//
//*****************************************************************************
uint32_t get_adc_phase( uint8_t channel_number, uint8_t ads_index )
{
    uint32_t phase_value;

    phase_value = (ADS131M02Driver_ReadSingleRegister( CH0_CFG_ADDRESS +  channel_number * (CH1_CFG_ADDRESS - CH0_CFG_ADDRESS), ads_index )
                    && CH0_CFG_PHASE0_MASK) >> PHASEn_SHIFT;
    return phase_value;
}

//*****************************************************************************
//
//! Writes phase of the specified channel
//!
//! @fn uint32_t set_adc_phase( uint8_t channel_number )
//!
//! @param channel_number is the 8-bit channel to read.
//!
//! @return Return TRUE, if succesful.
//
//*****************************************************************************
uint32_t set_adc_phase( uint8_t channel_number, uint32_t channel_phase, uint8_t ads_index )
{
    uint16_t regValue;

    regValue = ADS131M02Driver_ReadSingleRegister( CH0_CFG_ADDRESS +  channel_number * (CH1_CFG_ADDRESS - CH0_CFG_ADDRESS), ads_index );
    if ( !ADS131M02Driver_WriteSingleRegister( CH0_CFG_ADDRESS +  channel_number * (CH1_CFG_ADDRESS - CH0_CFG_ADDRESS),
                                ((channel_phase << PHASEn_SHIFT) && CH0_CFG_PHASE0_MASK) | regValue, TRUE, ads_index ) )
        return FALSE;
    return TRUE;
}

//*****************************************************************************
//
//! Reads the contents of a single register at the specified address.
//!
//! @fn uint16_t ADS131M02Driver_ReadSingleRegister(uint8_t address)
//!
//! @param address is the 8-bit address of the register to read.
//! @param ads_index is the index of the ADS, if multiple ADS share one SPI bus.
//!
//! @return Returns the 8-bit register read result.
//
//*****************************************************************************
uint16_t ADS131M02Driver_ReadSingleRegister( uint8_t address, uint8_t ads_index )
{
	/* Check that the register address is in range */
//	assert(address < NUM_REGISTERS);

//	uint16_t cmdResponse = 0;


// Build TX and RX byte array
	uint16_t opcode[COMMAND_WLENGTH * WLENGTH_BYTES] = {0};

	opcode[0] = OPCODE_RREG | (((uint16_t) address) << 7);
    opcode[1] = 0x0000;
    opcode[2] = 0x0000;

    uint8_t numberOfBytes = buildSPIarray( &opcode[0], 2, dataTx );

    ADS131_CS_LOW();

    // Send command : 6 byte = 3 byte (MSB + LSB + 0x00) + 3 byte (CRC NULL)
    ADS131SendReceiveByte(dataTx, dataRx, numberOfBytes);


//    TRACE_INFO("\n cmdResponse:%x", cmdResponse);

    // [FRAME 2] Send NULL command to retrieve the register data
	registerMap[address][ads_index] = ADS131M02Driver_SendCommand( OPCODE_NULL, ads_index );

//	TRACE_INFO("\n registerMap[%x]:%x", address, registerMap[address][ads_index]);

	ADS131_CS_HIGH();

	return registerMap[address][ads_index];
}


/************************************************************************************//**
 *
 * @brief ADS131M02Driver_WriteSingleRegister()
 *          Write data to a single register at the specified address
 *
 * @param[in]   address     Register address to write
 * @param[in]   data        8-bit data to write
 * @param[in]   readCheck   Perform a read after write to check sucessful write
 * @param[in]   ads_index   Index of the ADS, if multiple ADS share one SPI bus
 *
 * @return      Returns TRUE if successful
 */
Bool ADS131M02Driver_WriteSingleRegister( uint8_t address, uint16_t data, Bool readCheck, uint8_t ads_index )
{
    uint16_t dataRead = 0;
    /* Check that the register address is in range */
//    assert( address < NUM_REGISTERS );

    ADS131_CS_LOW();

    // (OPTIONAL) Enforce certain register field values when
    // writing to the MODE register to fix the operation mode
    if (MODE_ADDRESS == address)
    {
        data = enforce_selected_device_modes(data);
    }

    // Build TX and RX byte array
    uint16_t opcodes[COMMAND_WLENGTH * WLENGTH_BYTES] = { 0 };

    opcodes[0] = OPCODE_WREG | (((uint16_t) address) << 7);
    opcodes[1] = data;
    opcodes[2] = 0x0000;

    uint8_t numberOfBytes = buildSPIarray( &opcodes[0], 3, dataTx );

    // Send command
    ADS131SendReceiveByte(dataTx, dataRx, numberOfBytes);  //9 byte = 3 byte cmd + 3 byte register data + 3 byte CRC(NULL)

    // Update internal array for checking
    registerMap[address][ads_index] = data;

    if ( readCheck ) {
        dataRead = ADS131M02Driver_ReadSingleRegister( address, ads_index );
        if ( data != dataRead ) {
            return FALSE;
        }
    }

    ADS131_CS_HIGH();

    return TRUE;
}



/************************************************************************************//**
 *
 * @brief ADS131M02Driver_RequestData()
 *          Sends the read command and the Callback routine interprets the data
 *          NOTE: Call this function after /DRDY goes low and specify the
 *          the number of bytes to read and the starting position of data
 *
 * @param[in]   *s_ADCdata points to an adc_channel_data type-defined structure
 * @param[in]   ads_index   Index of the ADS, if multiple ADS share one SPI bus
 *
 * @return      Returns TRUE if the CRC-OUT of the data read detects an error.
 */
Bool ADS131M02Driver_RequestData( adc_channel_data *ADCdata, uint8_t ads_index )
{
    // Data Transfer                            and Receive packet
    // dataTx[0] = Command MSB                  dataRx[0] = Response MSB
    // dataTx[1] = Command LSB                  dataRx[1] = Response LSB
    // dataTx[2 + 0 * WLENGTH_BYTES] = CRC In   dataRx[2 + 0 * WLENGTH_BYTES] = Ch 0 data
    // dataTx[2 + 1 * WLENGTH_BYTES] = NULL     dataRx[2 + 1 * WLENGTH_BYTES] = Ch 1 data
    // dataTx[2 + 2 * WLENGTH_BYTES] = NULL     dataRx[2 + 2 * WLENGTH_BYTES] = Ch 2 data
    // dataTx[2 + 3 * WLENGTH_BYTES] = NULL     dataRx[2 + 3 * WLENGTH_BYTES] = Ch 3 data
    // dataTx[2 + 4 * WLENGTH_BYTES] = NULL     dataRx[2 + 4 * WLENGTH_BYTES] = Ch 4 data
    // dataTx[2 + 5 * WLENGTH_BYTES] = NULL     dataRx[2 + 5 * WLENGTH_BYTES] = Ch 5 data
    // dataTx[2 + 6 * WLENGTH_BYTES] = NULL     dataRx[2 + 6 * WLENGTH_BYTES] = Ch 6 data
    // dataTx[2 + 7 * WLENGTH_BYTES] = NULL     dataRx[2 + 7 * WLENGTH_BYTES] = Ch 7 data
    // dataTx[2 + 8 * WLENGTH_BYTES] = NULL     dataRx[2 + 8 * WLENGTH_BYTES] = CRC

	int32_t data0 = 0;
	int32_t data1 = 0;

//	data0 |= ADS131Read24bitData();
//	data1 |= ADS131Read24bitData();
//	data0 |= ADS131M02Driver_SendCommand(OPCODE_NULL, ads_index)<<16;
//	data0 |= ADS131M02Driver_SendCommand(OPCODE_NULL, ads_index) & 0xFFFF;
//
//	data1 |= ADS131M02Driver_SendCommand(OPCODE_NULL, ads_index)<<16;
//	data1 |= ADS131M02Driver_SendCommand(OPCODE_NULL, ads_index) & 0xFFFF;

//	data0 |= (ADS131M02Driver_SendCommand(DummyWord[0], ads_index)) << 16;
//	data0 |= ADS131M02Driver_SendCommand(DummyWord[0], ads_index);
//	data1 |= (ADS131M02Driver_SendCommand(DummyWord[0], ads_index)) << 16;
//	data1 |= ADS131M02Driver_SendCommand(DummyWord[0], ads_index);

//	uint16_t opcodes[COMMAND_WLENGTH * WLENGTH_BYTES] = { 0 };
//	uint8_t numberOfBytes = buildSPIarray( &opcodes[0], 2, dataTx );
//
//	for(uint8_t i = 0;i < 10;i++)
//	{
//		dataTx[i] = 0x00;
//	}
//
//	ADS131SendReceiveByte(dataTx, dataRx, numberOfBytes);

	ADCdata->channel0 = dataRx[0]<<16 | dataRx[1]<<8 | dataRx[2];
	ADCdata->channel1 = dataRx[3]<<16 | dataRx[4]<<8 | dataRx[5];

    return( TRUE );
}



//*****************************************************************************
//
//! Sends the specified SPI command to the ADC (NULL, STANDBY, or WAKEUP).
//!
//! @fn uint16_t ADS131M02Driver_SendCommand(uint16_t opcode)
//!
//! @param opcode SPI command byte.
//! @param ads_index   Index of the ADS, if multiple ADS share one SPI bus,it's single one here
//!
//! NOTE: Other commands have their own dedicated functions to support
//! additional functionality.
//!
//! @return ADC response byte (typically the STATUS byte).
//
//*****************************************************************************
uint16_t ADS131M02Driver_SendCommand(uint16_t opcode, uint8_t ads_index)
{
//	uint8_t lsb = (uint8_t)(opcode & 0XFF);
//	uint8_t msb = (uint8_t)((opcode >> 8) & 0XFF);


    // Build TX and RX byte array
    uint8_t numberOfBytes = buildSPIarray(&opcode, 2, dataTx);

   // Send command
    ADS131SendReceiveByte( dataTx, dataRx, numberOfBytes);

     // Combine response bytes and return as a 16-bit word, ignore the byte[2] ~ byte[5]
    uint16_t adcResponse = combineBytes(dataRx[0], dataRx[1]);

//	adcResponse |= ADS131SendByte(msb);
//	adcResponse |= ADS131SendByte(lsb);

    return adcResponse;
}



//*****************************************************************************
//
//! Resets the device.
//!
//! @fn void ADS131M02Driver_ResetDevice(void)
//!
//! @param ads_index   Index of the ADS, if multiple ADS share one SPI bus
//!
//! NOTE: This function does not capture DOUT data, but it could be modified
//! to do so.
//!
//! @return None.
//
//*****************************************************************************
void ADS131M02Driver_ResetDevice( uint8_t ads_index )
{
    // Build TX and RX byte array
    uint16_t opcode         = OPCODE_RESET;
    uint16_t response = 0;

    // Send command
    response = ADS131M02Driver_SendCommand(opcode, ads_index);

    // NOTE: The ADS131M0x's next response word should be (0xFF20 | CHANCNT),
    // if the response is 0x0011 (acknowledge of RESET command), then the device
    // did not receive a full SPI frame and the reset did not occur!

    // tSRLRST delay of 2048 Tclk
    System_DelayUs( DELAY_2048TCLK );

    if(OPCODE_RESET == response)
    {
    	TRACE_INFO("Reset cmd sent\n");
    }
    else if(OPCODE_RESET_RESPONSE == response)
    {
    	TRACE_INFO("Reset ok\n");
    }
    else
    {
    	TRACE_INFO("Reset ignore\n");
    }

    // Update register setting array to keep software in sync with device
    ADS131M02Driver_RestoreRegisterDefaults();

//    ADS131M02Driver_SendCommand( OPCODE_NULL, ads_index );

    // Write to MODE register to enforce mode settings
//    ADS131M02Driver_WriteSingleRegister( MODE_ADDRESS, MODE_DEFAULT, TRUE, ads_index );
}



//*****************************************************************************
//
//! Sends the LOCK command and verifies that registers are locked.
//!
//! @fn Bool ADS131M02Driver_LockRegisters(void)
//!
//! @param ads_index   Index of the ADS, if multiple ADS share one SPI bus
//!
//! @return Boolean to indicate if an error occurred (0 = no error; 1 = error)
//
//*****************************************************************************
Bool ADS131M02Driver_LockRegisters( uint8_t ads_index )
{
    Bool b_lock_error;

    // Build TX and RX byte array
    uint16_t opcode         = OPCODE_LOCK;

    // Send command
    ADS131M02Driver_SendCommand(opcode, ads_index);

    /* (OPTIONAL) Check for SPI errors by sending the NULL command and checking STATUS */

    /* (OPTIONAL) Read back the STATUS register and check if LOCK bit is set... */
    ADS131M02Driver_ReadSingleRegister( STATUS_ADDRESS, ads_index );
    if((getRegisterValue(STATUS_ADDRESS, ads_index) & STATUS_LOCK_LOCKED))
	{
    	TRACE_INFO("\n Register Locked");
    	b_lock_error = TRUE;
	}


    /* If the STATUS register is NOT read back,
     * then make sure to manually update the global register map variable... */
    //registerMap[STATUS_ADDRESS]  |= STATUS_LOCK_LOCKED;

    /* (OPTIONAL) Error handler */
    if (b_lock_error) {
        // Insert error handler function call here...
    }

    return b_lock_error;
}



//*****************************************************************************
//
//! Sends the UNLOCK command and verifies that registers are unlocked
//!
//! @fn Bool ADS131M02Driver_UnLockRegisters(void)
//!
//! @param ads_index   Index of the ADS, if multiple ADS share one SPI bus
//!
//! @return Boolean to indicate if an error occurred (0 = no error; 1 = error)
//
//*****************************************************************************
Bool ADS131M02Driver_UnLockRegisters( uint8_t ads_index )
{
	Bool b_unlock_error;

    // Build TX and RX byte array
    uint16_t opcode = OPCODE_UNLOCK;

    // Send command
    ADS131M02Driver_SendCommand(opcode, ads_index);

    /* (OPTIONAL) Check for SPI errors by sending the NULL command and checking STATUS */

    /* (OPTIONAL) Read the STATUS register and check if LOCK bit is cleared... */
    ADS131M02Driver_ReadSingleRegister( STATUS_ADDRESS, ads_index );
    if(!(getRegisterValue(STATUS_ADDRESS, ads_index) & STATUS_LOCK_UNLOCKED))
	{
		TRACE_INFO("\n Register Unlocked");
		b_unlock_error = TRUE;
	}

    /* If the STATUS register is NOT read back,
     * then make sure to manually update the global register map variable... */
    //registerMap[STATUS_ADDRESS]  &= !STATUS_LOCK_LOCKED;

    /* (OPTIONAL) Error handler */
    if (b_unlock_error) {
        // Insert error handler function call here...
    }

    return b_unlock_error;
}



//*****************************************************************************
//
//! Calculates the 16-bit CRC for the selected CRC polynomial.
//!
//! @fn uint16_t calculateCRC(const uint8_t dataBytes[], uint8_t numberBytes, uint16_t initialValue)
//!
//! @param dataBytes[] pointer to first element in the data byte array
//! @param numberBytes number of bytes to be used in CRC calculation
//! @param initialValue the seed value (or partial crc calculation), use 0xFFFF when beginning a new CRC computation
//!
//! NOTE: This calculation is shown as an example and is not optimized for speed.
//!
//! @return 16-bit calculated CRC word
//
//*****************************************************************************
uint16_t calculateCRC(const uint8_t dataBytes[], uint8_t numberBytes, uint16_t initialValue)
{
	/* Check that "dataBytes" is not a null pointer */
//	assert(dataBytes != 0x00);

	int         bitIndex, byteIndex;
	Bool        dataMSb;						/* Most significant bit of data byte */
	Bool        crcMSb;						    /* Most significant bit of crc byte  */

	/*
     * Initial value of crc register
     * NOTE: The ADS131M0x defaults to 0xFFFF,
     * but can be set at function call to continue an on-going calculation
     */
    uint16_t crc = initialValue;

    #ifdef CRC_ANSI
    /* ANSI CRC polynomial = x^16 + x^12 + x^5 + 1 */
    const uint16_t poly = 0x8005;
    #endif

    #ifdef CRC_CCITT
    /* CCITT CRC polynomial = x^16 + x^15 + x^2 + 1 */
    const uint16_t poly = 0x1021;
    #endif

    //
    // CRC algorithm
    //

    // Loop through all bytes in the dataBytes[] array
	for (byteIndex = 0; byteIndex < numberBytes; byteIndex++)
	{
	    // Point to MSb in byte
	    bitIndex = 0x80u;

	    // Loop through all bits in the current byte
	    while (bitIndex > 0)
	    {
	        // Check MSB's of data and crc
	        dataMSb = (Bool) (dataBytes[byteIndex] & bitIndex);
	        crcMSb  = (Bool) (crc & 0x8000u);

	        crc <<= 1;              /* Left shift CRC register */

	        // Check if XOR operation of MSBs results in additional XOR operations
	        if (dataMSb ^ crcMSb)
	        {
	            crc ^= poly;        /* XOR crc with polynomial */
	        }

	        /* Shift MSb pointer to the next data bit */
	        bitIndex >>= 1;
	    }
	}

	return crc;
}



//*****************************************************************************
//
//! Updates the registerMap[] array to its default values.
//!
//! @fn void ADS131M02Driver_RestoreRegisterDefaults( void )
//!
//! NOTES:
//! - If the MCU keeps a copy of the ADS131M0x register settings in memory,
//! then it is important to ensure that these values remain in sync with the
//! actual hardware settings. In order to help facilitate this, this function
//! should be called after powering up or resetting the device (either by
//! hardware pin control or SPI software command).
//!
//! - Reading back all of the registers after resetting the device can
//! accomplish the same result; however, this might be problematic if the
//! device was previously in CRC mode or the WLENGTH was modified, since
//! resetting the device exits these modes. If the MCU is not aware of this
//! mode change, then read register commands will return invalid data due to
//! the expectation of data appearing in a different byte position.
//!
//! @return None.
//
//*****************************************************************************
void ADS131M02Driver_RestoreRegisterDefaults( void )
{
    int ads_index;

    for ( ads_index = 0; ads_index < NUM_ADS131M0X; ads_index++ ) {
        registerMap[ID_ADDRESS][ads_index]             =   0x00;               /* NOTE: This a read-only register */
        registerMap[STATUS_ADDRESS][ads_index]         =   STATUS_DEFAULT;
        registerMap[MODE_ADDRESS][ads_index]           =   MODE_DEFAULT;
        registerMap[CLOCK_ADDRESS][ads_index]          =   CLOCK_DEFAULT;
        registerMap[GAIN1_ADDRESS][ads_index]          =   GAIN1_DEFAULT;
//        registerMap[GAIN2_ADDRESS][ads_index]          =   GAIN2_DEFAULT;
        registerMap[CFG_ADDRESS][ads_index]            =   CFG_DEFAULT;
        registerMap[THRSHLD_MSB_ADDRESS][ads_index]    =   THRSHLD_MSB_DEFAULT;
        registerMap[THRSHLD_LSB_ADDRESS][ads_index]    =   THRSHLD_LSB_DEFAULT;
        registerMap[CH0_CFG_ADDRESS][ads_index]        =   CH0_CFG_DEFAULT;
        registerMap[CH0_OCAL_MSB_ADDRESS][ads_index]   =   CH0_OCAL_MSB_DEFAULT;
        registerMap[CH0_OCAL_LSB_ADDRESS][ads_index]   =   CH0_OCAL_LSB_DEFAULT;
        registerMap[CH0_GCAL_MSB_ADDRESS][ads_index]   =   CH0_GCAL_MSB_DEFAULT;
        registerMap[CH0_GCAL_LSB_ADDRESS][ads_index]   =   CH0_GCAL_LSB_DEFAULT;
    #if (CHANNEL_COUNT > 1)
        registerMap[CH1_CFG_ADDRESS][ads_index]        =   CH1_CFG_DEFAULT;
        registerMap[CH1_OCAL_MSB_ADDRESS][ads_index]   =   CH1_OCAL_MSB_DEFAULT;
        registerMap[CH1_OCAL_LSB_ADDRESS][ads_index]   =   CH1_OCAL_LSB_DEFAULT;
        registerMap[CH1_GCAL_MSB_ADDRESS][ads_index]   =   CH1_GCAL_MSB_DEFAULT;
        registerMap[CH1_GCAL_LSB_ADDRESS][ads_index]   =   CH1_GCAL_LSB_DEFAULT;
    #endif
    #if (CHANNEL_COUNT > 2)
        registerMap[CH2_CFG_ADDRESS][ads_index]        =   CH2_CFG_DEFAULT;
        registerMap[CH2_OCAL_MSB_ADDRESS][ads_index]   =   CH2_OCAL_MSB_DEFAULT;
        registerMap[CH2_OCAL_LSB_ADDRESS][ads_index]   =   CH2_OCAL_LSB_DEFAULT;
        registerMap[CH2_GCAL_MSB_ADDRESS][ads_index]   =   CH2_GCAL_MSB_DEFAULT;
        registerMap[CH2_GCAL_LSB_ADDRESS][ads_index]   =   CH2_GCAL_LSB_DEFAULT;
    #endif
    #if (CHANNEL_COUNT > 3)
        registerMap[CH3_CFG_ADDRESS][ads_index]        =   CH3_CFG_DEFAULT;
        registerMap[CH3_OCAL_MSB_ADDRESS][ads_index]   =   CH3_OCAL_MSB_DEFAULT;
        registerMap[CH3_OCAL_LSB_ADDRESS][ads_index]   =   CH3_OCAL_LSB_DEFAULT;
        registerMap[CH3_GCAL_MSB_ADDRESS][ads_index]   =   CH3_GCAL_MSB_DEFAULT;
        registerMap[CH3_GCAL_LSB_ADDRESS][ads_index]   =   CH3_GCAL_LSB_DEFAULT;
    #endif
    #if (CHANNEL_COUNT > 4)
        registerMap[CH4_CFG_ADDRESS][ads_index]        =   CH4_CFG_DEFAULT;
        registerMap[CH4_OCAL_MSB_ADDRESS][ads_index]   =   CH4_OCAL_MSB_DEFAULT;
        registerMap[CH4_OCAL_LSB_ADDRESS][ads_index]   =   CH4_OCAL_LSB_DEFAULT;
        registerMap[CH4_GCAL_MSB_ADDRESS][ads_index]   =   CH4_GCAL_MSB_DEFAULT;
        registerMap[CH4_GCAL_LSB_ADDRESS][ads_index]   =   CH4_GCAL_LSB_DEFAULT;
    #endif
    #if (CHANNEL_COUNT > 5)
        registerMap[CH5_CFG_ADDRESS][ads_index]        =   CH5_CFG_DEFAULT;
        registerMap[CH5_OCAL_MSB_ADDRESS][ads_index]   =   CH5_OCAL_MSB_DEFAULT;
        registerMap[CH5_OCAL_LSB_ADDRESS][ads_index]   =   CH5_OCAL_LSB_DEFAULT;
        registerMap[CH5_GCAL_MSB_ADDRESS][ads_index]   =   CH5_GCAL_MSB_DEFAULT;
        registerMap[CH5_GCAL_LSB_ADDRESS][ads_index]   =   CH5_GCAL_LSB_DEFAULT;
    #endif
    #if (CHANNEL_COUNT > 6)
        registerMap[CH6_CFG_ADDRESS][ads_index]        =   CH6_CFG_DEFAULT;
        registerMap[CH6_OCAL_MSB_ADDRESS][ads_index]   =   CH6_OCAL_MSB_DEFAULT;
        registerMap[CH6_OCAL_LSB_ADDRESS][ads_index]   =   CH6_OCAL_LSB_DEFAULT;
        registerMap[CH6_GCAL_MSB_ADDRESS][ads_index]   =   CH6_GCAL_MSB_DEFAULT;
        registerMap[CH6_GCAL_LSB_ADDRESS][ads_index]   =   CH6_GCAL_LSB_DEFAULT;
    #endif
    #if (CHANNEL_COUNT > 7)
        registerMap[CH7_CFG_ADDRESS][ads_index]        =   CH7_CFG_DEFAULT;
        registerMap[CH7_OCAL_MSB_ADDRESS][ads_index]   =   CH7_OCAL_MSB_DEFAULT;
        registerMap[CH7_OCAL_LSB_ADDRESS][ads_index]   =   CH7_OCAL_LSB_DEFAULT;
        registerMap[CH7_GCAL_MSB_ADDRESS][ads_index]   =   CH7_GCAL_MSB_DEFAULT;
        registerMap[CH7_GCAL_LSB_ADDRESS][ads_index]   =   CH7_GCAL_LSB_DEFAULT;
    #endif
        registerMap[REGMAP_CRC_ADDRESS][ads_index]     =   REGMAP_CRC_DEFAULT;
    }
}



//****************************************************************************
//
// Helper functions
//
//****************************************************************************


//*****************************************************************************
//
//! Takes a 16-bit word and returns the most-significant byte.
//!
//! @fn uint8_t upperByte(uint16_t uint16_Word)
//!
//! @param uint16_Word is the original 16-bit word.
//!
//! @return 8-bit most-significant byte.
//
//*****************************************************************************
uint8_t upperByte(uint16_t uint16_Word)
{
    uint8_t msByte;
    msByte = (uint8_t) ((uint16_Word >> 8) & 0x00FF);

    return msByte;
}



//*****************************************************************************
//
//! Takes a 16-bit word and returns the least-significant byte.
//!
//! @fn uint8_t lowerByte(uint16_t uint16_Word)
//!
//! @param uint16_Word is the original 16-bit word.
//!
//! @return 8-bit least-significant byte.
//
//*****************************************************************************
uint8_t lowerByte(uint16_t uint16_Word)
{
    uint8_t lsByte;
    lsByte = (uint8_t) (uint16_Word & 0x00FF);

    return lsByte;
}



//*****************************************************************************
//
//! Takes two 8-bit words and returns a concatenated 16-bit word.
//!
//! @fn uint16_t combineBytes(uint8_t upperByte, uint8_t lowerByte)
//!
//! @param upperByte is the 8-bit value that will become the MSB of the 16-bit word.
//! @param lowerByte is the 8-bit value that will become the LSB of the 16-bit word.
//!
//! @return concatenated 16-bit word.
//
//*****************************************************************************
uint16_t combineBytes(uint8_t upperByte, uint8_t lowerByte)
{
    uint16_t combinedValue;
    combinedValue = ((uint16_t) upperByte << 8) | ((uint16_t) lowerByte);

    return combinedValue;
}



//*****************************************************************************
//
//! Combines ADC data bytes into a single signed 32-bit word.
//!
//! @fn int32_t signExtend(const uint8_t dataBytes[])
//!
//! @param dataBytes is a pointer to uint8_t[] where the first element is the MSB.
//!
//! @return Returns the signed-extend 32-bit result.
//
//*****************************************************************************
inline int32_t signExtend( uint8_t dataBytes[] )
{
#ifdef WORD_LENGTH_24BIT

    return (((int32_t) ((dataBytes[0] << 24) | (dataBytes[1] << 16) | (dataBytes[2] << 8))) >> 8);     // Right-shift of signed data maintains signed bit

#elif defined WORD_LENGTH_32BIT_SIGN_EXTEND

    return ((dataBytes[0] << 24) | (dataBytes[1] << 16) | (dataBytes[2] << 8) | dataBytes[3]);

#elif defined WORD_LENGTH_32BIT_ZERO_PADDED

    return (((int32_t) ((dataBytes[0] << 24) | (dataBytes[1] << 16) | (dataBytes[2] << 8))) >> 8);     // Right-shift of signed data maintains signed bit

#elif defined WORD_LENGTH_16BIT_TRUNCATED

    return (((int32_t) ((dataBytes[0] << 24) | (dataBytes[1] << 16))) >> 16);                 // Right-shift of signed data maintains signed bit

#endif
}



//****************************************************************************
//
// Internal functions
//
//****************************************************************************


//*****************************************************************************
//
//! Builds SPI TX data arrays according to number of opcodes provided and
//! currently programmed device word length.
//!
//! @fn uint8_t buildSPIarray(const uint16_t opcodeArray[], uint8_t numberOpcodes, uint8_t byteArray[])
//!
//! @param opcodeArray[] pointer to an array of 16-bit opcodes to use in the SPI command.
//! @param numberOpcodes the number of opcodes provided in opcodeArray[].
//! @param byteArray[] pointer to an array of 8-bit SPI bytes to send to the device.
//!
//! NOTE: The calling function must ensure it reserves sufficient memory for byteArray[]!
//!
//! @return number of bytes added to byteArray[].
//
//*****************************************************************************
uint8_t buildSPIarray(const uint16_t opcodeArray[], uint8_t numberOpcodes, uint8_t byteArray[])
{
    /*
     * Frame size = opcode word(s) + optional CRC word
     * Number of bytes per word = 2, 3, or 4
     * Total bytes = bytes per word * number of words
     */
    uint8_t numberWords     = numberOpcodes;
    uint8_t bytesPerWord    = WLENGTH_BYTES;
    uint8_t numberOfBytes   = numberWords * bytesPerWord;

    int i;
    for (i = 0; i < numberOpcodes; i++)
    {
        // NOTE: Be careful not to accidentally overflow the array here.
        // The array and opcodes are defined in the calling function, so
        // we are trusting that no mistakes were made in the calling function!
        byteArray[(i*bytesPerWord) + 0] = upperByte(opcodeArray[i]);
        byteArray[(i*bytesPerWord) + 1] = lowerByte(opcodeArray[i]);
        byteArray[(i*bytesPerWord) + 2] = 0x00;
    }

//    byteArray[0] = upperByte(opcodeArray[0]);
//	byteArray[1] = lowerByte(opcodeArray[0]);//CMD
//	if(1 < numberOpcodes)
//	{
//		byteArray[4] = 0x00;
//		byteArray[5] = 0x00;//NULL CRC
//	}
//	else
//	{
//		byteArray[2] = 0x00;
//		byteArray[3] = 0x00;// 24bit LSB = 0
//		byteArray[4] = 0x00;
//		byteArray[5] = 0x00;//NULL CRC
//	}


    return( numberOfBytes );
}



//*****************************************************************************
//
//! Modifies MODE register data to maintain device operation according to
//! preselected mode(s) (RX_CRC_EN, WLENGTH, etc.).
//!
//! @fn uint16_t enforce_selected_device_modes(uint16_t data)
//!
//! @param data uint16_t register data.
//!
//! @return uint16_t modified register data.
//
//*****************************************************************************
uint16_t enforce_selected_device_modes(uint16_t data)
{


    ///////////////////////////////////////////////////////////////////////////
    // Enforce RX_CRC_EN setting

#ifdef ENABLE_CRC_IN
    // When writing to the MODE register, ensure RX_CRC_EN bit is ALWAYS set
    data |= MODE_RX_CRC_EN_ENABLED;
#else
    // When writing to the MODE register, ensure RX_CRC_EN bit is NEVER set
    data &= ~MODE_RX_CRC_EN_ENABLED;
#endif // ENABLE_CRC_IN


    ///////////////////////////////////////////////////////////////////////////
    // Enforce WLENGH setting

#ifdef WORD_LENGTH_24BIT
    // When writing to the MODE register, ensure WLENGTH bits are ALWAYS set to 01b
    data = (data & ~MODE_WLENGTH_MASK) | MODE_WLENGTH_24BIT;
#elif defined WORD_LENGTH_32BIT_SIGN_EXTEND
    // When writing to the MODE register, ensure WLENGH bits are ALWAYS set to 11b
    data = (data & ~MODE_WLENGTH_MASK) | MODE_WLENGTH_32BIT_MSB_SIGN_EXT;
#elif defined WORD_LENGTH_32BIT_ZERO_PADDED
    // When writing to the MODE register, ensure WLENGH bits are ALWAYS set to 10b
    data = (data & ~MODE_WLENGTH_MASK) | MODE_WLENGTH_32BIT_LSB_ZEROES;
#elif defined WORD_LENGTH_16BIT_TRUNCATED
    // When writing to the MODE register, ensure WLENGH bits are ALWAYS set to 00b
    data = (data & ~MODE_WLENGTH_MASK) | MODE_WLENGTH_16BIT;
#endif


    ///////////////////////////////////////////////////////////////////////////
    // Enforce DRDY_FMT setting

#ifdef DRDY_FMT_PULSE
    // When writing to the MODE register, ensure DRDY_FMT bit is ALWAYS set
    data = (data & ~MODE_DRDY_FMT_MASK) | MODE_DRDY_FMT_NEG_PULSE_FIXED_WIDTH;
#else
    // When writing to the MODE register, ensure DRDY_FMT bit is NEVER set
    data = (data & ~MODE_DRDY_FMT_MASK) | MODE_DRDY_FMT_LOGIC_LOW;
#endif


    ///////////////////////////////////////////////////////////////////////////
    // Enforce CRC_TYPE setting

#ifdef CRC_CCITT
    // When writing to the MODE register, ensure CRC_TYPE bit is NEVER set
    data = (data & ~STATUS_CRC_TYPE_MASK) | STATUS_CRC_TYPE_16BIT_CCITT;
#elif defined CRC_ANSI
    // When writing to the MODE register, ensure CRC_TYPE bit is ALWAYS set
    data = (data & ~STATUS_CRC_TYPE_MASK) | STATUS_CRC_TYPE_16BIT_ANSI;
#endif

    // Return modified register data
    return data;
}



void DRDYInterrupt_Init(void)
{
	GPIO_InitTypeDef	GPIO_InitStructure;
	NVIC_InitTypeDef	NVIC_InitStructure;
	EXTI_InitTypeDef	EXTI_InitStructure;

	RCC_AHB1PeriphClockCmd(ADS131_DRDY_RCC, ENABLE);
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_SYSCFG, ENABLE);//使能SYSCFG时钟

	GPIO_InitStructure.GPIO_Pin = ADS131_DRDY_PIN;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN;
	GPIO_InitStructure.GPIO_Speed = GPIO_Fast_Speed;
	GPIO_InitStructure.GPIO_PuPd  = GPIO_PuPd_NOPULL;
	GPIO_Init(ADS131_DRDY_GPIO, &GPIO_InitStructure);

	SYSCFG_EXTILineConfig(ADS131_DRDY_PortSource, ADS131_DRDY_PinSource);

	EXTI_InitStructure.EXTI_Line 	= ADS131_DRDY_Line;
	EXTI_InitStructure.EXTI_Mode 	= EXTI_Mode_Interrupt;
	EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Falling;  //下降沿采集
	EXTI_InitStructure.EXTI_LineCmd	= ENABLE;
	EXTI_Init(&EXTI_InitStructure);

	NVIC_InitStructure.NVIC_IRQChannel 					= ADS131_DRDY_IRQ;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority= ADS131_IRQ_PRIORITY;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority 		= 0x02;
	NVIC_InitStructure.NVIC_IRQChannelCmd				= ENABLE;
	NVIC_Init(&NVIC_InitStructure);
}

void DRDYInterrupt_Status(Bool ret)
{
	NVIC_InitTypeDef	NVIC_InitStructure;
	NVIC_InitStructure.NVIC_IRQChannel 					= ADS131_DRDY_IRQ;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority= ADS131_IRQ_PRIORITY;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority 		= 0x02;
	if(TRUE == ret)
	{
		NVIC_InitStructure.NVIC_IRQChannelCmd				= DISABLE;
	}
	else
	{
		NVIC_InitStructure.NVIC_IRQChannelCmd				= ENABLE;
	}

	NVIC_Init(&NVIC_InitStructure);
}

Bool ADS131M02Driver_GetAD(OpticalSignalAD* data)
{
	Bool ret = TRUE;

	data->measure = s_ADCdata->channel0;
	data->reference = s_ADCdata->channel1;

	return ret;
}

float ADS131M02Driver_GetData(uint8_t index)
{
	float data = 0;
	Uint32 tData[2] = {0};
	float temp[2] = {0};

	tData[0] = tempData[index];
	if(tData[0] >= 0x800000)
	{
		tData[0] -= 0x800000;
		temp[0] = (float)tData[0]* 1.2 / 0x7FFFFF - 1.2;
	}
	else
	{
		temp[0] = (float)tData[0]* 1.2 / 0x7FFFFF;
	}

	tData[1] = tempData[index + 2];
	if(tData[1] >= 0x800000)
	{
		tData[1] -= 0x800000;
		temp[1] = (float)tData[1]* 1.2 / 0x7FFFFF - 1.2;
	}
	else
	{
		temp[1] = (float)tData[1]* 1.2 / 0x7FFFFF;
	}

//	data = channelData[index];
	data = (temp[0] + temp[1])/2;

	return data;
}

void ADS131_DRDY_IRQHandle(void)
{
	static int cnt = 0;
	uint32_t pData[2] ={ 0 };
	uint8_t buf[12] = { 0 };
	float temp[2] = {0};
	static uint8_t dataCnt = 0;
	EXTI_ClearITPendingBit(ADS131_DRDY_Line);

	if(1)
	{
		ADS131_CS_LOW();

		for(uint8_t i = 0; i < 12;i++)  //12 = 3 byte(STATUS Register Status ,LSB = 0)  + 3 byte(Channel0 DATA) + 3 byte(Channel1 DATA) + 3 byte(CRC)
		{
			buf[i] = ADS131SendByte(0x00);
		}
		pData[0] = buf[3] << 16 | buf[4] << 8 | buf[5];
		pData[1] = buf[6] << 16 | buf[7] << 8 | buf[8];
		s_ADCdata->channel0 = pData[0];
	    s_ADCdata->channel1 = pData[1];
//		TRACE_INFO("\n chl0AD:%d, chl1AD:%d \t",pData[0], pData[1]);

		ADS131_CS_HIGH();

		dataCnt++;
		if(dataCnt == 1)
		{
			tempData[0] = pData[0];
			tempData[1] = pData[1];
			tempData[2] = pData[0];
			tempData[3] = pData[1];
		}
		if(dataCnt == 2)
		{
			tempData[2] = pData[0];
			tempData[3] = pData[1];
			dataCnt = 0;
		}

//		if(cnt % 1000 == 0)
//		{
//			TRACE_INFO("\n 0:%f, 2:%f", channelData[0], channelData[2]);
//		}


		cnt = 0;
	}
	cnt++;
}

