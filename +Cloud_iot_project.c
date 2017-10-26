


/***************************************************************************//**
* @file sleep.c
*******************************************************************************
* @section License
* <b>(C) Copyright 2015 Silicon Labs, http://www.silabs.com</b>
*******************************************************************************
*
* Permission is granted to anyone to use this software for any purpose,
* including commercial applications, and to alter it and redistribute it
* freely, subject to the following restrictions:
*
* 1. The origin of this software must not be misrepresented; you must not
*    claim that you wrote the original software.
* 2. Altered source versions must be plainly marked as such, and must not be
*    misrepresented as being the original software.
* 3. This notice may not be removed or altered from any source distribution.
*
* DISCLAIMER OF WARRANTY/LIMITATION OF REMEDIES: Silicon Labs has no
* obligation to support this Software. Silicon Labs is providing the
* Software "AS IS", with no express or implied warranties of any kind,
* including, but not limited to, any implied warranties of merchantability
* or fitness for any particular purpose or warranties against infringement
* of any proprietary rights of a third party.
*
* Silicon Labs will not be liable for any consequential, incidental, or
* special damages, or any other relief, or for any claim by any third party,
* arising from your use of this Software.
*
*****************************************************************************/

#include "em_chip.h"
#include "em_cmu.h"
#include "em_emu.h"
#include "em_gpio.h"
#include "em_letimer.h"
#include "em_system.h"
#include "em_device.h"
#include "em_rtc.h"
#include "em_int.h"
#include "em_timer.h"
#include "em_acmp.h"
#include "em_adc.h"
#include "em_dma.h"
#include "dmactrl.h"
#include "TSL2561.h"
#include "em_i2c.h"
#include <stdbool.h>
#include "dmactrl.c"
#include <stdint.h>


#include "em_chip.h"
#include "em_cmu.h"
#include "em_emu.h"
#include "em_gpio.h"
#include "em_letimer.h"
#include "em_system.h"
#include "em_device.h"
#include "em_rtc.h"
#include "em_int.h"
#include "em_timer.h"
#include "em_acmp.h"
#include "em_adc.h"
#include "em_dma.h"
#include "dmactrl.h"
#include "TSL2561.h"
#include "em_i2c.h"
#include <stdbool.h>
#include "dmactrl.c"
#include <stdint.h>
#include "em_leuart.h"
#include "em_lesense.h"
#include "em_int.h"
#include "lesense_letouch.h"
#include "lesense_letouch_config.h"
#include "main.h"
#include "TSL25611.h"
#include "em_leuart.h"
#include "circular_Buffer.h"
#include "em_prs.h"
#include "em_pcnt.h"








/**************************************************************************//**
 *
 * @author Sanjana V K
 * @version 1.09
 ******************************************************************************
 * @section License
 * <b>(C) Copyright 2014 Silicon Labs, http://www.silabs.com</b>
 *******************************************************************************
 *
 * Permission is granted to anyone to use this software for any purpose,
 * including commercial applications, and to alter it and redistribute it
 * freely, subject to the following restrictions:
 *
 * 1. The origin of this software must not be misrepresented; you must not
 *    claim that you wrote the original software.
 * 2. Altered source versions must be plainly marked as such, and must not be
 *    misrepresented as being the original software.
 * 3. This notice may not be removed or altered from any source distribution.
 *
 * DISCLAIMER OF WARRANTY/LIMITATION OF REMEDIES: Silicon Labs has no
 * obligation to support this Software. Silicon Labs is providing the
 * Software "AS IS", with no express or implied warranties of any kind,
 * including, but not limited to, any implied warranties of merchantability
 * or fitness for any particular purpose or warranties against infringement
 * of any proprietary rights of a third party.
 *
 * Silicon Labs will not be liable for any consequential, incidental, or
 * special damages, or any other relief, or for any claim by any third party,
 * arising from your use of this Software.
 *
 ******************************************************************************/



uint8_t light_status;
uint8_t status;

#define EM0 0
#define EM1 1
#define EM2 2
#define EM3 3
/*Setting  the Period and the Excite Time*/
#define period 15
#define Excite 0.004
#define LFXOClock 32768
#define ULFRCOClock 1000
#define DMA_CHANNEL_ADC       0
#define Lowest_energy EM2

#define DMA_Switch 1
#define LowerLimit_Temp 15
#define UpperLimit_Temp 35
/* GPio*/
#define LED_GPIO_PORT gpioPortE
#define LED_GPIO_PIN  2
#define LED_1_GPIO_PORT gpioPortE
#define LED_1_GPIO_PIN  3
/* LESENSE */
#define LIGHTSENSE_CH             6
#define LIGHTSENSE_EXCITE_PORT    gpioPortD
#define LIGHTSENSE_EXCITE_PIN     6
#define LIGHTSENSE_SENSOR_PORT    gpioPortC
#define LIGHTSENSE_SENSOR_PIN     6
#define LEUART0_TX_PORT gpioPortD
#define LEUART0_TX_PIN  4
#define Light_Sensor_ACMP_Channel acmpChannel6
#define Light_Sensor_ACMP_Ref acmpChannelVDD
/*Setting VDD, Upper threshold and Lower Threshold*/
#define VDD 3.3
#define Voltage2 0x0200
#define Voltage61 0x3D00
/*Setting the Clock frequencies*/
#define LFXOClock 32768
#define ULFRCOClock 1000
/*Setting the Calibration mode*/
#define Calibration 0
/* ADC Transfer Data */
#define ADCSAMPLES                        500
#define ADC_SINGLECTRL_INPUTSEL_TEMP  adcSingleInpTemp
#define CALIBRATION          DEVINFO->CAL
#define CALIBRATION_MASK    _DEVINFO_CAL_TEMP_MASK
#define CALIBRATION_SHIFT   _DEVINFO_CAL_TEMP_SHIFT
#define CALIBRATION_ADC      DEVINFO->ADC0CAL2
#define CALIBRATION_ADC_MASK  _DEVINFO_ADC0CAL2_TEMP1V25_MASK
#define CALIBRATION_ADC_SHIFT _DEVINFO_ADC0CAL2_TEMP1V25_SHIFT
#define ADC_SINGLECTRL_REP 0x0001
#define ADC_SINGLESTART 0X0001
#define Passive_LightSense_Switch 0
#define ADC0_DMA_Arbitration dmaArbitrate1
#define Temp_Sense_Rate 10000
#define ADC0_resolution adcRes12Bit
#define ADC0_reference adcRef1V25
#define ADC0_acq_time adcAcqTime1
#define ADC0_rep true
#define ADC0_warmup adcWarmupNormal
#define Disable_Leuart leuartDisable
#define Leuart_reffreq 0
#define Leuart_baudrate 9600
#define Leuart_databits leuartDatabits8
#define Leuart_parity leuartNoParity
#define Leuart_stopbits leuartStopbits2
#define CapSense 1
#define LED_PORT gpioPortE
#define LED_PIN  2

/* RTC nominal frequency */
#define RTC_FREQ               32768

/* LESENSE number of channels possible to use, should be 16 */
#define NUM_LESENSE_CHANNELS    16

/* GPIO Port for analog comparators */
#define LESENSE_CH_PORT         gpioPortC




//uint16_t count=0;
//unsigned int sleep_block_counter[EM3+1];
unsigned int LEtimerFreq;
unsigned int LEtimerFreq1;
//uint32_t AComp0     = 0;
unsigned int LETIMER0_prescaler;
unsigned int Count;
unsigned int Count1;
//unsigned int period_stage=1;
float Osc_ratio;
float Final_temp;
/* Transfer Flag */
volatile bool transferActive;
/*Buffer to store data in the memory*/
//volatile uint16_t ramBufferAdcData[ADCSAMPLES];
float Final_temp1;
float Final_temp2;

uint32_t counter=0;
uint32_t button_counter=0;
static volatile uint16_t calibration_value[NUM_LESENSE_CHANNELS][NUMBER_OF_CALIBRATION_VALUES];
static volatile uint16_t buttons_pressed;
static volatile uint16_t channel_max_value[NUM_LESENSE_CHANNELS];
static volatile uint16_t channel_min_value[NUM_LESENSE_CHANNELS];

static uint16_t channels_used_mask;
static uint8_t num_channels_used;
static float channel_threshold_percent[NUM_LESENSE_CHANNELS];


/* DMA callback structure */
DMA_CB_TypeDef cb;

/*Declaring the functions used*/
void GPIO_setup(void);
void LETIMER_setup(void);
void LETIMER_Calibration_setup(void);
void LETIMER0_IRQHandler(void);
void blockSleepMode(int);
void unblocksleepmode(int);
//void unblocksleepmode(int);
//void sleep();
void Timer1_setup();
float convertToCelsius(int32_t adcsamples);
int16_t summation(volatile uint16_t Bufferdatacalc[ADCSAMPLES]);
void write(uint8_t Reg_address, uint8_t Reg_Value, uint8_t Read_Write);

/* Function prototypes */
static void LETOUCH_setupACMP(void);
static void LETOUCH_setupLESENSE(void);
static void LETOUCH_setupGPIO(void);
static void LETOUCH_setupRTC(void);
static void LETOUCH_setupCMU(void);

static uint16_t GetMaxValue(volatile uint16_t* A, uint16_t N);
static uint16_t GetMinValue(volatile uint16_t* A, uint16_t N);



/* DEFINES */

#define DMA_CHANNEL           0

#define BUF_MAX               7

#define WAKEUP_INTERVAL_MS    2000



/* GLOBAL VARIABLES */

char rxbuf[BUF_MAX];


/* Switch to allow alternation between the two strings */
bool     alternateSwitch = false;

//new
#define Heartrate_ACMP_Channel acmpChannel0
#define Heartrate_ACMP_Ref acmpChannelVDD
#define ACMP0_CHANNEL0_PORT gpioPortC
#define ACMP0_CHANNEL0_PIN 0




uint16_t letimer_acmp_count=0;
//uint16_t count=0;
uint16_t ACMP_count=0;
uint16_t Heart_rate=0;
//unsigned int sleep_block_counter[EM3+1];
unsigned int LEtimerFreq;
unsigned int LEtimerFreq1;
uint32_t AComp0     = 0;
unsigned int LETIMER0_prescaler;
unsigned int Count;
unsigned int Count1;
unsigned int period_stage=1;
float Osc_ratio;
float Final_temp;
/* Transfer Flag */
volatile bool transferActive;
/*Buffer to store data in the memory*/
volatile uint16_t ramBufferAdcData[ADCSAMPLES];


volatile uint16_t Bufferdatacalc[ADCSAMPLES];
void write(uint8_t Reg_address, uint8_t Reg_Value, uint8_t Read_Write);

int8_t irq_count = 0;
int8_t counter_tempirq = 0 ;
int8_t counter_lightirq = 0;
float convertToCelsius(int32_t adcSample);
void setupI2C(void);
//
void init_sensor(uint8_t Command_Reg1, uint8_t Reg_Value, uint8_t Read_Write);

void circ_buff(void)
{
_transmit_buffer.buff = allocate_memory(&_transmit_buffer);
			_transmit_buffer.head=_transmit_buffer.buff;
					 	_transmit_buffer.tail=_transmit_buffer.buff;
					 	_transmit_buffer.buff=_transmit_buffer.buff;
					 	_transmit_buffer.num_items= 0;
					 	_transmit_buffer.length = MAX_LEN;
}

/**************************************************************************//**
 * @brief  Defining the LEUART1 initialization data
 ******************************************************************************/

LEUART_Init_TypeDef leuart1Init =
{
  .enable   = LEUART_CMD_TXEN,       /* Activate data reception on LEUn_TX pin. */
  .refFreq  = off,                    /* Inherit the clock frequenzy from the LEUART clock source */
  .baudrate = Baud_Rate,                 /* Baudrate = 9600 bps */
  .databits = LEUART_CTRL_DATABITS_EIGHT,      /* Each LEUART frame containes 8 databits */
  .parity   = LEUART_CTRL_PARITY_NONE,       /* No parity bits in use */
  .stopbits = LEUART_CTRL_STOPBITS_ONE,      /* Setting the number of stop bits in a frame to 2 bitperiods */
};

/**************************************************************************//**
 * @brief  Initialization of LEUART0
 ******************************************************************************/

void initLeuart(void)
{
	if (Enery_Mode == EM3)
	{
	CMU_OscillatorEnable(cmuSelect_LFRCO,true,true);
	CMU_ClockSelectSet(cmuClock_LFB, cmuSelect_LFRCO);
	}
	else
	{
	CMU_ClockSelectSet(cmuClock_LFB, cmuSelect_LFXO);
	}
  LEUART_Init(LEUART0, &leuart1Init);

  /* Route LEUART1 TX pin to DMA location 0 */
  LEUART0->ROUTE = LEUART_ROUTE_TXPEN |
                   LEUART_ROUTE_LOCATION_LOC0;

  /* Enable GPIO for LEUART1. TX is on D4 */
  GPIO_PinModeSet(EXT_PORT_D,                /* GPIO port */
		  	  	  EXT_PIN_4,                        /* GPIO port number */
                  gpioModePushPull,         /* Pin mode is set to push pull */
                  1);                       /* High idle state */
  LEUART0->CTRL |= LEUART_CTRL_LOOPBK;
  LEUART0->CTRL |= LEUART_CTRL_AUTOTRI;
  NVIC_EnableIRQ(LEUART0_IRQn);
}

/**************************************************************************//**
 * @brief  LEUART0_IRQHandler LEUART0 Interrupt Handler
 ******************************************************************************/

void LEUART0_IRQHandler(void)
{
	INT_Disable();
uint32_t Flags = LEUART0->IF;
LEUART0->IFC = Flags;
LEUART0->IEN &= ~LEUART_IEN_TXBL;
initLeuart();
LEUART0->CMD |= LEUART_CMD_TXEN | LEUART_CMD_RXEN;
if(_leuart_status == 'D')
{
	if (counter_tempirq != 0)
	{
		LEUART0->IEN &= ~LEUART_IEN_TXC;
		if(temp<0)
		{
			temp1 = temp;
			if (CircBuff !=0)
					{
					LEUART0->TXDATA = read_delete_item_frombuffer(&_transmit_buffer);
					}
				else
				{
					LEUART0->TXDATA = (int8_t)((temp-temp1)*10);
				}
		}
		else
		{
			if (CircBuff !=0)
					{
					LEUART0->TXDATA = read_delete_item_frombuffer(&_transmit_buffer);
					}
				else
				{
					LEUART0->TXDATA = (uint8_t)(((float)temp-(int)temp)*10);
				}
		}
		counter_tempirq =0;
		while ((LEUART0->STATUS & LEUART_STATUS_TXC)==0);
		LEUART0->CMD |= LEUART_CMD_TXDIS;
	}
	else if(counter_tempirq == 0)
	{
	LEUART0->IEN |= LEUART_IEN_TXC;
	if (CircBuff !=0)
		{
		LEUART0->TXDATA = read_delete_item_frombuffer(&_transmit_buffer);
		}
	else
	{
	LEUART0->TXDATA = (int8_t)temp;
	}
	while ((LEUART0->STATUS & LEUART_STATUS_TXC)==0);
	counter_tempirq++;
	}

}
else if(_leuart_status == 'L')
{
	if (counter_lightirq!=0)
		{
		LEUART0->IEN &= ~LEUART_IEN_TXC;
		if (CircBuff !=0)
				{
				LEUART0->TXDATA = read_delete_item_frombuffer(&_transmit_buffer);
				}
			else
			{
				LEUART0->TXDATA = (uint8_t)_light_status;
			}
		while ((LEUART0->STATUS & LEUART_STATUS_TXC)==0);
		LEUART0->CMD |= LEUART_CMD_TXDIS;
		counter_lightirq = 0;
		}
	else if(counter_lightirq == 0)
	{
	LEUART0->IEN |= LEUART_IEN_TXC;
	if (CircBuff !=0)
			{
			LEUART0->TXDATA = read_delete_item_frombuffer(&_transmit_buffer);
			}
		else
		{
			LEUART0->TXDATA = (uint8_t)'O';
		}
	while ((LEUART0->STATUS & LEUART_STATUS_TXC)==0);
	counter_lightirq++;
	}
}
if(_leuart_status == 'H')
{
	if (counter_tempirq != 0)
	{
		LEUART0->IEN &= ~LEUART_IEN_TXC;

			if (CircBuff !=0)
					{
					LEUART0->TXDATA = read_delete_item_frombuffer(&_transmit_buffer);
					}
				else
				{
					LEUART0->TXDATA = (ACMP_count*4);
				}
		counter_tempirq =0;
		while ((LEUART0->STATUS & LEUART_STATUS_TXC)==0);
		LEUART0->CMD |= LEUART_CMD_TXDIS;
	}
	else if(counter_tempirq == 0)
	{
	LEUART0->IEN |= LEUART_IEN_TXC;
	if (CircBuff !=0)
		{
		LEUART0->TXDATA = read_delete_item_frombuffer(&_transmit_buffer);
		}
	else
	{
	LEUART0->TXDATA = (uint8_t)'H';
	}
	while ((LEUART0->STATUS & LEUART_STATUS_TXC)==0);
	counter_tempirq++;
	}

}
// Flags = LEUART0->IF;
//LEUART0->IFC = Flags;
INT_Enable();
}


void LETOUCH_Init(float sensitivity[]){

  uint8_t i;
  channels_used_mask = 0;
  num_channels_used = 0;

  /* Initialize channels used mask and threshold array for each channel */
  /* Uses the sensitivity array to deduce which channels to enable and how */
  /* many channels that are enabled */

  for(i = 0; i < NUM_LESENSE_CHANNELS; i++){

    /* Init min and max values for each channel */
    channel_max_value[i] = 0;
    channel_min_value[i] = 0xffff;

    /* Add to channels used mask if sensitivity is not zero */
    if(sensitivity[i] != 0.0){
      channel_threshold_percent[i] = sensitivity[i];
      channels_used_mask |= (1 << i);
      num_channels_used++;
    }
  }

  /* Disable interrupts while initializing */
  INT_Disable();

  /* Setup CMU. */
  LETOUCH_setupCMU();
  /* Setup GPIO. */
  LETOUCH_setupGPIO();
  /* Setup ACMP. */
  LETOUCH_setupACMP();
  /* Setup LESENSE. */
  LETOUCH_setupLESENSE();
  /* Do initial calibration "N_calibration_values * 10" times to make sure */
  /* it settles on values after potential startup transients */
  for(i = 0; i < NUMBER_OF_CALIBRATION_VALUES * 10; i++){
    LESENSE_ScanStart();
    LETOUCH_Calibration();
  }
  /* Setup RTC for calibration interrupt */
  LETOUCH_setupRTC();
  /* Initialization done, enable interrupts globally. */
  INT_Enable();

}

/***************************************************************************//**
 * @brief
 *   Get the buttons pressed variable, one bit for each channel pressed
 *   or'ed together.
 *
 * @return
 *   The touch buttons/pads that are in touched state is or'ed together and
 *   returned.
 ******************************************************************************/
uint16_t LETOUCH_GetChannelsTouched(void){

  return buttons_pressed;
}

/***************************************************************************//**
 * @brief
 *   Get the maximum value registered for a given channel.
 *
 * @param[in] channel
 *   The channel to get maximum value for
 *
 * @return
 *   The maximum value registered for the given channel.
 ******************************************************************************/
uint16_t LETOUCH_GetChannelMaxValue(uint8_t channel){

  return channel_max_value[channel];
}

/***************************************************************************//**
 * @brief
 *   Get the minimum value registered for a given channel.
 *
 * @param[in] channel
 *   The channel to get minimum value for
 *
 * @return
 *   The minimum value registered for the given channel.
 ******************************************************************************/
uint16_t LETOUCH_GetChannelMinValue(uint8_t channel){

  return channel_min_value[channel];
}

/**************************************************************************//**
 * @brief  Enable clocks for all the peripherals to be used
 *****************************************************************************/
static void LETOUCH_setupCMU( void )
{
  /* Ensure core frequency has been updated */
  SystemCoreClockUpdate();

  /* ACMP */
  CMU_ClockEnable(cmuClock_ACMP0, true);
  CMU_ClockEnable(cmuClock_ACMP1, true);

  /* GPIO */
  CMU_ClockEnable(cmuClock_GPIO, true);

 /* Low energy peripherals
 *   LESENSE
 *   LFXO clock must be enabled prior to enabling
 *   clock for the low energy peripherals */
  CMU_ClockSelectSet(cmuClock_LFA, cmuSelect_LFXO);
  CMU_ClockEnable(cmuClock_CORELE, true);
  CMU_ClockEnable(cmuClock_LESENSE, true);

  /* RTC */
  CMU_ClockEnable(cmuClock_RTC, true);

  /* Disable clock source for LFB clock */
  CMU_ClockSelectSet(cmuClock_LFB, cmuSelect_Disabled);
}

/**************************************************************************//**
 * @brief  Sets up the ACMP
 *****************************************************************************/
static void LETOUCH_setupACMP( void )
{
  /* Configuration structure for ACMP */
  /* See application note document for description of the different settings. */
  static const ACMP_CapsenseInit_TypeDef acmpInit =
  {
    .fullBias                 = true,            //Configured according to application note
    .halfBias                 = true,            //Configured according to application note
    .biasProg                 = 0x5,             //Configured according to application note
    .warmTime                 = acmpWarmTime512, //LESENSE uses a fixed warmup time
    .hysteresisLevel          = acmpHysteresisLevel5, //Configured according to application note
    .resistor                 = acmpResistor0,   //Configured according to application note
    .lowPowerReferenceEnabled = false,           //LP-reference can introduce glitches with captouch
    .vddLevel                 = 0x30,            //Configured according to application note
    .enable                   = false            //LESENSE enables the ACMP
  };

  /* Initialize ACMP in capsense mode*/
  //ACMP_CapsenseInit(ACMP0, &acmpInit);
  ACMP_CapsenseInit(ACMP1, &acmpInit);

}

/**************************************************************************//**
 * @brief  Sets up the LESENSE
 *****************************************************************************/
static void LETOUCH_setupLESENSE( void )
{
  uint8_t i;

  /* LESENSE configuration structure */
  static const LESENSE_Init_TypeDef initLesense =
  {
    .coreCtrl         =
    {
      .scanStart    = lesenseScanStartPeriodic,
      .prsSel       = lesensePRSCh0,
      .scanConfSel  = lesenseScanConfDirMap,
      .invACMP0     = false,
      .invACMP1     = false,
      .dualSample   = false,
      .storeScanRes = false,
      .bufOverWr    = true,
      .bufTrigLevel = lesenseBufTrigHalf,
      .wakeupOnDMA  = lesenseDMAWakeUpDisable,
      .biasMode     = lesenseBiasModeDutyCycle,
      .debugRun     = false
    },

    .timeCtrl         =
    {
      .startDelay     = 0x0
    },

    .perCtrl          =
    {
      .dacCh0Data     = lesenseDACIfData,
      .dacCh0ConvMode = lesenseDACConvModeDisable,
      .dacCh0OutMode  = lesenseDACOutModeDisable,
      .dacCh1Data     = lesenseDACIfData,
      .dacCh1ConvMode = lesenseDACConvModeDisable,
      .dacCh1OutMode  = lesenseDACOutModeDisable,
      .dacPresc       = 0,
      .dacRef         = lesenseDACRefBandGap,
      .acmp0Mode      = lesenseACMPModeDisable,   // only acmp mux controlled by lesense
      .acmp1Mode      = lesenseACMPModeMux,   // only acmp mux controlled by lesense
      .warmupMode     = lesenseWarmupModeNormal
    },

    .decCtrl          =


    {
      .decInput  = lesenseDecInputSensorSt,
      .initState = 0,
      .chkState  = false,
      .intMap    = false,
      .hystPRS0  = false,
      .hystPRS1  = false,
      .hystPRS2  = false,
      .hystIRQ   = false,
      .prsCount  = false,
      .prsChSel0 = lesensePRSCh0,
      .prsChSel1 = lesensePRSCh1,
      .prsChSel2 = lesensePRSCh2,
      .prsChSel3 = lesensePRSCh3
    }
  };

  /* Channel configuration */
  static const LESENSE_ChDesc_TypeDef initLesenseCh =
  {
    .enaScanCh     = true,
    .enaPin        = true,
    .enaInt        = true,
    .chPinExMode   = lesenseChPinExDis,
    .chPinIdleMode = lesenseChPinIdleDis,
    .useAltEx      = false,
    .shiftRes      = false,
    .invRes        = false,
    .storeCntRes   = true,
    .exClk         = lesenseClkLF,
    .sampleClk     = lesenseClkLF,
    .exTime        = 0x0,
    .sampleDelay   = SAMPLE_DELAY,
    .measDelay     = 0x0,
    .acmpThres     = 0x0,                   // don't care, configured by ACMPInit
    .sampleMode    = lesenseSampleModeCounter,
    .intMode       = lesenseSetIntLevel,
    .cntThres      = 0x0,                   // Configured later by calibration function
    .compMode      = lesenseCompModeLess
  };

  /* Initialize LESENSE interface _with_ RESET. */
  LESENSE_Init(&initLesense, true);

  /* Configure channels */
  for(i = 0; i < NUM_LESENSE_CHANNELS; i++){
    if((channels_used_mask >> i) & 0x1){
      LESENSE_ChannelConfig(&initLesenseCh, i);
    }
  }

  /* Set scan frequency */
  LESENSE_ScanFreqSet(0, LESENSE_SCAN_FREQUENCY);

  /* Set clock divisor for LF clock. */
  LESENSE_ClkDivSet(lesenseClkLF, lesenseClkDiv_1);

  /* Enable interrupt in NVIC. */
  NVIC_EnableIRQ(LESENSE_IRQn);

  /* Start scan. */
  LESENSE_ScanStart();
}

/**************************************************************************//**
 * @brief  Sets up the GPIO
 *****************************************************************************/
static void LETOUCH_setupGPIO( void )
{
  unsigned int i;

  /* Set GPIO pin mode to disabled for all active pins */
  for(i = 0; i < NUM_LESENSE_CHANNELS; i++){
    if((channels_used_mask >> i) & 0x1){
      GPIO_PinModeSet(LESENSE_CH_PORT, i, gpioModeDisabled, 0);
    }
  }
}

/**************************************************************************//**
 * @brief  Sets up the RTC
 *****************************************************************************/
void LETOUCH_setupRTC( void )
{
  /* RTC configuration */
  static const RTC_Init_TypeDef rtcInit =
  {
    .enable   = true,
    .debugRun = false,
    .comp0Top = true
  };

  RTC_Init(&rtcInit);

  /* Set the RTC calibration interrupt compare value */
  /* calibration interval defined in lesense_letouch_config.h */
  RTC_CompareSet( 0, CALIBRATION_INTERVAL * RTC_FREQ );

  RTC_IntEnable(RTC_IFS_COMP0);
  NVIC_EnableIRQ(RTC_IRQn);
}

/**************************************************************************//**
 * Calibration function
*****************************************************************************/
void LETOUCH_Calibration( void ){
  int i,k;
  uint16_t nominal_count;
  static uint8_t calibration_value_index = 0;

  /* Wait for current scan to finish */
  while(LESENSE->STATUS & LESENSE_STATUS_SCANACTIVE);

  /* Get position for first channel data in count buffer from lesense write pointer */
  k = ((LESENSE->PTR & _LESENSE_PTR_WR_MASK) >> _LESENSE_PTR_WR_SHIFT);

  /* Handle circular buffer wraparound */
  if(k >= num_channels_used){
    k = k - num_channels_used;
  }
  else{
    k = k - num_channels_used + NUM_LESENSE_CHANNELS;
  }

  /* Fill calibration values array with buffer values */
  for(i = 0; i < NUM_LESENSE_CHANNELS; i++){
    if((channels_used_mask >> i) & 0x1){
      calibration_value[i][calibration_value_index] = LESENSE_ScanResultDataBufferGet(k++);
    }
  }

  /* Wrap around calibration_values_index */
  calibration_value_index++;
  if(calibration_value_index >= NUMBER_OF_CALIBRATION_VALUES){
    calibration_value_index = 0;
  }

  /* Calculate max/min-value for each channel and set threshold */
  for(i = 0; i < NUM_LESENSE_CHANNELS; i++){
    if((channels_used_mask >> i) & 0x1){
      channel_max_value[i] = GetMaxValue(calibration_value[i], NUMBER_OF_CALIBRATION_VALUES);
      channel_min_value[i] = GetMinValue(calibration_value[i], NUMBER_OF_CALIBRATION_VALUES);

      nominal_count = channel_max_value[i];
      LESENSE_ChannelThresSet(i, 0x0,(uint16_t) (nominal_count - ((nominal_count * channel_threshold_percent[i])/100.0)) );
    }
  }
}

/**************************************************************************//**
 * Insertion sort, can be used to implement median filter instead of just using max-value
 * in calibration function.
*****************************************************************************/
/*
static void InsertionSort(uint16_t* A, uint16_t N){
  int i,j;
  uint16_t temp;

  for(i=1; i<N; i++)
  {
    temp = A[i];
    j = i-1;
    while(temp<A[j] && j>=0)
    {
      A[j+1] = A[j];
      j = j-1;
    }
    A[j+1] = temp;
  }
}
*/

/**************************************************************************//**
 * Returns maximum value in input array of size N
*****************************************************************************/
static uint16_t GetMaxValue(volatile uint16_t* A, uint16_t N){
  int i;
  uint16_t max = 0;

  for(i=0; i<N; i++)
  {
    if(max < A[i]){
      max = A[i];
    }
  }
  return max;
}

/**************************************************************************//**
 * Returns minimum value in input array of size N
*****************************************************************************/
static uint16_t GetMinValue(volatile uint16_t* A, uint16_t N){
  int i;
  uint16_t min = 0xffff;

  for(i=0; i<N; i++)
  {
    if(A[i] < min){
      min = A[i];
    }
  }
  return min;
}

/**************************************************************************//**
 * Interrupt handlers
 *****************************************************************************/

/**************************************************************************//**
 * @brief RTC_IRQHandler
 * Interrupt Service Routine for RTC, used for the calibration function
 *****************************************************************************/
void RTC_IRQHandler( void )
{
  /* Clear interrupt flag */
  RTC_IntClear(RTC_IFS_COMP0);

  LETOUCH_Calibration();

  /* Reset counter */
  RTC_CounterReset();
}

/**************************************************************************//**
 * @brief LESENSE_IRQHandler
 * Interrupt Service Routine for LESENSE Interrupt Line
 *****************************************************************************/
void LESENSE_IRQHandler( void )
{
  uint8_t channel, i, valid_touch;
  uint32_t interrupt_flags, tmp, channels_enabled;
  uint16_t threshold_value;

  /* Get interrupt flag */
  interrupt_flags = LESENSE_IntGet();
  /* Clear interrupt flag */
  LESENSE_IntClear(interrupt_flags);

  /* Interrupt handles only one channel at a time */
  /* therefore only first active channel found is handled by the interrupt. */
  for(channel = 0; channel < NUM_LESENSE_CHANNELS; channel++){
    if( (interrupt_flags >> channel) & 0x1 ){
      break;
    }
  }

  /* To filter out possible false touches, the suspected channel is measured several times */
  /* All samples should be below threshold to trigger an actual touch. */

  /* Disable other channels. */
  channels_enabled = LESENSE->CHEN;
  LESENSE->CHEN = 1 << channel;

   /* Evaluate VALIDATE_CNT results for touched channel. */
  valid_touch = 1;

  for(i = 0;i<VALIDATE_CNT;i++){
    /* Start new scan and wait while active. */
    LESENSE_ScanStart();
    while(LESENSE->STATUS & LESENSE_STATUS_SCANACTIVE);

    tmp = LESENSE->SCANRES;
    if((tmp & (1 << channel)) == 0){
      valid_touch = 0;
    }
  }

  /* Enable all channels again. */



  LESENSE->CHEN = channels_enabled;


  if(valid_touch){
    /* If logic was switched clear button flag and set logic back, else set button flag and invert logic. */
    if(LESENSE->CH[channel].EVAL & LESENSE_CH_EVAL_COMP){
      buttons_pressed &= ~(buttons_pressed);
      LESENSE->CH[channel].EVAL &= ~LESENSE_CH_EVAL_COMP;

      threshold_value = LESENSE->CH[channel].EVAL & (_LESENSE_CH_EVAL_COMPTHRES_MASK);
      /* Change threshold value 1 LSB for hysteresis. */
      threshold_value -= 1;
      LESENSE_ChannelThresSet(channel, 0, threshold_value);
    }
    else{
      buttons_pressed |= (1 << channel);
      //if(button_counter==1)
      if(buttons_pressed==0x300 || buttons_pressed==0x500 || buttons_pressed==0x900 || buttons_pressed==0x600 || buttons_pressed==0xA00 || buttons_pressed==0xC00)
      {
      LESENSE->CH[channel].EVAL |= LESENSE_CH_EVAL_COMP;

      threshold_value = LESENSE->CH[channel].EVAL & (_LESENSE_CH_EVAL_COMPTHRES_MASK);
      /* Change threshold value 1 LSB for hysteresis. */
      threshold_value += 1;
      LESENSE_ChannelThresSet(channel, 0, threshold_value);
      button_counter=0;
      }
      //else
      //button_counter++;
    }

  }

  /* Need to reset RTC counter so we don't get new calibration event right after buttons are pushed/released. */
  RTC_CounterReset();

}




/* Defining the LEUART0 initialization data */
LEUART_Init_TypeDef leuart0Init =
{
  .enable   = Disable_Leuart,       /* Deactivate data reception on LEUn_TX pin. */
  .refFreq  = Leuart_reffreq,                    /* Inherit the clock frequenzy from the LEUART clock source */
  .baudrate = Leuart_baudrate,                 /* Baudrate = 9600 bps */
  .databits = Leuart_databits,      /* Each LEUART frame containes 8 databits */
  .parity   = Leuart_parity,       /* No parity bits in use */
  .stopbits = Leuart_stopbits,      /* Setting the number of stop bits in a frame to 2 bitperiods */
};










/*Interrupt routine*/
void LETIMER0_IRQHandler(void)
{


/*Setting the dutycycle and period alternatively on LETIMER*/

	INT_Disable();
 int IntFlags;
 //GPIO_PinOutClear(LED_GPIO_PORT,LED_GPIO_PIN);
 IntFlags=LETIMER0->IF;
 Heart_rate= PCNT_CounterGet(PCNT1);
 /*Setting the dutycycle and period alternatively on LETIMER*/
 if(letimer_acmp_count>0)
 {
 initLeuart();
 _leuart_status = 'H';
     if (CircBuff !=0)
     		{
   	  put_item_tobuffer(&_transmit_buffer,'H');
   				put_item_tobuffer(&_transmit_buffer,(Heart_rate * 4));

   		}
LEUART0->IEN|=LEUART_IEN_TXBL;
 }


if((IntFlags & LETIMER_IF_UF)!=0)
 {

     LETIMER0->IFC=IntFlags;
     /* Starting transfer. Using Basic since every transfer must be initiated
     	 * * by the ADC. if you are using DMA*/

          if(DMA_Switch!=0)
          {
              DMA_ActivateBasic(DMA_CHANNEL_ADC,
     	    	                       true,
     	    	                       false,
     	    	                       (void *)ramBufferAdcData,
     	    	                       (void *)&(ADC0->SINGLEDATA),
     	    	                       ADCSAMPLES - 1);
       /* Setting flag to indicate that transfer is in progress
     	      * will be cleared by call-back function. */

             transferActive=true;
          }

        /*Blocking the sleep mode to EM1 for ADC*/
          blockSleepMode(EM1);
        /*Starting ADC Conversion*/
          ADC0->CMD|=ADC_SINGLESTART;
     letimer_acmp_count++;
     /*ACMP Initialisation*/
     PRS_init();
    PCNT1_Init();
     ACMPInit();
     ACMP_count=0;
         // ACMP_IntEnable(ACMP0, ACMP_IEN_EDGE); /* Enable edge interrupt */
          ACMP_Enable(ACMP0);

                  	/* Wait for warmup */
                  	while (!(ACMP0 ->STATUS & ACMP_STATUS_ACMPACT));


  }
INT_Enable();
}

void LETIMER_setup(void)
{

/*If calibrtaion is considered then calculate the period and excite time using the Oscillation Ratio*/

if(Calibration==1)
{
    LEtimerFreq=period*ULFRCOClock*Osc_ratio;
    LEtimerFreq1= Excite*ULFRCOClock*Osc_ratio;
    LEtimerFreq1+=1; /*Rounding off to 4*/
}

else
{
/*Period and excite time are set for EM0-EM2 using prescalar*/
    if (Lowest_energy==EM0||Lowest_energy==EM1||Lowest_energy==EM2)
    {
        LETIMER0_prescaler= period/2;
	  CMU->LFAPRESC0&= 0xfffff0ff;//clearing the prescaler register
	  CMU->LFAPRESC0|= LETIMER0_prescaler << 8;//shift prescaler into  position
	  LETIMER0_prescaler = 1 << LETIMER0_prescaler ;//the value of prescaler register
	  LEtimerFreq=period*(LFXOClock/LETIMER0_prescaler); // moving the final value to Comp0
	  LEtimerFreq1=Excite*(LFXOClock/LETIMER0_prescaler);
    }

      /*Period and excite time are set for EM3*/
    else
    {
        LEtimerFreq=period*ULFRCOClock;
	  LEtimerFreq1= Excite*ULFRCOClock;
    }
}

const LETIMER_Init_TypeDef letimerInit =
{
   .enable = false,                           /* Don't start counting on initialising till LETIMER enabled. */
   .debugRun       = false,                  /* Counter shall not keep running during debug halt. */
   .rtcComp0Enable = false,                  /* Don't start counting on RTC COMP0 match. */
   .rtcComp1Enable = false,                  /* Don't start counting on RTC COMP1 match. */
   .comp0Top       = true,                   /* Load COMP0 register into CNT when counter underflows. COMP0 is used as TOP */
   .bufTop         = false,                  /* Don't load COMP1 into COMP0 when REP0 reaches 0. */
   .out0Pol        = 0,                      /* Idle value for output 0. */
   .out1Pol        = 0,                      /* Idle value for output 1. */
   .ufoa0          = letimerUFOAPwm,         /* PWM output on output 0 */
   .ufoa1          = letimerUFOAPulse,       /* Pulse output on output 1*/
   .repMode        = letimerRepeatFree       /* Count until stopped */

};


     /*Initiaising the LETIMER*/
       LETIMER_Init(LETIMER0, &letimerInit);
       LETIMER0->CNT=LEtimerFreq;
      /*UF flag is generated*/
       LETIMER_CompareSet(LETIMER0, 0, LEtimerFreq);
     //  LETIMER_CompareSet(LETIMER0, 1, LEtimerFreq1);
       while(LETIMER0->SYNCBUSY!=0);
       LETIMER_IntClear(LETIMER0, LETIMER_IF_UF);
      // LETIMER_IntClear(LETIMER0, LETIMER_IF_COMP1);
       LETIMER_IntClear(LETIMER0, LETIMER_IF_COMP0);
       blockSleepMode(Lowest_energy);
       /* Enable LETIMER0 interrupt vector in NVIC*/
       NVIC_EnableIRQ(LETIMER0_IRQn);

}

/*LETIMER calibration routine while calibrating*/
void LETIMER_Calibration_setup(void)
{

    LETIMER_Enable(LETIMER0,false);
    const LETIMER_Init_TypeDef letimerInit =
    {
    .enable = false,                           /* Don't start counting on initialising till LETIMER enabled. */
    .debugRun       = false,                  /* Counter shall not keep running during debug halt. */
    .rtcComp0Enable = false,                  /* Don't start counting on RTC COMP0 match. */
    .rtcComp1Enable = false,                  /* Don't start counting on RTC COMP1 match. */
     .comp0Top       = true,                   /* Load COMP0 register into CNT when counter underflows. COMP0 is used as TOP */
     .bufTop         = false,                  /* Don't load COMP1 into COMP0 when   REP0 reaches 0. */
     .out0Pol        = 0,                      /* Idle value for output 0. */
     .out1Pol        = 0,                      /* Idle value for output 1. */
     .ufoa0          = letimerUFOAPwm,         /* PWM output on output 0 */
     .ufoa1          = letimerUFOAPulse,       /* Pulse output on output 1*/
     .repMode        = letimerRepeatOneshot    /* Count only once till it reaches zero */
     };

/*Initialie the LETIMER*/
    LETIMER_Init(LETIMER0, &letimerInit);

}


void GPIO_setup(void)
{
	/*Clock enable for GPIO*/
    CMU_ClockEnable(cmuClock_GPIO, true);

/* Configure the drive strength of the ports for the light sensor. */

   GPIO_DriveModeSet(LIGHTSENSE_EXCITE_PORT, gpioDriveModeLowest);
   GPIO_DriveModeSet(LIGHTSENSE_SENSOR_PORT, gpioDriveModeLowest);
   //GPIO_DriveModeSet(LED_GPIO_PORT, gpioDriveModeLowest);
   GPIO_DriveModeSet(LED_1_GPIO_PORT, gpioDriveModeLowest);

/* Initialize the 2 GPIO pins of the light sensor setup. */

  GPIO_PinModeSet(LIGHTSENSE_EXCITE_PORT, LIGHTSENSE_EXCITE_PIN,gpioModePushPull, 0);

   GPIO_PinModeSet(LIGHTSENSE_SENSOR_PORT, LIGHTSENSE_SENSOR_PIN, gpioModeInput, 1);



       /* Configure user led as output */
  //GPIO_PinModeSet(LED_GPIO_PORT, LED_GPIO_PIN, gpioModePushPullDrive, 0);
  GPIO_PinModeSet(LED_1_GPIO_PORT, LED_1_GPIO_PIN, gpioModePushPullDrive, 0);
  /* Configure PE1 as input with filter enable */
  	  GPIO_PinModeSet(Sensor_Interrupt_Port, Sensor_Interrupt_Pin, gpioModeInput, 1);
  	  GPIO_PinModeSet(Sensor_Power_Port,Sensor_Power_Pin,gpioModePushPull,0);
  	GPIO_PinModeSet(ACMP0_CHANNEL0_PORT, ACMP0_CHANNEL0_PIN, gpioModeInput, 1);

 }

/*Timer0 setup routine*/
void Timer0_setup()
{
	/*Timer works with EM1 as lowest energy mode*/
    //blockSleepMode(EM1);
    /*Initialising the Timer*/
    TIMER_Init_TypeDef timer0Init =
    {
      .enable     = false,
      .debugRun   = false,
      .prescale   = timerPrescale1,
      .clkSel     = timerClkSelHFPerClk,
      .fallAction = timerInputActionNone,
      .riseAction = timerInputActionNone,
      .mode       = timerModeUp,
      .dmaClrAct  = false,
      .quadModeX4 = false,
      .oneShot    = false,
      .sync       = false,
    };

/* Configure TIMER */
     TIMER_Init(TIMER0, &timer0Init);

}

/*Timer1 routine*/
void Timer1_setup()
{
	/*Timer initialising*/
    TIMER_Init_TypeDef timer1Init =
        {
          .enable     = false,
          .debugRun   = false,
          .prescale   = timerPrescale1,
          .clkSel     = timerClkSelCascade,
          .fallAction = timerInputActionNone,
          .riseAction = timerInputActionNone,
          .mode       = timerModeUp,
          .dmaClrAct  = false,
          .quadModeX4 = false,
          .oneShot    = false,
          .sync       = true,
        };
   /*Configue Timer1*/
    TIMER_Init(TIMER1, &timer1Init);

}

/*Calibration routine to calculate the oscillation ratio*/
void Calibration_setup(void)
{
	/*Initialise the Timer counters to zero*/
   TIMER0->CNT=0;
   TIMER1->CNT=0;
   /*Load the frequency of LFXO to LETIMER count*/
   LETIMER0->CNT=LFXOClock;
   /*Enable the LETIMER and TIMER*/
   LETIMER_Enable(LETIMER0,true);
   TIMER_Enable(TIMER0,true);
   /*TIMER counts upwards till LETIMER reaches zero*/
   while(LETIMER0->CNT!=0);
   /*Disable the TIMER and obtain the TIMER count value with TIMER1 count in MSB and TIMER0 count in LSB*/
   TIMER_Enable(TIMER0,false);
   Count= (TIMER1->CNT<<16)|TIMER0->CNT;
   /*Disable the LFXO and enable ULFRCO*/
   CMU_OscillatorEnable(cmuOsc_LFXO,false,false);
   CMU_OscillatorEnable(cmuOsc_ULFRCO,true,true);
   CMU_ClockSelectSet(cmuClock_LFA, cmuSelect_ULFRCO);
   /*Clear the TIMER Counters*/
   TIMER0->CNT=0;
   TIMER1->CNT=0;
   /*Load the value of ULFRCO frequency to LETIMER and enable LETIMER & TIMER0*/
   LETIMER0->CNT=ULFRCOClock;
   LETIMER_Enable(LETIMER0,true);
   TIMER_Enable(TIMER0,true);
   /*TIMER Counts upwards til LETIMER reaches zero*/
   while(LETIMER0->CNT!=0);
   /*Disable the TIMER0*/
   TIMER_Enable(TIMER0,false);
   /*Obtain the TIMER count value with TIMER1 count in MSB and TIMER0 count in LSB and calculate the oscillation ratio*/
   Count1=(TIMER1->CNT<<16)|TIMER0->CNT;
   Osc_ratio= (float)Count/(float)Count1;
   CMU_ClockEnable(cmuClock_TIMER0, false);
   CMU_ClockEnable(cmuClock_TIMER1, false);

}

/* Enable clocks required */
void Clock_setup(void)
{
	/* Enable clocks */
	CMU_ClockEnable(cmuClock_CORELE, true);
	CMU_ClockEnable(cmuClock_LETIMER0, true);
	CMU_ClockEnable(cmuClock_DMA, true);
	CMU_ClockEnable(cmuClock_ADC0, true);
	CMU_ClockEnable(cmuClock_ACMP0, true);
	CMU_ClockEnable(cmuClock_I2C1, true);

	  /* Enabling clocks, all other remain disabled */
	  CMU_ClockEnable(cmuClock_GPIO, true);       /* Enable GPIO clock */
	  /* Start LFXO, and use LFXO for low-energy modules */
	         	//
	  //CMU_ClockSelectSet(cmuClock_LFB, cmuSelect_LFXO);
	  CMU_ClockEnable(cmuClock_LEUART0, true);    /* Enable LEUART1 clock */
     if(Calibration==1)
    {

       /* Enable clock for TIMER module */
       CMU_ClockEnable(cmuClock_TIMER0, true);
       CMU_ClockEnable(cmuClock_TIMER1, true);
       CMU_ClockEnable(cmuClock_HFPER, true);
       CMU_OscillatorEnable(cmuOsc_LFXO,true,true);
       CMU_ClockSelectSet(cmuClock_LFA, cmuSelect_LFXO);


    }

    else
    {
       if (Lowest_energy==EM0||Lowest_energy==EM1||Lowest_energy==EM2)
		 {
			 CMU_ClockSelectSet(cmuClock_LFA, cmuSelect_LFXO);

		 }







	 else
	 {
		 CMU_ClockSelectSet(cmuClock_LFA, cmuSelect_ULFRCO);

	 }
    }


 }

void Clock_disable()
{
/* Enable clocks */
	CMU_ClockEnable(cmuClock_CORELE, false);
	CMU_ClockEnable(cmuClock_LETIMER0, false);
	CMU_ClockEnable(cmuClock_DMA, false);
	CMU_ClockEnable(cmuClock_ADC0, false);
	CMU_ClockEnable(cmuClock_ACMP0, false);
	CMU_ClockEnable(cmuClock_I2C1, false);

	  /* Enabling clocks, all other remain disabled */
	  CMU_ClockEnable(cmuClock_GPIO, false);       /* Enable GPIO clock */
	  /* Start LFXO, and use LFXO for low-energy modules */
	         	//
	  //CMU_ClockSelectSet(cmuClock_LFB, cmuSelect_LFXO);
	  CMU_ClockEnable(cmuClock_LEUART0,false);    /* Enable LEUART1 clock */
     if(Calibration==1)
    {

       /* Enable clock for TIMER module */
       CMU_ClockEnable(cmuClock_TIMER0, false);
       CMU_ClockEnable(cmuClock_TIMER1, false);
       CMU_ClockEnable(cmuClock_HFPER, false);
       CMU_OscillatorEnable(cmuOsc_LFXO,false,false);
       CMU_ClockSelectSet(cmuClock_LFA, cmuSelect_LFXO);


    }

    else
    {
       if (Lowest_energy==EM0||Lowest_energy==EM1||Lowest_energy==EM2)
		 {
			 CMU_ClockSelectSet(cmuClock_LFA, cmuSelect_LFXO);

		 }

	 else
	 {
		 CMU_ClockSelectSet(cmuClock_LFA, cmuSelect_ULFRCO);

	 }
    }
}

/* I claim that I have used blockSleepMode routine, sleep routine , unblocksleepmode routine, converttoCelsius routine from Silicon labs and below shows the permission*/
 /* Permission is granted to anyone to use this software for any purpose,
 * including commercial applications, and to alter it and redistribute it
 * freely, subject to the following restrictions:
 *
 * 1. The origin of this software must not be misrepresented; you must not
 *    claim that you wrote the original software.
 * 2. Altered source versions must be plainly marked as such, and must not be
 *    misrepresented as being the original software.
 * 3. This notice may not be removed or altered from any source distribution.*/

/*block sleep routine to block the setup to work only till the lowest energy state*/

void blockSleepMode(int energy_mode)
{
     INT_Disable();
     sleep_block_counter[energy_mode]++;
     INT_Enable();
}
/*sleep routine*/
void sleep()
 {
	if(sleep_block_counter[0]>0)
	{
		return;
	}
	else if(sleep_block_counter[1]>0)
	{
		EMU_EnterEM1();
	}
	else if(sleep_block_counter[2]>0)
	{
		EMU_EnterEM2(true);
	}
	else
	{
		EMU_EnterEM3(true);
	}
 }

/*Unblocks from the lowest energy mode and starts to work in EM0*/
void unblocksleepmode(int energy_mode)
{
	INT_Disable();
	if(sleep_block_counter[energy_mode] > 0)
	{
		sleep_block_counter[energy_mode]--;
	}
	INT_Enable();
}


float convertToCelsius(int32_t adcsamples)
{

    float temp;
    /* Factory calibration temperature from device information page. */
    float cal_temp_0 = (float)((CALIBRATION& CALIBRATION_MASK)>> CALIBRATION_SHIFT);
    float cal_value_0 = (float)((CALIBRATION_ADC& CALIBRATION_ADC_MASK)>> CALIBRATION_ADC_SHIFT);
    /* Temperaturegradient(fromdatasheet) */
    float t_grad= -6.27;
    temp = (cal_temp_0 -((cal_value_0 -adcsamples)  / t_grad));
    return temp;
}







/**************************************************************************//**
 * @brief  Call-back called when transfer is complete
 *****************************************************************************/
void transferComplete(unsigned int channel, bool primary, void *user)
{
	status = 'T';
  (void) channel;
  (void) primary;
  (void) user;
  INT_Disable();
  ADC0->CMD=ADC_CMD_SINGLESTOP;
  initLeuart();

  /* Clearing flag to indicate that transfer is complete */
  transferActive = false;
 /*To calculate the average*/
  summation(ramBufferAdcData);
  /*Checking for Temperature Range*/
  if(Final_temp>LowerLimit_Temp && Final_temp<UpperLimit_Temp)
  {
      GPIO_PinOutSet(LED_1_GPIO_PORT,LED_1_GPIO_PIN);
  }
  else
  {
      GPIO_PinOutClear(LED_1_GPIO_PORT,LED_1_GPIO_PIN);
  }
  _leuart_status = 'D';
    if (CircBuff !=0)
    		{
  	  put_item_tobuffer(&_transmit_buffer,(int8_t)Final_temp);
  			if(Final_temp<0)
  			{
  				temp1 = Final_temp;
  				put_item_tobuffer(&_transmit_buffer,(int8_t)((Final_temp-temp1)*10));
  			}
  			else
  			{
  				put_item_tobuffer(&_transmit_buffer,(uint8_t)(((float)Final_temp-(int)Final_temp)*10));
  			}
  		}

 LEUART0->IEN|=LEUART_IEN_TXBL;

   INT_Enable();
  unblocksleepmode(EM1);
}





/**************************************************************************//**
 * @brief Configure DMA for ADC RAM Transfer
 *****************************************************************************/
void setupDma(void)
{
  DMA_Init_TypeDef        dmaInit;
  DMA_CfgChannel_TypeDef  chnlCfg;
  DMA_CfgDescr_TypeDef    descrCfg;

  /* Initializing the DMA */
  dmaInit.hprot        = 0;
  dmaInit.controlBlock = dmaControlBlock;
  DMA_Init(&dmaInit);

  /* Setting up call-back function */
  cb.cbFunc  = transferComplete;
  cb.userPtr = NULL;

  /* Setting up channel */
  chnlCfg.highPri   = false;
  chnlCfg.enableInt = true;
  chnlCfg.select    = DMAREQ_ADC0_SINGLE;
  chnlCfg.cb        = &cb;
  DMA_CfgChannel(DMA_CHANNEL_ADC, &chnlCfg);

  /* Setting up channel descriptor */
  descrCfg.dstInc  = dmaDataInc2;
  descrCfg.srcInc  = dmaDataIncNone;
  descrCfg.size    = dmaDataSize2;
  descrCfg.arbRate = dmaArbitrate1;
  descrCfg.hprot   = 0;
  DMA_CfgDescr(DMA_CHANNEL_ADC, true, &descrCfg);
}



/**************************************************************************//**
 * @brief To Configure the ADC
 *****************************************************************************/
void setupAdc(void)
{
  ADC_Init_TypeDef        adcInit       = ADC_INIT_DEFAULT;
  ADC_InitSingle_TypeDef  adcInitSingle = ADC_INITSINGLE_DEFAULT;

  /* Configure ADC single mode*/
  adcInit.prescale = ADC_PrescaleCalc(1300000, 0); /* Set the required prescaler */
  ADC_Init(ADC0, &adcInit);

  adcInitSingle.input     = ADC_SINGLECTRL_INPUTSEL_TEMP; /* Input */
/*Intialising*/
  ADC_InitSingle(ADC0, &adcInitSingle);
  /*Setting the REPETITIVE BIT*/
  ADC0->SINGLECTRL|=ADC_SINGLECTRL_REP;
  /*If not using DMA, use the ADC Interrupt Handler*/
  if(DMA_Switch==0)
  {
      ADC0->IEN=ADC_IEN_SINGLE;
	NVIC_EnableIRQ(ADC0_IRQn);
   }
 }




/*Average Routine*/
int16_t summation(volatile uint16_t Bufferdatacalc[ADCSAMPLES])
{
    long int sum=0;
    int32_t avg=0;
    /*Summing up all the samples*/
    int16_t i;
    for(i=0;i<ADCSAMPLES;i++)
    {
    sum+=Bufferdatacalc[i];
    }
    /*Find the average and convert it to the celsius*/
    avg=sum/ADCSAMPLES;
    Final_temp= convertToCelsius(avg);
    return 0;
}
/*Initialising the ACMP*/
void ACMPInit(void)
{

    const ACMP_Init_TypeDef acmp0_init =
	{
	  false,                        /* Full bias current*/
	  false,                        /* Half bias current */
	  7,                            /* Biasprog current configuration */
	  false,                         /* Enable interrupt for falling edge */
	  true,                         /* Disable interrupt for rising edge*/
	  acmpWarmTime256,              /* Warm-up time in clock cycles, >140 cycles for 10us with 14MHz */
	  acmpHysteresisLevel1,         /* Hysteresis configuration */
	  0,                            /* Inactive comparator output value */
	  false,                        /* Enable low power mode */
	  38,                            /* Vdd reference scaling */
	  false,                         /* Enable ACMP */
	 };




        /* Init and set ACMP channels */
    ACMP_Init(ACMP0, &acmp0_init);

    /*Set the input channel to channel 6 which is connected to the input sensor*/
   // ACMP_ChannelSet(ACMP0, Light_Sensor_ACMP_Ref, Light_Sensor_ACMP_Channel);
    /*Set the input channel to channel 6 which is connected to the input sensor*/
        ACMP_ChannelSet(ACMP0, Heartrate_ACMP_Ref, Heartrate_ACMP_Channel);
        //ACMP_IntEnable(ACMP0, ACMP_IEN_EDGE); /* Enable edge interrupt */
       // ACMP_Enable(ACMP0);

        	/* Wait for warmup */
        //	while (!(ACMP0 ->STATUS & ACMP_STATUS_ACMPACT));
 /* Enable interrupts */
    NVIC_ClearPendingIRQ(ACMP0_IRQn);
    NVIC_EnableIRQ(ACMP0_IRQn);
}


void PRS_init()
{
	CMU_ClockEnable(cmuClock_PRS, true);
	/* PRS setup */
	//PRS->SWPULSE|= _PRS_SWPULSE_CH5PULSE_MASK;
	PRS->SWLEVEL|= _PRS_SWLEVEL_CH5LEVEL_MASK;

    PRS_SourceAsyncSignalSet(5,PRS_CH_CTRL_SOURCESEL_ACMP0,PRS_CH_CTRL_SIGSEL_ACMP0OUT);
}


void PCNT1_Init()
{
	/* Select LFRCO as clock source for LFA */
	  //CMU_ClockSelectSet(cmuClock_LFA, cmuSelect_LFXO);

	  /* Enabling all necessary clocks */
	 // CMU_ClockEnable(cmuClock_CORELE, true);     /* Enable CORELE clock */
	  CMU_ClockEnable(cmuClock_GPIO, true);       /* Enable clock for GPIO module */
	  CMU_ClockEnable(cmuClock_PCNT1, true);      /* Enable clock for PCNT module */


	  /* Set configuration for pulse counter */
	    PCNT_Init_TypeDef pcntInit =
	  {
	    .mode       = pcntModeDisable,  /* clocked by LFACLK */
	    .counter    = 0,                  /* Set initial value to 0 */
	    .top        = 0xFF,                 /* Set top to max value */
	    .negEdge    = false,              /* positive edges */
	    .countDown  = false,              /* up count */
	    .filter     = false,               /* filter enabled */
	  };

      /* Initialize Pulse Counter */
	  PCNT_Init(PCNT1, &pcntInit);
	  PCNT1->INPUT|= _PCNT_INPUT_S0PRSSEL_PRSCH5 | PCNT_INPUT_S0PRSEN ;

	  /* Enable PCNT overflow interrupt */
	 // PCNT_IntEnable(PCNT1, PCNT_IF_OF);

	  /* Enable PCNT1 interrupt vector in NVIC */
	 // NVIC_EnableIRQ(PCNT1_IRQn);

	  /* Route PCNT1 input to location 0 -> PCNT1_S0IN on PC4 */
	  //PCNT1->ROUTE = PCNT_ROUTE_LOCATION_LOC0;
	  PCNT1->CTRL |= _PCNT_CTRL_MODE_OVSSINGLE ;
}





/**************************************************************************//**
 * @brief  Main function
 *****************************************************************************/
int main(void)

{
/* Initialize chip */
  CHIP_Init();
  /*Set up the Clock routine and GPIO routine*/



  if(CapSense==1)
  {
  /* Bitmask for the currently touched channels */
    uint16_t channels_touched = 0;

    /* All four slider pads enabled, the position in the array indicates which pin from PC0 to PC15. */

    /* Four pins active, PC8, PC9, PC10 and PC11. A value of 0.0 indicates inactive channel. */
      float sensitivity[] = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,1.0, 1.0, 1.0, 1.0, 0.0, 0.0, 0.0, 0.0};

    /* Init GPIO for LED, turn LED off */
    CMU_ClockEnable(cmuClock_GPIO, true);
    GPIO_PinModeSet(LED_PORT, LED_PIN, gpioModePushPull, 0);

    /* Init Capacitive touch for channels configured in sensitivity array */
    LETOUCH_Init(sensitivity);
     /* If any channels are touched while starting, the calibration will not be correct. */
    /* Wait while channels are touched to be sure we continue in a calibrated state. */
    while(LETOUCH_GetChannelsTouched() != 0);
    /* Enter infinite loop. */
    while (1)
    {
      /* Get channels that are pressed, result is or'ed together */
      channels_touched = LETOUCH_GetChannelsTouched();

      /* Check if any channels are in the touched state. */

      if(channels_touched==0x900 && counter ==0)
      {
    	  /* Turn on LED */
    	  GPIO_PinModeSet(LED_PORT, LED_PIN, gpioModePushPull, 1);

    	   counter++;
    	   /*Set up the Clock routine and GPIO routine*/
    	     Clock_setup();
    	     GPIO_setup();
    	      /*If calibration is 1, setup all the below routines*/
    	     if(Calibration==1)
    	     {
    	    	  LETIMER_Calibration_setup();
    	    	  Timer0_setup();
    	    	  Timer1_setup();
    	    	  Calibration_setup();
    	     }

    	    ACMPInit();
    	    circ_buff();
    	    /*Call the LETIMER Setup routine*/
    	       LETIMER_setup();
    	       if(DMA_Switch==1)
    	                   {
    	                 	  setupDma();
    	                   }

    	                /* Configure ADC Sampling */
    	                   setupAdc();

    	                 	     	  /* Enter EM1 while DMA transfer is active to save power. Note that
    	                 	     	   	       	     	        * interrupts are disabled to prevent the ISR from being triggered
    	                 	     	   	       	     	        * after checking the transferActive flag, but before entering
    	                 	     	   	       	     	        * sleep. If this were to happen, there would be no interrupt to wake
    	                 	     	   	       	     	        * the core again and the MCU would be stuck in EM1. While the
    	                 	     	   	       	     	        * core is in sleep, pending interrupts will still wake up the
    	                 	     	   	       	     	        * core and the ISR will be triggered after interrupts are enabled
    	                 	     	   	       	     	        * again.
    	                 	     	   	       	     	        */
    	                   /* Initialize LEUART */
    	                        initLeuart();


    	       /* Configure DMA transfer from ADC to RAM */


    	      /*Enable the LETIMER Interrupt*/
    	      LETIMER_IntEnable(LETIMER0,LETIMER_IF_UF);
    	      NVIC_EnableIRQ(LETIMER0_IRQn);
    	      LETIMER_Enable(LETIMER0,true);

        	   EMU_EnterEM2(1);

        	   //sleep();
      /* Bitmask for the currently touched channels */
              	       uint16_t channels_touched = 0;

              	       /* All four slider pads enabled, the position in the array indicates which pin from PC0 to PC15. */

              	       /* Four pins active, PC8, PC9, PC10 and PC11. A value of 0.0 indicates inactive channel. */
              	         float sensitivity[] = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,1.0, 1.0, 1.0, 1.0, 0.0, 0.0, 0.0, 0.0};

              	       /* Init GPIO for LED, turn LED off */
              	       CMU_ClockEnable(cmuClock_GPIO, true);
              	   LETOUCH_Init(sensitivity);
                 /* If any channels are touched while starting, the calibration will not be correct. */
                     /* Wait while channels are touched to be sure we continue in a calibrated state. */
                     while(LETOUCH_GetChannelsTouched() != 0);

      }

     else if(channels_touched==0x900 && counter ==1)
      		{
        /* Turn off LED */
        GPIO_PinModeSet(LED_PORT, LED_PIN, gpioModePushPull, 0);
        GPIO_PinModeSet(LED_1_GPIO_PORT, LED_1_GPIO_PIN, gpioModePushPull, 0);
        counter=0;
       Clock_disable();
       /* Bitmask for the currently touched channels */
               	       uint16_t channels_touched = 0;
               	    buttons_pressed &= ~(buttons_pressed);

               	       /* All four slider pads enabled, the position in the array indicates which pin from PC0 to PC15. */

               	       /* Four pins active, PC8, PC9, PC10 and PC11. A value of 0.0 indicates inactive channel. */
               	         float sensitivity[] = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,1.0, 1.0, 1.0, 1.0, 0.0, 0.0, 0.0, 0.0};

               	       /* Init GPIO for LED, turn LED off */
               	       CMU_ClockEnable(cmuClock_GPIO, true);
               	   LETOUCH_Init(sensitivity);
                  /* If any channels are touched while starting, the calibration will not be correct. */
                      /* Wait while channels are touched to be sure we continue in a calibrated state. */
                      while(LETOUCH_GetChannelsTouched() != 0);
      }
      /* Enter EM2 */
                EMU_EnterEM2(true);
    }

  }

}

































