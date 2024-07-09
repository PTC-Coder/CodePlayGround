/***** Includes *****/

#include <stdio.h>
#include <stdint.h>
#include <stdlib.h>
#include <string.h>
#include "mxc_device.h"



#include "mxc_sys.h"
#include "mxc_delay.h"
#include "mxc_errors.h"
#include "sdhc_regs.h"
#include "led.h"
#include "tmr.h"
#include "uart.h"
#include "gpio.h"
#include "board.h"




#include "nvic_table.h"
#include "spi.h"
#include "i2c.h"
#include "sdhc_regs.h"
#include "sdhc_lib.h"
#include "ff.h"
#include "utils.h"
#include "dma.h"
#include "arm_math.h"

#include "DS3231_driver.h"   //Access to the RTC DS3231 peripheral
#include <time.h>

#include "./periphDirectAccess.txt"


// make sure the following is set BEFORE including the decimation filter code
//#define UNALIGNED_SUPPORT_DISABLE

//#include "arm_fir_decimate_fast_q15_bob.h"
#include "arm_fir_decimate_fast_q31_bob.h"


/***** #defines  *****/

/********************************************************************************************/

#define FIRST_SET_RTC true   //undefine this if you don't want to set the time on RTC
							 //Change the values of struct tm newTime to the date you want to set

/********************************************************************************************/
// set recording mode to either 48K/16 bit or 384k/24 bit
// note that the ADC runs at 384K all the time; in the 48K case, we decimate digitall//
//#define FS48K_16BIT
#define FS384K_24BIT
// RECORDING TIME IN DMA Blocks (each DMA block is 21.33 ms)

#define RECORDING_TIME_DMABLOCKS  1000//10000


//Pin 0.20 for ADC clock enable or disable

#define MXC_SPI2_SSIDX 0

//****************************************************************
//SPI defines for SPI1 slave
//****************************************************************

#define SPI_MASTER_IRQ SPI1_IRQn


// defines for DMA
#define DMA_bufflen 8192 // 24-bit words (not bytes)
#define DMA_bufflen_bytes 3*DMA_bufflen //in  bytes, = 8K 24-bit words;after decimation, = 1K 48KHz words).

#define buffLen_deci2x DMA_bufflen/2 // after 2x decimation
#define buffLen_deci4x DMA_bufflen/4 // after 4x decimation
#define buffLen_deci8x DMA_bufflen/8 // after 8x decimation


// 8K 24-bit words;after decimation, = 1K 48KHz words).
// note, since we don't have 24-bit data types, we need to use a 32 bit array to store these
// for now, lets try to use only 16 bits from the byte array;
// the decimation filter is faster for 16 bit



// 1st 2:1 decimator defines
#define decimate_2x_fir_numcoeffs0  7// # of filter taps, multiples of 4 run faster
#define decimate_fir_state_len0  DMA_bufflen + decimate_2x_fir_numcoeffs0 - 1

#define decimate_2x_fir_numcoeffs1  9// # of filter taps, multiples of 4 run faster
#define decimate_fir_state_len1  buffLen_deci2x + decimate_2x_fir_numcoeffs1 - 1


#define decimate_2x_fir_numcoeffs2  33// # of filter taps, multiples of 4 run faster
#define decimate_fir_state_len2  buffLen_deci4x + decimate_2x_fir_numcoeffs2 - 1





// I2C defines
#define I2C_MASTER MXC_I2C0_BUS0
#define I2C_SLAVE_ADDR (0x98) // for a write, read is 99
#define I2C_BYTES 2
#define I2C_FREQ 100000

#define MAX32666_I2C_BUS_3V3_PULLUPS (MXC_I2C2_BUS0)    //***Change the I2C Bus here
#define MAX32666_I2C_CFG_MASTER_MODE (1)
#define MAX32666_I2C_CLK_SPEED (MXC_I2C_STD_MODE)


// ************************ ADC/SPI Defines *******************


#define AD463X_REG_EXIT_CFG_MODE	0x14
#define AD463X_REG_MODES		0x20
#define AD463X_REG_OSCILATOR		0x21
#define AD463X_EXIT_CFG_MODE		NO_OS_BIT(0)
#define AD463X_REG_READ_DUMMY		0x00
#define AD463X_REG_READ		    	NO_OS_BIT(7)

// Error codes used in the Secure CRT functions

#define _SECURECRT_ERRCODE_VALUES_DEFINED
#define EINVAL          22
#define ERANGE          34
#define EILSEQ          42
#define STRUNCATE       80


//****************************************************************
//GPIO defines for ADC
//****************************************************************
//For the CS pin while using the SPI2 on the FTHR
#define MXC_GPIO_PORT_OUT0 MXC_GPIO0

// bit-bang slave-sel during ADC init
#define MXC_GPIO_PIN_OUT16 MXC_GPIO_PIN_16

//Pin 0.20 used for enabling and disabling CNV start signal through 74LV4060
#define MXC_GPIO_PIN_OUT20 MXC_GPIO_PIN_20

//Pin 0.21 use for hardware reset
#define MXC_GPIO_PIN_OUT21 MXC_GPIO_PIN_21

#define MXC_GPIO_PIN_OUT5 MXC_GPIO_PIN_5  // gpio5, pin 6 on feather, green LED on motherboard

#define MXC_GPIO_PIN_OUT3 MXC_GPIO_PIN_3  // gpio3, pin 5 on feather, blue led on motherboard

#define MXC_GPIO_PIN_OUT4 MXC_GPIO_PIN_4  // gpio4, pin 7 on feather

#define MXC_GPIO_PIN_OUT12 MXC_GPIO_PIN_12  // gpio12, pin 4 on feather connected to pin 6 of p8 header, used for timing code

#define MXC_GPIO_PIN_OUT30  MXC_GPIO_PIN_30 //Blue LED on FTHR board (not motherboard!)


// Parameters for PWM output
#define PORT_PWM MXC_GPIO0 //port
#define PIN_PWM MXC_GPIO_PIN_12 //pin
#define FREQ 200000 // (Hz)
#define DUTY_CYCLE 75 // (%)
#define PWM_TIMER MXC_TMR0 // must change PWM_PORT and PWM_PIN if changed

#define STRINGIFY(x) #x
#define TOSTRING(x) STRINGIFY(x)

#define MAXLEN 256


#define WAV_HEADER_SIZE 44
#define WAV_BLOCK_SIZE 512

#define testWriteLen 32768


/****************** TypeDef ******************/
/**
 * @brief Enumerated DS3231 I2C error mode.
 */
typedef enum{
    DS3231_I2C_NO_ERROR,
	MCU_MASTER_I2C_INIT_ERROR,
    DS3231_I2C_INIT_ERROR,
    DS3231_I2C_FREQSET_ERROR
} DS3231_i2cError;


/************************ Globals ******************************/
#define DEFAULT_FILENAME "0:Magpie00_19000101_000000.wav"
char savedFileName[31] = DEFAULT_FILENAME;

//==============DS3231 Related=========================
#define OUTPUT_MSG_BUFFER_SIZE       128U

ds3231_driver_t DS3231_RTC;
static struct tm ds3231_datetime;
static char ds3231_datetime_str[17];
static float ds3231_temperature;
static uint8_t output_msgBuffer[OUTPUT_MSG_BUFFER_SIZE];
const struct tm ds3231_dateTimeDefault = {
	.tm_year = 118U,
	.tm_mon = 00U,
	.tm_mday = 1U,
	.tm_hour = 0U,
	.tm_min = 0U,
	.tm_sec = 0U
};

// structs for spi,gpio


mxc_gpio_cfg_t gpio_out16;
mxc_gpio_cfg_t gpio_out20; // adc clock enable
mxc_gpio_cfg_t gpio_out5; //pin 6 on feather, green LED on motherboard
mxc_gpio_cfg_t gpio_in3; // pin 5 on feather, blue led on motherboard

mxc_gpio_cfg_t gpio_in30; // the blue LED on feather (not on motherboard!)
mxc_gpio_cfg_t gpio_out12; // pin 4 on feather, used for timing etsts

// structs for spi port
mxc_spi_req_t SPI2_req_master_ctrl_write; // use this struct when using spi2 to write to the adc control port
mxc_spi_req_t SPI1_req; // struct for SPI int


volatile uint32_t count_dma_irq = 0;
volatile uint32_t dataBlocksDmaCount=0,dataBlocksConsumedCount=0;
volatile uint8_t dataBlockWriteComplete=1; // init to 1 so the first dma interupt will work
//volatile FSIZE_t file_pos64 = 0;
volatile uint32_t writeNotComplete = 0;
static uint8_t bigDMAbuff[3*3*8192] = {0}; // up to 3 sd card dma interval stalls

FATFS *fs; //FFat Filesystem Object
FATFS fs_obj;
FIL file; //FFat  File Object
FRESULT err; //FFat Result (Struct)
FILINFO fno; //FFat File Information Object
DIR dir; //FFat Directory Object
TCHAR message[32768];
WORD message1[32768];
TCHAR directory[MAXLEN], cwd[MAXLEN], filename[MAXLEN], volume_label[24],
volume = '0';
TCHAR *FF_ERRORS[20];
uint32_t clusters_free = 0, sectors_free = 0, sectors_total = 0, volume_sn = 0;
UINT bytes_written = 0, bytes_read = 0, mounted = 0;
BYTE work[4096];
static uint8_t ADCdata[testWriteLen];





// global DMA var
static uint8_t dmaDestBuff[DMA_bufflen_bytes] = {0}; // bytes transferred here from the dma
//static uint8_t dmaDestBuffCopy[DMA_bufflen_bytes] = {0}; // will need to be longer for SD card stalls!


static q15_t dmaDestBuff_16bit[DMA_bufflen] = {0}; // same data but assembled back into 16-bit words
static q31_t dmaDestBuff_32bit[DMA_bufflen] = {0}; // same data but assembled back into 32-bit words

int mychannel = -1;



// 32-bit vars. To keep things sinple, the DMS always receives 24 bits, and is filtered
// using 32-bit cmsis routines to prevent extra quantizayion noise
// after filtering we can decide to use 16 or 24 bits to write to the sd
// note the 384KHz mode skips the fiters
// input buffers loaded by SPI ISR routine
// decimated buffers
// output of 1st decimator, 8k samples/4 = 2K samples
static q31_t dma_rx_deci_2x[buffLen_deci2x] = {0}; // 1st 2X decimator output
static q31_t dma_rx_deci_4x[buffLen_deci4x] = {0}; // 2nd 2X decimator
volatile q31_t dma_rx_deci_8x[buffLen_deci8x] = {0}; // 3rd 2X decimator
static q15_t dma_rx_deci_16bits_8x[buffLen_deci8x] = {0}; // 3rd 2X decimator



// note, 1st stage coeffcients have -3dB loss to avoid wrap-around in case of filter overshoot on a transient edge
static q31_t firCoeffs_stage0[decimate_2x_fir_numcoeffs0] = {
-42201666,18023525,423595866,727113801,423595866,18023525,-42201666};

static q31_t firCoeffs_stage1[decimate_2x_fir_numcoeffs1] = {
-35829136,-93392547,90204797,624894336,955274946,624894336,90204797,-93392547,-35829136};

static q31_t firCoeffs_stage2[decimate_2x_fir_numcoeffs2] = {
-2823963,804105,13756249,13832557,-12099816,-21810016,17681236,39284877,-22118934,-66381589,26258540,112809109,-29540929,-212849373,31655722,678451831,1041361918,678451831,31655722,-212849373,-29540929,112809109,26258540,-66381589,-22118934,39284877,17681236,-21810016,-12099816,13832557,13756249,804105,-2823963};



// the decimation filter state registers, required by the CMSIS routines
static q31_t firState_stage0[decimate_fir_state_len0] = {0};
static q31_t firState_stage1[decimate_fir_state_len1] = {0};
static q31_t firState_stage2[decimate_fir_state_len2] = {0};






static signed long int ozone_scope_var=0;

uint32_t getFifo=0;
int location = 0;
volatile uint32_t delta=0,deltaMax=0,deltaMaxi=0;
volatile uint32_t deltaTrace[256] = {0};
volatile uint32_t deltaTracei[256] = {0};


static uint32_t blockPtrModulo=0;
static uint32_t offset = 0;
static uint32_t blockPtrModuloDMA=0;
static uint32_t offsetDMA = 0;

uint8_t SPI1_rx_3byte[3];

//static q15_t spi_data_q15; // use for 16-bit

// I2C variables
static uint8_t i2cErr=0;
static int I2C_FLAG=0;

static uint8_t i2c_txdata[I2C_BYTES] = {0x1};
static uint8_t i2c_rxdata[I2C_BYTES] = {0x1};


// debug variables
// use volatile so compiler does not optimize these variables away when using -o2
volatile uint32_t debug0=0,debug1=0,debug2=0,debug3 = 0;
//static uint32_t temp0=0,temp1=0,temp2=0,temp3=0;
volatile uint32_t temp0=0,temp1=0,temp2=0,temp3=0;
volatile uint32_t errCount=0;


// function prototypes



// CMSIS instances. Note that the "fast" version
// uses the same structure as the regular version
arm_fir_decimate_instance_q31 Sdeci_2x_0;
arm_fir_decimate_instance_q31 Sdeci_2x_1;
arm_fir_decimate_instance_q31 Sdeci_2x_2;




// *********** start functions *******************



int mount()
{
	fs = &fs_obj;
	if ((err = f_mount(fs, "", 1)) != FR_OK) { //Mount the default drive to fs now
		debug1=20;
		//printf("Error opening SD card: %s\n", FF_ERRORS[err]);
		f_mount(NULL, "", 0);
	} else {
		debug1=22;
		//printf("SD card mounted.\n");
		mounted = 1;
	}

	f_getcwd(cwd, sizeof(cwd)); //Set the Current working directory

	return err;
}

int umount()
{
	if ((err = f_mount(NULL, "", 0)) != FR_OK) { //Unmount the default drive from its mount point
		printf("Error unmounting volume: %s\n", FF_ERRORS[err]);
	} else {
		printf("SD card unmounted.\n");
		mounted = 0;
	}

	return err;
}


//void PWMTimer()
//{
//    // Declare variables
//    mxc_gpio_cfg_t gpio_pwm; //to configure GPIO
//    mxc_tmr_cfg_t tmr; // to configure timer
//    unsigned int periodTicks = PeripheralClock / FREQ;
//    unsigned int dutyTicks = periodTicks * DUTY_CYCLE / 100;
//    debug1 = dutyTicks;
//    debug2 = periodTicks;
//
//    // Congfigure GPIO port and pin for PWM
//    gpio_pwm.port = PORT_PWM;
//    gpio_pwm.mask = PIN_PWM;
//    gpio_pwm.pad = MXC_GPIO_PAD_PULL_DOWN;
//    gpio_pwm.func = MXC_GPIO_FUNC_ALT4;
//    MXC_GPIO_Config(&gpio_pwm);
//
//    /*
//    Steps for configuring a timer for PWM mode:
//    1. Disable the timer
//    2. Set the pre-scale value
//    3. Set polarity, PWM parameters
//    4. Configure the timer for PWM mode
//    5. Enable Timer
//    */
//
//    MXC_TMR_Shutdown(PWM_TIMER);
//
//    tmr.pres = TMR_PRES_1;
//    tmr.mode = TMR_MODE_PWM;
//    tmr.cmp_cnt = periodTicks;
//    tmr.pol = 1;
//
//    MXC_TMR_Init(PWM_TIMER, &tmr);
//
//    if (MXC_TMR_SetPWM(PWM_TIMER, dutyTicks) != E_NO_ERROR) {
//        //printf("Failed TMR_PWMConfig.\n");
//    }
//
//    MXC_TMR_Start(PWM_TIMER);
//
//}
//
//
//
//void read_TMR0_regs() // for debug only
//{
//	//TMR0_CNT_direct_probe =  TMR0_CNT_direct;
//	TMR0_CMP_direct_probe =  TMR0_CMP_direct;
//	TMR0_PWM_direct_probe =  TMR0_PWM_direct;
//	//TMR0_INTR_direct_probe =  TMR0_INTR_direct;
//	//TMR0_CN_direct_probe =  TMR0_CN_direct;
//	//TMR0_NOLCMP_direct_probe =  TMR0_NOLCMP_direct;
//}
//



void read_SPI1_regs() // for debug only
{
	// note, these appear in the same order as in the user guide
	SPI1_DATA0_direct_probe =  SPI1_DATA0_direct;
	SPI1_CTRL0_direct_probe =  SPI1_CTRL0_direct;
	SPI1_CTRL1_direct_probe =  SPI1_CTRL1_direct;
	SPI1_CTRL2_direct_probe =  SPI1_CTRL2_direct;
	SPI1_SS_TIME_direct_probe =  SPI1_SS_TIME_direct;
	SPI1_CLK_CFG_direct_probe =  SPI1_CLK_CFG_direct;
	SPI1_DMA_direct_probe =  SPI1_DMA_direct;
	SPI1_INT_FL_direct_probe =  SPI1_INT_FL_direct;
	SPI1_INT_EN_direct_probe =  SPI1_INT_EN_direct;
	SPI1_WAKE_FL_direct_probe =  SPI1_WAKE_FL_direct;
	SPI1_WAKE_EN_direct_probe =  SPI1_WAKE_EN_direct;
	SPI1_STAT_direct_probe =  SPI1_STAT_direct;
}

void read_SPI2_regs() // for debug only
{
	// note, these appear in the same order as in the user guide
	SPI2_DATA0_direct_probe =  SPI2_DATA0_direct;
	SPI2_CTRL0_direct_probe =  SPI2_CTRL0_direct;
	SPI2_CTRL1_direct_probe =  SPI2_CTRL1_direct;
	SPI2_CTRL2_direct_probe =  SPI2_CTRL2_direct;
	SPI2_SS_TIME_direct_probe =  SPI2_SS_TIME_direct;
	SPI2_CLK_CFG_direct_probe =  SPI2_CLK_CFG_direct;
	SPI2_DMA_direct_probe =  SPI2_DMA_direct;
	SPI2_INT_FL_direct_probe =  SPI2_INT_FL_direct;
	SPI2_INT_EN_direct_probe =  SPI2_INT_EN_direct;
	SPI2_WAKE_FL_direct_probe =  SPI2_WAKE_FL_direct;
	SPI2_WAKE_EN_direct_probe =  SPI2_WAKE_EN_direct;
	SPI2_STAT_direct_probe =  SPI2_STAT_direct;

}


void read_DMA0_regs() // for debug only
{
	// note, these appear in the same order as in the user guide

	DMA0_CTRL_direct_probe = DMA0_CTRL_direct;
	DMA0_INTR_direct_probe  = DMA0_INTR_direct;
	DMA0_CH0_CFG_direct_probe = DMA0_CH0_CFG_direct;
	DMA0_CH0_ST_direct_probe = DMA0_CH0_ST_direct;
	DMA0_CH0_SRC_direct_probe = DMA0_CH0_SRC_direct;
	DMA0_CH0_DST_direct_probe = DMA0_CH0_DST_direct;
	DMA0_CH0_CNT_direct_probe = DMA0_CH0_CNT_direct;
	DMA0_CH0_SRC_RLD_direct_probe = DMA0_CH0_SRC_RLD_direct;
	DMA0_CH0_DST_RLD_direct_probe = DMA0_CH0_DST_RLD_direct;
	DMA0_CH0_CNT_RLD_direct_probe = DMA0_CH0_CNT_RLD_direct;
}




void reset_adc()
{
	mxc_gpio_cfg_t gpio_out21;
	gpio_out21.port =MXC_GPIO_PORT_OUT0;
	gpio_out21.mask= MXC_GPIO_PIN_OUT21;
	gpio_out21.pad = MXC_GPIO_PAD_NONE;
	gpio_out21.func = MXC_GPIO_FUNC_OUT;
	gpio_out21.vssel = MXC_GPIO_VSSEL_VDDIO;
	MXC_GPIO_Config(&gpio_out21);
	MXC_GPIO_OutClr(gpio_out21.port,gpio_out21.mask); // start with ADC disabled
	MXC_Delay(100000);
	MXC_GPIO_OutSet(gpio_out21.port,gpio_out21.mask); // enable ADC
}






// set the ADC into host clock mode; this outputs a 20MHz sclk on the busy pin
// that can be used to drive SPI 1 in slave mode
// so we don't need to do this ourselves externally
// must connect the busy pin to spi 1 sclk (requires board hack)
void set_adc_host_clock_mode()
{
	int ret;
	uint16_t reg_addr = 0;
	// over-rides the GPIO control of SS, so do all the ADC register init writes first

	uint8_t SPI2_tx_3byte[3];
	uint8_t SPI2_rx_3byte[3];

	// configure gpio bit-bang AFTER spi2 init
	gpio_out16.port = MXC_GPIO_PORT_OUT0;
	gpio_out16.mask = MXC_GPIO_PIN_OUT16;
	gpio_out16.pad = MXC_GPIO_PAD_NONE;
	gpio_out16.func = MXC_GPIO_FUNC_OUT;
	gpio_out16.vssel = MXC_GPIO_VSSEL_VDDIO;
	gpio_out16.drvstr = MXC_GPIO_DRVSTR_3;
	ret = MXC_GPIO_Config(&gpio_out16);
	MXC_GPIO_OutSet(gpio_out16.port, gpio_out16.mask); // SS high

	ret = MXC_SPI_Init(MXC_SPI2, 1, 0, 1, 0, 5000000, MAP_A);
	if (ret != E_NO_ERROR) {
		   debug1 = 2;
		}

	//Setting the data size
	ret = MXC_SPI_SetDataSize(MXC_SPI2, 8);// 8 bits/char
	if (ret != E_NO_ERROR) {
		   debug1 = 2;
		}
	//Setting width of the SPI in this case 4 wire for spi2 master
	ret = MXC_SPI_SetWidth(MXC_SPI2, SPI_WIDTH_STANDARD);
	if (ret != E_NO_ERROR) {
		   debug1 = 2;
		}
	//Setting the SPI mode
	ret = MXC_SPI_SetMode(MXC_SPI2, SPI_MODE_0);
	if (ret != E_NO_ERROR) {
		   debug1 = 2;
		}

	// Master 2 register write parameters
	SPI2_req_master_ctrl_write.spi = MXC_SPI2;
	SPI2_req_master_ctrl_write.txData = (uint8_t *)SPI2_tx_3byte;
	SPI2_req_master_ctrl_write.rxData = (uint8_t *)SPI2_rx_3byte;
	SPI2_req_master_ctrl_write.txLen = 3;
	SPI2_req_master_ctrl_write.rxLen = 1;
	SPI2_req_master_ctrl_write.ssIdx = MXC_SPI2_SSIDX;
	SPI2_req_master_ctrl_write.ssDeassert = 1;
	SPI2_req_master_ctrl_write.txCnt = 0;
	SPI2_req_master_ctrl_write.rxCnt = 0;
	SPI2_req_master_ctrl_write.completeCB = NULL;

// put in host clk mode
	// Separating first byte of 16 bit address and making sure the first bit is 1 for read operation
	reg_addr = 0x3fff;
	SPI2_tx_3byte[0] = AD463X_REG_READ |((reg_addr >> 8) & 0x7F); // Separating first byte of 16 bit address
	SPI2_tx_3byte[1] = (uint8_t)reg_addr;  //Separating second byte of address
	SPI2_tx_3byte[2] = AD463X_REG_READ_DUMMY;
	//	**** Perform Transaction ****
	MXC_GPIO_OutClr(gpio_out16.port, gpio_out16.mask);
	ret = MXC_SPI_MasterTransaction(&SPI2_req_master_ctrl_write);
	if (ret != E_NO_ERROR) {
		   debug1 = 2;
		}
	MXC_GPIO_OutSet(gpio_out16.port, gpio_out16.mask);
	MXC_Delay(4);
// ** now write clock divider for host clock mode
	reg_addr = AD463X_REG_OSCILATOR; // 0x21
	SPI2_tx_3byte[0] = ((reg_addr >> 8) & 0x7F); // Separating first byte of 16 bit address
	SPI2_tx_3byte[1] = (uint8_t)reg_addr;  //Separating second byte of address
	SPI2_tx_3byte[2] = 0x02;
	//	**** Perform Transaction ****
	MXC_GPIO_OutClr(gpio_out16.port, gpio_out16.mask);
	MXC_Delay(4);
	MXC_SPI_MasterTransaction(&SPI2_req_master_ctrl_write);
	MXC_GPIO_OutSet(gpio_out16.port, gpio_out16.mask);
	MXC_Delay(4);
	// ** now write host clock mode
	// ** note, once this is set, an sclk burst will appear on BUSY, but only when Slave Sel falls
	reg_addr = AD463X_REG_MODES; // 0x20
	SPI2_tx_3byte[0] = ((reg_addr >> 8) & 0x7F); // Separating first byte of 16 bit address
	SPI2_tx_3byte[1] = (uint8_t)reg_addr;  //Separating second byte of address
	SPI2_tx_3byte[2] = 0x20;
	//SPI2_tx_3byte[2] = 0x00;
	//	**** Perform Transaction ****
	MXC_GPIO_OutClr(gpio_out16.port, gpio_out16.mask);
	MXC_Delay(4);
	MXC_SPI_MasterTransaction(&SPI2_req_master_ctrl_write);
	MXC_GPIO_OutSet(gpio_out16.port, gpio_out16.mask);
	MXC_Delay(4);


	// now exit host mode
	reg_addr = AD463X_REG_EXIT_CFG_MODE; // 0x14
	SPI2_tx_3byte[0] = ((reg_addr >> 8) & 0x7F); // Separating first byte of 16 bit address
	SPI2_tx_3byte[1] = (uint8_t)reg_addr;  //Separating second byte of address
	SPI2_tx_3byte[2] = AD463X_EXIT_CFG_MODE; // 0x01
	//	**** Perform Transaction ****
	MXC_GPIO_OutClr(gpio_out16.port, gpio_out16.mask);
	MXC_Delay(4);

	MXC_SPI_MasterTransaction(&SPI2_req_master_ctrl_write);
	MXC_Delay(4);
	MXC_GPIO_OutSet(gpio_out16.port, gpio_out16.mask);
	MXC_Delay(100);
	// now shutdown SPI2 and the bit-bang GPIO SS control
	MXC_SPI_Shutdown(MXC_SPI2);
	// make gpio tri-state so ADC SS cab be driven by ext FS soure
	gpio_out16.func = MXC_GPIO_FUNC_IN;
	MXC_GPIO_Config(&gpio_out16);
	MXC_GPIO_Shutdown(gpio_out16.mask); // let the ADC clock series resistor drive the SPI1_Slave SS pin

}




// spi 1 receives the stream adc input and is connected to the dma, so it must be a slave.
// warning, board hacks are required to make spi1 work as a slave.
// its complicated because spi2 is set up to write to the adc control registers
// using gpio bit-bang for the slave select, but in adc-streaming mode, the
// slave select must be driven externally. So we connect the adc convert signal
// (a square wave at fs) to the slave-select pin using a resistor, When the spi2 control
// writes are done, the gpio is tri-stated and the pin then is driven through the resistor.
// This works but is pretty hacky, and a better solution should be found using
// a modified timing generator using external logic, with a tri-state output that
// can be enabled sing a gpio, so the slave select pin can either drive the output or be driven
// from the timing generator

void spi1_init_slave()
{
	// over-rides the GPIO control of SS, so do all the ADC register init writes first
	SPI1_req.spi = MXC_SPI1;
	SPI1_req.txData = NULL;
	SPI1_req.rxData = (uint8_t *)SPI1_rx_3byte;
	SPI1_req.txLen = 0;
	SPI1_req.rxLen = 3; // chars
	SPI1_req.ssIdx = 0;
	SPI1_req.ssDeassert = 1;
	SPI1_req.txCnt = 0;
	SPI1_req.rxCnt = 0;
	SPI1_req.completeCB = NULL;

	MXC_SPI_Init(MXC_SPI1, 0, 0, 0, 0, 0, MAP_A);
	//Setting the data size
	MXC_SPI_SetDataSize(MXC_SPI1, 8);// bits/char, each spi trans takes in 3 chars = 24 bits
	//Setting width of the SPI in this case 3- wire SPI for SPI1 master
	MXC_SPI_SetWidth(MXC_SPI1, SPI_WIDTH_3WIRE);
	//Setting the SPI mode
	MXC_SPI_SetMode(MXC_SPI1, SPI_MODE_1);

	MXC_SPI_SlaveTransactionAsync(&SPI1_req); // complete the init; don't use the data!

	MXC_SPI_SetRXThreshold(MXC_SPI1, 23); // threshold of 24 bytes ( 8 samples of 3 bytes each) to trigger dma
	SPI1_CTRL0_direct &= 0xfffffffe; // disable the port
	MXC_SPI_ClearRXFIFO(MXC_SPI1); // clear the fifo, start only on pos edge of Slave-sel-B



}



//I2C callback function
void I2C_Callback(mxc_i2c_req_t *req, int error)
{
    I2C_FLAG = error;
}


void DMA_CALLBACK_func(int a, int b)
// this gets called by the DMA 1st, and when this returns, it goes directly to the DMA0_IRQHandler()
{

}



void init_dma_MXC()
{
	MXC_DMA_Init(MXC_DMA0);
	mychannel = MXC_DMA_AcquireChannel(MXC_DMA0);

	mxc_dma_srcdst_t dma_transfer;
	dma_transfer.ch = mychannel;
	dma_transfer.source = NULL;
	dma_transfer.dest = &dmaDestBuff[0];
	dma_transfer.len = DMA_bufflen_bytes; // 3 X 8K bytes

	mxc_dma_config_t dma_config;
	dma_config.ch = mychannel;
	dma_config.reqsel = MXC_DMA_REQUEST_SPI1RX;
	dma_config.srcwd = MXC_DMA_WIDTH_BYTE;
	dma_config.dstwd = MXC_DMA_WIDTH_BYTE;
	dma_config.srcinc_en = 0; // this is ignored??
	dma_config.dstinc_en = 1;

	mxc_dma_adv_config_t advConfig;
	advConfig.ch = mychannel;
	advConfig.prio = MXC_DMA_PRIO_HIGH;
	advConfig.reqwait_en = 0;
	advConfig.tosel = MXC_DMA_TIMEOUT_4_CLK;
	advConfig.pssel = MXC_DMA_PRESCALE_DISABLE;
	advConfig.burst_size = 24;


	MXC_DMA_ConfigChannel(dma_config, dma_transfer);
	MXC_DMA_AdvConfigChannel(advConfig);
	MXC_DMA_SetSrcDst(dma_transfer); // is this redundant??
	MXC_DMA_SetSrcReload(dma_transfer);// is this redundant??
	MXC_DMA_SetChannelInterruptEn(mychannel, false, true); // ctz
	MXC_DMA_SetCallback(mychannel, DMA_CALLBACK_func);
	MXC_DMA_EnableInt(mychannel);

}







void write_wav_header(FIL *file) { // from chapGPT
	// 44 byte header with zero-fill
    uint8_t header[WAV_BLOCK_SIZE];
    //zero-fill oit to 512 bytes so that all block SD card writes align to sector size
    // note, the zero-filling occurs after the 44-byte wav header, 512 -44 = 468,
    //which is a multiple of both 3 (for 24-bit) and 2 (for 16-bit); required to not mess up
    // the byte alignment when the real data starts

    // Initialize header with default values
    memset(header, 0, WAV_BLOCK_SIZE);
    // RIFF header
    memcpy(header, "RIFF", 4); // Chunk ID
    // Chunk Size (placeholder, to be updated later)
    // Format
    memcpy(header + 8, "WAVE", 4);

    // fmt subchunk
    memcpy(header + 12, "fmt ", 4); // Subchunk1 ID
    *(uint32_t *)(header + 16) = 16; // Subchunk1 Size (16 for PCM)
    *(WORD *)(header + 20) = 1; // Audio Format (1 for PCM)
    *(WORD *)(header + 22) = 1; // Num Channels (1 for mono)
#ifdef FS48K_16BIT
    *(uint32_t *)(header + 24) = 48000; // Sample Rate (44.1 kHz)
    *(uint32_t *)(header + 28) = 48000 * 1 * 2; // Byte Rate (Sample Rate * Num Channels * BitsPerSample/8)
    *(WORD *)(header + 32) = 1 * 2; // Block Align (Num Channels * BitsPerSample/8)
    *(WORD *)(header + 34) = 16; // Bits Per Sample (16 bits)
#endif
#ifdef FS384K_24BIT
    *(uint32_t *)(header + 24) = 384000; // Sample Rate (44.1 kHz)
      *(uint32_t *)(header + 28) = 384000 * 1 * 3; // Byte Rate (Sample Rate * Num Channels * BitsPerSample/8)
      *(WORD *)(header + 32) = 1 * 3; // Block Align (Num Channels * BitsPerSample/8)
      *(WORD *)(header + 34) = 24; // Bits Per Sample (16 bits)
#endif


    // data subchunk
    memcpy(header + 36, "data", 4); // Subchunk2 ID
    // Subchunk2 Size, 4 bytes (40-43) (placeholder, to be updated later)


    // note the rest of the 512 bytes are already set to 0, so just write the whole block at once

    // Write header to file
    UINT bw;
    f_write(file, header, WAV_BLOCK_SIZE, &bw);
    debug2 = (uint32_t)bw;
    debug1 = 10;
}


// after wav file is closed, go back and find the size, and fill in the missing data in the header
void update_wav_header(FIL *file) { // from CHATgpt
    uint32_t fileSize = f_size(file);
    uint32_t chunkSize = fileSize - 8;
    uint32_t subchunk2Size = fileSize - WAV_HEADER_SIZE;

    // Seek to Chunk Size position and update
    f_lseek(file, 4);
    UINT bw;
    f_write(file, &chunkSize, 4, &bw);

    // Seek to Subchunk2 Size position and update
    f_lseek(file, 40);
    f_write(file, &subchunk2Size, 4, &bw);
}







// ************************* This is where all the work gets done.
// *** 8k input sample buffer appears here and persists for 20ms ***
void DMA0_IRQHandler()
{



#ifdef FS48K_16BIT
	uint32_t k,i,j;
	uint8_t dmaByte2,dmaByte1,dmaByte0;
	q31_t temp0_q31=0,temp1_q31=0,temp2_q31=0,temp3_q31=0;
	uint bw;
    int flags;
    MXC_DMA_Handler(MXC_DMA0);
    flags = MXC_DMA_ChannelGetFlags(mychannel); // clears the cfg enable bit
    MXC_DMA_ChannelClearFlags(mychannel, flags);



	k = DMA_bufflen; // loop counter
	i = 0; // byte pointer
	j=0; // word pointer
	//MXC_GPIO_OutSet(gpio_out12.port,gpio_out12.mask); // timing test

	// DMA byte-to-signed 32 bit word assembly process
	// Note that if the DMA streaming is turned off to run other processes, you
	// will need to use the "stall" mechanism to turn it back on (see main()) in order to insure
	// that you don't start off with partially-written words (1 or 2 bytes) in the SPI fifo
	while(k > 0) {
		// it's good to read the memory from the bottom up, because the low memory
		// will be the first to be over-written with new samples
		dmaByte2 = dmaDestBuff[i++]; //ms byte
		dmaByte1 = dmaDestBuff[i++]; //mid byte
		dmaByte0 = dmaDestBuff[i++]; //ls byte
		//dmaDestBuff_16bit[j++] = (q15_t)((dmaByte2 << 8) | dmaByte1);
		dmaDestBuff_32bit[j++]  = (q31_t)((dmaByte2 << 24) | (dmaByte1 << 16) | (dmaByte0 << 8)); // use for 24-bit case

		k--;

	}

	// do th fast fir filtering; must be compiled with 02 effort
	//I use a multi-rate filter for fastest speed,2:1 for each stage, 3 stages
	// note, for 384K only recording, you can skip the next 3 lines (no filtering).

////	MXC_GPIO_OutSet(gpio_out12.port,gpio_out12.mask); // timing test
	arm_fir_decimate_fast_q31_bob(&Sdeci_2x_0,dmaDestBuff_32bit,dma_rx_deci_2x,DMA_bufflen);
	arm_fir_decimate_fast_q31_bob(&Sdeci_2x_1,dma_rx_deci_2x,dma_rx_deci_4x,buffLen_deci2x);
	arm_fir_decimate_fast_q31_bob(&Sdeci_2x_2,dma_rx_deci_4x,dma_rx_deci_8x,buffLen_deci4x);
//	MXC_GPIO_OutClr(gpio_out12.port,gpio_out12.mask); // timing test

	// timing test results; the 3 filters above take about 9ms, and the dma byte-to-signed-data conversion
	// takes about 2ms. since we have 21.3 ms per DMA frame, we have about 10ms left.
	// this should be plenty of time to blast out data to the the SD cards (I hope!)
	// Note, this result is obtained with -o2 or -o3 compiler effort.
	// With -o1 effort, the times above are worse by almost 2x (leaving very little time for anything else)
	// With standard compiler effort, the dma processing does not finish at all!

	// *****************This is where you write to the SD card********
	// *** If you want to write 48KHz data, you have 1K words available in buffer dma_rx_deci_8x
	// ** If you want to write 384KHz data, you have 8K words in buffer dmaDestBuff_32bit

	arm_q31_to_q15(dma_rx_deci_8x,dma_rx_deci_16bits_8x,buffLen_deci8x); // convert from signed 32-bit to signed 16-bit
	//f_write(&file, dma_rx_deci_16bits_8x, buffLen_deci8x << 1, &bw); // # bytes = 2X word length of buffer, 16 bits

	dataBlocksDmaCount = 1;
    count_dma_irq++;

    // get ready for next dma transfer
    DMA0_CH0_CFG_direct |= 0x3; // enable dma and reload bits
    DMA0_CH0_CNT_RLD_direct |= 0x80000000; // redundant, do I have to do it again here?
#endif


#ifdef FS384K_24BIT

   // notem the data is sadly in big-endian format (location 0 is an msb)
    // but the wave file is little-endian, so we need to swap the
    // MSByte and the LSbyte (the middle byte can stay the same)
    uint32_t k,i,j;
	int flags;
	MXC_DMA_Handler(MXC_DMA0);
	flags = MXC_DMA_ChannelGetFlags(mychannel); // clears the cfg enable bit
	MXC_DMA_ChannelClearFlags(mychannel, flags);




// switch endian-ness while copying
	k = DMA_bufflen; // loop counter, in units of 24-bit samples
	i = 0; // byte pointer
	j=0; // word pointer
//	while(k > 0) {
//		// it's good to read the memory from the bottom up, because the low memory
//		// will be the first to be over-written with new samples
//		dmaDestBuffCopy[i+2] = dmaDestBuff[i]; //ms byte
//		dmaDestBuffCopy[i+1] = dmaDestBuff[i+1]; //mid byte
//		dmaDestBuffCopy[i] = dmaDestBuff[i+2]; //ls byte
//		i+=3;
//		k--;
//
//	}


	while(k > 0) {
		// it's good to read the memory from the bottom up, because the low memory
		// will be the first to be over-written with new samples
		bigDMAbuff[i+2 + offsetDMA] = dmaDestBuff[i]; //ms byte
		bigDMAbuff[i+1 + offsetDMA] = dmaDestBuff[i+1]; //mid byte
		bigDMAbuff[i + offsetDMA] = dmaDestBuff[i+2]; //ls byte
		i+=3;
		k--;

	}

	blockPtrModuloDMA = (blockPtrModuloDMA+1) & 0x0000000f; // wraps at 15
	offsetDMA = blockPtrModuloDMA*DMA_bufflen_bytes;

	dataBlocksDmaCount+= 1;
	//dataBlockWriteComplete = 0;
	count_dma_irq++;


	// get ready for next dma transfer
	DMA0_CH0_CFG_direct |= 0x3; // enable dma and reload bits
	DMA0_CH0_CNT_RLD_direct |= 0x80000000; // redundant, do I have to do it again here?

#endif



}

/*********************************************************************************************
*
* @name     DS3231_I2C_init
*
* @brief Function to initialize the USCI B1 peripheral. Setting this up
*           as a I2C bus.
*
*           P1_15 - SCL
*           P1_16 - SDA
*
* @return DS3231_i2cError enum type:   DS3231_I2C_NO_ERROR, 
*                                      DS3231_I2C_INIT_ERROR,
*                                      DS3231_I2C_FREQSET_ERROR
*
**********************************************************************************************/
DS3231_i2cError DS3231_I2C_init()
{
  if (MXC_I2C_Init(MAX32666_I2C_BUS_3V3_PULLUPS, MAX32666_I2C_CFG_MASTER_MODE, 0) != E_NO_ERROR)
  {
	printf("Unable to initialize I2C.\n");
    return MCU_MASTER_I2C_INIT_ERROR;
  }

  // I2C pins default to VDDIO for the logical high voltage, we want VDDIOH for 3.3v pullups
  const mxc_gpio_cfg_t i2c_2_pins = {
      .port = MXC_GPIO1,
      .mask = (MXC_GPIO_PIN_14 | MXC_GPIO_PIN_15),
      .pad = MXC_GPIO_PAD_NONE,
      .func = MXC_GPIO_FUNC_ALT1,
      .vssel = MXC_GPIO_VSSEL_VDDIOH,
      .drvstr = MXC_GPIO_DRVSTR_0,
  };
  MXC_GPIO_Config(&i2c_2_pins);

  if (MXC_I2C_SetFrequency(MAX32666_I2C_BUS_3V3_PULLUPS, MAX32666_I2C_CLK_SPEED) != MAX32666_I2C_CLK_SPEED)
  {
	printf("Unable to set I2C frequency.\n");
    return DS3231_I2C_FREQSET_ERROR;
  }

  DS3231_RTC = DS3231_Open();

  if(E_NO_ERROR != DS3231_RTC.init(MAX32666_I2C_BUS_3V3_PULLUPS))
  {
	printf("Unable to initialize RTC driver.\n");
	return DS3231_I2C_INIT_ERROR;
  }
 
  return DS3231_I2C_NO_ERROR;
}

/*=================================================================================
*]]]]]]]]]]]]]]]]]]]]]]]]]]]]]]]]]]]]]]]]]]]]]]]]]]]]]]]]]]]]]]]]]]]]]]]]]]]]]]]]]]
*
*								MAIN MODULE
*
*]]]]]]]]]]]]]]]]]]]]]]]]]]]]]]]]]]]]]]]]]]]]]]]]]]]]]]]]]]]]]]]]]]]]]]]]]]]]]]]]]]
===================================================================================*/

int main(void)
{

	MXC_Delay(MXC_DELAY_SEC(1));
	mxc_sdhc_cfg_t cfg;
	FF_ERRORS[0] = "FR_OK";
	FF_ERRORS[1] = "FR_DISK_ERR";
	FF_ERRORS[2] = "FR_INT_ERR";
	FF_ERRORS[3] = "FR_NOT_READY";
	FF_ERRORS[4] = "FR_NO_FILE";
	FF_ERRORS[5] = "FR_NO_PATH";
	FF_ERRORS[6] = "FR_INVLAID_NAME";
	FF_ERRORS[7] = "FR_DENIED";
	FF_ERRORS[8] = "FR_EXIST";
	FF_ERRORS[9] = "FR_INVALID_OBJECT";
	FF_ERRORS[10] = "FR_WRITE_PROTECTED";
	FF_ERRORS[11] = "FR_INVALID_DRIVE";
	FF_ERRORS[12] = "FR_NOT_ENABLED";
	FF_ERRORS[13] = "FR_NO_FILESYSTEM";
	FF_ERRORS[14] = "FR_MKFS_ABORTED";
	FF_ERRORS[15] = "FR_TIMEOUT";
	FF_ERRORS[16] = "FR_LOCKED";
	FF_ERRORS[17] = "FR_NOT_ENOUGH_CORE";
	FF_ERRORS[18] = "FR_TOO_MANY_OPEN_FILES";
	FF_ERRORS[19] = "FR_INVALID_PARAMETER";
	srand(12347439);
	int run = 1, input = -1;

	int ret=0;

	static uint8_t stall;
	static uint32_t ktrace = 0;

    printf("Initializing .....\n");
  
	//Init DS3231 RTC peripheral
	if(DS3231_I2C_NO_ERROR != DS3231_I2C_init())
	{
		printf("Unable to initialize DS3231 driver.\n");
		LED_On(LED_RED);
		return 1;
	}

	#ifdef FIRST_SET_RTC
	// //Set Date Time to something
	//Year is always Year - 1900
	//Month is 0-11 so subtract 1 from the month you want to set
	//Time is in UTC so set appropriately
	// hour is 0-23
	// min is 0-59
	// sec is 0-59
	struct tm newTime = {
		.tm_year = 2024 - 1900U,
		.tm_mon = 7 - 1U,
		.tm_mday = 8,
		.tm_hour = 21,
		.tm_min = 33,
		.tm_sec = 0
	};

	//Set Date Time on RTC. 
	
	if (E_NO_ERROR != DS3231_RTC.set_datetime(&newTime)) {
		printf("\nDS3231 set time error\n");
	} else {
		strftime((char*)output_msgBuffer, OUTPUT_MSG_BUFFER_SIZE, "\n-->Set DateTime: %F %TZ\r\n", &newTime);
		printf(output_msgBuffer);
	}
	#endif

	//Get Date Time from RTC
	
	if (E_NO_ERROR != DS3231_RTC.read_datetime(&ds3231_datetime, ds3231_datetime_str)) {
		printf("\nDS3231 read datetime error\n");
	} else {
		strftime((char*)output_msgBuffer, OUTPUT_MSG_BUFFER_SIZE, "\n-->DateTime: %F %TZ\r\n", &ds3231_datetime);
		printf(output_msgBuffer);

		strftime((char*)output_msgBuffer, OUTPUT_MSG_BUFFER_SIZE, "\n-->FileStampTime: %Y%m%d_%H%M%SZ\r\n", &ds3231_datetime);
		printf(output_msgBuffer);

		printf(ds3231_datetime_str);
	}
	//time_t currentTime = mktime(ds3231_datetime);

	//Get Temperature from RTC	
	if (E_NO_ERROR != DS3231_RTC.read_temperature(&ds3231_temperature)) {
		printf("\nDS3231 read temperature error\n");
	} else {
		sprintf((char*)output_msgBuffer, "\n-->Temperature: %.2f\r\n", ds3231_temperature);
		printf(output_msgBuffer);
	}
		
	
	
	// init the 3 fir 2x decimation filters

	arm_fir_decimate_init_q31(&Sdeci_2x_0,decimate_2x_fir_numcoeffs0,2,&firCoeffs_stage0[0],&firState_stage0[0],DMA_bufflen);
	arm_fir_decimate_init_q31(&Sdeci_2x_1,decimate_2x_fir_numcoeffs1,2,&firCoeffs_stage1[0],&firState_stage1[0],buffLen_deci2x);
	arm_fir_decimate_init_q31(&Sdeci_2x_2,decimate_2x_fir_numcoeffs2,2,&firCoeffs_stage2[0],&firState_stage2[0],buffLen_deci4x);




	//CARD writes, cluster size (== allocation size) may be 128Kb, whereas sector size is 512 bytes
	cfg.bus_voltage = MXC_SDHC_Bus_Voltage_3_3;
	cfg.block_gap = 0;
	cfg.clk_div = 0x0b0; // Maximum divide ratio, frequency must be <= 400 kHz during Card Identification phase
	// 7us clock period = 130khz, OK
	if (MXC_SDHC_Init(&cfg) != E_NO_ERROR)
	{
		printf("Unable to initialize SDHC driver.\n");
		LED_On(LED_RED);
		return 1;
	}

	
	// wait for card to be inserted
		while (!MXC_SDHC_Card_Inserted()) {
			LED_On(LED_RED);
			MXC_Delay(500000);
			LED_Off(LED_RED);
			MXC_Delay(500000);
		}
	printf("Card inserted.\n");
	LED_On(LED_GREEN);

	//////// FTHR2 SDCARD Enabling ////

    const mxc_gpio_cfg_t sd_card_en_pin = {
    .port = MXC_GPIO1,
    .mask = MXC_GPIO_PIN_6,
    .pad = MXC_GPIO_PAD_NONE,
    .func = MXC_GPIO_FUNC_OUT,
    .vssel = MXC_GPIO_VSSEL_VDDIO,
    .drvstr = MXC_GPIO_DRVSTR_0,
	};

	MXC_GPIO_Config(&sd_card_en_pin);
	MXC_GPIO_OutSet(sd_card_en_pin.port, sd_card_en_pin.mask);

	//////////////////////////////////

	// set up card to get it ready for a transaction
	if (MXC_SDHC_Lib_InitCard(10) == E_NO_ERROR) {
		printf("Card Initialized.\n");
		debug1 = 10;
	} else {
		printf("No card response! Remove card, reset MCU, and try again.\n");
		debug1 = 11;
		LED_Off(LED_GREEN);
		LED_On(LED_RED);
		return -1;
	}

	if (MXC_SDHC_Lib_Get_Card_Type() == CARD_SDHC) {
		printf("Card type: SDHC\n");
		debug1 = 12;
	} else {
		printf("Card type: MMC/eMMC\n");
		debug1 = 13;
	}

	/* Configure for fastest possible clock, must not exceed 52 MHz for eMMC */
	if (SystemCoreClock > 96000000) {
		debug1=16;
		//printf("SD clock ratio (at card) 4:1\n");
		MXC_SDHC_Set_Clock_Config(1);
	} else {
		debug1=17;
		//printf("SD clock ratio (at card) 2:1\n");
		MXC_SDHC_Set_Clock_Config(0);
	}

	// skip formatting
	// for now, format the card every time, so we just make a single file per pass
//	MKFS_PARM format_options = { .fmt = FM_EXFAT };
//
//	if ((err = f_mkfs("", &format_options, work, sizeof(work))) != FR_OK) { //Format the default drive to FAT32
//		printf("Error formatting SD card: %s\n", FF_ERRORS[err]);
//		debug1 = 10;
//	} else {
//		printf("Drive formatted.\n");
//		debug1 = 10;
//	}

	//Turn ON LED to indicate we are starting the recording.
	LED_Off(LED_RED);
	LED_Off(LED_BLUE);
	LED_Off(LED_GREEN);
	for (size_t i = 0; i < 5; i++)
	{
		LED_On(LED_BLUE);
		MXC_Delay(500000);
		LED_Off(LED_BLUE);
		MXC_Delay(500000);
	}
	printf("Start recording.\n");

	LED_On(LED_GREEN);     
	

	// mount the card
	mount();
	// create a file

	//Get Date Time from RTC 
	if(E_NO_ERROR != DS3231_RTC.read_datetime(&ds3231_datetime, ds3231_datetime_str))
	{
		sprintf(savedFileName,"%s",DEFAULT_FILENAME);
	} else {
		
		sprintf(savedFileName,"%s%s%s","0:Magpie00_",ds3231_datetime_str,".wav");
	}
	

	if ((err = f_open(&file, savedFileName, FA_CREATE_ALWAYS | FA_WRITE)) != FR_OK)
	{
		printf("Error opening file: %s\n", FF_ERRORS[err]);
		f_mount(NULL, "", 0);
		return err;
	}
	debug1=11;

	//Writing  Audio header file

	write_wav_header(&file);


	// now write some random data to the file.

//	for(k=0;k < testWriteLen;k++) {
//		ADCdata[k] = (uint8_t)rand();
//
//	}


	debug1= 16;

	// blue led pin 5 on feather
	// gpio_in30.port =MXC_GPIO_PORT_OUT0;
	// gpio_in30.mask= MXC_GPIO_PIN_OUT30;
	// gpio_in30.pad = MXC_GPIO_PAD_NONE;
	// gpio_in30.func = MXC_GPIO_FUNC_OUT;
	// gpio_in30.vssel = MXC_GPIO_VSSEL_VDDIO;
	// MXC_GPIO_Config(&gpio_in30);
	// MXC_GPIO_OutClr(gpio_in30.port,gpio_in30.mask); // set LOW (Led on)


	// gpio 5 is pin 6 on the feather header, connected to motherboard green LED
	gpio_out5.port =MXC_GPIO_PORT_OUT0;
	gpio_out5.mask= MXC_GPIO_PIN_OUT5;
	gpio_out5.pad = MXC_GPIO_PAD_NONE;
	gpio_out5.func = MXC_GPIO_FUNC_OUT;
	gpio_out5.vssel = MXC_GPIO_VSSEL_VDDIO;
	MXC_GPIO_Config(&gpio_out5);
	MXC_GPIO_OutSet(gpio_out5.port,gpio_out5.mask); // LED off


// adc fs clock enable. Note after the spi_init_slave, this will be over-written by the spi definition and will go high
	gpio_out20.port =MXC_GPIO_PORT_OUT0;
	gpio_out20.mask= MXC_GPIO_PIN_OUT20;
	gpio_out20.pad = MXC_GPIO_PAD_NONE;
	gpio_out20.func = MXC_GPIO_FUNC_OUT;
	gpio_out20.vssel = MXC_GPIO_VSSEL_VDDIO;
	MXC_GPIO_Config(&gpio_out20);
	MXC_GPIO_OutClr(gpio_out20.port,gpio_out20.mask); // start with adc clock disabled



	// gpio 3 is pin 5 on the feather header and blue LED on motherboard
	// we will disconnect the blue LED and use this gpio as in input from spi1 slave select
	// so we can syncronize starting the spi port with rising slave-sel-B
	gpio_in3.port =MXC_GPIO_PORT_OUT0;
	gpio_in3.mask= MXC_GPIO_PIN_OUT3;
	gpio_in3.pad = MXC_GPIO_PAD_NONE;
	gpio_in3.func = MXC_GPIO_FUNC_IN;
	gpio_in3.vssel = MXC_GPIO_VSSEL_VDDIO;
	MXC_GPIO_Config(&gpio_in3);

	// gpio 12, pin4 on feather, used for code timing tests
	gpio_out12.port = MXC_GPIO_PORT_OUT0;
	gpio_out12.mask = MXC_GPIO_PIN_OUT12;
	gpio_out12.pad = MXC_GPIO_PAD_NONE;
	gpio_out12.func = MXC_GPIO_FUNC_OUT;
	gpio_out12.vssel = MXC_GPIO_VSSEL_VDDIO;
	gpio_out12.drvstr = MXC_GPIO_DRVSTR_2;
	ret = MXC_GPIO_Config(&gpio_out12);




	// ***  set up I2C, write address 0x98, read addr 0x99

	// i2cErr = MXC_I2C_Init(I2C_MASTER, 1, 0);
	// mxc_i2c_req_t reqMaster;
	// reqMaster.i2c = I2C_MASTER;
	// reqMaster.addr = I2C_SLAVE_ADDR;
	// reqMaster.tx_buf = i2c_txdata;
	// reqMaster.tx_len = I2C_BYTES;
	// reqMaster.rx_buf = i2c_rxdata;
	// reqMaster.rx_len = 0;
	// reqMaster.restart = 0;
	// reqMaster.callback = NULL;
	// reqMaster.callback = I2C_Callback;
	// I2C_FLAG = 1;
	// MXC_I2C_SetFrequency(I2C_MASTER, I2C_FREQ);
	// i2cErr = MXC_I2C_MasterTransaction(&reqMaster);
	// MXC_Delay(10000);

	//reset_adc();
	MXC_Delay(100000);

//	// test reg read/write to adc
//	uint8_t regData=0;
//	ad4630_spi_reg_read_4_wire(0x3fff, regData); // enter configuration mode
//	ad4630_spi_reg_write_4_wire(0x000a, 0xaa); // write to scratchpad
//	ad4630_spi_reg_read_4_wire(0x000a, regData); // read scratchpad
//	ad4630_spi_reg_write_4_wire(0x0014, 0x01); // exit configuration mode
//
//	debug1 = (uint32_t)regData;




	MXC_Delay(100000);
	set_adc_host_clock_mode();
	MXC_Delay(100000);
	MXC_GPIO_OutSet(gpio_out20.port,gpio_out20.mask); // turn on adc clock

	spi1_init_slave();
	// over-rides the GPIO control of SS, so do all the ADC register init writes first
	// note this sets the adc clock enable gpio high accidentally, but its already ON
	debug1 = 11;
	// enable the DMA interupts
	//__enable_irq();
	NVIC_EnableIRQ(DMA0_IRQn);

	init_dma_MXC();

//	PWMTimer();
//	read_TMR0_regs();
	// note, the following could probably be done with MXC functions
	SPI1_DMA_direct = 0b00000000010110000000000000000000; // 24 bytes, enable rx fifo
	SPI1_DMA_direct |= 0x80000000;// receive dma enable


	stall = 1;
	while(stall) { // stall until a rising edge on slave-sel-B. This is to insure we have no partial writes (1 or 2 bytes) that mess up the dma
		temp1  = MXC_GPIO_InGet(gpio_in3.port,gpio_in3.mask); // L
		temp2 =  MXC_GPIO_InGet(gpio_in3.port,gpio_in3.mask); // H
		stall = (!temp2 || temp1);
	}
	SPI1_CTRL0_direct |= 0x00000001; // start the port (fifo was previously cleared)
	MXC_DMA_Start(mychannel); // sets bits 0 and 1 of control reg and bit 31 of count reload reg

	// note, the DMA enable and reload bits need to be set every time
	// in the IRQ handler routine, otherwise it only does a single block transfer





	// note that any variables d in the ISR, and then used here in main, must be of type volatile, for various geeky reasons
#ifdef FS48K_16BIT
	uint bw;
	while(count_dma_irq < RECORDING_TIME_DMABLOCKS) { // interupts happen here, count_dma_irq increments at fs/dmaBlockSize
		if(dataBlocksDmaCount) {
			// note that 16-bit data is stored little-endian, which is the same as the WAV RIFF format, so you can just write it
			f_write(&file, dma_rx_deci_16bits_8x, buffLen_deci8x << 1, &bw); // # bytes = 2X word length of buffer, 16 bits
			dataBlocksDmaCount=0;
		}

	}

#endif

#ifdef FS384K_24BIT
	UINT bw;
	while(count_dma_irq < RECORDING_TIME_DMABLOCKS) { // interupts happen here, count_dma_irq increments at fs/dmaBlockSize
		while((dataBlocksDmaCount - dataBlocksConsumedCount) > 0) { // there is normally a difference of 1, unless the SD card has stalled and the block writes have fallen behind
			delta = dataBlocksDmaCount - dataBlocksConsumedCount;

			// the following 6 lines are debug only to trace what happens during a stall. We can stall up to 16 DMA slots with available memory
			if(delta > 1) {
				deltaTrace[ktrace] = delta;
				deltaTracei[ktrace] = count_dma_irq;
				ktrace++;
//				deltaMax = delta;
//				deltaMaxi = count_dma_irq;
			}
			//LED_On(LED_RED);
			f_write(&file, bigDMAbuff + offset, DMA_bufflen_bytes, &bw); // # bytes = 3X word length of buffer, 24 bits
			dataBlocksConsumedCount+=1;
			blockPtrModulo = (blockPtrModulo+1) & 0x0000000f; // wraps at 15
			offset = blockPtrModulo*DMA_bufflen_bytes;
			//LED_Off(LED_RED);
		}

	}
#endif

	SPI1_CTRL0_direct &= 0xfffffffe; // stop the port
	MXC_DMA_Stop(mychannel);



	if ((err = f_close(&file)) != FR_OK)
	{
		debug1=13;
		f_mount(NULL, "", 0);
		return err;
	}

	//writing is done
	LED_Off(LED_GREEN);

	//Completing the data size update to SD Card
	LED_On(LED_RED);

	// re-open to finish the job
	f_open(&file, savedFileName, FA_READ | FA_WRITE);
	update_wav_header(&file); // go back and fill in the file size (2 different places)
	// close again
	f_close(&file);


	// unmount SD Card

	if ((err = f_mount(NULL, "", 0)) != FR_OK)
	{
		debug1 = 15;
		return err;
	}
	else
	{
		mounted = 0;
	}

	debug1 = count_dma_irq;


	//writing is done
	LED_Off(LED_RED);

	printf("Recording Completed.\n");
	printf("Reset the FTHR2 board to start another recording.\n");

	LED_On(LED_BLUE);

	for(;;){}

} // end of main



