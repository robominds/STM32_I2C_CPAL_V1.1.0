/**
  ******************************************************************************
  * @file    STM32_EVAL_CPAL/STM322xG_EVAL/stm322xg_eval_ioe_cpal.c
  * @author  MCD Application Team
  * @version V1.1.0
  * @date    17-June-2011
  * @brief   This file includes the IO Expander driver for STMPE811 IO Expander
  *          devices.
  *
  *          @note This file is a modified version of stm322xg_eval_ioe.c driver;
  *                I2C CPAL library drivers are used instead of the Standard Peripherals
  *                I2C driver.
  *
  ******************************************************************************
  * @attention
  *
  * THE PRESENT FIRMWARE WHICH IS FOR GUIDANCE ONLY AIMS AT PROVIDING CUSTOMERS
  * WITH CODING INFORMATION REGARDING THEIR PRODUCTS IN ORDER FOR THEM TO SAVE
  * TIME. AS A RESULT, STMICROELECTRONICS SHALL NOT BE HELD LIABLE FOR ANY
  * DIRECT, INDIRECT OR CONSEQUENTIAL DAMAGES WITH RESPECT TO ANY CLAIMS ARISING
  * FROM THE CONTENT OF SUCH FIRMWARE AND/OR THE USE MADE BY CUSTOMERS OF THE
  * CODING INFORMATION CONTAINED HEREIN IN CONNECTION WITH THEIR PRODUCTS.
  *
  * <h2><center>&copy; COPYRIGHT 2011 STMicroelectronics</center></h2>
  ******************************************************************************
  */

  /* File Info : ---------------------------------------------------------------
    This driver has been modified to use the CPAL library instead of the I2C standard
    peripheral driver.

    SUPPORTED FEATURES:
      - IO Read/write : Set/Reset and Read (Polling/Interrupt)
      - Joystick: config and Read (Polling/Interrupt)
      - Touch Screen Features: Single point mode (Polling/Interrupt)
      - TempSensor Feature: accuracy not determined (Polling).

    UNSUPPORTED FEATURES:
      - Row ADC Feature is not supported (not implemented on STM322xg-EVAL board)
  ----------------------------------------------------------------------------*/

/* Includes ------------------------------------------------------------------*/
#include "stm322xg_eval_bmp085_cpal.h"
#include "printf.h"
/* Private typedef -----------------------------------------------------------*/
/* Private defines -----------------------------------------------------------*/

#define TIMEOUT_MAX              0x3000 /*<! The value of the maximal timeout for I2C waiting loops */


/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/

uint32_t TimeOut = TIMEOUT_MAX; /*<! Value of Timeout when I2C communication fails */

CPAL_TransferTypeDef  BMP085_RXTransfer = {
                        /* Initialize TX Transfer structure */
                        pNULL,
                        0,
                        0,
                        0};

CPAL_TransferTypeDef  BMP085_TXTransfer = {
                        /* Initialize RX Transfer structure */
                        pNULL,
                        0,
                        0,
                        0};

uint8_t BMP085_Buffer[2] = {0x00,0x00};

const uint8_t BMP_PRESSURE_OSRS_VAL[]={0x34, 0x74, 0xB4, 0xF4};
uint8_t bmp085_osrs=0;

extern CPAL_InitTypeDef BMP085_DevStructure;

/* Private function prototypes -----------------------------------------------*/
static void BMP085_StructInit(void);
static void BMP085_INT_GPIO_Config(void);
static void BMP085_EXTI_Config(void);

#ifndef USE_Delay
static void delay(__IO uint32_t nCount);
#endif /* USE_Delay*/


/* Private functions ---------------------------------------------------------*/
int16_t AC1, AC2, AC3, B1, B2, MB, MC, MD;
uint16_t AC4, AC5, AC6;

static void ReadCalibration(void)
{
	printf(">>>ReadCalibration\r\n");
	/* read calibration data */
	AC1 = I2C_ReadDataBuffer(BMP085_ADDR,BMP085_CAL_AC1);
	printf("AC1=%d\r\n",AC1);
	AC2 = I2C_ReadDataBuffer(BMP085_ADDR,BMP085_CAL_AC2);
	printf("AC2=%d\r\n",AC2);
	AC3 = I2C_ReadDataBuffer(BMP085_ADDR,BMP085_CAL_AC3);
	printf("AC3=%d\r\n",AC3);
	AC4 = I2C_ReadDataBuffer(BMP085_ADDR,BMP085_CAL_AC4);
	printf("AC4=%u\r\n",AC4);
	AC5 = I2C_ReadDataBuffer(BMP085_ADDR,BMP085_CAL_AC5);
	printf("AC5=%u\r\n",AC5);
	AC6 = I2C_ReadDataBuffer(BMP085_ADDR,BMP085_CAL_AC6);
	printf("AC6=%u\r\n",AC6);
	B1 = I2C_ReadDataBuffer(BMP085_ADDR,BMP085_CAL_B1);
	printf("B1=0x%x, %d\r\n",B1, B1);
	B2 = I2C_ReadDataBuffer(BMP085_ADDR,BMP085_CAL_B2);
	printf("B2=0x%x, %d\r\n",B2, B2);

	MB = I2C_ReadDataBuffer(BMP085_ADDR,BMP085_CAL_MB);
	printf("MB=%d\r\n",MB);
	MC = I2C_ReadDataBuffer(BMP085_ADDR,BMP085_CAL_MC);
	printf("MC=%d\r\n",MC);
	MD = I2C_ReadDataBuffer(BMP085_ADDR,BMP085_CAL_MD);
	printf("MD=%d\r\n",MD);
	printf("<<<ReadCalibration\r\n");
}


/**
  * @brief  Returns the temperature value (in 16 bit format).
  * @param  None
  * @retval The temperature value.
  */
uint16_t BMP085_GetRawTemperature(void)
{
 	uint16_t tmp = 0;
	printf(">>>BMP085_GetRawTemperature\r\n");
  	/* temperature operation enable */
  	I2C_WriteDeviceRegister(BMP085_ADDR, BMP085_CONTROL, BMP085_READTEMPCMD);
	delay(100);

  	tmp = (uint16_t)I2C_ReadDataBuffer(BMP085_ADDR, BMP085_TEMPDATA);
	printf("<<<BMP085_GetRawTemperature = %d \r\n", tmp);
  	/* return the temperature row value */
  	return tmp;
}

/**
  * @brief  Returns the pressure value (in 16 bit format).
  * @param  None
  * @retval The pressure value.
  */
uint32_t BMP085_GetRawPressure(void)
{
 	uint32_t pressure = 0;
	printf(">>>BMP085_GetRawPressure\r\n");
  	/* pressure operation enable */
  	I2C_WriteDeviceRegister(BMP085_ADDR, BMP085_CONTROL, BMP_PRESSURE_OSRS_VAL[bmp085_osrs]);
	delay(10);
	/* 2 bytes */
  	pressure = (uint32_t)I2C_ReadDataBuffer(BMP085_ADDR, BMP085_PRESSUREDATA);
	printf("<<<BMP085_GetRawPressure = 0x%x\r\n", pressure);
  	/* return the temperature row value */
  	return pressure;
}

/*
 * in 0.1C
 *
 */
uint16_t BMP085_CalTemperature(void)
{
	int32_t X1, X2, B5;
	uint16_t ut = BMP085_GetRawTemperature();
	int32_t temperature=0;

	X1 = ((int32_t)ut - AC6) * AC5 >> 15;
	X2 = ((int32_t)MC << 11) / (X1 + MD);
	B5 = X1 + X2;
	temperature = (B5 + 8) >> 4;	/* in 0.1 C unit */
	printf("BMP085_CalTemperature = %ld/10 C\r\n", temperature);
	return temperature;
}

/*
 * pressure in Pascal (Pa)
 * 1 Pa	กÝ 1 Newton/m^2
 * 1 hPa = 100Pa
 * 1 bar = 10^5 Pa = 1000hPa
 * 1 atm = 1.01325 กั10^5 Pa = 1013.325 hPa
 */
int32_t BMP085_CalPressure(void)
{
	int32_t pressure;
	int32_t temperature;
	int32_t X1, X2, B5, B6, X3, B3, p;
	uint32_t B4, B7;
	int32_t ut = BMP085_GetRawTemperature();
	int32_t up = BMP085_GetRawPressure();

	printf(">>>BMP085_CalPressure ut=%d,up=%d\r\n", ut, up);
	X1 = ((int32_t)ut - AC6) * AC5 >> 15;
	X2 = ((int32_t)MC << 11) / (X1 + MD);
	B5 = X1 + X2;
	temperature = (B5 + 8) >> 4;	/* in 0.1 C unit */

	B6 = B5 - 4000;
	X1 = (B2 * (B6 * B6 >> 12)) >> 11;
	X2 = AC2 * B6 >> 11;
	X3 = X1 + X2;
	B3 = (((int32_t)AC1 * 4 + X3) + 2)/4;
	X1 = AC3 * B6 >> 13;
	X2 = (B1 * (B6 * B6 >> 12)) >> 16;
	X3 = ((X1 + X2) + 2) >> 2;
	B4 = (AC4 * (uint32_t) (X3 + 32768)) >> 15;
	B7 = ((uint32_t) up - B3) * (50000 >> bmp085_osrs);
	if( B7 < 0x80000000)
	    p = (B7 * 2) / B4 ;
	else
		p = (B7 / B4) * 2;

	X1 = (p >> 8) * (p >> 8);
	X1 = (X1 * 3038) >> 16;
	X2 = (-7357 * p) >> 16;
	pressure = p + ((X1 + X2 + 3791) >> 4);

	printf("<<<BMP085_CalPressure %ld Pa\r\n", pressure);
	return pressure;
}

/**
  * @brief  Resets the IO Expander by Software (SYS_CTRL1, RESET bit).
  * @param  DeviceAddr: The address of the IOExpander, could be : BMP085_ADDR
  *         or BMP085_ADDR.
  * @retval BMP085_OK: if all initializations are OK. Other value if error.
  */
uint8_t BMP085_Reset(uint8_t DeviceAddr)
{


  /* If all OK return BMP085_OK */
  return BMP085_OK;
}

/**
  * @brief  Writes a value in a register of the device through I2C.
  * @param  DeviceAddr: The address of the IOExpander, could be : BMP085_ADDR
  *         or BMP085_ADDR.
  * @param  RegisterAddr: The target register address
  * @param  RegisterValue: The target register value to be written
  * @retval BMP085_OK: if all operations are OK. Other value if error.
  */
uint8_t I2C_WriteDeviceRegister(uint8_t DeviceAddr, uint8_t RegisterAddr, uint8_t RegisterValue)
{
//  uint32_t read_verif = 0;

	//printf(">>>I2C_WriteDeviceRegister(0x%x,0x%x,0x%x)\r\n",DeviceAddr, RegisterAddr,RegisterValue);

  	BMP085_Buffer[0] = RegisterValue;

  	/* Disable all options */
  	BMP085_DevStructure.wCPAL_Options  = 0;

  	/* point to CPAL_TransferTypeDef structure */
  	BMP085_DevStructure.pCPAL_TransferTx = &BMP085_TXTransfer;

  	/* Configure transfer parameters */
  	BMP085_DevStructure.pCPAL_TransferTx->wNumData = 1;
  	BMP085_DevStructure.pCPAL_TransferTx->pbBuffer = BMP085_Buffer ;
  	BMP085_DevStructure.pCPAL_TransferTx->wAddr1   = (uint32_t)DeviceAddr;
  	BMP085_DevStructure.pCPAL_TransferTx->wAddr2   = (uint32_t)RegisterAddr;

  	/* Write Operation */
  	if (CPAL_I2C_Write(&BMP085_DevStructure)== CPAL_PASS)
  	{
 		//printf("CPAL_State=0%x\r\n", BMP085_DevStructure.CPAL_State);
  	  	while ((BMP085_DevStructure.CPAL_State != CPAL_STATE_READY) && (BMP085_DevStructure.CPAL_State != CPAL_STATE_ERROR) )
  	  	{ }
  	}

#ifdef VERIFY_WRITTENDATA
asdf
  /* Verify (if needed) that the loaded data is correct  */

  /* Read the just written register*/
  read_verif = I2C_ReadDeviceRegister(DeviceAddr, RegisterAddr);
  /* Load the register and verify its value  */
  if (read_verif != RegisterValue)
  {
    /* Control data wrongly transferred */
    read_verif = BMP085_FAILURE;
  }
  else
  {
    /* Control data correctly transferred */
    read_verif = 0;
  }
#endif

  	/* Return the verifying value: 0 (Passed) or 1 (Failed) */
	//printf("<<<I2C_WriteDeviceRegister\r\n");
 	return 0;
}

/**
  * @brief  Reads a register of the device through I2C.
  * @param  DeviceAddr: The address of the device, could be : BMP085_ADDR
  *         or BMP085_ADDR.
  * @param  RegisterAddr: The target register address (between 00x and 0x24)
  * @retval The value of the read register (0xAA if Timeout occurred)
  */
uint8_t I2C_ReadDeviceRegister(uint8_t DeviceAddr, uint8_t RegisterAddr)
{
  	BMP085_Buffer[0] = 0;

  	/* Disable all options */
  	BMP085_DevStructure.wCPAL_Options  = 0;

  	/* point to CPAL_TransferTypeDef structure */
  	BMP085_DevStructure.pCPAL_TransferRx = &BMP085_RXTransfer;

  	/* Configure transfer parameters */
  	BMP085_DevStructure.pCPAL_TransferRx->wNumData = 1;
  	BMP085_DevStructure.pCPAL_TransferRx->pbBuffer = BMP085_Buffer ;
  	BMP085_DevStructure.pCPAL_TransferRx->wAddr1   = (uint32_t)DeviceAddr;
  	BMP085_DevStructure.pCPAL_TransferRx->wAddr2   = (uint32_t)RegisterAddr;

  	/* Read Operation */
  	if(CPAL_I2C_Read(&BMP085_DevStructure) == CPAL_PASS)
  	{
  	  while ((BMP085_DevStructure.CPAL_State != CPAL_STATE_READY) && (BMP085_DevStructure.CPAL_State != CPAL_STATE_ERROR) )
  	  { }
  	}
  	/* Return the read value */
  	return BMP085_Buffer[0];
}

/**
  * @brief  Reads a buffer of 2 bytes from the device registers.
  * @param  DeviceAddr: The address of the device, could be : BMP085_ADDR
  *         or BMP085_ADDR.
  * @param  RegisterAddr: The target register address (between 00x and 0x24)
  * @retval A pointer to the buffer containing the two returned bytes (in halfword).
  */
uint16_t I2C_ReadDataBuffer(uint8_t DeviceAddr, uint32_t RegisterAddr)
{
  	uint8_t tmp =0 ;

  	BMP085_Buffer[0] = 0;
  	BMP085_Buffer[1] = 0;

  	/* Disable all options */
  	BMP085_DevStructure.wCPAL_Options  = 0; //CPAL_OPT_I2C_NOSTOP;

  	/* point to CPAL_TransferTypeDef structure */
  	BMP085_DevStructure.pCPAL_TransferRx = &BMP085_RXTransfer;

  	/* Configure transfer parameters */
  	BMP085_DevStructure.pCPAL_TransferRx->wNumData = 2;
  	BMP085_DevStructure.pCPAL_TransferRx->pbBuffer = BMP085_Buffer ;
  	BMP085_DevStructure.pCPAL_TransferRx->wAddr1   = (uint32_t)DeviceAddr;
  	BMP085_DevStructure.pCPAL_TransferRx->wAddr2   = (uint32_t)RegisterAddr;

  	/* read Operation */
  	if (CPAL_I2C_Read(&BMP085_DevStructure)== CPAL_PASS)
  	{
  	  while ((BMP085_DevStructure.CPAL_State != CPAL_STATE_READY) && (BMP085_DevStructure.CPAL_State != CPAL_STATE_ERROR) )
  	  { }
  	}

	//printf("I2C_ReadDataBuffer 0x%x 0x%x\r\n", BMP085_Buffer[0], BMP085_Buffer[1]);
  	tmp = BMP085_Buffer[0];
  	BMP085_Buffer[0] = BMP085_Buffer[1];
  	BMP085_Buffer[1] = tmp;

  	/* return buffer value */
  	return *(uint16_t *)BMP085_Buffer;
}

/**
  * @brief  Initialize the CPAL structure used to communicate with IO_Expanders.
  * @param  None
  * @retval None
  */
static void BMP085_StructInit(void)
{
  /* Set CPAL structure parameters to their default values */
  CPAL_I2C_StructInit(&BMP085_DevStructure);

  /* Set I2C clock speed */
  BMP085_DevStructure.pCPAL_I2C_Struct->I2C_ClockSpeed = I2C_SPEED;

#ifdef BMP085_IT
  /* Select Interrupt programming model and disable all options */
  BMP085_DevStructure.CPAL_ProgModel = CPAL_PROGMODEL_INTERRUPT;
  BMP085_DevStructure.wCPAL_Options  = 0;
#else
  /* Select DMA programming model and activate TX_DMA_TC and RX_DMA_TC interrupts */
  BMP085_DevStructure.CPAL_ProgModel = CPAL_PROGMODEL_DMA;
  BMP085_DevStructure.wCPAL_Options  = CPAL_OPT_DMATX_TCIT | CPAL_OPT_DMARX_TCIT ;
#endif /* BMP085_IT */

  /* point to CPAL_TransferTypeDef structure */
  BMP085_DevStructure.pCPAL_TransferTx = &BMP085_TXTransfer;
  BMP085_DevStructure.pCPAL_TransferRx = &BMP085_RXTransfer;
}

/**
  * @brief  Initialize the INT GPIO pin used by the IO expander.
  * @param  None
  * @retval None
  */
static void BMP085_INT_GPIO_Config(void)
{
#if 0
  GPIO_InitTypeDef GPIO_InitStructure;

  /* Enable BMP085_I2C and BMP085_I2C_GPIO_PORT & Alternate Function clocks */
  RCC_AHB1PeriphClockCmd(BMP085_IT_GPIO_CLK, ENABLE);
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_SYSCFG, ENABLE);

  /* Set EXTI pin as Input PullUp - IO_Expander_INT */
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_InitStructure.GPIO_Pin = BMP085_IT_PIN;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN;
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
  GPIO_Init(BMP085_IT_GPIO_PORT, &GPIO_InitStructure);

  /* Connect Button EXTI Line to Button GPIO Pin */
  SYSCFG_EXTILineConfig(BMP085_IT_EXTI_PORT_SOURCE, BMP085_IT_EXTI_PIN_SOURCE);
#endif
}

/**
  * @brief  Configures the IO expander Interrupt line and GPIO in EXTI mode.
  * @param  None
  * @retval None
  */
static void BMP085_EXTI_Config(void)
{
  GPIO_InitTypeDef GPIO_InitStructure;
  NVIC_InitTypeDef NVIC_InitStructure;
  EXTI_InitTypeDef EXTI_InitStructure;

  /* Enable GPIO clock */
  RCC_AHB1PeriphClockCmd(BMP085_IT_GPIO_CLK, ENABLE);
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_SYSCFG, ENABLE);

  /* Configure Button pin as input floating */
  GPIO_InitStructure.GPIO_Pin = BMP085_IT_PIN;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN;
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
  GPIO_Init(BMP085_IT_GPIO_PORT, &GPIO_InitStructure);

  /* Connect Button EXTI Line to Button GPIO Pin */
  SYSCFG_EXTILineConfig(BMP085_IT_EXTI_PORT_SOURCE, BMP085_IT_EXTI_PIN_SOURCE);

  /* Configure Button EXTI line */
  EXTI_InitStructure.EXTI_Line = BMP085_IT_EXTI_LINE;
  EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt;
  EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Rising_Falling;
  EXTI_InitStructure.EXTI_LineCmd = ENABLE;
  EXTI_Init(&EXTI_InitStructure);

  /* Enable and set Button EXTI Interrupt to the lowest priority */
  NVIC_InitStructure.NVIC_IRQChannel = BMP085_IT_EXTI_IRQn;
  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0x0F;
  NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0x0F;
  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
  NVIC_Init(&NVIC_InitStructure);
}

#ifndef USE_Delay
/**
  * @brief  Inserts a delay time.
  * @param  nCount: specifies the delay time length.
  * @retval None
  */
static void delay(__IO uint32_t nCount)
{
  __IO uint32_t index = 0;
  for(index = (100000 * nCount); index != 0; index--)
  {
  }
}
#endif /* USE_Delay*/

/**
  * @brief  Initialize and Configures the two IO_Expanders Functionalities
  *         (IOs, Touch Screen ..) and configures all STM3210C-EVAL necessary
  *         hardware (GPIOs, APB clocks ..).
  * @param  None
  * @retval BMP085_OK if all initializations done correctly. Other value if error.
  */
uint8_t BMP085_Config(void)
{
  	/* Configure the needed pins */
  	BMP085_INT_GPIO_Config();

  	BMP085_StructInit();

  	CPAL_I2C_Init(&BMP085_DevStructure);

	ReadCalibration();
	BMP085_CalTemperature();
	BMP085_CalPressure();
  	/* Configuration is OK */
  	printf("BMP085_Config OK\r\n");
  	return BMP085_OK;
}

/**
  * @brief  Configures The selected interrupts on the IO Expanders.
  * @param  BMP085_ITSRC_Source: the source of the interrupts. Could be one or a
  *         combination of the following parameters:
  *   @arg  BMP085_ITSRC_JOYSTICK: Joystick IO interrupts.
  *   @arg  BMP085_ITSRC_TSC: Touch Screen interrupts.
  *   @arg  BMP085_ITSRC_INMEMS: MEMS interrupt lines.
  * @retval BMP085_OK: if all initializations are OK. Other value if error.
  */
uint8_t BMP085_ITConfig(uint32_t BMP085_ITSRC_Source)
{
  /* Configure the Interrupt line as EXTI source */
  BMP085_EXTI_Config();

  /* If all OK return BMP085_OK */
  return BMP085_OK;
}

/******************* (C) COPYRIGHT 2011 STMicroelectronics *****END OF FILE****/
