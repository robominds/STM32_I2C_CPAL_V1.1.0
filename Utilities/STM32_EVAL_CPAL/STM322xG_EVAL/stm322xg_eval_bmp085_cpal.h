/**
  ******************************************************************************
  * @file    stm322xg_eval_bmp085_cpal.h
  * @author  MCD Application Team
  * @version V1.1.0
  * @date    17-June-2011
  * @brief   This file contains all the functions prototypes for the
  *          stm322xg_eval_bmp085_cpal.c firmware driver.
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
    SUPPORTED FEATURES:
      - IO Read/write : Set/Reset and Read (Polling/Interrupt)
      - Joystick: config and Read (Polling/Interrupt)
      - Touch Screen Features: Single point mode (Polling/Interrupt)
      - TempSensor Feature: accuracy not determined (Polling).

    UNSUPPORTED FEATURES:
      - Row ADC Feature is not supported (not implemented on STM322xG_EVAL board)
  ----------------------------------------------------------------------------*/


/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __STM322XG_EVAL_BMP085_CPAL_H
#define __STM322XG_EVAL_BMP085_CPAL_H

#ifdef __cplusplus
 extern "C" {
#endif

/*==========================================================================================================
                                             User NOTES
============================================================================================================

---------------------------------
   How To use the IOE Driver:
---------------------------------
-----  Configuration : Call BMP085_Config() function to Configure IOExpander. This
                       function return BMP085_OK if configuration is complete successfully.

-----  Using IOExpander in polling mode :

               *- Joystick : User call BMP085_JoyStickGetState() function and get the
                             returned value.
                             Value returned by this function can be one of these :
                               *  JOY_NONE  : no joystick is pushed.
                               *  JOY_SEL   : Center joystick is pushed.
                               *  JOY_DOWN  : Down joystick is pushed.
                               *  JOY_LEFT  : Left joystick is pushed.
                               *  JOY_RIGHT : Right joystick is pushed.
                               *  JOY_UP    : Up joystick is pushed.

               *- TouchScreen : User call BMP085_JoyStickGetState() function that return
                                a pointer to TS_State structure that contains these parameters:
                                  * TouchDetected : this value is nonzero if TouchScreen
                                    is touched.
                                  *  X : contain x position of touched point.
                                  *  Y : contain y position of touched point.
                                  *  Z : contain z position of touched point.

               *- Temperature Sensor : User call BMP085_TempSens_GetData() function to get
                                       Temperature value.

-----  Using IOExpander in interrupt mode : In InterruptHandler related to EXTI connected
                                            to IOExpander interrupt, User get the source of
                                            IOExpander interrupt by calling BMP085_GetGITStatus() function.
                                            Then depending on the source of interrupt user proceed
                                            as follow to use :

               *- Joystick : User call BMP085_JoyStickGetState() function and get the
                             returned value.

               *- TouchScreen : User call BMP085_JoyStickGetState() function that return
                                a pointer to TS_State structure.

               *- Temperature Sensor : User call BMP085_TempSens_GetData() function to get
                                       Temperature value.

               --- IMPORTANT NOTE ---: After each interrupt generated by IOExpander
                                       BMP085_ClearGITPending() and BMP085_ClearIOITPending()
                                       (when using joystick) must be called to clear
                                       pending IOExpander interrupts.
                                       If these are not called IOExpander will not respond for
                                       any excitation (pushing joystick or touching TouchScreen).

*********END OF User Notes**********************************************************************************/

/* Includes ------------------------------------------------------------------*/
#include "cpal_i2c.h"

/* Exported types ------------------------------------------------------------*/

/**
  * @brief  Touch Screen Information structure
  */
typedef struct
{
  uint16_t TouchDetected;
  uint16_t X;
  uint16_t Y;
  uint16_t Z;
}TS_STATE;

/**
  * @brief  Joystick State definitions
  */
#ifndef __STM32_EVAL_H
typedef enum
{
  JOY_NONE = 0,
  JOY_SEL = 1,
  JOY_DOWN = 2,
  JOY_LEFT = 3,
  JOY_RIGHT = 4,
  JOY_UP = 5
} JOYState_TypeDef
;
#endif /* __STM32_EVAL_H */

/**
  * @brief  IO_Expander Error codes
  */
typedef enum
{
  BMP085_OK = 0,
  BMP085_FAILURE,
  BMP085_TIMEOUT,
  PARAM_ERROR,
  IOE1_NOT_OPERATIONAL,
  IOE2_NOT_OPERATIONAL
}BMP085_Status_TypDef;

/**
  * @brief  IO bit values
  */
typedef enum
{
  BitReset = 0,
  BitSet = 1
}BMP085_BitValue_TypeDef;



/* Exported constants --------------------------------------------------------*/

/**
 * @brief Uncomment the line below to enable verifying each written byte in write
 *        operation. The I2C_WriteDeviceRegister() function will then compare the
 *        written and read data and return error status if a mismatch occurs.
 */
/* #define VERIFY_WRITTENDATA */

/**
 * @brief Uncomment the line below if you want to use user defined Delay function
 *        (for precise timing), otherwise default _delay_ function defined within
 *         this driver is used (less precise timing).
 */
/* #define USE_Delay */


#ifdef USE_Delay
#include "main.h"

  #define _delay_     Delay  /* !< User can provide more timing precise _delay_ function
                                   (with 10ms time base), using SysTick for example */
#else
  #define _delay_     delay      /* !< Default _delay_ function with less precise timing */
#endif

/*------------------------------------------------------------------------------
    Configuration
------------------------------------------------------------------------------*/

/**
  * @brief  CPAL Structure configuration
  */

/* Select I2C device (uncomment relative define) */
#define BMP085_DevStructure                I2C1_DevStructure
//#define BMP085_DevStructure                I2C2_DevStructure
//#define BMP085_DevStructure                I2C3_DevStructure


/* Select clock Speed */
/* To use the I2C at 400 KHz (in fast mode), the PCLK1 frequency (I2C peripheral
   input clock) must be a multiple of 10 MHz */

#define I2C_SPEED                        100000

/* Select interrupt programming model : By default DMA programming model is selected.
 To select interrupt programming model uncomment this define */
//#define BMP085_IT

/**
  * @brief  IO Expander Interrupt line on EXTI
  */
#define BMP085_IT_PIN                       GPIO_Pin_2
#define BMP085_IT_GPIO_PORT                 GPIOI
#define BMP085_IT_GPIO_CLK                  RCC_AHB1Periph_GPIOI
#define BMP085_IT_EXTI_PORT_SOURCE          EXTI_PortSourceGPIOI
#define BMP085_IT_EXTI_PIN_SOURCE           EXTI_PinSource2
#define BMP085_IT_EXTI_LINE                 EXTI_Line2
#define BMP085_IT_EXTI_IRQn                 EXTI2_IRQn

/**
  * @brief  The BOSCH bmp085 air pressure
  */
#define BMP085_ADDR              (0xEE)

#define BMP085_ULTRALOWPOWER 	(0)
#define BMP085_STANDARD      	(1)
#define BMP085_HIGHRES       	(2)
#define BMP085_ULTRAHIGHRES  	(3)
#define BMP085_CAL_AC1          (0xAA)  // R   Calibration data (16 bits)
#define BMP085_CAL_AC2          (0xAC)  // R   Calibration data (16 bits)
#define BMP085_CAL_AC3          (0xAE)  // R   Calibration data (16 bits)
#define BMP085_CAL_AC4          (0xB0)  // R   Calibration data (16 bits)
#define BMP085_CAL_AC5          (0xB2)  // R   Calibration data (16 bits)
#define BMP085_CAL_AC6          (0xB4)  // R   Calibration data (16 bits)
#define BMP085_CAL_B1           (0xB6)  // R   Calibration data (16 bits)
#define BMP085_CAL_B2           (0xB8)  // R   Calibration data (16 bits)
#define BMP085_CAL_MB           (0xBA)  // R   Calibration data (16 bits)
#define BMP085_CAL_MC           (0xBC)  // R   Calibration data (16 bits)
#define BMP085_CAL_MD           (0xBE)  // R   Calibration data (16 bits)

#define BMP085_CONTROL          (0xF4)
#define BMP085_READTEMPCMD      (0x2E)
#define BMP085_READPRESSURECMD  (0x34)	/* osrs=0 */

#define BMP085_TEMPDATA			(0xF6)
#define BMP085_PRESSUREDATA     (0xF6)



/*------------------------------------------------------------------------------
    Functional and Interrupt Management
------------------------------------------------------------------------------*/
/**
  * @brief  IO Expander Functionalities definitions
  */
#define BMP085_ADC_FCT              0x01
#define BMP085_TS_FCT               0x02
#define BMP085_IO_FCT               0x04
#define BMP085_TEMPSENS_FCT         0x08

/**
  * @brief  Global Interrupts definitions
  */
#define BMP085_GIT_GPIO             0x80
#define BMP085_GIT_ADC              0x40
#define BMP085_GIT_TEMP             0x20
#define BMP085_GIT_FE               0x10
#define BMP085_GIT_FF               0x08
#define BMP085_GIT_FOV              0x04
#define BMP085_GIT_FTH              0x02
#define BMP085_GIT_TOUCH            0x01


/*------------------------------------------------------------------------------
    STMPE811 device register definition
------------------------------------------------------------------------------*/
/**
  * @brief  Identification registers
  */
#define BMP085_REG_CHP_ID             0x00
#define BMP085_REG_ID_VER             0x02

/*------------------------------------------------------------------------------
    Functions parameters defines
------------------------------------------------------------------------------*/
/**
  * @brief  Global interrupt Enable bit
  */
#define BMP085_GIT_EN                0x01

/* Exported macro ------------------------------------------------------------*/
/* Exported functions ------------------------------------------------------- */

/**
  * @brief  Configuration and initialization functions
  */
uint8_t BMP085_Config(void);
uint8_t BMP085_ITConfig(uint32_t BMP085_ITSRC_Source);


/**
  * @brief Interrupts Management functions
  */
FlagStatus BMP085_GetGITStatus(uint8_t DeviceAddr, uint8_t Global_IT);
uint8_t BMP085_ClearGITPending(uint8_t DeviceAddr, uint8_t IO_IT);
FlagStatus BMP085_GetIOITStatus(uint8_t DeviceAddr, uint8_t IO_IT);
uint8_t BMP085_ClearIOITPending(uint8_t DeviceAddr, uint8_t IO_IT);

/**
  * @brief IO-Expander Control functions
  */
uint8_t BMP085_IsOperational(uint8_t DeviceAddr);
uint8_t BMP085_Reset(uint8_t DeviceAddr);


uint8_t BMP085_FnctCmd(uint8_t DeviceAddr, uint8_t Fct, FunctionalState NewState);
uint8_t BMP085_IOPinConfig(uint8_t DeviceAddr, uint8_t IO_Pin, uint8_t Direction);
uint8_t BMP085_GITCmd(uint8_t DeviceAddr, FunctionalState NewState);
uint8_t BMP085_GITConfig(uint8_t DeviceAddr, uint8_t Global_IT, FunctionalState NewState);
uint8_t BMP085_IOITConfig(uint8_t DeviceAddr, uint8_t IO_IT, FunctionalState NewState);

uint8_t I2C_WriteDeviceRegister(uint8_t DeviceAddr, uint8_t RegisterAddr, uint8_t RegisterValue);
uint8_t I2C_ReadDeviceRegister(uint8_t DeviceAddr, uint8_t RegisterAddr);
uint16_t I2C_ReadDataBuffer(uint8_t DeviceAddr, uint32_t RegisterAddr);

#define DUG_PRINTF   printf

#ifndef DUG_PRINTF
  #define DUG_PRINTF   printf
#endif


#ifdef __cplusplus
}
#endif
#endif /* __STM322XG_EVAL_BMP085_CPAL_H */


/******************* (C) COPYRIGHT 2011 STMicroelectronics *****END OF FILE****/
