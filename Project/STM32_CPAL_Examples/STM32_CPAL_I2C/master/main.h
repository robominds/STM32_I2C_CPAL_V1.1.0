/**
  ******************************************************************************
  * @file    Project/STM32_CPAL_Examples/STM32_CPAL_I2C/Two_Boards/main..h
  * @author  MCD Application Team
  * @version V1.1.0
  * @date    17-June-2011
  * @brief   Header file for main.c.
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


/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H
#define __MAIN_H

/* Includes ------------------------------------------------------------------*/
#include "cpal_i2c.h"

#include "stm32_eval.h"
#include <stdio.h>

#ifdef USE_STM322xG_EVAL
 #include "stm32f2xx.h"
#endif


#ifdef USE_STM322xG_EVAL
 #include "stm322xg_eval_lcd.h"
#endif

/* Exported types ------------------------------------------------------------*/

/* Exported constants --------------------------------------------------------*/

/* This define select used I2Cx device for communication */

#if defined (USE_STM32L152_EVAL) || defined (USE_STM3210C_EVAL) || defined (USE_STM322xG_EVAL)
 #define I2C_DevStructure        I2C1_DevStructure
#endif

#ifdef USE_STM32100E_EVAL
 #define I2C_DevStructure        I2C2_DevStructure
#endif

#define countof(a) (sizeof(a) / sizeof(*(a)))

#define MESSAGE1                (uint8_t*)"---STM32 CPAL Lib---"
#define MESSAGE2                (uint8_t*)"I2C 2xBoards Example"
#define MESSAGE3                (uint8_t*)"-------READY--------"

#define MEASSAGE_EPTY           (uint8_t*)"                    "

#define MAX_BUFF_SIZE           200
#define BUFFER_SIZE             (countof(tStateSignal)-1)

#define OWN_ADDRESS             0x74

/* To use the I2C at 400 KHz (in fast mode), the PCLK1 frequency (I2C peripheral
   input clock) must be a multiple of 10 MHz */
#define I2C_SPEED               300000  /* Speed in Hz */

#define ACTION_NONE             0xFF
#define ACTION_DISABLED         0xFD
#define ACTION_PENDING          0xFE
#define ACTION_PERIODIC         0xFC

#define STATE_OFF               0
#define STATE_ON                1

/* Exported macro ------------------------------------------------------------*/
/* Exported functions ------------------------------------------------------- */
extern uint8_t BMP085_Config(void);

#endif /* __MAIN_H */

/******************* (C) COPYRIGHT 2011 STMicroelectronics *****END OF FILE****/
