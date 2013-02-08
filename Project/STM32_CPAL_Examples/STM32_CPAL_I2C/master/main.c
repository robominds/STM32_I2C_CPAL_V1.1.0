/**
  ******************************************************************************
  * @file    Project/STM32_CPAL_Examples/STM32_CPAL_I2C/Two_Boards/main.c
  * @author  MCD Application Team
  * @version V1.1.0
  * @date    17-June-2011
  * @brief   Main program body
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

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "printf.h"
/* Private typedef -----------------------------------------------------------*/
typedef enum {FAILED = 0, PASSED = !FAILED} TestStatus;
/* Private define ------------------------------------------------------------*/

/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/

/* CPAL local transfer structures */
CPAL_TransferTypeDef  sRxStructure, sTxStructure;

/* Buffers tables declarations */
const uint8_t tStateSignal[]  = "STM32 CPAL 2xBoards Example: Signal State  ";
const uint8_t tSignal1[]      = "STM32 CPAL 2xBoards Example: Signal Signal1";
const uint8_t tSignal2[]      = "STM32 CPAL 2xBoards Example: Signal Signal2";

uint8_t tRxBuffer[MAX_BUFF_SIZE];
uint32_t BufferSize = BUFFER_SIZE;
uint8_t Color = 0;
__IO uint32_t ActionState = ACTION_NONE;
__IO uint32_t RecieverMode = 0;
__IO uint32_t TransmitMode = 0;
uint32_t PeriodicValue = 15000, Divider = 1;

/* Private function prototypes -----------------------------------------------*/

/* Private functions ---------------------------------------------------------*/
static void TIM6_Config(void);
static void TIM7_Config(uint32_t Period);
uint8_t Buffer_Check(uint8_t* pBuffer, uint8_t* pBuffer1, uint8_t* pBuffer2,  uint8_t* pBuffer3, uint16_t BufferLength);
TestStatus Buffer_Compare(uint8_t* pBuffer1, uint8_t* pBuffer2, uint16_t BufferLength);

/**
  * @brief  Main program.
  * @param  None
  * @retval None
  */
int main(void)
{
  /*!< At this stage the microcontroller clock setting is already configured,
       this is done through SystemInit() function which is called from startup
       file (startup_stm32xxx_xx.s) before to branch to application main.
       To reconfigure the default setting of SystemInit() function, refer to
       system_stm32xxx.c file
  */
	Printf_Init();

  	/* Output a message on Hyperterminal using printf function */
  	printf("\n\rUSART Printf Example: retarget the C library printf function to the USART\n\r");

   /* Initialize TIM6 */
  TIM6_Config();
#if 0
  /* Start CPAL communication configuration ***********************************/
  /* Initialize local Reception structures */
  sRxStructure.wNumData = BufferSize;       /* Maximum Number of data to be received */
  sRxStructure.pbBuffer = tRxBuffer;        /* Common Rx buffer for all received data */
  sRxStructure.wAddr1 = 0;                  /* Not needed */
  sRxStructure.wAddr2 = 0;                  /* Not needed */

  /* Initialize local Transmission structures */
  sTxStructure.wNumData = BufferSize;       /* Maximum Number of data to be received */
  sTxStructure.pbBuffer = (uint8_t*)tStateSignal;     /* Common Rx buffer for all received data */
  sTxStructure.wAddr1 = OWN_ADDRESS;        /* The own board address */
  sTxStructure.wAddr2 = 0;                  /* Not needed */

  /* Configure the device structure */
  CPAL_I2C_StructInit(&I2C_DevStructure);      /* Set all fields to default values */
  I2C_DevStructure.CPAL_Mode = CPAL_MODE_SLAVE;
#ifdef CPAL_I2C_DMA_PROGMODEL
  I2C_DevStructure.wCPAL_Options =  CPAL_OPT_NO_MEM_ADDR | CPAL_OPT_DMATX_TCIT | CPAL_OPT_DMARX_TCIT;
  I2C_DevStructure.CPAL_ProgModel = CPAL_PROGMODEL_DMA;
#elif defined (CPAL_I2C_IT_PROGMODEL)
  I2C_DevStructure.wCPAL_Options =  CPAL_OPT_NO_MEM_ADDR;
  I2C_DevStructure.CPAL_ProgModel = CPAL_PROGMODEL_INTERRUPT;
#else
 #error "Please select one of the programming model (in main.h)"
#endif
  I2C_DevStructure.pCPAL_I2C_Struct->I2C_ClockSpeed = I2C_SPEED;
  I2C_DevStructure.pCPAL_I2C_Struct->I2C_OwnAddress1 = OWN_ADDRESS;
  I2C_DevStructure.pCPAL_TransferRx = &sRxStructure;
  I2C_DevStructure.pCPAL_TransferTx = &sTxStructure;

  /* Initialize CPAL device with the selected parameters */
  CPAL_I2C_Init(&I2C_DevStructure);
#endif

	BMP085_Config();
  /* Infinite loop */
  while (1)
  {

  }
}


/**
  * @brief  Configures TIM6 and associated resources to generate an update interrupt each 50 ms.
  * @param  None
  * @retval None
  */
static void TIM6_Config(void)
{
  /* TIM6 is used to toggle Led1 and Led4 each 50ms */

  NVIC_InitTypeDef NVIC_InitStructure;
  TIM_TimeBaseInitTypeDef   TIM_TimeBaseStructure;

  /* TIMER clock enable */
  RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM6 , ENABLE);

  /* Enable the TIMER global Interrupt */
  NVIC_InitStructure.NVIC_IRQChannel = TIM6_DAC_IRQn;

  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
  NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
  NVIC_Init(&NVIC_InitStructure);

  TIM_TimeBaseStructure.TIM_Period  = 50000;
#ifdef USE_STM322xG_EVAL
  TIM_TimeBaseStructure.TIM_Prescaler = (SystemCoreClock/2000000)-1;
#endif /* USE_STM322xG_EVAL */
  TIM_TimeBaseStructure.TIM_ClockDivision = TIM_CKD_DIV1;
  TIM_TimeBaseStructure.TIM_CounterMode =  TIM_CounterMode_Up;
  TIM_TimeBaseInit(TIM6, &TIM_TimeBaseStructure);

  /* TIM IT enable */
  TIM_ITConfig(TIM6, TIM_IT_Update , ENABLE);

  /* TIM7 enable counter */
  TIM_Cmd(TIM6, ENABLE);
}

/**
  * @brief  Configures TIM7 and associated resources to generate an update interrupt.
  *         The period of Timer is depending on Period value.
  * @param  Period : affected to Timer Period
  * @retval None
  */
static void TIM7_Config(uint32_t Period)
{
  /* TIM7 is used to generate periodic interrupts. At each interrupt
  if Transmitter mode is selected, a status message is sent to other Board */

  NVIC_InitTypeDef NVIC_InitStructure;
  TIM_TimeBaseInitTypeDef   TIM_TimeBaseStructure;

  /* TIMER clock enable */
  RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM7 , ENABLE);

  NVIC_InitStructure.NVIC_IRQChannel = TIM7_IRQn;
  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0x0F;
  NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0x0F;
  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
  NVIC_Init(&NVIC_InitStructure);

  TIM_TimeBaseStructure.TIM_Period  = Period;
#ifdef USE_STM322xG_EVAL
  TIM_TimeBaseStructure.TIM_Prescaler = (SystemCoreClock/20000)-1;
#endif /* USE_STM322xG_EVAL */
  TIM_TimeBaseStructure.TIM_ClockDivision = TIM_CKD_DIV1;
  TIM_TimeBaseStructure.TIM_CounterMode =  TIM_CounterMode_Up;
  TIM_TimeBaseInit(TIM7, &TIM_TimeBaseStructure);

  /* TIM IT enable */
  TIM_ITConfig(TIM7, TIM_IT_Update , ENABLE);

  /* TIM7 enable counter */
  TIM_Cmd(TIM7, ENABLE);
}

#ifdef  USE_FULL_ASSERT

/**
  * @brief  Reports the name of the source file and the source line number
  *   where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t* file, uint32_t line)
{
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */

  /* Infinite loop */
  while (1)
  {
  }
}
#endif

/**
  * @}
  */


/******************* (C) COPYRIGHT 2011 STMicroelectronics *****END OF FILE****/
