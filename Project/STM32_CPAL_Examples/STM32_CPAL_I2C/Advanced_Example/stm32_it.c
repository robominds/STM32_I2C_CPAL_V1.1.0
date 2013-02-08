/**
  ******************************************************************************
  * @file    Project/STM32_CPAL_Examples/STM32_CPAL_I2C/Advanced_Example/stm32_it.c 
  * @author  MCD Application Team
  * @version V1.1.0
  * @date    17-June-2011
  * @brief   Main Interrupt Service Routines.
  *          This file provides template for all exceptions handler and 
  *          peripherals interrupt service routine.
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
#include "stm32_it.h"
#include "main.h"

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
extern __IO uint32_t ExitMutex;
extern __IO uint32_t GetMutex;
extern __IO uint32_t ShowMutex;
extern __IO Operation_Typedef ReqOperation;
extern __IO uint32_t NextRep;
extern __IO uint32_t SaveReportCmd;
extern __IO uint32_t DisplayTimeout;
extern __IO uint32_t ReadKey;
extern __IO uint32_t PressedKey;

/* Private function prototypes -----------------------------------------------*/
/* Private functions ---------------------------------------------------------*/

/******************************************************************************/
/*            Cortex-M3 Processor Exceptions Handlers                         */
/******************************************************************************/

/**
  * @brief   This function handles NMI exception.
  * @param  None
  * @retval None
  */
void NMI_Handler(void)
{
}

/**
  * @brief  This function handles Hard Fault exception.
  * @param  None
  * @retval None
  */
void HardFault_Handler(void)
{
  /* Go to infinite loop when Hard Fault exception occurs */
  while (1)
  {
  }
}

/**
  * @brief  This function handles Memory Manage exception.
  * @param  None
  * @retval None
  */
void MemManage_Handler(void)
{
  /* Go to infinite loop when Memory Manage exception occurs */
  while (1)
  {
  }
}

/**
  * @brief  This function handles Bus Fault exception.
  * @param  None
  * @retval None
  */
void BusFault_Handler(void)
{
  /* Go to infinite loop when Bus Fault exception occurs */
  while (1)
  {
  }
}

/**
  * @brief  This function handles Usage Fault exception.
  * @param  None
  * @retval None
  */
void UsageFault_Handler(void)
{
  /* Go to infinite loop when Usage Fault exception occurs */
  while (1)
  {
  }
}

/**
  * @brief  This function handles SVCall exception.
  * @param  None
  * @retval None
  */
void SVC_Handler(void)
{
}

/**
  * @brief  This function handles Debug Monitor exception.
  * @param  None
  * @retval None
  */
void DebugMon_Handler(void)
{
}

/**
  * @brief  This function handles PendSVC exception.
  * @param  None
  * @retval None
  */
void PendSV_Handler(void)
{
}

/******************************************************************************/
/*                 STM32xxxx Peripherals Interrupt Handlers                   */
/*  Add here the Interrupt Handler for the used peripheral(s) (PPP), for the  */
/*  available peripheral interrupt handlers name please refer to the startup */
/*  file (startup_stm32xxxx_xx.s).                                            */
/******************************************************************************/

#ifdef USE_STM322xG_EVAL
/**
  * @brief  This function handles External line 15 interrupt request.
  * @param  None
  * @retval None
  */
void EXTI15_10_IRQHandler(void)
{  
  if(EXTI_GetITStatus(KEY_BUTTON_EXTI_LINE) != RESET)
  {  
    if (ReadKey == 1)
    {
      PressedKey =  KEY;     
    }
    else
    {
      ExitMutex = 1;    
    }
    
    /* Clear EXTI interrupts pending bits */
    EXTI_ClearITPendingBit(KEY_BUTTON_EXTI_LINE);
  }  
}

/**
  * @brief  This function handles External line 2 interrupt request.
  * @param  None
  * @retval None
  */
void EXTI2_IRQHandler(void)
{        
  if(EXTI_GetITStatus(IOE_IT_EXTI_LINE) != RESET)
  {         
    static JOY_State_TypeDef JoyState = JOY_NONE;
    static TS_STATE* TS_State;
    
    /* Check if the interrupt source is the Touch Screen */
    if (IOE_GetGITStatus(IOE_1_ADDR, IOE_TS_IT) & IOE_TS_IT)
    { 
      /* Update the structure with the current position */
      TS_State = IOE_TS_GetState();  
      
      if ((TS_State->TouchDetected) && (TS_State->Y < 230) && (TS_State->Y > 200))
      {
        if ((TS_State->X > 10) && (TS_State->X < 70))
        {
          LCD_DisplayStringLine(Line7, (uint8_t*)" TS1                ");
        }
        else if ((TS_State->X > 90) && (TS_State->X < 150))
        {
          LCD_DisplayStringLine(Line7, (uint8_t*)"      TS2           ");
        }
        else if ((TS_State->X > 170) && (TS_State->X < 230))
        {
          LCD_DisplayStringLine(Line7, (uint8_t*)"           TS3      ");
        }     
        else if ((TS_State->X > 250) && (TS_State->X < 310))
        {
          LCD_DisplayStringLine(Line7, (uint8_t*)"                TS4 ");
        }          
      }
      else
      {
        STM_EVAL_LEDOff(LED1);
        STM_EVAL_LEDOff(LED2);
        STM_EVAL_LEDOff(LED3);
        STM_EVAL_LEDOff(LED4);
      }    	 
      
      /* Clear the interrupt pending bits */    
      IOE_ClearGITPending(IOE_1_ADDR, IOE_TS_IT); 
    }    
    else if (IOE_GetGITStatus(IOE_2_ADDR, IOE_GIT_GPIO))
    {       
      /* Get the Joystick State */
      JoyState = IOE_JoyStickGetState();
      
      switch (JoyState)
      {
      case JOY_NONE:
        break;
        
      case JOY_UP:
        if (ReadKey == 1)
        {
          PressedKey =  UP;     
        }
        else
        {
          if (GetMutex == 0)
          {
            ReqOperation = GET_REPORT_OP;
          }     
        }
        break;     
        
      case JOY_DOWN:
        if (ReadKey == 1)
        {
          PressedKey =  DOWN;     
        }
        else
        {
          if (GetMutex == 0)
          {
            ReqOperation = ERASE_EEPROM_OP;
          }
        }
        break;       
        
      case JOY_RIGHT :
        if (ReadKey == 1)
        {
          PressedKey =  RIGHT;     
        }
        else
        {        
          if (GetMutex == 1)
          {
            ShowMutex = 0;
          }
          else
          {
            ReqOperation = SHOW_TEMPERATURE_OP ;
          }
        }
        break;     
        
      case JOY_LEFT:
        if (ReadKey == 1)
        {
          PressedKey =  LEFT;     
        }
        else
        {        
          if (GetMutex == 1)
          {
            ShowMutex = 0;
          }
          else
          {
            ReqOperation = SHOW_TIME_OP;
          }
        }
        break;       
        
      case JOY_CENTER:
        if (ReadKey == 1)
        {
          PressedKey =  SEL;     
        }
        else
        {
          if (GetMutex == 1)
          {
            NextRep = 1;
            ExitMutex = 1; 
          }
        }
        break; 
        
      default:
        LCD_DisplayStringLine(Line9, (uint8_t*)"     JOY  ERROR     ");
        break;           
      }
      
      /* Clear the interrupt pending bits */    
      IOE_ClearGITPending(IOE_2_ADDR, IOE_GIT_GPIO);
      IOE_ClearIOITPending(IOE_2_ADDR, IOE_JOY_IT);  
    }   
    else
    {
      IOE_ClearGITPending(IOE_1_ADDR, ALL_IT);
      IOE_ClearGITPending(IOE_2_ADDR, ALL_IT);  
    }    
  }
  /* Clear EXTI interrupts pending bits */
  EXTI_ClearITPendingBit(IOE_IT_EXTI_LINE); 
}

#else

/**
  * @brief  This function handles External lines 9 to 5 interrupt request.
  * @param  None
  * @retval None
  */
void EXTI9_5_IRQHandler(void)
{  
  if(EXTI_GetITStatus(KEY_BUTTON_EXTI_LINE) != RESET)
  {  
    if (ReadKey == 1)
    {
      PressedKey =  KEY;     
    }
    else
    {
      ExitMutex = 1;    
    }
    
    /* Clear EXTI interrupts pending bits */
    EXTI_ClearITPendingBit(KEY_BUTTON_EXTI_LINE);
  }
  
 #ifdef USE_STM32100E_EVAL 
  if(EXTI_GetITStatus(SEL_BUTTON_EXTI_LINE) != RESET)
  {
    if (ReadKey == 1)
    {
      PressedKey =  SEL;     
    }
    else
    {
      if (GetMutex == 1)
      {
        NextRep = 1;
        ExitMutex = 1; 
      }
    }
    
    /* Clear EXTI interrupts pending bits */ 
    EXTI_ClearITPendingBit(SEL_BUTTON_EXTI_LINE);
  }
 #endif /* USE_STM32100E_EVAL */    
}

/**
  * @brief  This function handles External lines 15 to 10 interrupt request.
  * @param  None
  * @retval None
  */
void EXTI15_10_IRQHandler(void)
{  
 #ifdef USE_STM32100E_EVAL  
  if(EXTI_GetITStatus(RIGHT_BUTTON_EXTI_LINE) != RESET)
  {
    if (ReadKey == 1)
    {
      PressedKey =  RIGHT;     
    }
    else
    {
      if (GetMutex == 1)
      {
        ShowMutex = 0;
      }
      else
      {
        ReqOperation = SHOW_TEMPERATURE_OP ;
      }
    }
    
    /* Clear EXTI interrupts pending bits */
    EXTI_ClearITPendingBit(RIGHT_BUTTON_EXTI_LINE);
  }
  
  if(EXTI_GetITStatus(LEFT_BUTTON_EXTI_LINE) != RESET)
  {
    if (ReadKey == 1)
    {
      PressedKey =  LEFT;     
    }
    else
    {
      if (GetMutex == 1)
      {
        ShowMutex = 0;
      }
      else
      {
        ReqOperation = SHOW_TIME_OP;
      }
    }
    
    /* Clear EXTI interrupts pending bits */
    EXTI_ClearITPendingBit(LEFT_BUTTON_EXTI_LINE);
  }
  
  if(EXTI_GetITStatus(UP_BUTTON_EXTI_LINE) != RESET)
  {
    if (ReadKey == 1)
    {
      PressedKey =  UP;     
    }
    else
    {
      if (GetMutex == 0)
      {
        ReqOperation = GET_REPORT_OP;
      }
    }
    
    /* Clear EXTI interrupts pending bits */
    EXTI_ClearITPendingBit(UP_BUTTON_EXTI_LINE);
  }
  
  if(EXTI_GetITStatus(DOWN_BUTTON_EXTI_LINE) != RESET)
  {    
    if (ReadKey == 1)
    {
      PressedKey =  DOWN;     
    }
    else
    {
      if (GetMutex == 0)
      {
        ReqOperation = ERASE_EEPROM_OP;
      }
    }
    
    /* Clear EXTI interrupts pending bits */
    EXTI_ClearITPendingBit(DOWN_BUTTON_EXTI_LINE);
  }
 #endif /* USE_STM32100E_EVAL */ 
  
  if(EXTI_GetITStatus(IOE_IT_EXTI_LINE) != RESET)
  {     
 #ifdef  USE_STM3210C_EVAL     
    static JOY_State_TypeDef JoyState = JOY_NONE;
 #endif /* USE_STM3210C_EVAL */
    static TS_STATE* TS_State;
    
    /* Check if the interrupt source is the Touch Screen */
    if (IOE_GetGITStatus(IOE_1_ADDR, IOE_TS_IT) & IOE_TS_IT)
    { 
      /* Update the structure with the current position */
      TS_State = IOE_TS_GetState();  
      
      if ((TS_State->TouchDetected) && (TS_State->Y < 230) && (TS_State->Y > 200))
      {
        if ((TS_State->X > 10) && (TS_State->X < 70))
        {
          LCD_DisplayStringLine(Line7, (uint8_t*)" TS1                ");
        }
        else if ((TS_State->X > 90) && (TS_State->X < 150))
        {
          LCD_DisplayStringLine(Line7, (uint8_t*)"      TS2           ");
        }
        else if ((TS_State->X > 170) && (TS_State->X < 230))
        {
          LCD_DisplayStringLine(Line7, (uint8_t*)"           TS3      ");
        }     
        else if ((TS_State->X > 250) && (TS_State->X < 310))
        {
          LCD_DisplayStringLine(Line7, (uint8_t*)"                TS4 ");
        }          
      }
      else
      {
        STM_EVAL_LEDOff(LED1);
        STM_EVAL_LEDOff(LED2);
        STM_EVAL_LEDOff(LED3);
        STM_EVAL_LEDOff(LED4);
      }    
      
      /* Clear the interrupt pending bits */    
      IOE_ClearGITPending(IOE_1_ADDR, IOE_TS_IT); 
    }
 #ifdef USE_STM3210C_EVAL     
    else if (IOE_GetGITStatus(IOE_2_ADDR, IOE_GIT_GPIO))
    {       
      /* Get the Joystick State */
      JoyState = IOE_JoyStickGetState();
      
      switch (JoyState)
      {
      case JOY_NONE:
        break;
        
      case JOY_UP:
        if (ReadKey == 1)
        {
          PressedKey =  UP;     
        }
        else
        {
          if (GetMutex == 0)
          {
            ReqOperation = GET_REPORT_OP;
          }     
        }
        break;     
        
      case JOY_DOWN:
        if (ReadKey == 1)
        {
          PressedKey =  DOWN;     
        }
        else
        {
          if (GetMutex == 0)
          {
            ReqOperation = ERASE_EEPROM_OP;
          }
        }
        break;       
        
      case JOY_RIGHT :
        if (ReadKey == 1)
        {
          PressedKey =  RIGHT;     
        }
        else
        {        
          if (GetMutex == 1)
          {
            ShowMutex = 0;
          }
          else
          {
            ReqOperation = SHOW_TEMPERATURE_OP ;
          }
        }
        break;     
        
      case JOY_LEFT:
        if (ReadKey == 1)
        {
          PressedKey =  LEFT;     
        }
        else
        {        
          if (GetMutex == 1)
          {
            ShowMutex = 0;
          }
          else
          {
            ReqOperation = SHOW_TIME_OP;
          }
        }
        break;       
        
      case JOY_CENTER:
        if (ReadKey == 1)
        {
          PressedKey =  SEL;     
        }
        else
        {
          if (GetMutex == 1)
          {
            NextRep = 1;
            ExitMutex = 1; 
          }
        }
        break; 
        
      default:
        LCD_DisplayStringLine(Line9, (uint8_t*)"     JOY  ERROR     ");
        break;           
      }
      
      /* Clear the interrupt pending bits */    
      IOE_ClearGITPending(IOE_2_ADDR, IOE_GIT_GPIO);
      IOE_ClearIOITPending(IOE_2_ADDR, IOE_JOY_IT);   
    }
 #endif /* USE_STM3210C_EVAL */       
    else
    {
      /* Clear the interrupt pending bits */       
 #ifdef  USE_STM3210C_EVAL 
      IOE_ClearGITPending(IOE_1_ADDR, ALL_IT);
      IOE_ClearGITPending(IOE_2_ADDR, ALL_IT);
 #else 
      IOE_ClearGITPending(IOE_1_ADDR, ALL_IT);
 #endif /* USE_STM3210C_EVAL */   
    }    
  }
  
  /* Clear EXTI interrupts pending bits */
  EXTI_ClearITPendingBit(IOE_IT_EXTI_LINE); 
}
#endif /* USE_STM322xG_EVAL */

/**
  * @brief  This function handles TIM7 global interrupt request.
  * @param  None
  * @retval None
  */
void TIM7_IRQHandler(void)
{
  if (TIM_GetITStatus(TIM7, TIM_IT_Update) != RESET)
  {  
    /* if Display Timeout is working */    
    if(DisplayTimeout != TIMEOUT_DEFAULT)
    {
      DisplayTimeout--;
      
      if (!DisplayTimeout)
      {
        DisplayTimeout = TIMEOUT_DEFAULT;
        ExitMutex = 1;
      }
    }
    
    /* Toggle leds */
    STM_EVAL_LEDToggle(LED1);
    STM_EVAL_LEDToggle(LED2);
    STM_EVAL_LEDToggle(LED3);
    STM_EVAL_LEDToggle(LED4);
    
    /* if 20 TIM7 interrupt reached */
    if (SaveReportCmd == 20)
    {     
        ReqOperation = SAVE_REPORT_OP;
        SaveReportCmd = 0;     
    }
    else
    {
      SaveReportCmd++;
    }
    
    /* Clear TIM7 interrupt pending bits */
    TIM_ClearITPendingBit(TIM7, TIM_IT_Update);
  } 
}

/**
  * @}
  */ 


/******************* (C) COPYRIGHT 2011 STMicroelectronics *****END OF FILE****/
