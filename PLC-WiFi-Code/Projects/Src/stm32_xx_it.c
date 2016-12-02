  /**
  ******************************************************************************
  * @file    stm32_xx_it.c
  * @author  Central LAB
  * @version V1.0.0
  * @date    17-May-2015
  * @brief   Main Interrupt Service Routines.
  *          This file provides template for all exceptions handler and 
  *          peripherals interrupt service routine.
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; COPYRIGHT(c) 2015 STMicroelectronics</center></h2>
  *
  * Redistribution and use in source and binary forms, with or without modification,
  * are permitted provided that the following conditions are met:
  *   1. Redistributions of source code must retain the above copyright notice,
  *      this list of conditions and the following disclaimer.
  *   2. Redistributions in binary form must reproduce the above copyright notice,
  *      this list of conditions and the following disclaimer in the documentation
  *      and/or other materials provided with the distribution.
  *   3. Neither the name of STMicroelectronics nor the names of its contributors
  *      may be used to endorse or promote products derived from this software
  *      without specific prior written permission.
  *
  * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
  * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
  * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
  * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
  * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
  * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
  * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
  * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
  * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
  * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
  *
  ******************************************************************************
  */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "stm32_xx_it.h"
#include "wifi_module.h"
#include "wifi_module_conf.h"
#include "string.h"
#include "stdbool.h"
#include "ring_buffer.h"    
#include "X_NUCLEO_PLC01A1.h"
#include "stm32_spwf_wifi.h" 
#include "Ladder_Lib.h"
#include "stm32f4xx_hal.h"



#define SLEEP_RESUME_PREVENT    2000
 


/** @addtogroup User     User
  * @{
  */


/** @defgroup Interrupt_Private_Variables          Interrupt Private Variables
* @{
*/


uint8_t Query_CLT=0;
uint8_t WD_Refresh=0;
uint16_t npolling=0;
uint8_t fault_tick=0;
uint8_t refresh=0;
extern uint8_t index_c;
extern uint16_t delay_CNT[];
extern int8_t POG;

//extern uint8_t timer_1,timer_2;
extern uint16_t cnt1[],cnt4;
/**
  * @} // End Interrupt_Private_Variables 
  */

/** @defgroup Interrupt_Exported_Variables    Interrupt Exported Variables
* @{
*/
/* Private variables ---------------------------------------------------------*/
extern bool VNI_Transmission;
extern bool Timer_Running;
extern bool Standby_Timer_Running;
extern bool AT_Cmd_Processing;
extern uint32_t tickcount;
extern uint32_t standby_time;
extern uint8_t CLT_READ;
extern bool Deep_Sleep_Timer;
extern bool Deep_Sleep_Enabled;
extern bool Deep_Sleep_Callback;
extern bool polling;
extern bool RX_ClientFrame;
extern uint32_t sleep_count;
extern uint16_t WD_Reset;
extern TimerStruct_Typedef   tim_setting[MAX_TIM_PARAM];
extern BOARD_Typedef Board_State;
extern uint8_t timer1_cnt;
extern uint8_t timer2_cnt;
extern uint8_t cnt1TH,cnt2TH;
/**
  * @} // End Interrupt_Exported_Variables 
  */


/** @defgroup Interrupt_Exported_Typedef    Interrupt Exported Typedef
* @{
*/

/* Private typedef -----------------------------------------------------------*/
extern CounterStruct_Typedef    counter_up[];
extern UART_HandleTypeDef UartWiFiHandle,UartMsgHandle;
extern TIM_HandleTypeDef    TimHandle, PushTimHandle;
extern PLC_State_TypeDef PLC_State;
extern CLT_CheckState_Typedef CLT_State;
extern wifi_state_t wifi_state;                         /*!< wifi status variable    */
extern Decode_Status_Typedef Frame_decoding;            /*!< frame decoding variable */
extern BOARD_Typedef Board_State;                       /*!< state machine variable  */
extern TimerStruct_Typedef  tim_setting[MAX_TIM_PARAM];
extern TIM_HandleTypeDef htim2,htim1,htim,htim4;
/**
  * @} // End Interrupt_Exported_Typedef 
  */

/** @defgroup Interrupt_Private_Define    Interrupt Private define
* @{
*/

/* Private define ------------------------------------------------------------*/
#define listening_time 2000 //ms   
#define loop_time 200//10000

/**
  * @} // End Interrupt_Private_Define 
  */



/** @addtogroup Interrupt_Private_Routines   Interrupt Private Routines
  * @{
  */


/* Private function prototypes -----------------------------------------------*/
void USARTx_IRQHandler(void);    
void USARTx_PRINT_IRQHandler(void);
void USARTx_EXTI_IRQHandler(void);

/* Private functions ---------------------------------------------------------*/

/******************************************************************************/
/*            Cortex-M3 Processor Exceptions Handlers                         */
/******************************************************************************/

/**
  * @brief   This function handles NMI exception.
  */
void NMI_Handler(void)
{
}

/**
  * @brief  This function handles Hard Fault exception.
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
  */
void MemManage_Handler(void)
{
  /* Go to infinite loop when Memory Manage exception occurs */
  while (1)
  {
    //BSP_LED_On(LED2); 
  }
}

/**
  * @brief  This function handles Bus Fault exception.
  */
void BusFault_Handler(void)
{
  /* Go to infinite loop when Bus Fault exception occurs */
  while (1)
  {
    //BSP_LED_On(LED2); 
  }
}

/**
  * @brief  This function handles Usage Fault exception.
  */
void UsageFault_Handler(void)
{
  /* Go to infinite loop when Usage Fault exception occurs */
  while (1)
  {
    //BSP_LED_On(LED2); 
  }
}

/**
  * @brief  This function handles SVCall exception.
  */
void SVC_Handler(void)
{
  //BSP_LED_On(LED2); 
}

/**
  * @brief  This function handles Debug Monitor exception.
  */
void DebugMon_Handler(void)
{
  //BSP_LED_On(LED2); 
}

/**
  * @brief  This function handles PendSVC exception.
  */
void PendSV_Handler(void)
{
}

/******************************************************************************/
/*                 STM32F1xx Peripherals Interrupt Handlers                   */
/*  Add here the Interrupt Handler for the used peripheral(s) (PPP), for the  */
/*  available peripheral interrupt handler's name please refer to the startup */
/*  file (startup_stm32f1xx.s).                                               */
/******************************************************************************/

/**
  * @brief  This function handles TIM interrupt request.
  */
void TIMx_IRQHandler(void)
{
  HAL_TIM_IRQHandler(&TimHandle);
  
}





void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{       
  
   
        if(htim==&htim1)
        {
          if(cnt1[0]==cnt1TH)
            {
              if((POG!=1)&&(Board_State!=BOARD_LISTENING))
              {
                PLC_Polling();
              }
              timer1_cnt=1;
            }
            else
              cnt1[0]++; 

        }
        else
        {
          if(htim==&htim4)
          { 
            if(cnt1[1]==cnt2TH)
            {
              if((POG!=1)&&(Board_State!=BOARD_LISTENING))
              {
                PLC_Polling();
              }
              timer2_cnt=1;
            }
            else
              cnt1[1]++; 

          }
          else
          Wifi_TIM_Handler(htim);
        }

}










/**
* @brief This function handles TIM2 global interrupt.
*/
void TIM2_IRQHandler(void)
{
  /* USER CODE BEGIN TIM2_IRQn 0 */

  /* USER CODE END TIM2_IRQn 0 */
  HAL_TIM_IRQHandler(&htim2);
  /* USER CODE BEGIN TIM2_IRQn 1 */

  /* USER CODE END TIM2_IRQn 1 */
}




/**
  * @brief  This function handles SysTick Handler.
  */
void SysTick_Handler(void)
{
  /******** PLC ***********/
  if(counter_up[0].CNT_val!=0)
     delay_CNT[0]++;
  if(counter_up[1].CNT_val!=0)
    delay_CNT[1]++;
  if(counter_up[2].CNT_val!=0)
    delay_CNT[2]++;
  if(counter_up[3].CNT_val!=0)
    delay_CNT[3]++;
    
    
    if(WD_Reset>100)
    {
      WD_Reset++;
      npolling=0;
    } 
    else
      WD_Reset=0;
    /* end RESET SPURIOUS */

    if(WD_Refresh>=1)
    {
      WD_Refresh=0;
      HAL_GPIO_WritePin(GPIOC, GPIO_PIN_7, GPIO_PIN_RESET); //CS pin
      HAL_GPIO_WritePin(GPIOC, GPIO_PIN_7,GPIO_PIN_SET);//CS1 watchdog refresh
    }
    else
    {
      WD_Refresh++;      
    }
     
      if(Board_State==BOARD_LOOP)
      {
        
        //while(npolling==802);
        if(npolling>=loop_time)
         { 
           if(Frame_decoding==Completed)
           {
            Board_State=BOARD_LISTENING; 

            Query_CLT=0;
            CLT_READ=0;
            npolling=0;
            
            }
           
         }
        else
        {     CLT_READ=1;
//         if(Query_CLT>=10)
//            {
//              Query_CLT=0;
//              CLT_READ=1;
//            }
//           else
//           {
//             Query_CLT++;
//
//           }
            npolling++;
        }
      }
    else
     {  
       if((Board_State==BOARD_LISTENING)&&(Get_FlagStatus()!=1))
        {
          if(npolling>=listening_time)
            {              
               if(wifi_state==wifi_state_idle)
               {
               Board_State=BOARD_LOOP;
               
               npolling=0;
               }              
            } 
          else
            npolling++;  
        }
     }//prova
  HAL_IncTick();
 Wifi_SysTick_Isr();
}

/**
  * @brief  This function handles EXTI Handler.
  */
void USARTx_EXTI_IRQHandler(void)
{
  HAL_GPIO_EXTI_IRQHandler(WiFi_USART_RX_PIN);
}

/**
  * @brief  This function GPIO EXTI Callback.
  * @param  Pin number of the GPIO generating the EXTI IRQ
  */
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
  
  if(GPIO_Pin==WiFi_USART_RX_PIN && (HAL_GPIO_ReadPin(WiFi_USART_RX_GPIO_PORT, WiFi_USART_RX_PIN) == GPIO_PIN_SET))
  {
    HAL_NVIC_DisableIRQ(USARTx_EXTI_IRQn);
    UART_Configuration();//reconfigure the UART for WIND reception
    
    //Resume_Timer_Running = TRUE;
    AT_Cmd_Processing = WIFI_FALSE;
    Receive_Data();
  }
}





/********************  Timer component **********/

/**
* @brief This function handles TIM1 Update interrupt and TIM10 global interrupt.
*/
void TIM1_UP_TIM10_IRQHandler(void)
{
  /* USER CODE BEGIN TIM1_UP_TIM10_IRQn 0 */

  /* USER CODE END TIM1_UP_TIM10_IRQn 0 */
  HAL_TIM_IRQHandler(&htim1);
  /* USER CODE BEGIN TIM1_UP_TIM10_IRQn 1 */

  /* USER CODE END TIM1_UP_TIM10_IRQn 1 */
}

/**
* @brief This function handles TIM4 global interrupt.
*/
void TIM4_IRQHandler(void)
{
  /* USER CODE BEGIN TIM4_IRQn 0 */
//   uint16_t pippo=0;
//pippo=htim.Instance->CNT;
  /* USER CODE END TIM4_IRQn 0 */
  HAL_TIM_IRQHandler(&htim4);
  /* USER CODE BEGIN TIM4_IRQn 1 */

  /* USER CODE END TIM4_IRQn 1 */
}
/******************************************************************************/
/*                 STM32 Peripherals Interrupt Handlers                   */
/*  Add here the Interrupt Handler for the used peripheral(s) (PPP), for the  */
/*  available peripheral interrupt handler's name please refer to the startup */
/*  file (startup_stm32xxx.s).                                            */
/******************************************************************************/

/**
  * @brief  This function handles USARTx Handler.
  */
void USARTx_IRQHandler(void)
{
  HAL_UART_IRQHandler(&UartWiFiHandle);
}

/**
  * @brief  This function handles USARTx vcom Handler.
  */
#ifdef USART_PRINT_MSG
void USARTx_PRINT_IRQHandler(void)
{
   HAL_UART_IRQHandler(&UartMsgHandle);
}
#endif

/**
  * @}//  Interrupt_Private_Routines
  */ 

/**
  * @}//  End User
  */

/**
  * @}//  End User
  */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
