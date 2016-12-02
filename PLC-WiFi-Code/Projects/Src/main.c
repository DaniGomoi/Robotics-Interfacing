/**
  ******************************************************************************
  * @file    main.c
  * @author  System Lab Industrial & Motion Control Team
  * @version V1.0.0
  * @date    02-Dicember-2016
  * @brief   Main program body
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
#include "wifi_interface.h"
#include "wifi_module.h"
#include "stdio.h"
#include "stdbool.h"
#include "string.h"
#include "PLC_Handling.h"

#include "stm32f4xx_hal.h"
#include "stm32f4xx_hal_tim.h"
#include "stm32f4xx_nucleo.h"
#include "stm32_hal_legacy.h"
#include "stm32f4xx_hal_usart.h"
#include "stm32f4xx_hal_conf.h"
#include "X_NUCLEO_PLC01A1.h"
#include "X_NUCLEO_01A1.h"
#include "VNI8200XP.h"
#include "Ladder_Lib.h"



/** @addtogroup PLC_Demo_Example     PLC_Demo Example 
  * @{
  */

/** @addtogroup User        User
  * @{
  */

/* Private define ------------------------------------------------------------*/

/* Private macro -------------------------------------------------------------*/


/** @defgroup    Private_variables    Private variables
  * @{
  */


/* Private variables ---------------------------------------------------------*/
uint8_t rung_conf[MAX_SERVERDATA_RX];    
uint16_t WD_Reset = 0;    
uint8_t CLT_READ = 0;
int8_t  ServerData_RX[512];
uint8_t output_status=0;
uint16_t ToISOPLC =0;
uint8_t ToSPI=0;
uint8_t ToIPS=0;
uint8_t Output_Index=0;
uint16_t* param_array;
char* var;
bool RX_ClientFrame=WIFI_FALSE;
bool client_connection=WIFI_FALSE;

/**
  * @}  // end Private_variables 
  */



/** @defgroup    Private_typedef    Private typedef
  * @{
  */

/* Private typedef -----------------------------------------------------------*/
SPI_HandleTypeDef hspi1;
PLC_State_TypeDef PLC_State;
CLT_CheckState_Typedef CLT_State;
Decode_Status_Typedef Frame_decoding;
PLC_State_TypeDef PLC_State;
BOARD_Typedef Board_State;
TimerParam_Typedef Tim_Struct;
TIM_HandleTypeDef htim2,htim1,htim,htim4;
TimerParam_Typedef Tim_Setting;

/**
  * @}  // end Private_typedef 
  */



/** @defgroup    Exported_variables    Exported variables
  * @{
  */

extern uint8_t Input_CHS;
extern bool data_mode;
extern TIM_HandleTypeDef  TimHandle;
extern OutputStructure_Typedef output[MAX_OUTPUT_NUMBER];
extern OutputStructure_Typedef output_temp[MAX_OUTPUT_NUMBER];
extern uint8_t CLT_Device;
extern uint16_t npolling;
extern UART_HandleTypeDef UartHandle,UartMsgHandle;
extern char print_msg_buff[512];
extern char UserDataBuff[];
extern uint8_t output_buffer,VNI_SPI;

/**
  * @}  // end Exported_variables 
  */



/** @defgroup   Private_functions    Private functions
  * @{
  */

/* Private function prototypes -----------------------------------------------*/
static void initializePlc(void);
void SystemClock_Config(void);
void GPIO_Config(void);
void SPI1_Config(void);

/**
  * @}  // end Private_functions 
  */


/* Private functions ---------------------------------------------------------*/
#ifdef USART_PRINT_MSG
#define printf(arg)    sprintf((char*)print_msg_buff,arg);   \
                       HAL_UART_Transmit(&UartMsgHandle, (uint8_t*)print_msg_buff, strlen(print_msg_buff), 1000);
#endif                       


wifi_state_t wifi_state;
wifi_config config;
                       
uint8_t * ssid = "Ladder Demo Board";
char * seckey = "pacman76";
uint8_t channel_num = 6;
WiFi_Priv_Mode mode = None;//WPA2_Personal;     

char *answer;
uint16_t len;

 /**
  * @brief  Main program
  * @param  None
  * @retval None
  */
int main(void)
{
  
  WiFi_Status_t status = WiFi_MODULE_SUCCESS;
  uint8_t *protocol = "t";
  uint32_t portnumber = 32000;
Board_State=BOARD_RESET;
HAL_DeInit();
HAL_Init();

  /* Configure the system clock to 64 MHz */
  
  SystemClock_Config();
  
  /* configure the timers  */
      
  GPIO_Config();
  SPI1_Config();
  UART_Configuration();
  Timer_Config( );  
  
  initializePlc();
    
  /* Reset relay at startup to avoid FAULT */
  BSP_RELAY_Reset();
  

#ifdef USART_PRINT_MSG// remove becaue it is fo virtual com
  
  UART_Msg_Gpio_Init();
  USART_PRINT_MSG_Configuration();
  
#endif  
  
  config.power=active;
  config.power_level=high;
  config.dhcp=on;//use DHCP IP address

    /* Configuration for PLC */
  
  wifi_state = wifi_state_idle;
  
  /* Init the wi-fi module */  

  status = wifi_init(&config);

  if(status!=WiFi_MODULE_SUCCESS)
  {
    printf("Error in Config");
    return 0;
  }
 
    HAL_GPIO_WritePin(GPIOC, GPIO_PIN_7,GPIO_PIN_RESET);//CS1 watchdog refresh
  /* Delfault driving pin configuration for VNI */
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_6, GPIO_PIN_SET); //CS pin
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_10, GPIO_PIN_SET);//OUTEN pin

    HAL_GPIO_WritePin(GPIOC, GPIO_PIN_7,GPIO_PIN_SET);//CS1 watchdog refresh
    HAL_GPIO_WritePin(GPIOC, GPIO_PIN_7,GPIO_PIN_RESET);//CS1 watchdog refresh
    HAL_GPIO_WritePin(GPIOC, GPIO_PIN_7,GPIO_PIN_SET);//CS1 watchdog refresh
   
    Init_Output();
    ClearFlag_ResetOUT();
    BSP_RELAY_EN_Out();
    BSP_RELAY_SetOutputs(&ToSPI);
    PLC_State=PLC_RESET;
    CLT_State=Unchecked;
    Frame_decoding=Completed;
    
  while (1)
  {  

    
    
    do{ 
        HAL_GPIO_WritePin(GPIOC, GPIO_PIN_7, GPIO_PIN_RESET); 
        HAL_GPIO_WritePin(GPIOC, GPIO_PIN_7, GPIO_PIN_SET); 
      switch (wifi_state) 
        {
        case wifi_state_reset:
          break;
        case wifi_state_ready:
           HAL_Delay(1000);
         wifi_ap_start(ssid, channel_num);
         
          wifi_state = wifi_state_idle;
        break;
        
        case wifi_state_connected:
          //printf("\r\n >>connected...\r\n");
          wifi_state = wifi_state_socket;
        break;
        
        case wifi_state_disconnected:
          wifi_state = wifi_state_reset;
        break;

        case wifi_state_socket:
          //printf("\r\n >>WiFi_RW_Data\r\n");
          /* Read Write Socket data */        
          status = wifi_socket_server_open(portnumber, protocol);
          if(status == WiFi_MODULE_SUCCESS)
          {
            //data_mode=WIFI_TRUE;
            //printf("\r\n >>Server Socket Open OK \r\n");          
            //status = wifi_socket_server_close();//as per hands on first sends 13 bytes
          }       
          wifi_state = wifi_state_idle;
   
        break;

        case wifi_state_write:
          
             if(Get_FlagStatus()==1)
              {
                Get_FeedbackICs();                               
                Reset_FlagStatus();
              }
             else
               WiFiServer_Answer((uint8_t*)answer);
          
          wifi_state = wifi_state_idle;
        break;
            
        case wifi_state_idle:
        break;
          
        default:
        break;
      }  
    
      if(RX_ClientFrame==WIFI_TRUE)
      {
        Board_State=BOARD_LISTENING;
        RX_ClientFrame=WIFI_FALSE;   
        Save();
       
        if(WiFi_Decode (rung_conf))
          {
              if(Get_FlagStatus()==1)
              {
                VNI_TxRx(ToSPI);
                CLT_VNI_RxTx();
                 wifi_state=  wifi_state_write;
              }
              else
              {               
                  CLT_VNI_RxTx();
                  if(PLC_GetOutput(&ToISOPLC))
                  {
                            ToSPI=ToISOPLC;
                            output_status=VNI_TxRx(ToSPI);  
                            output_status=VNI_TxRx(ToSPI);  
                            ToIPS=ToISOPLC>>8;
                            BSP_ISO_Com_Settings();
                            BSP_OutputEnable_Pin(GPIO_PIN_SET);
                            BSP_DrivePin_GROUP(ToIPS);                        
                        if((output_status==0)&&(PLC_State==PLC_PROGRAMMED))
                        {
                          if(Get_ResetOUT()==1)
                          {
                            BSP_OutputEnable_Pin(GPIO_PIN_RESET);
                            WiFiServer_Answer("<RESET DONE>");
                            ClearFlag_ResetOUT();
                          }
                          else
                          WiFiServer_Answer("<DONE>");                          
                         
                        }
                        else
                        {
                          WiFiServer_Answer("<FAIL OUTPUTS>");                          
                        }
 
                        Reset_Count();
                   }
                  else//recursive dependency
                  {
                      WiFiServer_Answer("<FAIL EXPRESSION>");
                      Restore();
                  }  
              }
            
          }
          else
          {              
              WiFiServer_Answer("<FAIL SYNTAX>");
             Restore();
          }
       
       }
      else
      { 
        Board_State=BOARD_LOOP;
      }
          
     }while((Board_State==BOARD_LISTENING)||(Board_State==BOARD_RESET));

    if(CLT_READ!=0)
     {
        CLT_READ=0;
         CLT_VNI_RxTx();       
           PLC_Polling ();
          while(PLC_State==PLC_RESET); 

     }
  }
}
 



/**
* @brief  Initialize current limiter and relay.
*         Configures IOs and peripherals
* @param  None
* @retval None
*/
static void initializePlc(void)
{
  /* Initialize Relay and Current Limiter */
  BSP_Relay_Init();
  BSP_CurrentLimiter_Init();  
}



void PLC_Polling (void)
{

    if(PLC_GetOutput(&ToISOPLC))
    {
        ToSPI=ToISOPLC;
        VNI_TxRx(ToSPI);
        VNI_TxRx(ToSPI); 
        ToIPS=ToISOPLC>>8;
        BSP_ISO_Com_Settings();
        BSP_OutputEnable_Pin(GPIO_PIN_SET);
        BSP_DrivePin_GROUP(ToIPS);
    }   
}


void Get_FeedbackICs (void)
{
  
    Output_Index=0;
 
  WiFiServer_Answer(" DIGITAL OUTPUT FEEDBACK");
    WiFiServer_Answer("\r\n");

  do{
     HAL_GPIO_WritePin(GPIOC, GPIO_PIN_7,GPIO_PIN_SET);//watchdog
    var= GET_VNI_OutFB(Output_Index);
    WiFiServer_Answer((unsigned char*)var);
     WiFiServer_Answer("\r\n");
    Output_Index++;
     HAL_GPIO_WritePin(GPIOC, GPIO_PIN_7,GPIO_PIN_RESET);//watchdog

  }while (Output_Index<8);
  
  Output_Index=0;  
  
  do{
     HAL_GPIO_WritePin(GPIOC, GPIO_PIN_7,GPIO_PIN_SET);//watchdog
    var= GET_VNI_FB(Output_Index+4);
    WiFiServer_Answer((unsigned char*)var);
     WiFiServer_Answer("\r\n");
    Output_Index++;
     HAL_GPIO_WritePin(GPIOC, GPIO_PIN_7,GPIO_PIN_RESET);//watchdog

  }while (Output_Index<4);  
  
  Output_Index=0;
  WiFiServer_Answer(" DIGITAL INPUT FEEDBACK"); 
  WiFiServer_Answer("\r\n");

  do{
    
    HAL_GPIO_WritePin(GPIOC, GPIO_PIN_7,GPIO_PIN_SET);//watchdog
    var= GET_CLT_FB(Output_Index);
    WiFiServer_Answer((unsigned char*)var);
     WiFiServer_Answer("\r\n");
    Output_Index++;
    HAL_GPIO_WritePin(GPIOC, GPIO_PIN_7,GPIO_PIN_RESET);//watchdog

  }while (Output_Index<4);
  WiFiServer_Answer("<DONE>");
}


void WiFiServer_Answer(uint8_t* str_message)
{
    uint16_t len=0;
    if(str_message!=NULL)
    {
      answer=(char*)str_message;
      len = strlen(answer);            
      wifi_socket_server_write(len,answer);
    }
    answer=NULL; 
}

/**
  * @brief  System Clock Configuration
  *         The system Clock is configured as follow : 
  *            System Clock source            = PLL (HSI)
  *            SYSCLK(Hz)                     = 64000000
  *            HCLK(Hz)                       = 64000000
  *            AHB Prescaler                  = 1
  *            APB1 Prescaler                 = 2
  *            APB2 Prescaler                 = 1
  *            PLLMUL                         = 16
  *            Flash Latency(WS)              = 2
  * @param  None
  * @retval None
  */



#ifdef USE_STM32F4XX_NUCLEO

void SystemClock_Config(void)
{
        RCC_OscInitTypeDef RCC_OscInitStruct;
  RCC_ClkInitTypeDef RCC_ClkInitStruct;

  __PWR_CLK_ENABLE();

  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE2);

//  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
//  RCC_OscInitStruct.HSEState = RCC_HSE_BYPASS;
//  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
//  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
//  RCC_OscInitStruct.PLL.PLLM = 8;
//  RCC_OscInitStruct.PLL.PLLN = 256;
//  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV4;
//  RCC_OscInitStruct.PLL.PLLQ = 4;
//  HAL_RCC_OscConfig(&RCC_OscInitStruct);
//
//  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_SYSCLK|RCC_CLOCKTYPE_PCLK1;
//  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
//  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
//  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
//  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;
//  HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2);
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_BYPASS;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 8;
  RCC_OscInitStruct.PLL.PLLN = 256;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV4;
  RCC_OscInitStruct.PLL.PLLQ = 4;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

    /**Initializes the CPU, AHB and APB busses clocks 
    */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;
HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2);
  
  HAL_SYSTICK_Config(HAL_RCC_GetHCLKFreq()/1000);

  HAL_SYSTICK_CLKSourceConfig(SYSTICK_CLKSOURCE_HCLK);
}

#endif

/******** Wi-Fi Indication User Callback *********/

void ind_wifi_socket_data_received(uint8_t * data_ptr, uint32_t message_size, uint32_t chunk_size)
{
  uint16_t datalen= strlen((const char*)data_ptr);
  npolling=0;
 
  if(datalen>MAX_SERVERDATA_RX)
  {
    answer="<OVERSIZE>";
    wifi_state = wifi_state_write;
  }
  else
  {
    if(Frame_decoding!=Started)
    {
      memset(rung_conf,0,MAX_SERVERDATA_RX);
      for(uint16_t i=0;i<MAX_SERVERDATA_RX && i<datalen;i++) 
      {
        rung_conf[i]=data_ptr[i];
      }
       
       RX_ClientFrame =WIFI_TRUE;
    }
    else
    {
       answer="<BUSY>";
        wifi_state = wifi_state_write;
    }
  }
}


void ind_wifi_on()
{
    wifi_state = wifi_state_ready;
}

void ind_wifi_connected()
{
  wifi_state = wifi_state_connected;
}

void ind_socket_server_client_joined(void)
{
  client_connection=WIFI_TRUE;
  answer= "<OK>";
  wifi_state = wifi_state_write;

}

void ind_socket_server_client_left(void)
{
    client_connection=WIFI_FALSE;
}

void ind_wifi_ap_client_joined(uint8_t * client_mac_address)
{
  printf("\r\n>>client joined callback...\r\n");
  printf((const char*)client_mac_address);
  
}

void ind_wifi_ap_client_left(uint8_t * client_mac_address)
{
  printf("\r\n>>client left callback...\r\n");
  printf((const char*)client_mac_address);
//  wifi_state=wifi_state_reset ; 
}


/** 
  * @brief: GPIO configuration
  * @param: None
  * @retval: None
*/

void GPIO_Config(void)
{
 GPIO_InitTypeDef GPIO_InitStruct;
  /* GPIO Ports Clock Enable */
  __GPIOA_CLK_ENABLE();
  __GPIOC_CLK_ENABLE();
  __GPIOH_CLK_ENABLE();
  __GPIOB_CLK_ENABLE();


  /*Configure GPIO pins : PB10 PB6 */
  GPIO_InitStruct.Pin = GPIO_PIN_10|GPIO_PIN_6;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  GPIO_InitStruct.Speed = GPIO_SPEED_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : PC7 */
  GPIO_InitStruct.Pin = GPIO_PIN_10;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  
  /*Configure GPIO pins : PC0 PC1 PC7 */
  GPIO_InitStruct.Pin = GPIO_PIN_0|GPIO_PIN_1|GPIO_PIN_7;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  GPIO_InitStruct.Speed = GPIO_SPEED_HIGH;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : PA0 PA1 PA3 PA4 */
  GPIO_InitStruct.Pin = GPIO_PIN_0|GPIO_PIN_1|GPIO_PIN_3|GPIO_PIN_4;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_PULLDOWN;
  GPIO_InitStruct.Speed = GPIO_SPEED_HIGH;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : PA2 */
  GPIO_InitStruct.Pin = GPIO_PIN_2;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : PB0 PB6 */
  GPIO_InitStruct.Pin = GPIO_PIN_0|GPIO_PIN_6;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  GPIO_InitStruct.Speed = GPIO_SPEED_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pins : PB10 PB3 PB4 PB5 
                           PB8 PB9 */
  GPIO_InitStruct.Pin = GPIO_PIN_10|GPIO_PIN_3|GPIO_PIN_4|GPIO_PIN_5 
                          |GPIO_PIN_8|GPIO_PIN_9;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  GPIO_InitStruct.Speed = GPIO_SPEED_HIGH;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);
  
  
  
  
  
  
  
}

/** 
  * @brief: SPI configuration
  * @param: None
  * @retval: None
*/
void SPI1_Config(void)
{

  hspi1.Instance = SPI1;
  hspi1.Init.Mode = SPI_MODE_MASTER;
  hspi1.Init.Direction = SPI_DIRECTION_2LINES;
  hspi1.Init.DataSize = SPI_DATASIZE_16BIT;
  hspi1.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi1.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi1.Init.NSS = SPI_NSS_SOFT;
  hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_16;
  hspi1.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi1.Init.TIMode = SPI_TIMODE_DISABLED;
  hspi1.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLED;
  hspi1.Init.CRCPolynomial = 10;
  HAL_SPI_Init(&hspi1);

}

/** 
  * @brief:     PLC TIMER2 configuration
  * @param:     None
  * @retval:    None
*/
//void PLC_TIM1_Init(void)
//{
// // Calc_TimParam();
//  TIM_ClockConfigTypeDef sClockSourceConfig;
//  TIM_MasterConfigTypeDef sMasterConfig;
//
//  htim1.Instance = TIM1;
//  htim1.Init.Prescaler = 0;
//  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
//  htim1.Init.Period = 0;
//  htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
//  htim1.Init.RepetitionCounter = 0;
//  HAL_TIM_Base_Init(&htim1);
//
//  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
//  HAL_TIM_ConfigClockSource(&htim1, &sClockSourceConfig);
//
//  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
//  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
//  HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig);
//
//}


#ifdef  USE_FULL_ASSERT

/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
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
  
/**
* @}
*/
/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
