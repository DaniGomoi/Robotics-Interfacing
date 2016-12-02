/**
 ******************************************************************************
 * @file    wifi_module.c
 * @author  Central LAB
 * @version V1.0.0
 * @date    17-May-2015
 * @brief   Enable Wi-Fi functionality using AT cmd set
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
#include "wifi_module.h"
#include "stm32_spwf_wifi.h"
#include "ring_buffer.h"
#include "stdio.h"
#include "string.h"

/** @addtogroup MIDDLEWARE    MIDDLEWARE
* @{
*/ 


/** @defgroup  NUCLEO_WIFI_MODULE  NUCLEO WIFI MODULE
  * @brief Wi-Fi driver modules
  * @{
  */


/** @defgroup NUCLEO_WIFI_MODULE_Private_Defines  NUCLEO WIFI MODULE Private Defines
  * @{
  */


/**
  * @}
  */


/** @addtogroup NUCLEO_WIFI_MODULE_Private_Variables    NUCLEO WIFI MODULE Private Variables
  * @{
  */
/* Private variables ---------------------------------------------------------*/

/*All Buffers*/
uint8_t WiFi_AT_Cmd_Buff[1024];
uint8_t process_buffer[MAX_BUFFER_GLOBAL];
char UserDataBuff[MAX_BUFFER_GLOBAL*2];
uint8_t USART_RxBuffer[64];//This buffer is only used in the Init phase (to receive "\r\nOK\r\n")
char print_msg_buff[MAX_BUFFER_GLOBAL];
volatile uint8_t ring_buffer[RINGBUF_SIZE];/* Default size for ring buffer */
volatile uint8_t pop_buffer[1];
uint8_t uart_byte[1];
buffer_t big_buff;
wifi_scan *wifi_scanned_list; //[MAX_WIFI_SCAN_NETWORK];

__IO uint32_t packet_sent=WIFI_TRUE;
__IO wifi_bool WiFi_Connected = WIFI_FALSE;
__IO wifi_bool IsBuffer1Full = WIFI_FALSE,IsBuffer2Full = WIFI_FALSE; 
__IO uint8_t	uTimeOut;
__IO ITStatus UartReady = RESET;
__IO ITStatus Uart2Ready = RESET;
__IO ITStatus TxUartReady = RESET;

wifi_bool USART2_MesgReadyBit = WIFI_FALSE;
wifi_bool Received_HTTPResponce = WIFI_FALSE;
wifi_bool Single_Digit_Indication = WIFI_FALSE;
wifi_bool WiFi_Enabled =WIFI_FALSE;
wifi_bool HttpReqReady = WIFI_FALSE;
wifi_bool WiFi_In_Config_Mode = WIFI_FALSE;
wifi_bool WiFi_Configuration_Done = WIFI_FALSE;
wifi_bool IsConsoleActive = WIFI_FALSE ;
wifi_bool process_Indication_Completed = WIFI_TRUE,process_WiFi_Resp_Completed = WIFI_FALSE;
wifi_bool Timer_Running = WIFI_FALSE;
wifi_bool enable_dequeue = WIFI_TRUE;
wifi_bool Standby_Timer_Running = WIFI_FALSE;
wifi_bool trigger_wakeup_callback = WIFI_FALSE;
wifi_bool Deep_Sleep_Enabled = WIFI_FALSE;
wifi_bool Standby_Enabled = WIFI_FALSE;
wifi_bool Low_Power_Enabled = WIFI_FALSE;
wifi_bool command_mode=WIFI_TRUE;
wifi_bool data_mode=WIFI_FALSE;
wifi_bool Scan_Ongoing = WIFI_FALSE;
wifi_bool AT_Cmd_Ongoing = WIFI_FALSE;
wifi_bool AT_Cmd_Processing = WIFI_FALSE;
wifi_bool Uartx_Rx_Processing = WIFI_FALSE;
wifi_bool Client_Connected = WIFI_FALSE;
wifi_bool Client_Disconnected = WIFI_FALSE;
wifi_bool switch_by_default_to_command_mode = WIFI_TRUE;
wifi_bool data_pending_sockD=WIFI_FALSE;
wifi_bool enable_sock_read = WIFI_FALSE;
wifi_bool enable_query = WIFI_FALSE;
wifi_bool encoded_data_read = WIFI_FALSE;
wifi_bool Set_AT_Cmd_Response_False = WIFI_FALSE;
volatile uint32_t tickcount;
uint8_t SocketId; 
uint32_t SockON_Data_Len;

UART_HandleTypeDef UartWiFiHandle,UartMsgHandle;
WiFi_Config_HandleTypeDef WiFi_Config_Variables;

float fValue = 0;

extern wifi_bool Trace_Mode_Enable;
extern TIM_HandleTypeDef    TimHandle, PushTimHandle;
extern uint8_t current_Node_Index;
extern uint8_t Current_Node_Id;

__IO WiFi_WIND_State_TypeDef WiFi_WIND_State;//describes the current WIND number in processing

uint8_t WiFi_Resp_OK = 0;
uint32_t number_of_bytes=0;
uint32_t interim_number_of_bytes=0;
uint32_t Interim_SockON_Data_Len=0;
uint32_t bytes_to_be_read = 0;
uint32_t sock_total_count=0;
uint32_t sockD_total_count=0;
uint32_t ip_fragment_count=0;
uint32_t chunk_size;
uint32_t message_size;
uint32_t WIND55_count=0;
uint32_t WIND64_count=0;
uint8_t user_scan_number;

WiFi_AT_CMD_Response_t WiFi_Module_State;
wifi_bool tempcheck = WIFI_FALSE;
uint8_t *WiFi_Scan_Buffer;

wifi_bool Pending_SockON_Callback = WIFI_FALSE;
wifi_bool Pending_SockD_Callback = WIFI_FALSE;
wifi_bool SockON_Server_Closed_Callback = WIFI_FALSE;
wifi_bool Client_Socket_Close_Cmd = WIFI_FALSE;
wifi_bool standby_resume_callback = WIFI_FALSE;
wifi_bool HTTP_Data_available = WIFI_FALSE;
uint8_t remote_socket_closed_id;
volatile uint8_t wifi_ready = WIFI_FALSE;//Set once if wifi is ready for first time
volatile uint8_t wifi_connected = WIFI_FALSE;//Set once if wifi is connected for first time
volatile uint8_t wifi_client_connected = 0;//Set once if client is connected
volatile uint8_t wifi_client_disconnected = 0;//Set once if client is dis-connected
uint8_t gpio_value, gpio_dir, get_cfg_value[64];
WiFi_Status_t user_error_code = WiFi_MODULE_SUCCESS;
HAL_StatusTypeDef receive_status;
uint8_t no_of_open_client_sockets = 0;
wifi_bool open_sockets[8];//Max open sockets allowed is 8. Each array element depicts one socket (true=open, false=closed)
uint8_t client_MAC_address[17];//current client MAC address store

typedef enum {
start_r_detected = 0x00,
started,
stopped
} process_state;

uint8_t line_pointer = 0;
process_state state = stopped;
uint8_t * temp;//pop buffer temporary
uint8_t enable_uart_byte_data_receive=1;
uint8_t uart_data_receive_ready=1;
wifi_bool resume_receive_data = WIFI_FALSE;

WiFi_Status_t AT_RESPONSE = WiFi_AT_CMD_RESP_ERROR;
__IO wifi_bool AT_Response_Received = WIFI_FALSE;
wifi_bool Received_Last_Byte_CB = WIFI_FALSE;
wifi_bool Deep_Sleep_Timer = WIFI_FALSE;
wifi_bool Deep_Sleep_Callback = WIFI_FALSE;
uint32_t sleep_count = 0;
uint32_t standby_time = 0;
uint8_t scanned_ssids = 0;

GPIO_InitTypeDef  GPIO_InitStruct;
GPIO_InitTypeDef  WAKEUP_InitStruct;

#ifdef USART_PRINT_MSG
#define printf(arg)    {sprintf((char*)print_msg_buff,arg);   \
HAL_UART_Transmit(&UartMsgHandle, (uint8_t*)print_msg_buff, strlen(print_msg_buff), 1000);}
#endif   

// [DLI]
#ifdef WIFI_USE_VCOM

uint8_t console_listen_char[1];
uint8_t console_input_char[1];
uint8_t console_send_char[1];
uint8_t console_echo_char[1];
__IO ITStatus console_send_ready = RESET;
__IO ITStatus console_echo_ready = SET;
__IO ITStatus console_push_ready = RESET;

// Virtual-COM UART
void console_input() {
  HAL_UART_Receive_IT(&UartMsgHandle, (uint8_t *)console_input_char, 1);
}

void wifi_vcom(void) {

  if (console_push_ready == SET) {
     push_buffer(&big_buff, uart_byte);
     console_push_ready = RESET;
     HAL_UART_Receive_IT(&UartWiFiHandle, (uint8_t *)uart_byte, 1);
  }
 if(console_echo_ready == SET) {
    temp = pop_buffer_queue(&big_buff);
    if(temp != NULL) {
      console_echo_ready = RESET;
      HAL_UART_Transmit_IT(&UartMsgHandle, temp, 1);
    }
 }
}

#endif

/**
  * @}
  */

  
/** @defgroup NUCLEO_WIFI_MODULE_Private_Functions
  * @{
  */


/**
  * @brief  WiFi_Module_Init
  *         Initialize wifi module
  * @param  None
  * @retval None
  */
void WiFi_Module_Init(void)
{
#ifdef WIFI_USE_VCOM
  console_input();
#else
  WiFi_Module_State = Receive_Indication;
#endif  
  init(&big_buff, 1024);//Init the ring buffer  
  Receive_Data();
#ifndef WIFI_USE_VCOM
  Start_Timer();  
  memset(open_sockets,0x00, 8);//init the open socket array
  
  if (HAL_TIM_Base_Start_IT(&TimHandle) != HAL_OK)//Start the TIM timer
  {
    #if DEBUG_PRINT
    printf("Error");
    #endif
    Error_Handler();
  }
#endif  
  
}

/**
* @brief  WiFi_Configuration
*         Default Wifi configuration parameters
* @param  None
* @retval None
*/
void WiFi_Configuration()
{
  WiFi_Config_Variables.blink_led               = 1;
  WiFi_Config_Variables.ip_use_dhcp             = 1 ;

  /* Set the network privacy mode 
    (0=none, 1=WEP, 2=WPA-Personal (TKIP/AES) or WPA2-Personal (TKIP/AES)) */  
  WiFi_Config_Variables.wifi_mode               =  WiFi_STA_MODE;
  WiFi_Config_Variables.wifi_priv_mode          =  WPA_Personal;
  WiFi_Config_Variables.wifi_ssid               =  "NETGEAR54" ;
  WiFi_Config_Variables.Wifi_Sec_key            =  "12341234";
  
   /*Power Management Settings*/
  WiFi_Config_Variables.sleep_enabled           = 0;//0=disabled, 1=enabled
  WiFi_Config_Variables.standby_enabled         = 1;
  WiFi_Config_Variables.standby_time            = 10;//in seconds
  WiFi_Config_Variables.wifi_powersave          = 1;//0=Active, 1=PS Mode, 2=Fast-PS Mode
  WiFi_Config_Variables.wifi_operational_mode   = 11;//11= Doze mode, 12= Quiescent mode
  WiFi_Config_Variables.wifi_listen_interval    = 0; //Wakeup every n beacon
  WiFi_Config_Variables.wifi_beacon_wakeup      = 1;         
}



WiFi_Status_t SET_WiFi_WEPKey(char* seckey)
{
WiFi_Status_t status = WiFi_MODULE_SUCCESS;

Reset_AT_CMD_Buffer(); 

/* AT+S.SCFG=wifi_wpa_psk_text,helloworld : set password */
sprintf((char*)(char*)WiFi_AT_Cmd_Buff,"AT+S.SCFG=wifi_wep_keys[0],%s\r",seckey); 

status = USART_Transmit_AT_Cmd();
if(status == WiFi_MODULE_SUCCESS)
{
status = USART_Receive_AT_Resp(Receive_AT_Cmd_Response);
}
return status; 
}





/**
* @brief  wifi_reset
*         Reset WiFi module using PC12 gpio pin
* @param  None
* @retval None
*/
void wifi_reset(void)
{  
  RESET_WAKEUP_GPIO_CLK_ENABLE();
  __GPIOA_CLK_ENABLE();
  
  GPIO_InitStruct.Pin       = WiFi_RESET_GPIO_PIN;
  GPIO_InitStruct.Mode      = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull      = GPIO_PULLUP;
  GPIO_InitStruct.Speed     = GPIO_SPEED_HIGH;

  HAL_GPIO_Init(WiFi_RESET_GPIO_PORT, &GPIO_InitStruct);
  
  WiFi_Module_State = Receive_Indication; 
  WiFi_WIND_State.WiFiHWStarted = WIFI_FALSE;
  memset((void*)&WiFi_WIND_State,0x00,sizeof(WiFi_WIND_State)); /*reset the WIND State?*/
  
  /* ===   RESET PIN - PC12   ===*/
  HAL_GPIO_WritePin(WiFi_RESET_GPIO_PORT, WiFi_RESET_GPIO_PIN, GPIO_PIN_RESET);  
  HAL_Delay(10);  
  /* ===   SET PIN - PC12   ===*/
  HAL_GPIO_WritePin(WiFi_RESET_GPIO_PORT, WiFi_RESET_GPIO_PIN, GPIO_PIN_SET);
  HAL_Delay(10);
  HAL_GPIO_DeInit(WiFi_RESET_GPIO_PORT, WiFi_RESET_GPIO_PIN); 
  
  while(WiFi_WIND_State.WiFiHWStarted != WIFI_TRUE) {}
}
    
    
/**
* @brief  PowerUp_WiFi_Module
*         Power up Wi-Fi module,SET GPIO PA0 pin 
* @param  None
* @retval None
*/
void PowerUp_WiFi_Module(void)
{
  /* ===   SET PIN - PC12   ===*/
  HAL_GPIO_WritePin(WiFi_RESET_GPIO_PORT, WiFi_RESET_GPIO_PIN, GPIO_PIN_SET);
}


/**
* @brief  Receive_Data
*         Receive data from UART port
* @param  uint8_t number of bytes to be received
* @retval None
*/
void Receive_Data(void)
{
  HAL_GPIO_WritePin(WiFi_USART_RTS_GPIO_PORT, WiFi_USART_RTS_PIN, GPIO_PIN_RESET);//Assert RTS
  receive_status = HAL_UART_Receive_IT(&UartWiFiHandle, (uint8_t *)uart_byte, 1);
  if(receive_status!=HAL_OK)
    {
      #if DEBUG_PRINT
      printf("HAL_UARTx_Receive_IT Error");
      #endif
    }
  else {
    Uartx_Rx_Processing = WIFI_TRUE;
  }
}


/**
* @brief  Period elapsed callback in non blocking mode
*         This timer is used for calling back User registered functions with information
* @param  htim : TIM handle
* @retval None
*/
void Wifi_TIM_Handler(TIM_HandleTypeDef *htim)
{ 
  
  /**********************************************************************
  *                                                                     *
  *       Be careful not to make a blocking                             *
  *       call from this function, see                                  *
  *       example Socket_Read() and Socket_Close()                      *
  *                                                                     *
  **********************************************************************/
   
    {            
            /*If data is pending on client socket SOCKON, make read requests*/
            if(enable_sock_read == WIFI_TRUE)
                {    
                    flush_buffer_queue(&big_buff);    
                    Socket_Read(SockON_Data_Len);              
                    enable_sock_read = WIFI_FALSE;                 
                }
            
            /* Call Query, after notification for TLS is received */
            else if(enable_query == WIFI_TRUE)
                    {
                        //@TBD: Flushing the buffer may be detrimental if we have genuine follow on WIND55?
                        flush_buffer_queue(&big_buff);//Flush the buffer to remove WIND:55 in pipeline (This maybe a problem)
                        Socket_Pending_Data();
                        enable_query = WIFI_FALSE;
                    }
            
            else if(Pending_SockON_Callback==WIFI_TRUE)//for client socket
                    {
                        //Now callback to user with user_data pointer <UserDataBuff>              
                        ind_wifi_socket_data_received((uint8_t *)UserDataBuff, message_size, chunk_size);
                        memset(UserDataBuff, 0x00, MAX_BUFFER_GLOBAL);//Flush the buffer
                        Resume_Dequeue();
                        Pending_SockON_Callback=WIFI_FALSE;
                    }
            
            else if(Pending_SockD_Callback==WIFI_TRUE)//for server socket
                    {      
                        //if(command_mode)//if command_mode is achieved then callback else loop in this state
                        {
                        //Now callback to user with user_data pointer <UserDataBuff>
                        ind_wifi_socket_data_received((uint8_t *)UserDataBuff, message_size, chunk_size);
                        memset(UserDataBuff, 0x00, MAX_BUFFER_GLOBAL);//Flush the buffer
                        Resume_Dequeue();
                        Pending_SockD_Callback=WIFI_FALSE;      
                        }
                    }
            
            else if(Client_Socket_Close_Cmd==WIFI_TRUE)//for client socket
                    {
                        //Close the socket
                        //Change State to AT_Cmd_Response before calling socket_close()
                        WiFi_Module_State = Receive_AT_Cmd_Response;
                        wifi_socket_client_close(remote_socket_closed_id);
                        Client_Socket_Close_Cmd = WIFI_FALSE;
                        SockON_Server_Closed_Callback = WIFI_TRUE;
                    }
            
            else if(SockON_Server_Closed_Callback==WIFI_TRUE)//for client socket
                    {                        
                        //callback the user
                        ind_wifi_socket_client_remote_server_closed(&remote_socket_closed_id);
                        SockON_Server_Closed_Callback = WIFI_FALSE;
                    }
            
            else if(HTTP_Data_available==WIFI_TRUE)
                    {
                        ind_wifi_http_data_available((uint8_t *)UserDataBuff);
                        memset(UserDataBuff, 0x00, MAX_BUFFER_GLOBAL);//Flush the buffer
                        Resume_Dequeue();
                        HTTP_Data_available=WIFI_FALSE;    
                    }
            
            else if(Client_Connected == WIFI_TRUE)
                {
                        ind_socket_server_client_joined();
                        Client_Connected = WIFI_FALSE;
                }
            
            else if(Client_Disconnected == WIFI_TRUE)
                {
                    ind_socket_server_client_left();
                    Client_Disconnected = WIFI_FALSE;
                }
            
            //Make callbacks from here to user for pending events
            
            if(WiFi_WIND_State.WiFiHWStarted==WIFI_TRUE)
                {
                    if(wifi_ready == 2)//Twice reset for User Callback
                        {
                            wifi_ready++;
                            ind_wifi_on();//Call this once only...This if for wifi_on (instead of console active
                        }
                }
            
            if(WiFi_WIND_State.WiFiUp == WIFI_TRUE)
                {
                    if(wifi_connected == 0)
                        {
                            wifi_connected = 1;
                            ind_wifi_connected();//wifi connected
                        }    
                    WiFi_WIND_State.WiFiUp = WIFI_FALSE;
                }
            
            else if(WiFi_WIND_State.WiFiStarted_MiniAPMode == WIFI_TRUE)
                {
                    ind_wifi_ap_ready();
                    WiFi_WIND_State.WiFiStarted_MiniAPMode = WIFI_FALSE;
                }
            
            else if(WiFi_WIND_State.WiFiAPClientJoined == WIFI_TRUE)
                {
                      ind_wifi_ap_client_joined(client_MAC_address);
                      WiFi_WIND_State.WiFiAPClientJoined = WIFI_FALSE;
                }
            
            else if(WiFi_WIND_State.WiFiAPClientLeft == WIFI_TRUE)
                {
                      ind_wifi_ap_client_left(client_MAC_address);
                      WiFi_WIND_State.WiFiAPClientLeft = WIFI_FALSE;
                }
            
            else if(Deep_Sleep_Callback == WIFI_TRUE)
                {
                      ind_wifi_resuming();
                      Deep_Sleep_Callback = WIFI_FALSE;
                }
            
            else if(standby_resume_callback == WIFI_TRUE)
                {
                      ind_wifi_resuming();
                      standby_resume_callback = WIFI_FALSE;
                }
            
            else if(WiFi_WIND_State.WiFiHWFailure==WIFI_TRUE)
                {      
                      WiFi_WIND_State.WiFiHWFailure=WIFI_FALSE;
                      ind_wifi_error(WiFi_HW_FAILURE_ERROR);//call with error number      
                }
            
            else if(WiFi_WIND_State.HardFault==WIFI_TRUE)
                {      
                      WiFi_WIND_State.HardFault=WIFI_FALSE;
                      ind_wifi_error(WiFi_HARD_FAULT_ERROR);//call with error number      
                }
            
            else if(WiFi_WIND_State.StackOverflow==WIFI_TRUE)
                {      
                      WiFi_WIND_State.StackOverflow=WIFI_FALSE;
                      ind_wifi_error(WiFi_STACK_OVERFLOW_ERROR);//call with error number      
                }
            
            else if(WiFi_WIND_State.MallocFailed==WIFI_TRUE)
                {      
                      WiFi_WIND_State.MallocFailed=WIFI_FALSE;
                      ind_wifi_error(WiFi_MALLOC_FAILED_ERROR);//call with error number      
                }
            
            else if(WiFi_WIND_State.InitFailure==WIFI_TRUE)
                {      
                      WiFi_WIND_State.InitFailure=WIFI_FALSE;
                      ind_wifi_error(WiFi_INIT_ERROR);//call with error number      
                }
            
            else if(WiFi_WIND_State.StartFailed==WIFI_TRUE)
                {      
                      WiFi_WIND_State.StartFailed=WIFI_FALSE;
                      ind_wifi_error(WiFi_START_FAILED_ERROR);//call with error number      
                }
            
            else if(WiFi_WIND_State.WiFiException==WIFI_TRUE)
                {      
                      WiFi_WIND_State.WiFiException=WIFI_FALSE;
                      ind_wifi_error(WiFi_EXCEPTION_ERROR);//call with error number      
                }
            
            else if(WiFi_WIND_State.PS_Mode_Failure==WIFI_TRUE)
                {      
                      WiFi_WIND_State.PS_Mode_Failure=WIFI_FALSE;
                      ind_wifi_warning(WiFi_POWER_SAVE_WARNING);//call with error number      
                }
            
            else if(WiFi_WIND_State.HeapTooSmall==WIFI_TRUE)
                {      
                      WiFi_WIND_State.HeapTooSmall=WIFI_FALSE;
                      ind_wifi_warning(WiFi_HEAP_TOO_SMALL_WARNING);//call with error number      
                }
            
            else if(WiFi_WIND_State.WiFiSignalLOW==WIFI_TRUE)
                {      
                      WiFi_WIND_State.WiFiSignalLOW=WIFI_FALSE;
                      ind_wifi_warning(WiFi_SIGNAL_LOW_WARNING);//call with error number      
                }    
            
            else if(WiFi_WIND_State.WiFiDeauthentication == WIFI_TRUE)
                {
                      WiFi_WIND_State.WiFiDeauthentication = WIFI_FALSE;
                      ind_wifi_connection_error(WiFi_DE_AUTH);
                }
            
            else if(WiFi_WIND_State.WiFiDisAssociation == WIFI_TRUE)
                {
                      WiFi_WIND_State.WiFiDisAssociation = WIFI_FALSE;
                      ind_wifi_connection_error(WiFi_DISASSOCIATION);
                }
            
            else if(WiFi_WIND_State.WiFiJoinFailed == WIFI_TRUE)
                {
                      WiFi_WIND_State.WiFiJoinFailed = WIFI_FALSE;
                      ind_wifi_connection_error(WiFi_JOIN_FAILED);
                }
            
            else if(WiFi_WIND_State.WiFiScanBlewUp == WIFI_TRUE)
                {
                      WiFi_WIND_State.WiFiScanBlewUp = WIFI_FALSE;
                      ind_wifi_connection_error(WiFi_SCAN_BLEWUP);  //@TBD to check if user made call, so not call callback if true
                }
            
            else if(WiFi_WIND_State.WiFiScanFailed == WIFI_TRUE)
                {
                      WiFi_WIND_State.WiFiScanFailed = WIFI_FALSE;
                      ind_wifi_connection_error(WiFi_SCAN_FAILED);  //@TBD to check if user made call, so not call callback if true
                }
            
            else if(WiFi_WIND_State.WiFiUnHandledInd == WIFI_TRUE)
                {
                      WiFi_WIND_State.WiFiUnHandledInd = WIFI_FALSE;
                      ind_wifi_packet_lost(WiFi_UNHANDLED_IND_ERROR);  //@TBD to check if user made call, so not call callback if true
                }
            
            else if(WiFi_WIND_State.WiFiRXMgmt == WIFI_TRUE)
                {
                      WiFi_WIND_State.WiFiRXMgmt = WIFI_FALSE;
                      ind_wifi_packet_lost(WiFi_RX_MGMT);  //@TBD to check if user made call, so not call callback if true
                }
            
            else if(WiFi_WIND_State.WiFiRXData == WIFI_TRUE)
                {
                      WiFi_WIND_State.WiFiRXData = WIFI_FALSE;
                      ind_wifi_packet_lost(WiFi_RX_DATA);  //@TBD to check if user made call, so not call callback if true
                }  
            
            else if(WiFi_WIND_State.WiFiRxUnk == WIFI_TRUE)
                {
                      WiFi_WIND_State.WiFiRxUnk = WIFI_FALSE;
                      ind_wifi_packet_lost(WiFi_RX_UNK);  //@TBD to check if user made call, so not call callback if true
                }  
            
            else if(WiFi_WIND_State.WiFiSockdDataLost == WIFI_TRUE)
                {
                      WiFi_WIND_State.WiFiSockdDataLost = WIFI_FALSE;
                      ind_wifi_socket_server_data_lost();  //@TBD to check if user made call, so not call callback if true
                }
      }
}





/**
* @brief  Start_Timer
*         Start Timer 
* @param  None
* @retval None
*/
void Start_Timer()
{
  tickcount = WIFI_FALSE;
  Timer_Running = WIFI_TRUE;
}

/**
* @brief  Stop_Timer
*         Stop Timer request
* @param  None
* @retval None
*/
void Stop_Timer()
{  
  tickcount = WIFI_FALSE;  
  Timer_Running = WIFI_FALSE;    
  UartReady = SET;
}

/**
* @brief  Stop_Dequeue
*         Stop dequeuing data from the ring buffer
* @param  None
* @retval None
*/
void Stop_Dequeue()
{
  enable_dequeue = WIFI_FALSE;
}

/**
* @brief  Resume_Dequeue
*         Resume dequeuing data from the ring buffer
* @param  None
* @retval None
*/
void Resume_Dequeue()
{
  enable_dequeue = WIFI_TRUE;
}

/**
* @brief  Wifi_SysTick_Isr
*         Function called every SysTick to process buffer
* @param  None
* @retval None
*/
void Wifi_SysTick_Isr()
{
    //Check if Data is Paused
    if((Timer_Running) && (enable_dequeue==WIFI_TRUE) /*&& ((tickcount++) >= PROCESS_WIFI_TIMER)*/)
        {    
            Process_WiFi();
        }

    if(resume_receive_data == WIFI_TRUE)
        {
            if(is_half_empty(&big_buff))
                {
                    resume_receive_data = WIFI_FALSE;
                    Receive_Data();
                }
        }
    
 
    if(Standby_Timer_Running) // module is in sleep and after expiry RX will be conf as EXTI
    {
        if((standby_time++) >= EXTI_CONF_TIMER)
            {
                Standby_Timer_Running=WIFI_FALSE;
                standby_time = 0;
                //configure_to_exti();
            }
    }

    /*A Resume WIND:70 has come and triggered this
    So checking here if after that resume we fall back to sleep (another WIND69) within SLEEP_RESUME_PREVENT time.
    If yes, we assume it is a false resume and hence do nothing and go back to sleep
    If no WIND69 (going into sleep) has come, we can assume the resume was genuine and then enable the callback
    */
    if((Deep_Sleep_Timer) && ( sleep_count++) >= SLEEP_RESUME_PREVENT)
        {
            if(Deep_Sleep_Enabled == WIFI_TRUE)//which means we have received another WIND69 in the 2 seconds
                {
                    //do nothing, go back to sleep
                    Deep_Sleep_Enabled = WIFI_TRUE;
                    Deep_Sleep_Callback = WIFI_FALSE;
                }
    else if (Deep_Sleep_Enabled == WIFI_FALSE) //which means we have not received any WIND69 during the last 2 seconds
        {
            //enable the user callback as it is a genuine WIND70
            Deep_Sleep_Callback = WIFI_TRUE;
        }
    Stop_DeepSleep_Timer();
    }
}

#if 0
/**
* @brief  RX_EXTI_Isr
*         
* @param  None
* @retval None
*/
void RX_EXTI_Isr(uint16_t GPIO_Pin)
{
    
  if(GPIO_Pin==WiFi_USART_RX_PIN && (HAL_GPIO_ReadPin(WiFi_USART_RX_GPIO_PORT, WiFi_USART_RX_PIN) == GPIO_PIN_SET))
  {
    HAL_NVIC_DisableIRQ(USARTx_EXTI_IRQn);
    UART_Configuration();//reconfigure the UART for WIND reception
    
    //Resume_Timer_Running = WIFI_TRUE;
    AT_Cmd_Processing = WIFI_FALSE;
    Receive_Data();
  }
  
}
#endif

/**
* @brief  HAL_UART_TxCpltCallback
*         Tx Transfer completed callback
* @param  UsartHandle: UART handle 
* @retval None
*/
void HAL_UART_TxCpltCallback(UART_HandleTypeDef *UartHandleArg)
{
#ifdef WIFI_USE_VCOM
  if (UartHandleArg==&UartMsgHandle)
    console_echo_ready = SET;
#else
  /* Set transmission flag: transfer complete */
  TxUartReady = SET; 
#endif
}

/**
* @brief  HAL_UART_RxCpltCallback
*         Rx Transfer completed callback
* @param  UsartHandle: UART handle 
* @retval None
*/
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *UartHandleArg)
{ 
  
#ifdef WIFI_USE_VCOM
  if (UartHandleArg==&UartWiFiHandle)
#endif  
  {
#ifndef WIFI_USE_VCOM
  Uartx_Rx_Processing = WIFI_FALSE;
  Stop_Timer();
  __disable_irq();
  push_buffer(&big_buff, uart_byte);
  __enable_irq();
  Start_Timer();
#else
    console_push_ready = SET;
#endif

#ifndef WIFI_USE_VCOM  
  if(is_half_full(&big_buff))
  {
    resume_receive_data = WIFI_TRUE;
    HAL_GPIO_WritePin(WiFi_USART_RTS_GPIO_PORT, WiFi_USART_RTS_PIN, GPIO_PIN_SET);//De-assert RTS
  } else
    {
      if(AT_Cmd_Processing == WIFI_FALSE)
      {//call Rx only if TX is not under processing (AT command)
        receive_status = HAL_UART_Receive_IT(&UartWiFiHandle, (uint8_t *)uart_byte, 1);
        if(receive_status!=HAL_OK)
        {
          #if DEBUG_PRINT 
          printf("HAL_UARTx_Receive_IT Error");
          #endif
        }
        else {
          Uartx_Rx_Processing = WIFI_TRUE;
        }
      }  //else if (AT_Cmd_Processing == WIFI_TRUE)
    }
#endif  
  }
#ifdef WIFI_USE_VCOM
  else 
  {
    console_send_char[0] = console_input_char[0];
    console_input();
    console_send_ready = SET;
    HAL_UART_Transmit_IT(&UartWiFiHandle, (uint8_t*)console_send_char, 1);
  }
#endif
  
// if (UartHandleArg->Instance == USART2)
 {
 //  Uart2Ready = SET;
 }
    
}
  
/**
* @brief  USART_Receive_AT_Resp
*         Receive and check AT cmd response
* @param  WiFi_AT_CMD_Response_t : WIFi module next state 
* @retval WiFi_Status_t : Response of AT cmd  
*/

WiFi_Status_t USART_Receive_AT_Resp(WiFi_AT_CMD_Response_t state)
{

  WiFi_Module_State = state;
  while(AT_Response_Received != WIFI_TRUE) {
		//do nothing
	}
  AT_Response_Received = WIFI_FALSE;
  return AT_RESPONSE;    
  
}

  
/**
  * @brief  UART error callbacks
  * @param  UsartHandle: UART handle
  * @note   This example shows a simple way to report transfer error, and you can
  *         add your own implementation.
  * @retval None
  */
void HAL_UART_ErrorCallback(UART_HandleTypeDef *UartHandle)
{
    Error_Handler();
    Receive_Data();
}

/**
* @brief  Process_WiFi
*         Pop a byte from the circular buffer and send the byte for processing
*         This function should be called from main or should be run with a periodic timer
* @param  None
* @retval None
*/
void Process_WiFi(void)
{
    __disable_irq();
    temp=pop_buffer_queue(&big_buff);//contents of temp(pop_buffer) will not change till another de-queue is made
    __enable_irq();
    
    if(temp!=NULL) 
    {
      Process_Buffer(temp);
    }
}   


/**
* @brief  Process_Buffer
*         Process and construct a Wind Line buffer
* @param  ptr: pointer to one single byte
* @retval None
*/
void Process_Buffer(uint8_t * ptr)
{  
  static uint8_t startdetected=0;
  static uint32_t Fillptr=0, httpptr=0,count = 0;
  static uint8_t LineFeed = 0, index, chan_value;
  unsigned char rxdata = 0;
  int rssi_value;
  char SocketId_No[2];
  char databytes_No[4];
  char * pStr;
  
  rxdata =  *(ptr+0);  
  //printf(&rxdata);//check prints for debug...to be removed or kept in DEBUG statement
  if(WiFi_Module_State != Receive_SockD_Data ||
     WiFi_Module_State != Receive_SockON_Data)
    process_buffer[Fillptr++] = rxdata;
    switch (WiFi_Module_State)
    {
    case Receive_Indication:

      if(startdetected==0)
      {      
        if(rxdata == 0xD) // \r
        {
          LineFeed = 1;
        }
        if((LineFeed == 1) && (rxdata==0xA))// \n
        {
          startdetected=1;
        }
      }      
      else //if(startdetected==1)
      {
        
        
        /*now check if end of msg received*/
        if((process_buffer[Fillptr-2]==0xD) && (process_buffer[Fillptr-1]==0xA))
        {
          //end of msg received. Will not receive any other msg till we process this.
          startdetected=0;
          Fillptr=0;
          
          Stop_Timer();
          Process_WiFi_Indication_Cmd(&process_buffer[0]);
          Start_Timer();
          
        }
      }      
      break;
      
    case Receive_HTTP_Response:  //HTTP_Data      
      
      //Fill the HTTP User Buffer
      UserDataBuff[httpptr++]=rxdata;      

      if((((strstr((const char *)UserDataBuff,"\r\nOK\r"))) != NULL))
      {
        #if DEBUG_PRINT
        printf("\rOK\r\n");
        #endif
        AT_Response_Received = WIFI_TRUE;
        httpptr=0;
        Stop_Dequeue();
        flush_buffer_queue(&big_buff);
        HTTP_Data_available=WIFI_TRUE;
        WiFi_Module_State = Receive_Indication;
        AT_RESPONSE = WiFi_MODULE_SUCCESS;  

        startdetected=0;
        LineFeed =0;
        Fillptr=0;
        memset(process_buffer, 0x00, MAX_BUFFER_GLOBAL);        
      }
      else if(((strstr((const char *)UserDataBuff,"ERROR"))) != NULL)
      {
        #if DEBUG_PRINT
        printf("\r\nERROR\r\n");
        #endif    
        AT_Response_Received = WIFI_TRUE;
        httpptr=0;
        HTTP_Data_available=WIFI_FALSE;
        WiFi_Module_State = Receive_Indication;
        AT_RESPONSE = WiFi_AT_CMD_RESP_ERROR;
        startdetected=0;
        LineFeed =0;
        Fillptr=0;
        memset(process_buffer, 0x00, MAX_BUFFER_GLOBAL);       
      }      
      
      if(httpptr>=MAX_BUFFER_GLOBAL)
      {
        httpptr=0;
        Stop_Dequeue();
        HTTP_Data_available=WIFI_TRUE;
      }
      
      if(Fillptr>=MAX_BUFFER_GLOBAL)
      {
        startdetected=0;
        LineFeed =0;
        Fillptr=0;
      }
      
      break;
      
    case Receive_AT_Cmd_Data:  //Create_File
        printf((const char *)&rxdata);
        if((process_buffer[Fillptr-2]==0xD) && (process_buffer[Fillptr-1]==0xA))
        { 
          if((((strstr((const char *)process_buffer,"\r\nOK\r"))) != NULL))
          {
              #if DEBUG_PRINT
              printf("\rOK\r\n");
              #endif
              AT_Response_Received = WIFI_TRUE;
              WiFi_Module_State = Receive_Indication;
              AT_RESPONSE = WiFi_MODULE_SUCCESS;  
              startdetected=0;
              LineFeed =0;
              Fillptr=0;
              memset(process_buffer, 0x00, MAX_BUFFER_GLOBAL);        
          }
          else if(((strstr((const char *)process_buffer,"ERROR"))) != NULL)
          {
              #if DEBUG_PRINT
              printf("\r\nERROR\r\n");
              #endif    
              AT_Response_Received = WIFI_TRUE;
              //dataptr=0;
              WiFi_Module_State = Receive_Indication;
              AT_RESPONSE = WiFi_AT_CMD_RESP_ERROR;
              startdetected=0;
              LineFeed =0;
              Fillptr=0;
              memset(process_buffer, 0x00, MAX_BUFFER_GLOBAL);       
          }
        }
      
      if(Fillptr>=MAX_BUFFER_GLOBAL)
      {
        startdetected=0;
        LineFeed =0;
        Fillptr=0;
      }
      break;
      
    case Receive_WiFi_Scan_Response:  

        /*now check if end of msg received*/
        if((process_buffer[Fillptr-2]==0xD) && (process_buffer[Fillptr-1]==0xA))
        {       
          if(scanned_ssids < user_scan_number)
            {
                pStr = (char *) strstr((const char *)&process_buffer,"CHAN:");            
                if(pStr != NULL)
                    {
                        databytes_No[0] = *(pStr + 6) ;
                        databytes_No[1] = *(pStr + 7) ;
              
                        chan_value = (((databytes_No[0] - '0') * 10 ) + (databytes_No[1] - '0'));
                    }
                
                wifi_scanned_list[scanned_ssids].channel_num = chan_value;
                
                pStr = (char *) strstr((const char *)&process_buffer,"RSSI:");            
                if(pStr != NULL)
                    {
                        databytes_No[0] = *(pStr + 7) ;
                        databytes_No[1] = *(pStr + 8) ;
              
                        rssi_value = (((databytes_No[0] - '0') * 10 ) + (databytes_No[1] - '0'));
                    }
                
                wifi_scanned_list[scanned_ssids].rssi = -(rssi_value);
                
                pStr = (char *) strstr((const char *)&process_buffer,"SSID:");
                if(pStr != NULL)
                    {
                        index = 7;
                        while(*(pStr + index) != 0x27)
                            {
                                wifi_scanned_list[scanned_ssids].ssid[index-7] = *(pStr + index);
                                index++;
                                if(index==35) break; //max ssid lenght is 30 characters
                            }                                
                    }
                
                pStr = (char *) strstr((const char *)&process_buffer,"WPA ");            
                if(pStr != NULL)
                    {
                        wifi_scanned_list[scanned_ssids].sec_type.wpa = WIFI_TRUE;
                    } else
                        wifi_scanned_list[scanned_ssids].sec_type.wpa = WIFI_FALSE;
                
                pStr = (char *) strstr((const char *)&process_buffer,"WPA2 ");            
                if(pStr != NULL)
                    {
                        wifi_scanned_list[scanned_ssids].sec_type.wpa2 = WIFI_TRUE;
                    } else
                        wifi_scanned_list[scanned_ssids].sec_type.wpa2 = WIFI_FALSE;
                
                pStr = (char *) strstr((const char *)&process_buffer,"WPS ");            
                if(pStr != NULL)
                    {
                        wifi_scanned_list[scanned_ssids].sec_type.wps = WIFI_TRUE;
                    } else
                        wifi_scanned_list[scanned_ssids].sec_type.wps = WIFI_FALSE;
                
                
                scanned_ssids++;//increment total_networks
            }
          
          //end of one line from SCAN result       
          pStr = (char *) strstr((const char *)&process_buffer,"ERROR");
          if(pStr != NULL)
          {
           #if DEBUG_PRINT
           printf("ERROR Scan Failed"); 
           #endif
           memset(process_buffer, 0x00, MAX_BUFFER_GLOBAL);
           startdetected=0;
           LineFeed =0;
           Fillptr=0;
           count=0;
           AT_Response_Received = WIFI_TRUE;
           Scan_Ongoing = WIFI_FALSE; //Enable next scan
           WiFi_Module_State = Receive_Indication; 
           AT_RESPONSE = WiFi_AT_CMD_RESP_ERROR;     
           break;
          }
            
          #if DEBUG_PRINT
          printf((const char*)process_buffer);
          #endif
          
          if(((strstr((const char *)process_buffer,"OK\r\n"))) != NULL /*|| scanned_ssids==10*/)/*Max 10 networks supported*/
          {
            //print and go for next line
            //If Any part of scan line contains "OK" this will exit!!
            #if DEBUG_PRINT
            //printf("\nOK\r\n");   
            #endif            
            Scan_Ongoing = WIFI_FALSE; //Enable next scan             
            scanned_ssids=0;
            count=0;                        
            startdetected=0;
            LineFeed =0;
            Fillptr=0;
            AT_Response_Received = WIFI_TRUE;
            AT_RESPONSE = WiFi_MODULE_SUCCESS;
            WiFi_Module_State = Receive_Indication;
            memset(process_buffer, 0x00, MAX_BUFFER_GLOBAL);    
            break;
          }   
          
          startdetected=0;
          LineFeed =0;
          Fillptr=0;
          memset(process_buffer, 0x00, MAX_BUFFER_GLOBAL);          
        }
        
        if(Fillptr>=MAX_BUFFER_GLOBAL-1)
        {
          #if DEBUG_PRINT
          printf("\rHTTP: process_buffer Max Buffer Size reached\r\n");
          #endif          
          startdetected=0;
          LineFeed =0;
          Fillptr=0;          
          memset(process_buffer, 0x00, MAX_BUFFER_GLOBAL); 
        }
      
      break;
    case Receive_AT_Cmd_Response:            
        
        if((process_buffer[Fillptr-2]==0xD) && (process_buffer[Fillptr-1]==0xA))
        {
          //end of msg received. Will not receive any other msg till we process this.          
          //printf(process_buffer);
          
          if(((strstr((const char *)process_buffer,"OK"))) != NULL)
          {
            //print and go for next line
            //If Any part of scan line contains "OK" this will exit!!
            //printf("\nOK\r\n");
            AT_Response_Received = WIFI_TRUE;
            WiFi_Module_State = Receive_Indication; 
            AT_RESPONSE = WiFi_MODULE_SUCCESS;      
            if(Set_AT_Cmd_Response_False == WIFI_TRUE)
            {
              Set_AT_Cmd_Response_False = WIFI_FALSE;
              AT_Response_Received = WIFI_FALSE;
            }
          }
          if(((strstr((const char *)process_buffer,"ERROR"))) != NULL)
          {            
            AT_Response_Received = WIFI_TRUE;
            WiFi_Module_State = Receive_Indication; 
            AT_RESPONSE = WiFi_AT_CMD_RESP_ERROR;            
            if(Set_AT_Cmd_Response_False == WIFI_TRUE)
            {
              Set_AT_Cmd_Response_False = WIFI_FALSE;
              AT_Response_Received = WIFI_FALSE;
            }
          }          
          
          startdetected=0;
          LineFeed =0;
          Fillptr=0;
          memset(process_buffer, 0x00, MAX_BUFFER_GLOBAL);
          
        }      
       
       break;
    case Receive_SockD_Data:
      if(!data_mode) break;//if data_mode is not achieved till now wait and loop in the same state
      //checking here <\r\n +WIND:xxx>\r\n\r\n<Data>   
      
        UserDataBuff[count]=rxdata;          
        sockD_total_count++;
        count++;

        if(count == MAX_BUFFER_GLOBAL) {      
            chunk_size = count;
            message_size = number_of_bytes;
            count=0;
            Stop_Dequeue();
            Pending_SockON_Callback = WIFI_TRUE;//set this to callback to user with User Buffer pointer
        }
      
      if(sockD_total_count==number_of_bytes)//read number of expected bytes
      {
        chunk_size = count;
        message_size = number_of_bytes;
        
        startdetected=0;
        LineFeed =0;
        Fillptr=0;
        count=0;
        number_of_bytes=0;
        sockD_total_count=0;
        Stop_Dequeue();        
        data_pending_sockD=WIFI_FALSE;
        WiFi_Module_State = Receive_Indication;
        switch_by_default_to_command_mode=WIFI_TRUE;
        WiFi_switch_to_command_mode();//switch by default        
        Pending_SockD_Callback = WIFI_TRUE;//set this to callback to user with User Buffer pointer
        memset(process_buffer, 0x00, MAX_BUFFER_GLOBAL);
        break;
      }

      if(Fillptr>=MAX_BUFFER_GLOBAL)
      {
        startdetected=0;
        LineFeed =0;
        Fillptr=0;
        //memset(process_buffer, 0x00, MAX_BUFFER_GLOBAL); 
      }                                                           
      break;
      
    case Receive_SockON_ID:
      if(startdetected==0)
      {      
        if(rxdata == 0xD) // \r
        {
          LineFeed = 1;
        }
        if((LineFeed == 1) && (rxdata==0xA))// \n
        {
          startdetected=1;
        }
      }      
      else //if(startdetected==1)
      {
        if((process_buffer[Fillptr-2]==0xD) && (process_buffer[Fillptr-1]==0xA))
        { 
          pStr = (char *) strstr((const char *)&process_buffer,"ERROR:");
          
          if(pStr != NULL)
          {
            #if DEBUG_PRINT
            printf("ERROR: Failed to connect"); 
            #endif
           memset(process_buffer, 0x00, MAX_BUFFER_GLOBAL);
           startdetected=0;
           LineFeed =0;
           Fillptr=0;
           AT_Response_Received = WIFI_TRUE;
           WiFi_Module_State = Receive_Indication; 
           AT_RESPONSE = WiFi_AT_CMD_RESP_ERROR;     
           break;
          }

          pStr = (char *) strstr((const char *)&process_buffer,"ID: ");
          SocketId_No[0] = *(pStr + 4) ;
          SocketId_No[1] = *(pStr + 5) ;
          
          SocketId = (((SocketId_No[0] - '0') * 10 ) + (SocketId_No[1] - '0'));
          //new socket opened
          no_of_open_client_sockets++;
          if(no_of_open_client_sockets>8)//Max number of clients is 8
          {
            memset(process_buffer, 0x00, MAX_BUFFER_GLOBAL);
            startdetected=0;
            LineFeed =0;
            Fillptr=0;
            AT_Response_Received = WIFI_TRUE;
            WiFi_Module_State = Receive_Indication; 
            AT_RESPONSE = WiFi_NOT_SUPPORTED;     
            break;
          }
          open_sockets[SocketId] = WIFI_TRUE;
          memset(process_buffer, 0x00, MAX_BUFFER_GLOBAL);
          startdetected=0;
          LineFeed =0;
          Fillptr=0;

          WiFi_Module_State = Receive_AT_Cmd_Response;
        }
      }
      break;
      
    case Receive_Lenght_p:
      
      if((process_buffer[Fillptr-2]==0xD) && (process_buffer[Fillptr-1]==0xA))
      {           

        if (SockON_Data_Len!=0) // EQ. Process OK after DATALEN > 0 is found. 
        {
          if(((strstr((const char *)process_buffer,"OK"))) != NULL)
          {
            memset(process_buffer, 0x00, MAX_BUFFER_GLOBAL);
            startdetected=0;
            LineFeed =0;
            Fillptr=0;
            
            /*At this point, there might be more WIND:55 in the pipeline 
              if data exceeds 4 IP packets.*/
            // EQ. Found OK. Set enable_sock_read
            enable_sock_read = WIFI_TRUE;            
            
          }
          break;				
        }
        
        /* EQ. Check for DATALEN. SockON_Data_Len is 0 */
        pStr = (char *) strstr((const char *)&process_buffer,"DATALEN: ");
        
        if (pStr == NULL)
        {   
          /* Not found DATALEN, but SockON_Data_Len is still 0. Stay in this state */
          memset(process_buffer, 0x00, MAX_BUFFER_GLOBAL);
          startdetected=0;
          LineFeed =0;
          Fillptr=0;
        }
        else
        {  
          /* DATALENGHT found.  */  
          databytes_No[0] = *(pStr + 9);
          databytes_No[1] = *(pStr + 10);
          databytes_No[2] = *(pStr + 11);
          databytes_No[3] = *(pStr + 12);
          
          if( databytes_No[1] == '\r')
          {      
            SockON_Data_Len = databytes_No[0] - '0'; 
          }
          else if( databytes_No[2] == '\r')
          {
            SockON_Data_Len = (((databytes_No[0] - '0') * 10 ) + (databytes_No[1] - '0'));
          }
          else if( databytes_No[3] == '\r')
          {
            SockON_Data_Len = (((databytes_No[0] - '0') * 100 ) + ((databytes_No[1] - '0') * 10 ) + (databytes_No[2] - '0'));
          }
          else //it's a 4-digit number
          {
            SockON_Data_Len = ((databytes_No[0] - '0') * 1000 ) + ((databytes_No[1] - '0') * 100 ) + ((databytes_No[2] - '0') * 10) + (databytes_No[3] - '0');
          }
          
          if(SockON_Data_Len != 0){
            memset(process_buffer, 0x00, MAX_BUFFER_GLOBAL);
            startdetected=0;
            LineFeed =0;
            Fillptr=0;
          }
          else 
          {
            /* EQ. We have datalenght but it is 0. */
            /* No need to do a read after the query. Go to Receive Indication */
            /* We go to Receive AT Cmd and then to Receive Indication */
            WiFi_Module_State = Receive_AT_Cmd_Response;
          }
        }
      }
      break;
	  
    case Receive_SockON_Data:       
    
    /*Check for "ERROR:Not enough data in buffer"*/
    pStr = (char *) strstr((const char *)&process_buffer,"ERROR: ");
    
    if (pStr != NULL)
            { 
                  startdetected=0;
                  LineFeed =0;
                  Fillptr=0;
                  count=0;
                  ip_fragment_count=0;
                  sock_total_count=0;
                  SockON_Data_Len=0;          
                  WiFi_Module_State = Receive_Indication;
                  memset(process_buffer, 0x00, MAX_BUFFER_GLOBAL);
                  break;       
            }
    
    UserDataBuff[count]=rxdata;   
    sock_total_count++;
    ip_fragment_count++;               
    count++;
            
    if(count == MAX_BUFFER_GLOBAL) {      
        chunk_size = count;
        message_size = SockON_Data_Len;
        count=0;
        Stop_Dequeue();
        Pending_SockON_Callback = WIFI_TRUE;//set this to callback to user with User Buffer pointer
    }
        
      /*now check if end of msg received*/
      if(sock_total_count==(SockON_Data_Len))//read number of expected bytes
      {                   
#if DEBUG_PRINT
        printf("\nReached SockON_Data_Len \r\n");
#endif
        chunk_size = count;
        message_size = SockON_Data_Len;
        startdetected=0;
        LineFeed =0;
        Fillptr=0;
        count=0;
        ip_fragment_count=0;
        sock_total_count=0;
        SockON_Data_Len=0;
        Stop_Dequeue();//Stop dequeue till user callback returns
        Pending_SockON_Callback = WIFI_TRUE;//set this to callback to user with User Buffer pointer
        WIND55_count = 0;
        WiFi_Module_State = Receive_AT_Cmd_Response;
        enable_query = WIFI_TRUE;//do we have more data?
        memset(process_buffer, 0x00, MAX_BUFFER_GLOBAL);
        break;       
      }
      
      if(Fillptr>=MAX_BUFFER_GLOBAL)
      {
        startdetected=0;
        LineFeed =0;
        Fillptr=0;
        //memset(process_buffer, 0x00, MAX_BUFFER_GLOBAL); 
      }     
      
      break;
      
    case Receive_GPIO_Read:
      if((process_buffer[Fillptr-2]==0xD) && (process_buffer[Fillptr-1]==0xA))
        {  
          char *pStr;
          #if DEBUG_PRINT
          printf((const char*)process_buffer);
          #endif
          
          pStr = (char *) strstr((const char *)process_buffer,"= 0,");
          
          if(pStr != NULL)
          {
            gpio_value = 0;
          } else gpio_value = 1;    
          
          pStr = (char *) strstr((const char *)process_buffer,"out");
          
          if(pStr != NULL)
          {    
            gpio_dir=0;//out
          }
          else
          {    
            gpio_dir=1;//in
          }          
          
          startdetected=0;
          LineFeed =0;
          Fillptr=0;
          memset(process_buffer, 0x00, MAX_BUFFER_GLOBAL);
          WiFi_Module_State = Receive_AT_Cmd_Response;
        }
      break;
    case Receive_FOTA_Update:
      if((process_buffer[Fillptr-2]==0xD) && (process_buffer[Fillptr-1]==0xA))
        {  
          char *pStr;
          #if DEBUG_PRINT
          printf("*");     //print * till finish    
          #endif
          
          pStr = (char *) strstr((const char *)process_buffer,"Complete!");
          
          if(pStr != NULL)
          {
            #if DEBUG_PRINT
            printf("\r\nUpdate complete\r\n");
            #endif
            AT_Response_Received = WIFI_TRUE;
            //Now change the state
            WiFi_Module_State = Receive_Indication;
          }    
          
          startdetected=0;
          LineFeed =0;
          Fillptr=0;
          memset(process_buffer, 0x00, MAX_BUFFER_GLOBAL);
          //No change of state till we get "+WIND:17:F/W update complete!"
        }
      break;
    case Receive_Standby_Config:
      if((process_buffer[Fillptr-2]==0xD) && (process_buffer[Fillptr-1]==0xA))
      {  
        char *pStr;
        #if DEBUG_PRINT
        printf((const char*)process_buffer);     //print * till finish    
        #endif
        
        pStr = (char *) strstr((const char *)process_buffer,"+WIND:67:");
        
        if(pStr != NULL)
        {
          #if DEBUG_PRINT
          printf("\r\nGoing into standby..\r\n");  
          #endif
          
          WiFi_Module_State = Receive_Resume_Config;
        }    
        
        startdetected=0;
        LineFeed =0;
        Fillptr=0;
        memset(process_buffer, 0x00, MAX_BUFFER_GLOBAL);
        
      }
      break;   
    case Receive_Resume_Config:
      if((process_buffer[Fillptr-2]==0xD) && (process_buffer[Fillptr-1]==0xA))
      {  
        char *pStr;
        #if DEBUG_PRINT
        printf((const char*)process_buffer);     //print * till finish    
        #endif
        
        pStr = (char *) strstr((const char *)process_buffer,"+WIND:68:");
        
        if(pStr != NULL)
        {
          #if DEBUG_PRINT
          printf("\r\nResuming from standby..\r\n");   
          #endif
          AT_Response_Received = WIFI_TRUE;//let main run-on
          WiFi_Module_State = Receive_Indication;
        }    
        
        startdetected=0;
        LineFeed =0;
        Fillptr=0;
        memset(process_buffer, 0x00, MAX_BUFFER_GLOBAL);
        
      }
      break;
      
    case Receive_TLSCERT_Response: 
      
      if(Fillptr==127)//end of process_buffer size reached, so print the line and reset it
      {
        #if DEBUG_PRINT
        printf((const char*)process_buffer);
        #endif
        startdetected=0;
        LineFeed =0;
        Fillptr=0;
        memset(process_buffer, 0x00, MAX_BUFFER_GLOBAL);
        break;
      }

      if((process_buffer[Fillptr-2]==0xD) && (process_buffer[Fillptr-1]==0xA)) //<cr><lf> found
      {
        //end of msg received. Will not receive any other msg till we process this.          
        #if DEBUG_PRINT
        printf((const char*)process_buffer);
        #endif
        
        if(((strstr((const char *)process_buffer,"OK"))) != NULL)//get till the end of </html> tag
        {
          #if DEBUG_PRINT
          printf("\nTLS_CERT OK\r\n");   
          #endif
          AT_Response_Received = WIFI_TRUE;
          WiFi_Module_State = Receive_Indication;    
        }
        
        startdetected=0;
        LineFeed =0;
        Fillptr=0;
        memset(process_buffer, 0x00, MAX_BUFFER_GLOBAL);
        
      }
      break;
    case Receive_Get_Cfg_Response:
      if((process_buffer[Fillptr-2]==0xD) && (process_buffer[Fillptr-1]==0xA))
      {     
        //printf(process_buffer);
        
        pStr = (char *) strstr((const char *)process_buffer,"= ");
          
        if(pStr != NULL)
        {
          memcpy(get_cfg_value, pStr+2, (strlen((pStr))-2));
        }
          
        if(((strstr((const char *)process_buffer,"OK"))) != NULL)
        {
          AT_Response_Received = WIFI_TRUE;
          WiFi_Module_State = Receive_Indication; 
          AT_RESPONSE = WiFi_MODULE_SUCCESS;
        }
        
        startdetected=0;
        LineFeed =0;
        Fillptr=0;
        memset(process_buffer, 0x00, MAX_BUFFER_GLOBAL);
      }
      break;

    case Undefine_State:  //Create_File      
      break;
      
    default:
      break;
    }  

  }   
  
            
/**
* @brief  Process_WiFi_Indication_Cmd
*         Process WiFi indication command
* @param  pStr1: pointer of WiFi indication buffer
* @retval None
*/
void Process_WiFi_Indication_Cmd(uint8_t *process_buff_ptr)
{
   char * pStr = (char*)process_buff_ptr;
  char Indication_No[2]; 
  char databytes_No[4]; 
  WiFi_Indication_t Wind_No = Undefine_Indication;
  #if DEBUG_PRINT
  printf((const char*)process_buff_ptr);
  #endif
  char * ptr_offset;
  uint8_t i=0;
  
  if(pStr != NULL)
  {
  pStr = (char *) strstr((const char *)(pStr),"WIND:");
  
  if(pStr != NULL)
  {
  Indication_No[0] = *(pStr + 5) ;
  Indication_No[1] = *(pStr + 6) ;
  
  if( Indication_No[1] == ':')
  {
    Single_Digit_Indication = WIFI_TRUE;
    /*  Convert char to integer */   
    Wind_No = (WiFi_Indication_t)(Indication_No[0] - '0'); 
  }
  else
  {
    Single_Digit_Indication = WIFI_FALSE;
    /* Convert char to integer */   
    Wind_No = (WiFi_Indication_t)(((Indication_No[0] - '0') * 10 ) + (Indication_No[1] - '0'));
  }
  

  switch (Wind_No)
  { 
  case Console_Active:
      WiFi_WIND_State.ConsoleActive = WIFI_TRUE;
    break;
  case Poweron :
      WiFi_WIND_State.WiFiPowerON = WIFI_TRUE;
    break;
  case WiFi_Reset:
      WiFi_WIND_State.WiFiReset = WIFI_TRUE;
    break;
  case Watchdog_Running:
    break;
  case Heap_Too_Small:
	WiFi_WIND_State.HeapTooSmall=WIFI_TRUE;
    break;
  case WiFi_Hardware_Dead:
      WiFi_WIND_State.WiFiHWFailure = WIFI_TRUE;
    break;
  case Watchdog_Terminating:
    break;
  case SysTickConfigure:
    break;
  case Hard_Fault:
      WiFi_WIND_State.HardFault = WIFI_TRUE;
    break;   
  case StackOverflow:
    WiFi_WIND_State.StackOverflow=WIFI_TRUE;
    break;
  case MallocFailed:
    WiFi_WIND_State.MallocFailed=WIFI_TRUE;
    break;
  case Error:
    WiFi_WIND_State.InitFailure=WIFI_TRUE;
    break;
  case WiFi_PS_Mode_Failure:
    WiFi_WIND_State.PS_Mode_Failure = WIFI_TRUE;
    break;
  case CopyrightInfo:
    break;
  case WiFi_BSS_Regained:
    break;
  case WiFi_Signal_LOW:
    WiFi_WIND_State.WiFiSignalLOW=WIFI_TRUE;
    break;
  case WiFi_Signal_OK :
    break;
  case FW_update:
    break;
  case Encryption_key_Not_Recognized:
    break;
  case WiFi_Join :
      WiFi_WIND_State.WiFiJoin = WIFI_TRUE;
    break;
  case JOINFAILED :
    WiFi_WIND_State.WiFiJoinFailed = WIFI_TRUE;
    break;
  case WiFi_Scanning :
      WiFi_WIND_State.WiFiScanning = WIFI_TRUE;
    break;
  case SCANBLEWUP:
    WiFi_WIND_State.WiFiScanBlewUp = WIFI_TRUE;
    break;
  case SCANFAILED:
    WiFi_WIND_State.WiFiScanFailed = WIFI_TRUE;
    break;
  case WiFi_Up:
    WiFi_WIND_State.WiFiUp = WIFI_TRUE;
      
    break;
  case WiFi_Association_Successful:
    WiFi_WIND_State.WiFiAssociation = WIFI_TRUE;
    break;
  case WiFi_Started_MiniAP_Mode:
    WiFi_WIND_State.WiFiStarted_MiniAPMode = WIFI_TRUE;
    break;
  case WiFi__MiniAP_Associated:
    //Find out which client joined by parsing the WIND //+WIND:28
    ptr_offset = (char *) strstr((const char *)&process_buffer,"+WIND:28");
    for(i=17;i<=33;i++)
    client_MAC_address[i-17] = *(ptr_offset + i) ;    
    WiFi_WIND_State.WiFiAPClientJoined = WIFI_TRUE;
    break;
  case WiFi_MiniAP_Disassociated:
    //Find out which client left by parsing the WIND //+WIND:72
    ptr_offset = (char *) strstr((const char *)&process_buffer,"+WIND:72");
    for(i=17;i<=33;i++)
    client_MAC_address[i-17] = *(ptr_offset + i) ;
    WiFi_WIND_State.WiFiAPClientLeft = WIFI_TRUE;
    break;
  case Start_Failed :
      WiFi_WIND_State.StartFailed = WIFI_TRUE;  
    break;
  case WiFi_EXCEPTION :
      WiFi_WIND_State.WiFiException = WIFI_TRUE;
    break;
  case WiFi_Hardware_Started :
      wifi_ready++;
      WiFi_Enabled = WIFI_TRUE;
      WiFi_WIND_State.WiFiHWStarted = WIFI_TRUE;
      /*If this is a start-up after standby*/
      if(trigger_wakeup_callback == WIFI_TRUE)
        {
          trigger_wakeup_callback = WIFI_FALSE;
          Standby_Enabled = WIFI_FALSE;
          standby_resume_callback = WIFI_TRUE;
        }
    break;
  case WiFi_BSS_LOST:
    break;
  case WiFi_Unhandled_Event:
    break;
  case Scan_Complete:
      WiFi_WIND_State.WiFiScanComplete = WIFI_TRUE;
      Scan_Ongoing = WIFI_FALSE;
    break;
  case WiFi_UNHANDLED_IND:
    WiFi_WIND_State.WiFiUnHandledInd = WIFI_TRUE;
    break;
  case WiFi_UNHANDLED:
    break;
  case WiFi_Powered_Down:
      WiFi_Enabled = WIFI_FALSE;      
      //wifi_ready = 0;
      WiFi_WIND_State.WiFiHWStarted = WIFI_FALSE;
      WiFi_WIND_State.WiFiPowerDown = WIFI_TRUE;
    break;
  case WiFi_MiniAP_Mode :
      WiFi_WIND_State.WiFiMiniAPMode = WIFI_TRUE;
    break;
  case WiFi_Deauthentication:
      WiFi_WIND_State.WiFiDeauthentication = WIFI_TRUE;
    break;    
  case WiFi_Disassociation:
    WiFi_WIND_State.WiFiDisAssociation = WIFI_TRUE;
    break;
  case RX_MGMT:
    WiFi_WIND_State.WiFiRXMgmt = WIFI_TRUE;
    break;
  case RX_DATA:
    WiFi_WIND_State.WiFiRXData = WIFI_TRUE;
    break;
  case RX_UNK:
    WiFi_WIND_State.WiFiRxUnk = WIFI_TRUE;
    break;
  case DOT11_AUTHILLEGAL:
    break;
  case Creating_PSK:
    break; 
  case WPA_Terminated :
    break;
  case WPA_Supplicant_Failed:
    break;
  case WPA_Handshake_Complete:
    break;
  case GPIO_line:
    break;
  case Wakeup:
    break;
  case Factory_debug:
    break;

  case SockON_Data_Pending:
    
    /*+WIND:55:Pending Data:%d:%d*/   
    ptr_offset = (char *) strstr((const char *)&process_buffer,"Data:");
    
    /*Need to find out which socket ID has data pending*/
    databytes_No[0] = *(ptr_offset + 5) ;
          
    SocketId = (databytes_No[0] - '0');//Max number of sockets is 8 (so single digit)
          
    /* EQ. Check for ENC string to identify TLS case. Set enable_query */ 
    if ( (*(ptr_offset + 7) == 'E') && (*(ptr_offset + 8) == 'N') && (*(ptr_offset + 9) == 'C') )
    {
      encoded_data_read = WIFI_TRUE;
      if(WIND55_count==0)
            {
                WIND55_count++;
                SockON_Data_Len = 0;
                enable_query = WIFI_TRUE;
                break;
            } 
      else  
            {
                WIND55_count++;
            }
    }
    else
    {
      encoded_data_read = WIFI_FALSE;
      
      if (enable_sock_read == WIFI_TRUE)
      {	
        #if DEBUG_PRINT
        printf ("\n WIND:55 received while still doing reading....ignoring\r\n");
        #endif
        break;
      }
      enable_query = WIFI_TRUE;
    }
    break;
    
  case SockON_Server_Socket_Closed:
    //Find the id of the socket closed
    //ptr_offset = (char *) strstr((const char *)pStr,"+WIND:58");
    databytes_No[0] = *(pStr + 22) ;
    databytes_No[1] = *(pStr + 23) ;
    
    if( databytes_No[1] == '\r')
    {      
      remote_socket_closed_id = databytes_No[0] - '0'; 
    }
    else
    {
      remote_socket_closed_id = (((databytes_No[0] - '0') * 10 ) + (databytes_No[1] - '0'));
    }
    
    Client_Socket_Close_Cmd = WIFI_TRUE;
    //SockON_Server_Closed_Callback = WIFI_TRUE;
    break;
    
  case In_Command_Mode:
    command_mode=WIFI_TRUE;
    data_mode=WIFI_FALSE;
    break;  
  case In_Data_Mode:
    command_mode=WIFI_FALSE;
    data_mode=WIFI_TRUE;
    
    if(switch_by_default_to_command_mode==WIFI_TRUE)
    {
      if(!command_mode)
      {
        WiFi_switch_to_command_mode();//switch by default
      }    
    }
    if(data_pending_sockD==WIFI_TRUE)
      {
        data_pending_sockD=WIFI_FALSE;
        WiFi_Module_State = Receive_SockD_Data;
      }
    break;
  case Incoming_socket_client:
    Client_Connected = WIFI_TRUE;//Set this so that the callback can be made to the user
    wifi_client_connected=1;
    break;
  case Outgoing_socket_client:
    Client_Disconnected = WIFI_TRUE;//Set this so that the callback can be made to the user
     wifi_client_connected=0;
    break;
  case SockD_Dropping_Data:
    WiFi_WIND_State.WiFiSockdDataLost = WIFI_TRUE;
    break;
  case SockD_Pending_Data: //WIND:64
    //Start Reading data from Client Here.    
   // +WIND:64:Sockd Pending Data:1:130:130
    ptr_offset = (char *) strstr((const char *)&process_buffer,"Data:");
    
    //And now find the data length
      databytes_No[0] = *(ptr_offset + 8) ;//points to number just after 2nd colon
      databytes_No[1] = *(ptr_offset + 9) ;
      databytes_No[2] = *(ptr_offset + 10) ;
      databytes_No[3] = *(ptr_offset + 11) ;
  
    if( databytes_No[0] == ':')//then it is a 1 digit number
    {      
      databytes_No[0] = *(ptr_offset + 9) ;
      databytes_No[1] = *(ptr_offset + 10) ;
      databytes_No[2] = *(ptr_offset + 11) ;
      databytes_No[3] = *(ptr_offset + 12) ;
    }
    else if(databytes_No[1] == ':')//two digit number
    {      
      databytes_No[0] = *(ptr_offset + 10) ;
      databytes_No[1] = *(ptr_offset + 11) ;
      databytes_No[2] = *(ptr_offset + 12) ;
      databytes_No[3] = *(ptr_offset + 13) ;
    }
    else if(databytes_No[2] == ':')//three digit number
    {      
      databytes_No[0] = *(ptr_offset + 11) ;
      databytes_No[1] = *(ptr_offset + 12) ;
      databytes_No[2] = *(ptr_offset + 13) ;
      databytes_No[3] = *(ptr_offset + 14) ;
    }
    else if(databytes_No[3] == ':')//four digit number
    {      
      databytes_No[0] = *(ptr_offset + 12) ;
      databytes_No[1] = *(ptr_offset + 13) ;
      databytes_No[2] = *(ptr_offset + 14) ;
      databytes_No[3] = *(ptr_offset + 15) ;
    }
    
    if( databytes_No[1] == '\r')
    {      
      interim_number_of_bytes = databytes_No[0] - '0'; 
    }
    else if( databytes_No[2] == '\r')
    {
      interim_number_of_bytes = (((databytes_No[0] - '0') * 10 ) + (databytes_No[1] - '0'));
    }
    else if( databytes_No[3] == '\r')
    {
      interim_number_of_bytes = (((databytes_No[0] - '0') * 100 ) + ((databytes_No[1] - '0') * 10 ) + (databytes_No[2] - '0'));
    }
    else //it's a 4-digit number
    {
      interim_number_of_bytes = (((databytes_No[0] - '0') * 1000 ) + ((databytes_No[1] - '0') * 100 ) + ((databytes_No[2] - '0') * 10 ) + (databytes_No[3] - '0'));
    }
    
    if(WIND64_count>0)
      interim_number_of_bytes = interim_number_of_bytes - (730*WIND64_count);
    
    if(interim_number_of_bytes<730 || WIND64_count==3)
      {
        number_of_bytes = number_of_bytes + interim_number_of_bytes;
        WIND64_count=0;
        data_pending_sockD=WIFI_TRUE;
        interim_number_of_bytes = 0;
        
        if(!data_mode)
          {
            switch_by_default_to_command_mode=WIFI_FALSE;//we don't want to switch back to command mode after changing to data mode here
            WiFi_switch_to_data_mode();//switch by default  
          }
      }
    else 
      {
        WIND64_count++;
        number_of_bytes = number_of_bytes + interim_number_of_bytes;
        interim_number_of_bytes = 0;
      }        
    
    break;
    
  case Low_Power_Mode_Enabled:
    Low_Power_Enabled = WIFI_TRUE;
    break;    
  case Going_Into_Standby:
    Standby_Enabled = WIFI_TRUE;
    /*Below line is a patch for the resume callback to happen from the 
      HWStarted WIND that will come after Reset After Standby*/
    //trigger_wakeup_callback = WIFI_TRUE;
    break;
  case Resuming_From_Standby:
    Standby_Enabled = WIFI_FALSE;
    standby_resume_callback = WIFI_TRUE;
    break;    
  case Going_Into_DeepSleep:
    Deep_Sleep_Enabled = WIFI_TRUE;    
    break;
  case Resuming_From_DeepSleep:
    Deep_Sleep_Enabled = WIFI_FALSE;
    Start_DeepSleep_Timer();
    break;
  default:
    break;
  }     
  
  }
  }
  memset(process_buffer, 0x00, MAX_BUFFER_GLOBAL);
  process_Indication_Completed = WIFI_TRUE; 

}


/**
* @brief  Reset_AT_CMD_Buffer
*         Clear USART2 Rx buffer and Wi-Fi AT cmd buffer
* @param  None
* @retval None
*/
void Reset_AT_CMD_Buffer()
{
  memset(WiFi_AT_Cmd_Buff, 0x00, sizeof WiFi_AT_Cmd_Buff); 
  
}


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
* @brief  search_buffer
*         search for substring in a buffer that contains null
* @param  pSourceBuff   : pointer of source buffer
*         sourceBuffLen : length of string buffer
*         pSearchStringBuff : pointer of search string buffer
*         seartchStringLen  : length of search string buffer
* @retval pointer of source buffer
*/
char *search_buffer(char *pSourceBuff, uint16_t sourceBuffLen, char *pSearchStringBuff, uint16_t seartchStringLen)
{   /* warning: O(n^2) */
    int searchlen = sourceBuffLen - seartchStringLen + 1;
    for ( ; searchlen-- > 0; pSourceBuff++)
        if (!memcmp(pSourceBuff, pSearchStringBuff, seartchStringLen))
            return pSourceBuff;
    return NULL;
}


/**
* @brief  Delete_Colon
*         delete colon from input buffer
* @param  input : pointer of input buffer
* @retval return pointer of updated buffer
*/
char* Delete_Colon(char* input)                                         
{
    int i,j;
    char *output=input;
    
    /* Delete Colon */
    for (i = 0, j = 0; i<strlen(input); i++,j++)          
    {
#if 0     
      if (input[i]!=':') && (input[i]!='\r')&& (input[i]!='\n')&& (input[i]!='O')&& (input[i]!='K'))
            output[j]=input[i];                     
        else
            j--;                                     
#else
      if ((input[i] ==':') || (input[i]=='\r')|| (input[i]=='\n')|| (input[i]=='O')|| (input[i]=='K'))
            j--;                                     
        else
          output[j]=input[i];                            
    }
    //output[j]=NULL;    
#endif  
    return output;
}

/**
* @brief  Read_WiFi_SSID
*         Read SSID of WiFi module store in flash 
* @param  string : pointer of SSID
* @retval return status of AT cmd request
*/
WiFi_Status_t Read_WiFi_SSID(char *string)
{
  WiFi_Status_t status = WiFi_MODULE_SUCCESS;
  char * ssid = "wifi_ssid";
  char * pStr;
  /* AT+S.GCFG=wifi_ssid read SSID */
  Reset_AT_CMD_Buffer();
  
  /* AT : send AT command */  
  sprintf((char*)(char*)WiFi_AT_Cmd_Buff,AT_GET_CONFIGURATION_VALUE,ssid);   

  /* 
    Response :
    # wifi_ssid = 41:6E:53:53:49:44:00:00:00:00:00:00:00:00:00:00:00:00:00:
    00:00:00:00:00:00:00:00:00:00:00:00:00<cr><lf>
    <cr><lf>OK<cr><lf>
    */
  
  status = USART_Transmit_AT_Cmd();
  if(status == WiFi_MODULE_SUCCESS)
  {
    status = USART_Receive_AT_Resp(Receive_Get_Cfg_Response);
  }

    /* find a substring (# wifi_ssid = )inside a USART2_RxBuffer that may contain nulls */
    uint16_t sourceBuffLen = sizeof(get_cfg_value)-1; /* exclude null terminator from length */
    char searchStringBuff[] = "#  wifi_ssid = ";
    uint16_t stringBuffLen = sizeof(searchStringBuff)-1; /* exclude null terminator from length */    
    char *res = search_buffer((char *)&get_cfg_value, sourceBuffLen, searchStringBuff, stringBuffLen);
    
    pStr = (char *) (strstr((const char *)res,"= "));
    if(pStr != NULL)
    {
      strcat( string, (pStr + 2));      
      /* Remove colon,\r,\n,OK strings */
      memcpy(string, Delete_Colon(string) , 32); 
    }
    
  return status;    
}

/**
* @brief  Read_WiFi_SecKey
*         Read Security key of WiFi module store in flash 
* @param  string : pointer of Security key
* @retval return status of AT cmd request
*/
WiFi_Status_t Read_WiFi_SecKey(char *string)
{
  WiFi_Status_t status = WiFi_MODULE_SUCCESS;
  char *seckey = "wifi_wpa_psk_text";
  char *pStr;
  
  /* AT+S.GCFG=wifi_ssid read SSID */
  Reset_AT_CMD_Buffer();
  
  /* AT : send AT command */  
  sprintf((char*)(char*)WiFi_AT_Cmd_Buff,AT_GET_CONFIGURATION_VALUE,seckey);  
  
  status = USART_Transmit_AT_Cmd();
  if(status == WiFi_MODULE_SUCCESS)
  {
    status = USART_Receive_AT_Resp(Receive_Get_Cfg_Response);
  }


    /* find a substring (wifi_wpa_psk_text = )inside a USART2_RxBuffer that may contain nulls */
    uint16_t sourceBuffLen = sizeof(get_cfg_value)-1; /* exclude null terminator from length */
    char searchStringBuff[] = "wifi_wpa_psk_text = ";
    uint16_t stringBuffLen = sizeof(searchStringBuff)-1; /* exclude null terminator from length */    
    char *res = search_buffer((char *)&get_cfg_value, sourceBuffLen, searchStringBuff, stringBuffLen);
    
    pStr = (char *) (strstr((const char *)res,"= "));
    if(pStr != NULL)
    {
      strcat( string, (pStr + 2));      
      /* Remove colon,\r,\n,OK strings */
      memcpy(string, Delete_Colon(string) , 32); 
    }
    
  return status;    
  }

/**
* @brief  Read_WiFi_Mode
*         Read Wi-Fi mode 0: idle,1 =STA,2 =IBSS,3 =MiniAP
* @param  string : return wifi mode type
* @retval return status of AT cmd request
*/
WiFi_Status_t Read_WiFi_Mode(char *string)
{
  WiFi_Status_t status = WiFi_MODULE_SUCCESS;
  char *mode = "wifi_mode";
  char *pStr;
  
    /* AT+S.GCFG=wifi_mode */
  Reset_AT_CMD_Buffer();
  
  /* AT : send AT command */  
  sprintf((char*)(char*)WiFi_AT_Cmd_Buff,AT_GET_CONFIGURATION_VALUE,mode);  
  
  status = USART_Transmit_AT_Cmd();
  if(status == WiFi_MODULE_SUCCESS)
  {
    status = USART_Receive_AT_Resp(Receive_Get_Cfg_Response);
  }
  
  pStr = (char *) strstr((const char *)&get_cfg_value,"wifi_mode = ");
  if(pStr != NULL)
  {
    string[0] = *(pStr + 12) ;
  }

  return status ;
}

/**
* @brief  Write_WiFi_SSID
*         Store SSID in flash memory of WiFi module
* @param  string : pointer of SSID
* @retval return status of AT cmd request
*/
WiFi_Status_t Write_WiFi_SSID(char *string)
{
  WiFi_Status_t status = WiFi_MODULE_SUCCESS;  
  Reset_AT_CMD_Buffer(); 
  
  /* AT+S.SSIDTXT=abcd <ExampleSSID> //set SSID */
  sprintf((char*)(char*)WiFi_AT_Cmd_Buff,AT_SET_SSID,string);  
  
  status = USART_Transmit_AT_Cmd();
  if(status == WiFi_MODULE_SUCCESS)
  {
    status = USART_Receive_AT_Resp(Receive_AT_Cmd_Response);
  }
  
  /* AT&W :Save the settings on the flash memory */
  Reset_AT_CMD_Buffer();
  Save_Current_Setting();
  
  return status; 
  
}


/**
* @brief  Write_WiFi_SecKey
*         Store security key in flash memory of WiFi module
* @param  string : pointer of security key
* @retval return status of AT cmd request
*/
WiFi_Status_t Write_WiFi_SecKey(char *string)
{
  WiFi_Status_t status = WiFi_MODULE_SUCCESS;  
  Reset_AT_CMD_Buffer(); 
  
  /* AT+S.SCFG=wifi_wpa_psk_text,helloworld : set password */
  sprintf((char*)(char*)WiFi_AT_Cmd_Buff,"AT+S.SCFG=wifi_wpa_psk_text,%s\r",string);
  
  status = USART_Transmit_AT_Cmd();
  if(status == WiFi_MODULE_SUCCESS)
  {
    status = USART_Receive_AT_Resp(Receive_AT_Cmd_Response);
  }
  
  /* AT&W :Save the settings on the flash memory */
  Reset_AT_CMD_Buffer();
  Save_Current_Setting();
  
  return status;     
}



/**
* @brief  PrintErrorMsg
*         Print error message on UART terminal
* @param  None
* @retval None
*/
void PrintErrorMsg (void)
{
  Print_Msg("error in AT cmd",sizeof("error in AT cmd"));
}

/**
  * @brief  Print_Msg
  *         Print messages on UART terminal
  * @param  msgBuff : Contains data that need to be print
  * @param  length  : leghth of the data
  * @retval None
  */
void Print_Msg(char * msgBuff,uint8_t length)
{

}

/**
* @brief  Error_Handler
*         This function is executed in case of error occurrence.
* @param  None
* @retval None
*/
void Error_Handler(void)
{
  /* Turn LED2 on */
  BSP_LED_On(LED2);
  //The following while(1) is commented as it prevents standby functionality
  /*while(1)
  {
    //Error if LED2 is slowly blinking (1 sec. period)
    BSP_LED_Toggle(LED2); 
    HAL_Delay(1000); 
  } */ 
}


/**
* @brief  USART_Transmit_AT_Cmd
*         send AT cmd on UART port of wifi module.
* @param  None
* @retval WiFi_Status_t : status of AT cmd
*/
WiFi_Status_t USART_Transmit_AT_Cmd()
{
  int i;
  uint16_t size;
    
  //Check for Hardware Started
  if(WiFi_Enabled == WIFI_FALSE) 
    return WiFi_NOT_READY;
  //Check for Deep-Sleep or Standby Mode, return error if true
  if (Standby_Enabled == WIFI_TRUE || Deep_Sleep_Enabled == WIFI_TRUE)
    return WiFi_IN_LOW_POWER_ERROR;
  
  AT_Cmd_Processing = WIFI_TRUE;//Stop Any Rx between the TX call
  
  for(i=0, size=0; i<1023; i++){
            if(((WiFi_AT_Cmd_Buff[i]=='\r') || (WiFi_AT_Cmd_Buff[i]=='.')) && (WiFi_AT_Cmd_Buff[i+1]=='\0')){
              size=i+1;
              break;
            }
      }
  
  if (size == 0)
    {
        printf("ERROR in USART_Transmit_AT_Cmd!");
        return WiFi_UNHANDLED_IND_ERROR;
    }
  
#if defined(USART3_INT_MODE)
  if(HAL_UART_Transmit_IT(&UartWiFiHandle, (uint8_t *)WiFi_AT_Cmd_Buff, size)!= HAL_OK)
  {
    Error_Handler();
    return WiFi_HAL_UART_ERROR;
  }
  while (UartReady != SET);
  UartReady = RESET; 

#elif defined(USART3_POLLING_MODE)
  //while(Uartx_Rx_Processing!=WIFI_FALSE);
  if(HAL_UART_Transmit(&UartWiFiHandle, (uint8_t *)WiFi_AT_Cmd_Buff, size, 1000)!= HAL_OK)
  {
    Error_Handler();
    #if DEBUG_PRINT
    printf("HAL_UART_Transmit Error");
    #endif
    return WiFi_HAL_UART_ERROR;
  }
  
#else
 #error "Please select USART mode in your application (in wifi_module.h file)"
#endif
 
  AT_Cmd_Processing = WIFI_FALSE;//Re-enable Rx for UART
  if(Uartx_Rx_Processing == WIFI_FALSE)
    Receive_Data();//Start receiving Rx from the UART again, if and only if it was stopped in the previous Uartx_Rx_Handler
  return WiFi_MODULE_SUCCESS;
}

/**
* @brief  Start_DeepSleep_Timer
*         start the deep sleep timer.
* @param  None
* @retval void
*/
void Start_DeepSleep_Timer(void)
{
  Deep_Sleep_Timer = WIFI_TRUE;
  sleep_count = 0;
}

/**
* @brief  Stop_DeepSleep_Timer
*         stop the deep sleep timer.
* @param  None
* @retval void
*/
void Stop_DeepSleep_Timer()
{
  Deep_Sleep_Timer = WIFI_FALSE;
  sleep_count = 0;
}

#if 0
/**
* @brief  configure_to_exti
*         Configured the USART Rx pin to EXTI pin to capture standby wakeup interrupt
* @param  None
* @retval None
*/
void configure_to_exti()
{
  /*configure the pin*/
  
  HAL_NVIC_DisableIRQ(USARTx_IRQn);//Disable UART IRQ
  
  /* USART_RX Pin as EXTI IRQ*/
  GPIO_InitTypeDef  GPIO_InitStruct;
  GPIO_InitStruct.Pin = WiFi_USART_RX_PIN;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_HIGH;
#if defined (USE_STM32L0XX_NUCLEO) || (USE_STM32F4XX_NUCLEO) 
  GPIO_InitStruct.Alternate = 0;
#endif
  HAL_GPIO_Init(WiFi_USART_RX_GPIO_PORT, &GPIO_InitStruct);
  
  /* Configure the NVIC for EXTI */  
  HAL_NVIC_SetPriority(USARTx_EXTI_IRQn, 3, 0);    
  HAL_NVIC_EnableIRQ(USARTx_EXTI_IRQn);
}
#endif

/**
* @brief  WiFi_switch_to_command_mode
*         switch to command mode from data mode
* @param  None
* @retval None
*/
void WiFi_switch_to_command_mode(void)
{
  WiFi_Status_t status = WiFi_MODULE_SUCCESS;
  
  /* AT+S.*/  
  memset(WiFi_AT_Cmd_Buff, 0x00, sizeof WiFi_AT_Cmd_Buff);    
  
  sprintf((char*)(char*)WiFi_AT_Cmd_Buff,AT_DATA_TO_CMD_MODE);   //Notice the lower case
  
  status = USART_Transmit_AT_Cmd();
  if(status == WiFi_MODULE_SUCCESS)
  {
    //nothing to do
  }
}


/**
* @brief  WiFi_switch_to_data_mode
*         switch to data mode from command mode
* @param  None
* @retval None
*/
void WiFi_switch_to_data_mode(void)
{
  WiFi_Status_t status = WiFi_MODULE_SUCCESS;
  
  /* AT+S.*/  
  memset(WiFi_AT_Cmd_Buff, 0x00, sizeof WiFi_AT_Cmd_Buff);    
  
  sprintf((char*)(char*)WiFi_AT_Cmd_Buff,AT_CMD_TO_DATA_MODE);   //Notice the upper case
  
  status = USART_Transmit_AT_Cmd();
  
  if(status == WiFi_MODULE_SUCCESS)
  {
    //nothing to do
  }
   
}


/**
* @brief  Attention_Cmd
*         Attention command
* @param  None
* @retval WiFi_Status_t : status of AT cmd Request
*/
WiFi_Status_t Attention_Cmd()
{
  WiFi_Status_t status = WiFi_MODULE_SUCCESS;
  
  Reset_AT_CMD_Buffer(); 
  
  /* AT : send AT command */
  sprintf((char*)(char*)WiFi_AT_Cmd_Buff,AT_ATTENTION);  

  status = USART_Transmit_AT_Cmd();
  if(status == WiFi_MODULE_SUCCESS)
  {
    status = USART_Receive_AT_Resp(Receive_AT_Cmd_Response);
  }
  return status; 
}


/**
* @brief  SET_Power_State
*         SET power mode of wifi module
* @param  state : power mode of wi-fi module i.e active,sleep,standby,powersave
* @retval WiFi_Status_t : status of AT cmd Request
*/
WiFi_Status_t SET_Power_State(WiFi_Power_State_t state)
{
  WiFi_Status_t status = WiFi_MODULE_SUCCESS;
  
#if DEBUG_PRINT  
    printf("\r\n >>Soft Reset Wi-Fi module\r\n");
#endif
  
  Reset_AT_CMD_Buffer();
  
  /* AT : send AT command */
  sprintf((char*)(char*)WiFi_AT_Cmd_Buff,AT_SET_POWER_STATE,state);  
  WiFi_Module_State = Receive_Indication; 
  //WiFi_WIND_State.WiFiReset = WIFI_FALSE;
  WiFi_WIND_State.WiFiHWStarted = WIFI_FALSE;
  status = USART_Transmit_AT_Cmd();
  if(status != WiFi_MODULE_SUCCESS) 
    return status;
  memset((void*)&WiFi_WIND_State,0x00,sizeof(WiFi_WIND_State)); /*reset the WIND State?*/
  /* AT+CFUN=1 //Soft reset */
  while(WiFi_WIND_State.WiFiHWStarted != WIFI_TRUE);
  
 
  return status;
}
  

/**
* @brief  Display_Help_Text
*         this function will print a list of all commands supported with a brief help text for each cmd
* @param  None
* @retval WiFi_Status_t : status of AT cmd Request
*/
WiFi_Status_t Display_Help_Text()
{
  WiFi_Status_t status = WiFi_MODULE_SUCCESS;
  
  Reset_AT_CMD_Buffer(); 
  
  /* AT : send AT command */
  sprintf((char*)(char*)WiFi_AT_Cmd_Buff,AT_HELP_TEXT);  

  status = USART_Transmit_AT_Cmd();
  if(status == WiFi_MODULE_SUCCESS)
  {
    status = USART_Receive_AT_Resp(Receive_AT_Cmd_Response);
  }
  return status; 
}


/**
* @brief  GET_Configuration_Value
*         Get a wifi configuration value from the module
* @param  sVar_name : Name of the config variable
*         aValue    : value of config variable to be returned to user
* @retval WiFi_Status_t : status of AT cmd Request
*/
WiFi_Status_t GET_Configuration_Value(char* sVar_name,uint32_t *aValue)
{
  WiFi_Status_t status = WiFi_MODULE_SUCCESS;
  
  Reset_AT_CMD_Buffer(); 
  
  /* AT : send AT command */  
  sprintf((char*)(char*)WiFi_AT_Cmd_Buff,AT_GET_CONFIGURATION_VALUE,sVar_name);   

  status = USART_Transmit_AT_Cmd();
  if(status == WiFi_MODULE_SUCCESS)
  {
    status = USART_Receive_AT_Resp(Receive_Get_Cfg_Response);
  }
  memcpy(aValue,get_cfg_value, strlen((const char*)get_cfg_value));//copy user pointer to get_cfg_value
  memset(get_cfg_value, 0x00, strlen((const char*)get_cfg_value));
  return status; 
}

WiFi_Status_t SET_Configuration_Addr(char* sVar_name,char* addr)
{
  WiFi_Status_t status = WiFi_MODULE_SUCCESS;
  
  Reset_AT_CMD_Buffer(); 
  
  /* AT : send AT command */
  sprintf((char*)(char*)WiFi_AT_Cmd_Buff,AT_SET_CONFIGURATION_ADDRESS,sVar_name,addr);  

  status = USART_Transmit_AT_Cmd();
  if(status == WiFi_MODULE_SUCCESS)
  {
    status = USART_Receive_AT_Resp(Receive_AT_Cmd_Response);
  }
  return status; 

}

/**
* @brief  SET_Configuration_Value
*         SET the value of configuration variable
* @param  sVar_name : Name of the config variable
*         aValue    : value of config variable
* @retval WiFi_Status_t : status of AT cmd Request
*/
WiFi_Status_t SET_Configuration_Value(char* sVar_name,uint32_t aValue)
{
  WiFi_Status_t status = WiFi_MODULE_SUCCESS;

  Reset_AT_CMD_Buffer(); 
  
  /* AT : send AT command */
  sprintf((char*)(char*)WiFi_AT_Cmd_Buff,AT_SET_CONFIGURATION_VALUE,sVar_name,aValue);  

  status = USART_Transmit_AT_Cmd();
  if(status == WiFi_MODULE_SUCCESS)
  {
    status = USART_Receive_AT_Resp(Receive_AT_Cmd_Response);
  }

  return status; 
}

/**
* @brief  SET_SSID
*         SET SSID in flash memory of Wi-Fi module
* @param  ssid : pointer of SSID
* @retval WiFi_Status_t : status of AT cmd Request
*/
WiFi_Status_t SET_SSID(char* ssid)
{
  WiFi_Status_t status = WiFi_MODULE_SUCCESS;
  
  Reset_AT_CMD_Buffer(); 
  
  /* AT+S.SSIDTXT=abcd <ExampleSSID>  */
  sprintf((char*)(char*)WiFi_AT_Cmd_Buff,AT_SET_SSID,ssid);  

  status = USART_Transmit_AT_Cmd();
  if(status == WiFi_MODULE_SUCCESS)
  {
    status = USART_Receive_AT_Resp(Receive_AT_Cmd_Response);
  }
  return status; 
}



/**
* @brief  SET_WiFi_SecKey
*         SET wifi security key
* @param  seckey : pointer of security key
* @retval WiFi_Status_t : status of AT cmd Request
*/
WiFi_Status_t SET_WiFi_SecKey(char* seckey)
{
  WiFi_Status_t status = WiFi_MODULE_SUCCESS;
  
  Reset_AT_CMD_Buffer(); 
  
  /* AT+S.SCFG=wifi_wpa_psk_text,helloworld : set password */
  sprintf((char*)(char*)WiFi_AT_Cmd_Buff,AT_SET_SEC_KEY,seckey);  

  status = USART_Transmit_AT_Cmd();
  if(status == WiFi_MODULE_SUCCESS)
  {
    status = USART_Receive_AT_Resp(Receive_AT_Cmd_Response);
  }
  return status;    
}


/**
* @brief  Restore_Default_Setting
*         Restore the factory default values of the configuration variables 
*         and writes them to non volatile storage
* @param  None
* @retval WiFi_Status_t : status of AT cmd Request
*/
WiFi_Status_t Restore_Default_Setting()
{
  WiFi_Status_t status = WiFi_MODULE_SUCCESS;
  
  //Reset_AT_CMD_Buffer(); 
  
  /* AT&F: restore default setting */
  sprintf((char*)(char*)WiFi_AT_Cmd_Buff,AT_RESTORE_DEFAULT_SETTING);  

  status = USART_Transmit_AT_Cmd();
  
  if(status == WiFi_MODULE_SUCCESS)
  {
    status = USART_Receive_AT_Resp(Receive_AT_Cmd_Response);
  }
  return status; 
  
}

/**
* @brief  Save_Current_Setting
*         Store the current RAM-based setting to non-volatile storage
* @param  None
* @retval WiFi_Status_t : status of AT cmd Request
*/
WiFi_Status_t Save_Current_Setting()
{
  WiFi_Status_t status = WiFi_MODULE_SUCCESS;
  
  Reset_AT_CMD_Buffer(); 
  
  /* AT&W :Save the settings on the flash memory */
  sprintf((char*)(char*)WiFi_AT_Cmd_Buff,AT_SAVE_CURRENT_SETTING);  

  status = USART_Transmit_AT_Cmd();
  if(status == WiFi_MODULE_SUCCESS)
  {
    status = USART_Receive_AT_Resp(Receive_AT_Cmd_Response);
  }
  return status; 
}


/**
* @brief  ResetBuffer
*         Reset receive data/indication msg buffer
* @param  None
* @retval None
*/
void ResetBuffer()
{  
  
}


/**
* @brief  config_init_value
*         initalize config values before reset
* @param  None
* @retval None
*/
WiFi_Status_t config_init_value(char* sVar_name,uint8_t aValue)
{
  WiFi_Status_t status = WiFi_MODULE_SUCCESS;
  Reset_AT_CMD_Buffer();   
  sprintf((char*)(char*)WiFi_AT_Cmd_Buff,AT_SET_CONFIGURATION_VALUE,sVar_name,aValue);
  if(HAL_UART_Transmit(&UartWiFiHandle, (uint8_t *)WiFi_AT_Cmd_Buff, strlen((char*)WiFi_AT_Cmd_Buff),1000)!= HAL_OK)
  {
    Error_Handler();    
    return WiFi_HAL_UART_ERROR;
  }
  
  status = WaitForResponse(AT_RESP_LEN_OK);
  return status;
}

/**
* @brief  config_init_addr
*         initalize config strings/addresses before reset
* @param  None
* @retval None
*/
WiFi_Status_t config_init_addr(char* sVar_name,char* addr)
{
  WiFi_Status_t status = WiFi_MODULE_SUCCESS;
  Reset_AT_CMD_Buffer();   
  sprintf((char*)(char*)WiFi_AT_Cmd_Buff,AT_SET_CONFIGURATION_ADDRESS,sVar_name,addr);
  if(HAL_UART_Transmit(&UartWiFiHandle, (uint8_t *)WiFi_AT_Cmd_Buff, strlen((char*)WiFi_AT_Cmd_Buff),1000)!= HAL_OK)
  {
    Error_Handler();    
    return WiFi_HAL_UART_ERROR;
  }
  
  status = WaitForResponse(AT_RESP_LEN_OK);
  return status;

}


/**
* @brief  WaitForResponse
*         Wait for OK response
* @param  None
* @retval None
*/
WiFi_Status_t WaitForResponse(uint16_t alength)
{
  WiFi_Status_t status = WiFi_MODULE_SUCCESS;
  
  if(alength <= RxBufferSize)
  {
    if(HAL_UART_Receive(&UartWiFiHandle, (uint8_t *)USART_RxBuffer, alength,5000)!= HAL_OK)
    {
      Error_Handler();
      return WiFi_HAL_UART_ERROR;
    }
    
    if(((strstr((const char *)&USART_RxBuffer,"OK"))) == NULL)
    {
      return WiFi_AT_CMD_RESP_ERROR;
    }
    
  }
  
  return status;  
}
/**** Wi-Fi indication call back *************/
__weak void ind_wifi_warning(WiFi_Status_t warning_code)
{
}
	
__weak void ind_wifi_error(WiFi_Status_t error_code)
{
}

__weak void ind_wifi_connection_error(WiFi_Status_t status_code)
{
}

__weak void ind_wifi_connected(void)
{
}
	
__weak void ind_wifi_ap_ready(void)
{
}

__weak void ind_wifi_ap_client_joined(uint8_t * client_mac_address)
{
}

__weak void ind_wifi_ap_client_left(uint8_t * client_mac_address)
{
}

__weak void ind_wifi_on(void)
{
}

__weak void ind_wifi_packet_lost(WiFi_Status_t status_code)
{
}

__weak void ind_wifi_gpio_changed(void)
{
}

__weak void ind_wifi_socket_data_received(uint8_t * data_ptr, uint32_t message_size, uint32_t chunk_size)
{
}

__weak void ind_wifi_socket_client_remote_server_closed(uint8_t * socketID)
{
}

__weak void ind_wifi_socket_server_data_lost(void)
{
}

__weak void ind_socket_server_client_joined(void)
{
}

__weak void ind_socket_server_client_left(void)
{
}

__weak void ind_wifi_http_data_available(uint8_t * data_ptr)
{
}

__weak void ind_wifi_resuming(void)
{
}



/**
  * @}
  */ 

/**
  * @}
  */ 


/**
  * @}
  */ 

/******************* (C) COPYRIGHT 2015 STMicroelectronics *****END OF FILE****/

