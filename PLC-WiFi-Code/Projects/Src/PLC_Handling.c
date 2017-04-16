/**
  ******************************************************************************
  * @file    PLC_Handling.c
  * @author  System Lab Industrial & Motion Control Team
  * @version V1.0.0
  * @date    02-Dicember-2016
  * @brief
  ******************************************************************************
  *
  * COPYRIGHT(c) 2015 STMicroelectronics
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
#include "stm32f4xx_hal.h"
#include  "stm32_hal_legacy.h"
#include "main.h"
#include "PLC_Handling.h"

#include "stm32f4xx_hal_spi.h"
#include "stm32f4xx_nucleo.h"
#include "string.h"
#include "stdbool.h"
#include "stdio.h"
#include "X_NUCLEO_PLC01A1.h"
#include "VNI8200XP.h"
#include "CLT01_38S.h"

/** @addtogroup Drivers     Drivers
  * @{
  * @brief Demo Driver Layer
  */ 
    
    
    
/** @addtogroup BSP     BSP
  * @{
  * @brief Boar support package layer
  */ 


/** @addtogroup X_Nucleo_Modules     X-NucleoModules
  * @brief   X-Nucleo Layer
  *@{
  */    
    
/** @addtogroup X_Nucleo_PLC_Handling     X_Nucleo_PLC Handling
  * @brief    X-Nucleo Driver Layer
  *@{
  */    
    
 
    
/** @defgroup X_Nucleo_PLC_Handling_Private_variable     X_Nucleo_PLC Handling Private variable
  * @brief    
  *@{
  */ 


uint8_t FB_Data[3];
uint8_t FB_Data1[3];
uint8_t str_data[8]={0};
uint8_t output_prog=0; 
uint8_t device_FB=0;

/* VNI and CLT transmit and receive buffers for SPI communication */
uint8_t VNI_TxBuffer[2] = {0x00,0x00};          /*! < SPI VNI TX buffer            */
uint8_t VNI_RxBuffer[2] = {0x00,0x00};          /*! < SPI VNI RX buffer            */
uint8_t CLT_TxBuffer[2] = {0x00,0x00};          /*! < SPI CLT RX buffer            */
uint8_t CLT_RxBuffer[2] = {0x00,0x00};          /*! < SPI CLT RX buffer            */

/* Variables for Parity bits and calculation*/
uint8_t Parity_Cal0 = 0x00;                    /*! < temp VNI parity variable       */ 
uint8_t Parity_Cal1 = 0x00;                    /*! < temp VNI parity variable       */ 
uint8_t Parity_Cal2 = 0x00;                    /*! < temp VNI parity variable       */
uint8_t Parity_Cal3 = 0x00;                    /*! < temp VNI parity variable       */
uint8_t Parity_Cal4 = 0x00;                    /*! < temp VNI parity variable       */
uint8_t Parity_Cal5 = 0x00;                    /*! < temp VNI parity variable       */
uint8_t Parity_Cal6 = 0x00;                    /*! < temp VNI parity variable       */
uint8_t Parity_Cal7 = 0x00;                    /*! < temp VNI parity variable       */
uint8_t nP0 = 0x00;                     /*! < VNI parity variable       */
uint8_t P0 = 0x00;                      /*! < VNI parity variable       */
uint8_t P1 = 0x00;                      /*! < VNI parity variable       */
uint8_t P2 = 0x00;                       /*! < VNI parity variable       */
uint8_t Input_CHS=0x43;
uint8_t com_flag=0;    

/**
  * @} //End X_Nucleo_PLC_Handling_Private_variable
  */



/* Private variables ---------------------------------------------------------*/
HAL_StatusTypeDef spi_state;

/** @defgroup X_Nucleo_PLC_Handling_Exported_variable     X_Nucleo_PLC Handling Exported variable
  * @brief    
  * @{
  */

extern SPI_HandleTypeDef hspi1;
extern TIM_HandleTypeDef htim3;
extern TimerParam_Typedef Tim_Setting;
extern uint8_t wifi_frame[2];
extern PLC_State_TypeDef PLC_State;
extern CLT_CheckState_Typedef CLT_State;
extern uint8_t output_status;
extern uint8_t output_buffer;
extern volatile uint16_t Array_Index;
extern char FB_ARRAY[300];
/**
  * @} //End X_Nucleo_PLC Handling Exported variable
  */



/** @defgroup X_Nucleo_PLC_Handling_Exported_Functions     X_Nucleo_PLC Handling Exported Functions
  * @brief    
  * @{ 
  */
 
/** 
  * @brief: Read CLT01 input status
  * @param: None
  * @retval: CLT buffer
*/

int16_t CLT_VNI_RxTx(void)
{
  int16_t ret_CLT=0;
uint8_t* CLT_RxBuffer;
    CLT_RxBuffer= CLT01_38S_GetInpData();
  Input_CHS=CLT_RxBuffer[1];
  FB_Data1[0]=CLT_RxBuffer[0];
return ret_CLT;
}


/** 
  * @brief:     Read VNI8200XP input status
  * @param:     Output stage sonfiguration
  * @retval:    VNI fault byte
*/

uint8_t VNI_TxRx (uint8_t OutputStage)
{ 
  int8_t ret_VNI=-1;
  uint8_t* ret_byte; 
  uint8_t var1=0;
  memset(FB_Data,0,3);
  PLC_State=PLC_FAIL;
  
  ret_byte=BSP_RELAY_SetOutputs(&OutputStage);
var1=ret_byte[0];
  if(BSP_GetRelayStatus(&var1)==0)// check parity bit
    {
      PLC_State=PLC_PROGRAMMED;
      ret_VNI=0;
    }
  else if((BSP_GetRelayStatus(&var1)==6)||(BSP_GetRelayStatus(&var1)==4))
  {
     PLC_State=PLC_FAIL;
  }  

  FB_Data[0]=OutputStage;
  FB_Data[1]=ret_byte[0];
  FB_Data[2]=ret_byte[1];
  return ret_VNI;
}

/** 
  * @brief:     Get VNI8200XP ouput status
  * @param:     Output index
*/

void GET_VNI_OutFB(uint8_t Out_index )
{

  uint8_t VNI_Feedback=0;

       output_prog=(FB_Data[0]>>Out_index)&0x01;
//       output_prog=1;//TEST
         if(output_prog==1)
         {              
           VNI_Feedback=(FB_Data[2]>>Out_index)&0x01; 
//           VNI_Feedback=1;//TEST
            if(VNI_Feedback==0)
            {
              switch (Out_index)
              {
              case 0:
                FB_ARRAY[Array_Index+8]='1';
                break;
              case 1:
                FB_ARRAY[Array_Index+8] = '2';
                break;
              case 2:
                FB_ARRAY[Array_Index+8] = '3';
                break;
              case 3:
                FB_ARRAY[Array_Index+8] = '4';
                break;
              case 4:
                FB_ARRAY[Array_Index+8] = '5';
                break;
                case 5:
                  FB_ARRAY[Array_Index+8] = '6';
                break;
              case 6:
                FB_ARRAY[Array_Index+8] = '7';
                break;
              case 7:
                FB_ARRAY[Array_Index+8] = '8';
                break;
              }
              FB_ARRAY[Array_Index+9] = ' ';
              FB_ARRAY[Array_Index+10] = 'O';
              FB_ARRAY[Array_Index+11] = 'N';
              FB_ARRAY[Array_Index+12] = '\r';
              FB_ARRAY[Array_Index+13] = '\n';
              Array_Index+=14;
            }
            else if(VNI_Feedback==1)
            {
              switch (Out_index)
              {
                case 0:
              
                FB_ARRAY[Array_Index+8]='1';

                break;
              case 1:
                FB_ARRAY[Array_Index+8] = '2';
                break;
              case 2:
                FB_ARRAY[Array_Index+8] = '3';
                break;
              case 3:
                FB_ARRAY[Array_Index+8] = '4';
                break;
              case 4:
                FB_ARRAY[Array_Index+8] = '5';
                break;
                case 5:
                  FB_ARRAY[Array_Index+8] = '6';
                break;
              case 6:
                FB_ARRAY[Array_Index+8] = '7';
  
                break;
              case 7:
                FB_ARRAY[Array_Index+8] = '8';

                break;
              }
              FB_ARRAY[Array_Index+9] = ' ';
              FB_ARRAY[Array_Index+10] = 'F';
              FB_ARRAY[Array_Index+11] = 'A';
              FB_ARRAY[Array_Index+12] = 'I';
              FB_ARRAY[Array_Index+13] = 'L';
              FB_ARRAY[Array_Index+14] = '\r';
              FB_ARRAY[Array_Index+15] = '\n';
              Array_Index+=16;
            }
          
         }
         else if(output_prog==0)
         {
              switch (Out_index)
              {
                case 0:
                FB_ARRAY[Array_Index+8]='1';
                break;
              case 1:
                FB_ARRAY[Array_Index+8] = '2';
                break;
              case 2:
                FB_ARRAY[Array_Index+8] = '3';
                break;
              case 3:
                FB_ARRAY[Array_Index+8] = '4';
                break;
              case 4:
                FB_ARRAY[Array_Index+8] = '5';
                break;
                case 5:
                  FB_ARRAY[Array_Index+8] = '6';
                break;
              case 6:
                FB_ARRAY[Array_Index+8] = '7';
                break;
              case 7:
                FB_ARRAY[Array_Index+8] = '8';
                break;
              }
           FB_ARRAY[Array_Index+9] = ' ';
           FB_ARRAY[Array_Index+10] = 'O';
           FB_ARRAY[Array_Index+11] = 'F';
           FB_ARRAY[Array_Index+12] = 'F';           
           FB_ARRAY[Array_Index+13] = '\r';
           FB_ARRAY[Array_Index+14] = '\n';
           Array_Index+=15;
         }

}



/** 
  * @brief:     Get VNI8200XP IC status
  * @param:     Output index
*/

void GET_VNI_FB( uint8_t Out_index)
{

               
           device_FB=(FB_Data[1]>>Out_index)&0x01; 
//       device_FB=0;//TEST
            if(device_FB==0)
            {
              switch (Out_index)
              {
              case 4:
                FB_ARRAY[Array_Index+0] = 'P';
                FB_ARRAY[Array_Index+1] = 'O';
                FB_ARRAY[Array_Index+2] = 'W';
                FB_ARRAY[Array_Index+3] = 'E';           
                FB_ARRAY[Array_Index+4] = 'R';
                FB_ARRAY[Array_Index+5] = ' ';
                FB_ARRAY[Array_Index+6] = 'G';
                FB_ARRAY[Array_Index+7] = 'O';
                FB_ARRAY[Array_Index+8] = 'O';
                FB_ARRAY[Array_Index+9] = 'D';           
                FB_ARRAY[Array_Index+10] = ' ';
                FB_ARRAY[Array_Index+11] = 'O';
                FB_ARRAY[Array_Index+12] = 'K'; 
                FB_ARRAY[Array_Index+13] = '\r';
                FB_ARRAY[Array_Index+14] = '\n'; 
                Array_Index+=15;
                break;
              case 5:       
                FB_ARRAY[Array_Index+0] = 'P';
                FB_ARRAY[Array_Index+1] = 'C';           
                FB_ARRAY[Array_Index+2] = ' ';
                FB_ARRAY[Array_Index+3] = 'O';
                FB_ARRAY[Array_Index+4] = 'K'; 
                FB_ARRAY[Array_Index+5] = '\r';
                FB_ARRAY[Array_Index+6] = '\n';
                Array_Index+=7;
                break;
              case 6:
                FB_ARRAY[Array_Index+0] = 'T';
                FB_ARRAY[Array_Index+1] = 'W';           
                FB_ARRAY[Array_Index+2] = 'A';
                FB_ARRAY[Array_Index+3] = 'R';
                FB_ARRAY[Array_Index+4] = 'N';
                FB_ARRAY[Array_Index+5] = ' ';
                FB_ARRAY[Array_Index+6] = 'O';
                FB_ARRAY[Array_Index+7] = 'K';
                FB_ARRAY[Array_Index+8] = '\r';
                FB_ARRAY[Array_Index+9] = '\n';
                 Array_Index+=10;
                break;
              case 7:
                FB_ARRAY[Array_Index+0] = 'D';
                FB_ARRAY[Array_Index+1] = 'C';           
                FB_ARRAY[Array_Index+2] = '/';
                FB_ARRAY[Array_Index+3] = 'D';
                FB_ARRAY[Array_Index+4] = 'C';
                FB_ARRAY[Array_Index+5] = ' ';
                FB_ARRAY[Array_Index+6] = 'F';
                FB_ARRAY[Array_Index+7] = 'A';
                FB_ARRAY[Array_Index+8] = 'I';
                FB_ARRAY[Array_Index+9] = 'L';
                FB_ARRAY[Array_Index+10] = '\r';
                FB_ARRAY[Array_Index+11] = '\n';
                Array_Index+=12;
                break;
               }
            }
            else
            {
              switch (Out_index)
              {
              case 4:
                FB_ARRAY[Array_Index+0] = 'P';
                FB_ARRAY[Array_Index+1] = 'O';
                FB_ARRAY[Array_Index+2] = 'W';
                FB_ARRAY[Array_Index+3] = 'E';           
                FB_ARRAY[Array_Index+4] = 'R';
                FB_ARRAY[Array_Index+5] = ' ';
                FB_ARRAY[Array_Index+6] = 'G';
                FB_ARRAY[Array_Index+7] = 'O';
                FB_ARRAY[Array_Index+8] = 'O';
                FB_ARRAY[Array_Index+9] = 'D';           
                FB_ARRAY[Array_Index+10] = ' ';
                FB_ARRAY[Array_Index+11] = 'F';
                FB_ARRAY[Array_Index+12] = 'A';
                FB_ARRAY[Array_Index+13] = 'I';
                FB_ARRAY[Array_Index+14] = 'L';
                FB_ARRAY[Array_Index+15] = '\r';
                FB_ARRAY[Array_Index+16] = '\n';
                Array_Index+=17;
                break;
              case 5:
                FB_ARRAY[Array_Index+0] = 'P';
                FB_ARRAY[Array_Index+1] = 'C';           
                FB_ARRAY[Array_Index+2] = ' ';
                FB_ARRAY[Array_Index+3] = 'F';
                FB_ARRAY[Array_Index+4] = 'A';
                FB_ARRAY[Array_Index+5] = 'I';
                FB_ARRAY[Array_Index+6] = 'L';
                FB_ARRAY[Array_Index+7] = '\r';
                FB_ARRAY[Array_Index+8] = '\n';
                Array_Index+=9;  
                break;
              case 6:
                FB_ARRAY[Array_Index+0] = 'T';
                FB_ARRAY[Array_Index+1] = 'W';           
                FB_ARRAY[Array_Index+2] = 'A';
                FB_ARRAY[Array_Index+3] = 'R';
                FB_ARRAY[Array_Index+4] = 'N';
                FB_ARRAY[Array_Index+5] = ' ';
                FB_ARRAY[Array_Index+6] = 'F';
                FB_ARRAY[Array_Index+7] = 'A';
                FB_ARRAY[Array_Index+8] = 'I';
                FB_ARRAY[Array_Index+9] = 'L';
                FB_ARRAY[Array_Index+10] = '\r';
                FB_ARRAY[Array_Index+11] = '\n';
                Array_Index+=12;
                break;
              case 7:
                FB_ARRAY[Array_Index+0] = 'D';
                FB_ARRAY[Array_Index+1] = 'C';           
                FB_ARRAY[Array_Index+2] = '/';
                FB_ARRAY[Array_Index+3] = 'D';
                FB_ARRAY[Array_Index+4] = 'C';
                FB_ARRAY[Array_Index+5] = ' ';
                FB_ARRAY[Array_Index+6] = 'O';
                FB_ARRAY[Array_Index+7] = 'K';
                FB_ARRAY[Array_Index+8] = '\r';
                FB_ARRAY[Array_Index+9] = '\n';
                Array_Index+=10;
                break;
              }              
            }


}

/** 
  * @brief:     Get CLT01 IC status
  * @param:     Output index
*/

void GET_CLT_FB( uint8_t Out_index)
{

               
           device_FB=(FB_Data1[0]>>Out_index+6)&0x01; 
//      device_FB=0;//TEST
            if(device_FB==0)
            {
              switch (Out_index)
              {
                case 0:
                FB_ARRAY[Array_Index+0] = 'T';
                FB_ARRAY[Array_Index+1] = 'E';
                FB_ARRAY[Array_Index+2] = 'M';
                FB_ARRAY[Array_Index+3] = 'P';           
                FB_ARRAY[Array_Index+4] = 'E';
                FB_ARRAY[Array_Index+5] = 'R';
                FB_ARRAY[Array_Index+6] = 'A';
                FB_ARRAY[Array_Index+7] = 'T';
                FB_ARRAY[Array_Index+8] = 'U';
                FB_ARRAY[Array_Index+9] = 'R';           
                FB_ARRAY[Array_Index+10] = 'E';
                FB_ARRAY[Array_Index+11] = ' ';
                FB_ARRAY[Array_Index+12] = 'F';
                FB_ARRAY[Array_Index+13] = 'A';
                FB_ARRAY[Array_Index+14] = 'I';
                FB_ARRAY[Array_Index+15] = 'L';
                FB_ARRAY[Array_Index+16] = '\r';
                FB_ARRAY[Array_Index+17] = '\n';
                Array_Index+=18;
                  break;
                case 1:    
                  FB_ARRAY[Array_Index+0] = 'P';
                  FB_ARRAY[Array_Index+1] = 'O';
                  FB_ARRAY[Array_Index+2] = 'W';
                  FB_ARRAY[Array_Index+3] = 'E';           
                  FB_ARRAY[Array_Index+4] = 'R';
                  FB_ARRAY[Array_Index+5] = ' ';
                  FB_ARRAY[Array_Index+6] = 'G';
                  FB_ARRAY[Array_Index+7] = 'O';
                  FB_ARRAY[Array_Index+8] = 'O';
                  FB_ARRAY[Array_Index+9] = 'D';           
                  FB_ARRAY[Array_Index+10] = ' ';
                  FB_ARRAY[Array_Index+11] = 'F';
                  FB_ARRAY[Array_Index+12] = 'A';
                  FB_ARRAY[Array_Index+13] = 'I';
                  FB_ARRAY[Array_Index+14] = 'L';
                  FB_ARRAY[Array_Index+15] = '\r';
                  FB_ARRAY[Array_Index+16] = '\n';
                  Array_Index+=17;
                  break;
              }
              
            }
            else
            {
              switch (Out_index)
              {
                case 0:
                FB_ARRAY[Array_Index+0] = 'T';
                FB_ARRAY[Array_Index+1] = 'E';
                FB_ARRAY[Array_Index+2] = 'M';
                FB_ARRAY[Array_Index+3] = 'P';           
                FB_ARRAY[Array_Index+4] = 'E';
                FB_ARRAY[Array_Index+5] = 'R';
                FB_ARRAY[Array_Index+6] = 'A';
                FB_ARRAY[Array_Index+7] = 'T';
                FB_ARRAY[Array_Index+8] = 'U';
                FB_ARRAY[Array_Index+9] = 'R';           
                FB_ARRAY[Array_Index+10] = 'E';
                FB_ARRAY[Array_Index+11] = ' ';
                FB_ARRAY[Array_Index+12] = 'O';
                FB_ARRAY[Array_Index+13] = 'K';
                FB_ARRAY[Array_Index+14] = '\r';
                FB_ARRAY[Array_Index+15] = '\n';
                Array_Index+=16;
                  break;
                case 1:
                  FB_ARRAY[Array_Index+0] = 'P';
                  FB_ARRAY[Array_Index+1] = 'O';
                  FB_ARRAY[Array_Index+2] = 'W';
                  FB_ARRAY[Array_Index+3] = 'E';           
                  FB_ARRAY[Array_Index+4] = 'R';
                  FB_ARRAY[Array_Index+5] = ' ';
                  FB_ARRAY[Array_Index+6] = 'G';
                  FB_ARRAY[Array_Index+7] = 'O';
                  FB_ARRAY[Array_Index+8] = 'O';
                  FB_ARRAY[Array_Index+9] = 'D';           
                  FB_ARRAY[Array_Index+10] = ' ';
                  FB_ARRAY[Array_Index+11] = 'O';
                  FB_ARRAY[Array_Index+12] = 'K'; 
                  FB_ARRAY[Array_Index+13] = '\r';
                  FB_ARRAY[Array_Index+14] = '\n'; 
                  Array_Index+=15; 
                 break;
              }
            }

}









/** 
  * @brief Error Handler when Error come and Toggling of LED2 takes place
  * @param None
  * @retval None
*/
void PLC_Error_Handler(void)
{
  while(1)
  {
    BSP_LED_Toggle(LED2);
    HAL_Delay(1000);
  }
}

/** 
  * @brief Function to calculate PARITY BITS
  * @param None
  * @retval None
*/

void Parity_bits_VNI(void)
{
  Parity_Cal0 = VNI_TxBuffer[1] & 0x80;;
  Parity_Cal0 = Parity_Cal0>>7;
  
  Parity_Cal1 = VNI_TxBuffer[1] & 0x40;
  Parity_Cal1 = Parity_Cal1>>6;
  
  Parity_Cal2 = VNI_TxBuffer[1] & 0x20;
  Parity_Cal2 = Parity_Cal2>>5;
  
  Parity_Cal3 = VNI_TxBuffer[1] & 0x10;
  Parity_Cal3 = Parity_Cal3>>4;
  
  Parity_Cal4 = VNI_TxBuffer[1] & 0x08;
  Parity_Cal4 = Parity_Cal4>>3;
  
  Parity_Cal5 = VNI_TxBuffer[1] & 0x04;
  Parity_Cal5 = Parity_Cal5>>2;
  
  Parity_Cal6 = VNI_TxBuffer[1] & 0x02;
  Parity_Cal6 = Parity_Cal6>>1;
  
  Parity_Cal7 = VNI_TxBuffer[1] & 0x01;
 
  
  /* Caluculate parity bits based on output data */
  P2 = ((Parity_Cal7^Parity_Cal5)^Parity_Cal3)^Parity_Cal1;
  if(P2 == 0x01) 
    P2 = 0x08;
  else
    P2 = 0x00;
 
  P1 = ((Parity_Cal6^Parity_Cal4)^Parity_Cal2)^Parity_Cal0;
  if(P1 == 0x01)
    P1 = 0x04;
  else
    P1 = 0x00;
  
  P0 = ((((((Parity_Cal7^Parity_Cal6)^Parity_Cal5)^Parity_Cal4)^Parity_Cal3)
         ^Parity_Cal2)^Parity_Cal1)^Parity_Cal0;
  if(P0 == 0x01)
    P0 = 0X02;
  else
    P0 = 0x00;
   
  nP0 = 0x00;
  if(P0 == 0x02)
    nP0 = 0x00;
  else
    nP0 = 0x01;
  
  /* Set VNI_TxBuffer parity bits field */
  VNI_TxBuffer[0] = P2|P1|P0|nP0;
  
}


#ifdef OUTPUT_CYCLING
/**
  * @brief  PWM Pulse finished callback in non blocking mode 
  * @param  htim : TIM handle
  * @retval None
  */
void HAL_TIM_PWM_PulseFinishedCallback(TIM_HandleTypeDef *htim)
{
  /* frequency range is betwenn 50 Hz to 2kHz with duty cycle 10% to 90% */
  //Output_Cycling(0x00);//0x00 corresponds to all channels OFF
}

/**
  * @brief  Period elapsed callback in non blocking mode 
  * @param  htim : TIM handle
  * @retval None
  */
//void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
//{
//  /* frequency range is betwenn 50 Hz to 2kHz with duty cycle 10% to 90% */
// // Output_Cycling(0xFF);//0xFF corresponds to all channels ON
//}

#endif /* OUTPUT_CYCLING */

/**
  * @}// end 
  */






#ifdef USE_FULL_ASSERT

/**
   * @brief Reports the name of the source file and the source line number
   * where the assert_param error has occurred.
   * @param file: pointer to the source file name
   * @param line: assert_param error line source number
   * @retval None
   */
void assert_failed(uint8_t* file, uint32_t line)
{

  /* User can add his own implementation to report the file name and line number,
    ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */


}

#endif /* OUTPUT_CYCLING */

/**
  * @} // end 
  */

/**
  * @} // end 
  */

/**
  * @} // end 
  */

/**
  * @} // end 
  */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
