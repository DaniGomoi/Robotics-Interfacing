/**
  ******************************************************************************
  * @file               : X_NUCLEO_01A1.c
  * @Author	        : IPD System LAB - Automation and Motion Control Team
  * @Version		: V1.0.0
  * @Date               : 05/11/2015 
  * @Brief        	: Source files related to Expansion board routines
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; COPYRIGHT(c) 2014 STMicroelectronics</center></h2>
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
//
//#ifdef STM32F401xE
#include "stm32f4xx_hal.h"
#include "stm32f4xx_hal_def.h"
#include "stm32f4xx_hal_gpio.h"
//#endif




#include "stdint.h"
#include "stdbool.h"
#include "ISO8200BQ.h"
#include "X_NUCLEO_01A1.h"




/** @addtogroup Drivers     Drivers
  * @{
  * @brief Demo Driver Layer
  */ 
/** @addtogroup BSP     BSP
  * @{
  * @brief Boar support package layer
  */ 


/** @addtogroup X_Nucleo     X_Nucleo
  * @brief   X-Nucleo Layer
  *@{
  */

/** @addtogroup X_Nucleo_01A1     X_Nucleo_01A1
  * @brief    X-Nucleo Driver Layer
  *@{
  */

/** @defgroup X_Nucleo_01A1_Private_variables       X_Nucleo_01A1 Private variables
  * @brief  
  * @{
  */


uint8_t var_bit=0;      /*!< temporany variable*/
uint8_t bit=0;          /*!< temporany variable*/
bool SYNCH;                      /*!< exported boolean variables*/
/**
  * @}
  */



/** @defgroup X_Nucleo_01A1_Exported_variables  X_Nucleo_01A1  Exported variables 
  *  @{
  */


extern bool timer;                       /*!< exported boolean variables*/
GPIO_PinState GPIO_ISO_DRV;      /*!< exported status variables */
extern GPIO_PinState CH_STATE;           /*!< exported status variables */
extern GET_DRV gpio_value ;              /*!< exported status variables */
extern TIM_HandleTypeDef htim10;         /*!< exported library variable definition*/
extern IsoComponent_TypeDef PinHandle;

/**
  * @}
  */


/** @defgroup X_Nucleo_01A1_Exported_Functions   X_Nucleo_01A1  Exported Functions
  *  @{
  */

/**
  * @brief  Check communication settings with ISO8200BQ , DIRECT or Sinchronous access
  * 
*/
void BSP_ISO_Com_Settings(void)

{
 
 GPIO_ISO_DRV= HAL_GPIO_ReadPin(ISO_DRIVING,ISO_DRIVING_PIN);

  if(GPIO_ISO_DRV==GPIO_PIN_SET)
  {
    SYNCH=true;
  }
  else
  {
    SYNCH=false;
  }

  gpio_value=CHECKED;
}
/**
  * @brief  Initialize GPIOB Pin0 value.
  * @param  status: status value must be set to set or reset the OUT_EN device pin
  *         GPIO_PIN_SET : OUT_EN high
  *         GPIO_PIN_RESET: OUT_EN low
  * 
*/
void BSP_OutputEnable_Pin(GPIO_PinState status)
{   
 PinHandle.Pin_Drive(OUT_EN,OUT_EN_PIN,status);
}

/**
  * @brief  Forced all GPIOs connected to the device's inputs to High value.
  * 
  */

void BSP_AllPin_High(void)
{   
     if(SYNCH==false)
     {  
      PinHandle.Pin_Load (LOAD, LOAD_PIN, GPIO_PIN_RESET);//LOAD
      PinHandle.Pin_Sinch (SINCH, SINCH_PIN, GPIO_PIN_RESET);//SINCH         
      PinHandle.Pin_Drive(IN1, IN1_PIN, GPIO_PIN_SET);
      
#ifdef DO_PLC     
      PinHandle.Pin_Drive(IN2, IN2_PIN, GPIO_PIN_SET);
      PinHandle.Pin_Drive(IN3, IN3_PIN, GPIO_PIN_SET); 
      PinHandle.Pin_Drive(IN4, IN4_PIN, GPIO_PIN_SET); 
#endif
      
      PinHandle.Pin_Drive(IN5, IN5_PIN, GPIO_PIN_SET);
      PinHandle.Pin_Drive(IN6, IN6_PIN, GPIO_PIN_SET);
      PinHandle.Pin_Drive(IN7, IN7_PIN, GPIO_PIN_SET);
      PinHandle.Pin_Drive(IN8, IN8_PIN, GPIO_PIN_SET);
     }
     else
     {
      PinHandle.Pin_Load (LOAD, LOAD_PIN, GPIO_PIN_SET);//LOAD
      PinHandle.Pin_Sinch(SINCH, SINCH_PIN, GPIO_PIN_SET);//SINCH  
      PinHandle.Pin_Load(LOAD, LOAD_PIN, GPIO_PIN_RESET);//LOAD           
   
      PinHandle.Pin_Drive(IN1, IN1_PIN, GPIO_PIN_SET);
  #ifdef DO_PLC  
      PinHandle.Pin_Drive(IN2, IN2_PIN, GPIO_PIN_SET);
      PinHandle.Pin_Drive(IN3, IN3_PIN, GPIO_PIN_SET); 
     PinHandle.Pin_Drive(IN4, IN4_PIN, GPIO_PIN_SET); 
  #endif    
      PinHandle.Pin_Drive(IN5, IN5_PIN, GPIO_PIN_SET);
      PinHandle.Pin_Drive(IN6, IN6_PIN, GPIO_PIN_SET);
      PinHandle.Pin_Drive(IN7, IN7_PIN, GPIO_PIN_SET);
      PinHandle.Pin_Drive(IN8, IN8_PIN, GPIO_PIN_SET);


      PinHandle.Pin_Load(LOAD, LOAD_PIN, GPIO_PIN_SET);//LOAD
      PinHandle.Pin_Sinch(SINCH, SINCH_PIN, GPIO_PIN_RESET);//SINCH  
      PinHandle.Pin_Sinch(SINCH, SINCH_PIN, GPIO_PIN_RESET);//SINCH 
      PinHandle.Pin_Sinch(SINCH, SINCH_PIN, GPIO_PIN_RESET);//SINCH 
      PinHandle.Pin_Sinch(SINCH, SINCH_PIN, GPIO_PIN_SET);//SINCH         
       
     }
}

/**
  * @brief  Forced all GPIOs connected to the device's inputs to Low value.
  * 
  */
void BSP_AllPin_Low(void)
{  

     if(SYNCH==false)
     {  
      PinHandle.Pin_Load (LOAD, LOAD_PIN, GPIO_PIN_RESET);//LOAD
      PinHandle.Pin_Sinch (SINCH, SINCH_PIN, GPIO_PIN_RESET);//SINCH     
  
        PinHandle.Pin_Drive(IN1, IN1_PIN, GPIO_PIN_RESET);
#ifdef DO_PLC       
        PinHandle.Pin_Drive(IN2, IN2_PIN, GPIO_PIN_RESET);
        PinHandle.Pin_Drive(IN3, IN3_PIN, GPIO_PIN_RESET); 
        PinHandle.Pin_Drive(IN4, IN4_PIN, GPIO_PIN_RESET); 
#endif
        PinHandle.Pin_Drive(IN5, IN5_PIN, GPIO_PIN_RESET);
        PinHandle.Pin_Drive(IN6, IN6_PIN, GPIO_PIN_RESET);
        PinHandle.Pin_Drive(IN7, IN7_PIN, GPIO_PIN_RESET);
        PinHandle.Pin_Drive(IN8, IN8_PIN, GPIO_PIN_RESET);


     }
     else
     {
       PinHandle.Pin_Load(LOAD, LOAD_PIN, GPIO_PIN_SET);//LOAD
       PinHandle.Pin_Sinch(SINCH, SINCH_PIN, GPIO_PIN_SET);//SINCH  
       PinHandle.Pin_Load(LOAD, LOAD_PIN, GPIO_PIN_RESET);//LOAD           
    
      PinHandle.Pin_Drive(IN1, IN1_PIN, GPIO_PIN_RESET);
#ifdef DO_PLC        
      PinHandle.Pin_Drive(IN2, IN2_PIN, GPIO_PIN_RESET);
      PinHandle.Pin_Drive(IN3, IN3_PIN, GPIO_PIN_RESET); 
      PinHandle.Pin_Drive(IN4, IN4_PIN, GPIO_PIN_RESET); 
#endif 
      PinHandle.Pin_Drive(IN5, IN5_PIN, GPIO_PIN_RESET);
      PinHandle.Pin_Drive(IN6, IN6_PIN, GPIO_PIN_RESET);
      PinHandle.Pin_Drive(IN7, IN7_PIN, GPIO_PIN_RESET);
      PinHandle.Pin_Drive(IN8, IN8_PIN, GPIO_PIN_RESET);

     
      
      PinHandle.Pin_Load(LOAD, LOAD_PIN, GPIO_PIN_SET);//LOAD
      PinHandle.Pin_Sinch(SINCH, SINCH_PIN, GPIO_PIN_RESET);//SINCH  
      PinHandle.Pin_Sinch(SINCH, SINCH_PIN, GPIO_PIN_RESET);//SINCH 
      PinHandle.Pin_Sinch(SINCH, SINCH_PIN, GPIO_PIN_RESET);//SINCH 
      PinHandle.Pin_Sinch(SINCH, SINCH_PIN, GPIO_PIN_SET);//SINCH     
       
     }

}

/**
  * @brief  Drives in group mode the GPIOx connected to the device's inputs 
  * and to Load and Synch signal according to ISO communication settings.
  * @param  channel: device output configuration
  * 
  */

void BSP_DrivePin_GROUP(uint8_t channel)
{  
 //int8_t ret=-1;
           if(SYNCH==false)
           {  
            PinHandle.Pin_Load (LOAD, LOAD_PIN, GPIO_PIN_RESET);//LOAD
            PinHandle.Pin_Sinch (SINCH, SINCH_PIN, GPIO_PIN_RESET);//SINCH   
           }
           else
           {
            PinHandle.Pin_Load(LOAD, LOAD_PIN, GPIO_PIN_SET);//LOAD
            PinHandle.Pin_Sinch(SINCH, SINCH_PIN, GPIO_PIN_SET);//SINCH  
            PinHandle.Pin_Load(LOAD, LOAD_PIN, GPIO_PIN_RESET);//LOAD  
           }
          for(bit=0;bit<8;bit++)
          {
            var_bit=(channel>>bit)&0x01;
            switch (bit)//channel
               {
                   case 0: 

                       if(var_bit==0)
                       {                  
                            PinHandle.Pin_Drive(IN1, IN1_PIN, GPIO_PIN_RESET);                   

                       }
                       else
                       {
                         if(var_bit==1)
                         { 
                              PinHandle.Pin_Drive(IN1, IN1_PIN, GPIO_PIN_SET); 
                         }  
                       }
                       
                       break;
#ifdef DO_PLC
                   case 1: 

                      if(var_bit==0)
                       {
                               PinHandle.Pin_Drive(IN2, IN2_PIN, GPIO_PIN_RESET);   
                                 
                       }
                       else
                       {
                          if(var_bit==1)
                           {
                                PinHandle.Pin_Drive(IN2, IN2_PIN, GPIO_PIN_SET); 
                                       
                           } 
                       }
                          break;
                  case 2:

                         if(var_bit==0)
                         {
                                     PinHandle.Pin_Drive(IN3, IN3_PIN, GPIO_PIN_RESET);  
                          }
                         else
                         {
                           if(var_bit==1)
                             {
                                       PinHandle.Pin_Drive(IN3, IN3_PIN, GPIO_PIN_SET); 
                              }
                         }
                         break;
                   case 3: 

                        if(var_bit==0)
                         {

                               PinHandle.Pin_Drive(IN4, IN4_PIN, GPIO_PIN_RESET);   
                      
                          }
                        else
                        {
                           if(var_bit==1)
                           {
     
                               PinHandle.Pin_Drive(IN4, IN4_PIN, GPIO_PIN_SET);   
                                           
                            } 
                        }
                         break;
#endif
                        case 4: 
                       if(var_bit==0)
                         {
                            PinHandle.Pin_Drive(IN5, IN5_PIN, GPIO_PIN_RESET);
                         }
                          else 
                         {
                          if(var_bit==1)
                             {
                            PinHandle.Pin_Drive(IN5, IN5_PIN, GPIO_PIN_SET); 
                             }
                         }  
                       
                         break;
                   case 5: 
                         if(var_bit==0)
                         {
                            PinHandle.Pin_Drive(IN6, IN6_PIN, GPIO_PIN_RESET);
                         }
                          else 
                         {
                            if(var_bit==1)
                             {
                                PinHandle.Pin_Drive(IN6, IN6_PIN, GPIO_PIN_SET); 
                             }
                         }  
                         
                         break;
                   case 6: 
                       if(var_bit==0)
                         {
                            PinHandle.Pin_Drive(IN7, IN7_PIN, GPIO_PIN_RESET);
                         }
                          else 
                         {
                            if(var_bit==1)
                             {
                                PinHandle.Pin_Drive(IN7, IN7_PIN, GPIO_PIN_SET);
                             }
                         }                        
                         break;
                   case 7: 
                        if(var_bit==0)
                         {
                            PinHandle.Pin_Drive(IN8, IN8_PIN, GPIO_PIN_RESET);
                         }
                          else 
                         {
                           if(var_bit==1)
                             {
                                PinHandle.Pin_Drive(IN8, IN8_PIN, GPIO_PIN_SET); 
                             }
                         }                       
                         break;
                 }
              }
              if(SYNCH==true)
               {
                PinHandle.Pin_Load(LOAD, LOAD_PIN, GPIO_PIN_RESET);//LOAD
                PinHandle.Pin_Load(LOAD, LOAD_PIN, GPIO_PIN_RESET);//LOAD
                PinHandle.Pin_Load(LOAD, LOAD_PIN, GPIO_PIN_SET);//LOAD
                PinHandle.Pin_Sinch(SINCH, SINCH_PIN, GPIO_PIN_RESET);//SYNCH  
                PinHandle.Pin_Sinch(SINCH, SINCH_PIN, GPIO_PIN_RESET);//SYNCH 
                PinHandle.Pin_Sinch(SINCH, SINCH_PIN, GPIO_PIN_RESET);//SYNCH 
                PinHandle.Pin_Sinch(SINCH, SINCH_PIN, GPIO_PIN_SET);//SINCH   
                PinHandle.Pin_Sinch(SINCH, SINCH_PIN, GPIO_PIN_SET);//SINCH
                PinHandle.Pin_Sinch(SINCH, SINCH_PIN, GPIO_PIN_SET);//SINCH
                
                //ret=0;
               }
   // return ret; 
}


/**
  * @brief  Drives in single mode the GPIOx connected to the device's inputs 
  *         and the Load and Synch signal according to ISO communication settings.
  * @param  channel: device output
  * @param  Status: device outputs status
  *             0 : outputs OFF
  *             1 : outputs ON 
  * @retval None
  */

void BSP_DrivePin_SINGLE(uint8_t channel,uint8_t Status)
{  
           if(SYNCH==false)
           {  
            PinHandle.Pin_Load(LOAD, LOAD_PIN, GPIO_PIN_RESET);//LOAD
            PinHandle.Pin_Sinch(SINCH, SINCH_PIN, GPIO_PIN_RESET);//SINCH  
           }
           else
           {
            PinHandle.Pin_Load(LOAD, LOAD_PIN, GPIO_PIN_SET);//LOAD
             PinHandle.Pin_Sinch(SINCH, SINCH_PIN, GPIO_PIN_SET);//SINCH  
            PinHandle.Pin_Load(LOAD, LOAD_PIN, GPIO_PIN_RESET);//LOAD  
           }
            
            switch (channel)//channel
               {
                   case 1: 

                       if(Status==0)
                       {                  
                            PinHandle.Pin_Drive(IN1, IN1_PIN, GPIO_PIN_RESET);                   

                       }
                       else
                       {
                         if(Status==1)
                         { 
                              PinHandle.Pin_Drive(IN1, IN1_PIN, GPIO_PIN_SET); 
                         }  
                       }
                       
                       break;
#ifdef DO_PLC
                   case 2: 

                      if(Status==0)
                       {
                               PinHandle.Pin_Drive(IN2, IN2_PIN, GPIO_PIN_RESET);   
                                 
                       }
                       else
                       {
                          if(Status==1)
                           {
                                PinHandle.Pin_Drive(IN2, IN2_PIN, GPIO_PIN_SET); 
                                       
                           } 
                       }
                          break;
                  case 3:

                         if(Status==0)
                         {
                                     PinHandle.Pin_Drive(IN3, IN3_PIN, GPIO_PIN_RESET);  
                          }
                         else
                         {
                           if(Status==1)
                             {
                                       PinHandle.Pin_Drive(IN3, IN3_PIN, GPIO_PIN_SET); 
                              }
                         }
                         break;
                   case 4: 

                        if(Status==0)
                         {

                              PinHandle.Pin_Drive(IN4, IN4_PIN, GPIO_PIN_RESET);   
                      
                          }
                        else
                        {
                           if(Status==1)
                           {
     
                               PinHandle.Pin_Drive(IN4, IN4_PIN, GPIO_PIN_SET);   
                                           
                            } 
                        }
                         break;
#endif
                        case 5: 
                       if(Status==0)
                         {
                            PinHandle.Pin_Drive(IN5, IN5_PIN, GPIO_PIN_RESET);
                         }
                          else 
                         {
                          if(Status==1)
                             {
                            PinHandle.Pin_Drive(IN5, IN5_PIN, GPIO_PIN_SET); 
                             }
                         }  
                       
                         break;
                   case 6: 
                         if(Status==0)
                         {
                            PinHandle.Pin_Drive(IN6, IN6_PIN, GPIO_PIN_RESET);
                         }
                          else 
                         {
                            if(Status==1)
                             {
                                PinHandle.Pin_Drive(IN6, IN6_PIN, GPIO_PIN_SET); 
                             }
                         }  
                         
                         break;
                   case 7: 
                       if(Status==0)
                         {
                            PinHandle.Pin_Drive(IN7, IN7_PIN, GPIO_PIN_RESET);
                         }
                          else 
                         {
                            if(Status==1)
                             {
                                PinHandle.Pin_Drive(IN7, IN7_PIN, GPIO_PIN_SET);
                             }
                         }                        
                         break;
                   case 8: 
                        if(Status==0)
                         {
                            PinHandle.Pin_Drive(IN8, IN8_PIN, GPIO_PIN_RESET);
                         }
                          else 
                         {
                           if(Status==1)
                             {
                                PinHandle.Pin_Drive(IN8, IN8_PIN, GPIO_PIN_SET); 
                             }
                         }                       
                         break;
                 }
             
              if(SYNCH==true)
               {
                PinHandle.Pin_Load(LOAD, LOAD_PIN, GPIO_PIN_RESET);//LOAD
                PinHandle.Pin_Load(LOAD, LOAD_PIN, GPIO_PIN_RESET);//LOAD
                PinHandle.Pin_Load(LOAD, LOAD_PIN, GPIO_PIN_RESET);//LOAD
                
                PinHandle.Pin_Sinch(SINCH, SINCH_PIN, GPIO_PIN_RESET);//SYNCH  
                PinHandle.Pin_Sinch(SINCH, SINCH_PIN, GPIO_PIN_RESET);//SYNCH 
                PinHandle.Pin_Sinch(SINCH, SINCH_PIN, GPIO_PIN_RESET);//SYNCH 
                PinHandle.Pin_Sinch(SINCH, SINCH_PIN, GPIO_PIN_SET);//SINCH         
               }
     
}


/**
  * @brief  Routine for Component pin driving
  * @param  GPIOx: gpio port corresponding to Load pin
  * @param  GPIO_Pin: gpio pin number corresponding to Load pin 
  * @param  PinState: gpio pin state
  *         GPIO_PIN_RESET : Load Low
  *         GPIO_PIN_SET : Load High
  */

void ISO8200BQ_IO_Write(void* GPIOx, uint16_t GPIO_Pin, uint8_t PinState)
{
  HAL_GPIO_WritePin((GPIO_TypeDef*) GPIOx,GPIO_Pin,(GPIO_PinState)PinState);
  
}



/**
  * @}      //end X_Nucleo_01A1_Exported Function
  */

/**
  * @}      // end X_Nucleo_01A1
  */


/**
  * @}      // end X-Nucleo
*/

/**
  * @}      // end BSP 
  */

/**
  * @}      // Drivers
  */
 /************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/        







