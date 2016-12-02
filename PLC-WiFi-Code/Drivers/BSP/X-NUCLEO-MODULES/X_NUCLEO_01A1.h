/**
  ******************************************************************************
  * @File               : X_NUCLEO_01A1.h
  * @Author	        : IPD System LAB - Automation and Motion Control Team
  * @Version		: V1.0.0
  * @Date               : 05/11/2015 
  * @Brief        	: Header files related to Expansion board routines
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



/** @addtogroup   X_Nucleo_01A1    X_Nucleo_01A1  
  *  @{
  */

/** @addtogroup X_Nucleo_01A1_Exported_Functions   X_Nucleo_01A1  Exported Functions
  *  @{
  */

void BSP_ISO_Com_Settings(void);
void BSP_DrivePin_GROUP(uint8_t channel);
void BSP_DrivePin_SINGLE(uint8_t channel,uint8_t Status);
void BSP_AllPin_Low(void);
void BSP_AllPin_High(void);
void BSP_OutputEnable_Pin(GPIO_PinState status);

/**
  * @}
  */


/** @defgroup   X_Nucleo_01A1_Exported_types    X_Nucleo_01A1  Exported types
  *  @{
  */

    /** 
      * @brief  GPIO Bit SET and Bit RESET to identify access driving
      */

typedef enum     
                                                  
{
  CHECKED   =0, 
  UNCHECKED=1  
}GET_DRV;

      /** 
        * @brief  User push-button state to select output driving modality
        */


 typedef enum
{
  Pressed   =0, 
  NotPressed=1  
}State_Button;   

/**
  * @}
  */

/** @defgroup   X_Nucleo_01A1_Exported_constant    X_Nucleo_01A1  Exported constant
  *  @{
*/

#define DO_PLC                                 /*!<connetion with X-Nucleo-PLC01A1*/
//#define DO_Motor                              /*!<connection with IHM07*/



#ifdef DO_PLC                                 /*!<MCU GPIO configuration if X-Nucleo-PLC01A1
                                                    is connected to this shield*/ 

#define LOAD            GPIOC                    /*!< MCU GPIO used for LOAD signal*/
#define LOAD_PIN        GPIO_PIN_0              /*!<MCU GPIO PIN used for */

#define SINCH           GPIOC                   /*!<MCU GPIO used for Sinch signal*/
#define SINCH_PIN       GPIO_PIN_1               /*!<MCU GPIO PIN used for Sinch signal*/


#define ISO_DRIVING       GPIOA                 /*!<MCU GPIO used to set 
                                                    ISO8200BQ driving access, 
                                                    Direct or Sinchronous*/

#define ISO_DRIVING_PIN   GPIO_PIN_2             /*!<MCU GPIO PIN used to set 
                                                    ISO8200BQ driving access,
                                                    Direct or Sinchronous */


#define OUT_EN          GPIOB                   /*!<MCU GPIO used to manage ISO8200BQ
                                                    output enable signal*/
#define OUT_EN_PIN      GPIO_PIN_0               /*!<MCU GPIO PIN used to manage ISO8200BQ
                                                    output enable signal */

#define IN1             GPIOA                   /*!<MCU GPIO used to manage ISO8200BQ
                                                    input  */
#define IN1_PIN         GPIO_PIN_0               /*!<MCU GPIO PIN used to manage ISO8200BQ
                                                    input */

#define IN2             GPIOA                   /*!<MCU GPIO used to manage ISO8200BQ
                                                    input */ 
#define IN2_PIN         GPIO_PIN_1               /*!<MCU GPIO PIN used to manage ISO8200BQ
                                                    input */

#define IN3             GPIOA                  /*!<MCU GPIO used to manage ISO8200BQ
                                                    input */ 
#define IN3_PIN         GPIO_PIN_3              /*!<MCU GPIO PIN used to manage ISO8200BQ
                                                    input */ 

#define IN4             GPIOA                   /*!<MCU GPIO used to manage ISO8200BQ
                                                    input */ 
#define IN4_PIN         GPIO_PIN_4               /*!<MCU GPIO PIN used to manage ISO8200BQ
                                                    input */

#define IN5             GPIOB                   /*!<MCU GPIO used to manage ISO8200BQ
                                                    input */ 
#define IN5_PIN         GPIO_PIN_4               /*!<MCU GPIO PIN used to manage ISO8200BQ
                                                    input*/

#define IN6             GPIOB                   /*!<MCU GPIO used to manage ISO8200BQ
                                                    input*/ 
#define IN6_PIN         GPIO_PIN_5              /*!<MCU GPIO PIN used to manage ISO8200BQ
                                                    input */


#define IN7             GPIOB                   /*!<MCU GPIO used to manage ISO8200BQ
                                                    input */ 
#define IN7_PIN         GPIO_PIN_9              /*!<MCU GPIO PIN used to manage ISO8200BQ
                                                    input */

#define IN8             GPIOB                   /*!<MCU GPIO used to manage ISO8200BQ
                                                    input */ 
#define IN8_PIN         GPIO_PIN_8              /*!<MCU GPIO PIN used to manage ISO8200BQ
                                                    input */

#endif

#ifdef DO_Motor                                 /*!<MCU GPIO configuration if IHM07
                                                    is connected to this shield*/ 

#define LOAD            GPIOC                   /*!< MCU GPIO used for LOAD signal*/
#define LOAD_PIN        GPIO_PIN_0               /*!<MCU GPIO PIN used for */

#define SINCH           GPIOC                    /*!<MCU GPIO used for Sinch signal*/
#define SINCH_PIN       GPIO_PIN_7             /*!<MCU GPIO PIN used for Sinch signal*/

#define ISO_DRIVING       GPIOA                 /*!<MCU GPIO used to set 
                                                    ISO8200BQ driving access, 
                                                    Direct or Sinchronous*/
#define ISO_DRIVING_PIN   GPIO_PIN_2             /*!<MCU GPIO PIN used to set 
                                                    ISO8200BQ driving access,
                                                    Direct or Sinchronous */

#define OUT_EN          GPIOB                    /*!<MCU GPIO used to manage ISO8200BQ
                                                    output enable signal*/
#define OUT_EN_PIN      GPIO_PIN_6              /*!<MCU GPIO PIN used to manage ISO8200BQ
                                                    output enable signal */

#define IN1             GPIOA                   /*!<MCU GPIO used to manage ISO8200BQ
                                                    input */ 
#define IN1_PIN         GPIO_PIN_0              /*!<MCU GPIO PIN used to manage ISO8200BQ
                                                    input */

#define IN2             GPIOA                    /*!<MCU GPIO used to manage ISO8200BQ
                                                    input */ 
#define IN2_PIN         GPIO_PIN_1              /*!<MCU GPIO PIN used to manage ISO8200BQ
                                                    input */

#define IN3             GPIOA                   /*!<MCU GPIO used to manage ISO8200BQ
                                                    input */  
#define IN3_PIN         GPIO_PIN_3              /*!<MCU GPIO PIN used to manage ISO8200BQ
                                                    input */ 

#define IN4             GPIOA                   /*!<MCU GPIO used to manage ISO8200BQ
                                                    input */  
#define IN4_PIN         GPIO_PIN_4              /*!<MCU GPIO PIN used to manage ISO8200BQ
                                                    input */  

#define IN5             GPIOB                   /*!<MCU GPIO used to manage ISO8200BQ
                                                    input */  
#define IN5_PIN         GPIO_PIN_10            /*!<MCU GPIO PIN used to manage ISO8200BQ
                                                    input */ 

#define IN6             GPIOB                   /*!<MCU GPIO used to manage ISO8200BQ
                                                    input */  
#define IN6_PIN         GPIO_PIN_3              /*!<MCU GPIO PIN used to manage ISO8200BQ
                                                    input */ 


#define IN7             GPIOB                   /*!<MCU GPIO used to manage ISO8200BQ
                                                    input */ 
#define IN7_PIN         GPIO_PIN_9              /*!<MCU GPIO PIN used to manage ISO8200BQ
                                                    input */  

#define IN8             GPIOB                   /*!<MCU GPIO used to manage ISO8200BQ
                                                    input */  
#define IN8_PIN         GPIO_PIN_8              /*!<MCU GPIO PIN used to manage ISO8200BQ
                                                    input */  

#endif


/**
  * @} // end X_Nucleo_01A1   Exported constant
  */

/**
  * @} // end X_Nucleo_01A1
  */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
