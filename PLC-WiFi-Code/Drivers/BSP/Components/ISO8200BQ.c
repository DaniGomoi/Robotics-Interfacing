/**
  ******************************************************************************
  * @file               : ISO8200BQ.c
  * @Author				: IPD System LAB - Automation and Motion Control Team
  * @Version			: V1.0.0
  * @Date               : 05/11/2015 
  * @Brief		        : Source file related to device driving 
           
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



#include "stdint.h"
#include "stdbool.h"
#include "ISO8200BQ.h"





/** @addtogroup Drivers     Drivers
  * @{
  * @brief Demo Driver Layer
  */    
  
/** @addtogroup BSP     BSP
  * @{
  * @brief Board support package
  */    
    
/** @addtogroup Components     Components
  * @{
  * @brief Component layer
  */

/** @defgroup ISO8200BQ    ISO8200BQ
  * @{ 
  * @brief Device  Layer
  */


/** @defgroup Component_TypeDef    Component TypeDef
  * @{ 
  * @brief Struct pointer function 
  */

IsoComponent_TypeDef PinHandle=
{
     ISO_DrivePin,
     ISO_DriveLoad,    
     ISO_DriveSinch  
};

/**
  * @} //Component TypeDef
  */

/** @defgroup ISO8200BQ_Exported_Functions     ISO8200BQ Exported Functions
  *  @{
    * @brief Device Driving exported Function 
  */
    
    
/**
  * @brief  Drives  Load signal             
  * @param  GPIOx: gpio port corresponding to Load pin
  * @param  GPIO_Pin: gpio pin number corresponding to Load pin 
  * @param  PinState: gpio pin state
  *         GPIO_PIN_RESET : Load Low
  *         GPIO_PIN_SET : Load High
  * 
  */

void ISO_DriveLoad(void* GPIOx, uint16_t GPIO_Pin,uint8_t PinState)
{
   ISO8200BQ_IO_Write(GPIOx, GPIO_Pin, PinState);
}

/**
  * @brief   Drives  Synch signal  
  * @param   GPIOx: gpio port corresponding to Synch pin
  * @param  GPIO_Pin: gpio pin number corresponding to Synch pin 
  * @param  PinState: gpio pin state
  *         GPIO_PIN_RESET : Synch Low
  *         GPIO_PIN_SET : Synch High
  * 
  */
void ISO_DriveSinch (void* GPIOx, uint16_t GPIO_Pin,uint8_t PinState)
{
   ISO8200BQ_IO_Write(GPIOx, GPIO_Pin, PinState);
}


/**
  * @brief  Drives the Input pin of ISO8200BQ corresponding to device's outputs             
  * @param  GPIOx: gpio port corresponding to inputs pin
  * @param  GPIO_Pin: gpio pin number corresponding to inputs pin
  * @param  PinState: gpio pin state
  *         GPIO_PIN_RESET : Output OFF
  *         GPIO_PIN_SET : OutputON
  * 
*/

void ISO_DrivePin(void* GPIOx, uint16_t GPIO_Pin,uint8_t PinState)
{
	ISO8200BQ_IO_Write(GPIOx, GPIO_Pin, PinState);
}

/**
  * @} //end ISO8200BQ Exported Functions
  */

/**
  * @}  //end ISO8200BQ
  */
/**
  * @}  //end Components
   */
/**
  * @}  //close group  BSP
  */

/**
  * @}  //close group  Drivers
  */



/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
