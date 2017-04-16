/**
  ******************************************************************************
  * @file    PLC_Handling.h
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

/** @addtogroup       X_Nucleo_PLC_Handling_Exported_Define     X_Nucleo_PLC Handling Exported Define
  * @brief    
  * @{ 
  */

/** @defgroup X_Nucleo_PLC_Handling_Exported_Define     X_Nucleo_PLC Handling Exported Define
  * @brief    
  * @{ 
  */

//#define OUTPUT_CYCLING

#ifdef OUTPUT_CYCLING
/*      The frequency range of Output_cycling is expressed in Hz 
        with 10% duty cycle to 90% duty cycle                   */

#endif /* OUTPUT_CYCLING */
#define OUTPUT_CYCLING_FREQ  1
#define OUTPUT_CYCLING_DUTY    20

/**
  * @}// end 
  */

/** @defgroup X_Nucleo_PLC_Handling_TypeDef_Struct     X_Nucleo_PLC Handling TypeDef Struct
  * @brief    
  * @{ 
  */

typedef struct 
{
uint16_t Timer_Prescaler;
  uint16_t Timer_Period;
  uint16_t Timer_Duty;
  uint8_t Timer_freq;
  
}TimerParam_Typedef;
/**
  * @}// end 
  */




/** @addtogroup X_Nucleo_PLC_Handling_Exported_Functions     X_Nucleo_PLC Handling Exported Functions
  * @brief    
  * @{ 
  */


void PLC_Error_Handler(void);
int16_t CLT_VNI_RxTx(void); /*!< SPI transmission */ 
uint8_t VNI_TxRx(uint8_t OutputStage);
void Parity_bits_VNI(void);
void PLC_TIM2_Init(void);
void Calc_TimParam (void);
void GET_VNI_OutFB(uint8_t Out_index );
void GET_VNI_FB( uint8_t Out_index);
void GET_CLT_FB( uint8_t Out_index);
/**
  * @}// end 
  */

/**
  * @}// end 
  */


/**
  * @}// end 
  */
