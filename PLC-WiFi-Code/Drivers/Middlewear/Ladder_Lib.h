/**
  ******************************************************************************
  * @file    Ladder_Lib.h
  * @author  System Lab Industrial & Motion Control Team
  * @version V1.0.0
  * @date    02-Dicember-2015
  * @brief   
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
  
#include "stdint.h"


/** @defgroup Ladder_Logic_Define    Ladder Logic Define
  * @{
  * @brief Exported variable
  */

#define MAX_OUTPUT_NUMBER               30   //start from 1 to 8 digital output port, from 9 to 16 internal output (timer, counter)
#define EXPRESSION_MAX_SIZE             64
#define _SETBit_(_var_,_pos_)      _var_= (_var_)|(0x01<<_pos_)
#define _RESETBit_(_var_,_pos_)      _var_= (_var_) &(~(0x01<<_pos_))
#define MAX_SERVERDATA_RX               512
#define MAX_NETNUMBER                    2
#define MASK_MSB_OUT                  0x80
#define MASK_MSB_RES                  0x30
#define MAX_PARAM                    4
#define cnt_threshold                    6
#define MAX_COMPONENT_NUMBER             50

/**
  * @}
  */


/** @defgroup Ladder_Logic_Struct_Typedef    Ladder Logic Struct Typedef
  * @{
  * @brief Exported variable
  */

typedef struct
{
  uint8_t Expression[EXPRESSION_MAX_SIZE];
  uint8_t Programmed;   //  Exspression flag
  int8_t output_value;  // -1 Expression not evaluated; 0 output false; 1 output true; 
  
 } OutputStructure_Typedef;

typedef struct
{
  uint16_t  TIM_number;
  uint16_t TIM_period;
  uint16_t TIM_cnt;
  uint8_t  TIM_output;
  }
TimerStruct_Typedef;

typedef struct
{
  uint16_t CNT_number;
  uint8_t CNT_dir;
  uint16_t CNT_val;
  uint8_t CNT_output;
}
CounterStruct_Typedef;



void Save (void);
void Restore (void);
void Init_Output(void);
int16_t WiFi_Decode (uint8_t* frame);
uint8_t PLC_GetOutput (uint16_t* ToOutput) ;
uint8_t Get_Input(uint8_t param, uint8_t value);
void Get_AND (uint8_t* pointer, uint8_t inputs);
void Get_OR (uint8_t* pointer, uint8_t inputs);
void Get_NOT(uint8_t* pointer,uint8_t position);
uint8_t To_Evaluated (void);
uint8_t Get_Next(uint8_t current);
uint8_t* Send_Programming_Log();
uint8_t Get_ResetOUT(void);
void ClearFlag_ResetOUT (void);
uint8_t Component_parser(void);
void Reset_Count (void);
void ToAnalizeCounterIN(void);
 int8_t Get_FlagStatus(void);
void Reset_FlagStatus(void); 
/**
  * @}
  */



