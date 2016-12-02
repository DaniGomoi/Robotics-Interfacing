/**
  ******************************************************************************
  * @file               : ISO8200BQ.h
  * @Author	        : IPD System LAB - Automation and Motion Control Team
  * @Version		: V1.0.0
  * @Date               : 05/11/2015 
  * @Brief	        : Include file related to device driving
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

/** @addtogroup ISO8200BQ_Exported_Functions  ISO8200BQ Exported Functions
  *  @{
  */
#include "IPS_ISO.h"



void ISO_DriveLoad(void* GPIOx, uint16_t GPIO_Pin,uint8_t PinState);
void ISO_DrivePin(void* GPIOx, uint16_t GPIO_Pin,uint8_t PinState);
void ISO_DriveSinch(void* GPIOx, uint16_t GPIO_Pin,uint8_t PinState);
extern void ISO8200BQ_IO_Write(void* GPIOx, uint16_t GPIO_Pin, uint8_t PinState);

/**
  * @}//close group ISO8200BQ Exported Functions
  */





/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
