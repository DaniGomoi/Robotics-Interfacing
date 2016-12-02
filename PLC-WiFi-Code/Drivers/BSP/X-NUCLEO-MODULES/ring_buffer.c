/**
 ******************************************************************************
 * @file    ring_buffer.c
 * @author  Central LAB
 * @version V1.0.0
 * @date    17-May-2015
 * @brief   Implements the Circular Buffer management of the Wi-Fi module
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

#include <stdio.h>
#include <stdlib.h>
#include "ring_buffer.h"

/** @addtogroup MIDDLEWARE   MIDDLEWARE
* @{
*/ 


/** @defgroup  NUCLEO_WIFI_UTILS   NUCLEO WIFI UTILS
  * @brief Wi-Fi buffer utility
  * @{
  */ 


/** @defgroup NUCLEO_WIFI_UTILS_Private_Defines    NUCLEO WIFI UTILS Private Defines
  * @{
  */

/**
  * @}
  */

/** @defgroup NUCLEO_WIFI_UTILS_Private_Variables   NUCLEO WIFI UTILS Private Variables
  * @{
  */
extern uint8_t ring_buffer[1024];
extern uint8_t pop_buffer[1];
#define ELEMENT_SIZE 1

               
/**
  * @}
  */

/** @defgroup NUCLEO_WIFI_UTILS_Private_Functions   NUCLEO WIFI UTILS Private Functions
  * @{
  */

/**
  * @brief  init
  *         Initialize a circular buffer of type buffer_t
  * @param  None
  * @retval None
  */
void init(buffer_t *buffer, int size) {
    buffer->size = size;
    buffer->start = 0;
    buffer->count = 0;
    buffer->end = 0;
    buffer->element = ring_buffer;
}


/**
  * @brief  flush_buffer_queue
  *         flushes the buffer
  * @param  None
  * @retval None
  */ 
void flush_buffer_queue(buffer_t *buffer) 
{
  buffer->start = buffer->end;//the tail goes up to the head and buffer becomes empty
  buffer->count = 0;
}

/**
  * @brief  is_half_full
  *         checks if the buffer is half full (empty)
  * @param  None
  * @retval None
  */ 
int is_half_full(buffer_t *buffer)
{
  int bufsize = buffer->size;
  if (buffer->count >= bufsize - 100) 
  { 
    //printf("half full!");
    return 1;
  } 
  else 
  {
    return 0;
  }
}

/**
  * @brief  is_half_empty
  *         checks if the buffer is less than half
  * @param  None
  * @retval None
  */ 
int is_half_empty(buffer_t *buffer)
{
  int bufsize = buffer->size;
  if (buffer->count <= 100) 
  { 
    return 1;
  } 
  else 
  {
    return 0;
  }
}

/**
  * @brief  full
  *         indicates if the given buffer is full or not
  * @param  None
  * @retval None
  */
int full(buffer_t *buffer) 
{
  int bufsize = buffer->size;
  if (buffer->count == bufsize) 
  { 
    return 1;
  } 
  else 
  {
    return 0;
  }
}

/**
  * @brief  empty
  *         indicates if the given buffer is empty or not
  * @param  None
  * @retval None
  */
int empty(buffer_t *buffer) {
    if (buffer->count == 0) {
        return 1;
    } else {
        return 0;
    }
}

/**
  * @brief  push_buffer
  *         pushes a new item onto the circular buffer (queues it)
  * @param  None
  * @retval None
  */
void push_buffer(buffer_t *buffer, uint8_t *data) 
{
  int bufsize;
  
  if (full(buffer)) 
  {     
    //Buffer overflow and no more space
    //MPD: No Action taken here; in case of buffer overflow, do we need to overwrite last buffer?
    //printf("\r\nRing Buffer Full!!\r\n");
    //printf(data);
    return;
  } else 
  {
    buffer->count++;    
    memcpy(&buffer->element[buffer->end], data, ELEMENT_SIZE);
    buffer->end = buffer->end + ELEMENT_SIZE;
    
    //wrap around if max size is reached
    bufsize = (buffer->size);
    if (buffer->end >= bufsize) 
    {
      buffer->end = 0;
    }
  }
}
 
/**
  * @brief  pop_buffer_queue
  *         dequeues the circular buffer
  * @param  None
  * @retval None
  */ 
uint8_t * pop_buffer_queue(buffer_t *buffer) 
{
  uint8_t * element;
  int bufsize;
  
  element = &pop_buffer[0];
  if (empty(buffer)) 
  {
    //printf("\r\nRing Buffer Empty!!\r\n");
    return NULL;
  } 
  else 
  {       
    /* First in First Out*/
    memcpy(element, &buffer->element[buffer->start], ELEMENT_SIZE);
    buffer->start = buffer->start + ELEMENT_SIZE;
    buffer->count--;
    
    bufsize = (buffer->size);
    if (buffer->start >= bufsize) 
    {
      buffer->start = 0;
    }    
    return element;
  }
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

/******************* (C) COPYRIGHT 2011 STMicroelectronics *****END OF FILE****/
