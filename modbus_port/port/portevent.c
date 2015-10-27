/*
 * FreeModbus Libary: Atmel AT91SAM3S Demo Application
 * Copyright (C) 2010 Christian Walter <cwalter@embedded-solutions.at>
 *
 * 
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 * 1. Redistributions of source code must retain the above copyright
 *   notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *   notice, this list of conditions and the following disclaimer in the
 *   documentation and/or other materials provided with the distribution.
 * 3. The name of the author may not be used to endorse or promote products
 *   derived from this software without specific prior written permission.
 * 
 * THIS SOFTWARE IS PROVIDED BY THE AUTHOR ``AS IS'' AND ANY EXPRESS OR
 * IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES
 * IF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED.
 * IN NO EVENT SHALL THE AUTHOR BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT
 * NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
 * DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
 * THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF
 * THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 * File: $Id: portevent.c,v 1.1 2010/06/05 09:57:48 wolti Exp $
 */

/* ----------------------- Modbus includes ----------------------------------*/
#include "mb.h"
#include "mbport.h"
#include "cmsis_os.h"
#include "stm32f1xx_hal.h"

/* ----------------------- Variables ----------------------------------------*/
static eMBEventType eQueuedEvent;
static BOOL     xEventInQueue;
xQueueHandle mbEventQueue;
extern UART_HandleTypeDef huart1;

/* ----------------------- Start implementation -----------------------------*/
BOOL
xMBPortEventInit( void )
{
  mbEventQueue = xQueueCreate(1, sizeof(eMBEventType));
  return TRUE;
}

BOOL
xMBPortEventPost( eMBEventType eEvent )
{
  //HAL_UART_Transmit(&huart1, "P\n\r" , 3, 0xFFFF);
   portBASE_TYPE taskWoken;
   xQueueSendFromISR(mbEventQueue, &eEvent, &taskWoken);
   return TRUE;
}

BOOL
xMBPortFirstEventPost( eMBEventType eEvent )
{
  //HAL_UART_Transmit(&huart1, "P\n\r" , 3, 0xFFFF);
   portBASE_TYPE taskWoken;
   xQueueSend(mbEventQueue, &eEvent, portMAX_DELAY);
   return TRUE;
}

BOOL
xMBPortEventGet( eMBEventType * eEvent )
{
  //HAL_UART_Transmit(&huart1, "G" , 1, 0xFFFF);

 xQueueReceive(mbEventQueue, eEvent, portMAX_DELAY);
  //HAL_UART_Transmit(&huart1, "g" , 1, 0xFFFF);
 return TRUE;


}
