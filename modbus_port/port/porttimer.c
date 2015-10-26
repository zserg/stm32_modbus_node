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
 * File: $Id: porttimer.c,v 1.2 2010/06/06 13:46:42 wolti Exp $
 */

/* ----------------------- System includes ----------------------------------*/

/* ----------------------- Modbus includes ----------------------------------*/
#include "port.h"
#include "mb.h"
#include "mbport.h"
#include "stm32f1xx_hal.h"
#include "stm32f1xx_hal_tim.h"

/* ----------------------- Defines ------------------------------------------*/

#if MB_TIMER_DEBUG == 1
#define TIMER_PIN { 1 << 6, PIOA, ID_PIOA, PIO_OUTPUT_1, PIO_DEFAULT }
#endif

#ifndef SYSTICK_COUNTFLAG
/* missing in CMSIS */
#define SYSTICK_COUNTFLAG                   ( 16 )             
#endif
/* ----------------------- Static variables ---------------------------------*/
#if MB_TIMER_DEBUG == 1
const static Pin xTimerDebugPins[] = { TIMER_PIN };
#endif
extern TIM_HandleTypeDef htim1;
extern uint32_t uwTick;
extern UART_HandleTypeDef huart1;
/* ----------------------- Start implementation -----------------------------*/
BOOL
xMBPortTimersInit( USHORT usTim1Timerout50us )
{
  HAL_UART_Transmit(&huart1, "_TI_" , 4, 0xFFFF);
#if MB_TIMER_DEBUG == 1
    PIO_Configure( xTimerDebugPins, PIO_LISTSIZE( xTimerDebugPins ) );
#endif
    __HAL_TIM_DISABLE_IT(&htim1, TIM_IT_UPDATE);

    //htim1.Init.Period = ( 50 * usTim1Timerout50us );
    htim1.Init.Period = ( 50 * 1000 );
    HAL_TIM_Base_Init(&htim1);

    HAL_NVIC_ClearPendingIRQ(TIM1_UP_IRQn);
    __HAL_TIM_CLEAR_IT(&htim1, TIM_IT_UPDATE);

    return TRUE;
}




void 
vMBPortTimerClose( void )
{
    __HAL_TIM_DISABLE_IT(&htim1, TIM_IT_UPDATE);
    __HAL_TIM_DISABLE(&htim1);
}

void
vMBPortTimersEnable(  )
{
  HAL_UART_Transmit(&huart1, "t" , 1, 0xFFFF);
#if MB_TIMER_DEBUG == 1
    PIO_Set( &xTimerDebugPins[0] );  
#endif  
    // TIM1->CNT = 0;
    __HAL_TIM_ENABLE_IT(&htim1, TIM_IT_UPDATE);
    __HAL_TIM_ENABLE(&htim1);
}

void
vMBPortTimersDisable(  )
{
    __HAL_TIM_DISABLE(&htim1);
#if MB_TIMER_DEBUG == 1
    PIO_Clear( &xTimerDebugPins[0] );
#endif   
}

void
vMBPortTimersDelay( USHORT usTimeOutMS )
{
  HAL_Delay((uint32_t) usTimeOutMS);

}

void
TCX_IRQHANDLER( void )
{
   HAL_UART_Transmit(&huart1,  "T", 1, 0xFFFF);
    __HAL_TIM_DISABLE(&htim1);
    __HAL_TIM_CLEAR_IT(&htim1, TIM_IT_UPDATE);
  
#if MB_TIMER_DEBUG == 1
        PIO_Clear( &xTimerDebugPins[0] );
#endif
        ( void )pxMBPortCBTimerExpired(  );
}
