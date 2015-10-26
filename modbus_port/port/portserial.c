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
 * File: $Id: portserial.c,v 1.2 2010/06/06 13:46:42 wolti Exp $
 */

/* ----------------------- Platform includes --------------------------------*/
#include <stdlib.h>
#include "port.h"
#include "stm32f1xx_hal.h"

/* ----------------------- Modbus includes ----------------------------------*/
#include "mb.h"
#include "mbport.h"

/* ----------------------- Defines ------------------------------------------*/

/* ----------------------- Static variables ---------------------------------*/


//===========================
#define USARTDEPin_GPIOx    GPIOA
#define USARTDEPin_GPIO_PIN GPIO_PIN_4

extern UART_HandleTypeDef huart2;
extern UART_HandleTypeDef huart1;

void
vMBPortSerialEnable(BOOL xRxEnable, BOOL xTxEnable )
{

    if( xRxEnable )
    {
        SET_BIT(huart2.Instance->CR1, USART_CR1_RE);
        __HAL_UART_ENABLE_IT(&huart2, UART_IT_RXNE);

    }
    else
    {
        __HAL_UART_DISABLE_IT(&huart2, UART_IT_RXNE);
        CLEAR_BIT(huart2.Instance->CR1, USART_CR1_RE);
    }

    if( xTxEnable )
    {
        HAL_GPIO_WritePin(USARTDEPin_GPIOx, USARTDEPin_GPIO_PIN, GPIO_PIN_SET);
        SET_BIT(huart2.Instance->CR1, USART_CR1_TE);
        __HAL_UART_ENABLE_IT(&huart2, UART_IT_TXE);
        __HAL_UART_DISABLE_IT(&huart2, UART_IT_TC);
    }
    else
    {
        __HAL_UART_DISABLE_IT(&huart2, UART_IT_TXE);
        __HAL_UART_ENABLE_IT(&huart2, UART_IT_TC);
    }
}

BOOL
xMBPortSerialInit( UCHAR ucPORT, ULONG ulBaudRate, UCHAR ucDataBits, eMBParity eParity )
{
    BOOL            bStatus = FALSE;

    huart2.Instance = USART2;
    huart2.Init.BaudRate = ulBaudRate;
    huart2.Init.StopBits = UART_STOPBITS_1;
    huart2.Init.Mode = UART_MODE_TX_RX;
    huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
    huart2.Init.OverSampling = UART_OVERSAMPLING_16;
    
        bStatus = TRUE;
        switch ( eParity )
        {
        case MB_PAR_NONE:
            huart2.Init.Parity = UART_PARITY_NONE;
            break;
        case MB_PAR_ODD:
            huart2.Init.Parity = UART_PARITY_ODD;
            break;
        case MB_PAR_EVEN:
            huart2.Init.Parity = UART_PARITY_EVEN;
            break;
        default:
            bStatus = FALSE;
            break;
        }

        switch ( ucDataBits )
        {
        case 8:
            huart2.Init.WordLength = UART_WORDLENGTH_8B;
            break;
        default:
            bStatus = FALSE;
        }

        if( TRUE == bStatus )
        {
            HAL_UART_Init(&huart2);
        }

    return bStatus;
}

void
vMBPortSerialClose( void )
{
    HAL_GPIO_WritePin(USARTDEPin_GPIOx, USARTDEPin_GPIO_PIN, GPIO_PIN_RESET);
}

BOOL
xMBPortSerialPutByte( CHAR ucByte )
{
    huart2.Instance->DR = ucByte;
    return TRUE;
}

BOOL
xMBPortSerialGetByte( CHAR * pucByte )
{
    *pucByte = (CHAR) huart2.Instance->DR;
    return TRUE;
}

void vUARTHandler( UART_HandleTypeDef *husart )
{
  uint32_t tmp_flag, tmp_it_source;
  uint8_t data;
  
  tmp_flag = __HAL_UART_GET_FLAG(husart, UART_FLAG_RXNE);
  tmp_it_source = __HAL_UART_GET_IT_SOURCE(husart, UART_IT_RXNE);
  if((tmp_flag != RESET) && (tmp_it_source != RESET))
  { 
  //HAL_UART_Transmit(&huart1, "s0" , 2, 0xFFFF);
	  pxMBFrameCBByteReceived(  );
  }

  
  tmp_flag = __HAL_UART_GET_FLAG(husart, UART_FLAG_TXE);
  tmp_it_source = __HAL_UART_GET_IT_SOURCE(husart, UART_IT_TXE);
  if((tmp_flag != RESET) && (tmp_it_source != RESET))
  { 
  //HAL_UART_Transmit(&huart1, "s1" , 2, 0xFFFF);
        pxMBFrameCBTransmitterEmpty(  );
  }
  
  tmp_flag = __HAL_UART_GET_FLAG(husart, UART_FLAG_TC);
  tmp_it_source = __HAL_UART_GET_IT_SOURCE(husart, UART_IT_TC);
  if((tmp_flag != RESET) && (tmp_it_source != RESET))
  { 
  HAL_UART_Transmit(&huart1, "s2" , 2, 0xFFFF);
      HAL_GPIO_WritePin(USARTDEPin_GPIOx, USARTDEPin_GPIO_PIN, GPIO_PIN_RESET);
      __HAL_UART_DISABLE_IT(husart, UART_IT_TC);
  }

}
