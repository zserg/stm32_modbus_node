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

/* ----------------------- Modbus includes ----------------------------------*/
#include "mb.h"
#include "mbport.h"

/* ----------------------- Defines ------------------------------------------*/
#define USART0_ENABLED          ( 1 )
#define USART0_IDX              ( 0 )

#define USART1_ENABLED          ( 1 )
#define USART1_IDX              ( USART0_IDX + USART0_ENABLED * 1 )

#define USART_IDX_LAST          ( USART1_IDX )

#define USART_INVALID_PORT      ( 0xFF )
#define USART_NOT_RE_IDX        ( 3 )
#define USART_DE_IDX            ( 4 )

/* ----------------------- Static variables ---------------------------------*/

#if USART1_ENABLED == 1
const Pin       xUSART0Pins[] = {
    PIN_USART0_TXD,
    PIN_USART0_RXD
};
#endif

#if USART1_ENABLED == 1
const Pin       xUSART1NotREPin = { 1 << 25, PIOA, ID_PIOA, PIO_OUTPUT_0, PIO_DEFAULT };
const Pin       xUSART1DEPin = { 1 << 24, PIOA, ID_PIOA, PIO_OUTPUT_0, PIO_DEFAULT };

const Pin       xUSART1Pins[] = {
    PIN_USART1_TXD,
    PIN_USART1_RXD,
    {1 << 23, PIOA, ID_PIOA, PIO_OUTPUT_1, PIO_DEFAULT}
};
#endif

const struct xUSARTHWMappings_t
{
    Usart          *pUsart;
    unsigned int    xUSARTID;
    IRQn_Type       xUSARTIrq;
    const Pin      *USARTNotREPin;
    const Pin      *USARTDEPin;
    const Pin      *xUSARTPins;
    uint32_t        xUSARTPinsCnt;


} xUSARTHWMappings[] =
{
#if USART0_ENABLED == 1
    {
    USART0, ID_USART0, USART0_IRQn, NULL, NULL, &xUSART0Pins[0], PIO_LISTSIZE( xUSART0Pins )},
#endif
#if USART1_ENABLED == 1
    {
    USART1, ID_USART1, USART1_IRQn, &xUSART1NotREPin, &xUSART1DEPin, &xUSART1Pins[0], PIO_LISTSIZE( xUSART1Pins )},
#endif
};

static UCHAR    ucUsedPort = USART_INVALID_PORT;



//===========================
#define USARTDEPin_GPIOx    GPIOA
#define USARTDEPin_GPIO_PIN GPIO_PIN_4

extern UART_HandleTypeDe huart2;

void
vMBPortSerialEnable(UART_HandleTypeDef *huart, BOOL xRxEnable, BOOL xTxEnable )
{

    if( xRxEnable )
    {
        //USART_SetReceiverEnabled( xUSARTHWMappings[ucUsedPort].pUsart, 1 );
        //USART_EnableIt( xUSARTHWMappings[ucUsedPort].pUsart, US_IDR_RXRDY );
        SET_BIT(huart->Instance->CR1, USART_CR1_RE);
        __HAL_UART_ENABLE_IT(huart, UART_IT_RXNE);

    }
    else
    {
        //USART_DisableIt( xUSARTHWMappings[ucUsedPort].pUsart, US_IDR_RXRDY );
        //USART_SetReceiverEnabled( xUSARTHWMappings[ucUsedPort].pUsart, 0 );
        __HAL_UART_DISABLE_IT(huart, UART_IT_RXNE);
        CLEAR_BIT(huart->Instance->CR1, USART_CR1_RE);
    }

    if( xTxEnable )
    {
        //if( NULL != xUSARTHWMappings[ucUsedPort].USARTNotREPin )
        //{
        //    PIO_Set( xUSARTHWMappings[ucUsedPort].USARTNotREPin );
       // }
        //if( NULL != xUSARTHWMappings[ucUsedPort].USARTDEPin )
        //{
        //    PIO_Set( xUSARTHWMappings[ucUsedPort].USARTDEPin );
        //}
        HAL_GPIO_WritePin(USARTDEPin_GPIOx, USARTDEPin_GPIO_PIN, GPIO_PIN_SET);
        //USART_SetTransmitterEnabled( xUSARTHWMappings[ucUsedPort].pUsart, 1 );
        //USART_EnableIt( xUSARTHWMappings[ucUsedPort].pUsart, US_IER_TXRDY );
        //USART_DisableIt( xUSARTHWMappings[ucUsedPort].pUsart, US_IER_TXEMPTY );
        SET_BIT(huart->Instance->CR1, USART_CR1_TE);
        __HAL_UART_ENABLE_IT(huart, UART_IT_TXE);
        __HAL_UART_DISABLE_IT(huart, UART_IT_TC);
    }
    else
    {
        //USART_DisableIt( xUSARTHWMappings[ucUsedPort].pUsart, US_IDR_TXRDY );
        //USART_EnableIt( xUSARTHWMappings[ucUsedPort].pUsart, US_IER_TXEMPTY );
        __HAL_UART_DISABLE_IT(huart, UART_IT_TXE);
        __HAL_UART_ENABLE_IT(huart, UART_IT_TC);
    }
}

BOOL
xMBPortSerialInit( UCHAR ucPORT, ULONG ulBaudRate, UCHAR ucDataBits, eMBParity eParity )
{
    BOOL            bStatus = FALSE;

    huart2.Instance = USART2;
    huart2.Init.BaudRate = 9600;
    huart2.Init.StopBits = UART_STOPBITS_1;
    huart2.Init.Mode = UART_MODE_TX_RX;
    huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
    huart2.Init.OverSampling = UART_OVERSAMPLING_16;
    
        bStatus = TRUE;
        switch ( eParity )
        {
        case MB_PAR_NONE:
            huart.Init.Parity = UART_PARITY_NONE;
            break;
        case MB_PAR_ODD:
            huart.Init.Parity = UART_PARITY_ODD;
            break;
        case MB_PAR_EVEN:
            huart.Init.Parity = UART_PARITY_EVEN;
            break;
        default:
            bStatus = FALSE;
            break;
        }

        switch ( ucDataBits )
        {
        case 8:
            huart.Init.WordLength = UART_WORDLENGTH_8B;
            break;
        default:
            bStatus = FALSE;
        }

        if( TRUE == bStatus )
        {
            HAL_UART_Init(&huart2);
        }
    }

    return bStatus;
}

void
vMBPortSerialClose( void )
{
    HAL_GPIO_WritePin(USARTDEPin_GPIOx, USARTDEPin_GPIO_PIN, GPIO_PIN_CLEAR);
}

BOOL
xMBPortSerialPutByte( CHAR ucByte )
{
    //USART1->US_THR = ucByte;
    huart2->pTxBuffPtr = pData;
    return TRUE;
}

BOOL
xMBPortSerialGetByte( CHAR * pucByte )
{
    *pucByte = USART1->US_RHR;
    return TRUE;
}

void
vUSARTHandler( void )
{
    uint32_t        uiCSR;
    uint32_t        uiIMR;
    uiCSR = xUSARTHWMappings[ucUsedPort].pUsart->US_CSR;
    uiIMR = xUSARTHWMappings[ucUsedPort].pUsart->US_IMR;
    uint32_t        uiCSRMasked = uiCSR & uiIMR;
    if( uiCSRMasked & US_CSR_RXRDY )
    {
        pxMBFrameCBByteReceived(  );
    }
    if( uiCSRMasked & US_CSR_TXRDY )
    {
        pxMBFrameCBTransmitterEmpty(  );
    }
    if( uiCSRMasked & US_CSR_TXEMPTY )
    {
        if( NULL != xUSARTHWMappings[ucUsedPort].USARTDEPin )
        {
            PIO_Clear( xUSARTHWMappings[ucUsedPort].USARTDEPin );
        }
        if( NULL != xUSARTHWMappings[ucUsedPort].USARTNotREPin )
        {
            PIO_Clear( xUSARTHWMappings[ucUsedPort].USARTNotREPin );
        }
        USART_DisableIt( xUSARTHWMappings[ucUsedPort].pUsart, US_IER_TXEMPTY );
    }
}

#if USART1_ENABLED == 1
void
USART1_IrqHandler( void )
{
    vUSARTHandler(  );
}
#endif

#if USART0_ENABLED == 1
void
USART0_IrqHandler( void )
{
    vUSARTHandler(  );
}
#endif
