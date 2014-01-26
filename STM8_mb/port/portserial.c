/*
 * FreeModbus Libary: BARE Port
 * Copyright (C) 2006 Christian Walter <wolti@sil.at>
 *
 * This library is free software; you can redistribute it and/or
 * modify it under the terms of the GNU Lesser General Public
 * License as published by the Free Software Foundation; either
 * version 2.1 of the License, or (at your option) any later version.
 *
 * This library is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 * Lesser General Public License for more details.
 *
 * You should have received a copy of the GNU Lesser General Public
 * License along with this library; if not, write to the Free Software
 * Foundation, Inc., 51 Franklin St, Fifth Floor, Boston, MA  02110-1301  USA
 *
 * File: $Id: portserial.c,v 1.1 2006/08/22 21:35:13 wolti Exp $
 */

#include "port.h"
#include "../../main.h"
#include "stm8s.h"
#include "stm8s_type.h"

#include "stm8s_uart2.h"

extern uint16_t ledMode;

/* ----------------------- Modbus includes ----------------------------------*/
#include "mb.h"
#include "mbport.h"

/* ----------------------- static functions ---------------------------------*/
//static void prvvUARTTxReadyISR( void );
//static void prvvUARTRxISR( void );

/* ----------------------- Start implementation -----------------------------*/
void
vMBPortSerialEnable( BOOL xRxEnable, BOOL xTxEnable )
{
    /* If xRXEnable enable serial receive interrupts. If xTxENable enable
     * transmitter empty interrupts.
     */
	if(xRxEnable == TRUE)
    {
        UART2_ITConfig(UART2_IT_RXNE, ENABLE);
      //USART1->CR1 |= USART_CR1_RXNEIE;	//включим прерывание если входной буфер не пуст
//	  SetDirRx();			// переключаем только после последнего байта
	  // по хорошему до конца последнего байта и по приему запрещать
      if (ledMode == ledModbus_en) 
    	  LEDoff();
    }
    else
    {
        UART2_ITConfig(UART2_IT_RXNE, DISABLE);
      //USART1->CR1 &= ~USART_CR1_RXNEIE;	//выключим прерывание если входной буфер не пуст
    }

    if(xTxEnable == TRUE)
    {
        UART2_ITConfig(UART2_IT_TXE, ENABLE);
        UART2_ITConfig(UART2_IT_TC, ENABLE);
      //USART1->CR1 |= USART_CR1_TXEIE;	//включим прерывание если выходной буфер пуст
	  //USART1->CR1 |= USART_CR1_TCIE;		// ждем отправки последнего байта
	  setTx();
      if (ledMode == ledModbus_en) 
    	  LEDon();
	}
    else
    {
        UART2_ITConfig(UART2_IT_TXE, DISABLE);
      //USART1->CR1 &= ~USART_CR1_TXEIE;	//выключим прерывание если выходной буфер пуст
    }
}

BOOL
xMBPortSerialInit( UCHAR ucPORT, ULONG ulBaudRate, UCHAR ucDataBits, eMBParity eParity )
{
	// надо будет переделать, под билиотеку
	   // Инициализация  UART
  /* //UART2->BRR1 = 0x08;
   //UART2->BRR2 = 0x0B; // Baudrate 115200
    UART2->BRR1 = 0x09;
    UART2->BRR2 = 0x0A;     // какая-то хрень с генератором, работаем на частоте больше 16 МГц



    UART2->CR2 |= UART2_CR2_TEN;
    UART2->CR2 |= UART2_CR2_REN;
    //UART2->CR2 |= UART2_CR2_TCIEN;
   UART2->CR2 |= UART2_CR2_RIEN;
*/


    UART2_Init(ulBaudRate,
               UART2_WORDLENGTH_8D,
               UART2_STOPBITS_1,
               UART2_PARITY_NO,
               UART2_SYNCMODE_CLOCK_DISABLE,
               UART2_MODE_TXRX_ENABLE);

   // порты для уарта
   // Tx
   GPIO_Init(GPIOD, GPIO_PIN_5, GPIO_MODE_OUT_PP_LOW_FAST);
               /*
   GPIOD->DDR = 1 << 5;   // Ножка PD0 конфигурируется на вывод
   GPIOD->CR1 = 1 << 5;   // Выход типа Push-pull
   GPIOD->CR2 = 1 << 5;
   */
   // dir
   GPIO_Init(GPIOD, GPIO_PIN_7, GPIO_MODE_OUT_PP_LOW_FAST);
   /*GPIOD->DDR = 1 << 7;   // Ножка PD0 конфигурируется на вывод
   GPIOD->CR1 = 1 << 7;   // Выход типа Push-pull
   GPIOD->CR2 = 1 << 7;*/
   // Rx
   GPIO_Init(GPIOD, GPIO_PIN_6, GPIO_MODE_IN_PU_NO_IT);

    setRx();

    return TRUE;
}

BOOL
xMBPortSerialPutByte( CHAR ucByte )
{
    /* Put a byte in the UARTs transmit buffer. This function is called
     * by the protocol stack if pxMBFrameCBTransmitterEmpty( ) has been
     * called. */
	UART2->DR = ucByte;
    return TRUE;
}

BOOL
xMBPortSerialGetByte( CHAR * pucByte )
{
    /* Return the byte in the UARTs receive buffer. This function is called
     * by the protocol stack after pxMBFrameCBByteReceived( ) has been called.
     */
	*pucByte = UART2->DR;
    return TRUE;
}

/* Create an interrupt handler for the transmit buffer empty interrupt
 * (or an equivalent) for your target processor. This function should then
 * call pxMBFrameCBTransmitterEmpty( ) which tells the protocol stack that
 * a new character can be sent. The protocol stack will then call
 * xMBPortSerialPutByte( ) to send the character.
 */
/*
static void prvvUARTTxReadyISR( void )
{
    pxMBFrameCBTransmitterEmpty(  );
}
*/

/* Create an interrupt handler for the receive interrupt for your target
 * processor. This function should then call pxMBFrameCBByteReceived( ). The
 * protocol stack will then call xMBPortSerialGetByte( ) to retrieve the
 * character.
 */
/*
static void prvvUARTRxISR( void )
{
    pxMBFrameCBByteReceived(  );
}
*/


// прерывание по окончанию передачи
#pragma vector = 22
__interrupt void UART2_Tx_ISR(void)
{
    static u8 p;
    p++;

    if (UART2_GetFlagStatus(UART2_FLAG_TXE) == SET) {
        pxMBFrameCBTransmitterEmpty(  );
    }

    if (UART2_GetFlagStatus(UART2_FLAG_TC) == SET) {
        UART2_ITConfig(UART2_IT_TC, DISABLE);
        UART2->SR &= ~UART2_SR_TC;              // именно так, в библиотеке такого нет
        setRx();
    }
}

//  прерывание по приему
#pragma vector = 23
__interrupt void UART2_Rx_ISR(void)
{
	UART2->SR &= ~UART2_SR_RXNE;    // не обязательно
    pxMBFrameCBByteReceived(  );
}
