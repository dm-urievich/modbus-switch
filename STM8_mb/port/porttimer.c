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
 * File: $Id: porttimer.c,v 1.1 2006/08/22 21:35:13 wolti Exp $
 */

/* ----------------------- Platform includes --------------------------------*/
#include "port.h"

/* ----------------------- Modbus includes ----------------------------------*/
#include "mb.h"
#include "mbport.h"
#include "stm8s_tim4.h"

/* ----------------------- static functions ---------------------------------*/
//static void prvvTIMERExpiredISR( void );

/* ----------------------- Start implementation -----------------------------*/
BOOL
xMBPortTimersInit( USHORT usTim1Timerout50us )
{
    TIM4->ARR = usTim1Timerout50us * 6;	    // one tick 50 us
    TIM4->PSCR = 7;				// делим на 128 (2^7), 125.000 один тик 8 мкс

    return TRUE;
}


void
vMBPortTimersEnable(  )
{
    /* Enable the timer with the timeout passed to xMBPortTimersInit( ) */
	TIM4->CNTR = 0;
    TIM4->CR1 |= TIM4_CR1_CEN;
    TIM4->IER |= TIM4_IER_UIE;
}

void
vMBPortTimersDisable(  )
{
    /* Disable any pending timers. */
    TIM4->CR1 &= ~TIM4_CR1_CEN;
    TIM4->IER &= ~TIM4_IER_UIE;
    TIM4->CNTR = 0;
}

/* Create an ISR which is called whenever the timer has expired. This function
 * must then call pxMBPortCBTimerExpired( ) to notify the protocol stack that
 * the timer has expired.
 */

#pragma vector = 25
__interrupt void Timer_ISR(void)
{
    static u8 c;

    TIM4->SR1 &= ~TIM4_SR1_UIF;
    //TIM4_ClearFlag(TIM4_FLAG_UPDATE);
    pxMBPortCBTimerExpired(  );

    c++;
}

