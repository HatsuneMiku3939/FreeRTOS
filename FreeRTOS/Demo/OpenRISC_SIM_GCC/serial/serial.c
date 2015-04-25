/*
    FreeRTOS V7.3.0 - Copyright (C) 2012 Real Time Engineers Ltd.

    FEATURES AND PORTS ARE ADDED TO FREERTOS ALL THE TIME.  PLEASE VISIT
    http://www.FreeRTOS.org TO ENSURE YOU ARE USING THE LATEST VERSION.

    ***************************************************************************
     *                                                                       *
     *    FreeRTOS tutorial books are available in pdf and paperback.        *
     *    Complete, revised, and edited pdf reference manuals are also       *
     *    available.                                                         *
     *                                                                       *
     *    Purchasing FreeRTOS documentation will not only help you, by       *
     *    ensuring you get running as quickly as possible and with an        *
     *    in-depth knowledge of how to use FreeRTOS, it will also help       *
     *    the FreeRTOS project to continue with its mission of providing     *
     *    professional grade, cross platform, de facto standard solutions    *
     *    for microcontrollers - completely free of charge!                  *
     *                                                                       *
     *    >>> See http://www.FreeRTOS.org/Documentation for details. <<<     *
     *                                                                       *
     *    Thank you for using FreeRTOS, and thank you for your support!      *
     *                                                                       *
    ***************************************************************************


    This file is part of the FreeRTOS distribution.

    FreeRTOS is free software; you can redistribute it and/or modify it under
    the terms of the GNU General Public License (version 2) as published by the
    Free Software Foundation AND MODIFIED BY the FreeRTOS exception.
    >>>NOTE<<< The modification to the GPL is included to allow you to
    distribute a combined work that includes FreeRTOS without being obliged to
    provide the source code for proprietary components outside of the FreeRTOS
    kernel.  FreeRTOS is distributed in the hope that it will be useful, but
    WITHOUT ANY WARRANTY; without even the implied warranty of MERCHANTABILITY
    or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General Public License for
    more details. You should have received a copy of the GNU General Public
    License and the FreeRTOS license exception along with FreeRTOS; if not it
    can be viewed here: http://www.freertos.org/a00114.html and also obtained
    by writing to Richard Barry, contact details for whom are available on the
    FreeRTOS WEB site.

    1 tab == 4 spaces!

    ***************************************************************************
     *                                                                       *
     *    Having a problem?  Start by reading the FAQ "My application does   *
     *    not run, what could be wrong?"                                     *
     *                                                                       *
     *    http://www.FreeRTOS.org/FAQHelp.html                               *
     *                                                                       *
    ***************************************************************************


    http://www.FreeRTOS.org - Documentation, training, latest versions, license
    and contact details.

    http://www.FreeRTOS.org/plus - A selection of FreeRTOS ecosystem products,
    including FreeRTOS+Trace - an indispensable productivity tool.

    Real Time Engineers ltd license FreeRTOS to High Integrity Systems, who sell
    the code with commercial support, indemnification, and middleware, under
    the OpenRTOS brand: http://www.OpenRTOS.com.  High Integrity Systems also
    provide a safety engineered and independently SIL3 certified version under
    the SafeRTOS brand: http://www.SafeRTOS.com.
*/

/*
  BASIC INTERRUPT DRIVEN SERIAL PORT DRIVER FOR USART.
*/

/* Scheduler includes. */
#include "FreeRTOS.h"
#include "queue.h"
#include "task.h"

/* Demo application includes. */
#include "serial.h"

/* bsp includes. */
#include "support.h"
#include "spr_defs.h"
#include "uart.h"
#include "interrupts.h"

/*-----------------------------------------------------------*/

/* Constants to setup and access the USART. */
#define serINVALID_COMPORT_HANDLER        ( ( xComPortHandle ) 0 )
#define serINVALID_QUEUE                  ( ( xQueueHandle ) 0 )
#define serHANDLE                         ( ( xComPortHandle ) 1 )
#define serNO_BLOCK                       ( ( portTickType ) 0 )

/*-----------------------------------------------------------*/

/* Queues used to hold received characters, and characters waiting to be
transmitted. */
static xQueueHandle xRxedChars;
static xQueueHandle xCharsForTx;

/*-----------------------------------------------------------*/

/* Forward declaration. */
static void vprvSerialCreateQueues( unsigned portBASE_TYPE uxQueueLength,
                                    xQueueHandle *pxRxedChars,
                                    xQueueHandle *pxCharsForTx );

/*-----------------------------------------------------------*/
static void vUSART_ISR( void *arg )
{
    /* Now we can declare the local variables. */
    arg = arg;
    signed portCHAR cChar;
    portBASE_TYPE xHigherPriorityTaskWoken = pdFALSE;
    unsigned portLONG ulStatus;
    portBASE_TYPE retstatus;

    /* What caused the interrupt? */
    ulStatus = uart_get_iir(0);

    // TX RADY INTERRUPT
    if (ulStatus & UART_IIR_THRI)
    {
        /* The interrupt was caused by the THR becoming empty.  Are there any
        more characters to transmit?
        Because FreeRTOS is not supposed to run with nested interrupts, put all OS
        calls in a critical section . */

        /* entering, exiting ciritical section around xQueueReceiveFromISR is not
        required. OpenRISC automaticaly disable interrupt when expection occurs */
        retstatus = xQueueReceiveFromISR( xCharsForTx, &cChar, &xHigherPriorityTaskWoken );

        if (retstatus == pdTRUE)
        {
            /* A character was retrieved from the queue so can be sent to the
             THR now. */
            uart_putc_noblock(0, cChar);
        }
        else
        {
            /* Queue empty, nothing to send so turn off the Tx interrupt. */
            uart_txint_disable(0);
        }
    }

    // RX RADY INTERRUPT
    if (ulStatus & UART_IIR_RDI)
    {
        /* The interrupt was caused by the receiver getting data. */
        cChar = uart_getc_noblock(0);

        /* Because FreeRTOS is not supposed to run with nested interrupts, put all OS
        calls in a critical section . but in case of OpenRISC, it is not required. Tick
        , External interrupt are automaticaly disabled. */
        xQueueSendFromISR(xRxedChars, &cChar, &xHigherPriorityTaskWoken);
    }

    /* The return value will be used by portEXIT_SWITCHING_ISR() to know if it
    should perform a vTaskSwitchContext(). */
    // return ( xHigherPriorityTaskWoken );
}
/*-----------------------------------------------------------*/


/*
 * Init the serial port for the Minimal implementation.
 */
xComPortHandle xSerialPortInitMinimal( unsigned portLONG ulWantedBaud, unsigned portBASE_TYPE uxQueueLength )
{
    xComPortHandle xReturn = serHANDLE;

    /* Create the rx and tx queues. */
    vprvSerialCreateQueues( uxQueueLength, &xRxedChars, &xCharsForTx );

    /* Configure USART. */
    if( ( xRxedChars != serINVALID_QUEUE ) &&
        ( xCharsForTx != serINVALID_QUEUE ) &&
        ( ulWantedBaud != ( unsigned portLONG ) 0 ) )
    {
        portENTER_CRITICAL();
        {
            // register interrupt handler
            int_add(UART0_IRQ, vUSART_ISR, 0x0);
        }
        portEXIT_CRITICAL();
    }
    else
    {
        xReturn = serINVALID_COMPORT_HANDLER;
    }

    return xReturn;
}
/*-----------------------------------------------------------*/

signed portBASE_TYPE xSerialGetChar( xComPortHandle pxPort, signed portCHAR *pcRxedChar, portTickType xBlockTime )
{
    /* The port handle is not required as this driver only supports UART0. */
    ( void ) pxPort;

    /* Get the next character from the buffer.  Return false if no characters
    are available, or arrive before xBlockTime expires. */
    if( xQueueReceive( xRxedChars, pcRxedChar, xBlockTime ) )
    {
        return pdTRUE;
    }
    else
    {
        return pdFALSE;
    }
}
/*-----------------------------------------------------------*/

void vSerialPutString( xComPortHandle pxPort, const signed portCHAR * const pcString, unsigned portSHORT usStringLength )
{
    usStringLength = usStringLength;
    signed portCHAR *pxNext;

    /* NOTE: This implementation does not handle the queue being full as no
    block time is used! */

    /* The port handle is not required as this driver only supports UART0. */
    ( void ) pxPort;

    /* Send each character in the string, one at a time. */
    pxNext = ( signed portCHAR * ) pcString;
    while( *pxNext )
    {
        xSerialPutChar( pxPort, *pxNext, serNO_BLOCK );
        pxNext++;
    }
}
/*-----------------------------------------------------------*/

signed portBASE_TYPE xSerialPutChar( xComPortHandle pxPort, signed portCHAR cOutChar, portTickType xBlockTime )
{
    /* The port handle is not required as this driver only supports UART0. */
    ( void ) pxPort;

    /* Place the character in the queue of characters to be transmitted. */
    if( xQueueSend( xCharsForTx, &cOutChar, xBlockTime ) != pdPASS )
    {
        return pdFAIL;
    }

    /* Turn on the Tx interrupt so the ISR will remove the character from the
    queue and send it.   This does not need to be in a critical section as
    if the interrupt has already removed the character the next interrupt
    will simply turn off the Tx interrupt again. */
    uart_txint_enable(0);

    return pdPASS;
}
/*-----------------------------------------------------------*/

void vSerialClose( xComPortHandle xPort )
{
  /* Not supported as not required by the demo application. */
  xPort = xPort;                // prevent compiler warning
}
/*-----------------------------------------------------------*/

/*###########################################################*/

/*
 * Create the rx and tx queues.
 */
static void vprvSerialCreateQueues(  unsigned portBASE_TYPE uxQueueLength, xQueueHandle *pxRxedChars, xQueueHandle *pxCharsForTx )
{
    /* Create the queues used to hold Rx and Tx characters. */
    xRxedChars = xQueueCreate( uxQueueLength, ( unsigned portBASE_TYPE ) sizeof( signed portCHAR ) );
    xCharsForTx = xQueueCreate( uxQueueLength + 1, ( unsigned portBASE_TYPE ) sizeof( signed portCHAR ) );

    /* Pass back a reference to the queues so the serial API file can
    post/receive characters. */
    *pxRxedChars = xRxedChars;
    *pxCharsForTx = xCharsForTx;
}
/*-----------------------------------------------------------*/
