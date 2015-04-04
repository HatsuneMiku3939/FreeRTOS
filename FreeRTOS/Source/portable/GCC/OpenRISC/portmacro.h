/*
    FreeRTOS V7.1.1 - Copyright (C) 2012 Real Time Engineers Ltd.


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
     *    not run, what could be wrong?                                      *
     *                                                                       *
     *    http://www.FreeRTOS.org/FAQHelp.html                               *
     *                                                                       *
    ***************************************************************************


    http://www.FreeRTOS.org - Documentation, training, latest information,
    license and contact details.

    http://www.FreeRTOS.org/plus - A selection of FreeRTOS ecosystem products,
    including FreeRTOS+Trace - an indispensable productivity tool.

    Real Time Engineers ltd license FreeRTOS to High Integrity Systems, who sell
    the code with commercial support, indemnification, and middleware, under
    the OpenRTOS brand: http://www.OpenRTOS.com.  High Integrity Systems also
    provide a safety engineered and independently SIL3 certified version under
    the SafeRTOS brand: http://www.SafeRTOS.com.
*/

#ifndef PORTMACRO_H
#define PORTMACRO_H

#ifdef __cplusplus
extern "C" {
#endif

//-----------------------------------------------------------
// Port specific definitions
//-----------------------------------------------------------
// Type definitions.
#define portCHAR        char
#define portFLOAT       float
#define portDOUBLE      double
#define portLONG        long
#define portSHORT       short
#define portSTACK_TYPE  unsigned portLONG
#define portBASE_TYPE   long
#define portTickType    unsigned portLONG
#define portMAX_DELAY   (portTickType)0xffffffff

#if( configUSE_16_BIT_TICKS == 1 )
    #effor "configUSE_16_BIT_TICKS must be 0"
#endif

/*-----------------------------------------------------------*/
#define portSTACK_GROWTH                -1
#define portTICK_RATE_MS                ( (portTickType) 1000 / configTICK_RATE_HZ )
#define portBYTE_ALIGNMENT              4
#define portCRITICAL_NESTING_IN_TCB     1
#define portINSTRUCTION_SIZE            ( ( portSTACK_TYPE ) 4 )
#define portNO_CRITICAL_SECTION_NESTING ( ( portSTACK_TYPE ) 0 )
#define portPOINTER_SIZE_TYPE						unsigned long

#define portYIELD_FROM_ISR()            portYIELD()
#define portYIELD()     \
    __asm__ __volatile__ (  "l.nop       \n\t"  \
                            "l.sys 0x0FCC\n\t"  \
                            "l.nop       \n\t"  \
    );
#define portNOP()       __asm__ __volatile__ ( "l.nop" )


/*-----------------------------------------------------------*/
#define portDISABLE_INTERRUPTS() { \
    extern inline void vPortDisableInterrupts( void ); \
    vPortDisableInterrupts(); \
}
#define portENABLE_INTERRUPTS() { \
    extern inline void vPortEnableInterrupts( void ); \
    vPortEnableInterrupts(); \
}

#define portENTER_CRITICAL() { \
	extern void vTaskEnterCritical( void ); \
	vTaskEnterCritical(); \
}
#define portEXIT_CRITICAL() { \
    extern void vTaskExitCritical( void ); \
    vTaskExitCritical(); \
}
/*-----------------------------------------------------------*/

/* Task function macros as described on the FreeRTOS.org WEB site. */
#define portTASK_FUNCTION_PROTO( vFunction, pvParameters ) \
	void vFunction( void *pvParameters )
#define portTASK_FUNCTION( vFunction, pvParameters ) \
	void vFunction( void *pvParameters )

/*
    Context layout
    0x00    r9
    0x04    r2
    0x08    r3
    0x0C    r4
    0x10    r5
    0x14    r6
    0x18    r7
    0x1C    r8
    0x20    r10
    0x24    r11
    0x28    r12
    0x2C    r13
    0x30    r14
    0x34    r15
    0x38    r16
    0x3C    r17
    0x40    r18
    0x44    r19
    0x48    r20
    0x4C    r21
    0x50    r22
    0x54    r23
    0x58    r24
    0x5C    r25
    0x60    r26
    0x64    r27
    0x68    r28
    0x6C    r29
    0x70    r30
    0x74    r31
    0x78    ESR
    0x7C    EPCR
*/

#define REDZONE_SIZE        (128)
#define CONTEXT_SIZE        (128)
#define STACKFRAME_SIZE     (CONTEXT_SIZE + REDZONE_SIZE)

#ifdef __cplusplus
}
#endif

#endif /* PORTMACRO_H */
