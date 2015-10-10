/*
    FreeRTOS V8.1.0 - Copyright (C) 2014 Real Time Engineers Ltd.
    All rights reserved

    VISIT http://www.FreeRTOS.org TO ENSURE YOU ARE USING THE LATEST VERSION.

    ***************************************************************************
     *                                                                       *
     *    FreeRTOS provides completely free yet professionally developed,    *
     *    robust, strictly quality controlled, supported, and cross          *
     *    platform software that has become a de facto standard.             *
     *                                                                       *
     *    Help yourself get started quickly and support the FreeRTOS         *
     *    project by purchasing a FreeRTOS tutorial book, reference          *
     *    manual, or both from: http://www.FreeRTOS.org/Documentation        *
     *                                                                       *
     *    Thank you!                                                         *
     *                                                                       *
    ***************************************************************************

    This file is part of the FreeRTOS distribution.

    FreeRTOS is free software; you can redistribute it and/or modify it under
    the terms of the GNU General Public License (version 2) as published by the
    Free Software Foundation >>!AND MODIFIED BY!<< the FreeRTOS exception.

    >>!   NOTE: The modification to the GPL is included to allow you to     !<<
    >>!   distribute a combined work that includes FreeRTOS without being   !<<
    >>!   obliged to provide the source code for proprietary components     !<<
    >>!   outside of the FreeRTOS kernel.                                   !<<

    FreeRTOS is distributed in the hope that it will be useful, but WITHOUT ANY
    WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS
    FOR A PARTICULAR PURPOSE.  Full license text is available from the following
    link: http://www.freertos.org/a00114.html

    1 tab == 4 spaces!

    ***************************************************************************
     *                                                                       *
     *    Having a problem?  Start by reading the FAQ "My application does   *
     *    not run, what could be wrong?"                                     *
     *                                                                       *
     *    http://www.FreeRTOS.org/FAQHelp.html                               *
     *                                                                       *
    ***************************************************************************

    http://www.FreeRTOS.org - Documentation, books, training, latest versions,
    license and Real Time Engineers Ltd. contact details.

    http://www.FreeRTOS.org/plus - A selection of FreeRTOS ecosystem products,
    including FreeRTOS+Trace - an indispensable productivity tool, a DOS
    compatible FAT file system, and our tiny thread aware UDP/IP stack.

    http://www.OpenRTOS.com - Real Time Engineers ltd license FreeRTOS to High
    Integrity Systems to sell under the OpenRTOS brand.  Low cost OpenRTOS
    licenses offer ticketed support, indemnification and middleware.

    http://www.SafeRTOS.com - High Integrity Systems also provide a safety
    engineered and independently SIL3 certified version for use in safety and
    mission critical applications that require provable dependability.

    1 tab == 4 spaces!
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

typedef portSTACK_TYPE          StackType_t;
typedef portBASE_TYPE           BaseType_t;
typedef unsigned portBASE_TYPE  UBaseType_t;
typedef portTickType            TickType_t;

#if( configUSE_16_BIT_TICKS == 1 )
    #error "configUSE_16_BIT_TICKS must be 0"
#endif

#ifndef configSYSTICK_CLOCK_HZ
    #define configSYSTICK_CLOCK_HZ configCPU_CLOCK_HZ
#endif

/*-----------------------------------------------------------*/
#define portSTACK_GROWTH                -1
#define portTICK_PERIOD_MS              ( \
    (portTickType) 1000 / configTICK_RATE_HZ \
)
#define portBYTE_ALIGNMENT              4
#define portCRITICAL_NESTING_IN_TCB     1
#define portINSTRUCTION_SIZE            ( ( portSTACK_TYPE ) 4 )
#define portNO_CRITICAL_SECTION_NESTING ( ( portSTACK_TYPE ) 0 )
#define portPOINTER_SIZE_TYPE           unsigned long

#define portYIELD_FROM_ISR(x)           { if(x) vTaskSwitchContext() }
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
