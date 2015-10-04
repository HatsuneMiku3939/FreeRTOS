/*
    FreeRTOS V8.0.1 - Copyright (C) 2014 Real Time Engineers Ltd.
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

/*-----------------------------------------------------------
 * Implementation of functions defined in portable.h for the OpenRISC port.
 *----------------------------------------------------------*/


/* Scheduler includes. */
#include "FreeRTOS.h"
#include "task.h"

/* Processor constants. */
#include "port_spr_defs.h"

/* Jump buffer */
#include <setjmp.h>
static jmp_buf jmpbuf;

/* Tick Timer Interrupt handler */
void vTickHandler( void );

/* Setup the timer to generate the tick interrupts. */
static void prvSetupTimerInterrupt( void );

/* For writing into SPR. */
static inline void mtspr(unsigned long spr, unsigned long value)
{
    asm("l.mtspr\t\t%0,%1,0": : "r" (spr), "r" (value));
}

/* For reading SPR. */
static inline unsigned long mfspr(unsigned long spr)
{
    unsigned long value;
    asm("l.mfspr\t\t%0,%1,0" : "=r" (value) : "r" (spr));
    return value;
}

/* forward decleation */
inline void vPortDisableInterrupts( void );
inline void vPortEnableInterrupts( void );

/*
 * Initialise the stack of a task to look exactly as if a call to
 * portSAVE_CONTEXT had been called. Context layout is described in
 * portmarco.h
 *
 * See header file for description.
 */
portSTACK_TYPE *pxPortInitialiseStack( portSTACK_TYPE *pxTopOfStack,
                                       TaskFunction_t pxCode,
                                       void *pvParameters )
{
    unsigned portLONG uTaskSR = mfspr(SPR_SR);

    // Supervisor mode
    uTaskSR |= SPR_SR_SM;
    // Tick interrupt enable, All External interupt enable
    uTaskSR |= (SPR_SR_TEE | SPR_SR_IEE);

    // allocate redzone
    pxTopOfStack -= REDZONE_SIZE/4;

    /* Setup the initial stack of the task.  The stack is set exactly as
    expected by the portRESTORE_CONTEXT() macro. */
    *(--pxTopOfStack) = (portSTACK_TYPE)pxCode;         // SPR_EPCR_BASE(0)
    *(--pxTopOfStack) = (portSTACK_TYPE)uTaskSR;        // SPR_ESR_BASE(0)

    *(--pxTopOfStack) = (portSTACK_TYPE)0x00000031;     // r31
    *(--pxTopOfStack) = (portSTACK_TYPE)0x00000030;     // r30
    *(--pxTopOfStack) = (portSTACK_TYPE)0x00000029;     // r29
    *(--pxTopOfStack) = (portSTACK_TYPE)0x00000028;     // r28
    *(--pxTopOfStack) = (portSTACK_TYPE)0x00000027;     // r27
    *(--pxTopOfStack) = (portSTACK_TYPE)0x00000026;     // r26
    *(--pxTopOfStack) = (portSTACK_TYPE)0x00000025;     // r25
    *(--pxTopOfStack) = (portSTACK_TYPE)0x00000024;     // r24
    *(--pxTopOfStack) = (portSTACK_TYPE)0x00000023;     // r23
    *(--pxTopOfStack) = (portSTACK_TYPE)0x00000022;     // r22
    *(--pxTopOfStack) = (portSTACK_TYPE)0x00000021;     // r21
    *(--pxTopOfStack) = (portSTACK_TYPE)0x00000020;     // r20
    *(--pxTopOfStack) = (portSTACK_TYPE)0x00000019;     // r19
    *(--pxTopOfStack) = (portSTACK_TYPE)0x00000018;     // r18
    *(--pxTopOfStack) = (portSTACK_TYPE)0x00000017;     // r17
    *(--pxTopOfStack) = (portSTACK_TYPE)0x00000016;     // r16
    *(--pxTopOfStack) = (portSTACK_TYPE)0x00000015;     // r15
    *(--pxTopOfStack) = (portSTACK_TYPE)0x00000014;     // r14
    *(--pxTopOfStack) = (portSTACK_TYPE)0x00000013;     // r13
    *(--pxTopOfStack) = (portSTACK_TYPE)0x00000012;     // r12
    *(--pxTopOfStack) = (portSTACK_TYPE)0x00000011;     // r11
    *(--pxTopOfStack) = (portSTACK_TYPE)0x00000010;     // r10
    *(--pxTopOfStack) = (portSTACK_TYPE)0x00000008;     // r8
    *(--pxTopOfStack) = (portSTACK_TYPE)0x00000007;     // r7
    *(--pxTopOfStack) = (portSTACK_TYPE)0x00000006;     // r6
    *(--pxTopOfStack) = (portSTACK_TYPE)0x00000005;     // r5
    *(--pxTopOfStack) = (portSTACK_TYPE)0x00000004;     // r4
    *(--pxTopOfStack) = (portSTACK_TYPE)pvParameters;   // task argument
    *(--pxTopOfStack) = (portSTACK_TYPE)0x00000002;     // r2
    *(--pxTopOfStack) = (portSTACK_TYPE)pxCode;         // PC

    return pxTopOfStack;
}

portBASE_TYPE xPortStartScheduler( void )
{
    if(setjmp((void *)jmpbuf) == 0) {
        /* Start the timer that generates the tick ISR.
     * Interrupts are disabled here already. */
        prvSetupTimerInterrupt();

        /* Start the first task. */
        asm volatile (
        " .global   pxCurrentTCB    \n\t"
        /*   restore stack pointer          */
        "   l.movhi r3, hi(pxCurrentTCB)        \n\t"
        "   l.ori   r3, r3, lo(pxCurrentTCB)    \n\t"
        "   l.lwz   r3, 0x0(r3)     \n\t"
        "   l.lwz   r1, 0x0(r3)     \n\t"
        /*   restore context                */
        "   l.lwz   r9,  0x00(r1)   \n\t"
        "   l.lwz   r2,  0x04(r1)   \n\t"
        "   l.lwz   r6,  0x14(r1)   \n\t"
        "   l.lwz   r7,  0x18(r1)   \n\t"
        "   l.lwz   r8,  0x1C(r1)   \n\t"
        "   l.lwz   r10, 0x20(r1)   \n\t"
        "   l.lwz   r11, 0x24(r1)   \n\t"
        "   l.lwz   r12, 0x28(r1)   \n\t"
        "   l.lwz   r13, 0x2C(r1)   \n\t"
        "   l.lwz   r14, 0x30(r1)   \n\t"
        "   l.lwz   r15, 0x34(r1)   \n\t"
        "   l.lwz   r16, 0x38(r1)   \n\t"
        "   l.lwz   r17, 0x3C(r1)   \n\t"
        "   l.lwz   r18, 0x40(r1)   \n\t"
        "   l.lwz   r19, 0x44(r1)   \n\t"
        "   l.lwz   r20, 0x48(r1)   \n\t"
        "   l.lwz   r21, 0x4C(r1)   \n\t"
        "   l.lwz   r22, 0x50(r1)   \n\t"
        "   l.lwz   r23, 0x54(r1)   \n\t"
        "   l.lwz   r24, 0x58(r1)   \n\t"
        "   l.lwz   r25, 0x5C(r1)   \n\t"
        "   l.lwz   r26, 0x60(r1)   \n\t"
        "   l.lwz   r27, 0x64(r1)   \n\t"
        "   l.lwz   r28, 0x68(r1)   \n\t"
        "   l.lwz   r29, 0x6C(r1)   \n\t"
        "   l.lwz   r30, 0x70(r1)   \n\t"
        "   l.lwz   r31, 0x74(r1)   \n\t"
        /*  restore SPR_ESR_BASE(0), SPR_EPCR_BASE(0) */
        "   l.lwz   r3,  0x78(r1)   \n\t"
        "   l.lwz   r4,  0x7C(r1)   \n\t"
        "   l.mtspr r0,  r3, %1     \n\t"
        "   l.mtspr r0,  r4, %2     \n\t"
        /*   restore clobber register     */
        "   l.lwz   r3,  0x08(r1)   \n\t"
        "   l.lwz   r4,  0x0C(r1)   \n\t"
        "   l.lwz   r5,  0x10(r1)   \n\t"
        "   l.addi  r1,  r1, %0     \n\t"
        "   l.rfe                   \n\t"
        "   l.nop                   \n\t"
        :
        :   "n"(STACKFRAME_SIZE),
            "n"(SPR_ESR_BASE),
            "n"(SPR_EPCR_BASE)
        );

        /* Should not get here! */
    } else {
        /* Retrun by vPortEndScheduler */
    }

    return 0;
}

void vPortEndScheduler( void )
{
    // Tick stop
    mtspr(SPR_SR, mfspr(SPR_SR) & (~SPR_SR_TEE));

    // return to xPortStartScheduler
    longjmp((void *)jmpbuf, 1);
}

/*
 * Setup the tick timer to generate the tick interrupts at the required frequency.
 */
static void prvSetupTimerInterrupt( void )
{
    const unsigned portLONG ulTickPeriod =
        configSYSTICK_CLOCK_HZ / configTICK_RATE_HZ;

    // Disable tick timer exception recognition
    mtspr(SPR_SR, mfspr(SPR_SR) & ~SPR_SR_TEE);

    // clears interrupt
    mtspr(SPR_TTMR, mfspr(SPR_TTMR) & ~(SPR_TTMR_IP));

    // Set period of one cycle, restartable mode
    mtspr(SPR_TTMR,
      SPR_TTMR_IE | SPR_TTMR_RT | (ulTickPeriod & SPR_TTMR_PERIOD));

    // Reset counter
    mtspr(SPR_TTCR, 0);

    // set OR1200 to accept exceptions
    mtspr(SPR_SR, mfspr(SPR_SR) | SPR_SR_TEE);
}

inline void vPortDisableInterrupts( void )
{
    // Tick, interrupt stop
    mtspr(SPR_SR, mfspr(SPR_SR) & ~(SPR_SR_TEE|SPR_SR_IEE));
}

inline void vPortEnableInterrupts( void )
{
    // Tick, interrupt start
    mtspr(SPR_SR, mfspr(SPR_SR) | (SPR_SR_TEE|SPR_SR_IEE));
}

/*
 * naked attribute is ignored or32-elf-gcc 4.5.1-or32-1.0rc1
 * use assemble routines in portasm.S
 */
#if 0
void vTickHandler( void )
{
    // clears interrupt
    mtspr(SPR_TTMR, mfspr(SPR_TTMR) & ~(SPR_TTMR_IP));

    /* Increment the RTOS tick count, then look for the highest priority
       task that is ready to run. */
    xTaskIncrementTick();

    // The cooperative scheduler requires a normal simple Tick ISR to
    // simply increment the system tick.
#if configUSE_PREEMPTION == 0
    // nothing to do here
#else
    /* Save the context of the current task. */
    portSAVE_CONTEXT();

    /* Find the highest priority task that is ready to run. */
    vTaskSwitchContext();

    portRESTORE_CONTEXT();
#endif
}
#endif
