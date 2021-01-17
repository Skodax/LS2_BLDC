/*
 * Copyright (c) 2015-2019, Texas Instruments Incorporated
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * *  Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 *
 * *  Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 *
 * *  Neither the name of Texas Instruments Incorporated nor the names of
 *    its contributors may be used to endorse or promote products derived
 *    from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR
 * PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR
 * CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,
 * EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
 * PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS;
 * OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
 * WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR
 * OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
 * EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

/*
 *  ======== LS2_BLDC ========
 */

/* XDC Module Headers */
#include <xdc/std.h>
#include <xdc/runtime/System.h>
#include <xdc/runtime/Diags.h>
#include <xdc/runtime/Log.h>
#include <xdc/runtime/Timestamp.h>
#include <xdc/runtime/Types.h>

/* BIOS Module Headers */
#include <ti/sysbios/BIOS.h>

#include <ti/sysbios/hal/Timer.h>
#include <ti/sysbios/hal/Hwi.h>

#include <ti/sysbios/knl/Swi.h>
#include <ti/sysbios/knl/Task.h>
#include <ti/sysbios/knl/Idle.h>

#include <ti/sysbios/knl/Event.h>
#include <ti/sysbios/knl/Semaphore.h>
#include <ti/sysbios/knl/Mailbox.h>

/* Drivers header files */
#include "ti_drivers_config.h"
#include <ti/drivers/GPIO.h>
#include <ti/drivers/PWM.h>
#include <ti/drivers/ADCBuf.h>

/* Board header files */
#include <ti/drivers/Board.h>

/* Handlers RTOS */
extern Task_Handle taskMotorControl;
extern Event_Handle eventMotorControl;

/* User libraries */
#include "library/BLDC/BLDC_driver.h"
#include "library/LCD/LCD.h"



/* Function declaration */
void taskMotorControlFx(UArg arg1, UArg arg2);

/*
 *  ======== main ========
 */
int main()
{
    /* Call driver init functions */
    Board_init();
    GPIO_init();
    PWM_init();
    ADCBuf_init();

    /* Enable interrupts */
    GPIO_enableInt(Board_BUTTON_S1_GPIO);
    GPIO_enableInt(Board_BUTTON_S2_GPIO);
    GPIO_enableInt(MKII_BUTTON1_GPIO);
    GPIO_enableInt(MKII_BUTTON2_GPIO);

    /* Start msg */
    System_printf("Starting... \n");
    System_flush();

    /* Start BIOS */
    BIOS_start();
    return(0);
}

// HWI - ?

void hwiButtonS1Fx(void){
    GPIO_clearInt(Board_BUTTON_S1_GPIO);
    Event_post(eventMotorControl, Event_Id_01);
}

void hwiMkiiButton2Fx(void){
    GPIO_clearInt(MKII_BUTTON2_GPIO);
    Event_post(eventMotorControl, Event_Id_01);
}

void hwiButtonS2Fx(void){
    GPIO_clearInt(Board_BUTTON_S2_GPIO);
    Event_post(eventMotorControl, Event_Id_02);
}

void hwiMkiiButton1Fx(void){
    GPIO_clearInt(MKII_BUTTON1_GPIO);
    Event_post(eventMotorControl, Event_Id_02);
}

// TIMER


// SWI


// TASK
void taskMotorControlFx(UArg arg1, UArg arg2){

    UInt events = 0;

    while(1){

            events = Event_pend(eventMotorControl, Event_Id_NONE, Event_Id_00 | Event_Id_01 | Event_Id_02 , BIOS_WAIT_FOREVER);

            switch(events){
            case Event_Id_00:
                // UNUSED
//                System_printf("Timer Period: %d \n", timerPeriod);
//                System_flush();
                break;

            case Event_Id_01:
                BLDC_start();
                System_printf("Motor acceleration has started! \n");
                System_flush();
                break;

            case (Event_Id_01 | Event_Id_02):                                   // Start motor and Stop motor (simulatneously) -> stop motor
                System_printf("Motor Start & Stop simultaneous trigger \n");     // Todo: Remove when realease
            case Event_Id_02:
                BLDC_stop();
                System_printf("Motor acceleration has stoped! \n");
                System_flush();
                break;

            default:
                System_printf("Unknown event on taskMotorControl. Event: %d \n", events);
                System_flush();
                while(1);

            }
        }
}



