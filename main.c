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

/****************************************************************************************************************************************************
 *      LS2 BLDC MOTOR CONTROLER
 ****************************************************************************************************************************************************/

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

/* User libraries */
#include "library/ADC/ADC.h"

/****************************************************************************************************************************************************
 *      Main Function
 ****************************************************************************************************************************************************/
int main()
{
    /* Call driver initializer functions */
    Board_init();
    GPIO_init();
    PWM_init();
    //ADCBuf_init();

    bool error = true;
    error = ADCinit();
    if(!error){
        System_printf("ADC initialization failed! \n");
       System_flush();
    }

    /* Enable interrupts */
    GPIO_enableInt(Board_BUTTON_S1_GPIO);
    GPIO_enableInt(Board_BUTTON_S2_GPIO);
    GPIO_enableInt(MKII_BUTTON1_GPIO);
    GPIO_enableInt(MKII_BUTTON2_GPIO);

    /* Clear all possible interrupts */
    GPIO_clearInt(Board_BUTTON_S1_GPIO);
    GPIO_clearInt(Board_BUTTON_S2_GPIO);
    GPIO_clearInt(MKII_BUTTON1_GPIO);
    GPIO_clearInt(MKII_BUTTON2_GPIO);

    /* Start message */
    System_printf("Starting... \n");
    System_flush();

    /* Start BIOS */
    BIOS_start();
    return(0);
}
