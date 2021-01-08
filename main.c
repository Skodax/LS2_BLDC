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

/* BIOS Module Headers */
#include <ti/sysbios/BIOS.h>
#include <ti/sysbios/hal/Hwi.h>
#include <ti/sysbios/knl/Task.h>
#include <ti/sysbios/knl/Event.h>
#include <ti/sysbios/knl/Semaphore.h>

/* Drivers header files */
#include "ti_drivers_config.h"
#include <ti/drivers/GPIO.h>
#include <ti/drivers/PWM.h>
#include <ti/drivers/Timer.h>

/* Board header files */
#include <ti/drivers/Board.h>

/* Handlers RTOS */
extern Timer_Handle timerInitialAcceleration;
extern Task_Handle taskPhaseChange;
extern Event_Handle eventPhaseChange;

/* Function declaration */
//void hwiLaunchpadButtonsFx(UArg arg);
void timerInitialAccelerationFx(UArg arg);
void taskPhaseChangeFx(UArg arg1, UArg arg2);
void setPhase(uint8_t phase);

/* PWM - WILL GO INTO A TASK */
PWM_Handle pwmA;
PWM_Handle pwmB;
PWM_Handle pwmC;
PWM_Params pwmParams;
uint32_t   dutyValue;

/*
 *  ======== main ========
 */
int main()
{
    /* Call driver init functions */
    Board_init();
    GPIO_init();
    PWM_init();

    /* Enable interrupts */
    GPIO_enableInt(Board_BUTTON_S1_GPIO);
    GPIO_enableInt(Board_BUTTON_S2_GPIO);

    /* RED LED ON */
    //GPIO_write(Board_LED_0_GPIO, CONFIG_LED_ON);

    /* Start msg */
    System_printf("Starting... \n");
    System_flush();

    /* Start BIOS */
    BIOS_start();
    return(0);
}

// HWI

//void hwiPort1Fx(UArg arg){
//    GPIO_clearInt(Board_BUTTON_S1_GPIO);
//    GPIO_clearInt(Board_BUTTON_S2_GPIO);
//    Event_post(eventPhaseChange, Event_Id_00);
//}

//void hwiButtonS1Fx(void){
//    GPIO_clearInt(Board_BUTTON_S1_GPIO);
//    Event_post(eventPhaseChange, Event_Id_01);
//}
//
//void hwiButtonS2Fx(void){
//    GPIO_clearInt(Board_BUTTON_S2_GPIO);
//    Event_post(eventPhaseChange, Event_Id_02);
//}

// TIMER
void timerInitialAccelerationFx(UArg arg){
    Event_post(eventPhaseChange, Event_Id_00);
}

// TASK
void taskPhaseChangeFx(UArg arg1, UArg arg2){

    uint8_t phase = 0;
    UInt events = 0;
    //int32_t timerStatus = 0;

    // INIT PWM - WILL GO INTO A SEPARATE TASK
    // Initialize the PWM parameters
    PWM_Params_init(&pwmParams);
    pwmParams.idleLevel = PWM_IDLE_LOW;      // Output low when PWM is not running
    pwmParams.periodUnits = PWM_PERIOD_US;   // Period is in us
    pwmParams.periodValue = 100;            // 100us -> 10KHz
    pwmParams.dutyUnits = PWM_DUTY_FRACTION; // Duty is in fractional percentage
    pwmParams.dutyValue = 0;                 // 0% initial duty cycle

    // Open the PWM instance
    pwmA = PWM_open(PHASE_A_HIN, &pwmParams);
    if (pwmA == NULL) {
        System_printf("No s'ha pogut agafar el driver PWM A \n");
        System_flush();
        while (1);
    }

    pwmB = PWM_open(PHASE_B_HIN, &pwmParams);
    if (pwmB == NULL) {
        System_printf("No s'ha pogut agafar el driver PWM B \n");
        System_flush();
        while (1);
    }

    pwmC = PWM_open(PHASE_C_HIN, &pwmParams);
    if (pwmC == NULL) {
        System_printf("No s'ha pogut agafar el driver PWM C \n");
        System_flush();
        while (1);
    }

    // PWM duty
    dutyValue = (uint32_t) (((uint64_t) PWM_DUTY_FRACTION_MAX * 10) / 100);


    // CUT CURRENT TO MOTOR
    setPhase(phase);            // phase = 0, cuts current

    while(1){
        events = Event_pend(eventPhaseChange, Event_Id_NONE, Event_Id_00 /*| Event_Id_01 | Event_Id_02*/, BIOS_WAIT_FOREVER);

        switch(events){
        case Event_Id_00:

            // Phase increment
            phase++;
            if(phase > 6){phase = 1;}

            setPhase(phase);

            // Print
            //System_printf("%d\n", phase);
            //System_flush();

            break;

//        case Event_Id_01:
//
//            // Start Initial acceleration timer
//            timerStatus = Timer_start(timerInitialAcceleration);
//
//            switch (timerStatus) {
//                case Timer_STATUS_SUCCESS:
//                    System_printf("Motor acceleration has started! \n");
//                    System_flush();
//                    break;
//
//                case Timer_STATUS_ERROR:
//                    System_printf("Error occurred on acceleration timer startup! \n");
//                    System_flush();
//                    break;
//
//                default:
//                    System_printf("Unknown error on acceleration timer startup: %d \n", timerStatus);
//                    System_flush();
//                    break;
//            }
//            break;
//
//            case Event_Id_02:
//
//                // Stop Initial acceleration timer
//                Timer_stop(timerInitialAcceleration);
//                System_printf("Motor acceleration has stoped! \n");
//                break;

        default:
            System_printf("Unknown event on taskPhaseChange. Event: %d \n", events);
            System_flush();
            while(1);

        }
    }
}

// FUNCTIONS
void setPhase(uint8_t phase){

    PWM_setDuty(pwmA, dutyValue);
    PWM_setDuty(pwmB, dutyValue);
    PWM_setDuty(pwmC, dutyValue);

    switch (phase) {
        case 1:
            PWM_start(pwmA);
            PWM_stop(pwmB);
            PWM_stop(pwmC);

            GPIO_write(PHASE_A_LIN, 0);
            GPIO_write(PHASE_B_LIN, 1);
            GPIO_write(PHASE_C_LIN, 0);
            break;

        case 2:
            PWM_start(pwmA);
            PWM_stop(pwmB);
            PWM_stop(pwmC);

            GPIO_write(PHASE_A_LIN, 0);
            GPIO_write(PHASE_B_LIN, 0);
            GPIO_write(PHASE_C_LIN, 1);
            break;

        case 3:
            PWM_stop(pwmA);
            PWM_start(pwmB);
            PWM_stop(pwmC);

            GPIO_write(PHASE_A_LIN, 0);
            GPIO_write(PHASE_B_LIN, 0);
            GPIO_write(PHASE_C_LIN, 1);
            break;

        case 4:
            PWM_stop(pwmA);
            PWM_start(pwmB);
            PWM_stop(pwmC);

            GPIO_write(PHASE_A_LIN, 1);
            GPIO_write(PHASE_B_LIN, 0);
            GPIO_write(PHASE_C_LIN, 0);
            break;

        case 5:
            PWM_stop(pwmA);
            PWM_stop(pwmB);
            PWM_start(pwmC);

            GPIO_write(PHASE_A_LIN, 1);
            GPIO_write(PHASE_B_LIN, 0);
            GPIO_write(PHASE_C_LIN, 0);
            break;

        case 6:
            PWM_stop(pwmA);
            PWM_stop(pwmB);
            PWM_start(pwmC);

            GPIO_write(PHASE_A_LIN, 0);
            GPIO_write(PHASE_B_LIN, 1);
            GPIO_write(PHASE_C_LIN, 0);
            break;

        default:

            // CUT CURRENT TO THE MOTOR
            PWM_stop(pwmA);
            PWM_stop(pwmB);
            PWM_stop(pwmC);

            GPIO_write(PHASE_A_LIN, 0);
            GPIO_write(PHASE_B_LIN, 0);
            GPIO_write(PHASE_C_LIN, 0);

            //Todo: Remove when release
            System_printf("Phase: CUT CURRENT \n");
            System_flush();

            break;
    }


}


