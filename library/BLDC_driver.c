/*
 * BLDC_driver.c
 *
 *  Created on: Jan 9, 2021
 *      Author: jango
 */

/****************************************************************************************************************************************************
 *      DEPENDENCIES
 ****************************************************************************************************************************************************/

/* HEADER FILE */
#include "BLDC_driver.h"

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

/* Board header files */
#include <ti/drivers/Board.h>


/****************************************************************************************************************************************************
 *      MACROS
 ****************************************************************************************************************************************************/
/* Motor phase change Events */
#define EVENT_MOTOR_STOP            Event_Id_00
#define EVENT_CHANGE_PHASE          Event_Id_01

/* Speed calculations */
#define STEPS_PER_LAP               43              // Motor steps per lap. Number of phase changes needed to complete a lap. Todo: Make sure 42 are the actual steps of the motor
#define TIME_BUFF_LEN               32              // Time buffer lenght for speed calculation
#define TIME_AVG_SHIFT              5               // Shift factor for average calculation

/* Motor acceleration */
#define MOTOR_INITIAL_PWM           10      //40
#define MOTOR_INITIAL_PERIOD        105
#define MOTOR_MIN_PERIOD            35      //20
#define MOTOR_INITIAL_ACC           5       //1


/****************************************************************************************************************************************************
 *      RTOS HANDLERS
 ****************************************************************************************************************************************************/

extern Timer_Handle timerInitialAcceleration;
extern Timer_Handle timerAccelerator;
extern Swi_Handle swiAccelerateMotor;
extern Task_Handle taskPhaseChange;
extern Task_Handle taskSpeedCalculator;
extern Event_Handle eventPhaseChange;
extern Mailbox_Handle mbxPhaseChangeTime;

/****************************************************************************************************************************************************
 *      DIVER HANDLERS
 ****************************************************************************************************************************************************/
PWM_Handle PWM_A;
PWM_Handle PWM_B;
PWM_Handle PWM_C;

/****************************************************************************************************************************************************
 *      FUNCTION DECLARATION
 ****************************************************************************************************************************************************/
void timerInitialAccelerationFx(UArg arg);
void timerAcceleratorFx(UArg arg);
void taskPhaseChangeFx(UArg arg1, UArg arg2);
void taskSpeedCalculatorFx(UArg arg1, UArg arg2);
void setPhase(uint8_t phase);

/****************************************************************************************************************************************************
 *      HWI
 ****************************************************************************************************************************************************/

void timerInitialAccelerationFx(UArg arg){
    // CHANGE PHASE
    Event_post(eventPhaseChange, EVENT_CHANGE_PHASE);
}

void timerAcceleratorFx(UArg arg){
    Swi_post(swiAccelerateMotor);
}


/****************************************************************************************************************************************************
 *      SWI
 ****************************************************************************************************************************************************/

void swiAccelerateMotorFx(UArg arg1, UArg arg2){

    // ACCELERATE MOTOR
    // Reduce timer period
    UInt32 timerPeriod;
    timerPeriod = Timer_getPeriod(timerInitialAcceleration);
    timerPeriod -= MOTOR_INITIAL_ACC;       // 5

    if(timerPeriod > MOTOR_MIN_PERIOD){   // 35
        Timer_setPeriod(timerInitialAcceleration, timerPeriod);
        Timer_start(timerInitialAcceleration);
    } else {
        Timer_stop(timerAccelerator);
    }

}


/****************************************************************************************************************************************************
 *      TASK
 ****************************************************************************************************************************************************/

void taskPhaseChangeFx(UArg arg1, UArg arg2){

    /* PWM INITIALIZATION */
    // Initialize the PWM parameters
    PWM_Params PWM_params;
    PWM_Params_init(&PWM_params);
    PWM_params.idleLevel = PWM_IDLE_LOW;         // Output low when PWM is not running
    PWM_params.periodUnits = PWM_PERIOD_US;      // Period is in us
    PWM_params.periodValue = 100;                // 100us -> 10KHz
    PWM_params.dutyUnits = PWM_DUTY_FRACTION;    // Duty is in fractional percentage
    PWM_params.dutyValue = 0;                    // 0% initial duty cycle

    // Open the PWM instances
    PWM_A = PWM_open(PHASE_A_HIN, &PWM_params);
    PWM_B = PWM_open(PHASE_B_HIN, &PWM_params);
    PWM_C = PWM_open(PHASE_C_HIN, &PWM_params);

    // Check if PWM have been opened
    if (PWM_A == NULL || PWM_B == NULL || PWM_C == NULL) {
        System_printf("No s'ha pogut agafar el driver per els PWM dels motors \n");
        System_flush();
        while (1);
    }

    /* Duty cycle and phase */
    uint8_t phase = 0;
    uint32_t dutyCycle;
    dutyCycle = (uint32_t) (((uint64_t) PWM_DUTY_FRACTION_MAX * MOTOR_INITIAL_PWM) / 100);     // Initial duty -> cycle 10%

    /* Events */
    UInt events = 0;

    /* Motor initialization */
    /* Stop and cut power to the motor */
    setPhase(0);                                                                // phase = 0, cuts power

    /* Speed measurement */
    uint32_t tstart, tstop = 0;                                                 // Timestamps store
    uint32_t timeBuff[TIME_BUFF_LEN];                                           // Phase change times buffer
    uint8_t i = 0;                                                              // Buffer index

    /* MAIN TASK LOOP */
    while(1){

        /*
         * WAIT FOR EVENTS
         *  - EVENT_MOTOR_STOP:     Stop the motor
         *  - EVENT_CHANGE_PHASE:   Switches to next phase of the motor
         */
        events = Event_pend(eventPhaseChange, Event_Id_NONE, EVENT_MOTOR_STOP | EVENT_CHANGE_PHASE, BIOS_WAIT_FOREVER);

        /* EVENT ACTIONS */
        if(events & EVENT_MOTOR_STOP){

            /* Reset state */
            phase = tstart = tstop = i = 0;                                         // Reset phase and speed calculating parameters
            setPhase(phase);                                                        // Stop motor

        } else if (events & EVENT_CHANGE_PHASE){

            /* Phase change */
            phase++;
            if(phase > 6){phase = 1;}

            /* Time between phase changes */
            tstop = Timestamp_get32();
            if(tstart){                                                             // If there's a previous phase
                timeBuff[i] = tstop - tstart;                                       // Time between current phase and previous phase

                i++;                                                                // Increment buffer index
                i %= TIME_BUFF_LEN;                                                 // Keep index between 0 and TIME_BUFF_LEN

                if(!i){                                                             // If buffer is full
                    Mailbox_post(mbxPhaseChangeTime, timeBuff, BIOS_NO_WAIT);       // Then send it to speed calculator
                }
            }
            tstart = tstop;                                                         // Start counting for next phase

            /* Motor State */
            PWM_setDuty(PWM_A, dutyCycle);
            PWM_setDuty(PWM_B, dutyCycle);
            PWM_setDuty(PWM_C, dutyCycle);
            setPhase(phase);

        } else {

            /* Unknown event */
            setPhase(0);                                                                // Stop motor
            System_printf("Unknown event on taskPhaseChange. Event: %d \n", events);    // Report error
            System_flush();
            while(1);                                                                   // Halts program. Todo: Change to System_abort

        }
    }
}

void taskSpeedCalculatorFx(UArg arg1, UArg arg2){

    /* Time buffer */
    uint8_t i = 0;
    uint32_t timeBuffa[TIME_BUFF_LEN];
    uint32_t time;

    /* Speed conversion */
    int32_t speed = 0;                                      // Motor speed
    Types_FreqHz freq;
    Timestamp_getFreq(&freq);                               // Get timestamp module freq.
    int32_t speedHumanize = 60 * freq.lo / STEPS_PER_LAP;   // Constant to convert from timestamp units to RMP
                                                            // 60 is for converting sec. to min.
                                                            // freq.lo is for converting timestamp counts to seconds
                                                            // STEPS_PER_LAP are the steps (phase changes) to complete a lap in the motor

    while(1){

        /* Get time buffer from taskPhaseChange */
        Mailbox_pend(mbxPhaseChangeTime, timeBuffa, BIOS_WAIT_FOREVER);

        /* Average calculation */
        time = 0;                                               // Reset accumulator
        for(i = 0; i < TIME_BUFF_LEN; i++){                     // Sum all times in the buffer
            time += timeBuffa[i];
        }

        time >>= TIME_AVG_SHIFT;                                // Divide by the number of sumands

        /* Print speed */
        speed = speedHumanize / time;                           // Convert into RPM
        System_printf("Motor speed (RPM): %d \n", speed);       // Don't flush so execution isn't iterrupted

    }
}


/****************************************************************************************************************************************************
 *      FUNCTIONS
 ****************************************************************************************************************************************************/

/* Public */
void BLDC_start(void){

    /* Start Initial acceleration timer */
    Timer_setPeriod(timerInitialAcceleration, MOTOR_INITIAL_PERIOD);
    Timer_start(timerInitialAcceleration);
    Timer_start(timerAccelerator);
}

void BLDC_stop(void){

    /* Stop Initial acceleration timer and motor */
    Timer_stop(timerInitialAcceleration);
    Timer_stop(timerAccelerator);
    Event_post(eventPhaseChange, EVENT_MOTOR_STOP);

    UInt32 timerPeriod;
    timerPeriod = Timer_getPeriod(timerInitialAcceleration);
    System_printf("Timer Period: %d \n", timerPeriod);
}

/* Private */
void setPhase(uint8_t phase){

    switch (phase) {
        case 1:
            PWM_start(PWM_A);
            PWM_stop(PWM_B);
            PWM_stop(PWM_C);

            GPIO_write(PHASE_A_LIN, 0);
            GPIO_write(PHASE_B_LIN, 1);
            GPIO_write(PHASE_C_LIN, 0);
            break;

        case 2:
            PWM_start(PWM_A);
            PWM_stop(PWM_B);
            PWM_stop(PWM_C);

            GPIO_write(PHASE_A_LIN, 0);
            GPIO_write(PHASE_B_LIN, 0);
            GPIO_write(PHASE_C_LIN, 1);
            break;

        case 3:
            PWM_stop(PWM_A);
            PWM_start(PWM_B);
            PWM_stop(PWM_C);

            GPIO_write(PHASE_A_LIN, 0);
            GPIO_write(PHASE_B_LIN, 0);
            GPIO_write(PHASE_C_LIN, 1);
            break;

        case 4:
            PWM_stop(PWM_A);
            PWM_start(PWM_B);
            PWM_stop(PWM_C);

            GPIO_write(PHASE_A_LIN, 1);
            GPIO_write(PHASE_B_LIN, 0);
            GPIO_write(PHASE_C_LIN, 0);
            break;

        case 5:
            PWM_stop(PWM_A);
            PWM_stop(PWM_B);
            PWM_start(PWM_C);

            GPIO_write(PHASE_A_LIN, 1);
            GPIO_write(PHASE_B_LIN, 0);
            GPIO_write(PHASE_C_LIN, 0);
            break;

        case 6:
            PWM_stop(PWM_A);
            PWM_stop(PWM_B);
            PWM_start(PWM_C);

            GPIO_write(PHASE_A_LIN, 0);
            GPIO_write(PHASE_B_LIN, 1);
            GPIO_write(PHASE_C_LIN, 0);
            break;

        default:

            // CUT CURRENT TO THE MOTOR
            PWM_stop(PWM_A);
            PWM_stop(PWM_B);
            PWM_stop(PWM_C);

            GPIO_write(PHASE_A_LIN, 0);
            GPIO_write(PHASE_B_LIN, 0);
            GPIO_write(PHASE_C_LIN, 0);

            //Todo: Remove when release
            System_printf("Phase: CUT CURRENT \n");
            System_flush();

            break;
    }
}
