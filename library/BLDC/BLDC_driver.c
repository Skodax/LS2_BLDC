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
/* Motor control Events */
#define EVENT_MOTOR_MBX_SPEED       Event_Id_00
#define EVENT_MOTOR_ACCELERATE      Event_Id_01
#define EVENT_MOTOR_STOP            Event_Id_02

/* Motor phase change Events */
#define EVENT_PHASE_STOP            Event_Id_00
#define EVENT_CHANGE_PHASE          Event_Id_01

/* Speed calculation Events */
#define EVENT_MBX_TIME              Event_Id_00
#define EVENT_SPEED_0               Event_Id_01

/* Speed calculations */
#define STEPS_PER_LAP               43              // Motor steps per lap. Number of phase changes needed to complete a lap. Todo: Make sure 42 are the actual steps of the motor
#define TIME_BUFF_LEN               32              // Time buffer lenght for speed calculation
#define TIME_AVG_SHIFT              5               // Shift factor for average calculation

/* Motor status */
#define MOTOR_CTR_OL                1               // Motor control type -> Open loop
#define MOTOR_CTR_CL                0               // Motor control type -> Closed loop

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
extern Task_Handle taskMotorControl;
extern Task_Handle taskPhaseChange;
extern Task_Handle taskSpeedCalculator;
extern Event_Handle eventMotorControl;
extern Event_Handle eventPhaseChange;
extern Event_Handle eventSpeed;
extern Mailbox_Handle mbxPhaseChangeTime;
extern Mailbox_Handle mbxTheoricalSpeed;
extern Mailbox_Handle mbxMotorSpeed;


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
void taskMotorControlFx(UArg arg1, UArg arg2);
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
void taskMotorControlFx(UArg arg1, UArg arg2){

    /* Events */
    uint32_t events;

    /* Motor status */
    int32_t speed = 0;                              // Motor stopped by default
    uint8_t ctrlType = MOTOR_CTR_OL;                // Motor control Open Loop by default
    uint32_t timerPeriod = 0;                       // Timer period for OL control
    uint32_t dutyCycle = 0;                         // Duty cycle for CL control

    /* External control */
    int16_t joystick = 0;                           // No actions by default

    /* Main task loop */
    while(1){

        /* Wait for events */
        events = Event_pend(eventMotorControl, Event_Id_NONE, EVENT_MOTOR_MBX_SPEED | EVENT_MOTOR_ACCELERATE | EVENT_MOTOR_STOP, BIOS_WAIT_FOREVER);

        /* Get data */
        Mailbox_pend(mbxMotorSpeed, &joystick, BIOS_NO_WAIT);                       // Get joystick data

        /* When MOTOR STOP event is trigger no other event is executed */
        if((events & EVENT_MOTOR_STOP)){

            /* Stop motor */
            speed = dutyCycle = timerPeriod = joystick = 0;     // Reset control variables
            ctrlType = MOTOR_CTR_OL;                            // Set motor control to OL
            Timer_stop(timerInitialAcceleration);               // Stop OpenLoop control timer
            Timer_stop(timerAccelerator);                       // Stop OpenLoop accelerator timer

            /* Cut current */
            Event_post(eventPhaseChange, EVENT_PHASE_STOP);     // Cut power to the motor

        } else {

            if(events & EVENT_MOTOR_MBX_SPEED){
                if(ctrlType == MOTOR_CTR_OL){

                    /* Open Loop control */
                    speed += (int32_t) joystick;                // Accelerate depending on the joysticks position
                    if(speed < 0){speed = 0;}                   // Minimum speed -> 0
                    // Todo: maximum speed

                    if(!speed){
                        Event_post(eventMotorControl, EVENT_MOTOR_STOP);        // If speed is 0 then stop the motor
                    } else {

                        /* Convert to Timer period and Duty Cycle */
                        timerPeriod = MOTOR_INITIAL_PERIOD - speed;
                        Timer_setPeriod(timerInitialAcceleration, timerPeriod);
                        Timer_start(timerInitialAcceleration);
                    }



                } else {

                    /* Safe stop */
                    Event_post(eventPhaseChange, EVENT_PHASE_STOP);     // Cut power to the motor

                    /* Report error and halt */
                    System_printf("Unknown ctrlType on taskMotorControl. Event: %d \n", ctrlType);
                    System_flush();
                    while(1);
                }
            }
        }

        /* Unexpected event detector */
        if(events & ~(EVENT_MOTOR_MBX_SPEED | EVENT_MOTOR_ACCELERATE | EVENT_MOTOR_STOP)){

            /* Safe stop */
            Event_post(eventPhaseChange, EVENT_PHASE_STOP);     // Cut power to the motor

            /* Report error and halt */
            System_printf("Unknown event on taskMotorControl. Event: %d \n", events);
            System_flush();
            while(1);
        }
    }
}

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
        events = Event_pend(eventPhaseChange, Event_Id_NONE, EVENT_PHASE_STOP | EVENT_CHANGE_PHASE, BIOS_WAIT_FOREVER);

        /* EVENT ACTIONS */
        if(events & EVENT_PHASE_STOP){

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

    /* Events */
    uint32_t events;

    /* Time buffer */
    uint8_t i = 0;
    uint32_t timeBuff[TIME_BUFF_LEN];
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

        events = Event_pend(eventSpeed, Event_Id_NONE, EVENT_SPEED_0 | EVENT_MBX_TIME, BIOS_WAIT_FOREVER);

        if(events & EVENT_SPEED_0){

            /* Send speed */
            speed = 0;                                                      // Motor has stoped
            Mailbox_post(mbxTheoricalSpeed, &speed, BIOS_WAIT_FOREVER);     // Wait until speed can be printed

        } else if(events & EVENT_MBX_TIME){

            /* Get time buffer from taskPhaseChange */
            Mailbox_pend(mbxPhaseChangeTime, timeBuff, BIOS_NO_WAIT);

            /* Average calculation */
            time = 0;                                                   // Reset accumulator
            for(i = 0; i < TIME_BUFF_LEN; i++){                         // Sum all times in the buffer
                time += timeBuff[i];
            }

            time >>= TIME_AVG_SHIFT;                                    // Divide by the number of sumands

            /* RPM conversion */
            speed = speedHumanize / time;                               // Convert into RPM

            /* Send speed */
            Mailbox_post(mbxTheoricalSpeed, &speed, BIOS_NO_WAIT);      // Send data to LCD
        }

        //System_printf("Motor speed (RPM): %d \n", speed);         // Don't flush so execution isn't iterrupted

    }
}


/****************************************************************************************************************************************************
 *      FUNCTIONS
 ****************************************************************************************************************************************************/

/* Public */ // Deprected
//void BLDC_start(void){
//
//    /* Start Initial acceleration timer */
//    Timer_setPeriod(timerInitialAcceleration, MOTOR_INITIAL_PERIOD);
//    Timer_start(timerInitialAcceleration);
//    Timer_start(timerAccelerator);
//}

void BLDC_stop(void){

    /* Stop Initial acceleration timer and motor */
//    Timer_stop(timerInitialAcceleration);
//    Timer_stop(timerAccelerator);
//    Event_post(eventPhaseChange, EVENT_PHASE_STOP);

    //UInt32 timerPeriod;
    //timerPeriod = Timer_getPeriod(timerInitialAcceleration);
    //System_printf("Timer Period: %d \n", timerPeriod);
    Event_post(eventMotorControl, EVENT_MOTOR_STOP);
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

            // Set speed tot 0
            Event_post(eventSpeed, EVENT_SPEED_0);

            //Todo: Remove when release
            System_printf("Phase: CUT CURRENT \n");
            System_flush();

            break;
    }
}
