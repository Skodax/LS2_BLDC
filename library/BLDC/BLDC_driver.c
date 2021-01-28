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

/* Driverib */
#include <ti/devices/msp432p4xx/driverlib/driverlib.h>
//#include <ti/devices/msp432p4xx/driverlib/gpio.h>                           // Used for retrieving IFG from the ports

/* Board header files */
#include <ti/drivers/Board.h>

/* User libraries */
#include "../utilities/utilities.h"
#include "../ADC/ADC.h"                                                     // Used for proper ADC configuration when motor phase changes


/****************************************************************************************************************************************************
 *      MACROS
 ****************************************************************************************************************************************************/
/* Motor control Events */
#define EVENT_MOTOR_MBX_SPEED       Event_Id_00
#define EVENT_MOTOR_TOGGLE_STATUS   Event_Id_01
#define EVENT_MOTOR_STOP            Event_Id_02

/* Motor phase change Events */
#define EVENT_PHASE_STOP            Event_Id_00
#define EVENT_CHANGE_PHASE          Event_Id_01

/* Speed calculation Events */
#define EVENT_ELECTRIC_REVOLUTION   Event_Id_00
#define EVENT_SPEED_0               Event_Id_01

/* Speed calculations */
#define STEPS_PER_LAP               43              // Motor steps per lap. Number of phase changes needed to complete a lap. Todo: Make sure 42 are the actual steps of the motor
#define STEPS_PER_ELECTRIC_REV      6               // There's 6 phase changes in a complete electric revolution
#define ELECTRIC_REV_PER_LAP        (STEPS_PER_LAP/STEPS_PER_ELECTRIC_REV)
#define TIME_BUFF_LEN               32              // Number of time samples to average in order to calculate motor's speed
#define TIME_AVG_SHIFT              5               // Shift factor for average calculation

/* Motor status */
#define MOTOR_ENABLED               1               // Motor enabled and ready to go
#define MOTOR_DISABLED              0
#define MOTOR_CTR_OL                1               // Motor control type -> Open loop
#define MOTOR_CTR_CL                0               // Motor control type -> Closed loop

/* Motor limits */
#define MOTOR_OL_MAX_SPEED          2500            // Max speed with the MOTOR_MAX_DUTY

/****************************************************************************************************************************************************
 *      RTOS HANDLERS
 ****************************************************************************************************************************************************/
extern Hwi_Handle hwiPort1;
extern Hwi_Handle hwiPort3;
extern Hwi_Handle hwiPort5;

extern Swi_Handle swiMotorStop;
extern Swi_Handle swiMotorToggleStatus;

extern Task_Handle taskMotorControl;
extern Task_Handle taskPhaseChange;
extern Task_Handle taskSpeedCalculator;

extern Timer_Handle timerMotorControlOL;
extern Timer_Handle timerMotorAcc;

extern Event_Handle eventMotorControl;
extern Event_Handle eventPhaseChange;
extern Event_Handle eventSpeed;

extern Mailbox_Handle mbxPhaseChangeTime;
extern Mailbox_Handle mbxTheoricalSpeed;
extern Mailbox_Handle mbxMotorSpeed;
extern Mailbox_Handle mbxDutyCycle;

/****************************************************************************************************************************************************
 *      DIVER HANDLERS
 ****************************************************************************************************************************************************/
PWM_Handle PWM_A;
PWM_Handle PWM_B;
PWM_Handle PWM_C;
PWM_Handle BEMF_REF;

/****************************************************************************************************************************************************
 *      FUNCTION DECLARATION
 ****************************************************************************************************************************************************/
void hwiPort1Fx(UArg arg);
void hwiPort3Fx(UArg arg);
void hwiPort5Fx(UArg arg);
void timerMotorControlOLFx(UArg arg);
void timerMotorAccFx(UArg arg);

void swiMotorStopFx(UArg arg1, UArg arg2);
void swiMotorToggleStatusFx(UArg arg1, UArg arg2);

void taskMotorControlFx(UArg arg1, UArg arg2);
void taskPhaseChangeFx(UArg arg1, UArg arg2);
void taskSpeedCalculatorFx(UArg arg1, UArg arg2);

void setPhase(uint8_t phase);
uint32_t dutyCycle(uint8_t percentage);
uint32_t dutyCycleForOLCtrl(int32_t speed);
void motorStatusLED(uint8_t motorEnabeled);

/****************************************************************************************************************************************************
 *      HWI
 ****************************************************************************************************************************************************/

/* Port 1 */
void hwiPort1Fx(UArg arg){

    /* Launchpad Button S1 */
    if(P1->IFG & PORT_BIT_1){
        GPIO_clearInt(Board_BUTTON_S1_GPIO);                // Clear interrupt flag
        Swi_post(swiMotorStop);                             // Stop motor
    }

    /* Launchpad Button S2 */
    if(P1->IFG & PORT_BIT_4){
        GPIO_clearInt(Board_BUTTON_S2_GPIO);                // Clear interrupt flag
        Swi_post(swiMotorToggleStatus);                     // Toggle motor status (enabled/disabled)
    }
}

/* Port 3 */
void hwiPort3Fx(UArg arg){

    /* MKII Button 2 */
    if(P3->IFG & PORT_BIT_5){
        GPIO_clearInt(MKII_BUTTON2_GPIO);                   // Clear interrupt flag
        Swi_post(swiMotorToggleStatus);                     // Toggle motor status (enabled/disabled)
    }
}

/* Port 5 */
void hwiPort5Fx(UArg arg){

    /* MKII Button 1 */
    if(P5->IFG & PORT_BIT_1){
        GPIO_clearInt(MKII_BUTTON1_GPIO);                       // Clear interrupt flag
        Swi_post(swiMotorStop);                                 // Stop the motor
    }
}

/* Timer */
void timerMotorControlOLFx(UArg arg){
    Event_post(eventPhaseChange, EVENT_CHANGE_PHASE);           // Change phase (next one)
}

/* Timer - Acceleration */
void timerMotorAccFx(UArg arg){
    Event_post(eventMotorControl, EVENT_MOTOR_MBX_SPEED);
}

/****************************************************************************************************************************************************
 *      SWI
 ****************************************************************************************************************************************************/
void swiMotorStopFx(UArg arg1, UArg arg2){

    /* Trigger Motor stop Events */
    Event_post(eventPhaseChange, EVENT_PHASE_STOP);             // Cut power to the motor
    Event_post(eventMotorControl, EVENT_MOTOR_STOP);            // Motor control at stop mode
}

void swiMotorToggleStatusFx(UArg arg1, UArg arg2){

    /* Toggle motor status */
    Event_post(eventMotorControl, EVENT_MOTOR_TOGGLE_STATUS);
}

/****************************************************************************************************************************************************
 *      TASK
 ****************************************************************************************************************************************************/
void taskMotorControlFx(UArg arg1, UArg arg2){

    /* Events */
    uint32_t events;

    /* Motor status */
    int32_t speed = 0;                                      // Motor stopped by default
    uint8_t ctrlType = MOTOR_CTR_OL;                        // Motor control Open Loop by default
    uint32_t timerPeriod = 0;                               // Timer period for OL control
    uint32_t dutyCycleRaw = 0;                              // Duty cycle for CL control
    uint8_t motorEnabeled = MOTOR_DISABLED;                 // Motor disabled by default
    motorStatusLED(motorEnabeled);                          // Turn on Red/Green LEDs depending on the motor status

    /* External control */
    int16_t joystick = 0;                                   // No actions by default
    Types_FreqHz freq;
    Timer_getFreq(timerMotorControlOL, &freq);              // Get OL motor control timer's frequency
    int32_t speedToTime = 60 * freq.lo / STEPS_PER_LAP;     // Convert RPM to timer frequency


    /* Main task loop */
    while(1){

        /* Wait for events */
        events = Event_pend(eventMotorControl, Event_Id_NONE, EVENT_MOTOR_MBX_SPEED | EVENT_MOTOR_TOGGLE_STATUS | EVENT_MOTOR_STOP, BIOS_WAIT_FOREVER);

        /* Get data */
        Mailbox_pend(mbxMotorSpeed, &joystick, BIOS_NO_WAIT);                       // Get joystick data

        /* When MOTOR STOP event is trigger no other event is executed */
        if((events & EVENT_MOTOR_STOP)){

            /* Stop motor */
            speed = dutyCycleRaw = timerPeriod = joystick = 0;                      // Reset control variables
            ctrlType = MOTOR_CTR_OL;                                                // Set motor control to OL
            Mailbox_post(mbxDutyCycle, &dutyCycleRaw, BIOS_NO_WAIT);                // Send duty cycle to the motor
            Timer_stop(timerMotorControlOL);                                        // Stop OpenLoop control timer

            // Todo: Remove
            System_flush();

        } else {

            if(events & EVENT_MOTOR_TOGGLE_STATUS){

                /* Toggle motor enabled status */
                motorEnabeled = !motorEnabeled;                                     // Toggle status
                motorStatusLED(motorEnabeled);                                      // Turn on Red/Green LEDs depending on the motor status

                /* Acceleration timer */
                if(motorEnabeled){
                    speed = 0;
                    Timer_start(timerMotorAcc);

                } else {
                    Timer_stop(timerMotorAcc);

                    // Todo: Remove -> Debug
                    MAP_ADC14_disableConversion();
                }

                if(!motorEnabeled){
                    Swi_post(swiMotorStop);                                         // If disable motor then stop the motor
                }


            } else if((events & EVENT_MOTOR_MBX_SPEED) & motorEnabeled){
                if(ctrlType == MOTOR_CTR_OL){

                    /* Open Loop control */
                    /*
                    speed += (int32_t) (joystick);///10);                               // Accelerate depending on the joysticks position
                    if(speed < 0){speed = 0;}                                           // Minimum speed -> 0 RPM
                    else if(speed > MOTOR_OL_MAX_SPEED){speed = MOTOR_OL_MAX_SPEED;}    // Maximum speed in OL control
                    */

                    /* Acceleration timer */
                    speed += 50;
                    if(speed > MOTOR_OL_MAX_SPEED){
                        speed = MOTOR_OL_MAX_SPEED;
                        Timer_stop(timerMotorAcc);

                        // Todo: Remove -> Debug
                        // Start sensing bemf
                        ADC14_configureMultiSequenceMode(ADC_MEM0, ADC_MEM2, true);
                        //MAP_ADC14_configureSingleSampleMode(ADC_MEM0, true);
                        MAP_ADC14_enableConversion();
                    }


                    /* Motor action */
                    if(!speed){
                        Swi_post(swiMotorStop);                                     // If speed is 0 then stop the motor
                    } else {

                        /* Convert to Timer period and Duty Cycle */
                        timerPeriod = speedToTime / speed;                          // Convert RPM to timer period
                        dutyCycleRaw = dutyCycleForOLCtrl(speed);                   // Get corresponding duty cycle for current speed (Look Up Table)

                        /* Set speed */
                        Mailbox_post(mbxDutyCycle, &dutyCycleRaw, BIOS_NO_WAIT);    // Send Duty Cycle to the actual motor drive task (taskPhaseChange)
                        Timer_setPeriod(timerMotorControlOL, timerPeriod);          // Set timer period (set motor speed)
                        Timer_start(timerMotorControlOL);                           // Restart timer (with setPeriod timer automatically stops)
                    }
                } else {

                    /* UNKNOWN MOTOR CONTROL TYPE */
                    /* Safe stop */
                    Swi_post(swiMotorStop);

                    /* Report error and halt */
                    System_printf("Unknown ctrlType on taskMotorControl. Event: %d \n", ctrlType);
                    System_flush();
                    while(1);
                }
            }
        }

        /* UNEXPECTED EVENT DETECTION */
        if(events & ~(EVENT_MOTOR_MBX_SPEED | EVENT_MOTOR_TOGGLE_STATUS | EVENT_MOTOR_STOP)){

            /* Safe stop */
            Event_post(eventPhaseChange, EVENT_PHASE_STOP);                         // Cut power to the motor

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
    PWM_params.periodValue = 30;                // 100us -> 10KHz
    PWM_params.dutyUnits = PWM_DUTY_FRACTION;    // Duty is in fractional percentage
    PWM_params.dutyValue = 0;                    // 0% initial duty cycle

    // Open the PWM instances
    PWM_A = PWM_open(PHASE_A_HIN, &PWM_params);
    PWM_B = PWM_open(PHASE_B_HIN, &PWM_params);
    PWM_C = PWM_open(PHASE_C_HIN, &PWM_params);
    BEMF_REF = PWM_open(BEMF_READ_REF, &PWM_params);

    // Check if PWM have been opened
    if (PWM_A == NULL || PWM_B == NULL || PWM_C == NULL || BEMF_REF == NULL) {
        System_printf("No s'ha pogut agafar el driver per els PWM dels motors \n");
        System_flush();
        while (1);
    }

    /* Duty cycle and phase */
    uint8_t phase = 0;
    uint32_t dutyCycleRaw = 0;

    /* Events */
    UInt events = 0;

    /* Motor initialization */
    /* Stop and cut power to the motor */
    setPhase(0);                                                                // phase = 0, cuts power
    PWM_setDuty(BEMF_REF, dutyCycle(50));                                       // Set BEMF Ref. duty at 50%
    PWM_start(BEMF_REF);                                                        // Start the signal. It never can be stopped. ADC is triggered with this signal

    /* MAIN TASK LOOP */
    while(1){

        /*
         * WAIT FOR EVENTS
         *  - EVENT_MOTOR_STOP:     Stop the motor
         *  - EVENT_CHANGE_PHASE:   Switches to next phase of the motor
         */
        events = Event_pend(eventPhaseChange, Event_Id_NONE, EVENT_PHASE_STOP | EVENT_CHANGE_PHASE, BIOS_WAIT_FOREVER);

        /* Get duty cycle */
        Mailbox_pend(mbxDutyCycle, &dutyCycleRaw, BIOS_NO_WAIT);

        /* EVENT ACTIONS */
        if(events & EVENT_PHASE_STOP){

            /* Reset state */
            phase = dutyCycleRaw = 0;                                               // Reset phase and speed calculating parameters
            setPhase(phase);                                                        // Stop motor
            confAdcBemf(phase);                                                     // Stop reading BEMF

            /* BEMF Control Development*/
            GPIO_write(BEMF_PHC_GPIO, 0);                                           // Phase check LOW when motor is stopped

        } else if (events & EVENT_CHANGE_PHASE){

            /* Phase change */
            phase++;
            if(phase > 6){

                /* Electric revolution completed */
                phase = 1;                                                          // Reset phase
                Event_post(eventSpeed, EVENT_ELECTRIC_REVOLUTION);                  // Motor speed calculation
            }

            /* Motor State */
            PWM_setDuty(PWM_A, dutyCycleRaw);
            PWM_setDuty(PWM_B, dutyCycleRaw);
            PWM_setDuty(PWM_C, dutyCycleRaw);
            setPhase(phase);

            /* BEMF reading */
            confAdcBemf(phase);                                                     // Configure ADC to read the proper phase

            /* BEMF Control Development*/
            GPIO_toggle(BEMF_PHC_GPIO);                                             // Phase check LOW when motor is stopped

        } else {

            /* Unknown event */
            setPhase(0);                                                                // Stop motor
            confAdcBemf(0);                                                             // Stop reading BEMF
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
    /* Speed measurement */
    uint32_t tstart, tstop = 0;                                     // Timestamps store variables
    uint8_t i = 0;
    uint32_t time;

    /* Speed conversion */
    int32_t speed = 0;                                              // Motor speed
    Types_FreqHz freq;
    Timestamp_getFreq(&freq);                                       // Get timestamp module freq.
    int32_t speedHumanize = 60 * freq.lo / ELECTRIC_REV_PER_LAP;    // Constant to convert from timestamp units to RMP
                                                                    // 60 is for converting sec. to min.
                                                                    // freq.lo is for converting timestamp counts to seconds
                                                                    // ELECTRIC_REV_PER_LAP are the number of electric revolutions to complete a lap in the motor

    while(1){

        events = Event_pend(eventSpeed, Event_Id_NONE, EVENT_SPEED_0 | EVENT_ELECTRIC_REVOLUTION, BIOS_WAIT_FOREVER);

        if(events & EVENT_SPEED_0){

            /* Send speed */
            speed = 0;                                                      // Motor has stoped
            time = 0;                                                       // Reset time accumulator
            Mailbox_post(mbxTheoricalSpeed, &speed, BIOS_WAIT_FOREVER);     // Wait until speed can be printed

        } else if(events & EVENT_ELECTRIC_REVOLUTION){

            /* Get time buffer from taskPhaseChange */
            //Mailbox_pend(mbxPhaseChangeTime, timeBuff, BIOS_NO_WAIT);

            /* Get Timestamp */
            tstop = Timestamp_get32();

            /* Add measurement */
            if(tstart){                                                             // If there's a previous electric revolution
                time += tstop - tstart;                                             // Accumulate time between current electric revolution and the previous one

                i++;                                                                // Increment buffer index
                i %= TIME_BUFF_LEN;                                                 // Keep index between 0 and TIME_BUFF_LEN

                if(!i){

                    /* Buffer is full */
                    time >>= TIME_AVG_SHIFT;                                        // Average calculation
                    speed = speedHumanize / time;                                   // Convert into RPM
                    Mailbox_post(mbxTheoricalSpeed, &speed, BIOS_NO_WAIT);          // Send data to LCD
                    time = 0;                                                       // Reset time accumulator
                }
            }

            /* Next measurement */
            tstart = tstop;                                                         // Start counting for next phase

        }
    }
}


/****************************************************************************************************************************************************
 *      FUNCTIONS
 ****************************************************************************************************************************************************/
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

            // Set speed to 0
            Event_post(eventSpeed, EVENT_SPEED_0);

            //Todo: Remove when release
//            System_printf("Phase: CUT CURRENT \n");
//            System_flush();

            break;
    }
}

uint32_t dutyCycle(uint8_t percentage){
    /* Duty Cycle calculator
     * In:  Duty cycle from 0 to 100 (percentage %)
     * Out: Duty value for the PWM driver
     */
    return (uint32_t) (((uint64_t) PWM_DUTY_FRACTION_MAX * percentage) / 100);
}

uint32_t dutyCycleForOLCtrl(int32_t speed){

    /* Open loop Duty Cycle calculator
     * Depending on the speed that the motor is turning in OL
     * demands a different Duty Cycle.
     * This function is a look up table with experimental values
     * that returns the optimal Duty Cycle value for the giving
     * timer period.
     */

    if(speed > 1800){  return dutyCycle(30);}
    else if(speed > 1400){  return dutyCycle(25);}
    else if(speed > 1100){  return dutyCycle(20);}
    else if(speed > 700){   return dutyCycle(15);}
    else {                  return dutyCycle(10);}
}

void motorStatusLED(uint8_t motorEnabeled){

    /* Launchpad LEDs */
    GPIO_write(Board_LED_Red_GPIO, !motorEnabeled);                     // Turn on Red LED when the motor is disabled
    GPIO_write(Board_LED_Green_GPIO, motorEnabeled);                    // Turn on Green LED when the motor is enabled

    /* MKII Boosterpack LEDs */
    GPIO_write(MKII_LED_Red_GPIO, !motorEnabeled);                      // Turn on Red LED when the motor is disabled
    GPIO_write(MKII_LED_Green_GPIO, motorEnabeled);                     // Turn on Green LED when the motor is enabled
}

