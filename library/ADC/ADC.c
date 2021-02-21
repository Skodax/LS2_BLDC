/*
 * ADC.c
 *
 *  Created on: Jan 22, 2021
 *      Author: jango
 */

/****************************************************************************************************************************************************
 *      DEPENDENCIES
 ****************************************************************************************************************************************************/

/* HEADER FILE */
#include "ADC.h"

#include <xdc/std.h>
#include <stdio.h>
#include <xdc/runtime/System.h>
#include <xdc/runtime/Diags.h>
#include <xdc/runtime/Log.h>
#include <xdc/runtime/Timestamp.h>
#include <xdc/runtime/Types.h>
/* BIOS Header files */
#include <ti/sysbios/BIOS.h>
#include <ti/sysbios/hal/Hwi.h>
#include <ti/sysbios/knl/Swi.h>
#include <ti/sysbios/knl/Task.h>
#include <ti/sysbios/knl/Clock.h>
#include <ti/sysbios/knl/Event.h>
#include <ti/sysbios/knl/Mailbox.h>

/* Drivers header files */
#include "ti_drivers_config.h"
#include <ti/drivers/GPIO.h>
#include <ti/drivers/PWM.h>

/* Driverib */
#include <ti/devices/msp432p4xx/driverlib/driverlib.h>

/* Board header files */
#include <ti/drivers/Board.h>

/* User libraries */
#include "../utilities/utilities.h"
#include "../BLDC/BLDC_driver.h"                            // Used for trigger the timer for phase change on BEMF zero cross on closed loop control
#include "../LCD/LCD.h"                                     // Used to change the page of the LCD with joystick actions

/****************************************************************************************************************************************************
 *      MACROS
 ****************************************************************************************************************************************************/

/* ADC */
#define ADC_MEM_PHASE_A                             ADC_MEM0            // Memory where Phase A BEMF will be stored
#define ADC_MEM_PHASE_B                             ADC_MEM1            // Memory where Phase B BEMF will be stored
#define ADC_MEM_PHASE_C                             ADC_MEM2            // Memory where Phase C BEMF will be stored
#define ADC_MEM_JOYSTICK_X                          ADC_MEM3            // Memory where Joystick X axis value will be stored
#define ADC_MEM_JOYSTICK_Y                          ADC_MEM4            // Memory where Joystick Y axis value will be stored

#define ADC_SEQ_ST_ADD                              ADC_MEM_PHASE_A     // Conversion sequence start address
#define ADC_SEQ_SP_ADD                              ADC_MEM_JOYSTICK_Y  // Conversion sequence stop address

#define ADC_WIN_TH                                  800                  // Comparation window threshold value -> Range = [-WIN_TH, WIN_TH]
#define ADC_ALL_INT                                 0xFFFFFFFFFFFFFFFF  // All interruptions mask

/* Events */
#define EVENT_JOYSTICK_READ                         Event_Id_00         // Retrieve joystick reading

/* Joystick Events */
#define JOYSTICK_EVENT_BLOCKED                              1   // Don't trigger joytick event
#define JOYSTICK_EVENT_UNBLOCKED                            0   // Trigger joystick event

/****************************************************************************************************************************************************
 *      JOYSTICK PARAMETERS
 ****************************************************************************************************************************************************/

const Range rangeX = {-31488, 31424};              // Experimental edges of the X axis
const Range rangeY = {-32760, 32664};              // Experimental edges of the Y axis

const Range idleZone = {-10, 10};                           // Joystick zone that is considered idle (no user action)
const Point idlePoint = {0, 0};                             // Joystick point considered idle (each point in the idle zone will be converted to idlePoint)

/****************************************************************************************************************************************************
 *      RTOS HANDLERS
 ****************************************************************************************************************************************************/

extern Swi_Handle swiBemfZeroCross;
extern Task_Handle taskADC;                 // la tasca productora de l'ADC
extern Event_Handle eventADC;

extern Clock_Handle clockJoystickRead;
extern Event_Handle eventLCD;
extern Mailbox_Handle mbxJoystick;
extern Mailbox_Handle mbxMotorSpeed;


/****************************************************************************************************************************************************
 *      FUNCTION DECLARATION
 ****************************************************************************************************************************************************/

void hwiADCFx(UArg arg);                    // Executed when ADC has a new reading or a window interruption
void clockJoystickReadFx(UArg arg0);        // Trigger joystick reading
void taskADCFx(UArg arg0, UArg arg1);       // Data depuration comeing from the ADC
bool ADCinit(void);                         // ADC configuration with driverlib
void joystickEventsChangePage(Point *joystick, uint8_t *eventPageBlocked);

/****************************************************************************************************************************************************
 *      GLOBALS
 ****************************************************************************************************************************************************/


/* les seguents variables no son estrictament necessaries, pero ajuden a seguir
 * la implementacio del trigger Schmidt al HWI.
*/
volatile uint_fast64_t enableInts; // interrupcions habilitades de l'ADC en 'aquest moment'
volatile uint_fast64_t actualInts; // valor actual del registre de INTs de l'ADC

/* Debug */
#define PHASE_A_BUFF_LEN        200
uint16_t i, current_phase = 0;
int16_t PhaseA_buff[PHASE_A_BUFF_LEN];
int16_t PhaseB_buff[PHASE_A_BUFF_LEN];
int16_t PhaseC_buff[PHASE_A_BUFF_LEN];
uint32_t PhaseA_time[PHASE_A_BUFF_LEN];

#define ZERO_CROSS_BUFF_LEN     50
#define ZERO_CROSS_UP           200
#define ZERO_CROSS_DOWN         -200
uint16_t j = 0;
int16_t ZeroCross_buff[ZERO_CROSS_BUFF_LEN];
uint32_t ZeroCross_time[ZERO_CROSS_BUFF_LEN];

#define EVENT_PHASE             Event_Id_01
#define EVENT_ZERO_CROSS_UP     Event_Id_02
#define EVENT_ZERO_CROSS_DOWN   Event_Id_03
#define EVENT_JOY_X             Event_Id_04
#define EVENT_JOY_Y             Event_Id_05

/****************************************************************************************************************************************************
 *      HWI
 ****************************************************************************************************************************************************/
void hwiADCFx(UArg arg){

    /* Get interrupt flags */
    uint_fast64_t status = MAP_ADC14_getInterruptStatus();
    uint_fast64_t enabled = MAP_ADC14_getEnabledInterruptStatus();

    /* Lower Window - Zero Cross from below */
    if(status & enabled & ADC_LO_INT){

        MAP_ADC14_disableInterrupt(ADC_LO_INT);                 // Disable this interruption, no further interruption until next phase
        Swi_post(swiBemfZeroCross);                             // Set and trigger the Closed Loop Control timer

        /* Debug */
        GPIO_write(BEMF_ZCD_GPIO, 0);
//        Event_post(eventADC, EVENT_ZERO_CROSS_DOWN);
    }

    /* Upper Window - Zero Cross from above */
    if(status  & enabled & ADC_HI_INT){

        MAP_ADC14_disableInterrupt(ADC_HI_INT);                 // Disable this interruption, no further interruption until next phase
        Swi_post(swiBemfZeroCross);                             // Set and trigger the Closed Loop Control timer

        /* Debug */
        GPIO_write(BEMF_ZCD_GPIO, 1);
//        Event_post(eventADC, EVENT_ZERO_CROSS_UP);
    }

    /* Debug */
//    if(status  & enabled & ADC_INT0){
//        Event_post(eventADC, EVENT_PHASE);
//    }

    /* Clear interrupt flags */
    MAP_ADC14_clearInterruptFlag(status & enabled);
}

/****************************************************************************************************************************************************
 *      SWI
 ****************************************************************************************************************************************************/
void clockJoystickReadFx(UArg arg0){
    Event_post(eventADC, EVENT_JOYSTICK_READ);
}

/****************************************************************************************************************************************************
 *      TASK
 ****************************************************************************************************************************************************/

void taskADCFx(UArg arg0, UArg arg1){
    /* Joystick data */
    Point joystick = {0, 0};
    Point prevJoytick = {0, 0};

    /* Debug */
    for(i = 0; i < PHASE_A_BUFF_LEN; i++){
        PhaseA_buff[i] = 0;
        PhaseB_buff[i] = 0;
        PhaseC_buff[i] = 0;
        PhaseA_time[i] = 0;
    }

    i = 0;

    for(j = 0; j < ZERO_CROSS_BUFF_LEN; j++){
        ZeroCross_buff[j] = 0;
        ZeroCross_time[j] = 0;
    }

    j = 0;

    /* Events */
    uint32_t events = 0;

    /* Joystick events parameters */
    uint8_t eventPageChangeBlocked = JOYSTICK_EVENT_BLOCKED;

    while(1){

        events = Event_pend(eventADC, Event_Id_NONE, EVENT_JOYSTICK_READ | EVENT_PHASE | EVENT_ZERO_CROSS_DOWN | EVENT_ZERO_CROSS_UP, BIOS_WAIT_FOREVER); // Todo: change to a semaphore get rid of the debug code

        if(events & EVENT_JOYSTICK_READ){

            /* Get conversion */
            joystick.x = ADC14_getResult(ADC_MEM_JOYSTICK_X);
            joystick.y = ADC14_getResult(ADC_MEM_JOYSTICK_Y);

            /* Result normalization */
            joystick.x = map(joystick.x, (Range *) &rangeX, (Range *) &joystickNormalized);           // Transform X results into comprehensible range
            joystick.y = map(joystick.y, (Range *) &rangeY, (Range *) &joystickNormalized);           // Transform Y results into comprehensible range
            discretizePoint(&joystick, (Range *) &idleZone, (Range *) &idleZone, (Point *) &idlePoint); // Make a wide zone in the center of the joystick that will be considered as idelPoint


            /* Joystick events */
            joystickEventsChangePage(&joystick, &eventPageChangeBlocked);                               // Event trigger for LCD page change

            /* Send data */
            if((prevJoytick.x != joystick.x) || (prevJoytick.y != joystick.y)){
                Mailbox_post(mbxJoystick, &joystick, BIOS_NO_WAIT);                                     // If there's new data then send it to the LCD
                prevJoytick.x = joystick.x;
                prevJoytick.y = joystick.y;
            }

            /* Motor Speed control */
            Mailbox_post(mbxMotorSpeed, &joystick.y, BIOS_NO_WAIT);                                     // Send the control speed to the motor


        }

        if(events & EVENT_PHASE){

            PhaseA_buff[i] = ADC14_getResult(ADC_MEM_PHASE_A);
            PhaseB_buff[i] = ADC14_getResult(ADC_MEM_PHASE_B);
            PhaseC_buff[i] = ADC14_getResult(ADC_MEM_PHASE_C);
            PhaseA_time[i] = Timestamp_get32();
            i++;
            if(i >= PHASE_A_BUFF_LEN){i = 0;}
        }

        if(events & EVENT_ZERO_CROSS_DOWN){
            ZeroCross_buff[j] = ZERO_CROSS_DOWN;
            ZeroCross_time[j] = Timestamp_get32();
            j++;
            if(j >= ZERO_CROSS_BUFF_LEN){j = 0;}
        }

        if(events & EVENT_ZERO_CROSS_UP){
            ZeroCross_buff[j] = ZERO_CROSS_UP;
            ZeroCross_time[j] = Timestamp_get32();
            j++;
            if(j >= ZERO_CROSS_BUFF_LEN){j = 0;}
        }
    }
}

/****************************************************************************************************************************************************
 *      FUNCTIONS
 ****************************************************************************************************************************************************/

// Todo: Remove error detection when the ADC is fully implemented
bool ADCinit(void)
{
    /* ADC initialization with DriverLib */
    bool errorInit = true;                                          // variable per portar control d'errors en la inicialitzacio

    MAP_ADC14_enableModule();                                       // Enable ADC - turn on ADC
    MAP_ADC14_disableConversion();                                  // No conversion - needed for configuration

    /* ADC clock */
    errorInit = MAP_ADC14_initModule(
                                        ADC_CLOCKSOURCE_ADCOSC,     // Internal oscillator - 25MHz
                                        ADC_PREDIVIDER_1,           // Don't divide the clock signal
                                        ADC_DIVIDER_1,              // Don't divide the clock signal
                                        ADC_NOROUTE                 // No internal readings (internal signals of the uC)
                                    );

    if(!errorInit){return errorInit;}                               // check error

    /* Power mode */
    errorInit = MAP_ADC14_setPowerMode(ADC_UNRESTRICTED_POWER_MODE);// Full power mode
    if(!errorInit){return errorInit;} //check error

    /* Resolution */
    MAP_ADC14_setResolution(ADC_14BIT);                             // 14 bits of resolution (maximum resolution)

    /* Data type */
    errorInit = MAP_ADC14_setResultFormat(ADC_SIGNED_BINARY);       // Use signed values
    if(!errorInit){return errorInit;}                               //check error

    /* Sample and Hold timing */
    errorInit = MAP_ADC14_setSampleHoldTime(
                                            ADC_PULSE_WIDTH_16,     // 16 cycles for MEM0-7 i MEM24-31
                                            ADC_PULSE_WIDTH_16      // 16 cycles for MEM8 a MEM23
                                            );

    if(!errorInit){return errorInit;}                               //check error

    /* ADC trigger */
    errorInit = MAP_ADC14_setSampleHoldTrigger(
                                                ADC_TRIGGER_SOURCE3,    // Use TA1_C1 (Reference signal - same timing of the PWM signals used for the motor)
                                                false                   // Trigger conversion at the rising edge of the signal
                                               );

    /* Cycles iteration */
    // Todo: try with automatic iteration
    errorInit = MAP_ADC14_enableSampleTimer(ADC_MANUAL_ITERATION);      // Manualy trigger the next sample
    if(!errorInit){return errorInit;} //check error

    /* Comparation Window */
    errorInit = MAP_ADC14_setComparatorWindowValue(
                                                    ADC_COMP_WINDOW0,   // Configure Comparation Window 0
                                                    ADC_WIN_TH,          // Lower edge of the window
                                                    -ADC_WIN_TH        // Upper edge of the window
                                                   );
    if(!errorInit){return errorInit;}                                   //check error

    MAP_ADC14_disableInterrupt(ADC_ALL_INT);                            // Disable all interrupts

    /* BEMF Phase A */
    MAP_GPIO_setAsPeripheralModuleFunctionInputPin(
                                                    GPIO_PORT_P8,
                                                    GPIO_PIN5 | GPIO_PIN4,          // Set Pin 8.5 and 8.4
                                                    GPIO_TERTIARY_MODULE_FUNCTION   // As an ADC input
                                                    );

    errorInit = MAP_ADC14_configureConversionMemory(
                                                    ADC_MEM_PHASE_A,                // ADC conversion result memory address
                                                    ADC_VREFPOS_AVCC_VREFNEG_VSS,   // Ref voltatge a 3.3V i -3.3V
                                                    ADC_INPUT_A20,                  // P8.5 (PHASE_A BEMF)
                                                    ADC_DIFFERENTIAL_INPUTS         // Diff amb P8.4 (ADC_INPUT_A21)
                                                    );

    /* BEMF Phase B */
    MAP_GPIO_setAsPeripheralModuleFunctionInputPin(
                                                    GPIO_PORT_P8,
                                                    GPIO_PIN7 | GPIO_PIN6,          // Set Pin 8.7 and 8.6
                                                    GPIO_TERTIARY_MODULE_FUNCTION   // As an ADC input
                                                    );

    errorInit = MAP_ADC14_configureConversionMemory(
                                                    ADC_MEM_PHASE_B,                // ADC conversion result memory address
                                                    ADC_VREFPOS_AVCC_VREFNEG_VSS,   // Ref voltatge a 3.3V i -3.3V
                                                    ADC_INPUT_A18,                  // P8.7 (PHASE_A BEMF)
                                                    ADC_DIFFERENTIAL_INPUTS         // Diff amb P8.6 (ADC_INPUT_A19)
                                                    );

    /* BEMF Phase C */
    MAP_GPIO_setAsPeripheralModuleFunctionInputPin(
                                                    GPIO_PORT_P9,
                                                    GPIO_PIN1 | GPIO_PIN0,          // Set Pin 9.1 and 9.0
                                                    GPIO_TERTIARY_MODULE_FUNCTION   // As an ADC input
                                                    );

    errorInit = MAP_ADC14_configureConversionMemory(
                                                    ADC_MEM_PHASE_C,                // ADC conversion result memory address
                                                    ADC_VREFPOS_AVCC_VREFNEG_VSS,   // Ref voltatge a 3.3V i -3.3V
                                                    ADC_INPUT_A16,                  // P9.1 (PHASE_C BEMF)
                                                    ADC_DIFFERENTIAL_INPUTS         // Diff amb P9.0 (ADC_INPUT_A17)
                                                    );

    /* Joystick X axis */
    MAP_GPIO_setAsPeripheralModuleFunctionInputPin(
                                                    GPIO_PORT_P6,
                                                    GPIO_PIN0,                      // Set Pin 6.0
                                                    GPIO_TERTIARY_MODULE_FUNCTION   // As an ADC input
                                                    );

    errorInit = MAP_ADC14_configureConversionMemory(
                                                    ADC_MEM_JOYSTICK_X,             // ADC conversion result memory address
                                                    ADC_VREFPOS_AVCC_VREFNEG_VSS,   // Ref voltatge a 3.3V i -3.3V
                                                    ADC_INPUT_A15,                  // P6.0 (Joystick X axis)
                                                    ADC_NONDIFFERENTIAL_INPUTS      // No diferential input
                                                    );

    /* Joystick X axis */
    MAP_GPIO_setAsPeripheralModuleFunctionInputPin(
                                                    GPIO_PORT_P4,
                                                    GPIO_PIN4,                      // Set Pin 6.0
                                                    GPIO_TERTIARY_MODULE_FUNCTION   // As an ADC input
                                                    );

    errorInit = MAP_ADC14_configureConversionMemory(
                                                    ADC_MEM_JOYSTICK_Y,             // ADC conversion result memory address
                                                    ADC_VREFPOS_AVCC_VREFNEG_VSS,   // Ref voltatge a 3.3V i -3.3V
                                                    ADC_INPUT_A9,                   // P4.4 (Joystick Y axis)
                                                    ADC_NONDIFFERENTIAL_INPUTS      // No diferential input
                                                    );


    if(!errorInit){return errorInit;}                                               //check error

    /* Interruptions */
    MAP_ADC14_clearInterruptFlag (ADC_ALL_INT);                                     // Clear all interruption flags

    /* Debug */
//    ADC14_enableInterrupt(ADC_INT0);
    ADC14_configureMultiSequenceMode(ADC_SEQ_ST_ADD, ADC_SEQ_SP_ADD, true);

    MAP_Interrupt_enableInterrupt(INT_ADC14);                                       // Enable ADC14 interruptions
    MAP_Interrupt_enableMaster();

    /* Enable conversions */
    MAP_ADC14_enableConversion();

    return errorInit;
}

/****************************************************************************************************************************************************
 *      FUNCTIONS
 ****************************************************************************************************************************************************/
void confAdcBemf(uint8_t phase){

    /* Configure the ADC module depending on the current phase.
     * Set the comparison window to the current BEMF phase. */

    /* Prepare ADC for configuration */
    MAP_ADC14_disableConversion();                                                      // To configure the ADC needs to be disabled
    MAP_ADC14_clearInterruptFlag(ADC_HI_INT | ADC_LO_INT);

    /* Phase configuration */
    switch (phase) {
        case 1:

            MAP_ADC14_enableComparatorWindow(ADC_MEM_PHASE_C, ADC_COMP_WINDOW0);        // Phase C has the BEMF signal
            MAP_ADC14_disableComparatorWindow(ADC_MEM_PHASE_A);                         // Disable other phases comparison window
            MAP_ADC14_disableComparatorWindow(ADC_MEM_PHASE_B);                         // Disable other phases comparison window

            MAP_ADC14_enableInterrupt(ADC_LO_INT);                                      // Interrupt when BEMF crosses 0 from above
            MAP_ADC14_disableInterrupt(ADC_HI_INT);                                     // Disable window upper site interrupt
            break;

        case 2:

            MAP_ADC14_enableComparatorWindow(ADC_MEM_PHASE_B, ADC_COMP_WINDOW0);        // Phase B has the BEMF signal
            MAP_ADC14_disableComparatorWindow(ADC_MEM_PHASE_A);                         // Disable other phases comparison window
            MAP_ADC14_disableComparatorWindow(ADC_MEM_PHASE_C);                         // Disable other phases comparison window

            MAP_ADC14_enableInterrupt(ADC_HI_INT);                                      // Interrupt when BEMF crosses 0 from below
            MAP_ADC14_disableInterrupt(ADC_LO_INT);                                     // Disable window lower site interrupt
            break;

        case 3:

            MAP_ADC14_enableComparatorWindow(ADC_MEM_PHASE_A, ADC_COMP_WINDOW0);        // Phase A has the BEMF signal
            MAP_ADC14_disableComparatorWindow(ADC_MEM_PHASE_B);                         // Disable other phases comparison window
            MAP_ADC14_disableComparatorWindow(ADC_MEM_PHASE_C);                         // Disable other phases comparison window

            MAP_ADC14_enableInterrupt(ADC_LO_INT);                                      // Interrupt when BEMF crosses 0 from above
            MAP_ADC14_disableInterrupt(ADC_HI_INT);                                     // Disable window upper site interrupt
            break;

        case 4:

            MAP_ADC14_enableComparatorWindow(ADC_MEM_PHASE_C, ADC_COMP_WINDOW0);        // Phase C has the BEMF signal
            MAP_ADC14_disableComparatorWindow(ADC_MEM_PHASE_A);                         // Disable other phases comparison window
            MAP_ADC14_disableComparatorWindow(ADC_MEM_PHASE_B);                         // Disable other phases comparison window

            MAP_ADC14_enableInterrupt(ADC_HI_INT);                                      // Interrupt when BEMF crosses 0 from below
            MAP_ADC14_disableInterrupt(ADC_LO_INT);                                     // Disable window lower site interrupt
            break;

        case 5:

            MAP_ADC14_enableComparatorWindow(ADC_MEM_PHASE_B, ADC_COMP_WINDOW0);        // Phase B has the BEMF signal
            MAP_ADC14_disableComparatorWindow(ADC_MEM_PHASE_A);                         // Disable other phases comparison window
            MAP_ADC14_disableComparatorWindow(ADC_MEM_PHASE_C);                         // Disable other phases comparison window

            MAP_ADC14_enableInterrupt(ADC_LO_INT);                                      // Interrupt when BEMF crosses 0 from above
            MAP_ADC14_disableInterrupt(ADC_HI_INT);                                     // Disable window upper site interrupt
            break;

        case 6:

            MAP_ADC14_enableComparatorWindow(ADC_MEM_PHASE_A, ADC_COMP_WINDOW0);        // Phase  has the BEMF signal
            MAP_ADC14_disableComparatorWindow(ADC_MEM_PHASE_B);                         // Disable other phases comparison window
            MAP_ADC14_disableComparatorWindow(ADC_MEM_PHASE_C);                         // Disable other phases comparison window

            MAP_ADC14_enableInterrupt(ADC_HI_INT);                                      // Interrupt when BEMF crosses 0 from below
            MAP_ADC14_disableInterrupt(ADC_LO_INT);                                     // Disable window lower site interrupt
            break;

        default:

            MAP_ADC14_disableComparatorWindow(ADC_MEM_PHASE_A);                         // Disable comparison window on the phase signal
            MAP_ADC14_disableComparatorWindow(ADC_MEM_PHASE_B);                         // Disable comparison window on the phase signal
            MAP_ADC14_disableComparatorWindow(ADC_MEM_PHASE_C);                         // Disable comparison window on the phase signal
            MAP_ADC14_disableInterrupt(ADC_HI_INT | ADC_LO_INT);                        // On motor stop the comparison window is disabled
    }


    /* Prepare ADC for reading */
    MAP_ADC14_clearInterruptFlag(ADC_HI_INT | ADC_LO_INT);                              // Clear compare window interrupt flags
    MAP_ADC14_enableConversion();                                                       // Enable ADC conversion (triggered by Timer A1)


}

void joystickEventsChangePage(Point *joystick, uint8_t *eventPageBlocked){

    /* Trigger several events depending on the joystick's actions */
    /* LCD Page change */
    if(*eventPageBlocked){

        if((joystick->x > -JOYSTICK_PAGE_EVENT_THRESHOLD) && (joystick->x < JOYSTICK_PAGE_EVENT_THRESHOLD)){
            *eventPageBlocked = JOYSTICK_EVENT_UNBLOCKED;                       // Unblock event when the joystick's position out of the trigger zone
        }
    } else {

        if(joystick->x > JOYSTICK_PAGE_EVENT_THRESHOLD){

            Event_post(eventLCD, EVENT_NEXT_PAGE);                              // Trigger next page event
            *eventPageBlocked = JOYSTICK_EVENT_BLOCKED;                         // Block event until joystick get out of the trigger zone

        } else if(joystick->x < -JOYSTICK_PAGE_EVENT_THRESHOLD){

            Event_post(eventLCD, EVENT_PREVIOUS_PAGE);                          // Trigger previous page event
            *eventPageBlocked = JOYSTICK_EVENT_BLOCKED;                         // Block event until joystick get out of the trigger zone
        }
    }
}
