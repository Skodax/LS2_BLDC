/*
 * Joystick.c
 *
 *  Created on: Jan 17, 2021
 *      Author: jango
 */

/****************************************************************************************************************************************************
 *      DEPENDENCIES
 ****************************************************************************************************************************************************/

/* HEADER FILE */
#include "Joystick.h"

/* XDC Module Headers */
#include <xdc/std.h>
#include <xdc/runtime/System.h>
#include <xdc/runtime/Diags.h>
#include <xdc/runtime/Log.h>
#include <xdc/runtime/Timestamp.h>
#include <xdc/runtime/Types.h>

/* BIOS Module Headers */
#include <ti/sysbios/BIOS.h>

#include <ti/sysbios/knl/Clock.h>
#include <ti/sysbios/knl/Task.h>

#include <ti/sysbios/knl/Event.h>
#include <ti/sysbios/knl/Semaphore.h>
#include <ti/sysbios/knl/Mailbox.h>

/* Drivers header files */
#include "ti_drivers_config.h"
#include <ti/drivers/ADC.h>

/* Board header files */
#include <ti/drivers/Board.h>


/****************************************************************************************************************************************************
 *      MACROS
 ****************************************************************************************************************************************************/

/****************************************************************************************************************************************************
 *      RTOS HANDLERS
 ****************************************************************************************************************************************************/
extern Task_Handle taskJoystickRead;
extern Clock_Handle clockJoystickRead;
extern Semaphore_Handle semJoystickRead;

/****************************************************************************************************************************************************
 *      DIVER HANDLERS
 ****************************************************************************************************************************************************/


/****************************************************************************************************************************************************
 *      FUNCTION DECLARATION
 ****************************************************************************************************************************************************/
void taskJoystickReadFx(UArg arg1, UArg arg2);
void clockJoystickReadFx(UArg arg0);

/****************************************************************************************************************************************************
 *      SWI
 ****************************************************************************************************************************************************/
void clockJoystickReadFx(UArg arg0){
    Semaphore_post(semJoystickRead);
}

/****************************************************************************************************************************************************
 *      TASK
 ****************************************************************************************************************************************************/
void taskJoystickReadFx(UArg arg1, UArg arg2){

    /* ADC initialization */
    ADC_Handle adc;
    ADC_Params params;
    int_fast16_t res;
    ADC_Params_init(&params);

    /* Joystick data */
    uint16_t joystickX;
    uint16_t joystickY;

    /* Main loop */
    while(1){

        /* Wait for semaphore */
        Semaphore_pend(semJoystickRead, BIOS_WAIT_FOREVER);

        /* X axis */
        adc = ADC_open(JOYSTICK_ADC_X, &params);
        if (adc == NULL) {
            System_printf("Error initializing ADC0 for Joystick X axis\n");
            System_flush();
            while (1);
        }
        res = ADC_convert(adc, &joystickX);

        if (res == ADC_STATUS_SUCCESS) {
            System_printf("Joystick X: %7d", joystickX);
        } else {
            System_printf("Joystick X: FAILED ");
        }

        ADC_close(adc);

        /* Y axis */
        adc = ADC_open(JOYSTICK_ADC_Y, &params);
        if (adc == NULL) {
            System_printf("Error initializing ADC0 for Joystick Y axis\n");
            System_flush();
            while (1);
        }
        res = ADC_convert(adc, &joystickY);

        if (res == ADC_STATUS_SUCCESS) {
            System_printf(" Y: %7d", joystickY);
        } else {
            System_printf(" Y: FAILED ");
        }

        ADC_close(adc);

        /* Send data */
        System_printf("\n");
        System_flush();

    }

}
