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
#include <ti/drivers/ADCBuf.h>

/* Board header files */
#include <ti/drivers/Board.h>


/****************************************************************************************************************************************************
 *      MACROS
 ****************************************************************************************************************************************************/
#define SAMPLE_BUFF_SIZE                                    1   // Number of adc conversion per sample

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
    ADCBuf_Handle adcBuf;
    ADCBuf_Params params;
    int_fast16_t res;                                                   // ADC response for conversion (succeded or failed)
    ADCBuf_Params_init(&params);                                        // Default configuration: ONE_SHOT, MODE_BLOCKING ...


    /* Joystick data */
    uint16_t joystickX[SAMPLE_BUFF_SIZE];
    uint16_t joystickY[SAMPLE_BUFF_SIZE];

    /* Conversion configure */
    ADCBuf_Conversion joystickConv[2];                                  // Convert the 2 channels of the joystick

    joystickConv[0].arg = NULL;
    joystickConv[0].adcChannel = JOYSTICK_ADC0_X;
    joystickConv[0].sampleBuffer = joystickX;
    joystickConv[0].sampleBufferTwo = NULL;
    joystickConv[0].samplesRequestedCount = SAMPLE_BUFF_SIZE;

    joystickConv[1].arg = NULL;
    joystickConv[1].adcChannel = JOYSTICK_ADC0_Y;
    joystickConv[1].sampleBuffer = joystickY;
    joystickConv[1].sampleBufferTwo = NULL;
    joystickConv[1].samplesRequestedCount = SAMPLE_BUFF_SIZE;

    /* Main loop */
    while(1){

        /* Wait for semaphore */
        Semaphore_pend(semJoystickRead, BIOS_WAIT_FOREVER);

        /* Convert */
        adcBuf = ADCBuf_open(JOYSTICK_ADC0, &params);
        if (!adcBuf) {
            System_printf("Error initializing ADC0 for Joystick\n");
            System_flush();
            while (1);
        }

        res = ADCBuf_convert(adcBuf, joystickConv, 2);
        if (res == ADCBuf_STATUS_SUCCESS) {
            System_printf("Joystick X:%5d \t Y:%5d\n", joystickX[0], joystickY[0]);
        } else {
            System_printf("Joystick conversion failed\n");
        }

        ADCBuf_close(adcBuf);
        System_flush();
    }

}
