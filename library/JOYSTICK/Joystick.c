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
extern Mailbox_Handle mbxJoystick;

/****************************************************************************************************************************************************
 *      JOYSTICK PARAMETERS
 ****************************************************************************************************************************************************/
typedef struct{
    uint16_t min;
    uint16_t max;
} Range_unsigned;

const Range_unsigned rangeX = {114, 16172};                 // Experimental edges of the X axis
const Range_unsigned rangeY = {0, 16380};                   // Experimental edges of the Y axis

//const Range joystickNormalized = {100, -100};               // Joystick axis range normalized. The range is inverted to correct the MKII inversion
const Range idleZone = {-10, 10};                           // Joystick zone that is considered idle (no user action)
const Point idlePoint = {0, 0};                             // Joystick point considered idle (each point in the idle zone will be converted to idlePoint)

/****************************************************************************************************************************************************
 *      FUNCTION DECLARATION
 ****************************************************************************************************************************************************/
void taskJoystickReadFx(UArg arg1, UArg arg2);
void clockJoystickReadFx(UArg arg0);
int16_t map(int16_t x, Range *in, Range *out);
void discretizePoint(Point *point, Range *rangeX, Range *rangeY, Point *refPoint);

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
    uint16_t joystickX[SAMPLE_BUFF_SIZE];                               // Samples buffer for X axis
    uint16_t joystickY[SAMPLE_BUFF_SIZE];                               // Samples buffer for Y axis
    Point joystick;

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

        /* Access ADC */
        adcBuf = ADCBuf_open(JOYSTICK_ADC0, &params);                                                       // Access to the ADC pheripheric
        if (adcBuf) {

            /* Take a sample */
            res = ADCBuf_convert(adcBuf, joystickConv, 2);                                                  // Trigger joystick conversion
            if (res == ADCBuf_STATUS_SUCCESS) {
                //System_printf("Joystick X:%5d \t Y:%5d", joystickX[0], joystickY[0]);

                /* Result normalization */
                joystick.x = map(joystickX[0], (Range *) &rangeX, (Range *) &joystickNormalized);           // Transform X results into comprehensible range
                joystick.y = map(joystickY[0], (Range *) &rangeY, (Range *) &joystickNormalized);           // Transform Y results into comprehensible range
                discretizePoint(&joystick, (Range *) &idleZone, (Range *) &idleZone, (Point *) &idlePoint); // Make a wide zone in the center of the joystick that will be considered as idelPoint

                /* Close ADC */
                ADCBuf_close(adcBuf);

                /* Send data */
                Mailbox_post(mbxJoystick, &joystick, BIOS_WAIT_FOREVER);
                //System_printf("\t[%4d,%4d]", joystick.x, joystick.y);


            } else {
                System_printf("Joystick conversion failed\n");
                System_flush();
                ADCBuf_close(adcBuf);
            }

            //System_printf("\n");
            //System_flush();


        } else {
            System_printf("Error initializing ADC0 for Joystick\n");
            System_flush();
            while (1);
        }

    }
}

/****************************************************************************************************************************************************
 *      FUNCTIONS
 ****************************************************************************************************************************************************/

/* MAP
 * Change the value acording to the input and output range.
 * Adapted from arduino's native map()
 */
int16_t map(int16_t x, Range *in, Range *out){
    return (x - in->min) * (out->max - out->min) / (in->max - in->min) + out->min;
}

/* INACTIVE ZONE
 * If the input point is inside the zone detemined by rangeX and rangeY it will be
 * considered the refPoint. This function is useful for setting the joystick center
 * as a stable point.
 *
 * Range limits are not considered inside the discretization zone.
 */
void discretizePoint(Point *point, Range *rangeX, Range *rangeY, Point *refPoint){

    /* Evaluate if the point is inside the X and Y range */
    if((point->x < rangeX->max) && (point->x > rangeX->min) && (point->y < rangeY->max) && (point->y >rangeY->min)){

        /* Discretize point to refPoint */
        point->x = refPoint->x;
        point->y = refPoint->y;
    }
}

/*
 * DEPRECATED CODE
 * Used for axis edges' determination
 *
 * @ Task initilization
    Range RangeX = {0, 16383};
    Range RangeY = {0, 16383};

   @ Task loop
    if(joystickX[0] > RangeX.max){RangeX.max = joystickX[0];}
    if(joystickX[0] < RangeX.min){RangeX.min = joystickX[0];}
    if(joystickY[0] > RangeY.max){RangeY.max = joystickY[0];}
    if(joystickY[0] < RangeY.min){RangeY.min = joystickY[0];}

    System_printf("\t X range [%5d,%5d]\t Y range [%5d,%5d]", RangeX.min, RangeX.max, RangeY.min, RangeY.max);
 *
 */
