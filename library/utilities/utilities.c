/*
 * utilities.c
 *
 *  Created on: Jan 18, 2021
 *      Author: jango
 */

/* HEADER FILE */
#include "utilities.h"

/* Drivers header files */
#include <ti/drivers/PWM.h>

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

/* PWM DUTY CYCLE CALCULATOR
 * In:  Duty cycle in percentage
 * Out: Actual duty cycle for the driver
 */
uint32_t dutyCycle(uint8_t percentage){
    /* Duty Cycle calculator
     * In:  Duty cycle from 0 to 100 (percentage %)
     * Out: Duty value for the PWM driver
     */
    return (uint32_t) (((uint64_t) PWM_DUTY_FRACTION_MAX * percentage) / 100);
}
