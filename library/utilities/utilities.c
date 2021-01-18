/*
 * utilities.c
 *
 *  Created on: Jan 18, 2021
 *      Author: jango
 */

/* HEADER FILE */
#include "utilities.h"

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
