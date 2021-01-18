/*
 * utilities.h
 *
 *  Created on: Jan 18, 2021
 *      Author: jango
 */

#ifndef LIBRARY_UTILITIES_UTILITIES_H_
#define LIBRARY_UTILITIES_UTILITIES_H_

#include <xdc/std.h>

/****************************************************************************************************************************************************
 *      DEFINITIONS
 ****************************************************************************************************************************************************/
typedef struct{
    int16_t min;
    int16_t max;
} Range;

typedef struct{
    int16_t x;
    int16_t y;
} Point;

typedef struct{
    uint16_t min;
    uint16_t max;
} Range_unsigned;


/****************************************************************************************************************************************************
 *      FUNCTIONS
 ****************************************************************************************************************************************************/

/* MAP
 * Change the value acording to the input and output range.
 * Adapted from arduino's native map()
 */
int16_t map(int16_t x, Range *in, Range *out);


/* INACTIVE ZONE
 * If the input point is inside the zone detemined by rangeX and rangeY it will be
 * considered the refPoint. This function is useful for setting the joystick center
 * as a stable point.
 *
 * Range limits are not considered inside the discretization zone.
 */
void discretizePoint(Point *point, Range *rangeX, Range *rangeY, Point *refPoint);


#endif /* LIBRARY_UTILITIES_UTILITIES_H_ */
