/*
 * Joystick.h
 *
 *  Created on: Jan 17, 2021
 *      Author: jango
 */

#ifndef LIBRARY_JOYSTICK_JOYSTICK_H_
#define LIBRARY_JOYSTICK_JOYSTICK_H_

/****************************************************************************************************************************************************
 *      DEFINITIONS
 ****************************************************************************************************************************************************/
#include <xdc/std.h>
typedef struct{
    int16_t min;
    int16_t max;
} Range;

typedef struct{
    int16_t x;
    int16_t y;
} Point;


/****************************************************************************************************************************************************
 *      CONSTANTS
 ****************************************************************************************************************************************************/
static const Range joystickNormalized = {100, -100};                // Joystick axis range normalized. The range is inverted to correct the MKII inversion

#define JOYSTICK_PAGE_EVENT_THRESHOLD                          90   // Joystick threshold value that trigger the change page event (next page / previous page)

#endif /* LIBRARY_JOYSTICK_JOYSTICK_H_ */
