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
static const Range joystickNormalized = {100, -100};               // Joystick axis range normalized. The range is inverted to correct the MKII inversion

#endif /* LIBRARY_JOYSTICK_JOYSTICK_H_ */
