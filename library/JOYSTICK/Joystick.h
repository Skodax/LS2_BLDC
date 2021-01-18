/*
 * Joystick.h
 *
 *  Created on: Jan 17, 2021
 *      Author: jango
 */

#ifndef LIBRARY_JOYSTICK_JOYSTICK_H_
#define LIBRARY_JOYSTICK_JOYSTICK_H_

#include "../utilities/utilities.h"

/****************************************************************************************************************************************************
 *      MACROS
 ****************************************************************************************************************************************************/
#define JOYSTICK_PAGE_EVENT_THRESHOLD                          90   // Joystick threshold value that trigger the change page event (next page / previous page)


/****************************************************************************************************************************************************
 *      CONSTANTS
 ****************************************************************************************************************************************************/
static const Range joystickNormalized = {100, -100};                // Joystick axis range normalized. The range is inverted to correct the MKII inversion
static const Range joystickNormalizedNI = {-100, 100};              // Joystick axis range normalized not inverted.

#endif /* LIBRARY_JOYSTICK_JOYSTICK_H_ */
