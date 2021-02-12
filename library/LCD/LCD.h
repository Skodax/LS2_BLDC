/*
 * LCD.h
 *
 *  Created on: Jan 15, 2021
 *      Author: jango
 */

#ifndef LIBRARY_LCD_LCD_H_
#define LIBRARY_LCD_LCD_H_

#include <ti/sysbios/knl/Event.h>

/****************************************************************************************************************************************************
 *      MACROS
 ****************************************************************************************************************************************************/

/* Events */
#define EVENT_THEORICAL_SPEED                       Event_Id_00
#define EVENT_JOYSTICK_DATA                         Event_Id_02
#define EVENT_NEXT_PAGE                             Event_Id_04
#define EVENT_PREVIOUS_PAGE                         Event_Id_05
#define EVENT_MOTOR_STATUS                          Event_Id_06


#endif /* LIBRARY_LCD_LCD_H_ */
