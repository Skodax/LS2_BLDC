/*
 * LCD.c
 *
 *  Created on: Jan 15, 2021
 *      Author: jango
 */


/****************************************************************************************************************************************************
 *      DEPENDENCIES
 ****************************************************************************************************************************************************/

/* HEADER FILE */
#include "LCD.h"

/* XDC Module Headers */
#include <xdc/std.h>
#include <xdc/runtime/System.h>
#include <xdc/runtime/Diags.h>
#include <xdc/runtime/Log.h>
#include <xdc/runtime/Timestamp.h>
#include <xdc/runtime/Types.h>

/* BIOS Module Headers */
#include <ti/sysbios/BIOS.h>

#include <ti/sysbios/hal/Timer.h>
#include <ti/sysbios/hal/Hwi.h>

#include <ti/sysbios/knl/Swi.h>
#include <ti/sysbios/knl/Task.h>
#include <ti/sysbios/knl/Idle.h>

#include <ti/sysbios/knl/Event.h>
#include <ti/sysbios/knl/Semaphore.h>
#include <ti/sysbios/knl/Mailbox.h>

/* Drivers header files */
#include "ti_drivers_config.h"
#include <ti/drivers/GPIO.h>
#include <ti/drivers/SPI.h>

/* LCD header files */
#include <ti/grlib/grlib.h>
#include "../LcdDriver/Crystalfontz128x128_ST7735.h"
#include <stdio.h>

/* Board header files */
#include <ti/drivers/Board.h>

/* User libraries */
#include "../JOYSTICK/Joystick.h"


/****************************************************************************************************************************************************
 *      MACROS
 ****************************************************************************************************************************************************/

/* LCD parameters */
#define CTX                                         g_sContext                  // Context
#define CTXP                                        &CTX                        // Context pointer

#define DISPLAY_LEFT_MARGIN                         5                           // Left margin
#define DISPLAY_LEFT_EDGE                           DISPLAY_LEFT_MARGIN         // Left edge
#define DISPLAY_RIGHT_MARGIN                        5                           // Right margin
#define DISPLAY_RIGHT_EDGE                          LCD_HORIZONTAL_MAX - DISPLAY_RIGHT_MARGIN

#define FONT_TITLE                                  &g_sFontCmss18b             // Font used for titles
#define FONT_NORMAL                                 &g_sFontCmss14              // Font used for the main text
#define FONT_SMALL                                  &g_sFontCmss12              // Font used for secondary text

#define COLOR_BACKGROUND                            GRAPHICS_COLOR_BLACK        // Color used for LCD background
#define COLOR_HEADER_BACKGROUND                     GRAPHICS_COLOR_RED          // Color used for Header background
#define COLOR_TEXT                                  GRAPHICS_COLOR_WHITE        // Color used for the main text
#define COLOR_TEXT_MUTED                            0x00CCCCCC                  // Color used for secondary text

/* Data Cards parameters */
#define DATA_CARD_LINE_YOFFSET                      15                          // Height between y origin coordinate and line
#define DATA_CARD_VALUE_YOFFSET                     20                          // Height between y origin coordinate and value
#define DATA_CARD_UNITS_YOFFSET                     21                          // Height between y origin coordinate and units
#define DATA_CARD_VALUE_INLINE_YOFFSET              0                           // Height between y origin coordinate and value when is a inline card
#define DATA_CARD_UNITS_INLINE_YOFFSET              1                           // Height between y origin coordinate and units when is a inline card

/* Pages parameters */
#define PAGE_MOTOR                                  0
#define PAGE_JOYSTICK                               1
#define PAGE_DEFAULT                                PAGE_JOYSTICK

/* Speed parameters */
#define SPEED_LEN                                   7                           // Speed string length

/* Joystick parameters */
#define JOYSTICK_LEN                                6

/****************************************************************************************************************************************************
 *      DISPLAY HANDLERS
 ****************************************************************************************************************************************************/

/* Definitions */
typedef struct {                                                            // Handle for Data Cards
    int32_t valueX;                                                         // Stores the X coord. where the value will be drawn
    int32_t valueY;                                                         // Stores the Y coord. where the value will be drawn
} DataCard_Handle;

typedef enum {
    DATA_CARD_TYPE_NORMAL,
    DATA_CARD_TYPE_INLINE
}DataCard_Type;

/* Handlers */
Graphics_Context CTX;
Graphics_Rectangle header = {0,0, LCD_HORIZONTAL_MAX, 30};                  // Display header background
DataCard_Handle theoricalSpeedHdl;                                          // Theorical speed card
DataCard_Handle joystickXHdl;                                               // Joystick X value card
DataCard_Handle joystickYHdl;                                               // Joystick Y value card


/****************************************************************************************************************************************************
 *      RTOS HANDLERS
 ****************************************************************************************************************************************************/
extern Task_Handle taskLcd;
extern Mailbox_Handle mbxTheoricalSpeed;
extern Mailbox_Handle mbxJoystick;
/****************************************************************************************************************************************************
 *      DIVER HANDLERS
 ****************************************************************************************************************************************************/
SPI_Handle spi;

/****************************************************************************************************************************************************
 *      FUNCTION DECLARATION
 ****************************************************************************************************************************************************/
void taskLcdFx(UArg arg0, UArg arg1);
void drawHeader(int8_t *string);
void drawDataCard(DataCard_Handle *handle, int8_t *label, int8_t *units, int32_t y, DataCard_Type type);
void drawDataCardValue(DataCard_Handle *handle, int8_t *data);
void pageMotorTemplate();
void pageJoystickTemplate();

/****************************************************************************************************************************************************
 *      TASK
 ****************************************************************************************************************************************************/
void taskLcdFx(UArg arg0, UArg arg1){

    /* LCD initialize */
    Crystalfontz128x128_Init();
    Crystalfontz128x128_SetOrientation(LCD_ORIENTATION_DOWN);                                   // Turn LCD, user will see it from below
    Graphics_initContext(CTXP, &g_sCrystalfontz128x128, &g_sCrystalfontz128x128_funcs);         // Initialize graphic context

   /* Page */
   uint8_t page = PAGE_DEFAULT;                                                                 // Start with the default page

   /* Theorical Speed */
   uint32_t theroricalSpeed = 0;                                                                // Speed calculated from the commands sent to the motor
   char theoricalSpeedStr[SPEED_LEN];                                                           // Speed as string

   /* Joystick values */
   Point joystick;
   char joystickXStr[JOYSTICK_LEN];
   char joystickYStr[JOYSTICK_LEN];

   while(1){

       /* Page reset */
      Graphics_setForegroundColor(CTXP, COLOR_TEXT);
      Graphics_setBackgroundColor(CTXP, COLOR_BACKGROUND);
      Graphics_clearDisplay(CTXP);

      /* Page initialization */
      switch (page) {
        case PAGE_MOTOR:
            pageMotorTemplate();
            break;

        case PAGE_JOYSTICK:
            pageJoystickTemplate();
            break;
        default:
            break;
      }

      /* Page loop */
      // Only refresh necessary data
      while(1){
          switch (page) {
            case PAGE_MOTOR:

               Mailbox_pend(mbxTheoricalSpeed, &theroricalSpeed, BIOS_WAIT_FOREVER);                    // Wait and get theorical speed
               sprintf(theoricalSpeedStr, "%7d", theroricalSpeed);                                      // Convert to string (right align the number and fill str. with trailing spaces)
               drawDataCardValue(&theoricalSpeedHdl, (int8_t *) theoricalSpeedStr);                     // Refresh theorical speed value
               break;

            case PAGE_JOYSTICK:

                Mailbox_pend(mbxJoystick, &joystick, BIOS_WAIT_FOREVER);
                sprintf(joystickXStr, "%4d", joystick.x);
                sprintf(joystickYStr, "%4d", joystick.y);
                drawDataCardValue(&joystickXHdl, (int8_t *) joystickXStr);
                drawDataCardValue(&joystickYHdl, (int8_t *) joystickYStr);
                break;

            default:
                break;
        }
      }

   }



}

/****************************************************************************************************************************************************
 *      FUNCTIONS
 ****************************************************************************************************************************************************/

void drawHeader(int8_t *string){

    /* Header background */
    Graphics_setForegroundColor(CTXP, COLOR_HEADER_BACKGROUND);
    Graphics_drawRectangle(CTXP, &header);
    Graphics_fillRectangle(CTXP, &header);

    /* Title */
    Graphics_setFont(CTXP, FONT_TITLE);
    Graphics_setForegroundColor(CTXP, COLOR_BACKGROUND);
    Graphics_drawStringCentered(CTXP,
                                    string,
                                    AUTO_STRING_LENGTH,
                                    64,
                                    15,
                                    TRANSPARENT_TEXT);
    Graphics_setForegroundColor(CTXP, COLOR_TEXT);
}

void drawDataCard(DataCard_Handle *handle, int8_t *label, int8_t *units, int32_t y, DataCard_Type type){

    /* DATA CARD
     *
     * Draws a boiler plate for value representation, draws label and units.
     * In the handle it stores the necessary data for later draw the data value.
     * */

    /* Label and line */
    Graphics_setFont(CTXP, FONT_SMALL);
    Graphics_setForegroundColor(CTXP, COLOR_TEXT_MUTED);
    Graphics_drawString(CTXP, label, AUTO_STRING_LENGTH, DISPLAY_LEFT_EDGE, y, TRANSPARENT_TEXT);                   // Draw card label
    Graphics_drawLineH(CTXP, DISPLAY_LEFT_EDGE, DISPLAY_RIGHT_EDGE, y + DATA_CARD_LINE_YOFFSET);                    // Draw a label underline

    /* Data units */
    int32_t unitsX = DISPLAY_RIGHT_EDGE;
    int32_t unitsY = y;
    if(type == DATA_CARD_TYPE_INLINE){
        unitsY += DATA_CARD_UNITS_INLINE_YOFFSET;
    } else {
        unitsY += DATA_CARD_UNITS_YOFFSET;
    }

    unitsX -= Graphics_getStringWidth(CTXP, units, AUTO_STRING_LENGTH);                                             // Right align text calculations
    Graphics_setForegroundColor(CTXP, COLOR_TEXT);                                                                  // Set font color
    Graphics_drawString(CTXP, units, AUTO_STRING_LENGTH, unitsX, unitsY, TRANSPARENT_TEXT);                         // Draw value units

    /* Data magnitude (value) */
    handle->valueX = unitsX - DISPLAY_LEFT_EDGE;                                                                    // Fill handle structure
    if(type == DATA_CARD_TYPE_INLINE){
        handle->valueY = y + DATA_CARD_VALUE_INLINE_YOFFSET;                                                        // Fill handle structure when card is inline type
    } else {
        handle->valueY = y + DATA_CARD_VALUE_YOFFSET;                                                               // Fill handle structure
    }
}

void drawDataCardValue(DataCard_Handle *handle, int8_t *data){

    /* Data magnitude (value) */
    Graphics_setFont(CTXP, FONT_NORMAL);                                                                            // Set font
    Graphics_setForegroundColor(CTXP, COLOR_TEXT);                                                                  // Set font color

    int32_t dataX = handle->valueX;
    dataX -= Graphics_getStringWidth(CTXP, data, AUTO_STRING_LENGTH);                                               // Right align text calculations
    Graphics_drawString(CTXP, data, AUTO_STRING_LENGTH, dataX, handle->valueY, OPAQUE_TEXT);                        // Draw value
}

/* PAGE TEMPLATES (initialization) */

void pageMotorTemplate(){
    /* Header */
    drawHeader((int8_t *)"BLDC Motor");                                                          // Header for the page

    /* Body */
    drawDataCard(                                                                                // Draw a boilerplate for theroical speed value
                    &theoricalSpeedHdl,                                                          // Card handle
                    (int8_t *)"Theorical speed",                                                 // Card label
                    (int8_t *)"RPM",                                                             // Card units
                    40,                                                                          // Card y coord.
                    DATA_CARD_TYPE_NORMAL                                                        // Data below the label
                );
}

void pageJoystickTemplate(){
    /* Header */
    drawHeader((int8_t *)"Joystick");                                                           // Header for the page

    /* Body */
    drawDataCard(                                                                               // Draw a boilerplate for joystick X value
                    &joystickXHdl,                                                              // Card handle
                    (int8_t *)"X coordinate",                                                   // Card label
                    (int8_t *)"",                                                               // Card units
                    35,                                                                         // Card y coord.
                    DATA_CARD_TYPE_INLINE                                                       // Data next to the label
                );

    drawDataCard(                                                                               // Draw a boilerplate for joystick X value
                    &joystickYHdl,                                                              // Card handle
                    (int8_t *)"Y coordinate",                                                   // Card label
                    (int8_t *)"",                                                               // Card units
                    55,                                                                         // Card y coord.
                    DATA_CARD_TYPE_INLINE                                                       // Data next to the label
                );
}

