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


/****************************************************************************************************************************************************
 *      MACROS
 ****************************************************************************************************************************************************/
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

#define DATA_CARD_LINE_YOFFSET                      15                          // Height between y origin coordinate and line
#define DATA_CARD_UNITS_YOFFSET                     26                          // Height between y origin coordinate and units
#define DATA_CARD_VALUE_YOFFSET                     25                          // Height between y origin coordinate and value

#define SPEED_LEN                                   7                           // Speed string length

/****************************************************************************************************************************************************
 *      DISPLAY HANDLERS
 ****************************************************************************************************************************************************/

/* Definitions */
typedef struct {                                                            // Handle for Data Cards
    int32_t valueX;                                                         // Stores the X coord. where the value will be drawn
    int32_t valueY;                                                         // Stores the Y coord. where the value will be drawn
} DataCard_Handle;

/* Handlers */
Graphics_Context CTX;
Graphics_Rectangle header = {0,0, LCD_HORIZONTAL_MAX, 30};                  // Display header background
DataCard_Handle theoricalSpeed;                                             // Theorical speed card


/****************************************************************************************************************************************************
 *      RTOS HANDLERS
 ****************************************************************************************************************************************************/
extern Task_Handle taskLcd;
extern Mailbox_Handle mbxTheoricalSpeed;
/****************************************************************************************************************************************************
 *      DIVER HANDLERS
 ****************************************************************************************************************************************************/
SPI_Handle spi;

/****************************************************************************************************************************************************
 *      FUNCTION DECLARATION
 ****************************************************************************************************************************************************/
void taskLcdFx(UArg arg0, UArg arg1);
void drawHeader(int8_t *string);
void drawDataCard(DataCard_Handle *handle, int8_t *label, int8_t *units, int32_t y);
void drawDataCardValue(DataCard_Handle *handle, int8_t *data);

/****************************************************************************************************************************************************
 *      TASK
 ****************************************************************************************************************************************************/
void taskLcdFx(UArg arg0, UArg arg1){

    /* LCD initialize */
    Crystalfontz128x128_Init();
    Crystalfontz128x128_SetOrientation(LCD_ORIENTATION_DOWN);                                   // Turn LCD, user will see it from below

    /* Initializes graphics context */
   Graphics_initContext(CTXP, &g_sCrystalfontz128x128, &g_sCrystalfontz128x128_funcs);
   Graphics_setForegroundColor(CTXP, COLOR_TEXT);
   Graphics_setBackgroundColor(CTXP, COLOR_BACKGROUND);
   Graphics_clearDisplay(CTXP);

   /* Draw screen template */
   drawHeader((int8_t *)"BLDC Motor");                                                          // Header for the page
   drawDataCard(                                                                                // Draw a boilerplate for theroical speed value
                   &theoricalSpeed,                                                             // Card handle
                   (int8_t *)"Theorical speed",                                                 // Card label
                   (int8_t *)"RPM",                                                             // Card units
                   40                                                                           // Card y coord.
               );

   /* Theorical Speed */
   uint32_t theroricalSpeed = 0;                                                                // Speed calculated from the commands sent to the motor
   char theoricalSpeedStr[SPEED_LEN];                                                           // Speed as string

   while(1){
       Mailbox_pend(mbxTheoricalSpeed, &theroricalSpeed, BIOS_WAIT_FOREVER);                    // Wait and get theorical speed
       sprintf(theoricalSpeedStr, "%7d", theroricalSpeed);                                      // Convert to string (right align the number and fill str. with trailing spaces)
       drawDataCardValue(&theoricalSpeed, (int8_t *) theoricalSpeedStr);                        // Refresh theorical speed value
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

void drawDataCard(DataCard_Handle *handle, int8_t *label, int8_t *units, int32_t y){

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
    uint32_t unitsX = DISPLAY_RIGHT_EDGE;
    unitsX -= Graphics_getStringWidth(CTXP, units, AUTO_STRING_LENGTH);                                             // Right align text calculations
    Graphics_setForegroundColor(CTXP, COLOR_TEXT);                                                                  // Set font color
    Graphics_drawString(CTXP, units, AUTO_STRING_LENGTH, unitsX, y + DATA_CARD_UNITS_YOFFSET, TRANSPARENT_TEXT);    // Draw value units

    /* Data magnitude (value) */
    handle->valueX = unitsX - DISPLAY_LEFT_EDGE;                                                                    // Fill handle structure
    handle->valueY = y + DATA_CARD_VALUE_YOFFSET;
}

void drawDataCardValue(DataCard_Handle *handle, int8_t *data){

    /* Data magnitude (value) */
    Graphics_setFont(CTXP, FONT_NORMAL);                                                                            // Set font
    Graphics_setForegroundColor(CTXP, COLOR_TEXT);                                                                  // Set font color

    int32_t dataX = handle->valueX;
    dataX -= Graphics_getStringWidth(CTXP, data, AUTO_STRING_LENGTH);                                              // Right align text calculations
    Graphics_drawString(CTXP, data, AUTO_STRING_LENGTH, dataX, handle->valueY, OPAQUE_TEXT);                      // Draw value
}

