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
#include "../utilities/utilities.h"
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
#define PAGE_MOTOR                                  0                           // Motor's page index
#define PAGE_JOYSTICK                               1                           // Joystick's page index
#define PAGE_COUNT                                  2                           // Number of defined pages
#define PAGE_DEFAULT                                PAGE_MOTOR                  // Page to be show after booting

/* Data values */
#define SPEED_LEN                                   7                           // Speed string length
#define JOYSTICK_LEN                                6                           // Joystick values string length

/****************************************************************************************************************************************************
 *      DISPLAY HANDLERS
 ****************************************************************************************************************************************************/

/* Definitions */
typedef struct {                                                            // Handle for Data Cards
    int32_t valueX;                                                         // Stores the X coord. where the value will be drawn
    int32_t valueY;                                                         // Stores the Y coord. where the value will be drawn
} DataCard_Handle;

typedef enum {                                                              // Data Cards types
    DATA_CARD_TYPE_NORMAL,                                                  // Card value displayed under the label
    DATA_CARD_TYPE_INLINE                                                   // Card value displayed next to the label
} DataCard_Type;

typedef struct {                                                            // Grlib doesn't have a circle shape defined
    int32_t x;                                                              // Here's a handler for a circle
    int32_t y;
    int32_t radius;
} Graphics_Circle;

/* Handlers */
Graphics_Context CTX;
Graphics_Rectangle header = {0,0, LCD_HORIZONTAL_MAX, 30};                  // Display header background
Graphics_Circle joystickGrPt;                                               // Circle for joystick's graphical representation
Graphics_Circle joystickGrBd;                                               // Border circle for joystick's graphical representation
Range joystickGrRangeX;                                                     // Range of movement in the x axis
Range joystickGrRangeY;                                                     // Range of movement in the y axis
DataCard_Handle theoricalSpeedHdl;                                          // Theorical speed card
DataCard_Handle joystickXHdl;                                               // Joystick X value card
DataCard_Handle joystickYHdl;                                               // Joystick Y value card


/****************************************************************************************************************************************************
 *      RTOS HANDLERS
 ****************************************************************************************************************************************************/
extern Task_Handle taskLcd;
extern Event_Handle eventLCD;
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
void pageMotorTemplate(void);
void pageJoystickTemplate(void);
void joystickGraphRefresh(Point *joystick);

/****************************************************************************************************************************************************
 *      TASK
 ****************************************************************************************************************************************************/
void taskLcdFx(UArg arg0, UArg arg1){

    /* LCD initialize */
    Crystalfontz128x128_Init();
    Crystalfontz128x128_SetOrientation(LCD_ORIENTATION_DOWN);                                   // Turn LCD, user will see it from below
    Graphics_initContext(CTXP, &g_sCrystalfontz128x128, &g_sCrystalfontz128x128_funcs);         // Initialize graphic context

   /* Page control and Events */
   int8_t page = PAGE_DEFAULT;                                                                  // Start with the default page
   uint32_t events;                                                                             // LCD refresh trigger events
   uint32_t eventOrMask;                                                                        // Each page depends on different events this mask will listen only to the current page

   /* Data values */
   //Bool hasMsg;                                                                                 // Check if the mailbox has message
   uint32_t theroricalSpeed = 0;                                                                // Speed calculated from the commands sent to the motor
   char theoricalSpeedStr[SPEED_LEN];                                                           // Speed as string

   Point joystick;                                                                              // Joystick position
   char joystickXStr[JOYSTICK_LEN];                                                             // X axis value as string
   char joystickYStr[JOYSTICK_LEN];                                                             // Y axis value as string

   /* Main Loop */
   while(1){

       /* Page reset */
      Graphics_setForegroundColor(CTXP, COLOR_TEXT);
      Graphics_setBackgroundColor(CTXP, COLOR_BACKGROUND);
      Graphics_clearDisplay(CTXP);

      /* Page initialization */
      switch (page) {
        case PAGE_MOTOR:
            pageMotorTemplate();                                                                        // Draw motor page
            eventOrMask = EVENT_THEORICAL_SPEED;                                                        // Subscribe to theorical speed change event
            break;

        case PAGE_JOYSTICK:
            pageJoystickTemplate();                                                                     // Draw joystick page
            eventOrMask = EVENT_JOYSTICK_DATA;                                                          // Subscribe to joystick data event
            break;

        default:
            System_printf("LCD unrecognized page\n");                                                   // Unrecognized page alert
            System_flush();
            break;
      }

      /* General events */
      eventOrMask |= EVENT_NEXT_PAGE | EVENT_PREVIOUS_PAGE;                                             // Page change event subscription

      /* Current page refresh */
      // Only refresh necessary data
      while(1){

          /* Wait only for events needed by the current page */
          events = Event_pend(eventLCD, Event_Id_NONE, eventOrMask, 100000);

          /* Page flow control */
          if(events & EVENT_NEXT_PAGE){
              page++;                                                                                           // Next page when Joystick is pushed to the right
              page %= PAGE_COUNT;                                                                               // Cycle trough pages
              break;                                                                                            // Break while loop (redraw entire page)

          } else if(events & EVENT_PREVIOUS_PAGE){
              page--;                                                                                           // Previous page when Joystick is pushed to the right
              if(page < 0){page = PAGE_COUNT-1;}                                                                // Cycle trough pages
              break;                                                                                            // Break while loop (redraw entire page)
          }

          Mailbox_pend(mbxTheoricalSpeed, &theroricalSpeed, BIOS_NO_WAIT);                         // Get theorical speed
          Mailbox_pend(mbxJoystick, &joystick, BIOS_NO_WAIT);                                     // Get joystick values

          /* Page refresh */
          switch (page) {
            case PAGE_MOTOR:

               if(events & EVENT_THEORICAL_SPEED){
                   sprintf(theoricalSpeedStr, "%7d", theroricalSpeed);                                      // Convert to string (right align the number and fill str. with trailing spaces)
                   drawDataCardValue(&theoricalSpeedHdl, (int8_t *) theoricalSpeedStr);                     // Refresh theorical speed value
               }
               break;

            case PAGE_JOYSTICK:

                if(events & EVENT_JOYSTICK_DATA){
                    sprintf(joystickXStr, "%4d", joystick.x);                                               // Convert to string (right align and fill with spaces)
                    sprintf(joystickYStr, "%4d", joystick.y);                                               // Convert to string (right align and fill with spaces)
                    drawDataCardValue(&joystickXHdl, (int8_t *) joystickXStr);                              // Refresh X axis value
                    drawDataCardValue(&joystickYHdl, (int8_t *) joystickYStr);                              // Refresh Y axis value
                    joystickGraphRefresh(&joystick);                                                        // Move joystick's graph inner circle
                }
                break;

            default:
                System_printf("LCD unrecognized page\n");
                System_flush();
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
    drawDataCardValue(&theoricalSpeedHdl, (int8_t *) "0");                                      // By default write 0 speed
}

void pageJoystickTemplate(){

    /* Header */
    drawHeader((int8_t *)"Joystick");                                                           // Header for the page

    /* Body */
    // X axis
    drawDataCard(                                                                               // Draw a boilerplate for joystick X value
                    &joystickXHdl,                                                              // Card handle
                    (int8_t *)"X coordinate",                                                   // Card label
                    (int8_t *)"",                                                               // Card units
                    35,                                                                         // Card y coord.
                    DATA_CARD_TYPE_INLINE                                                       // Data next to the label
                );

    // Y axis
    drawDataCard(                                                                               // Draw a boilerplate for joystick X value
                    &joystickYHdl,                                                              // Card handle
                    (int8_t *)"Y coordinate",                                                   // Card label
                    (int8_t *)"",                                                               // Card units
                    55,                                                                         // Card y coord.
                    DATA_CARD_TYPE_INLINE                                                       // Data next to the label
                );

    // Graphical joystick's representation
    joystickGrBd.x = LCD_HORIZONTAL_MAX >> 1;                                                   // Border circle center of the x axis
    joystickGrBd.y = 97;                                                                        // Border circle center y point
    joystickGrBd.radius = 20;                                                                   // Border circle radius

    joystickGrRangeX.max = joystickGrBd.x + joystickGrBd.radius;
    joystickGrRangeX.min = joystickGrBd.x - joystickGrBd.radius;
    joystickGrRangeY.max = joystickGrBd.y + joystickGrBd.radius;
    joystickGrRangeY.min = joystickGrBd.y - joystickGrBd.radius;

    Graphics_setForegroundColor(CTXP, GRAPHICS_COLOR_CADET_BLUE);
    Graphics_drawCircle(CTXP, joystickGrBd.x, joystickGrBd.y, joystickGrBd.radius);             // Draw border cirlce

    joystickGrPt.x = joystickGrBd.x;                                                            // Pointer circle (joystick's position)
    joystickGrPt.y = joystickGrBd.y;                                                            // Initial display at the center of the border circle
    joystickGrPt.radius = 5;
    Graphics_setForegroundColor(CTXP, GRAPHICS_COLOR_BLUE_VIOLET);
    Graphics_fillCircle(CTXP, joystickGrPt.x, joystickGrPt.y, joystickGrPt.radius);             // Draw inner circle
}

void joystickGraphRefresh(Point *joystick){

    //Graphics_Circle prevCircle = {joystickGrPt.x, joystickGrPt.y, joystickGrPt.radius};

    /* Delete previous frame circle */
    Graphics_setForegroundColor(CTXP, COLOR_BACKGROUND);
    Graphics_fillCircle(CTXP, joystickGrPt.x, joystickGrPt.y, joystickGrPt.radius);             // Erase previous circle

    /* Position calculation */
    joystickGrPt.x = map(joystick->x, (Range *) &joystickNormalizedNI, &joystickGrRangeX);      // Calculate X point
    joystickGrPt.y = map(joystick->y, (Range *) &joystickNormalized, &joystickGrRangeY);        // Calculate Y point

    /* Redraw border circle */
    Graphics_setForegroundColor(CTXP, GRAPHICS_COLOR_CADET_BLUE);
    Graphics_drawCircle(CTXP, joystickGrBd.x, joystickGrBd.y, joystickGrBd.radius);             // With the erasing some pixel may been damaged

    /* Draw new circle */
    Graphics_setForegroundColor(CTXP, GRAPHICS_COLOR_BLUE_VIOLET);
    Graphics_fillCircle(CTXP, joystickGrPt.x, joystickGrPt.y, joystickGrPt.radius);             // Draw new circle

}
