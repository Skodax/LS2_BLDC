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
#include "../images/images.h"
#include "../BLDC/BLDC_driver.h"

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

/* Footer parameters */
#define FOOTER_BULLET_Y                             123                         // Footer vertical position
#define FOOTER_BULLET_SEPARATION                    10                          // Separation between bullets (pages indicator)
#define FOOTER_BULLER_RADIUS                        2                           // Bullet radius

/* Data Cards parameters */
#define DATA_CARD_LINE_YOFFSET                      15                          // Height between y origin coordinate and line
#define DATA_CARD_VALUE_YOFFSET                     20                          // Height between y origin coordinate and value
#define DATA_CARD_UNITS_YOFFSET                     21                          // Height between y origin coordinate and units
#define DATA_CARD_VALUE_INLINE_YOFFSET              0                           // Height between y origin coordinate and value when is a inline card
#define DATA_CARD_UNITS_INLINE_YOFFSET              1                           // Height between y origin coordinate and units when is a inline card

/* Pages parameters */
#define PAGE_WELCOME                                0
#define PAGE_MOTOR                                  1                           // Motor's page index
#define PAGE_JOYSTICK                               2                           // Joystick's page index
#define PAGE_CONTROLS1                              3                           // Controls diagram page index
#define PAGE_CONTROLS2                              4                           // Controls detailed page index
#define PAGE_COUNT                                  5                           // Number of defined pages
#define PAGE_DEFAULT                                PAGE_WELCOME                // Page to be show after booting

/* Data values */
#define SPEED_LEN                                   7                           // Speed string length
#define JOYSTICK_LEN                                6                           // Joystick values string length

/* Motor status parameters */
#define MOTOR_CTRLTYPE_OL_STR                       "OL"                        // Sting for open loop control
#define MOTOR_CTRLTYPE_CL_STR                       "CL"                        // Sting for open loop control

/* Duty Cycle bar parameters */
#define DUTY_BAR_Y                                  95                          // Y corrdinate of the bar
#define DUTY_BAR_HEIGHT                             16                          // Height of the bar
#define DUTY_BAR_TEXT_X                             LCD_HORIZONTAL_MAX >> 1     // Center of the screen
#define DUTY_BAR_TEXT_Y                             DUTY_BAR_Y + 7              // Center of the screen
#define DUTY_BAR_DISABLED_STR                       "------"                      // Text when the motor is not enabled

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
DataCard_Handle motorCtrTypeHdl;                                            // Motor control type card

DataCard_Handle joystickXHdl;                                               // Joystick X value card
DataCard_Handle joystickYHdl;                                               // Joystick Y value card

Graphics_Rectangle dutyBarBorder = {                                        // Duty cycle bar border rectangle
                                        DISPLAY_LEFT_EDGE,
                                        DUTY_BAR_Y,
                                        DISPLAY_RIGHT_EDGE,
                                        DUTY_BAR_Y + DUTY_BAR_HEIGHT
                                    };

Range dutyBarLimits = {DISPLAY_LEFT_EDGE + 1, DISPLAY_RIGHT_EDGE -1};       // Duty cycle bar limits
Range dutyCycleLimits = {0, 100};                                           // Duty cycle limits

Graphics_Rectangle dutyBar = {                                              // Duty cycle bar border rectangle
                                DISPLAY_LEFT_EDGE + 1,                      // Initial position -> zero
                                DUTY_BAR_Y + 1,
                                DISPLAY_LEFT_EDGE + 1,
                                DUTY_BAR_Y + DUTY_BAR_HEIGHT - 1
                             };



/****************************************************************************************************************************************************
 *      RTOS HANDLERS
 ****************************************************************************************************************************************************/
extern Task_Handle taskLcd;
extern Event_Handle eventLCD;
extern Mailbox_Handle mbxTheoricalSpeed;
extern Mailbox_Handle mbxJoystick;
extern Mailbox_Handle mbxMotorStatus;

/****************************************************************************************************************************************************
 *      DIVER HANDLERS
 ****************************************************************************************************************************************************/
SPI_Handle spi;

/****************************************************************************************************************************************************
 *      FUNCTION DECLARATION
 ****************************************************************************************************************************************************/
void taskLcdFx(UArg arg0, UArg arg1);
void drawHeader(int8_t *string);
void drawFooter(uint8_t currentPage, uint8_t numberOfPages);
void drawDataCard(DataCard_Handle *handle, int8_t *label, int8_t *units, int32_t y, DataCard_Type type);
void drawDataCardValue(DataCard_Handle *handle, int8_t *data);
void pageWelcomeTemplate(void);
void pageMotorTemplate(Motor *motor);
void pageJoystickTemplate(void);
void pageControls1Template(void);
void pageControls2Template(void);
void joystickGraphRefresh(Point *joystick);
void drawDutyBar(Motor *motor);

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
   uint32_t theroricalSpeed = 0;                                                                // Speed calculated from the commands sent to the motor
   char theoricalSpeedStr[SPEED_LEN];                                                           // Speed as string
   Motor motor = {MOTOR_DISABLED, MOTOR_CTR_OL, 0};                                             // Motor status

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
        case PAGE_WELCOME:
            pageWelcomeTemplate();                                                                      // Draw welcome page
            eventOrMask = 0;                                                                            // Don't subscribe to any events (static page)
            break;

        case PAGE_MOTOR:
            pageMotorTemplate(&motor);                                                                  // Draw motor page
            eventOrMask = EVENT_THEORICAL_SPEED | EVENT_MOTOR_STATUS;                                   // Subscribe to theorical speed change event and motor status
            break;

        case PAGE_JOYSTICK:
            pageJoystickTemplate();                                                                     // Draw joystick page
            eventOrMask = EVENT_JOYSTICK_DATA;                                                          // Subscribe to joystick data event
            break;

        case PAGE_CONTROLS1:
            pageControls1Template();                                                                    // Draw controls diagram page
            eventOrMask = 0;                                                                            // Don't subscribe to any events (static page)
            break;

        case PAGE_CONTROLS2:
            pageControls2Template();                                                                    // Draw controls diagram page
            eventOrMask = 0;                                                                            // Don't subscribe to any events (static page)
            break;

        default:
            System_printf("LCD unrecognized page\n");                                                   // Unrecognized page alert
            System_flush();
            break;
      }

      /* Footer */
      drawFooter(page, PAGE_COUNT);                                                                     // Draw footer for the current page

      /* General events */
      eventOrMask |= EVENT_NEXT_PAGE | EVENT_PREVIOUS_PAGE;                                             // Page change event subscription

      /* Current page refresh */
      // Only refresh necessary data
      while(1){

          /* Wait only for events needed by the current page */
          events = Event_pend(eventLCD, Event_Id_NONE, eventOrMask, 100);                               // If Clock period is changed timeout need to be changed

          /* Page flow control */
          if(events & EVENT_NEXT_PAGE){
              page++;                                                                                   // Next page when Joystick is pushed to the right
              page %= PAGE_COUNT;                                                                       // Cycle trough pages
              break;                                                                                    // Break while loop (redraw entire page)

          } else if(events & EVENT_PREVIOUS_PAGE){
              page--;                                                                                   // Previous page when Joystick is pushed to the right
              if(page < 0){page = PAGE_COUNT-1;}                                                        // Cycle trough pages
              break;                                                                                    // Break while loop (redraw entire page)
          }

          /* Get data - empty mailbox */
          Mailbox_pend(mbxTheoricalSpeed, &theroricalSpeed, BIOS_NO_WAIT);                              // Get theorical speed
          Mailbox_pend(mbxMotorStatus, &motor, BIOS_NO_WAIT);                                           // Get motor status
          Mailbox_pend(mbxJoystick, &joystick, BIOS_NO_WAIT);                                           // Get joystick values

          /* Page refresh */
          switch (page) {
            case PAGE_MOTOR:

               if(events & EVENT_THEORICAL_SPEED){

                   /* Motor speed */
                   sprintf(theoricalSpeedStr, "%7d", theroricalSpeed);                                      // Convert to string (right align the number and fill str. with trailing spaces)
                   drawDataCardValue(&theoricalSpeedHdl, (int8_t *) theoricalSpeedStr);                     // Refresh theorical speed value

               }

               if(events & EVENT_MOTOR_STATUS){

                   /* Motor control type */
                   if(motor.ctrlType == MOTOR_CTR_OL){
                       drawDataCardValue(&motorCtrTypeHdl, (int8_t *) MOTOR_CTRLTYPE_OL_STR);
                   } else {
                       drawDataCardValue(&motorCtrTypeHdl, (int8_t *) MOTOR_CTRLTYPE_CL_STR);
                   }

                   /* Duty cycle bar */
                   drawDutyBar(&motor);                                                                     // Duty cycle bar
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

void drawFooter(uint8_t currentPage, uint8_t numberOfPages){

    /* Page bullets */
    // Draw a series of bullets that represents the different pages of the GUI
    // The current page will be represented as a filled bullet

    if(!numberOfPages){return;}                                                             // If the number of pages is 0 then don't draw footer
    int32_t bulletX = LCD_HORIZONTAL_MAX >> 1;                                              // Center footer in the x axis
    //float portion = (numberOfPages-1)/2.0;
    bulletX -= (numberOfPages-1)/2.0 * FOOTER_BULLET_SEPARATION;                                          // First bullet position

    Graphics_setForegroundColor(CTXP, COLOR_HEADER_BACKGROUND);                             // Bullet's color

    uint8_t i;
    for(i = 0; i < numberOfPages; i++){

        /* Draw bullet */
        if(i == currentPage){
            Graphics_fillCircle(CTXP, bulletX, FOOTER_BULLET_Y, FOOTER_BULLER_RADIUS);
        } else {
            Graphics_drawCircle(CTXP, bulletX, FOOTER_BULLET_Y, FOOTER_BULLER_RADIUS);
        }

        /* Next bullet position */
        bulletX += FOOTER_BULLET_SEPARATION;

    }


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

void pageWelcomeTemplate(void){
    /* HEADER */
    drawHeader((int8_t *)"Welcome");                                                            // Header for the page

    /* BODY */
    /* Text */
    Graphics_setFont(CTXP, FONT_SMALL);
    Graphics_setForegroundColor(CTXP, COLOR_TEXT);
    Graphics_drawString(
                        CTXP,
                        (int8_t *)"Brushless DC motor",
                        AUTO_STRING_LENGTH,
                        DISPLAY_LEFT_EDGE,
                        35,
                        TRANSPARENT_TEXT
                        );
    Graphics_drawString(
                        CTXP,
                        (int8_t *)"control application.",
                        AUTO_STRING_LENGTH,
                        DISPLAY_LEFT_EDGE,
                        45,
                        TRANSPARENT_TEXT
                        );
    Graphics_drawString(
                        CTXP,
                        (int8_t *)"Depending on the motor",
                        AUTO_STRING_LENGTH,
                        DISPLAY_LEFT_EDGE,
                        60,
                        TRANSPARENT_TEXT
                        );
    Graphics_drawString(
                        CTXP,
                        (int8_t *)"speed, the control type",
                        AUTO_STRING_LENGTH,
                        DISPLAY_LEFT_EDGE,
                        70,
                        TRANSPARENT_TEXT
                        );
    Graphics_drawString(
                        CTXP,
                        (int8_t *)"will switch automatically.",
                        AUTO_STRING_LENGTH,
                        DISPLAY_LEFT_EDGE,
                        80,
                        TRANSPARENT_TEXT
                        );
    Graphics_setForegroundColor(CTXP, COLOR_TEXT_MUTED);
    Graphics_drawString(
                        CTXP,
                        (int8_t *)"Use the joystick to slide",
                        AUTO_STRING_LENGTH,
                        DISPLAY_LEFT_EDGE,
                        95,
                        TRANSPARENT_TEXT
                        );
    Graphics_drawString(
                        CTXP,
                        (int8_t *)"though the pages.",
                        AUTO_STRING_LENGTH,
                        DISPLAY_LEFT_EDGE,
                        105,
                        TRANSPARENT_TEXT
                        );

}

void pageMotorTemplate(Motor *motor){
    /* HEADER */
    drawHeader((int8_t *)"BLDC Motor");                                                          // Header for the page

    /* BODY */
    /* Speed */
    drawDataCard(                                                                                // Draw a boilerplate for theroical speed value
                    &theoricalSpeedHdl,                                                          // Card handle
                    (int8_t *)"Speed",                                                           // Card label
                    (int8_t *)"RPM",                                                             // Card units
                    40,                                                                          // Card y coord.
                    DATA_CARD_TYPE_INLINE                                                        // Data inline with the label
                );
    drawDataCardValue(&theoricalSpeedHdl, (int8_t *) "0");                                      // By default write 0 speed

    /* Control Type */
    drawDataCard(                                                                               // Draw boilerplate for motor control type value
                    &motorCtrTypeHdl,                                                           // Card handle
                    (int8_t *)"Control type",                                                   // Card label
                    (int8_t *)"",                                                               // Card units
                    65,                                                                         // Card y corrd
                    DATA_CARD_TYPE_INLINE                                                       // Data displayed inline with the label
                 );
    drawDataCardValue(&motorCtrTypeHdl, (int8_t *) MOTOR_CTRLTYPE_OL_STR);

    /* Duty cycle and status */
    Graphics_setForegroundColor(CTXP, COLOR_TEXT_MUTED);
    Graphics_drawRectangle(CTXP, &dutyBarBorder);                                               // Draw duty bar border
    dutyBar.xMin = (uint8_t ) dutyBarLimits.min;                                                // Reset duty bar, paint it all
    dutyBar.xMax = dutyBar.xMin;
    drawDutyBar(motor);
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

void pageControls1Template(void){
    /* HEADER */
    drawHeader((int8_t *)"Controls 1");                                                            // Header for the page

    /* BODY */
    /* Image */
    Graphics_drawImage(CTXP, &ControlDiagram4BPP_UNCOMP, 0, 35);

    /* Text */
    Graphics_setFont(CTXP, FONT_SMALL);
    Graphics_setForegroundColor(CTXP, COLOR_TEXT_MUTED);
    Graphics_drawString(
                        CTXP,
                        (int8_t *)"More info on the next",
                        AUTO_STRING_LENGTH,
                        DISPLAY_LEFT_EDGE,
                        100,
                        TRANSPARENT_TEXT
                        );
    Graphics_drawString(
                        CTXP,
                        (int8_t *)"page",
                        AUTO_STRING_LENGTH,
                        DISPLAY_LEFT_EDGE,
                        110,
                        TRANSPARENT_TEXT
                        );

}

void pageControls2Template(void){
    /* HEADER */
    drawHeader((int8_t *)"Controls 2");                                                            // Header for the page

    /* BODY */
    /* Text */
    Graphics_setFont(CTXP, FONT_SMALL);
    Graphics_setForegroundColor(CTXP, COLOR_TEXT);
    Graphics_drawString(
                        CTXP,
                        (int8_t *)"1. Enable/disable motor",
                        AUTO_STRING_LENGTH,
                        DISPLAY_LEFT_EDGE,
                        40,
                        TRANSPARENT_TEXT
                        );
    Graphics_drawString(
                        CTXP,
                        (int8_t *)"2. Stop the motor",
                        AUTO_STRING_LENGTH,
                        DISPLAY_LEFT_EDGE,
                        55,
                        TRANSPARENT_TEXT
                        );
    Graphics_drawString(
                        CTXP,
                        (int8_t *)"3. Change motor speed",
                        AUTO_STRING_LENGTH,
                        DISPLAY_LEFT_EDGE,
                        70,
                        TRANSPARENT_TEXT
                        );
    Graphics_drawString(
                        CTXP,
                        (int8_t *)"4. Change display page",
                        AUTO_STRING_LENGTH,
                        DISPLAY_LEFT_EDGE,
                        85,
                        TRANSPARENT_TEXT
                        );
    Graphics_drawString(
                        CTXP,
                        (int8_t *)"5. Motor status indicator",
                        AUTO_STRING_LENGTH,
                        DISPLAY_LEFT_EDGE,
                        100,
                        TRANSPARENT_TEXT
                        );

}

/* Utilities */
void drawDutyBar(Motor *motor){
    Graphics_setFont(CTXP, FONT_SMALL);
    if(motor->enabled){

        /* Get percentage */
        uint8_t duty = dutyCycleInv(motor->dutyCycleRaw);
        uint8_t nextBarValue = (uint8_t) map((int16_t) duty, &dutyCycleLimits, &dutyBarLimits);         // Convert from duty cycle to bar x coordinate pixel

        /* Duty cycle bar
        * Draw or erase only the difference between the last frame and the new one
        */
        if(nextBarValue > dutyBar.xMax){

            /* Increase bar */
            dutyBar.xMax = nextBarValue;                                                                // Extend the rectangle to the new value (extend to the right)
            Graphics_setForegroundColor(CTXP, COLOR_TEXT);                                              // Draw bar portion (white color)
            Graphics_fillRectangle(CTXP, &dutyBar);                                                     // Actual bar draw
            dutyBar.xMin = dutyBar.xMax;                                                                // Prepare rectangle for next refresh

        } else if(nextBarValue < dutyBar.xMin) {

            /* Decrease bar */
            dutyBar.xMin = nextBarValue;                                                                // Extend the rectangle to the new value (extend to the left)
            Graphics_setForegroundColor(CTXP, COLOR_BACKGROUND);                                        // Erase bar portion (black color)
            Graphics_fillRectangle(CTXP, &dutyBar);                                                     // Actual bar draw
            dutyBar.xMax = dutyBar.xMin;                                                                // Prepare rectangle for next refresh
        }

        /* Percentage value */
        char dutyStr[6];                                                                                // Percentage string
        sprintf(dutyStr, "%3d%%", duty);                                                                // Convert to a percentage with '%' symbol
        Graphics_setForegroundColor(CTXP, COLOR_TEXT);
        Graphics_drawStringCentered(CTXP, (int8_t *) dutyStr, AUTO_STRING_LENGTH, DUTY_BAR_TEXT_X, DUTY_BAR_TEXT_Y, OPAQUE_TEXT);

    } else {

        /* Motor disabled text */
        Graphics_setForegroundColor(CTXP, COLOR_HEADER_BACKGROUND);
        Graphics_drawStringCentered(CTXP, (int8_t *) DUTY_BAR_DISABLED_STR, AUTO_STRING_LENGTH, DUTY_BAR_TEXT_X, DUTY_BAR_TEXT_Y, OPAQUE_TEXT);

        /* Erase duty bar */
        if(dutyBar.xMin != dutyBarLimits.min){
            dutyBar.xMin = (uint8_t ) dutyBarLimits.min;                                                // Decrease bar to the start point
            Graphics_setForegroundColor(CTXP, COLOR_BACKGROUND);                                        // Erase bar portion (black color)
            Graphics_fillRectangle(CTXP, &dutyBar);                                                     // Actual bar draw
            dutyBar.xMax = dutyBar.xMin;                                                                // Prepare rectangle for next refresh
        }
    }
}
