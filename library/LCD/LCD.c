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
#define COLOR_TEXT_MUTED                            GRAPHICS_COLOR_WHITE_SMOKE  // Color used for secondary text

#define SPEED_LEN                                   7                           // Speed string length

/****************************************************************************************************************************************************
 *      DISPLAY HANDLERS
 ****************************************************************************************************************************************************/
Graphics_Context CTX;
Graphics_Rectangle header = {0,0, LCD_HORIZONTAL_MAX, 30};                  // Display header background
Graphics_Rectangle theoricalSpeedBg;                                        // Theorical speed magnitude background
uint32_t theoricalSpeedRightEdge;

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
void drawTheoricalSpeedCard();
void drawTheoricalSpeed();

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
   drawHeader((int8_t *)"BLDC Motor");
   drawTheoricalSpeedCard();

   /* Speed */
   uint32_t speed = 0;
   char speedStr[SPEED_LEN];

   while(1){
       Mailbox_pend(mbxTheoricalSpeed, &speed, BIOS_WAIT_FOREVER);
       sprintf(speedStr, "%7d", speed);
       drawTheoricalSpeed(speedStr);
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

void drawTheoricalSpeedCard(){

    /* Label and line */
    Graphics_setFont(CTXP, FONT_SMALL);
    Graphics_setForegroundColor(CTXP, COLOR_TEXT_MUTED);
    Graphics_drawString(CTXP, (int8_t *)"Theorical speed", AUTO_STRING_LENGTH, 5, 40, TRANSPARENT_TEXT);
    Graphics_drawLineH(CTXP, DISPLAY_LEFT_EDGE, DISPLAY_RIGHT_EDGE, 55);

    /* Speed units */
    uint32_t unitsX = DISPLAY_RIGHT_EDGE;
    unitsX -= Graphics_getStringWidth(CTXP, (int8_t *)"RPM", AUTO_STRING_LENGTH);
    Graphics_drawString(CTXP, (int8_t *)"RPM", AUTO_STRING_LENGTH, unitsX, 65, TRANSPARENT_TEXT);

    /* Speed magnitude */
    Graphics_setFont(CTXP, FONT_NORMAL);
    Graphics_setForegroundColor(CTXP, COLOR_TEXT);
    theoricalSpeedRightEdge = unitsX - DISPLAY_LEFT_EDGE;
    /*theoricalSpeedBg.xMax = theoricalSpeedRightEdge;
    theoricalSpeedBg.xMin = theoricalSpeedRightEdge - 50;
    theoricalSpeedBg.yMax = 70;
    theoricalSpeedBg.yMin = 60;*/






}

void drawTheoricalSpeed(int8_t *speed){

    /*
    Graphics_setForegroundColor(CTXP, COLOR_BACKGROUND);
    Graphics_drawRectangle(CTXP, &theoricalSpeedBg);
    Graphics_fillRectangle(CTXP, &theoricalSpeedBg);

    Graphics_setFont(CTXP, FONT_NORMAL);
    Graphics_setForegroundColor(CTXP, COLOR_TEXT);*/
    int32_t speedX = theoricalSpeedRightEdge;
    speedX -= Graphics_getStringWidth(CTXP, speed, AUTO_STRING_LENGTH);
    Graphics_drawString(CTXP, speed, AUTO_STRING_LENGTH, speedX, 65, OPAQUE_TEXT);
}

