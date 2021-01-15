/* --COPYRIGHT--,BSD
 * Copyright (c) 2015, Texas Instruments Incorporated
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * *  Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 *
 * *  Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 *
 * *  Neither the name of Texas Instruments Incorporated nor the names of
 *    its contributors may be used to endorse or promote products derived
 *    from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR
 * PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR
 * CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,
 * EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
 * PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS;
 * OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
 * WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR
 * OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
 * EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 * --/COPYRIGHT--*/
//*****************************************************************************
//
// HAL_MSP_EXP432P401R_Crystalfontz128x128_ST7735.c -
//           Hardware abstraction layer for using the Educational Boosterpack's
//           Crystalfontz128x128 LCD with MSP-EXP432P401R LaunchPad
//
//*****************************************************************************

/*---------------------------*/
/* PREVIOUS TO SYSCONFIG */
//#include "../Board.h"

/* SYSCONFIG COMPATIBLE */
#include "ti_drivers_config.h"
/*---------------------------*/

#include "HAL_MSP_EXP432P401R_Crystalfontz128x128_ST7735_drivers.h"
#include <ti/grlib/grlib.h>
//#include <ti/devices/msp432p4xx/driverlib/driverlib.h>
#include <ti/drivers/GPIO.h>
#include <ti/drivers/SPI.h>
#include <stdint.h>

extern SPI_Handle spi;

void HAL_LCD_PortInit(void)
{
    // LCD_SCK
    // configured at spi_init
    //GPIO_setAsPeripheralModuleFunctionOutputPin(LCD_SCK_PORT, LCD_SCK_PIN, GPIO_PRIMARY_MODULE_FUNCTION);

    // LCD_MOSI
    // configured at spi_init
    //GPIO_setAsPeripheralModuleFunctionOutputPin(LCD_MOSI_PORT, LCD_MOSI_PIN, GPIO_PRIMARY_MODULE_FUNCTION);

    // LCD_CS
    // configured at spi_init
    //GPIO_setAsOutputPin(LCD_CS_PORT, LCD_CS_PIN);

    // LCD_RST
    //GPIO_setAsOutputPin(LCD_RST_PORT, LCD_RST_PIN);
    // LCD_RS
    //GPIO_setAsOutputPin(LCD_DC_PORT, LCD_DC_PIN);
    if ((GPIO_setConfig(LCD_RST, GPIO_CFG_OUT_STD | GPIO_CFG_OUT_LOW)  == GPIO_STATUS_ERROR) ||
        (GPIO_setConfig(LCD_DC,  GPIO_CFG_OUT_STD | GPIO_CFG_OUT_HIGH) == GPIO_STATUS_ERROR) ||
        (GPIO_setConfig(LCD_CS,  GPIO_CFG_OUT_STD | GPIO_CFG_OUT_LOW) == GPIO_STATUS_ERROR) )
    {
        //error while setting ports for LCD
        while(1);
    }
}

void HAL_LCD_SpiInit()
{
    SPI_Params      spiParams;
    SPI_init();  // Initialize the SPI driver
    SPI_Params_init(& spiParams);  // Initialize SPI parameters
    spiParams.transferMode = SPI_MODE_BLOCKING;   // this spi can only be used in task
    spiParams.transferTimeout = SPI_WAIT_FOREVER; // already set by SPI_Params_init
    spiParams.transferCallbackFxn = NULL;         // already set by SPI_Params_init
    spiParams.mode = SPI_MASTER;                  // already set by SPI_Params_init
    spiParams.bitRate = LCD_SPI_CLOCK_SPEED;      // according to crystalfontz
    spiParams.dataSize = 8;                       // 8-bit data size
    spiParams.frameFormat = SPI_POL0_PHA0;        // according to crystalfontz
    spi = SPI_open(LCD_EUSCI_BASE, & spiParams);     // open spi for LCD
    if (spi == NULL) {
        while (1);  // SPI_open() failed
    }

    //GPIO_setOutputLowOnPin(LCD_CS_PORT, LCD_CS_PIN); //ja l'hem iniciat a gpio_init

    //GPIO_setOutputHighOnPin(LCD_DC_PORT, LCD_DC_PIN); //ja l'hem iniciat a gpio_init
}


//*****************************************************************************
//
// Writes a command to the CFAF128128B-0145T.  This function implements the basic SPI
// interface to the LCD display.
//
//*****************************************************************************
void HAL_LCD_writeCommand(uint8_t command)
{
    SPI_Transaction spiTransaction;
    uint8_t bufferCommand[1];
    uint8_t receiveBuffer[1];
    bool    transferOK;

    // Fill in transmitBuffer
    spiTransaction.count = 1;
    bufferCommand[0] = command;
    spiTransaction.txBuf = (void *)bufferCommand;
    spiTransaction.rxBuf = (void *)receiveBuffer;
    // Set to command mode
    //GPIO_setOutputLowOnPin(LCD_DC_PORT, LCD_DC_PIN);
    GPIO_write(LCD_DC, GPIO_CFG_OUT_LOW);
    transferOK = SPI_transfer(spi, &spiTransaction);
    if (!transferOK) {
        // Error in SPI or transfer already in progress.
        while (1);
    }
    // Set back to data mode
    //GPIO_setOutputHighOnPin(LCD_DC_PORT, LCD_DC_PIN);
    GPIO_write(LCD_DC, GPIO_CFG_OUT_HIGH);
}


//*****************************************************************************
//
// Writes a data to the CFAF128128B-0145T.  This function implements the basic SPI
// interface to the LCD display.
//
//*****************************************************************************
void HAL_LCD_writeData(uint8_t data)
{
    SPI_Transaction spiTransaction;
    uint8_t bufferData[1];
    uint8_t receiveBuffer[1];
    bool    transferOK;

    // Fill in transmitBuffer
    spiTransaction.count = 1;
    bufferData[0] = data;
    spiTransaction.txBuf = (void *)bufferData;
    spiTransaction.rxBuf = (void *)receiveBuffer;
    // Send data
    transferOK = SPI_transfer(spi, &spiTransaction);
    if (!transferOK) {
        // Error in SPI or transfer already in progress.
        while (1);
    }
}

//*****************************************************************************
//
//! Provides a small delay.
//!
//! \param ui32Count is the number of delay loop iterations to perform.
//!
//! This function provides a means of generating a delay by executing a simple
//! 3 instruction cycle loop a given number of times.  It is written in
//! assembly to keep the loop instruction count consistent across tool chains.
//!
//! It is important to note that this function does NOT provide an accurate
//! timing mechanism.  Although the delay loop is 3 instruction cycles long,
//! the execution time of the loop will vary dramatically depending upon the
//! application's interrupt environment (the loop will be interrupted unless
//! run with interrupts disabled and this is generally an unwise thing to do)
//! and also the current system clock rate and flash timings (wait states and
//! the operation of the prefetch buffer affect the timing).
//!
//! For best accuracy, a system timer should be used with code either polling
//! for a particular timer value being exceeded or processing the timer
//! interrupt to determine when a particular time period has elapsed.
//!
//! \return None.
//
//*****************************************************************************
#if defined( __ICCARM__ ) || defined(DOXYGEN)
void
SysCtlDelay(uint32_t ui32Count)
{
    __asm("    subs    r0, #1\n"
          "    bne.n   SysCtlDelay\n"
          "    bx      lr");
}
#endif
#if defined(codered) || defined( __GNUC__ ) || defined(sourcerygxx)
void __attribute__((naked))
SysCtlDelay(uint32_t ui32Count)
{
    __asm("    subs    r0, #1\n"
          "    bne     SysCtlDelay\n"
          "    bx      lr");
}
#endif
#if defined(rvmdk) || defined( __CC_ARM )
__asm void
SysCtlDelay(uint32_t ui32Count)
{
    subs    r0, #1;
    bne     SysCtlDelay;
    bx      lr;
}
#endif
