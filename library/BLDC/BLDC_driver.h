/*
 * BLDC_driver.h
 *
 *  Created on: Jan 9, 2021
 *      Author: jango
 */

#ifndef LIBRARY_BLDC_DRIVER_H_
#define LIBRARY_BLDC_DRIVER_H_

#include <xdc/std.h>

/* Motor status */
#define MOTOR_ENABLED               1               // Motor enabled and ready to go
#define MOTOR_DISABLED              0
#define MOTOR_CTR_OL                1               // Motor control type -> Open loop
#define MOTOR_CTR_CL                0               // Motor control type -> Closed loop

typedef struct{
    uint8_t enabled;
    uint8_t ctrlType;
    uint32_t dutyCycleRaw;
} Motor;

uint32_t dutyCycle(uint8_t percentage);             // Returns duty fraction for PWM driver
uint8_t dutyCycleInv(uint32_t dutyRaw);             // Retruns duty in percentage

#endif /* LIBRARY_BLDC_DRIVER_H_ */
