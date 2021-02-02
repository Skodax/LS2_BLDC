/*
 * ADC.h
 *
 *  Created on: Jan 22, 2021
 *      Author: jango
 */

#ifndef LIBRARY_ADC_ADC_H_
#define LIBRARY_ADC_ADC_H_

#include "ti_drivers_config.h"
#include <ti/devices/msp432p4xx/driverlib/driverlib.h>

bool ADCinit(void);
void confAdcBemf(uint8_t phase);

/* Debug */
void phaseChangeLog(void);

#endif /* LIBRARY_ADC_ADC_H_ */
