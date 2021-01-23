/*
 * ADC.c
 *
 *  Created on: Jan 22, 2021
 *      Author: jango
 */

/****************************************************************************************************************************************************
 *      DEPENDENCIES
 ****************************************************************************************************************************************************/

/* HEADER FILE */
#include "ADC.h"

#include <xdc/std.h>
#include <stdio.h>
#include <xdc/runtime/System.h>
#include <xdc/runtime/Diags.h>
#include <xdc/runtime/Log.h>
#include <xdc/runtime/Timestamp.h>
#include <xdc/runtime/Types.h>
/* BIOS Header files */
#include <ti/sysbios/BIOS.h>
#include <ti/sysbios/hal/Hwi.h>
#include <ti/sysbios/knl/Swi.h>
#include <ti/sysbios/knl/Task.h>
#include <ti/sysbios/knl/Semaphore.h>
#include <ti/sysbios/knl/Queue.h>

/* Drivers header files */
#include "ti_drivers_config.h"
#include <ti/drivers/GPIO.h>
#include <ti/drivers/PWM.h>

/* Driverib */
#include <ti/devices/msp432p4xx/driverlib/driverlib.h>

/* Board header files */
#include <ti/drivers/Board.h>

/* User libraries */


/****************************************************************************************************************************************************
 *      MACROS
 ****************************************************************************************************************************************************/
/*els segUents defines son per configurar els pins de l'ADC dins de la
 * funcio ADCinit().
 */
#define PinConfigChannel(config) (((config) >> 10) & 0x1F)
#define PinConfigModuleFunction(config) (((config) >> 8) & 0x3)
#define PinConfigPort(config) (((config) >> 4) & 0xF)
#define PinConfigPin(config) (1 << ((config) & 0x7))

/* el seguent define es la mida del buffer de dades a la tasca productora.
 * Atencio: si s'augmenta molt, no s'ha d'oblidar ampliar la memoria de pila
 * d'aquesta tasca.
 */
#define DATA_MAX 8 //buffer adc

/****************************************************************************************************************************************************
 *      RTOS HANDLERS
 ****************************************************************************************************************************************************/
extern Swi_Handle swiADCResults;

extern Task_Handle taskADC;                 // la tasca productora de l'ADC
extern Task_Handle taskPlot;                // la tasca consumidora d'enviar dades a consola
extern Semaphore_Handle consumerStop;       // senyal per d'espera per enviar dades a consola
extern Semaphore_Handle producerStop;       // senyal per parar la producció de dades
extern Semaphore_Handle startConversion;    // senyal per fer la prendre dada de l'ADC1
       Queue_Handle     myQueue;            // gestor de la cua associada al buffer

/****************************************************************************************************************************************************
 *      FUNCTION DECLARATION
 ****************************************************************************************************************************************************/
/*declaracio de prototips de funcions*/
void swiADCResultsFx(UArg arg0, UArg arg1);

void hwiADCFx(UArg arg);                    // funcio HWI associada a l'ADC i trigger-Schmidt
void ADC_task_fxn(UArg arg0, UArg arg1);    // tasca productora
void plot_task_fxn(UArg arg0, UArg arg1);   // tasca consumidora:enviament a consola
void my_idle_fxn(void);                     // res a fer
bool ADCinit(void);                         // configuracio driverlib ADC

/****************************************************************************************************************************************************
 *      GLOBALS
 ****************************************************************************************************************************************************/
/* prototips de variables*/
typedef struct{
    Queue_Elem element;   // estructura de punters propia de la cua
    int16_t   adcValue;  // dada de l'ADC
    uint32_t   tstamp;    // temps de produccio de la dada de l'ADC

} Queue_data;

/* les seguents variables no son estrictament necessaries, pero ajuden a seguir
 * la implementacio del trigger Schmidt al HWI.
*/
volatile uint_fast64_t enableInts; // interrupcions habilitades de l'ADC en 'aquest moment'
volatile uint_fast64_t actualInts; // valor actual del registre de INTs de l'ADC

/****************************************************************************************************************************************************
 *      HWI
 ****************************************************************************************************************************************************/
void hwiADCFx(UArg arg){
    enableInts = MAP_ADC14_getEnabledInterruptStatus(); // capturem les int que estan habilitades
    actualInts = MAP_ADC14_getInterruptStatus();        // capturem les int que s'han disparat
    /*implementacio del trigger Schmidt: la finestra de comparacio esta associada al ADCMEM0 i
     * els valors de la finestra es troben definits a la ADCinit() */
    if(enableInts & actualInts & ADC_LO_INT){ // si s'ha disparat la INT de que el senyal d'entrada
                                              // esta per SOTA del valor de finestra de comparacio,
        MAP_ADC14_disableInterrupt(ADC_LO_INT);   // DESHABILITEM la INT de 'valor per sota' i
        MAP_ADC14_enableInterrupt(ADC_HI_INT);    // HABILITEM la INT de 'valor per sobre'.
        GPIO_write(Zero_Cross_Check_GPIO, 0);
        //GPIO_toggle(Zero_Cross_Check_GPIO);             // Assenyalem el pas pel valor baix en flanc de baixada
        // aquest conjunt d'intruccions anteriors es un trigger per flanc de baixada
   }
    if(enableInts & actualInts & ADC_HI_INT){ // si s'ha disparat la INT de que el senyal d'entrada
                                              // esta per SOBRE del valor de finestra de comparacio,
        MAP_ADC14_disableInterrupt(ADC_HI_INT);   // DESHABILITEM la INT de 'valor per sobre' i
        MAP_ADC14_enableInterrupt(ADC_LO_INT);    // HABILITEM la INT de 'valor per sota'.
        //GPIO_toggle(Zero_Cross_Check_GPIO);             // Assenyalem el pas pel valor baix en flanc de pujada
        GPIO_write(Zero_Cross_Check_GPIO, 1);
        // aquest conjunt d'intruccions anteriors es un trigger per flanc de pujada

    }

    if(enableInts & actualInts & ADC_INT0){
        Swi_post(swiADCResults);
    }

    /* NOTA sobre el trigger Schmidt: hem programat un cicle d'histeresi sobre les INT que estan
     * habilitades i deshabilitades.
     */
    MAP_ADC14_clearInterruptFlag(
            ADC_IN_INT | ADC_LO_INT | ADC_HI_INT | ADC_INT0 | ADC_INT1); // neteja flags
    //Semaphore_post(startConversion); // assemyalem al productor la disponibilitat de les dades ADC
}

void swiADCResultsFx(UArg arg0, UArg arg1){
    int16_t result = (int_fast16_t) ADC14_getResult(ADC_MEM0);
    System_printf("%8d\n", result);
}

/****************************************************************************************************************************************************
 *      FUNCTIONS
 ****************************************************************************************************************************************************/
/*
 * ADCinit() configura l'ADC fent servir les funcions de la driverlib.
 * Nota: per fer-ho 100% RTOS, s'hauria de tornar a reescriure la part de l'ADC de la llibreria 'drivers'.
 */
bool ADCinit(void)
{
    bool errorInit = true; // variable per portar control d'errors en la inicialitzacio

    MAP_ADC14_enableModule();        // posar en ON el modul ADC (alimentar-lo)
    MAP_ADC14_disableConversion();   // deshabilitar la conversio: necessari per fer modificar els registres de control de l'ADC

    //configuracio clock ADC: es de 25 MHz aprox
    errorInit = MAP_ADC14_initModule(ADC_CLOCKSOURCE_ADCOSC, //25 MHZ aprox
                                     ADC_PREDIVIDER_1,       // predivider a 1
                                     ADC_DIVIDER_1,          // divider a 1
                                     ADC_NOROUTE);           //no associem senyals interns
    if(!errorInit){return errorInit;} // check error

    // configuracio del tipus d'alimentacio per l'ADC
    errorInit = MAP_ADC14_setPowerMode(ADC_UNRESTRICTED_POWER_MODE); // full power
    if(!errorInit){return errorInit;} //check error

    // bits de resolucio de l'ADC
    MAP_ADC14_setResolution(ADC_14BIT); // no retorna res aquesta funcio

    // format de les dades de l'ADC
    errorInit = MAP_ADC14_setResultFormat(ADC_SIGNED_BINARY); // format sencer amb signe
    if(!errorInit){return errorInit;} //check error

    // duracio del sample and hold en unitats de clk
    errorInit = MAP_ADC14_setSampleHoldTime(ADC_PULSE_WIDTH_16,  // 16 cicles pels registres MEM0-7 i MEM24-31
                                            ADC_PULSE_WIDTH_16); // 16 cicles pels registres MEM8 a MEM23
    if(!errorInit){return errorInit;} //check error

    // senyal de trigger per realitzar la conversio a l'ADC
    errorInit = MAP_ADC14_setSampleHoldTrigger(ADC_TRIGGER_SOURCE3, //TA1_C1
                                               false);              //rising edge
    // la conversio s'associa al flanc de l'ADC (en mode Automatic, l'ADC inicia una nova conversio
    // just en acabar la conversio actual i sense esperar un nou trigger; es a dir, el trigger dispara
    // l'inici de la conversio automatica).

    errorInit = MAP_ADC14_enableSampleTimer(ADC_MANUAL_ITERATION/*ADC_AUTOMATIC_ITERATION*/);
    if(!errorInit){return errorInit;} //check error

    if(!errorInit){return errorInit;} //check error
    // limits baix i alt de la finestra de comparacio 0 (hi ha tambe una finestra 1)
    errorInit = MAP_ADC14_setComparatorWindowValue(ADC_COMP_WINDOW0,  //configurem el comparador 0
                                                   -256,              // valor baix finestra (int16)
                                                   255);              // valor alt finestra
    if(!errorInit){return errorInit;} //check error

    MAP_ADC14_disableInterrupt(0xFFFFFFFFFFFFFFFF);  // deshabilitem totes les interrupcions
    MAP_ADC14_enableInterrupt(ADC_HI_INT);           // habilitem la interrupcio que assenyala que
                                                     // el valor de l'ADC esta per sobre del
                                                     // valor alt de la finestra de comparacio

    //configuracio de l'ADC0
    errorInit = ADC14_configureSingleSampleMode(ADC_MEM0, true);
    errorInit = MAP_ADC14_configureConversionMemory(ADC_MEM0,                     // mapejat a MEM0
                                                    ADC_VREFPOS_AVCC_VREFNEG_VSS, // ref voltatge a 3.3V i -3.3V
                                                    ADC_INPUT_A20,                // P8.5 (PHASE_A BEMF)
                                                    ADC_DIFFERENTIAL_INPUTS);     // Diff amb P8.4 (ADC_INPUT_A21)
    if(!errorInit){return errorInit;} //check error

    // associem l'ADC0, mitjançant l'ADCMEM0, a la finestra de comparacio 0
    errorInit = MAP_ADC14_enableComparatorWindow(ADC_MEM0,          //associa la finestra de comparacio a MEM0
                                                 ADC_COMP_WINDOW0); //fem servir el comparador 0
    if(!errorInit){return errorInit;} //check error

    /*
    //configurem el GPIO 5.5 i 5.4 per a funcionalitat de l'ADC
    MAP_GPIO_setAsPeripheralModuleFunctionInputPin(      // configuracio del P5.5
        PinConfigPort(((0 << 10) | 0x0355)),
        PinConfigPin(((0 << 10) | 0x0355)),
        PinConfigModuleFunction(((0 << 10) | 0x0355)));


    MAP_GPIO_setAsPeripheralModuleFunctionInputPin(      // configuracio del P5.4
            PinConfigPort(((1 << 10) | 0x0354)),
            PinConfigPin(((1 << 10) | 0x0354)),
            PinConfigModuleFunction(((1 << 10) | 0x0354)));*/

    MAP_GPIO_setAsPeripheralModuleFunctionInputPin(GPIO_PORT_P8, GPIO_PIN5 | GPIO_PIN4, GPIO_TERTIARY_MODULE_FUNCTION);

    /*
    //Configuracio ADC11 cap a ADCMEM1 i single ended
    errorInit = MAP_ADC14_configureConversionMemory(ADC_MEM1,                     // registre 0
                                                    ADC_VREFPOS_AVCC_VREFNEG_VSS, // ref a 3.3V
                                                    ADC_INPUT_A11,                // P4.2
                                                    false);                       // single-ended
    if(!errorInit){return errorInit;} //check error

    //configurem el GPIO 4.2 per a funcionalitat de l'ADC
    MAP_GPIO_setAsPeripheralModuleFunctionInputPin(
            PinConfigPort(((11 << 10) | 0x0342)),
            PinConfigPin(((11 << 10) | 0x0342)),
            PinConfigModuleFunction(((11 << 10) | 0x0342)));

    // Mode de multisequencia: a cada trigger, es fa la conversio des del MEM0 fins al MEM1 en mode de repeticio
    errorInit = MAP_ADC14_configureMultiSequenceMode(ADC_MEM0, ADC_MEM1, true);
    if(!errorInit){return errorInit;} //check error
    */

    // les operacions de configuracio s'han acabat

    // neteja de flags d'interrupcio
    ADC14_clearInterruptFlag (ADC_INT0 | ADC_INT1 | ADC_HI_INT | ADC_LO_INT | ADC_IN_INT);//netejar flag
    // A partir d'aqui, s'ha d'iniciar la conversio; pero no la inciem fins que, a dins de la tasca
    // productora, s'hagi configurat i engegat el TA1_1

    // si fa falta, comprovem quines interrupcions tenim habilitades i quin es l'estat actual
    // de les flags d'interrupcio (posar aqui un breakpoint en debug)
    enableInts = ADC14_getEnabledInterruptStatus();
    actualInts = ADC14_getInterruptStatus();

    // Todo: Posar aixo a la inicialitzacio de la tasca del ADC
    /* Enabling Interrupts */
    MAP_Interrupt_enableInterrupt(INT_ADC14);
    MAP_Interrupt_enableMaster();
    /* s'habilita la conversio de l'ADC: a partir d'aquest moment, l'ADC agafa dades al ritme del PWM*/
    MAP_ADC14_enableConversion();

    return errorInit;
}

