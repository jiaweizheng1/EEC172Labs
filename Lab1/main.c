//*****************************************************************************
//
// Copyright (C) 2014 Texas Instruments Incorporated - http://www.ti.com/ 
// 
// 
//  Redistribution and use in source and binary forms, with or without 
//  modification, are permitted provided that the following conditions 
//  are met:
//
//    Redistributions of source code must retain the above copyright 
//    notice, this list of conditions and the following disclaimer.
//
//    Redistributions in binary form must reproduce the above copyright
//    notice, this list of conditions and the following disclaimer in the 
//    documentation and/or other materials provided with the   
//    distribution.
//
//    Neither the name of Texas Instruments Incorporated nor the names of
//    its contributors may be used to endorse or promote products derived
//    from this software without specific prior written permission.
//
//  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS 
//  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT 
//  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
//  A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT 
//  OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, 
//  SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT 
//  LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
//  DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
//  THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT 
//  (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE 
//  OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
//
//*****************************************************************************

//*****************************************************************************
//
// Application Name     - Blinky
// Application Overview - The objective of this application is to showcase the 
//                        GPIO control using Driverlib api calls. The LEDs 
//                        connected to the GPIOs on the LP are used to indicate 
//                        the GPIO output. The GPIOs are driven high-low 
//                        periodically in order to turn on-off the LEDs.
// Application Details  -
// http://processors.wiki.ti.com/index.php/CC32xx_Blinky_Application
// or
// docs\examples\CC32xx_Blinky_Application.pdf
//
//*****************************************************************************

//****************************************************************************
//
//! \addtogroup blinky
//! @{
//
//****************************************************************************

// Standard includes
#include <stdio.h>

// Driverlib includes
#include "hw_types.h"
#include "hw_ints.h"
#include "hw_memmap.h"
#include "hw_common_reg.h"
#include "interrupt.h"
#include "hw_apps_rcm.h"
#include "prcm.h"
#include "rom.h"
#include "rom_map.h"
#include "prcm.h"
#include "gpio.h"
#include "utils.h"

//new libraries to be included
#include "uart_if.h"
#include "stdint.h"

// Common interface includes
#include "gpio_if.h"

#include "pinmux.h"

#define APPLICATION_VERSION     "1.1.1"

//*****************************************************************************
//                 GLOBAL VARIABLES -- Start
//*****************************************************************************
#if defined(ccs)
extern void (* const g_pfnVectors[])(void);
#endif
#if defined(ewarm)
extern uVectorEntry __vector_table;
#endif
//*****************************************************************************
//                 GLOBAL VARIABLES -- End
//*****************************************************************************


//*****************************************************************************
//                      LOCAL FUNCTION PROTOTYPES                           
//*****************************************************************************
void LEDBlinkyRoutine();
static void BoardInit(void);

//*****************************************************************************
//                      LOCAL FUNCTION DEFINITIONS                         
//*****************************************************************************

//*****************************************************************************
//
//! Configures the pins as GPIOs and peroidically toggles the lines
//!
//! \param None
//! 
//! This function  
//!    1. Configures 3 lines connected to LEDs as GPIO
//!    2. Sets up the GPIO pins as output
//!    3. Periodically toggles each LED one by one by toggling the GPIO line
//!
//! \return None
//
//*****************************************************************************
void LEDBlinkyRoutine()
{
    int state = -1; //initially invalid; not in sw3 or sw2 mode.

    while(1)
    {
        int32_t sw3 = GPIOPinRead(GPIOA1_BASE, 0x20);   //poll sw3 and sw2.
        int32_t sw2 = GPIOPinRead(GPIOA2_BASE, 0x40);

        if(sw3 != 0) //sw3 high
        {
            if(state < 0 || state > 7)  //only print message once. condition: state is not currently sw3 mode.
            {
                Message("SW3 pressed\n\r");
                state = 0; //on entry to sw3 mode, current state should start from 0 then transition to 1,2,3,4,5,6,7,0,1,2,...
            }
            GPIOPinWrite(GPIOA3_BASE, 0x10, 0); //write low to PIN 18 when sw3 is high, independent of current state.

        }
        if(sw2 != 0) //sw2 high
        {
            if(state < 8)   //only print message once. condition: state is not currently in sw2 mode.
            {
                Message("SW2 pressed\n\r");
                state = 8;  //on entry to sw2 mode, current state should start from 8 the transition to 8,8,8,...
            }
            GPIOPinWrite(GPIOA3_BASE, 0x10, 0x10);  //write high to PIN 18 when sw2 is high, independent of current state.
        }

        if(state != -1) //dont display LEDs or do anything for when in invalid state
        {
            MAP_UtilsDelay(2000000);
            if(state == 1 || state == 3 || state == 5 || state == 7 || state == 8)//turn on the correct LEDs based on current state.
            {
                GPIO_IF_LedOn(MCU_RED_LED_GPIO);
            }
            if(state == 2 || state == 3 || state == 6 || state == 7 || state == 8)
            {
                GPIO_IF_LedOn(MCU_ORANGE_LED_GPIO);
            }
            if(state >= 4)
            {
                GPIO_IF_LedOn(MCU_GREEN_LED_GPIO);
            }
            MAP_UtilsDelay(2000000);    //turn off all the LEDs when done.
            GPIO_IF_LedOff(MCU_RED_LED_GPIO);
            GPIO_IF_LedOff(MCU_ORANGE_LED_GPIO);
            GPIO_IF_LedOff(MCU_GREEN_LED_GPIO);

            //update state for next iteration. See lab report for in depth analysis of why these two loops are written in this way.
            //basically, for sw3 mode we want to go through these stages in order: 0,1,2,3,4,5,6,7,0,1,2,3,...
            //and basically, for sw2 mode we want to go through these stages: 8,8,8,8,8,8,...
            if(state == 7)
            {
                state = 0;
            }
            else if(state < 8)
            {
                state++;
            }
        }
    }
}
//*****************************************************************************
//
//! Board Initialization & Configuration
//!
//! \param  None
//!
//! \return None
//
//*****************************************************************************
static void
BoardInit(void)
{
/* In case of TI-RTOS vector table is initialize by OS itself */
#ifndef USE_TIRTOS
    //
    // Set vector table base
    //
#if defined(ccs)
    MAP_IntVTableBaseSet((unsigned long)&g_pfnVectors[0]);
#endif
#if defined(ewarm)
    MAP_IntVTableBaseSet((unsigned long)&__vector_table);
#endif
#endif
    
    //
    // Enable Processor
    //
    MAP_IntMasterEnable();
    MAP_IntEnable(FAULT_SYSTICK);

    PRCMCC3200MCUInit();
}
//****************************************************************************
//
//! Main function
//!
//! \param none
//! 
//! This function  
//!    1. Invokes the LEDBlinkyTask
//!
//! \return None.
//
//****************************************************************************
int
main()
{
    //
    // Initialize Board configurations
    //
    BoardInit();
    
    //
    // Power on the corresponding GPIO port B for 9,10,11.
    // Set up the GPIO lines to mode 0 (GPIO)
    //
    PinMuxConfig();
    GPIO_IF_LedConfigure(LED1|LED2|LED3);

    GPIO_IF_LedOff(MCU_ALL_LED_IND);
    
    InitTerm(); //configure UART to be used
    ClearTerm();    //clears the terminal
    Message("\t\t*************************************************\n\r");   //print the usage instructions to the terminal
    Message("\t\t        CC3200 GPIO Application        \n\r");
    Message("\t\t*************************************************\n\r");
    Message("\n\n\r");
    Message("\t\t****************************************************\n\r");
    Message("\t\t        Push SW3 to start LED binary counting        \n\r");
    Message("\t\t        Push SW2 to blink LEDs on and off        \n\r");
    Message("\t\t****************************************************\n\r");

    //
    // Start the LEDBlinkyRoutine, which we modified to fix Part II criteria.
    //
    LEDBlinkyRoutine(); //infinite while loop for polling SW3 and SW2, and also for displaying the behaviors when SW3 or SW2 is pressed.
    return 0;
}

//*****************************************************************************
//
// Close the Doxygen group.
//! @}
//
//*****************************************************************************
