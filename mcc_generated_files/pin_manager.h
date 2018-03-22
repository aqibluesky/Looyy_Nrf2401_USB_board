/**
  @Generated Pin Manager Header File

  @Company:
    Microchip Technology Inc.

  @File Name:
    pin_manager.h

  @Summary:
    This is the Pin Manager file generated using MPLAB? Code Configurator

  @Description:
    This header file provides implementations for pin APIs for all pins selected in the GUI.
    Generation Information :
        Product Revision  :  MPLAB? Code Configurator - v2.25.2
        Device            :  PIC16F1786
        Version           :  1.01
    The generated drivers are tested against the following:
        Compiler          :  XC8 v1.34
        MPLAB             :  MPLAB X v2.35 or v3.00
 */

/*
Copyright (c) 2013 - 2015 released Microchip Technology Inc.  All rights reserved.

Microchip licenses to you the right to use, modify, copy and distribute
Software only when embedded on a Microchip microcontroller or digital signal
controller that is integrated into your product or third party product
(pursuant to the sublicense terms in the accompanying license agreement).

You should refer to the license agreement accompanying this Software for
additional information regarding your rights and obligations.

SOFTWARE AND DOCUMENTATION ARE PROVIDED "AS IS" WITHOUT WARRANTY OF ANY KIND,
EITHER EXPRESS OR IMPLIED, INCLUDING WITHOUT LIMITATION, ANY WARRANTY OF
MERCHANTABILITY, TITLE, NON-INFRINGEMENT AND FITNESS FOR A PARTICULAR PURPOSE.
IN NO EVENT SHALL MICROCHIP OR ITS LICENSORS BE LIABLE OR OBLIGATED UNDER
CONTRACT, NEGLIGENCE, STRICT LIABILITY, CONTRIBUTION, BREACH OF WARRANTY, OR
OTHER LEGAL EQUITABLE THEORY ANY DIRECT OR INDIRECT DAMAGES OR EXPENSES
INCLUDING BUT NOT LIMITED TO ANY INCIDENTAL, SPECIAL, INDIRECT, PUNITIVE OR
CONSEQUENTIAL DAMAGES, LOST PROFITS OR LOST DATA, COST OF PROCUREMENT OF
SUBSTITUTE GOODS, TECHNOLOGY, SERVICES, OR ANY CLAIMS BY THIRD PARTIES
(INCLUDING BUT NOT LIMITED TO ANY DEFENSE THEREOF), OR OTHER SIMILAR COSTS.
 */

#ifndef PIN_MANAGER_H
#define PIN_MANAGER_H




/*LED 引脚定义*/
#define LED1_SetHigh()    do { LATC2 = 1; } while(0)
#define LED1_SetLow()   do { LATC2 = 0; } while(0)

#define LED2_SetHigh()    do { LATC3 = 1; } while(0)
#define LED2_SetLow()   do { LATC3 = 0; } while(0)

/*nRF24L01 引脚定义*/


#define WL_CE_SetHigh() do{LATC4 = 1;}while(0)
#define WL_CE_SetLow() do{LATC4 = 0;}while(0)

#define WL_CSN_SetHigh() do{LATC5 = 1;}while(0)
#define WL_CSN_SetLow() do{LATC5 = 0;}while(0)

#define WL_MISO_SetHigh() do{LATB5 = 1;}while(0)
#define WL_MISO_SetLow() do{LATB5 = 0;}while(0)
#define WL_MISO_GetValue()  RB5

#define WL_IRQ_SetHigh() do{LATB4 = 1;}while(0)
#define WL_IRQ_SetLow() do{LATB4 = 0;}while(0)
#define WL_IRQ_GetValue()  RB4

#define WL_SCK_SetHigh() do{LATB3 = 1;}while(0)
#define WL_SCK_SetLow() do{LATB3 = 0;}while(0)

#define WL_MOSI_SetHigh() do{LATB2 = 1;}while(0)
#define WL_MOSI_SetLow() do{LATB2 = 0;}while(0)

/**
 * @Param
    none
 * @Returns
    none
 * @Description
    GPIO and peripheral I/O initialization
 * @Example
    PIN_MANAGER_Initialize();
 */
void PIN_MANAGER_Initialize(void);

/**
 * @Param
    none
 * @Returns
    none
 * @Description
    Interrupt on Change Handling routine
 * @Example
    PIN_MANAGER_IOC();
 */
void PIN_MANAGER_IOC(void);

#endif // PIN_MANAGER_H
/**
 End of File
 */