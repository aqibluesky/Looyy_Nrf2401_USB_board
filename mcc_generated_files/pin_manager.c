/**
  Generated Pin Manager File

  Company:
    Microchip Technology Inc.

  File Name:
    pin_manager.c

  Summary:
    This is the Pin Manager file generated using MPLAB? Code Configurator

  Description:
    This header file provides implementations for pin APIs for all pins selected in the GUI.
    Generation Information :
        Product Revision  :  MPLAB? Code Configurator - v2.25.2
        Device            :  PIC16F1786
        Driver Version    :  1.02
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

#include <xc.h>
#include "pin_manager.h"

/* ���ų�ʼ��*/
void PIN_MANAGER_Initialize(void) {
    LATA = 0x00;        //PA����
    TRISA = 0x00;       //����״̬RA0~RA7    �����ݷ���0Ϊ�����1Ϊ���룬����������Ҫ���ó������״̬�� 
    ANSELA = 0xFF;      //ģ����ƼĴ������� 11111111 1Ϊ����ģ�⣬������֡�
    WPUA = 0x00;        //�������Ĵ������� 1ʹ��������0��ֹ����

    LATB = 0x00;        //PB����  
    TRISB = 0x30;       //
    ANSELB = 0x00;      //01011111
    WPUB = 0x00;
  
    LATC = 0x00;        //PC����
    TRISC = 0x80;        
    WPUC = 0x00;

    TRISE = 0x08;      //1000
    WPUE = 0x00;

    OPTION_REGbits.nWPUEN = 0x01; 

    APFCON1 = 0x20;     //�����������üĴ���
    APFCON2 = 0x00;

    //�������� �ó����ģʽ �͵�ƽ
    LATA0 = 0;
    LATA1 = 0;
    LATA2 = 0;
    LATA3 = 0;
    LATA4 = 0;
    LATA5 = 0;
    LATA6 = 0;
    LATA7 = 0;
    
    LATB0 = 0;
    LATB1 = 0;
    
    LATC0 = 0;
    LATC1 = 0;
  
     
    
}
/**
 End of File
 */