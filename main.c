/**
  Generated Main Source File

  Company:
    Microchip Technology Inc.

  File Name:
    main.c

  Summary:
    This is the main file generated using MPLAB? Code Configurator

  Description:
    This header file provides implementations for driver APIs for all modules selected in the GUI.
    Generation Information :
        Product Revision  :  MPLAB? Code Configurator - v2.25.2
        Device            :  PIC16F1786
        Driver Version    :  2.00 
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
additional information regarding your rights and obligations.0

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

#include<math.h>
#include "mcc_generated_files/mcc.h"
#include"mcc_generated_files/NRF24L01.h"
#include"main.h"

uint32_t running=0;
uint8_t RF_State=0;

void main(void) 
{
    

  
    // initialize the device 
    SYSTEM_Initialize();//系统初始化
    
   
    // Enable the Global Interrupts
    INTERRUPT_GlobalInterruptEnable();

    // Enable the Peripheral Interrupts
    INTERRUPT_PeripheralInterruptEnable();

    INTERRUPT_IOInterruptEnable();
    
    INTERRUPT_BP4INIT();//下降沿触发
    
    // Disable the Global Interrupts
    //INTERRUPT_GlobalIn terruptDisable();

    // Disable the Peripheral Interrupts
    //INTERRUPT_PeripheralInterruptDisable();
 
    
    INTERRUPT_BP4REST();
    NRF24L01_Init();//24L01初始化
    INTERRUPT_BP4INIT();//下降沿触发
    
    while(NRF24L01_Check())
    {
        LED1_SetLow();
        LED2_SetLow();
        __delay_ms(1000);
        LED1_SetHigh();
        LED2_SetHigh();
        __delay_ms(1000);
    }    
    
    INTERRUPT_BP4REST();
    RX_Mode();//接收模式
    INTERRUPT_BP4INIT();//下降沿触发
    
    
    LED1_SetHigh();
    LED2_SetHigh();        
    
 
    while (1) 
    { 
        running++;
        
        if(running>0xffff)
        {
            running=0;
            if(EUSART_RX_COUNT!=0)
            {
                //EUSART_RX_COUNT=0;
            }    
            
        }    
        
        if(1==EUSART_RX_FLAG)//收到串口数据
        {
            LED1_SetLow();           
            __delay_ms(5);
            LED1_SetHigh(); // 红灯
            //EUSART_Write_MSG(EUSART_RX_BUF,10);
            //TX_ADDRESS[4]=EUSART_RX_BUF[0];
            NRF24L01_SendMSG(EUSART_RX_BUF);
            EUSART_RX_FLAG=0;
            for(uint8_t i=0;i<10;i++)
            {
                EUSART_RX_BUF[i]=0x00;
            }    
              
        } 
        
        if(1==NRF_RX_FLAG)      //无线接收到数据
        {
            /*
            EUSART_Write_MSG(RF_COUNT,1);
            if(RF_COUNT>0xfe)
            {
                RF_COUNT=0;
            } 
             */    
            EUSART_Write_MSG(NRF_RX_BUF,10);
            
            LED2_SetLow();
            __delay_ms(5);
             LED2_SetHigh(); // 红灯
            NRF_RX_FLAG=0;
            //EUSART_Write(0x01);
            
        }

    }
}




void NRF24L01_SendMSG(uint8_t *MSG)
{
    uint8_t state,i;
    
    INTERRUPT_BP4REST();//关闭中断  

    TX_Mode();//发送模式
  
    
    state=NRF24L01_TxPacket(MSG);  
    __delay_ms(1);
    
    RX_Mode();//接收模式
    
    INTERRUPT_BP4INIT();//打开中断
    //return state; 

}        


