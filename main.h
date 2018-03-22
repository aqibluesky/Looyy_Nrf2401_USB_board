#ifndef _MAIN_H
#define _MAIN_H



//串口接受发送
uint8_t  EUSART_RX_BUF[10]={};
uint8_t  EUSART_TX_BUF[10]={};
uint8_t  EUSART_RB_BUF_TEMP[1]={};
//无线接受发送
uint8_t  NRF_RX_BUF[10]={};
uint8_t  NRF_TX_BUF[10]={};

uint8_t EUSART_RX_COUNT=0;

uint8_t EUSART_RX_FLAG=0;

uint8_t NRF_RX_FLAG=0;

//uint16_t RF_COUNT=0;

void NRF24L01_SendMSG(uint8_t *MSG);


#endif