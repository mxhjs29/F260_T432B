/**
  ******************************************************************************
  * Copyright (c) 2018,�����пƺƵ�Ƽ����޹�˾
  * All rights reserved.
  * �ļ����ƣ�UART.h
  * ժ    Ҫ��
  *
  * ��ǰ�汾��V1.0
  * ��    �ߣ������пƺƵ�Ƽ����޹�˾�з��� 
  * ������ڣ�    
  * �޸�˵����
  * 
  *
  * ��ʷ�汾�����¶�MSP432���첽�����շ�֧��
  *
  *
  *******************************************************************************/
#ifndef _UART_H
#define _UART_H
//�ⲿ�ļ�����
#include "include.h"

#define MAX_RECEIVE_CNT     150
#define MAX_TRANSMIT_CNT    50



typedef struct
{
    UART_HandleTypeDef moduleInstance;
    
    /*���Ͳ������ݱ���*/
    bool TxCommpleate;
    queue_t qTx;
    uint8_t TxCnt;
    uint8_t TxLength;
    uint8_t TxBuff[MAX_TRANSMIT_CNT];
    
    /*���ղ������ݱ���*/
    bool RxStart;
    void (*RxHandle)(uint8_t *Ptr, uint8_t Length);
    uint8_t RxTimeout;
    int16_t RxCnt;
    uint8_t RxBuff[MAX_RECEIVE_CNT];
    uint8_t RxHandleBuff[MAX_RECEIVE_CNT];
    int16_t last_rx_cnt;
    
    uint8_t offset;
    uint8_t above;
    
}Usart_t;

/*
 * UART1 SBUS
 * UART2 ANO_OF
*/
typedef enum
{
    UART_1,
    UART_2,
    UART_3,
    UART_4,
    UART_5,
    UART_6,
    
    Num_USART,
}emUSART_t;

extern Usart_t UsartGroup[Num_USART];

void USART_Init(void);
void USART_TX(Usart_t*, uint8_t* pTx, uint8_t len);
void PollingUSART(void);
void usart_rx_idle_handle(emUSART_t uart);
#endif

/******************* (C) ��Ȩ���� 2018 �����пƺƵ�Ƽ����޹�˾ *******************/

