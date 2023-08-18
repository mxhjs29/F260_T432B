/**
  ******************************************************************************
  * Copyright (c) 2018,北京中科浩电科技有限公司
  * All rights reserved.
  * 文件名称：UART.h
  * 摘    要：
  *
  * 当前版本：V1.0
  * 作    者：北京中科浩电科技有限公司研发部 
  * 完成日期：    
  * 修改说明：
  * 
  *
  * 历史版本：更新对MSP432的异步串口收发支持
  *
  *
  *******************************************************************************/
#ifndef _UART_H
#define _UART_H
//外部文件引用
#include "include.h"

#define MAX_RECEIVE_CNT     200
#define MAX_TRANSMIT_CNT    50



typedef struct
{
    UART_HandleTypeDef moduleInstance;
    
    /*发送部分数据变量*/
    bool TxCommpleate;
    queue_t qTx;
    uint8_t TxCnt;
    uint8_t TxLength;
    uint8_t TxBuff[MAX_TRANSMIT_CNT];
    
    /*接收部分数据变量*/
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

#pragma pack(1)
typedef struct
{
	uint8_t head;
	uint8_t id;
	uint16_t dist;
	uint16_t amp;
	uint16_t temp;
	uint8_t check_sum;
}TF_Luna_t;


#pragma pack()
typedef struct
{
	uint8_t head;
	float   x;
	float   y;
	float   z;
	float   roll;
	float   pitch;
	float   yaw;
	uint8_t end;
}position_t;


typedef struct
{
	uint8_t head;  //1
	long odom_x:24;  //3
	long odom_y:24;  //3
	long fire_x:24;  //3
	long fire_y:24;  //3
	uint8_t end;   //1
}position_send_t;
#pragma

union LongUint8_t
{
    long l:24;
    uint8_t u[3];
};

extern Usart_t UsartGroup[Num_USART];

void USART_Init(void);
void USART_TX(Usart_t*, uint8_t* pTx, uint8_t len);
void PollingUSART(void);
void usart_rx_idle_handle(emUSART_t uart);
#endif

/******************* (C) 版权所有 2018 北京中科浩电科技有限公司 *******************/

