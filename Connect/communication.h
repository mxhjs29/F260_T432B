/**
  ******************************************************************************
  * Copyright (c) 2018,北京中科浩电科技有限公司
  * All rights reserved.
  * 文件名称：communication.h
  * 摘    要：
  *
  * 当前版本：V1.0
  * 作    者：北京中科浩电科技有限公司研发部 
  * 完成日期：    
  * 修改说明：
  * 
  *
  * 历史版本：
  *
  *
  *******************************************************************************/
#ifndef _COMMUNICATION_H
#define _COMMUNICATION_H
//外部文件引用
#include "include.h"
#include "gcs.h"
#include "nrf24l01.h"


//宏定义区



//数据结构声明
typedef enum
{
    Rx_Mode = 0,
    Tx_Mode,
    Unknow_Mode,
}mode_t;

typedef struct
{
    uint16_t Tx_Success_Count;
    uint16_t Tx_Fail_Count;
    float Tx_Packet_Loss_Rate;
    
    uint16_t Rx_Success_Count;
}communication_info_t;

typedef struct
{
    NRF24L01_Manager_t *Hardware_Mannager;
    Queue_t qTx;
    Queue_t qRx;
    mode_t mode;
    communication_info_t communication_info;
}NRF_Mannager_t;


//Extern引用
extern NRF_Mannager_t NRF_Mannager;


//函数声明
void NRF_Radio_Init(void);
void NRF_Radio_Send(uint8_t *ptr,uint8_t length);
void PollingNRF(void);
void SwitchPort(uint8_t Connect_Port,uint8_t *ptr);
#endif

/******************* (C) 版权所有 2018 北京中科浩电科技有限公司 *******************/
