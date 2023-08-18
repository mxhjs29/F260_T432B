/**
  ******************************************************************************
  * Copyright (c) 2018,�����пƺƵ�Ƽ����޹�˾
  * All rights reserved.
  * �ļ����ƣ�queue.h
  * ժ    Ҫ��
  *
  * ��ǰ�汾��V1.0
  * ��    �ߣ������пƺƵ�Ƽ����޹�˾�з��� 
  * ������ڣ�    
  * �޸�˵����
  * 
  *
  * ��ʷ�汾��
  *
  *
  *******************************************************************************/
#ifndef _QUEUE_H
#define _QUEUE_H
//�ⲿ�ļ�����
#include "stdint.h"
#include "stdbool.h"
#include "string.h"


//�궨����
#define QUEUE_DATA_MAXLENGTH         32
#define QUEUE_DATA_BASE_LENGTH       10


//���ݽṹ����
typedef struct 
{
    uint8_t Buff[QUEUE_DATA_MAXLENGTH];
    uint8_t Length;
}Buff_t;

typedef struct
{
    uint8_t front;
    uint8_t rear;
    Buff_t Buff[QUEUE_DATA_BASE_LENGTH];
}queue_t;


//Extern����
extern queue_t USB_Send_Queue;


//��������
void Queue_Init(queue_t *qPtr);
bool enQueue(queue_t *qPtr, uint8_t *Buff, uint8_t length);
bool deQueue(queue_t *qPtr, uint8_t *Buff, uint8_t *length);

#endif

/******************* (C) ��Ȩ���� 2018 �����пƺƵ�Ƽ����޹�˾ *******************/
