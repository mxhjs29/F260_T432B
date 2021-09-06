/**
  ******************************************************************************
  * Copyright (c) 2018,�����пƺƵ�Ƽ����޹�˾
  * All rights reserved.
  * �ļ����ƣ�queue.c
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

/*==============================================================================
                         ##### How to use this driver #####
==============================================================================
��������ʹ�÷�ʽ���£�
1.ʵ����һ�����нṹ��
2.ʹ�ú���enQueue��ʹ���������
3.ʹ�ú���deQueue��ʹ���ݳ�����


*/
//�ⲿ�ļ�����
#include "my_queue.h"

 
/******************************************************************************
  * �������ƣ�Queue_Init
  * ������������ʼ��һ������
  * ��    �룺queue_t *qPtr����Ҫ��ʼ���Ķ���ָ��
  * ��    ����void
  * ��    �أ�void
  * ��    ע��null
  *    
  *
******************************************************************************/
void Queue_Init(queue_t *qPtr)
{
    qPtr->front = 0;
    qPtr->rear = 0;
}

/******************************************************************************
  * �������ƣ�enQueue
  * �����������������
  * ��    �룺queue_t *qPtr����Ҫ��ӵĽṹ��ָ��
              uint8_t *Buff����Ҫ��ӵ�����
              uint8_t length����Ҫ��ӵ����ݳ��ȣ��������32�����ȣ�
  * ��    ����void
  * ��    �أ��Ƿ���ӳɹ���false�����ʧ��
                           true����ӳɹ�
  * ��    ע��null
  *    
  *
******************************************************************************/
bool enQueue(queue_t *qPtr, uint8_t *Buff, uint8_t length)
{
    //(qu->rear + 1) % maxsize == qu->front
    if ((qPtr->rear + 1) % QUEUE_DATA_BASE_LENGTH == qPtr->front)
    {
        return false;
    }

    qPtr->rear = (qPtr->rear + 1) % QUEUE_DATA_BASE_LENGTH;
    memcpy(qPtr->Buff[qPtr->rear].Buff, Buff, length);
    qPtr->Buff[qPtr->rear].Length = length;
    return true;
}

/******************************************************************************
  * �������ƣ�deQueue
  * �������������г���
  * ��    �룺queue_t *qPtr����Ҫ���ӵĽṹ��ָ��
  * ��    ����uint8_t *Buff�����ӵ����ݵ�ַ
              uint8_t *length�����ӵ����ݳ��ȵ�ַ
  * ��    �أ��Ƿ���ӳɹ���false������ʧ��
                           true�����ӳɹ�
  * ��    ע��null  
  *    
  *
******************************************************************************/
bool deQueue(queue_t *qPtr, uint8_t *Buff, uint8_t *length)
{
    if (qPtr->front == qPtr->rear)
        return false;
    
    qPtr->front = (qPtr->front + 1) % QUEUE_DATA_BASE_LENGTH;
    int Length = qPtr->Buff[qPtr->front].Length;
    
    if(Length >= QUEUE_DATA_MAXLENGTH)
    {
        return false;
    }
    
    *length = Length;
    memcpy(Buff, qPtr->Buff[qPtr->front].Buff, Length);
    
    return true;
}


/******************* (C) ��Ȩ���� 2018 �����пƺƵ�Ƽ����޹�˾ *******************/