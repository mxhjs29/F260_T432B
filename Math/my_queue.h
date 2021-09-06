/**
  ******************************************************************************
  * Copyright (c) 2018,北京中科浩电科技有限公司
  * All rights reserved.
  * 文件名称：queue.h
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
#ifndef _QUEUE_H
#define _QUEUE_H
//外部文件引用
#include "stdint.h"
#include "stdbool.h"
#include "string.h"


//宏定义区
#define QUEUE_DATA_MAXLENGTH         32
#define QUEUE_DATA_BASE_LENGTH       10


//数据结构声明
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


//Extern引用
extern queue_t USB_Send_Queue;


//函数声明
void Queue_Init(queue_t *qPtr);
bool enQueue(queue_t *qPtr, uint8_t *Buff, uint8_t length);
bool deQueue(queue_t *qPtr, uint8_t *Buff, uint8_t *length);

#endif

/******************* (C) 版权所有 2018 北京中科浩电科技有限公司 *******************/
