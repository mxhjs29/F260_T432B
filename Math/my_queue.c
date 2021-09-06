/**
  ******************************************************************************
  * Copyright (c) 2018,北京中科浩电科技有限公司
  * All rights reserved.
  * 文件名称：queue.c
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

/*==============================================================================
                         ##### How to use this driver #####
==============================================================================
队列驱动使用方式如下：
1.实例化一个队列结构体
2.使用函数enQueue，使数据入队列
3.使用函数deQueue，使数据出队列


*/
//外部文件引用
#include "my_queue.h"

 
/******************************************************************************
  * 函数名称：Queue_Init
  * 函数描述：初始化一个队列
  * 输    入：queue_t *qPtr：需要初始化的队列指针
  * 输    出：void
  * 返    回：void
  * 备    注：null
  *    
  *
******************************************************************************/
void Queue_Init(queue_t *qPtr)
{
    qPtr->front = 0;
    qPtr->rear = 0;
}

/******************************************************************************
  * 函数名称：enQueue
  * 函数描述：队列入队
  * 输    入：queue_t *qPtr：需要入队的结构体指针
              uint8_t *Buff：需要入队的数据
              uint8_t length：需要入队的数据长度（最长不超过32个长度）
  * 输    出：void
  * 返    回：是否入队成功：false：入队失败
                           true：入队成功
  * 备    注：null
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
  * 函数名称：deQueue
  * 函数描述：队列出队
  * 输    入：queue_t *qPtr：需要出队的结构体指针
  * 输    出：uint8_t *Buff：出队的数据地址
              uint8_t *length：出队的数据长度地址
  * 返    回：是否入队成功：false：出队失败
                           true：出队成功
  * 备    注：null  
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


/******************* (C) 版权所有 2018 北京中科浩电科技有限公司 *******************/
