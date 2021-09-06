/**
  ******************************************************************************
  * Copyright (c) 2018,北京中科浩电科技有限公司
  * All rights reserved.
  * 文件名称：ZKHD_Link.c
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



*/
//外部文件引用
#include "ZKHD_Link.h"
#include "string.h"
#include "gcs.h"
#include "my_queue.h"

//宏定义区



//Extern引用



//私有函数区
void ZKHD_Link_GCS_To_Remote_Handle(ZKHD_Link_Head_t *,uint8_t *ptr, uint8_t length);
void ZKHD_Link_GCS_To_FMU_Handle(ZKHD_Link_Head_t *,uint8_t *ptr, uint8_t length);


//私有变量区

/******************************************************************************
  * 函数名称：ZKHD_Link_MakeFrame
  * 函数描述：组一个发向上位机的帧
  * 输    入：uint8_t Send_ID：发送者ID
            uint8_t Receive_ID:接收者ID
            uint8_t Msg_ID：消息ID
            uint8_t *ptr:需要发送的数据地址
            uint16_t Length：需要发送的数据的长度
            
  * 输    出：uint8_t *Tx_Buff：最终完成的帧起始地址
              uint8_t *Tx_Buff_Length：最终完成的帧的长度
  * 返    回：void 
  * 备    注：null 
  *    
  *
******************************************************************************/
void ZKHD_Link_MakeFrame(uint8_t Send_ID,
                        uint8_t Receive_ID,
                        uint8_t Msg_ID,
                        uint8_t *ptr,
                        uint16_t Length,
                        uint8_t *Tx_Buff,
                        uint8_t *Tx_Buff_Length)
{
    uint8_t Buff[100];
    int sum = 0;
    
    ZKHD_Link_Head_t ZKHD_Link_Head;
    
    ZKHD_Link_Head.Send_ID = Send_ID;
    ZKHD_Link_Head.Receive_ID = Receive_ID;
    ZKHD_Link_Head.Start = 0xAA;
    ZKHD_Link_Head.Length = Length;
    ZKHD_Link_Head.Message_ID = Msg_ID;
    
    memcpy(Buff,&ZKHD_Link_Head.Start, sizeof(ZKHD_Link_Head_t));
    memcpy(Buff + sizeof(ZKHD_Link_Head_t),ptr, Length);
    
    for(int i = 0; i < Length + 6; i++)
    {
        sum += Buff[i];
    }
    
    Buff[Length + sizeof(ZKHD_Link_Head_t)] = sum & 0xFF;
    
    memcpy(Tx_Buff, Buff, Length + sizeof(ZKHD_Link_Head_t) + 1);
    *Tx_Buff_Length = Length + sizeof(ZKHD_Link_Head_t) + 1;
}

/******************************************************************************
  * 函数名称：ZKHD_Link_Handle
  * 函数描述：接收来自上位机的数据，并解析
  * 输    入：uint8_t *Rx_Buff：接收数据的地址
  * 输    出：void
  * 返    回：void 
  * 备    注：由于长度信息包含在了接收数据的长度中，所以可以单独指出长度数据    
  *    
  *
******************************************************************************/
void ZKHD_Link_Handle(uint8_t *Rx_Buff)
{
    ZKHD_Link_Head_t *ZKHD_Link_Head_Ptr;
    
    ZKHD_Link_Head_Ptr = (ZKHD_Link_Head_t*)Rx_Buff;
    
    if(ZKHD_Link_Head_Ptr->Send_ID == Device_GCS)
    {
        if(ZKHD_Link_Head_Ptr->Receive_ID == Device_Remote)
        {
            ZKHD_Link_GCS_To_Remote_Handle(ZKHD_Link_Head_Ptr,&ZKHD_Link_Head_Ptr->Message_ID + 1, ZKHD_Link_Head_Ptr->Length);
        }else if(ZKHD_Link_Head_Ptr->Receive_ID == Device_FMU)
        {
            ZKHD_Link_GCS_To_FMU_Handle(ZKHD_Link_Head_Ptr,&ZKHD_Link_Head_Ptr->Message_ID + 1, ZKHD_Link_Head_Ptr->Length);
        }
    }
}

void ZKHD_Link_FMU_To_GCS_Handle(uint8_t *ptr, uint8_t length)
{    

    enQueue(&USB_Send_Queue, ptr, length);

}

/******************************************************************************
  * 函数名称：ZKHD_Link_GCS_To_Remote_Handle
  * 函数描述：上位机发送到遥控器的处理函数
  * 输    入：ZKHD_Link_Head_t *ZKHD_Link_Head：ZKHD_Link协议头
  *           uint8_t *ptr：接收数据的地址；
  *           uint8_t length：接收数据的长度；
  * 输    出：void
  * 返    回：void 
  * 备    注：null      
  *    
  *
******************************************************************************/
void ZKHD_Link_GCS_To_Remote_Handle(ZKHD_Link_Head_t *ZKHD_Link_Head, uint8_t *ptr, uint8_t length)
{
    
}

/******************************************************************************
  * 函数名称：ZKHD_Link_GCS_To_FMU_Handle
  * 函数描述：上位机发送到飞控的处理函数
  * 输    入：ZKHD_Link_Head_t *ZKHD_Link_Head：ZKHD_Link协议头
  *           uint8_t *ptr：接收数据的地址；
  *           uint8_t length：接收数据的长度；
  * 输    出：void
  * 返    回：void 
  * 备    注：null    
  *    
  *
******************************************************************************/
void ZKHD_Link_GCS_To_FMU_Handle(ZKHD_Link_Head_t *ZKHD_Link_Head, uint8_t *ptr, uint8_t length)
{
    gcs_ReceiveHandle(ZKHD_Link_Head, ptr, length);
} 


/******************* (C) 版权所有 2018 北京中科浩电科技有限公司 *******************/
