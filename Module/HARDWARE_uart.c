/**
  ******************************************************************************
  * Copyright (c) 2018,�����пƺƵ�Ƽ����޹�˾
  * All rights reserved.
  * �ļ����ƣ�UART.c
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
�����ڵ��������

*/
//�ⲿ�ļ�����
#include "HARDWARE_uart.h"
#include "pid.h"
#include "include.h"
#include "Remote.h"
#include "ZKHD_Link.h"
#include "myMath.h"
#include "program_ctrl.h"
#include "FollowLine.h"
#include "usart.h"
#include "ano_of.h"
#include "stm32f4xx_it.h"
#include "UWB.h"
#define USART_RX_TIMEOUT_MAX 5

TF_Luna_t TF_Luna;
bool is_uart_init_compleate = false;
extern Remote_t Remote;
bool lbUsartRx = false;
int TimeoutUSART = 0;
uint8_t SBusRxBuff[150] = {0};

uint8_t UART4_TxBUFF_1[5];
uint8_t UART4_TxBUFF_2[5];
Usart_t UsartGroup[Num_USART];

void UART_1_ReceiveHandle(uint8_t *ptr, uint8_t length);
void UART_2_ReceiveHandle(uint8_t *ptr, uint8_t length);
void UART_3_ReceiveHandle(uint8_t *ptr, uint8_t length);
void UART_4_ReceiveHandle(uint8_t *ptr, uint8_t length);
void UART_5_ReceiveHandle(uint8_t *ptr, uint8_t length);
void UART_6_ReceiveHandle(uint8_t *ptr, uint8_t length);
static int UART_Receive_DMA_No_IT(UART_HandleTypeDef* huart, uint8_t* pData, uint32_t Size);
/******************************************************************************
  * �������ƣ�USART_Init
  * �������������ڳ�ʼ��
  * ��    �룺void
  * ��    ����void
  * ��    �أ�void
  * ��    ע��TI��˾ͨ��������ַ�ṩ���û�������������Ϣ
******************************************************************************/
void USART_Init()
{
    UsartGroup[UART_1].RxHandle = UART_1_ReceiveHandle;
    UsartGroup[UART_2].RxHandle = UART_2_ReceiveHandle;
    UsartGroup[UART_3].RxHandle = UART_3_ReceiveHandle;
    UsartGroup[UART_4].RxHandle = UART_4_ReceiveHandle;
    UsartGroup[UART_5].RxHandle = UART_5_ReceiveHandle;
    UsartGroup[UART_6].RxHandle = UART_6_ReceiveHandle;
   
    UsartGroup[UART_1].TxCommpleate = true;
    UsartGroup[UART_2].TxCommpleate = true;
    UsartGroup[UART_3].TxCommpleate = true;
    UsartGroup[UART_4].TxCommpleate = true;
    UsartGroup[UART_5].TxCommpleate = true;
    UsartGroup[UART_6].TxCommpleate = true;

    UsartGroup[UART_1].moduleInstance = huart1;
    UsartGroup[UART_2].moduleInstance = huart2;
    UsartGroup[UART_3].moduleInstance = huart3;
    UsartGroup[UART_4].moduleInstance = huart4;
    UsartGroup[UART_5].moduleInstance = huart5;
    UsartGroup[UART_6].moduleInstance = huart6;
    
    __HAL_UART_ENABLE_IT(&huart1, UART_IT_IDLE);
    __HAL_UART_ENABLE_IT(&huart2, UART_IT_RXNE);
    __HAL_UART_ENABLE_IT(&huart3, UART_IT_IDLE);
    __HAL_UART_ENABLE_IT(&huart4, UART_IT_IDLE);
    __HAL_UART_ENABLE_IT(&huart5, UART_IT_IDLE);
    __HAL_UART_ENABLE_IT(&huart6, UART_IT_IDLE);
    
    HAL_UART_Receive_DMA(&UsartGroup[UART_1].moduleInstance,UsartGroup[UART_1].RxBuff,MAX_RECEIVE_CNT);
//    HAL_UART_Receive_DMA(&UsartGroup[UART_2].moduleInstance,UsartGroup[UART_2].RxBuff,MAX_RECEIVE_CNT);
    HAL_UART_Receive_DMA(&UsartGroup[UART_3].moduleInstance,UsartGroup[UART_3].RxBuff,MAX_RECEIVE_CNT);
    HAL_UART_Receive_DMA(&UsartGroup[UART_4].moduleInstance,UsartGroup[UART_4].RxBuff,MAX_RECEIVE_CNT);
    HAL_UART_Receive_DMA(&UsartGroup[UART_5].moduleInstance,UsartGroup[UART_5].RxBuff,MAX_RECEIVE_CNT);
    HAL_UART_Receive_DMA(&UsartGroup[UART_6].moduleInstance,UsartGroup[UART_6].RxBuff,MAX_RECEIVE_CNT);

    __HAL_UART_CLEAR_OREFLAG(&UsartGroup[UART_1].moduleInstance);
    __HAL_UART_CLEAR_OREFLAG(&UsartGroup[UART_2].moduleInstance);
    __HAL_UART_CLEAR_OREFLAG(&UsartGroup[UART_3].moduleInstance);
    __HAL_UART_CLEAR_OREFLAG(&UsartGroup[UART_4].moduleInstance);
    __HAL_UART_CLEAR_OREFLAG(&UsartGroup[UART_5].moduleInstance);
    __HAL_UART_CLEAR_OREFLAG(&UsartGroup[UART_6].moduleInstance);
    
    UsartGroup[UART_1].last_rx_cnt = MAX_RECEIVE_CNT;
    UsartGroup[UART_2].last_rx_cnt = MAX_RECEIVE_CNT;
    UsartGroup[UART_3].last_rx_cnt = MAX_RECEIVE_CNT;
    UsartGroup[UART_4].last_rx_cnt = MAX_RECEIVE_CNT;
    UsartGroup[UART_5].last_rx_cnt = MAX_RECEIVE_CNT;
    UsartGroup[UART_6].last_rx_cnt = MAX_RECEIVE_CNT;
    
    is_uart_init_compleate = true;
	

	
}

static int UART_Receive_DMA_No_IT(UART_HandleTypeDef* huart, uint8_t* pData, uint32_t Size)
{
    uint32_t tmp1 = 0;
    tmp1 = huart->RxState;
    if (tmp1 == HAL_UART_STATE_READY)
    {
        if ((pData == NULL) || (Size == 0))
        {
            return HAL_ERROR;
        }
        __HAL_LOCK(huart);
        huart->pRxBuffPtr = pData;
        huart->RxXferSize = Size;
        huart->ErrorCode  = HAL_UART_ERROR_NONE;
        HAL_DMA_Start(huart->hdmarx, (uint32_t)&huart->Instance->DR,(uint32_t)pData, Size);
        SET_BIT(huart->Instance->CR3, USART_CR3_DMAR);
        __HAL_UNLOCK(huart);
        return HAL_OK;
    }
    else
    {
        return HAL_BUSY;
    }
}


/******************************************************************************
  * �������ƣ�PollingUSART
  * ������������ѯ���ڽ�����Ϣ
  * ��    �룺void
  * ��    ����void
  * ��    �أ�void
  * ��    ע������ϵͳ����λ��ͨ�ţ��˴�Ϊ���
  *
  *
******************************************************************************/
void PollingUSART()
{
    if(!is_uart_init_compleate)
        return ;
    for (int i = 0; i < Num_USART; i++)
    {
        if (UsartGroup[i].TxCommpleate)
        {
            // �ж�DMA�����Ƿ����
//            if(UsartGroup[i].moduleInstance.hdmatx->State == HAL_DMA_STATE_READY)
//            {
                if (deQueue(&UsartGroup[i].qTx, UsartGroup[i].TxBuff, &UsartGroup[i].TxLength))
                {
                    UsartGroup[i].TxCommpleate = false;
//                    HAL_StatusTypeDef HAL_UART_Transmit_DMA(UART_HandleTypeDef *huart, uint8_t *pData, uint16_t Size)
                    HAL_UART_Transmit_DMA(&UsartGroup[i].moduleInstance, UsartGroup[i].TxBuff, UsartGroup[i].TxLength); 
                }
//            }
        }
    }
}

/******************************************************************************
  * �������ƣ�USART_TX
  * �������������жϵķ�ʽ��������
  * ��    �룺
  * Usart_t usart:ѡ��Ҫ�������ݵ�ģ��
  * uint8_t *ptx:Ҫ���͵����ݵ�ַ
  * uint8_t len:Ҫ���͵����ݳ���
  * ��    ����void
  * ��    �أ�void
  * ��    ע��null
  *
  *
******************************************************************************/
void USART_TX(Usart_t *usart, uint8_t *pTx, uint8_t len)
{
    for (int i = 0; i < Num_USART; i++)
    {
        if (usart == &UsartGroup[i])
        {
            enQueue(&UsartGroup[i].qTx, pTx, len);
            break;
        }
    }
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
    if(huart == &huart1)
    {

    }
    
    if(huart == &huart2)
    {
        
    }
    
    if(huart == &huart3)
    {
        
    }
    
    if(huart == &huart4)
    {
        
    }
    
    if(huart == &huart5)
    {
        
    }
    
    if(huart == &huart6)
    {
        
    }
    
}

void usart_rx_idle_handle(emUSART_t uart)
{
    if(!is_uart_init_compleate)
        return ;
    
//    __HAL_DMA_DISABLE(UsartGroup[uart].moduleInstance.hdmarx);
    HAL_UART_DMAStop(&UsartGroup[uart].moduleInstance);
    
	UsartGroup[uart].RxCnt = MAX_RECEIVE_CNT - UsartGroup[uart].moduleInstance.hdmarx->Instance->NDTR;
    
	memcpy(UsartGroup[uart].RxHandleBuff, UsartGroup[uart].RxBuff, UsartGroup[uart].RxCnt);
	
	UsartGroup[uart].RxHandle(UsartGroup[uart].RxBuff, UsartGroup[uart].RxCnt);

  
	
//    __HAL_DMA_SET_COUNTER(UsartGroup[uart].moduleInstance.hdmarx, MAX_RECEIVE_CNT);
//    __HAL_DMA_ENABLE(UsartGroup[uart].moduleInstance.hdmarx);
    UsartGroup[uart].moduleInstance.RxState = HAL_UART_STATE_READY;
    HAL_UART_Receive_DMA(&UsartGroup[uart].moduleInstance,UsartGroup[uart].RxBuff,MAX_RECEIVE_CNT);
}

void UART_1_ReceiveHandle(uint8_t *ptr, uint8_t length)
{
    memcpy(SBusRxBuff, ptr, length);
    Remote.update_flag = true;
    RemotePolling();
}

void UART_2_ReceiveHandle(uint8_t *ptr, uint8_t length)
{
    for(int i = 0;i<MAX_RECEIVE_CNT;i++)
    {
        AnoOF_GetOneByte(ptr[i]);
    }
}

void UART_3_ReceiveHandle(uint8_t *ptr, uint8_t length)
{

	if(*ptr==0xaa)
	{
		FollowManager.GroundOpenmvFramePtr = (Ground_OpenMVFrame_t *)ptr;
	}
}

void UART_4_ReceiveHandle(uint8_t *ptr, uint8_t length)
{
	if(*ptr == 'f' && *(ptr+2) == 'e')
	{
		if(*(ptr+1)==1)
			FollowApriTag = true;
	}
		
	
}

static uint8_t a;
void UART_5_ReceiveHandle(uint8_t *ptr, uint8_t length)
{
	
	if(*ptr == 0x59)
	{
		TF_Luna.head = *ptr;
		TF_Luna.id = *(ptr + 1);
		if(TF_Luna.head == 0x59 && TF_Luna.id == 0x59)
		{
			TF_Luna.dist = (uint16_t)ptr[3] << 8 | ptr[2];
			TF_Luna.amp = (uint16_t)ptr[5] << 8 | ptr[4];
			TF_Luna.temp = (uint16_t)ptr[7] << 8 | ptr[6];
			TF_Luna.check_sum = ptr[8];
			if(TF_Luna.amp >= 100)
				ANO_OF.ALT = TF_Luna.dist;

		}
	}

}

void UART_6_ReceiveHandle(uint8_t *ptr, uint8_t length)
{

	

	
	
	static uint16_t b=0;
//	for(b=0;b<128;b++)
//	{
		if(ptr[b] == 0x55 && ptr[b+1] == 0x04)
		{
			UWB_Manager.frame_head=ptr[b+0];
			UWB_Manager.function_mark=ptr[b+1];
			if(ptr[b+0]==0x55&&ptr[b+1]==0x04)
			{
				UWB_Manager.frame_length=ptr[b+2]|ptr[b+3]<<8;
				UWB_Manager.role=ptr[b+4];
				UWB_Manager.id=ptr[b+5];
				if(UWB_Manager.role==0x02&&UWB_Manager.id==0x00)
				{
					UWB_Manager.system_time=ptr[b+6]|ptr[b+7]<<8|ptr[b+8]<<16|ptr[b+9]<<24;
					UWB_Manager.pos_x=(ptr[b+13]|ptr[b+14]<<8|ptr[b+15]<<16)/10;
					UWB_Manager.pos_y=(ptr[b+16]|ptr[b+17]<<8|ptr[b+18]<<16)/10;
					UWB_Manager.pos_z=(ptr[b+19]|ptr[b+20]<<8|ptr[b+21]<<16)/10;//100000;			
				}		
			}
		
		}
			
//	}

	

}


/******************* (C) ��Ȩ���� 2018 �����пƺƵ�Ƽ����޹�˾ *******************/
