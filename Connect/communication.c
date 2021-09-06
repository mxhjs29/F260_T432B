/**
  ******************************************************************************
  * Copyright (c) 2018,北京中科浩电科技有限公司
  * All rights reserved.
  * 文件名称：communication.c
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
#include "communication.h"
#include "gcs.h"
#include "nrf24l01.h"


//宏定义区
#define NRF_CH_FLASH_ADDR       (uint8_t*)PAGE_ADDRESS
#define NRF_RX_ADDR_FLASH_ADDR  (uint8_t*)(PAGE_ADDRESS + 2)
#define NRF_TX_ADDR_FLASH_ADDR  (uint8_t*)(PAGE_ADDRESS + 2 + 5)


//Extern引用
extern uint8_t WriteBuf(uint8_t regaddr, uint8_t *pBuf, uint8_t datalen);    //写数据区
extern uint8_t ReadBuf(uint8_t regaddr, uint8_t *pBuf, uint8_t datalen);     //读数据区          
extern uint8_t ReadReg(uint8_t regaddr);                                     //读寄存器
extern uint8_t WriteReg(uint8_t regaddr, uint8_t data);                      //写寄存器


//私有函数区



//私有变量区
NRF_Mannager_t NRF_Mannager;
uint8_t TX_ADDRESS[]= {0xE1, 0xE2, 0xE3, 0xE4, 0xE5};    //本地默认发送地址
uint8_t RX_ADDRESS[]= {0x00, 0x00, 0x00, 0x00, 0xE5};    //本机默认接收地址


/******************************************************************************
  * 函数名称：NRF_Radio_Init
  * 函数描述：初始化NRF24L01通信机制
  * 输    入：void
  * 输    出：void
  * 返    回：void
  * 备    注：null
  *    
  *
******************************************************************************/
void NRF_Radio_Init(void)
{
    uint8_t defalut[5] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF};
    
    /*此处缺少从FLASH处读取地址*/
    NRF_Mannager.Hardware_Mannager = &g_NRFManager;
    
    //表示未初始化,使用默认值
    if(memcmp((uint8_t*)NRF_RX_ADDR_FLASH_ADDR, defalut, 5) == 0)
    {
        memcpy(g_NRFManager.Rx_Addr, RX_ADDRESS, 5);
        memcpy(g_NRFManager.Tx_Addr, TX_ADDRESS, 5);
        g_NRFManager.RC_Frequency = NRF24L01_FREQ;
    }else
    {
        
        g_NRFManager.RC_Frequency = *((uint8_t*)NRF_CH_FLASH_ADDR);
        memcpy(g_NRFManager.Rx_Addr, NRF_RX_ADDR_FLASH_ADDR, 5);
        memcpy(g_NRFManager.Tx_Addr, NRF_TX_ADDR_FLASH_ADDR, 5);
    }
    
    g_NRFManager.NRF_Mode = NRF_Mode_RX2;
    NRF24L01Init(&g_NRFManager);
}

/******************************************************************************
  * 函数名称：NRF_Radio_Send
  * 函数描述：NRF24L01通信发送
  * 输    入：uint8_t *ptr：发送的数据的地址
  *           uint8_t length：发送的数据的长度
  * 输    出：void
  * 返    回：void 
  * 备    注：null    
  *    
  *
******************************************************************************/
void NRF_Radio_Send(uint8_t *ptr, uint8_t length)
{
    enQueue(&NRF_Mannager.qTx, ptr, length);
}

/******************************************************************************
  * 函数名称：PollingNRF
  * 函数描述：NRF24L01中断轮训函数
  * 输    入：void
  * 输    出：void
  * 返    回：void 
  * 备    注：null    
  *    
  *
******************************************************************************/
void PollingNRF()
{
    static int rx_succsess_cnt = 0;
    uint8_t state = 0;
    uint8_t rxbuff_temp[RX_PLOAD_WIDTH]; 
    
    state = ReadReg(STATUS);
    WriteReg(SPI_WRITE_REG + STATUS, state);
    
    if(state & MAX_TX)
    {
        WriteReg(STATUS, MAX_TX);
        NRF_Mannager.communication_info.Tx_Success_Count++;
        
        NRF_Mannager.Hardware_Mannager->set_rx_mode();
        NRF_Mannager.mode = Rx_Mode;
    }
   
    if(state & RX_OK)
    {
        rx_succsess_cnt++;
        WriteReg(STATUS, RX_OK);
        state = ReadReg(STATUS);
        ReadBuf(RD_RX_PLOAD, rxbuff_temp, RX_PLOAD_WIDTH);//读取数据
        enQueue(&NRF_Mannager.qRx, rxbuff_temp, RX_PLOAD_WIDTH);       
        WriteReg(FLUSH_RX, 0xff);          //清除RX FIFO寄存器 
        NRF_Mannager.communication_info.Rx_Success_Count++;\
        
        SwitchPort(Connect_Port_NRF, rxbuff_temp);
        AnalyRC();
    }
    
    {//处理发送信息
        uint8_t buff_temp[32];
        uint8_t length;
        
        memset(buff_temp, 0xFF, 32);
        if(deQueue(&NRF_Mannager.qTx, buff_temp,&length))
        {
            switch(g_NRFManager.NRF_Mode)
            {
                case NRF_Mode_TX:
                case NRF_Mode_TX2:
                    NRF24L01TxPacket(buff_temp, length);
                    break;
                case NRF_Mode_RX2:
                    NRF24L01TxPacketAp(buff_temp, length);
                    break;
                default:
                    break;
            }
        }
    }

    //计算丢包率
    NRF_Mannager.communication_info.Tx_Packet_Loss_Rate = 
        NRF_Mannager.communication_info.Tx_Fail_Count * 1.0f / (NRF_Mannager.communication_info.Tx_Fail_Count + NRF_Mannager.communication_info.Tx_Success_Count);
}

/******************************************************************************
  * 函数名称：SwitchPort
  * 函数描述：通信端口选择
  * 输    入：数据接收端口开关，用来给出端口信息
  * 输    出：void
  * 返    回：void 
  * 备    注：null    
  *    
  *
******************************************************************************/
void SwitchPort(Connect_Port_t Connect_Port, uint8_t *ptr)
{
    switch(Connect_Port)
    {
        case Connect_Port_USB:
            g_UAVinfo.Connect_Port = Connect_Port_USB;
            break;
        case Connect_Port_NRF:
            g_UAVinfo.Connect_Port = Connect_Port_NRF;
            break;
        default:
            break;
    }
    ZKHD_Link_Handle(ptr);
}

/******************* (C) 版权所有 2018 北京中科浩电科技有限公司 *******************/
