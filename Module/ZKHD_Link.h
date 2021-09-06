/**
  ******************************************************************************
  * Copyright (c) 2018,北京中科浩电科技有限公司
  * All rights reserved.
  * 文件名称：ZKHD_Link.h
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

#ifndef _ZKHD_LINK_H
#define _ZKHD_LINK_H
//外部文件引用
#include "stdint.h"
#include "stdbool.h"


//宏定义区



//数据结构声明
#pragma pack (1)
typedef struct
{
    uint8_t Start;
    uint16_t Length;
    
    uint8_t Send_ID;
    uint8_t Receive_ID;
    uint8_t Message_ID;
}ZKHD_Link_Head_t;
#pragma pack ()    

typedef enum{
    Device_GCS,
    Device_FMU,
    Device_Remote,
    
    Num_Device,
}emDevice_t;

typedef enum
{
    MsgID_0_Remote_Info,
    MsgID_1_Remote_Config,
    
    Num_Serarch,
}MSG_GCS_To_Remote_t;

typedef enum
{
    MsgID_0_Channel_Info = 0,
    MsgID_1_Config_Info,
    
    Num_Answer,
}MSG_Remote_To_GCS_t;

typedef enum
{
    MsgID_0_Attitude_Info = 0,
    MsgID_1_Motor_Info,
    MsgID_2_Sensor_Info,
    //罗盘
    MsgID_4_Alt_Info = 4,	
    MsgID_5_Hardware_Info,
    MsgID_6_PID_Info,
    MsgID_7_Freq,
    MsgID_8_OrtherInfoGroup1,
    MsgID_9_HardwareList,
    MsgID_10_Line_Info,
    MsgID_11_Mode,
    MsgID_12_Bat,
    MsgID_13_CMDInfo,
    MsgID_14_motor_type,
    MsgID_15_UWBInfo,
    MsgID_16_UWBcoordinate,
    MsgID_17_FFT_datas,
    MsgID_18_Fuse_datas,
    MsgID_19_Acc_datas,
    Num_MSG_FMU_To_GCS,
}MSG_FMU_To_GCS_t;

typedef enum
{
    temp = 0,
    
}MSG_GCS_To_FMU_t;

//Extern引用



//函数声明
void ZKHD_Link_Handle(uint8_t  *Rx_Buff);
void ZKHD_Link_MakeFrame(uint8_t Send_ID,
                          uint8_t Receive_ID,
                          uint8_t Msg_ID,
                          uint8_t *ptr,
                          uint16_t Length,
                          uint8_t *Tx_Buff,uint8_t *);
void ZKHD_Link_FMU_To_GCS_Handle(uint8_t *ptr, uint8_t length);
#endif

/******************* (C) 版权所有 2018 北京中科浩电科技有限公司 *******************/
