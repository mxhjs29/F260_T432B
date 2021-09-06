/**
  ******************************************************************************
  * Copyright (c) 2018,北京中科浩电科技有限公司
  * All rights reserved.
  * 文件名称：gcs.h
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
#ifndef _GCS_H
#define _GCS_H
//外部文件引用
#include "main.h"
#include "ZKHD_Link.h"
#include "height_control.h"
#include "remote.h"
#include "imu.h"
#include "control.h"

//宏定义区
#define RATE_5HZ        (5)
#define RATE_10HZ       (10)
#define RATE_50HZ       (50)
#define RATE_100HZ      (100)
#define UART_GCS        UART_6
#define ISRATE(hz)      (Cnt % hz == 0)

//数据结构声明
typedef enum
{
    Stabilize_Mode = 0,
    Altitude_Hold,
    AutoLand,
    AutoMode,
    
    Num_Of_Mode,
}Mode_t;

typedef enum
{
    Connect_Port_USB = 0,
    Connect_Port_NRF,
    
    Num_Connect_Port,
}Connect_Port_t;

typedef enum
{
    MsgID_0_Set_PID = 0,
    MsgID_1_Select_PID,
    MsgID_2_Set_RxAddr,
    MsgID_3_Set_RxFreq,
    MsgID_4_Set_TxAddr,
    
    Num_Msg,
}emMsg_GCS_TO_FMU_t;

typedef struct 
{
    int8_t PID_Select;
    int8_t PID_Pos_Rate;
    int8_t PID_Offset;
    int8_t PID_Set;
}PID_Config_t;

typedef struct 
{
    uint32_t            Runtime;                           //飞行器上电时间      单位：s
    uint8_t             Firmware_Ver;                      //固件版本
    Mode_t              UAV_Mode;                          //当前飞行模式        0：自稳 1：定高
    int16_t             *Motor;                            //电机输出指针
    Connect_Port_t      Connect_Port;                      //连接端口            0:USB     1:NRF
    HeightInfo_t        *Alt;                              //高度信息指针
    MPU6050Manager_t    *Mpu6050;                          //传感器数据指针
    Remote_t            *Remote;                           //遥控器数据指针
    Attitude_t          *AngE;                             //姿态数据指针
    FMUflg_t            *FMUflg;
}UAV_info_t;

//Extern引用
extern UAV_info_t g_UAVinfo;


//初始化和处理函数
void PollingGCS(void);
void gcs_init(void);
void gcs_ReceiveHandle(ZKHD_Link_Head_t *ZKHD_Link_Info, uint8_t *ptr, uint8_t length);
void UpdateUSBQueue(void);

//数据更新函数
void update2gcs_Info(char *p);
void update2gcsotherinfogroup1(void);
void update2gcs_mode(void);
void update2gcsdevecelist(void);
void UpdateToGCSPidInfo(void);
void UpdateToGCSAttitude(void);
void UpdateToGCSAltitude(void);
void UpdateToGCSMPU6050(void);
void UpdateToGCSMotor(void);
void UpdateToGCSHardwareInfo(void);
void UpdateToGCSFreq(void);
void UpdateToGCSLine(int16_t l1, int16_t l2, int16_t l3, int16_t l4, int16_t l5, int16_t l6, int16_t l7, int16_t l8);
void UpdateToGCSLine2(int16_t l1, int16_t l2, int16_t l3, int16_t l4, int16_t l5, int16_t l6, int16_t l7, int16_t l8);
#endif

/******************* (C) 版权所有 2018 北京中科浩电科技有限公司 *******************/
