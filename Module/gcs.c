/**
  ******************************************************************************
  * Copyright (c) 2018,北京中科浩电科技有限公司
  * All rights reserved.
  * 文件名称：gcs.c
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
地面站驱动可按如下方式使用：
1.调用 gcs_init() 函数，以初始化指针指向；
2.将地面站发送到飞控的数据写入函数 ZKHD_Link_GCS_To_FMU_Handle() 中；
3.以一定周期调用如下函数即可在地面站中看到飞控当前信息；
UpdateToGCSPidInfo();
UpdateToGCSAttitude();
UpdateToGCSAltitude();
UpdateToGCSMPU6050();
UpdateToGCSMotor();
UpdateToGCSHardwareInfo();
UpdateToGCSFreq();
*/

//外部文件引用
#include "fmuConfig.h"
#include "gcs.h"
#include "ZKHD_Link.h"
#include "height_control.h"
//#include "battery.h"
#include "Ano_OF.h"
#include "HARDWARE_uart.h"
//#include "ANO_DT.h"
#include "sdk.h"
//#include "spl06.h"
#include "my_queue.h"
//宏定义区



//Extern引用
extern queue_t USB_Send_Queue;
extern PIDInfo_t *(pPidObject[]);
extern int16_t motor[4];
extern _ano_of_st ANO_OF;
//私有函数区



//私有变量区
UAV_info_t g_UAVinfo;
uint8_t PID_select;
PID_Config_t *PID_Config_Info;
float *base = 0;
/******************************************************************************
  * 函数名称：gcs_init
  * 函数描述：初始化地面站相关指针指向
  * 输    入：void
  * 输    出：void
  * 返    回：void
  * 备    注：null    
  *    
  *
******************************************************************************/
void gcs_init(void)
{
    uint8_t ver[5] = {0x00,0x20,0x21,0x08,0x05};
    g_UAVinfo.Firmware_Ver = 02;

    
    g_UAVinfo.Alt = &HeightInfo;
    g_UAVinfo.AngE = &g_Attitude;
    g_UAVinfo.Firmware_Ver = FIRMWARE_INFO;
    g_UAVinfo.Motor = motor;
    g_UAVinfo.Mpu6050 = &g_MPUManager;
    g_UAVinfo.Remote = &Remote;
    g_UAVinfo.FMUflg = &g_FMUflg;
}

/******************************************************************************
  * 函数名称：gcs_ReceiveHandle
  * 函数描述：地面站接收处理函数
  * 输    入：ZKHD_Link_Head_t *ZKHD_Link_Info：ZKHD_Link协议头
              uint8_t *ptr：实际数据地址
              uint8_t length：实际数据长度
  * 输    出：void
  * 返    回：void
  * 备    注：null   
  *
  *
******************************************************************************/
void gcs_ReceiveHandle(ZKHD_Link_Head_t *ZKHD_Link_Info, uint8_t *ptr, uint8_t length)
{
    switch(ZKHD_Link_Info->Message_ID)
    {
        case MsgID_0_Set_PID:
            PID_Config_Info = (PID_Config_t*)ptr;
        
            base = &PIDGroup[PID_Config_Info->PID_Select + 3 * PID_Config_Info->PID_Pos_Rate].kp;
            base += PID_Config_Info->PID_Offset;
            *base += PID_Config_Info->PID_Set / 10.0f;
            break;
        case MsgID_1_Select_PID:
            PID_select = *ptr;
            break;
        default:
            break;
    }
}

/******************************************************************************
  * 函数名称：UpdateToGCSPidInfo
  * 函数描述：更新飞控PID信息到地面站中
  * 输    入：void
  * 输    出：void
  * 返    回：void
  * 备    注：null     
  *    
  *
******************************************************************************/
void UpdateToGCSPidInfo()
{
    static uint8_t yaobai = 0;
    int16_t RawBuff[8];
    uint8_t SendBuff[32];
    uint8_t SendLength;
    
    float *ptr = &PIDGroup[PID_select + yaobai * 3].kp;

    if(yaobai)
    {
        yaobai = 0;
    }
    else
    {
        yaobai = 1;
    }
    
    RawBuff[0] = PID_select;
    RawBuff[0] |= yaobai << 8;
    
    for(int i = 0; i < 7; i++)
    {
        RawBuff[i + 1] = (int16_t)(*ptr++ * 100);
    }
    
    ZKHD_Link_MakeFrame( Device_FMU,
                        Device_GCS,
                        MsgID_6_PID_Info,
                        (uint8_t*)RawBuff,
                        16,
                        SendBuff,
                        &SendLength
                        );

    enQueue(&USB_Send_Queue, SendBuff, SendLength);
}

/******************************************************************************
  * 函数名称：PollingGCS
  * 函数描述：轮询发送GCS的数据
  * 输    入：void
  * 输    出：void
  * 返    回：void 
  * 备    注：固定10ms运行一次
  *    
  *
******************************************************************************/
int16_t FbAlt = 0;
int16_t ExpAlt = 0;
extern s2_t s2_height;
void PollingGCS()
{
    static uint32_t Cnt = 0;
 
    switch(Cnt)
    {
        case 0:
            update2gcsdevecelist();
            UpdateToGCSAttitude();
            break;
        case 1:
            update2gcs_mode();
            UpdateToGCSAltitude();
            break;
        case 2:
            update2gcsotherinfogroup1();
            
            break;
        case 3:
            UpdateToGCSHardwareInfo();
            break;
        case 4:
            UpdateToGCSMPU6050();
            break;
        default:
            break;
    }

    Cnt++;
    
    Cnt = Cnt % 5;
    UpdateUSBQueue();
}

/******************************************************************************
  * 函数名称：UpdateUSBQueue
  * 函数描述：从USB队列中取出数据发送
  * 输    入：void
  * 输    出：void
  * 返    回：void 
  * 备    注：固定周期运行    
  *    
  *
******************************************************************************/
void UpdateUSBQueue()
{
#ifdef USB_CONNECT
    uint8_t Buff[QUEUE_DATA_MAXLENGTH];
    uint8_t length = 0;
    
    USBD_CDC_HandleTypeDef *hcdc = (USBD_CDC_HandleTypeDef*)hUsbDeviceFS.pClassData;
    
    if (hcdc->TxState == 0)
    {
        if (deQueue(&USB_Send_Queue, Buff,&length))
        {
            CDC_Transmit_FS(Buff, length);
        }
    }
#endif
//#ifdef RADIO
    uint8_t Buff[MAX_RECEIVE_CNT];
    uint8_t length = 0;
    
    if (deQueue(&USB_Send_Queue, Buff,&length))
    {
//        HAL_UART_Transmit_DMA(&UsartGroup[UART_GCS].moduleInstance, Buff, length); 
//        USART_TX(&UsartGroup[UART_GCS], Buff, length);
    }
//#endif
}

/******************************************************************************
  * 函数名称：UpdateToGCSAttitude
  * 函数描述：更新飞控姿态信息到地面站中
  * 输    入：void
  * 输    出：void
  * 返    回：void
  * 备    注：null      
  *    
  *
******************************************************************************/
void UpdateToGCSAttitude(void)
{
    uint8_t Buff[20];
    int16_t Angle_Int16[3];
    int sum = 0;
    Buff[0] = 0xAA; //帧头
    Buff[1] = 6;   //长度低八位
    Buff[2] = 0;    //长度高八位
    Buff[3] = 1;    //发送者ID
    Buff[4] = 0;    //接收者ID
    Buff[5] = 0;    //消息ID

    Angle_Int16[0] = (int16_t)(-g_Attitude.pitch * 100);
    Angle_Int16[1] = (int16_t)(-g_Attitude.roll * 100);
    Angle_Int16[2] = (int16_t)(g_Attitude.yaw * 100);

    memcpy(Buff + 6,(uint8_t*)Angle_Int16, 6);

    for(int i = 0; i<((Buff[1]|Buff[2] <<8) + 6); i++)
    {
        sum += Buff[i];
    }

    Buff[(Buff[1]|Buff[2] << 8) + 6] = sum & 0xFF;
    sum = 0;
    
    enQueue(&USB_Send_Queue, Buff,(Buff[1]|Buff[2] << 8) + 7);
}

/******************************************************************************
  * 函数名称：UpdateToGCSAltitude
  * 函数描述：更新飞控高度信息到地面站中
  * 输    入：void
  * 输    出：void
  * 返    回：void
  * 备    注：null      
  *    
  *
******************************************************************************/
void UpdateToGCSAltitude(void)
{
    uint8_t DataBuff[20];
    uint8_t SendBuff[20];
    uint8_t SendLength;
    volatile int16_t Alt = 0;
    
    Alt = (int16_t)(HeightInfo.Z_Postion);
    
    memcpy(DataBuff,(uint8_t *)&Alt, 2);

    ZKHD_Link_MakeFrame( Device_FMU,
                    Device_GCS,
                    MsgID_4_Alt_Info,
                    DataBuff,
                    2,
                    SendBuff,
                    &SendLength
                   );
    
    enQueue(&USB_Send_Queue, SendBuff, SendLength);
}

/******************************************************************************
  * 函数名称：UpdateToGCSMPU6050
  * 函数描述：更新g_MPUManager传感器信息到地面站中
  * 输    入：void
  * 输    出：void
  * 返    回：void
  * 备    注：null      
  *    
  *
******************************************************************************/
void UpdateToGCSMPU6050(void)
{
    uint8_t DataBuff[20];
    uint8_t SendBuff[25];
    uint8_t SendLength;
    
    memcpy(DataBuff,(uint8_t*)g_UAVinfo.Mpu6050, 12);

    ZKHD_Link_MakeFrame( Device_FMU,
                    Device_GCS,
                    MsgID_2_Sensor_Info,
                    DataBuff,
                    12,
                    SendBuff,
                    &SendLength
                   );
    
    enQueue(&USB_Send_Queue, SendBuff, SendLength);
}

/******************************************************************************
  * 函数名称：UpdateToGCSMotor
  * 函数描述：更新电机输出信息到地面站中
  * 输    入：void
  * 输    出：void
  * 返    回：void
  * 备    注：null      
  *
  *
******************************************************************************/
void UpdateToGCSMotor(void)
{
    uint8_t DataBuff[8];
    uint8_t SendBuff[20];
    uint8_t SendLength;
    
    memcpy(DataBuff, (uint8_t *)motor, sizeof(int16_t)*4);

    ZKHD_Link_MakeFrame( Device_FMU,
                    Device_GCS,
                    MsgID_1_Motor_Info,
                    DataBuff,
                    8,
                    SendBuff,
                    &SendLength
                   );
    enQueue(&USB_Send_Queue, SendBuff, SendLength);
}

/******************************************************************************
  * 函数名称：UpdateToGCSHardwareInfo
  * 函数描述：更新硬件输出信息到地面站中
  * 输    入：void
  * 输    出：void
  * 返    回：void
  * 备    注：null      
  *
  *
******************************************************************************/
void UpdateToGCSHardwareInfo(void)
{
    uint8_t DataBuff[13];
    uint8_t SendBuff[20];
    uint8_t SendLength;
    
    DataBuff[0] = g_UAVinfo.Firmware_Ver;//版本信息
//    memcpy(DataBuff + 1, g_UAVinfo.NRF_MannagerPtr->Rx_Addr, 5);//发送地址
//    memcpy(DataBuff + 1 + 5, g_UAVinfo.NRF_MannagerPtr->Tx_Addr, 5);//发送地址
    DataBuff[11] = g_UAVinfo.Runtime&0xFF;
    DataBuff[12] = g_UAVinfo.Runtime>>8;

    ZKHD_Link_MakeFrame( Device_FMU,
                        Device_GCS,
                        MsgID_5_Hardware_Info,
                        DataBuff,
                        13,
                        SendBuff,
                        &SendLength
                       );
    
    enQueue(&USB_Send_Queue, SendBuff, SendLength);
}

/******************************************************************************
  * 函数名称：UpdateToGCSFreq
  * 函数描述：更新飞控输出NRF24L01频率信息到地面站中
  * 输    入：void
  * 输    出：void
  * 返    回：void
  * 备    注：null      
  *
  *
******************************************************************************/
void UpdateToGCSFreq()
{
    uint8_t SendBuff[20];
    uint8_t SendLength;
    
//    ZKHD_Link_MakeFrame( Device_FMU,
//                    Device_GCS,
//                    MsgID_7_Freq,
//                    &g_UAVinfo.NRF_MannagerPtr->RC_Frequency,
//                    1,
//                    SendBuff,
//                    &SendLength
//                    );
//    
//    enQueue(&USB_Send_Queue, SendBuff, SendLength);
}

/******************************************************************************
  * 函数名称：UpdateToGCSLine
  * 函数描述：更新曲线绘制信息到地面站中
  * 输    入：int16_t l1:曲线1
              int16_t l2:曲线2
              int16_t l3:曲线3
              int16_t l2:曲线4
              int16_t l3:曲线5
              int16_t l1:曲线6
              int16_t l2:曲线7
              int16_t l3:曲线8
  * 输    出：void
  * 返    回：void
  * 备    注：null      
  *
  *
******************************************************************************/
void UpdateToGCSLine(int16_t l1, int16_t l2, int16_t l3, int16_t l4, int16_t l5, int16_t l6, int16_t l7, int16_t l8)
{
    int16_t DataBuff[8];
    uint8_t SendBuff[30];
    uint8_t SendLength;
    
    DataBuff[0] = l1;
    DataBuff[1] = l2;
    DataBuff[2] = l3;
    DataBuff[3] = l4;
    DataBuff[4] = l5;
    DataBuff[5] = l6;
    DataBuff[6] = l7;
    DataBuff[7] = l8;
    

    ZKHD_Link_MakeFrame( Device_FMU,
                    Device_GCS,
                    200,
                    (uint8_t*)DataBuff,
                    16,
                    SendBuff,
                    &SendLength
                    );
    
    enQueue(&USB_Send_Queue, SendBuff, SendLength);
}

/******************************************************************************
  * 函数名称：UpdateToGCSLine2
  * 函数描述：更新曲线绘制信息到地面站中，此函数适用于山外调试助手
  * 输    入：int16_t l1:曲线1
              int16_t l2:曲线2
              int16_t l3:曲线3
              int16_t l2:曲线4
              int16_t l3:曲线5
              int16_t l1:曲线6
              int16_t l2:曲线7
              int16_t l3:曲线8
  * 输    出：void
  * 返    回：void
  * 备    注：null  
  *
******************************************************************************/
void UpdateToGCSLine2(int16_t l1, int16_t l2, int16_t l3, int16_t l4, int16_t l5, int16_t l6, int16_t l7, int16_t l8)
{
    int16_t DataBuff[8];
    uint8_t SendBuff[30];
    
    DataBuff[0] = l1;
    DataBuff[1] = l2;
    DataBuff[2] = l3;
    DataBuff[3] = l4;
    DataBuff[4] = l5;
    DataBuff[5] = l6;
    DataBuff[6] = l7;
    DataBuff[7] = l8;
    
    SendBuff[0] = 03;
    SendBuff[1] = 0xFC;
    SendBuff[18] = 0xFC;
    SendBuff[19] = 0x03;
    memcpy(SendBuff + 2,(uint8_t*)DataBuff, 16);

    ZKHD_Link_FMU_To_GCS_Handle(SendBuff, 20);
}

void update2gcsdevecelist(void)
{
    uint8_t DataBuff[15];
    uint8_t SendBuff[30];
    uint8_t SendLength;

    memset(DataBuff, 0, sizeof(DataBuff));
    DataBuff[0] = g_MPUManager.Check;
    //DataBuff[1] 罗盘
    DataBuff[2] = 1;    //气压计
    DataBuff[3] = 0;
    
    //ANO光流
    if (ANO_OF.fc_data_online == 1)
    {
        DataBuff[7] = 1;//声呐
        DataBuff[4] = 1;
    }
    else
    {
        DataBuff[7] = 0;//声呐
        DataBuff[4] = 0;
    }

    //优象光流
    DataBuff[5] = 0;
    //DataBuff[6] GPS
    
    if(Remote.thr < 1000)
    {
        DataBuff[8] = 0;// SBUS
    }else if(Remote.AUX3 == 1500)
    {
        DataBuff[8] = 0;// SBUS
    }else
    {
        DataBuff[8] = 1;
    }
    
    DataBuff[9] = 0; //UWB
    DataBuff[10] = 0;
    //DataBuff[11] 设备0
    //DataBuff[12] 设备1
    //DataBuff[13] 设备2
    //DataBuff[14] 设备3

    ZKHD_Link_MakeFrame(Device_FMU,
                        Device_GCS,
                        MsgID_9_HardwareList,
                        DataBuff,
                        15,
                        SendBuff,
                        &SendLength);

    ZKHD_Link_FMU_To_GCS_Handle(SendBuff, SendLength);
}

void update2gcs_mode(void)
{
    uint8_t DataBuff[20];
    uint8_t SendBuff[20];
    uint8_t SendLength;

    uint8_t mode = (int16_t)g_UAVinfo.UAV_Mode;

    memcpy(DataBuff, (uint8_t *)&mode, 1);

    ZKHD_Link_MakeFrame(Device_FMU,
                        Device_GCS,
                        MsgID_11_Mode,
                        DataBuff,
                        1,
                        SendBuff,
                        &SendLength);

    ZKHD_Link_FMU_To_GCS_Handle(SendBuff, SendLength);
}

void update2gcsotherinfogroup1(void)
{
    uint8_t DataBuff[30];
    uint8_t SendBuff[40];
    uint8_t SendLength;
    int16_t *Dataptr;

    Dataptr = (int16_t *)&DataBuff[0];

    //气压高度值
//    *Dataptr = g_SPL06Manager.fALT * 100;

    //气压速度值
    *(Dataptr + 1) = 0;

    //融合高度值
    *(Dataptr + 2) = (int16_t)HeightInfo.Z_Postion;

    //融合速度值
    *(Dataptr + 3) = (int16_t)HeightInfo.Z_Speed;

    //光流X
    *(Dataptr + 4) = (int16_t)ANO_OF.DX2;

    //光流Y
    *(Dataptr + 5) = (int16_t)ANO_OF.DY2;
    
    *(Dataptr + 6) = 0;
    *(Dataptr + 7) = 0;

    //激光距离
    *(Dataptr + 8) = ANO_OF.ALT;
    *(Dataptr + 9) = 0;
    *(Dataptr + 10) = 0;

    ZKHD_Link_MakeFrame(Device_FMU,
                        Device_GCS,
                        MsgID_8_OrtherInfoGroup1,
                        DataBuff,
                        22,
                        SendBuff,
                        &SendLength);

    ZKHD_Link_FMU_To_GCS_Handle(SendBuff, SendLength);
}

void update2gcs_Info(char *p)
{
    char strbuff[25];
    uint8_t str_len;
    uint8_t SendBuff[32];
    uint8_t SendLength;

    strcpy(strbuff, p);
    str_len = strlen(strbuff); //计算字符串长度

    ZKHD_Link_MakeFrame(Device_FMU,
                        Device_GCS,
                        MsgID_13_CMDInfo,
                        (uint8_t *)strbuff,
                        str_len,
                        SendBuff,
                        &SendLength);

    ZKHD_Link_FMU_To_GCS_Handle(SendBuff, SendLength);
}

/******************* (C) 版权所有 2018 北京中科浩电科技有限公司 *******************/
