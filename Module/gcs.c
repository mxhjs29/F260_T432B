/**
  ******************************************************************************
  * Copyright (c) 2018,�����пƺƵ�Ƽ����޹�˾
  * All rights reserved.
  * �ļ����ƣ�gcs.c
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
����վ�����ɰ����·�ʽʹ�ã�
1.���� gcs_init() �������Գ�ʼ��ָ��ָ��
2.������վ���͵��ɿص�����д�뺯�� ZKHD_Link_GCS_To_FMU_Handle() �У�
3.��һ�����ڵ������º��������ڵ���վ�п����ɿص�ǰ��Ϣ��
UpdateToGCSPidInfo();
UpdateToGCSAttitude();
UpdateToGCSAltitude();
UpdateToGCSMPU6050();
UpdateToGCSMotor();
UpdateToGCSHardwareInfo();
UpdateToGCSFreq();
*/

//�ⲿ�ļ�����
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
//�궨����



//Extern����
extern queue_t USB_Send_Queue;
extern PIDInfo_t *(pPidObject[]);
extern int16_t motor[4];
extern _ano_of_st ANO_OF;
//˽�к�����



//˽�б�����
UAV_info_t g_UAVinfo;
uint8_t PID_select;
PID_Config_t *PID_Config_Info;
float *base = 0;
/******************************************************************************
  * �������ƣ�gcs_init
  * ������������ʼ������վ���ָ��ָ��
  * ��    �룺void
  * ��    ����void
  * ��    �أ�void
  * ��    ע��null    
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
  * �������ƣ�gcs_ReceiveHandle
  * ��������������վ���մ�����
  * ��    �룺ZKHD_Link_Head_t *ZKHD_Link_Info��ZKHD_LinkЭ��ͷ
              uint8_t *ptr��ʵ�����ݵ�ַ
              uint8_t length��ʵ�����ݳ���
  * ��    ����void
  * ��    �أ�void
  * ��    ע��null   
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
  * �������ƣ�UpdateToGCSPidInfo
  * �������������·ɿ�PID��Ϣ������վ��
  * ��    �룺void
  * ��    ����void
  * ��    �أ�void
  * ��    ע��null     
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
  * �������ƣ�PollingGCS
  * ������������ѯ����GCS������
  * ��    �룺void
  * ��    ����void
  * ��    �أ�void 
  * ��    ע���̶�10ms����һ��
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
  * �������ƣ�UpdateUSBQueue
  * ������������USB������ȡ�����ݷ���
  * ��    �룺void
  * ��    ����void
  * ��    �أ�void 
  * ��    ע���̶���������    
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
  * �������ƣ�UpdateToGCSAttitude
  * �������������·ɿ���̬��Ϣ������վ��
  * ��    �룺void
  * ��    ����void
  * ��    �أ�void
  * ��    ע��null      
  *    
  *
******************************************************************************/
void UpdateToGCSAttitude(void)
{
    uint8_t Buff[20];
    int16_t Angle_Int16[3];
    int sum = 0;
    Buff[0] = 0xAA; //֡ͷ
    Buff[1] = 6;   //���ȵͰ�λ
    Buff[2] = 0;    //���ȸ߰�λ
    Buff[3] = 1;    //������ID
    Buff[4] = 0;    //������ID
    Buff[5] = 0;    //��ϢID

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
  * �������ƣ�UpdateToGCSAltitude
  * �������������·ɿظ߶���Ϣ������վ��
  * ��    �룺void
  * ��    ����void
  * ��    �أ�void
  * ��    ע��null      
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
  * �������ƣ�UpdateToGCSMPU6050
  * ��������������g_MPUManager��������Ϣ������վ��
  * ��    �룺void
  * ��    ����void
  * ��    �أ�void
  * ��    ע��null      
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
  * �������ƣ�UpdateToGCSMotor
  * �������������µ�������Ϣ������վ��
  * ��    �룺void
  * ��    ����void
  * ��    �أ�void
  * ��    ע��null      
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
  * �������ƣ�UpdateToGCSHardwareInfo
  * ��������������Ӳ�������Ϣ������վ��
  * ��    �룺void
  * ��    ����void
  * ��    �أ�void
  * ��    ע��null      
  *
  *
******************************************************************************/
void UpdateToGCSHardwareInfo(void)
{
    uint8_t DataBuff[13];
    uint8_t SendBuff[20];
    uint8_t SendLength;
    
    DataBuff[0] = g_UAVinfo.Firmware_Ver;//�汾��Ϣ
//    memcpy(DataBuff + 1, g_UAVinfo.NRF_MannagerPtr->Rx_Addr, 5);//���͵�ַ
//    memcpy(DataBuff + 1 + 5, g_UAVinfo.NRF_MannagerPtr->Tx_Addr, 5);//���͵�ַ
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
  * �������ƣ�UpdateToGCSFreq
  * �������������·ɿ����NRF24L01Ƶ����Ϣ������վ��
  * ��    �룺void
  * ��    ����void
  * ��    �أ�void
  * ��    ע��null      
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
  * �������ƣ�UpdateToGCSLine
  * �����������������߻�����Ϣ������վ��
  * ��    �룺int16_t l1:����1
              int16_t l2:����2
              int16_t l3:����3
              int16_t l2:����4
              int16_t l3:����5
              int16_t l1:����6
              int16_t l2:����7
              int16_t l3:����8
  * ��    ����void
  * ��    �أ�void
  * ��    ע��null      
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
  * �������ƣ�UpdateToGCSLine2
  * �����������������߻�����Ϣ������վ�У��˺���������ɽ���������
  * ��    �룺int16_t l1:����1
              int16_t l2:����2
              int16_t l3:����3
              int16_t l2:����4
              int16_t l3:����5
              int16_t l1:����6
              int16_t l2:����7
              int16_t l3:����8
  * ��    ����void
  * ��    �أ�void
  * ��    ע��null  
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
    //DataBuff[1] ����
    DataBuff[2] = 1;    //��ѹ��
    DataBuff[3] = 0;
    
    //ANO����
    if (ANO_OF.fc_data_online == 1)
    {
        DataBuff[7] = 1;//����
        DataBuff[4] = 1;
    }
    else
    {
        DataBuff[7] = 0;//����
        DataBuff[4] = 0;
    }

    //�������
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
    //DataBuff[11] �豸0
    //DataBuff[12] �豸1
    //DataBuff[13] �豸2
    //DataBuff[14] �豸3

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

    //��ѹ�߶�ֵ
//    *Dataptr = g_SPL06Manager.fALT * 100;

    //��ѹ�ٶ�ֵ
    *(Dataptr + 1) = 0;

    //�ںϸ߶�ֵ
    *(Dataptr + 2) = (int16_t)HeightInfo.Z_Postion;

    //�ں��ٶ�ֵ
    *(Dataptr + 3) = (int16_t)HeightInfo.Z_Speed;

    //����X
    *(Dataptr + 4) = (int16_t)ANO_OF.DX2;

    //����Y
    *(Dataptr + 5) = (int16_t)ANO_OF.DY2;
    
    *(Dataptr + 6) = 0;
    *(Dataptr + 7) = 0;

    //�������
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
    str_len = strlen(strbuff); //�����ַ�������

    ZKHD_Link_MakeFrame(Device_FMU,
                        Device_GCS,
                        MsgID_13_CMDInfo,
                        (uint8_t *)strbuff,
                        str_len,
                        SendBuff,
                        &SendLength);

    ZKHD_Link_FMU_To_GCS_Handle(SendBuff, SendLength);
}

/******************* (C) ��Ȩ���� 2018 �����пƺƵ�Ƽ����޹�˾ *******************/
