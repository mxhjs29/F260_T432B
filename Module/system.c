/**
  ******************************************************************************
  * Copyright (c) 2018,�����пƺƵ�Ƽ����޹�˾
  * All rights reserved.
  * �ļ����ƣ�kernel.c
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
ѭ������������ʹ�øú�����ѵ��������


*/
//�ⲿ�ļ�����
#include "include.h"
//#include "communication.h"
#include "gcs.h"
#include "speed_estimator.h"
#include "led.h"
//#include "battery.h"
#include "stdio.h"
#include "HARDWARE_uart.h"
//#include "SPL06.h"
//#include "timer_drv.h"
#include "Ano_OF.h"
#include "pos_ctrl.h"
#include "program_ctrl.h"
//#include "ANO_GCS_DT.h"
#include "FollowLine.h"
//#include "ANO_DT.h"
#include "sdk.h"
#include "beep.h"
#include "lcd.h"

//�궨����

#define KernelRunningCheck  if(!KernelRunning){ return;}

//Extern����
extern void lcd_update(uint8_t cnt);
extern void update_status(void);

//˽�к�����
void Update(void);
void UpdateUSBQueue(void);


//˽�б�����
bool KernelRunning = false;
extern bool InitComplete;
int16_t Angle_Int16[3];
uint8_t Buff[20];
int sum = 0;

/******************************************************************************
  * �������ƣ�KernelPolling
  * ����������������ѯ����
  * ��    �룺void
  * ��    ����void
  * ��    �أ�void 
  * ��    ע��1ms����һ��    
  *    
  *
******************************************************************************/
void KernelPolling()
{
    static uint32_t Cnt = 0;

    //�������м��
    KernelRunningCheck;

    KernelRunning = false;
 
    //ʱ�����ѯ����
    Cnt++;
    
    //333Hz,�޸ĵ��δ��ʱ������
    if (Cnt % 3 == 0)
    {        
        //����PID����������
        FlightPidControl(0.003f);
        
        //����߼�����
        MotorControl();
    }
    
    //125Hz
    if (Cnt % 8 == 0)
    { 
        //����ŷ����
        GetAngle(&g_Attitude);
    }

    //91Hz
    if (Cnt % 11 == 0)
    {
        //����״̬��������
        AnoOF_State_Task(11);
        
        float distance = FollowManager.FrontOpenmvFramePtr->cnt1 | (FollowManager.FrontOpenmvFramePtr->Target >> 8);
    }
    
    //100Hz
    if (Cnt % 10 == 0)
    {        
        //����Ѳ������
        //���棡����
        //�˺��������Զ��������ݣ������д˺����Ժ�
        //����LaunchPad�尴ť���Զ�����ʱ�����ɻ�������������������̵�ʱ��
        //��Ҫ���׽��ע��
        
        //�˺���Ϊ�̿���ں����������˽�̿ػ��ƺ󣬿�ȡ���˺���ע��
        UpdateCentControl(0.01f);
        
        sdk_update(0.01f);
    }
    
    //50Hz����
    if (Cnt % 20 == 0)
    {
        //�۲⴫�������ݼ���
        WZ_Obs_Calcu(0.02f);
        
        //Z�����ݻ�������
        WZ_Fix_Calcu(0.02f);
        
        //�߶ȿ�����
        ALT_Ctrl(0.02f);        
        
        //λ�ÿ�����
        POS_Ctrl(0.02f);
    }
    
    //20Hz����
    if (Cnt % 10 == 0)
    {
        PollingGCS();
       
        lcd_update(0);
    }
    
    update_status();

    //LED��ѵ����
    PollingLED();
    
    //������ѵ����
    PollingUSART();
    
    beep_update();
}

/******************* (C) ��Ȩ���� 2018 �����пƺƵ�Ƽ����޹�˾ *******************/
