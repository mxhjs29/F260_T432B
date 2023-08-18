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
#include "power.h"
#include "bt.h"
#include "stm32_hal_legacy.h"
#include "tim.h"
#include "usart.h"
#include "UWB.h"
//�궨����

#define KernelRunningCheck  if(!KernelRunning){ return;}

//Extern����
extern void lcd_update(uint8_t cnt);
extern void update_status(void);
extern int ano_of_alt_count;
extern int ano_of_count;


//˽�к�����
void Update(void);
void UpdateUSBQueue(void);


//˽�б�����
bool KernelRunning = false;
extern bool InitComplete;
int16_t Angle_Int16[3];
uint8_t Buff[20];
int sum = 0;
uint8_t UART4_SEND_BUFF[15];
union LongUint8_t t;
extern int freq_count;
uint8_t test_1[4];
position_send_t car_send;
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
    
//	__HAL_TIM_SetCompare(&htim2, TIM_CHANNEL_2 , 10000);
	
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
        
//        float distance = FollowManager.FrontOpenmvFramePtr->cnt1 | (FollowManager.FrontOpenmvFramePtr->Target >> 8);
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
//        bt_update();
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

	 if (Cnt % 350 == 0)
    {
		uint8_t j;
		        static uint8_t lsif = 0;
		UART4_SEND_BUFF[0] = 'a';
		
		t.l = UWB_Manager.pos_x - start_position.x;
		if(t.l < 0)
			t.l *= -1;
		for(j = 0;j<3 ;j++)
			UART4_SEND_BUFF[j+1] = t.u[j];
		t.l = UWB_Manager.pos_y - start_position.y;
		if(t.l < 0) 
			t.l *= -1;
		for(j = 0;j<3 ;j++)
			UART4_SEND_BUFF[j+4] = t.u[j];
		t.l = fire_position.x;
		for(j = 0;j<3 ;j++)
			UART4_SEND_BUFF[j+7] = t.u[j];
		t.l = fire_position.y;
		for(j = 0;j<3 ;j++)
			UART4_SEND_BUFF[j+10] = t.u[j];
	
		UART4_SEND_BUFF[13] = 0x0d;
		UART4_SEND_BUFF[14] = 0x0a;
	
		HAL_UART_Transmit(&huart4, UART4_SEND_BUFF, 15, 2000);
		
		static uint8_t set_time=0;
		
    }
    //1Hz����
    if (Cnt % 1000 == 0)
    {
        ano_of_alt_count = 0;
        ano_of_count = 0;	
		
    }
    
    update_status();

    //LED��ѵ����
    PollingLED();
    
    //������ѵ����
    PollingUSART();
    
    beep_update();
    
    power_update();
}

/******************* (C) ��Ȩ���� 2018 �����пƺƵ�Ƽ����޹�˾ *******************/
