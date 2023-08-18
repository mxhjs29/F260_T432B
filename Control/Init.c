/**
  ******************************************************************************
  * Copyright (c) 2018,�����пƺƵ�Ƽ����޹�˾
  * All rights reserved.
  * �ļ����ƣ�Init.c
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
Ӳ����ʼ������ʹ��ʱֻ��Ҫ����Hadrware_Init����


*/
//�ⲿ�ļ�����
#include "include.h"
#include "gcs.h"
#include "led.h"
//#include "spl06.h"
//#include "communication.h"
//#include "battery.h"
#include "FollowLine.h"
#include "server.h"
#include "sdk.h"
//#include "timer_drv.h"
#include "power.h"
#include "ano_of.h"
#include "motor.h"
#include "lcd_init.h"
#include "lcd.h"
#include "lcd_pic.h"
#include "HARDWARE_i2c.h"
#include "HARDWARE_uart.h"
#include "tim.h"
//�궨����
#define TEST_NUM 1 

#define HARDWARE_CHECK_LED    LED_STATUS_ON;LED_POWER_ON;Delay_ms(100);\
                              LED_STATUS_OFF;LED_POWER_OFF;Delay_ms(100);\
                              LED_STATUS_ON;LED_POWER_ON;Delay_ms(100);\
                              LED_STATUS_OFF;LED_POWER_OFF;Delay_ms(100);
#ifdef STM32
#define HARDWARE_CHECK        g_MPUManager.Check && \
                              g_SPL06Manager.Check && \
                              g_NRFManager.Check
#endif
//Extern����
bool InitComplete = false;
extern _ano_of_st ANO_OF;

uint8_t test[] = "test";
//˽�к�����
void PID_Init(void); 

//˽�б�����

uint32_t SysTick_count; //ϵͳʱ�����
queue_t USB_Send_Queue;
/******************************************************************************
  * �������ƣ�Hadrware_Init
  * ������������ʼ������Ӳ���Ͳ���
  * ��    �룺void
  * ��    ����void
  * ��    �أ�void 
  * ��    ע��null    
  *    
  *
******************************************************************************/
 void Hadrware_Init(void)
{
    USART_Init();
    Motor_Init();
    //POWER����
//    SPL06Init();                    //SPL06��ʼ��
    PID_Init();                     //PID������ʼ��
//    server_init();                  //�����ʼ��
    Follow_Init();                  //Ѳ�߿�������ʼ��
    gcs_init();                     //����վ��Ϣ��ʼ��
    Remote_init();                  //ң������ʼ��
    sdk_init();                     //SDK��ʼ��
    MPU6050Init();
    LEDInit();                      //LED���Ƴ�ʼ��
    HAL_TIM_Base_Start_IT(&htim4);
    LCD_Init();
    LCD_Fill(0,0,160,180,GRAY);
    
    //�ж�MPU6050��SPL06��ʼ���Ƿ�ɹ�
    InitComplete = true;
    
}

/******************************************************************************
  * �������ƣ�PID_Init
  * ������������ʼ������PID����
  * ��    �룺void
  * ��    ����void
  * ��    �أ�void 
  * ��    ע��null
  *
  *
******************************************************************************/
void PID_Init(void)
{
    PIDGroup[emPID_Pitch_Pos].kp   = 8.0f;
    PIDGroup[emPID_Pitch_Spd].kp   = 1.1f;
    PIDGroup[emPID_Pitch_Spd].ki   = 1.1f;
    PIDGroup[emPID_Pitch_Spd].kd   = 0.06f;
    PIDGroup[emPID_Pitch_Spd].IntegLimitHigh = 10;
    PIDGroup[emPID_Pitch_Spd].IntegLimitLow = -10;

    PIDGroup[emPID_Roll_Pos].kp    = 8.0f;
    PIDGroup[emPID_Roll_Spd].kp    = 1.1f;
    PIDGroup[emPID_Roll_Spd].ki    = 1.0f;
    PIDGroup[emPID_Roll_Spd].kd    = 0.04f;
    PIDGroup[emPID_Roll_Spd].IntegLimitHigh = 10;
    PIDGroup[emPID_Roll_Spd].IntegLimitLow = -10;

    PIDGroup[emPID_Yaw_Pos].kp     = 8.0f;
    PIDGroup[emPID_Yaw_Spd].kp     = 2.0f;
    PIDGroup[emPID_Yaw_Spd].kd     = 0.00f;
    PIDGroup[emPID_Yaw_Spd].OutLimitHigh = 100;
    PIDGroup[emPID_Yaw_Spd].OutLimitLow = -100;

    PIDGroup[emPID_Roll_Spd].IntegLimitHigh = 50; 
    PIDGroup[emPID_Roll_Spd].IntegLimitLow = -50;
}

/******************* (C) ��Ȩ���� 2018 �����пƺƵ�Ƽ����޹�˾ *******************/
