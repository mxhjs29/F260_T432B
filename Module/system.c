/**
  ******************************************************************************
  * Copyright (c) 2018,北京中科浩电科技有限公司
  * All rights reserved.
  * 文件名称：kernel.c
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
循环核心驱动，使用该函数轮训就绪函数


*/
//外部文件引用
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
//宏定义区

#define KernelRunningCheck  if(!KernelRunning){ return;}

//Extern引用
extern void lcd_update(uint8_t cnt);
extern void update_status(void);
extern int ano_of_alt_count;
extern int ano_of_count;


//私有函数区
void Update(void);
void UpdateUSBQueue(void);


//私有变量区
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
  * 函数名称：KernelPolling
  * 函数描述：核心轮询程序
  * 输    入：void
  * 输    出：void
  * 返    回：void 
  * 备    注：1ms运行一次    
  *    
  *
******************************************************************************/
void KernelPolling()
{
    static uint32_t Cnt = 0;

    //核心运行检查
    KernelRunningCheck;

    KernelRunning = false;
 
    //时间段轮询计数
    Cnt++;
    
//	__HAL_TIM_SetCompare(&htim2, TIM_CHANNEL_2 , 10000);
	
    //333Hz,修改到滴答计时器里面
    if (Cnt % 3 == 0)
    {        
        //飞行PID控制器更新
        FlightPidControl(0.003f);
        
        //电机逻辑控制
        MotorControl();
    }
    
    //125Hz
    if (Cnt % 8 == 0)
    { 
        //更新欧拉角
        GetAngle(&g_Attitude);
    }

    //91Hz
    if (Cnt % 11 == 0)
    {
        //光流状态更新任务
        AnoOF_State_Task(11);
        
//        float distance = FollowManager.FrontOpenmvFramePtr->cnt1 | (FollowManager.FrontOpenmvFramePtr->Target >> 8);
    }
    
    //100Hz
    if (Cnt % 10 == 0)
    {        

        //更新巡线任务
        //警告！！！
        //此函数包含自动解锁内容，当运行此函数以后
        //按下LaunchPad板按钮会自动倒计时解锁飞机，当您不清楚解锁流程的时候
        //不要轻易解除注释
        
        //此函数为程控入口函数，当您了解程控机制后，可取消此函数注释
        UpdateCentControl(0.01f);
//        bt_update();
        sdk_update(0.01f);
		
    }
    
    //50Hz任务
    if (Cnt % 20 == 0)
    {
        //观测传感器数据计算
        WZ_Obs_Calcu(0.02f);
        
        //Z轴数据互补修正
        WZ_Fix_Calcu(0.02f);
        
        //高度控制器
        ALT_Ctrl(0.02f);        
        
        //位置控制器
        POS_Ctrl(0.02f);
    }
    
    //20Hz任务
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
    //1Hz任务
    if (Cnt % 1000 == 0)
    {
        ano_of_alt_count = 0;
        ano_of_count = 0;	
		
    }
    
    update_status();

    //LED轮训函数
    PollingLED();
    
    //串口轮训函数
    PollingUSART();
    
    beep_update();
    
    power_update();
}

/******************* (C) 版权所有 2018 北京中科浩电科技有限公司 *******************/
