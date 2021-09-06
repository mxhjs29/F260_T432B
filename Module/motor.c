/**
  ******************************************************************************
  * Copyright (c) 2018,北京中科浩电科技有限公司
  * All rights reserved.
  * 文件名称：motor.c
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
电机驱动调用方式如下：
1.调用函数UpdateMotor直接驱动电机；
电机PWM初始化已经在main函数中完成。


*/
//外部文件引用
#include "motor.h"
#include "myMath.h"
#include "tim.h"
//#include "timer_drv.h"
//#include "eeprom.h"
//宏定义区


//Extern引用


//私有函数区
void Motor_Calc(void);

//私有变量区

/******************************************************************************
  * 函数名称：UpdateMotor
  * 函数描述：驱动电机转动，电机输出取值0-1000
  * 输    入：
  * int16_t M1:电机1输出取值
  * int16_t M2:电机2输出取值
  * int16_t M3:电机3输出取值
  * int16_t M4:电机4输出取值
  * 输    出：void
  * 返    回：void
  * 备    注：null
  *    
  *
******************************************************************************/
void UpdateMotor(int16_t M1, int16_t M2, int16_t M3, int16_t M4)
{
    M1 = LIMIT(M1, 0, 999);   //电机取值限幅 0-999
    M2 = LIMIT(M2, 0, 999);
    M3 = LIMIT(M3, 0, 999);
    M4 = LIMIT(M4, 0, 999);

    TIM1->CCR1 = M3 + 1000;
    TIM1->CCR2 = M1 + 1000;
    TIM1->CCR3 = M2 + 1000;
    TIM1->CCR4 = M4 + 1000;
}

/******************************************************************************
  * 函数名称：Motor_Init
  * 函数描述：初始化PWM波输出
  * 输    入：void
  * 输    出：void
  * 返    回：void
  * 备    注：null
  *    950 = 5%
  *    2000 = 10%
******************************************************************************/
void Motor_Init()
{
    HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);
    HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_2);
    HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_3);
    HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_4);
    
//    HAL_Delay(3500);


//    Motor_Calc();
}

void Motor_Unlock()
{

}

void Motor_Lock()
{
    
}

void Motor_Calc()
{
    uint8_t CalcCompleateFlag = 0;
    UpdateMotor(1000,1000,1000,1000);
    HAL_Delay(2500);
    UpdateMotor(00,00,00,00);
    HAL_Delay(2500);
}

/******************* (C) 版权所有 2018 北京中科浩电科技有限公司 *******************/

