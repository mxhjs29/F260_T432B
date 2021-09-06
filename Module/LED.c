/**
  ******************************************************************************
  * Copyright (c) 2018,北京中科浩电科技有限公司
  * All rights reserved.
  * 文件名称：LED.c
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
LED灯驱动使用方式如下：
g_LedManager为LED灯控制结构体，要控制LED的闪烁，只需要更改此结构体中的
枚举值即可——g_LedManager.emLed_Status
例如：
1.想要状态灯亮起，需要以下语句
g_LedManager.emLed_Status = StatusOn;

其他枚举量源自于.h文件中的emLED_Status_t

*/
//外部文件引用
#include "LED.h"

//宏定义区
#define LED_FLASH_FREQ      100


//Extern引用


//私有函数区


//私有变量区
LedManager_t g_LedManager;


/******************************************************************************
  * 函数名称：LEDInit
  * 函数描述：初始化LED灯
  * 输    入：void
  * 输    出：void
  * 返    回：void
  * 备    注：null
  *    
  *
******************************************************************************/
void LEDInit(void)        
{
    //LED灯状态配置
    g_LedManager.emLEDStatus = StatusOff;
    g_LedManager.emLEDPower = PowerOff;    
    
    g_LedManager.emLED_RGB_B = RGB_LED_Off;
    g_LedManager.emLED_RGB_R = RGB_LED_Off;
    g_LedManager.emLED_RGB_G = RGB_LED_Off;
}

/******************************************************************************
  * 函数名称：PollingLED
  * 函数描述：轮询当前是否有LED亮起任务就绪
  * 输    入：void
  * 输    出：void
  * 返    回：void
  * 备    注：null
  *
  *
******************************************************************************/
void PollingLED()
{
    g_LedManager.u16FlashTime++;

    
    //根据LED灯管理器状态配置LED灯
    switch(g_LedManager.emLED_RGB_B)
    {
        case RGB_LED_On:
            LED_RGB_B_ON;
            break;
        case RGB_LED_Flash:
            if(g_LedManager.u16FlashTime % LED_FLASH_FREQ == 0)
            {
                LED_RGB_B_TOGGLE;
            }
            break;
        case RGB_LED_Off:
            LED_RGB_B_OFF;
            break;
        case RGB_LED_Toggle:
            LED_RGB_B_TOGGLE;
            break;
        default:
            break;
    }
    
    switch(g_LedManager.emLED_RGB_R)
    {
        case RGB_LED_On:
            LED_RGB_R_ON;
            break;
        case RGB_LED_Flash:
            if(g_LedManager.u16FlashTime % LED_FLASH_FREQ == 0)
            {
                LED_RGB_R_TOGGLE;
            }
            break;
        case RGB_LED_Off:
            LED_RGB_R_OFF;
            break;
        case RGB_LED_Toggle:
            LED_RGB_R_TOGGLE;
            break;
        default:
            break;
    }
    
    switch(g_LedManager.emLED_RGB_G)
    {
        case RGB_LED_On:
            LED_RGB_G_ON;
            break;
        case RGB_LED_Flash:
            if(g_LedManager.u16FlashTime % LED_FLASH_FREQ == 0)
            {
                LED_RGB_G_TOGGLE;
            }
            break;
        case RGB_LED_Off:
            LED_RGB_G_OFF;
            break;
        case RGB_LED_Toggle:
            LED_RGB_G_TOGGLE;
            break;
        default:
            break;
    }
    
}

/******************* (C) 版权所有 2018 北京中科浩电科技有限公司 *******************/
