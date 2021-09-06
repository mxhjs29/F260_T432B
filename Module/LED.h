/**
  ******************************************************************************
  * Copyright (c) 2018,北京中科浩电科技有限公司
  * All rights reserved.
  * 文件名称：LED.h
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
#ifndef __LED_H
#define __LED_H
//外部文件引用
#include "main.h"

// 
//宏定义区
#define LED_ON(pin,port)    HAL_GPIO_WritePin(port,pin,GPIO_PIN_RESET)
#define LED_OFF(pin,port)    HAL_GPIO_WritePin(port,pin,GPIO_PIN_SET)
#define LED_TOGGLE(pin,port)    HAL_GPIO_TogglePin(port,pin)

#define LED_RGB_B_ON    LED_ON(LED_B_Pin,LED_B_GPIO_Port)
#define LED_RGB_B_OFF   LED_OFF(LED_B_Pin,LED_B_GPIO_Port)
#define LED_RGB_B_TOGGLE   LED_TOGGLE(LED_B_Pin,LED_B_GPIO_Port)

#define LED_RGB_G_ON    LED_ON(LED_G_Pin,LED_G_GPIO_Port)
#define LED_RGB_G_OFF   LED_OFF(LED_G_Pin,LED_G_GPIO_Port)
#define LED_RGB_G_TOGGLE   LED_TOGGLE(LED_G_Pin,LED_G_GPIO_Port)

#define LED_RGB_R_ON    LED_ON(LED_R_Pin,LED_R_GPIO_Port)
#define LED_RGB_R_OFF   LED_OFF(LED_R_Pin,LED_R_GPIO_Port)
#define LED_RGB_R_TOGGLE   LED_TOGGLE(LED_R_Pin,LED_R_GPIO_Port)

typedef enum
{
    RGB_LED_Toggle = 0,                   //状态灯取反
    RGB_LED_On,                       //状态灯亮起
    RGB_LED_Off,                      //状态灯熄灭    
    RGB_LED_Flash,                    //状态灯闪烁
}emRGB_LED_t;

typedef enum
{
    StatusToggle = 0,                   //状态灯取反
    StatusOn,                       //状态灯亮起
    StatusOff,                      //状态灯熄灭    
    StatusFlash,                    //状态灯闪烁
}emLEDStatus_t;

typedef enum
{
    PowerOn = 0,                        //电源灯亮起
    PowerOff,                       //电源灯熄灭  
    PowerToggle,                    //电源灯取反
    PowerFlash,                     //电源灯闪烁 
}emLEDPower_t;


//数据结构声明
typedef struct
{
    uint16_t u16FlashTime;          //闪烁计时,1ms累加一次
    
    emLEDPower_t   emLEDPower;
    emLEDStatus_t  emLEDStatus;
    emRGB_LED_t emLED_RGB_R;
    emRGB_LED_t emLED_RGB_G;
    emRGB_LED_t emLED_RGB_B;
}LedManager_t;


//Extern引用
extern LedManager_t g_LedManager;


//函数声明
void LEDInit(void);
void PollingLED(void);

#endif

/******************* (C) 版权所有 2018 北京中科浩电科技有限公司 *******************/
