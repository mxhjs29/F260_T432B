/**
  ******************************************************************************
  * Copyright (c) 2018,�����пƺƵ�Ƽ����޹�˾
  * All rights reserved.
  * �ļ����ƣ�LED.h
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
#ifndef __LED_H
#define __LED_H
//�ⲿ�ļ�����
#include "main.h"

// 
//�궨����
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
    RGB_LED_Toggle = 0,                   //״̬��ȡ��
    RGB_LED_On,                       //״̬������
    RGB_LED_Off,                      //״̬��Ϩ��    
    RGB_LED_Flash,                    //״̬����˸
}emRGB_LED_t;

typedef enum
{
    StatusToggle = 0,                   //״̬��ȡ��
    StatusOn,                       //״̬������
    StatusOff,                      //״̬��Ϩ��    
    StatusFlash,                    //״̬����˸
}emLEDStatus_t;

typedef enum
{
    PowerOn = 0,                        //��Դ������
    PowerOff,                       //��Դ��Ϩ��  
    PowerToggle,                    //��Դ��ȡ��
    PowerFlash,                     //��Դ����˸ 
}emLEDPower_t;


//���ݽṹ����
typedef struct
{
    uint16_t u16FlashTime;          //��˸��ʱ,1ms�ۼ�һ��
    
    emLEDPower_t   emLEDPower;
    emLEDStatus_t  emLEDStatus;
    emRGB_LED_t emLED_RGB_R;
    emRGB_LED_t emLED_RGB_G;
    emRGB_LED_t emLED_RGB_B;
}LedManager_t;


//Extern����
extern LedManager_t g_LedManager;


//��������
void LEDInit(void);
void PollingLED(void);

#endif

/******************* (C) ��Ȩ���� 2018 �����пƺƵ�Ƽ����޹�˾ *******************/
