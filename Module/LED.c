/**
  ******************************************************************************
  * Copyright (c) 2018,�����пƺƵ�Ƽ����޹�˾
  * All rights reserved.
  * �ļ����ƣ�LED.c
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
LED������ʹ�÷�ʽ���£�
g_LedManagerΪLED�ƿ��ƽṹ�壬Ҫ����LED����˸��ֻ��Ҫ���Ĵ˽ṹ���е�
ö��ֵ���ɡ���g_LedManager.emLed_Status
���磺
1.��Ҫ״̬��������Ҫ�������
g_LedManager.emLed_Status = StatusOn;

����ö����Դ����.h�ļ��е�emLED_Status_t

*/
//�ⲿ�ļ�����
#include "LED.h"

//�궨����
#define LED_FLASH_FREQ      100


//Extern����


//˽�к�����


//˽�б�����
LedManager_t g_LedManager;


/******************************************************************************
  * �������ƣ�LEDInit
  * ������������ʼ��LED��
  * ��    �룺void
  * ��    ����void
  * ��    �أ�void
  * ��    ע��null
  *    
  *
******************************************************************************/
void LEDInit(void)        
{
    //LED��״̬����
    g_LedManager.emLEDStatus = StatusOff;
    g_LedManager.emLEDPower = PowerOff;    
    
    g_LedManager.emLED_RGB_B = RGB_LED_Off;
    g_LedManager.emLED_RGB_R = RGB_LED_Off;
    g_LedManager.emLED_RGB_G = RGB_LED_Off;
}

/******************************************************************************
  * �������ƣ�PollingLED
  * ������������ѯ��ǰ�Ƿ���LED�����������
  * ��    �룺void
  * ��    ����void
  * ��    �أ�void
  * ��    ע��null
  *
  *
******************************************************************************/
void PollingLED()
{
    g_LedManager.u16FlashTime++;

    
    //����LED�ƹ�����״̬����LED��
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

/******************* (C) ��Ȩ���� 2018 �����пƺƵ�Ƽ����޹�˾ *******************/
