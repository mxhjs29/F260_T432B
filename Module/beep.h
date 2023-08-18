#ifndef _BEEP_H
#define _BEEP_H

#include "stdint.h"

#define BEEP_ON HAL_GPIO_WritePin(BEEP_GPIO_Port,BEEP_Pin,GPIO_PIN_RESET)
#define BEEP_OFF HAL_GPIO_WritePin(BEEP_GPIO_Port,BEEP_Pin,GPIO_PIN_SET)
#define BEEP_TOGGLE HAL_GPIO_TogglePin(BEEP_GPIO_Port,BEEP_Pin)

typedef enum
{
    beep_on = 0,                    //���������
    beep_off,                       //�����Ϩ�� 
    beep_flash,                     //�������˸
    beep_clockwiseFlash,            //null
}embeep_t;

typedef struct
{
    uint16_t u16FlashTime;          //��˸��ʱ,1ms�ۼ�һ��
    uint16_t beep_flash_freq;
    embeep_t   beep;
}BeepManager_t;

extern BeepManager_t BeepManager;

void beep_init(void);
void beep_update(void);


#endif
