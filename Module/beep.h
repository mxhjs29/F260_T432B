#ifndef _BEEP_H
#define _BEEP_H

#include "stdint.h"

#define BEEP_ON HAL_GPIO_WritePin(BEEP_GPIO_Port,BEEP_Pin,GPIO_PIN_RESET)
#define BEEP_OFF HAL_GPIO_WritePin(BEEP_GPIO_Port,BEEP_Pin,GPIO_PIN_SET)
#define BEEP_TOGGLE HAL_GPIO_TogglePin(BEEP_GPIO_Port,BEEP_Pin)

typedef enum
{
    beep_on = 0,                    //电机灯亮起
    beep_off,                       //电机灯熄灭 
    beep_flash,                     //电机灯闪烁
    beep_clockwiseFlash,            //null
}embeep_t;

typedef struct
{
    uint16_t u16FlashTime;          //闪烁计时,1ms累加一次
    uint16_t beep_flash_freq;
    embeep_t   beep;
}BeepManager_t;

extern BeepManager_t BeepManager;

void beep_init(void);
void beep_update(void);


#endif
