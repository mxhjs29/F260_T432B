#ifndef _BEEP_H
#define _BEEP_H

#include "stdint.h"

#define BEEP_ON P3OUT |= GPIO_PIN0
#define BEEP_OFF P3OUT &= ~GPIO_PIN0
#define BEEP_TOGGLE GPIO_toggleOutputOnPin(GPIO_PORT_P3,GPIO_PIN0)

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
