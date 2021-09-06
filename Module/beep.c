#include "beep.h"
#include "main.h"

BeepManager_t BeepManager;

void beep_init()
{
    BeepManager.beep_flash_freq = 200;
    BeepManager.beep = beep_on;
}

void beep_update()
{
    BeepManager.u16FlashTime++;
    
    switch(BeepManager.beep)
    {
        case beep_on:
            BEEP_ON;
            break;
       
        case beep_flash:
            if(BeepManager.u16FlashTime % BeepManager.beep_flash_freq == 0)
            {
                BEEP_TOGGLE;
            }
            break;
        case beep_off:
            BEEP_OFF;
            break;
        default:
            break;
    }
}
