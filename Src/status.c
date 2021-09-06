#include "stdint.h"
#include "include.h"
#include "gcs.h"
#include "led.h"
#include "followline.h"

extern UAV_info_t g_UAVinfo;
void update_status()
{
    if(g_UAVinfo.FMUflg->unlock == 1)
    {
        g_LedManager.emLED_RGB_R = RGB_LED_On;
        g_LedManager.emLED_RGB_G = RGB_LED_Off;
    }else
    {
        g_LedManager.emLED_RGB_R = RGB_LED_Off;
        g_LedManager.emLED_RGB_G = RGB_LED_On;
    }
    
    if(FollowManager.ActionList == ActionCountdown)
    {
        g_LedManager.emLED_RGB_B = RGB_LED_Flash;
    }else
    {
        g_LedManager.emLED_RGB_B = RGB_LED_Off;
    }
}
