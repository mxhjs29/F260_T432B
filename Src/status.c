#include "stdint.h"
#include "include.h"
#include "gcs.h"
#include "led.h"
#include "followline.h"
#include "ano_of.h"
#include "UWB.h"

extern UAV_info_t g_UAVinfo;
void update_status()
{
    if(g_UAVinfo.FMUflg->unlock == 1)
    {
        g_LedManager.emLED_RGB_B = RGB_LED_Off;
        g_LedManager.emLED_RGB_G = RGB_LED_Off;
    }else
    {
        g_LedManager.emLED_RGB_B = RGB_LED_Off;
        g_LedManager.emLED_RGB_G = RGB_LED_Off;
    }

	if(FollowManager.GroundOpenmvFramePtr->FormType == fire)
    {
			g_LedManager.emLED_RGB_R = RGB_LED_On;
    }else
    {
        g_LedManager.emLED_RGB_R = RGB_LED_Off;
    }
}
