#include "stdint.h"
#include "lcd.h"
#include "mpu6050.h"
#include "ano_of.h"
#include "followline.h"

bool is_system_init = false;
int count = 0;
void lcd_update(uint8_t cnt)
{
    int Y_offset = 15;
    
    if(g_UAVinfo.FMUflg->unlock == 1 || FollowManager.ActionList == ActionCountdown)
    {
        return ;
    }
    
    count++;

//    LCD_Fill(0,0,160,180,GRAY);
    
    if(g_MPUManager.Check)
    {
      LCD_ShowString(5,5,"MPU On-line",LIGHTBLUE,GRAY,12,0);
    }else if(g_MPUManager.initing)
    {
      LCD_ShowString(5,5,"MPU Initializing",LIGHTBLUE,GRAY,12,0);
    }else
    {
        LCD_ShowString(5,5,"MPU Off-line",LGRAY,GRAY,12,0);
    }

    if(ANO_OF.QUALITY == 0)
    {
      LCD_ShowString(5,5 + Y_offset,"ANO_OF QUAT:",LGRAY,GRAY,12,0);
      LCD_ShowIntNum(5 + 12 * 6 + 5,5 + Y_offset,ANO_OF.QUALITY,3,LGRAY,GRAY,12,0);
    }else
    {
      LCD_ShowString(5,5 + Y_offset,"ANO_OF QUAT:",LIGHTBLUE,GRAY,12,0);
      LCD_ShowIntNum(5 + 12 * 6 + 5,5 + Y_offset,ANO_OF.QUALITY,3,LIGHTBLUE,GRAY,12,0);
    }
    
    //OpenMVÐÅÏ¢
    if(FollowManager.ptrFrame->FormType == 0xFF)
    {
        LCD_ShowString(5,5 + Y_offset * 2,"OpenMVG found nothing",LIGHTBLUE,GRAY,12,0);
    }else if(FollowManager.ptrFrame->FormType == ApriTag)
    {
        LCD_ShowString(5,5 + Y_offset * 2,"OpenMVG : AprilTag",LIGHTBLUE,GRAY,12,0);
    }else if(FollowManager.ptrFrame->Start != 0xAAAA)
    {
        LCD_ShowString(5,5 + Y_offset * 2,"OpenMVG Off-line",LGRAY,GRAY,12,0);
    }
}
