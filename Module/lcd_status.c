#include "stdint.h"
#include "lcd.h"
#include "mpu6050.h"
#include "ano_of.h"
#include "followline.h"
#include "lcd_status.h"
#include "string.h"
#include "HARDWARE_uart.h"
//void lcd_exchange_page();
void flush_lcd(void);
lcd_manager_t lcd_manager;
lcd_message_info_t temp_;
void lcd_update(uint8_t cnt)
{    
    static bool _action_countdown = true;
    
    if(g_UAVinfo.FMUflg->unlock == 1)
    {
        return ;
    }
    
    if(FollowManager.ActionList == ActionCountdown)
    {
        if(_action_countdown)
        {
            _action_countdown = false;
            lcd_message(&temp_, true,(uint8_t*)FollowLine_ActionList_str[(uint8_t)FollowManager.ActionList], false, 0);
        }
        
        return ;
    }
    
    _action_countdown = true;
    
    lcd_message(&temp_, true,(uint8_t*)FollowLine_ActionList_str[(uint8_t)FollowManager.ActionList], false, 0);
    
    if((GPIOE->IDR & GPIO_PIN_3) == 0)
    {
        while((GPIOE->IDR & GPIO_PIN_3) == 0);
        
        lcd_exchange_page();
    }
    
    switch(lcd_manager.current_page)
    {
        //快速检查页面
        case page1:
            flush_lcd();

            if(g_MPUManager.Check)
            {
                lcd_message(&lcd_manager.lcd_page_info[page1].message[lcd_message_0], true,"MPU On-line", false, 0);
            }else if(g_MPUManager.initing)
            {
                lcd_message(&lcd_manager.lcd_page_info[page1].message[lcd_message_0], true,"MPU Initializing", false, 0);
            }else
            {
                lcd_message(&lcd_manager.lcd_page_info[page1].message[lcd_message_0], false,"MPU Off-line", false, 0);
            }
            
            if(ANO_OF.QUALITY == 0)
            {
                lcd_message(&lcd_manager.lcd_page_info[page1].message[lcd_message_1], false,"ANO_OF Off-line", false, ANO_OF.QUALITY);
            }else
            {
                lcd_message(&lcd_manager.lcd_page_info[page1].message[lcd_message_1], true,"ANO_OF QUAT:", true, ANO_OF.QUALITY);
            }
            
            if(FollowManager.ptrFrame->FormType == 0xFF)
            {
                lcd_message(&lcd_manager.lcd_page_info[page1].message[lcd_message_2], true,(uint8_t*)"OpenMVG found nothing", false, 0);
//                LCD_ShowString(5,5 + Y_offset * 2,"OpenMVG found nothing",LIGHTBLUE,GRAY,12,0);
            }else if(FollowManager.ptrFrame->FormType == ApriTag)
            {
//                LCD_ShowString(5,5 + Y_offset * 2,"OpenMVG : AprilTag",LIGHTBLUE,GRAY,12,0);
                lcd_message(&lcd_manager.lcd_page_info[page1].message[lcd_message_2], true,"OpenMVG : AprilTag", false, 0);
            }else if((uint16_t)FollowManager.ptrFrame->Start != 0xAAAA)
            {
//                LCD_ShowString(5,5 + Y_offset * 2,"OpenMVG Off-line",LGRAY,GRAY,12,0);
                lcd_message(&lcd_manager.lcd_page_info[page1].message[lcd_message_2], false,"OpenMVG Off-line", false, 0);

            }
            break;
        case page2:
            flush_lcd();
            
            if((uint16_t)FollowManager.FrontOpenmvFramePtr->Start != 0xAAAA || (uint16_t)FollowManager.GroundOpenmvFramePtr->Start == 0xAAAA)
            {
                lcd_message(&lcd_manager.lcd_page_info[page2].message[lcd_message_0], true,"OpenMV Front", false, 0);
            }else
            {
                lcd_message(&lcd_manager.lcd_page_info[page2].message[lcd_message_0], false,"OpenMV Front", false, 0);
            }
            
            if(FollowManager.FrontOpenmvFramePtr->FormType == 0xFF)
            {
                lcd_message(&lcd_manager.lcd_page_info[page1].message[lcd_message_1], true,(uint8_t*)"OpenMVF found nothing", false, 0);
//                LCD_ShowString(5,5 + Y_offset * 2,"OpenMVG found nothing",LIGHTBLUE,GRAY,12,0);
            }else if(FollowManager.FrontOpenmvFramePtr->FormType == Pole)
            {
//                LCD_ShowString(5,5 + Y_offset * 2,"OpenMVG : AprilTag",LIGHTBLUE,GRAY,12,0);
                lcd_message(&lcd_manager.lcd_page_info[page1].message[lcd_message_1], true,"OpenMVF : Pole", false, 0);
            }else
            {
                lcd_message(&lcd_manager.lcd_page_info[page1].message[lcd_message_1], false,"OpenMVF Off-line", false, 0);
            }
            
            
            if((uint16_t)FollowManager.GroundOpenmvFramePtr->Start == 0xAAAA)
            {
                lcd_message(&lcd_manager.lcd_page_info[page2].message[lcd_message_2], true,"OpenMV Ground", false, 0);
            }else
            {
                lcd_message(&lcd_manager.lcd_page_info[page2].message[lcd_message_2], true,"OpenMV Ground", false, 0);
            }
            
            if(FollowManager.GroundOpenmvFramePtr->FormType == 0xFF)
            {
                lcd_message(&lcd_manager.lcd_page_info[page1].message[lcd_message_3], true,(uint8_t*)"OpenMVG found nothing", false, 0);
//                LCD_ShowString(5,5 + Y_offset * 2,"OpenMVG found nothing",LIGHTBLUE,GRAY,12,0);
            }else if(FollowManager.FrontOpenmvFramePtr->FormType == Cirle)
            {
//                LCD_ShowString(5,5 + Y_offset * 2,"OpenMVG : AprilTag",LIGHTBLUE,GRAY,12,0);
                lcd_message(&lcd_manager.lcd_page_info[page1].message[lcd_message_3], true,"OpenMVG : Cirle", false, 0);
            }else
            {
                lcd_message(&lcd_manager.lcd_page_info[page1].message[lcd_message_3], false,"OpenMVG Off-line", false, 0);
            }

            break;
        case page3:
            
            break;
        case page4:
            
            break;
        default:
            break;
    }
    
}

void lcd_status_init(void)
{
    lcd_manager.Y_offset = 15;
    
    lcd_manager.back_color = GRAY;
    lcd_manager.font_color_select = LIGHTBLUE;
    lcd_manager.font_color_unselect = LGRAY;
    temp_.lcd_message = lcd_message_4;
    for(int i = 0;i < len_page;i++)
    {
        for(int p = 0;p<len_message;p++)
        {
            lcd_manager.lcd_page_info[i].lcd_page = i;
            lcd_manager.lcd_page_info[i].message[p].lcd_message = p;
        }
        
    }
}

void lcd_message(lcd_message_info_t *msg_select, bool select_enable,uint8_t *msg, bool data_enable,int data)
{
    if(select_enable)
    {
        LCD_ShowString(5,5 + lcd_manager.Y_offset * (uint8_t)msg_select->lcd_message,
                        msg,lcd_manager.font_color_select,lcd_manager.back_color,12,0);
    }else
    {
        LCD_ShowString(5,5 + lcd_manager.Y_offset * (uint8_t)msg_select->lcd_message,
                        msg,lcd_manager.font_color_unselect,lcd_manager.back_color,12,0);
    }
    
    if(data_enable)
    {
        LCD_ShowIntNum(100,5 + lcd_manager.Y_offset * (uint8_t)msg_select->lcd_message,
                        data,3,lcd_manager.font_color_select,lcd_manager.back_color,12,0);
    }

    
}

void lcd_exchange_page()
{
    lcd_manager.current_page += 1;
    
    if(lcd_manager.current_page > len_page)
    {
        lcd_manager.current_page = 0;
    }
}

void flush_lcd()
{
    static lcd_page_t last_page;
    
    if(last_page != lcd_manager.current_page)
    {
        LCD_Fill(0,0,160,180,lcd_manager.back_color);
    }
    
    last_page = lcd_manager.current_page;
}















