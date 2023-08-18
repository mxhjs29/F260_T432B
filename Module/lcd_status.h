#ifndef _LCD_STATUS_H
#define _LCD_STATUS_H

#include "stdint.h"
#include "stdbool.h"

char FollowLine_ActionList_str[20][30] = 
{
    {"ActionWaitting"},
    {"ActionCountdown"},
    {"ActionTakeOff"},
    {"ActionHoverStartPoint"},
    {"ActionGoForward"},
    {"ActionGoRight"},
    {"ActionGoLeft"},
    {"ActionGoBack"},
    {"ActionGoRound"},
    {"ActionGoForward2"},
    {"ActionFindPole"},
    {"ActionCloseToPole"},
    {"ActionTurnRound"},
    {"ActionFindLandSpace"},
    {"ActionResetAngle"},
    {"ActionPreLand"},
    {"ActionLand"},
    {"ActionLock"},
};

typedef enum
{
    page1 = 0,
    page2,
    page3,
    page4,
    
    len_page,
}lcd_page_t;

typedef enum
{
    lcd_message_0 = 0,
    lcd_message_1,
    lcd_message_2,
    lcd_message_3,
    lcd_message_4,
    
    len_message,
}lcd_message_t;

typedef struct
{
    lcd_message_t lcd_message;
    char message_info[20];
}lcd_message_info_t;

typedef struct
{
    lcd_page_t lcd_page;
    lcd_message_info_t message[len_message];
}lcd_page_info_t;

typedef struct
{
    lcd_page_info_t lcd_page_info[len_page];
    lcd_page_t current_page;
    
    int Y_offset;
    int back_color;
    int font_color_unselect;
    int font_color_select;
    
}lcd_manager_t;

extern lcd_manager_t lcd_manager;

void lcd_message(lcd_message_info_t *msg_select, bool select_enable,uint8_t *msg, bool data_enable,int data);
//void lcd_message(lcd_message_t msg_select, bool select_enable,char *msg, bool data_enable,int data);
void lcd_exchange_page(void);
void lcd_status_init(void);

#endif
