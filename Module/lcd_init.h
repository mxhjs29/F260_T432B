#ifndef __LCD_INIT_H
#define __LCD_INIT_H

#include "main.h"

#define USE_HORIZONTAL 2  //设置横屏或者竖屏显示 0或1为竖屏 2或3为横屏


#if USE_HORIZONTAL==0||USE_HORIZONTAL==1
#define LCD_W 80
#define LCD_H 160

#else
#define LCD_W 160
#define LCD_H 80
#endif


//-----------------LCD端口定义---------------- 

#define LCD_CLR(port,pin)    HAL_GPIO_WritePin(port,pin,GPIO_PIN_RESET)
#define LCD_SET(port,pin)    HAL_GPIO_WritePin(port,pin,GPIO_PIN_SET)

//PB4
#define LCD_SCLK_Clr() LCD_CLR(GPIOB,GPIO_PIN_4)//SCL=SCLK
#define LCD_SCLK_Set() LCD_SET(GPIOB,GPIO_PIN_4)

//PB3
#define LCD_MOSI_Clr() LCD_CLR(GPIOB,GPIO_PIN_3)//SDA=MOSI
#define LCD_MOSI_Set() LCD_SET(GPIOB,GPIO_PIN_3)

//PD6
#define LCD_RES_Clr()  LCD_CLR(GPIOD,GPIO_PIN_6)//RES
#define LCD_RES_Set()  LCD_SET(GPIOD,GPIO_PIN_6)

//PD7
#define LCD_DC_Clr()   LCD_CLR(GPIOD,GPIO_PIN_7)//DC
#define LCD_DC_Set()   LCD_SET(GPIOD,GPIO_PIN_7)
 		     
//PB5
#define LCD_CS_Clr()   LCD_CLR(GPIOB,GPIO_PIN_5)//CS
#define LCD_CS_Set()   LCD_SET(GPIOB,GPIO_PIN_5)

//PD4
#define LCD_BLK_Clr()  LCD_CLR(GPIOD,GPIO_PIN_4)//BLK
#define LCD_BLK_Set()  LCD_SET(GPIOD,GPIO_PIN_4)

void LCD_GPIO_Init(void);//初始化GPIO
void LCD_Writ_Bus(uint8_t dat);//模拟SPI时序
void LCD_WR_DATA8(uint8_t dat);//写入一个字节
void LCD_WR_DATA(uint16_t dat);//写入两个字节
void LCD_WR_REG(uint8_t dat);//写入一个指令
void LCD_Address_Set(uint16_t x1,uint16_t y1,uint16_t x2,uint16_t y2);//设置坐标函数
void LCD_Init(void);//LCD初始化
#endif




