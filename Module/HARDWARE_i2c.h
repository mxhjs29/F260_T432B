/**
  ******************************************************************************
  * Copyright (c) 2018,北京中科浩电科技有限公司
  * All rights reserved.
  * 文件名称：i2c.h
  * 摘    要：
  *
  * 当前版本：V1.0
  * 作    者：北京中科浩电科技有限公司研发部 
  * 完成日期：    
  * 修改说明：
  * 
  *
  * 历史版本：
  *
  *
  *******************************************************************************/
#ifndef _I2C_H
#define _I2C_H
//外部文件引用
#include "include.h"

//宏定义区
//PB6 SCL
//PB7 SDA

//    PB6     ------> I2C1_SCL
//    PB7     ------> I2C1_SDA 
#define IIC_SDA_In  GPIOB->MODER &= 0xFFFF3FFF;//PB7输入模式
#define IIC_SDA_Out GPIOB->MODER |= 0x00004000;//PB7输出模式
#define IIC_SDA_H   GPIOB->ODR |= GPIO_PIN_7;
#define IIC_SDA_L   GPIOB->ODR &= ~GPIO_PIN_7;
#define IIC_SCL_H   GPIOB->ODR |= GPIO_PIN_6;
#define IIC_SCL_L   GPIOB->ODR &= ~GPIO_PIN_6;

#define IIC_SDA_READ (GPIOB->IDR & GPIO_PIN_7)

//函数声明
uint8_t I2C_Read_Bytes(uint8_t Slaveaddr, uint8_t REG_Address, uint8_t *ptr, uint8_t length);
uint8_t I2C_Read_Byte(uint8_t Slaveaddr, uint8_t REG_Address);
void I2C_Write_Byte(uint8_t Slaveaddr, uint8_t REG_Address, uint8_t REG_data);
void I2C_Init(void);

#endif


/******************* (C) 版权所有 2018 北京中科浩电科技有限公司 *******************/
