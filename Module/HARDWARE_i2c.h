/**
  ******************************************************************************
  * Copyright (c) 2018,�����пƺƵ�Ƽ����޹�˾
  * All rights reserved.
  * �ļ����ƣ�i2c.h
  * ժ    Ҫ��
  *
  * ��ǰ�汾��V1.0
  * ��    �ߣ������пƺƵ�Ƽ����޹�˾�з��� 
  * ������ڣ�    
  * �޸�˵����
  * 
  *
  * ��ʷ�汾��
  *
  *
  *******************************************************************************/
#ifndef _I2C_H
#define _I2C_H
//�ⲿ�ļ�����
#include "include.h"

//�궨����
//PB6 SCL
//PB7 SDA

//    PB6     ------> I2C1_SCL
//    PB7     ------> I2C1_SDA 
#define IIC_SDA_In  GPIOB->MODER &= 0xFFFF3FFF;//PB7����ģʽ
#define IIC_SDA_Out GPIOB->MODER |= 0x00004000;//PB7���ģʽ
#define IIC_SDA_H   GPIOB->ODR |= GPIO_PIN_7;
#define IIC_SDA_L   GPIOB->ODR &= ~GPIO_PIN_7;
#define IIC_SCL_H   GPIOB->ODR |= GPIO_PIN_6;
#define IIC_SCL_L   GPIOB->ODR &= ~GPIO_PIN_6;

#define IIC_SDA_READ (GPIOB->IDR & GPIO_PIN_7)

//��������
uint8_t I2C_Read_Bytes(uint8_t Slaveaddr, uint8_t REG_Address, uint8_t *ptr, uint8_t length);
uint8_t I2C_Read_Byte(uint8_t Slaveaddr, uint8_t REG_Address);
void I2C_Write_Byte(uint8_t Slaveaddr, uint8_t REG_Address, uint8_t REG_data);
void I2C_Init(void);

#endif


/******************* (C) ��Ȩ���� 2018 �����пƺƵ�Ƽ����޹�˾ *******************/
