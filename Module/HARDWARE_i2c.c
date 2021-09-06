/**
  ******************************************************************************
  * Copyright (c) 2018,北京中科浩电科技有限公司
  * All rights reserved.
  * 文件名称：i2c.c
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

/*==============================================================================
                         ##### How to use this driver #####
==============================================================================
i2c初始化函数


*/
//外部文件引用
#include "HARDWARE_i2c.h"



//私有函数区
void IIC_SendByte(uint32_t dat);
void IIC_RecvACK(void);
void IIC_SendACK(uint32_t ack);
void IIC_Stop(void);
uint32_t IIC_RecvByte(void);
void IIC_Start(void);

//私有变量区
unsigned char Ack_Count = 0;

/******************************************************************************
  * 函数名称：I2C_Init
  * 函数描述：初始化I2c总线
  * 输    入：void
  * 输    出：void
  * 返    回：void 
  * 备    注：null    
  *    
  *
******************************************************************************/
void I2C_Init(void)
{
    IIC_SDA_In;
    IIC_SDA_Out;
    GPIOB->MODER &= 0xffffCfff;//PB7输出模式
    GPIOB->MODER |= 0x00001000;//PB7输出模式
}

/******************************************************************************
  * 函数名称：delay
  * 函数描述：I2c延时
  * 输    入：
  * uint32_t n:延时的周期
  * 输    出：void
  * 返    回：void 
  * 备    注：null    
  *    
  *
******************************************************************************/
int cnt = 15;
void delay(uint32_t n)
{
    n = cnt;
    while (n--)
        ;
}

/******************************************************************************
  * 函数名称：IIC_Start
  * 函数描述：I2c开始信号
  * 输    入：void
  * 输    出：void
  * 返    回：void 
  * 备    注：null    
  *    
  *
******************************************************************************/
void IIC_Start(void)
{
    IIC_SDA_Out;
    
    IIC_SDA_H;
    //    delay(1);
    ;
    IIC_SCL_H;
    delay(1);
    IIC_SDA_L;
    delay(1);
    IIC_SCL_L;
    delay(1);
}

/******************************************************************************
  * 函数名称：IIC_Stop
  * 函数描述：I2c停止信号
  * 输    入：void
  * 输    出：void
  * 返    回：void 
  * 备    注：null    
  *    
  *
******************************************************************************/
void IIC_Stop(void)
{
//    GPIOB->BSRR

    IIC_SDA_Out;
    IIC_SCL_L;
    IIC_SDA_L;
    delay(1);
    IIC_SCL_H;
    delay(1);
    IIC_SDA_H;
}

/******************************************************************************
  * 函数名称：IIC_SendACK
  * 函数描述：发送ACK
  * 输    入：void
  * 输    出：void
  * 返    回：void 
  * 备    注：null    
  *    
  *
******************************************************************************/
void IIC_SendACK(uint32_t ack)
{
    

    IIC_SDA_Out;
    IIC_SCL_L;
    delay(1);
    if (!ack)
    {
        IIC_SDA_H;
    }
    else
    {
        IIC_SDA_L;
    }
    delay(1);
    IIC_SCL_H;
    delay(1);
    IIC_SCL_L;
    delay(1);
    IIC_SDA_In; //释放总线
}

/******************************************************************************
  * 函数名称：IIC_RecvACK
  * 函数描述：接收ACk
  * 输    入：void
  * 输    出：void
  * 返    回：void 
  * 备    注：null    
  *    
  *
******************************************************************************/
void IIC_RecvACK(void)
{
    IIC_SDA_In;

    IIC_SCL_H;
    while (IIC_SDA_READ)
    {
        Ack_Count++;
        if (Ack_Count > 5)
        {
            IIC_Stop();
            Ack_Count = 0;
            break;
        }
    }

    IIC_SCL_L;
    delay(1);
}

/******************************************************************************
  * 函数名称：IIC_SendByte
  * 函数描述：I2c发送一个byte
  * 输    入：
  * uint32_t dat:要发送的数据
  * 输    出：void
  * 返    回：void 
  * 备    注：null    
  *    
  *
******************************************************************************/
void IIC_SendByte(uint32_t dat)
{
    uint8_t i;

    IIC_SDA_Out;
    i = 8;
    //  IIC_SCL_L;
    //  delay(1);
    while (i--)
    {
        if ((dat & 0x80))
        {
            IIC_SDA_H;
        }
        else
        {
            IIC_SDA_L;
        }
        dat <<= 1;
        delay(1);
        IIC_SCL_H;
        delay(1);
        IIC_SCL_L;
        delay(1);
    }
    IIC_SDA_In; //释放总线
    //  IIC |= (1 << SDA);
    //  IIC_RecvACK();
}

/******************************************************************************
  * 函数名称：IIC_RecvByte
  * 函数描述：I2c接收一个byte
  * 输    入：void
  * 输    出：void
  * 返    回：接收到的数据
  * 备    注：null    
  *    
  *
******************************************************************************/
uint32_t IIC_RecvByte()
{
    uint8_t i;
    uint8_t dat = 0;

    IIC_SDA_In;
    i = 8;
    while (i--)
    {
        //    IIC_SCL_L;
        //    delay(1);
        IIC_SCL_H;
        dat <<= 1;
        if (IIC_SDA_READ)
        {
            dat |= 1;
        }
        else
        {
            dat |= 0;
        }
        IIC_SCL_L;
        delay(1);
    }
    return dat;
}

/******************************************************************************
  * 函数名称：I2C_Write_Byte
  * 函数描述：I2c往从机寄存器地址写数据
  * 输    入：
  * uint32_t Slaveaddr:从机地址
  * uint32_t REG_Address:寄存器地址
  * uint32_t REG_data：寄存器数据
  * 输    出：void
  * 返    回：void 
  * 备    注：null    
  *    
  *
******************************************************************************/
void I2C_Write_Byte(uint8_t Slaveaddr, uint8_t REG_Address, uint8_t REG_data)
{
    IIC_Start();
    IIC_SendByte(Slaveaddr);
    IIC_RecvACK();
    IIC_SendByte(REG_Address);
    IIC_RecvACK();
    IIC_SendByte(REG_data);
    IIC_RecvACK();
    IIC_Stop();
}

/******************************************************************************
  * 函数名称：I2C_Read_Byte
  * 函数描述：I2c往从机寄存器读数据
  * 输    入：
  * uint32_t Slaveaddr：从机地址
  * uint32_t REG_Address:寄存器地址
  * 输    出：void
  * 返    回：读取到的数据
  * 备    注：null    
  *    
  *
******************************************************************************/
uint8_t I2C_Read_Byte(uint8_t Slaveaddr, uint8_t REG_Address)
{
    uint32_t REG_data;
    IIC_Start();
    IIC_SendByte(Slaveaddr);
    IIC_RecvACK();
    IIC_SendByte(REG_Address);
    IIC_RecvACK();
    IIC_Start();
    IIC_SendByte(Slaveaddr + 1);
    IIC_RecvACK();
    REG_data = IIC_RecvByte();
    IIC_SendACK(0);
    IIC_Stop();
    return REG_data;
}

uint8_t I2C_Read_Bytes(uint8_t Slaveaddr, uint8_t REG_Address, uint8_t *ptr, uint8_t len)
{
    IIC_Start();
    IIC_SendByte(Slaveaddr);
    IIC_RecvACK();
    IIC_SendByte(REG_Address);
    IIC_RecvACK();
    IIC_Start();
    IIC_SendByte(Slaveaddr + 1);
    IIC_RecvACK();
    while (len)
    {
        if (len == 1)
        {
            *ptr = IIC_RecvByte();
            IIC_SendACK(0);
        }
        else
        {
            *ptr = IIC_RecvByte();
            IIC_SendACK(1);
        }
        ptr++;
        len--;
    }
    IIC_Stop();
    return 0;
}

/******************* (C) 版权所有 2018 北京中科浩电科技有限公司 *******************/
