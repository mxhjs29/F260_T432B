/**
  ******************************************************************************
  * Copyright (c) 2018,北京中科浩电科技有限公司
  * All rights reserved.
  * 文件名称：fmuConfig.h
  * 摘    要：
  *
  * 当前版本：V1.0
  * 作    者：北京中科浩电科技有限公司研发部 
  * 完成日期：    
  * 修改说明：
  * 
  *
  * 历史版本：
  *         固件号：100
  *         原始版本
  *
  *         固件号：101
  *         更新：
  *         1.支持修改发送接收地址；
  *         2.将收发信息改为USB接口
  *
  *         固件号：102
  *         更新：
  *         1.支持定高功能；
  *         2.支持大桨叶；
  *         3.更新LED显示；
  *         4.更新YAW轴控制；
  *         5.更新定高下放防误触功能；
  *         
  *******************************************************************************/
#ifndef __FMU_CONFIG_H
#define __FMU_CONFIG_H
//外部文件引用
#include "main.h"


//宏定义区
/*开启相关硬件*/
//设备选择区
#define ZKHD_HAWK_I2C_ENABLE
#define ZKHD_HAWK_SPI_ENABLE
#define ZKHD_HAWK_USART1_ENABLE
#define ZKHD_HAWK_USART2_ENABLE
#define ZKHD_HAWK_MOTOR_LED_ENABLE
#define ZKHD_HAWK_RGB_LED_ENABLE
#define ZKHD_HAWK_ADC_ENABLE
#define ZKHD_HAWK_FLASH_ENABLE
#define ZKHD_HAWK_GPIO_ENABLE

/*设备总线选择定义*/
#ifdef HARDWARE_NRF24L01
#define NRF_RW  Hardware_SPI1_RW
#define SET_NRF_CSN                 Hardware_SPI1_Device1_CSN_H
#define CLR_NRF_CSN                 Hardware_SPI1_Device1_CSN_L
#define NRF24L01CE_H                GPIO_Pin1_Set(1)
#define NRF24L01CE_L                GPIO_Pin1_Set(0)
#define READ_NRF24L01_IRQ           (GPIOA->IDR&GPIO_PIN_15)          //IRQ主机数据输入 PA15
#endif

/*主芯片选择*/
//#define MSP432
//#define STM32
#ifdef MSP432
#define RADIO
#define SBUS

#endif

//#define STM32
#ifdef STM32
#deine NRF24L01

#endif


#define DEBUG_PORT             A2
#define DEBUG
#define FIRMWARE_INFO          102

/*高度控制常量*/

#define MAX_ISFD_ATTITUDE      40
#define MIN_ALT_CM             50
#define MAX_ALT_CM             200
#define MAX_ALT_RATE           0.05
#define FIX_ALT_RATE           20
#define MAX_REMOTE_THROTTLE    2000
#define THROTTLE_DEAD_ZONE     0.2f

#define MOTOR_MAX_INIT_VALUE   850
#define MOTOR_MAX_VALUE   1000
#define NRF24L01_FREQ          0

//数据结构声明



//Extern引用



//函数声明

#endif

/******************* (C) 版权所有 2018 北京中科浩电科技有限公司 *******************/
