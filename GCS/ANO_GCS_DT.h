/**
  ******************************************************************************
  * Copyright (c) 2018,北京中科浩电科技有限公司
  * All rights reserved.
  * 文件名称：height_control.h
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
#ifndef __ano_gcs_dt_H
#define __ano_gcs_dt_H
//外部文件引用
#include "include.h"
#include "program_ctrl.h"

//宏定义区
#define ANO_DT_USE_USART2

//数据结构声明


//Extern引用




//函数声明
//static
static void ANO_DT_SendCmd(u8 dest, u8 fun, u16 cmd1, u16 cmd2, u16 cmd3, u16 cmd4, u16 cmd5);
static void ANO_DT_Send_Data(u8 *dataToSend , u8 length);
//public
void ANO_DT_Data_Receive_Anl_Task(void);//查询是否有数据需要解析
void ANO_DT_Data_Receive_Prepare(u8 data);//接收BYTE数据（放入串口接收中断）
#endif

/******************* (C) 版权所有 2018 北京中科浩电科技有限公司 *******************/

