/**
  ******************************************************************************
  * Copyright (c) 2018,北京中科浩电科技有限公司
  * All rights reserved.
  * 文件名称：include.h
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
#ifndef __INCLUDE_H
#define __INCLUDE_H
//外部文件引用
//第一批被引用的头文件
#include "include.h"
#include "stdint.h"
#include "stdbool.h"
#include "main.h"

//DataBase
#include "my_queue.h"
#include "fmuConfig.h"
//#include "motor.h"

//宏定义区
#define SUCCESS 0
#define FAILED  1


//数据结构声明



//Extern引用



//函数声明
void KernelPolling(void);

#endif

/******************* (C) 版权所有 2018 北京中科浩电科技有限公司 *******************/
