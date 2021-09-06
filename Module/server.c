/*
 * @权限: Copyright (c) 2019,北京中科浩电科技有限公司
 * @描述: 本文件用于舵机初始化和角速度控制
 * @版本: v1.0
 * @文件名: server.c
 * @作者: 北京中科浩电科技有限公司高教产品开发部
 * @完成日期: 219-8-6
 * @最后更新时间: 
 */
#include "server.h"
#include "include.h"

/**
 * @函数名称: server_init
 * @函数描述: 舵机函数初始化，生成50HZ的PWM波
 * @输入: void
 * @输出: void
 * @返回: void
 * @备注: null
 */
void server_init(void)
{

}

/**
 * @函数名称: set_server_angle
 * @函数描述: 设置两个舵机角速度
 * @输入: serverID_t ID：选择舵机ID
 *        float palstance：舵机角速度
 * @输出: void
 * @返回: void
 * @备注: 注意输出的是角速度
 */
void set_server_angle(serverID_t ID, float palstance)
{
    switch (ID)
    {
        case ServerV:
            
            break;
        case ServerH:

            break;
        case ServerP:
            
            break;
        default:
            break;
    }
}
