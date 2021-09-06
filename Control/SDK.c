#include "SDK.h"
#include "program_ctrl.h"
#include "FollowLine.h"
#include "myMath.h"
#include "Ano_OF.h"
#include "math.h"

//void sdk_target_set(float x_pos,float y_pos,float velocity_x, float velocity);
//
#define MAX_VELOCITY_X 80
#define MAX_VELOCITY_Y 80
#define MAX_ALT_THR 20
#define LAND_SPEED -50
extern float PIDGroup_desired_yaw_pos_tmp;
extern _ano_of_st ANO_OF;

typedef enum
{
    sdk_alt = 0,
    // sdk_velocity_x,
    // sdk_velocity_y,
    sdk_pos_x,
    sdk_pos_y,

    sdk_pid_list,
} sdk_pid_list_t;

PIDInfo_t sdk_pid[sdk_pid_list];
void sdk_update_s_2(float raw, s2_t *s2);

sdk_manager_t sdk_manager;
s2_t s2_height;

/******************************************************************************
  * 函数名称：sdk_init
  * 函数描述：sdk的初始化
  * 输    入：void
  * 输    出：void
  * 返    回：void 
  * 备    注：null    
  *    
  *
******************************************************************************/
void sdk_init()
{
    sdk_pid[sdk_alt].OutLimitHigh = MAX_ALT_THR;
    sdk_pid[sdk_alt].OutLimitLow = -MAX_ALT_THR;

    sdk_pid[sdk_alt].kp = 0.3;
    sdk_pid[sdk_alt].ki = 0.01;

    sdk_pid[sdk_alt].IntegLimitHigh = 10;
    sdk_pid[sdk_alt].IntegLimitLow = -10;
    
    //位置控制PID
    sdk_pid[sdk_pos_x].kp = 0.55f;
    sdk_pid[sdk_pos_x].ki = 0.1f;
    sdk_pid[sdk_pos_y].kp = 0.55f;
    sdk_pid[sdk_pos_y].ki = 0.1f;
    
    //位置控制器输出限幅
    sdk_pid[sdk_pos_x].OutLimitHigh = 20;
    sdk_pid[sdk_pos_x].OutLimitLow = -20;
    sdk_pid[sdk_pos_y].OutLimitHigh = 20;
    sdk_pid[sdk_pos_y].OutLimitLow = -20;
    
    sdk_pid[sdk_pos_x].IntegLimitHigh = 10;
    sdk_pid[sdk_pos_x].IntegLimitLow = -10;
    sdk_pid[sdk_pos_y].IntegLimitHigh = 10;
    sdk_pid[sdk_pos_y].IntegLimitLow = -10;

    sdk_manager.sdk_alt_step = 2;
    sdk_manager.sdk_yaw_step = 2;
    sdk_manager.sdk_yaw_d_angle = 0.1f;

    sdk_manager.sdk_auto_takeoff = true;
    sdk_manager.yaw_pos_ptr = &PIDGroup_desired_yaw_pos_tmp;
    
    s2_height.length = 5;
}

/******************************************************************************
  * 函数名称：sdk_takeoff
  * 函数描述：使用sdk进行无人机的起飞工作，包含自动解锁等内容
  * 输    入：alitude  自动起飞后的飞行高度
  * 输    出：void
  * 返    回：void 
  * 备    注：注意安全，谨慎使用    
  *    
  *
******************************************************************************/
void sdk_takeoff(float alititude)
{
    sdk_manager.sdk_alt_step = 0;
    sdk_pid[sdk_alt].desired = alititude;
}

/******************************************************************************
  * 函数名称：sdk_land
  * 函数描述：使用sdk进行无人机的降落工作，包含自动上锁等内容
  * 输    入：void
  * 输    出：void
  * 返    回：void 
  * 备    注：注意安全，谨慎使用    
  *    
  *
******************************************************************************/
void sdk_land()
{
    sdk_manager.sdk_alt_step = 1;
}

void sdk_reset_Location()
{
    sdk_manager.location_x = 0;
    sdk_manager.location_y = 0;
}

/******************************************************************************
  * 函数名称：sdk_update
  * 函数描述：使用sdk进行数据更新操作
  * 输    入：float dt 单位运行时间
  * 输    出：void
  * 返    回：void 
* 备    注：该任务已经挂在kernel任务列表下执行，不需要进行更改，如果您已经知道您
    *更改的内容，则可以进行尝试
  *    
  *
******************************************************************************/
void sdk_update(float dt)
{
    //更新位置的时候记得旋转坐标系
    sdk_update_position();

    sdk_update_s_2(HeightInfo.Z_Postion, &s2_height);
    
    switch (sdk_manager.sdk_alt_step)
    {
    case 0:
        sdk_pid[sdk_alt].measured = ANO_OF.ALT;
        UpdatePID(&sdk_pid[sdk_alt], dt);

        sdk_manager.sdk_alt_out = sdk_pid[sdk_alt].out;

        //此处缺少高度飞行成功以后的状态切换
        if (sdk_pid[sdk_alt].Err < 10)
        {
            static int cnt = 0;
            cnt++;
            reset_i(&sdk_pid[sdk_alt]);

            if (cnt < 10)
            {
                sdk_manager.sdk_alt_step = 2;
            }
        }

        if (sdk_manager.sdk_auto_takeoff == true)
        {
            sdk_manager.sdk_auto_takeoff = false;
            sdk_unlock();
        }

        break;
    case 1:
        sdk_manager.sdk_alt_out = LAND_SPEED;

        //此处缺少碰撞检测并且停机的逻辑
        if (HeightInfo.Z_Postion < 15)
        {
            static int cnt = 0;
            cnt++;

            if (cnt > 500)
            {
                sdk_manager.sdk_alt_step = 2;
                sdk_lock();
            }
        }
        break;
    default:
        sdk_manager.sdk_alt_out = 0;
        break;
    }

    switch (sdk_manager.sdk_yaw_step)
    {
    case 0:
        PIDGroup_desired_yaw_pos_tmp += sdk_manager.sdk_yaw_d_angle;
        sdk_manager.yaw_mark += sdk_manager.sdk_yaw_d_angle;

        if (PIDGroup[emPID_Yaw_Pos].desired >= 180)
        {
            PIDGroup_desired_yaw_pos_tmp -= 360;
        }
        else if (PIDGroup[emPID_Yaw_Pos].desired < -180)
        {
            PIDGroup_desired_yaw_pos_tmp += 360;
        }

        sdk_manager.sdk_yaw_angle_count--;
        if (sdk_manager.sdk_yaw_angle_count == 0)
        {
            sdk_manager.sdk_yaw_step = 1;
        }
        break;
    case 1:

        break;
    default:
        break;
    }
}

/******************************************************************************
  * 函数名称：sdk_alititude_set
  * 函数描述：使用sdk修改期望高度值
  * 输    入：void
  * 输    出：void
  * 返    回：void 
    * 备    注：暂不开放
  *    
  *
******************************************************************************/
void sdk_alititude_set(float distance)
{
    sdk_pid[sdk_alt].desired = distance;
}

/******************************************************************************
  * 函数名称：sdk_velocity_set
  * 函数描述：使用sdk修改期望水平速度期望值
  * 输    入：float x float y表示水平速度方向和大小
  * 输    出：void
  * 返    回：void 
    * 备    注：速度参考方向可以按照此图来进行，一般来说，速度绝对值设定在20即可完成大部分程控赛题
  *
  *
       (+X)
        |
        |
(-Y)---------(+Y)
        |
        |
       (-X)

******************************************************************************/
void sdk_velocity_set(float x, float y)
{
    sdk_manager.sdk_velocity_x = LIMIT(x, -MAX_VELOCITY_X, MAX_VELOCITY_X);
    sdk_manager.sdk_velocity_y = LIMIT(y, -MAX_VELOCITY_Y, MAX_VELOCITY_Y);
}

/******************************************************************************
  * 函数名称：sdk_lock
* 函数描述：使用sdk对无人机进行上锁
  * 输    入：void
  * 输    出：void
  * 返    回：void 
  * 备    注：void
******************************************************************************/
void sdk_lock()
{
    //上锁
    g_FMUflg.unlock = 0;
}

/******************************************************************************
  * 函数名称：sdk_unlock
* 函数描述：使用sdk对无人机进行解锁
  * 输    入：void
  * 输    出：void
  * 返    回：void 
  * 备    注：void
******************************************************************************/
void sdk_unlock()
{
    //解锁
    g_FMUflg.unlock = 1;
}

/******************************************************************************
  * 函数名称：sdk_yaw_set
* 函数描述：使用sdk对无人机进行转角飞行
  * 输    入：void
  * 输    出：void
  * 返    回：void 
  * 备    注：void
******************************************************************************/
#define fuzhi(x) x < 0 ? x : x * -1
void sdk_yaw_set(float angle)
{
    static float last_angle = 0;

    if (last_angle != angle)
    {
        sdk_manager.sdk_yaw_step = 1;
    }

    last_angle = angle;

    if (sdk_manager.sdk_yaw_step != 0)
    {
        sdk_manager.sdk_yaw_angle_count = absFloat(angle / sdk_manager.sdk_yaw_d_angle);
        sdk_manager.sdk_yaw_step = 0;
    }

    if (angle > 0)
    {
        sdk_manager.sdk_yaw_d_angle = fuzhi(sdk_manager.sdk_yaw_d_angle);
    }
    else
    {
        sdk_manager.sdk_yaw_d_angle = absFloat(sdk_manager.sdk_yaw_d_angle);
    }
}

/******************************************************************************
  * 函数名称：sdk_round_set
  * 函数描述：使用sdk对围绕某一半径做圆形运动，相当于绕杆飞行，飞行时，无人机机头对准杆
  * 输    入：float distance, float angle, uint8_t R_
    distance：表示圆形运动的半径
    float angle：表示运行的角度
    uint8_t R_：表示左转还是右转，如果R大于0，则俯视下无人机绕杆逆时针旋转，如果R小于0，则俯视下无人机绕杆顺时针旋转
  * 输    出：void
  * 返    回：void 
  * 备    注：void
******************************************************************************/
void sdk_round_set(float distance, float angle, uint8_t R_)
{
    /*
    distance实际上是这个圈的半径，我需要计算相关的信息
    t = angle / d_angle;
    v_r = 2 * pi * distance * angle / 360 / t

    eg :distance 50 cm
    angle:360

    0.2 = 角速度  每10ms + 0.2  =  1s + 20°

    t = 360 / 20 = 360 / 20 = 18s;
    v_r = 2 * 3.1415926 * 50 * 360 / 360 / 18 = 17.453;
    */
    float pi = 3.1415926f;
    float t = 0;
    t = angle / (sdk_manager.sdk_yaw_d_angle * 100);
    
    
    if (R_ > 0)
    {
        sdk_manager.sdk_velocity_y = 2 * pi * distance * angle / 360 / t;
    }
    else
    {
        sdk_manager.sdk_velocity_y = 2 * pi * distance * angle / 360 / t * -1;
    }

    sdk_yaw_set(angle);
}

/******************************************************************************
  * 函数名称：sdk_velocity_reset
* 函数描述：重置水平速度
  * 输    入：void
  * 输    出：void
  * 返    回：void 
* 备    注：相当于让飞机空中悬停静止
******************************************************************************/
void sdk_velocity_reset()
{
    sdk_manager.sdk_velocity_y = 0;
    sdk_manager.sdk_velocity_x = 0;
}

/******************************************************************************
  * 函数名称：sdk_velociyt_x_set
  * 函数描述：设定X轴的速度值
  * 输    入：float X 表示Y轴的速度设定值
  * 输    出：void
  * 返    回：void
  * 备    注：单独设定x轴的速度值
******************************************************************************/
void sdk_velociyt_x_set(float x)
{
    sdk_manager.sdk_velocity_x = LIMIT(x, -MAX_VELOCITY_X, MAX_VELOCITY_X);
    // sdk_manager.sdk_velocity_y = LIMIT(y, -MAX_VELOCITY_Y, MAX_VELOCITY_Y);
}

/******************************************************************************
  * 函数名称：sdk_velociyt_y_set
* 函数描述：设定Y轴的速度值
* 输    入：float y 表示Y轴的速度设定值
  * 输    出：void
  * 返    回：void 
* 备    注：单独设定y轴的速度值
******************************************************************************/
void sdk_velociyt_y_set(float y)
{
    // sdk_manager.sdk_velocity_x = LIMIT(x, -MAX_VELOCITY_X, MAX_VELOCITY_X);
    sdk_manager.sdk_velocity_y = LIMIT(y, -MAX_VELOCITY_Y, MAX_VELOCITY_Y);
}

/******************************************************************************
  * 函数名称：is_yaw_set_compleate
* 函数描述：确认是否YAW轴的转动已经结束
* 输    入：void
  * 输    出：void
  * 返    回：void 
* 备    注：yaw的转动需要时间，此函数用以确认yaw的转动是否结束
******************************************************************************/
bool is_yaw_set_compleate()
{
    bool result = false;

    if (sdk_manager.sdk_yaw_step != 0)
    {
        result = true;
    }

    return result;
}

/******************************************************************************
  * 函数名称：sdk_yaw_reset
* 函数描述：重置yaw角
* 输    入：void
  * 输    出：void
  * 返    回：void 
* 备    注：相当于让飞机重新归位
******************************************************************************/
void sdk_yaw_reset()
{
    PIDGroup_desired_yaw_pos_tmp = 0;
}

/******************************************************************************
  * 函数名称：sdk_yaw_stop
* 函数描述：停止yaw角转动
* 输    入：void
  * 输    出：void
  * 返    回：void 
* 备    注：相当于让飞机重新归位
******************************************************************************/
void sdk_yaw_stop()
{
    sdk_manager.sdk_yaw_step = 1;
    sdk_manager.sdk_yaw_angle_count = 0;
}

/******************************************************************************
  * 函数名称：sdk_yaw_little
* 函数描述：让飞机旋转微小的yaw角，适用于0.5度之间
* 输    入：void
  * 输    出：void
  * 返    回：void 
* 备    注：void
******************************************************************************/
void sdk_yaw_little(float yaw)
{
    //注意，这里的YAW取值范围需要在-5 5之间
    yaw = LIMIT(yaw, -0.5, 0.5);
    PIDGroup_desired_yaw_pos_tmp += yaw;
}

void sdk_reset_position()
{
    sdk_manager.location_x = 0;
    sdk_manager.location_y = 0;
}

void sdk_update_position()
{
    if(ANO_OF.DX2FIX > -100 && ANO_OF.DX2FIX < 100 && 
       ANO_OF.DY2FIX > -100 && ANO_OF.DY2FIX < 100)
    {
        sdk_manager.location_x += ANO_OF.DX2FIX * 0.01f;
        sdk_manager.location_y += ANO_OF.DY2FIX * 0.01f * -1;
    }
}


void sdk_pos_set(float x,float y)
{
    sdk_pid[sdk_pos_x].measured = sdk_manager.location_x;
    sdk_pid[sdk_pos_x].desired = x;
    UpdatePID(&sdk_pid[sdk_pos_x],0.01f);
    sdk_velociyt_x_set(sdk_pid[sdk_pos_x].out);
    
    sdk_pid[sdk_pos_y].measured = sdk_manager.location_y;
    sdk_pid[sdk_pos_y].desired = y;
    UpdatePID(&sdk_pid[sdk_pos_y],0.01f);
    sdk_velociyt_y_set(sdk_pid[sdk_pos_y].out);
}

bool is_pos_x_set_compleate(float max_err)
{
    bool result = false;
    
    if(ABS(sdk_pid[sdk_pos_x].Err) < max_err)
    {
        result = true;
    }
    
    return result;
}

bool is_pos_y_set_compleate(float max_err)
{
    bool result = false;
    
    if(ABS(sdk_pid[sdk_pos_y].Err) < max_err)
    {
        result = true;
    }
    
    return result;
}

bool is_pos_set_compleate(float max_err)
{
    bool result = false;
    
    if(    is_pos_y_set_compleate(max_err) 
        && is_pos_x_set_compleate(max_err))
    {
        result = true;
    }
    
    return result;
}

/*
       |cos(θ)   -sin(θ)|
M(θ) = |                |
       |sin(θ)    cos(θ)|
    |x|
a = | |
    |y|
            |x * cos(θ) + y * sin(θ)|
b = a * M = |                       |
            |x * -sin(θ)+ y * cos(θ)|

二维旋转矩阵
* 函数名称：RotateLocation
* 函数描述：在没有偏移的情况下，水平旋转某一坐标
* 输    入：int16_t *input 输入的坐标地址
            float xita 需要旋转的角度
* 输    出：int16_t *output 输出的坐标地址
* 返    回：void 
* 备    注：void
*/

void RotateLocation(int16_t *input, int16_t *output, float xita)
{
    float cosdata = cosf(xita / 57.3f);
    float sindata = sinf(xita / 57.3f);
    output[0] = input[0] * cosdata + input[1] * sindata;
    output[1] = input[0] * -sindata + input[1] * cosdata;
}


void sdk_update_s_2(float raw, s2_t *s2)
{
    float sum = 0;
    float avg = 0;
    
//    memmove(s2->data_temp + 1, s2->data_temp, s2->length - 1);
    
    for(int i = 1;i < s2->length;i++)
    {
        s2->data_temp[i] = s2->data_temp[i - 1];
    }
    s2->data_temp[0] = raw;
    
    //计算avg
    for(int i = 0;i < s2->length;i++)
    {
        sum += s2->data_temp[i];
    }
    
    avg = sum / s2->length;
    
    //计算s2
    sum = 0;
    for(int i = 0;i < s2->length;i++)
    {
        sum += squa(s2->data_temp[i] - avg);
    }
    
    s2->s2 = sum / (s2->length - 1);
}
