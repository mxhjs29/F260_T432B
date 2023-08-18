/*
本程序完成2019年电赛试题主要内容

如果用户认为已经掌握该文件使用方法，请删除此文件，然后添加FollowLine.c文件

飞行路线逻辑
- 起飞
- 延时
- 向右飞
- 是否看到黄色中轴处于X轴1/2处
    是
    进入YAW轴锁定状态
- 是否超过5S
    是
    向右飞行
- 是否检测到杆
    是
    向右飞行5S
- 是否飞行完成
    是
    回转180°
- 是否回转完成
    是
    向后飞行
- 是否向后飞行完成
    是
    向右飞行
- 是否检测到黄色物体
    是
    向右飞行
- 向右飞行5s
- 是否检测到杆
    是
    向右飞行3s
- 降落
- 上锁


*/

#include "FollowLine.h"
#include "HARDWARE_uart.h"
#include "stdbool.h"
#include "pid.h"
#include "myMath.h"
#include "gcs.h"
#include "sdk.h"
#include "stdbool.h"

extern Usart_t UsartGroup[Num_USART];
extern PIDInfo_t PIDGroup[emNum_Of_PID_List];
extern UAV_info_t g_UAVinfo;
bool FollowLine = false;
bool FollowApriTag = false;

bool is_set_round(float first_angle, float current_angle);
void proControl(int16_t Distance, int16_t Speed);
void TimeoutCheck(void);
void UpdateStatus(void);
void UpdateAction(float dt);
void UpdateButton(void);
void ActionHoldPoint(int8_t Err, int16_t HoldTime, FSMList_t NextAction);
bool ActionFormChange(int8_t HoldTime, FormType_t TargetFormType, FSMList_t NextAction);
void UpdateDebugInfo(void);
void HoldCurrentPostion(float dt);
void HoldYawAngle(float dt);
float update_yaw_info_in_360(void);
FollowManager_t FollowManager;
SonarManager_t SonarManager;

/*
        |+X
        |
        |
+Y------------- -Y
        |
        |
        |-X
*/

void Follow_Init()
{
    FollowManager.ptrPIDInfoV = &PIDGroup[emPID_FolloLinePosVertically];
    FollowManager.ptrPIDInfoH = &PIDGroup[emPID_FolloLinePosHorizontally];
    FollowManager.ptrPIDInfoY = &PIDGroup[emPID_FollowSpdYaw];

    FollowManager.ptrPIDInfoV->kp = 1.5f;
    FollowManager.ptrPIDInfoH->kp = 1.5f;

    FollowManager.ptrPIDInfoH->DeathArea = 3;
    FollowManager.ptrPIDInfoV->DeathArea = 3;

    PIDGroup[emPID_FolloLineSpdVertically].kp = 0.45f;
    PIDGroup[emPID_FolloLineSpdVertically].ki = 0.13f;
    PIDGroup[emPID_FolloLineSpdVertically].kd = 0.014f;

    PIDGroup[emPID_FolloLineSpdHorizontally].kp = 0.45f;
    PIDGroup[emPID_FolloLineSpdHorizontally].ki = 0.13f;
    PIDGroup[emPID_FolloLineSpdHorizontally].kd = 0.014f;

    PIDGroup[emPID_FolloLinePosVertically].desired = 240 / 2;
    PIDGroup[emPID_FolloLinePosHorizontally].desired = 320 / 2;

    FollowManager.ptrPIDInfoH->OutLimitHigh = 20;
    FollowManager.ptrPIDInfoH->OutLimitLow = -20;
    FollowManager.ptrPIDInfoV->OutLimitHigh = 20;
    FollowManager.ptrPIDInfoV->OutLimitLow = -20;

    FollowManager.CountDownNumMs = MAX_COUNTDOWN;
    FollowManager.TargetAltitudeCM = TARGETALTITUDECM;

    FollowManager.FrontOpenmvFramePtr2 = (OpenMVFrame2_t *)UsartGroup[FrontViewOpenmv].RxBuff;
    FollowManager.ptrUAVInfo = &g_UAVinfo;
    
    FollowManager.ptrPIDInfoY->kp = 0.01f;
    FollowManager.ptrPIDInfoY->ki = 0.0005f;

    FollowManager.ptrPIDInfoY->OutLimitLow = -0.5;
    FollowManager.ptrPIDInfoY->OutLimitHigh = 0.5;
    FollowManager.ptrPIDInfoY->desired = 320 / 2;
}

//以100hz的速度轮询 10ms
void UpdateCentControl(float dt)
{
    //判断OpenMV返回的数据是否可用，有的时候OpenMV会返回无效数据
    if (FollowManager.FrontOpenmvFramePtr2->CentPoint.x1 > 2000 || FollowManager.FrontOpenmvFramePtr2->CentPoint.y1 > 2000)
        return;

    //更新距离
    FollowManager.distance = FollowManager.FrontOpenmvFramePtr2->cnt1 | FollowManager.FrontOpenmvFramePtr2->Target >> 8;

    //更新按钮控制实践
    UpdateButton();

    //更新程控状态线
    UpdateStatus();

    //更新程控动作线
    UpdateAction(dt);
}

float angle_temp_dt = 0;
//此函数只做状态判断和状态更新
void UpdateStatus()
{
    uint16_t distance = 0;
    static float pre_yaw_temp = 0;
    static int cnt = 0;

    //根据ActionList的内容，进入不同的状态
    switch (FollowManager.ActionList)
    {
    //判断
    case ActionWaitting:
        //Do nothing;
        break;

    //倒计时状态
    case ActionCountdown:
    {
        //倒计时，数据初始填充位于Follow_Init中
        FollowManager.CountDownNumMs--;

        //当倒计时结束时候，状态变更为ActionTakeOff
        if (FollowManager.CountDownNumMs <= 0)
        {
            FollowManager.ActionList = ActionTakeOff;
        }
    }
    break;

    //自动起飞状态
    case ActionTakeOff:
    {
        //自动起飞动作持续时间为5s（500 * 10ms = 5000ms = 5s），然后跳到ActionHoverStartPoint动作；
        ActionHoldPoint(MAX_HOVER_ERR, 500, ActionHoverStartPoint);
    }
    break;

    case ActionHoverStartPoint:
        ActionHoldPoint(MAX_HOVER_ERR, 300, ActionGoRight);
        //        ActionHoldPoint(MAX_HOVER_ERR, 200, ActionFindLandSpace);
        break;
    case ActionGoRight:
        //是否扫描到第一个跟杆
        if(FollowManager.distance < 100)
        {
            //yes
            //继续又偏，等待扫描到黄色一维码
            ActionHoldPoint(MAX_HOVER_ERR, 10, ActionFindoneDimensionalCodeStep1);
        }
        
        break;
    case ActionFindoneDimensionalCodeStep1:
        //黄色一维码是否处于视野的1/2处
        if(FollowManager.FrontOpenmvFramePtr2->CentPoint.x1 < 160 && FollowManager.FrontOpenmvFramePtr2->FormType == 201)
        {
            //是
            //进入一维码2段
            FollowManager.ActionList = ActionFindoneDimensionalCodeStep2;
        }
        break;
    case ActionFindoneDimensionalCodeStep2:
        //开始进行水平校准控制
        FollowManager.ptrPIDInfoY->desired = 320/2;
        //选择性进行前进控制
        //当误差小于MAX_ERR时
        
        //调整5s的YAW轴，进入go right2
        ActionHoldPoint(MAX_HOVER_ERR, 300, ActionGoRight2);
        break;
    case ActionGoRight2:
        //是否扫到杆
        //是
        //进入 ActionCloseToPole
        ActionHoldPoint(MAX_HOVER_ERR, 300, ActionCloseToPole);
        break;
    case ActionCloseToPole:
        {
            static bool enter = false;
            if(FollowManager.distance < 100 || enter)
            {
                enter = true;
                
                ActionHoldPoint(MAX_HOVER_ERR, 600, ActionTurnRound);
            }
        }
        break;
    case ActionTurnRound:
        // 目标：保持杆在画面中心
        // 缓慢调整yaw角到180°
        //  yaw是否调整完毕
        //      是
        //      继续向后飞行
        if (is_yaw_set_compleate())
        {
            FollowManager.ActionList = ActionGoBack;
        }
        break;
    case ActionGoBack:
        ActionHoldPoint(MAX_HOVER_ERR, 600, ActionGoRight3);
        break;
    case ActionGoRight3:
        //选择性进行前进控制
        {
            static bool enter = false;
        
            if((FollowManager.FrontOpenmvFramePtr2->CentPoint.x1 < 160 && FollowManager.FrontOpenmvFramePtr2->FormType == 201) || enter)
            {
                enter = true;
                //校准控制
                ActionHoldPoint(MAX_HOVER_ERR, 10, ActionFindoneDimensionalCodeStep3);
            }
        }
        //限时继续右偏ActionGoRight4
        break;
    case ActionFindoneDimensionalCodeStep3:
        ActionHoldPoint(MAX_HOVER_ERR, 500, ActionGoRight4);
        break;
    case ActionGoRight4:
        //是否扫描到了杆
        if(FollowManager.distance < 100)
        {
            //是
            //切换到ActionPreLand
            ActionHoldPoint(MAX_HOVER_ERR, 10, ActionPreLand);
        }
       
        break;
    case ActionPreLand:
        ActionHoldPoint(MAX_HOVER_ERR, 200, ActionLand);
        break;

    //自动降落状态倒计时结束以后，进入上锁动作
    case ActionLand:
    {
        //倒计时逻辑
        static int Cnt = MAX_TIMEOUT1;

        if (Cnt-- < 0)
        {
            FollowManager.ActionList = ActionLock;
        }
    }
    break;

    //上锁动作
    case ActionLock:
        FollowManager.ActionList = ActionWaitting;
        break;
    default:
        break;
    }
}

extern float PIDGroup_desired_yaw_pos_tmp;
//只执行动作
void UpdateAction(float dt)
{
    static float last_yaw = 0;
    switch (FollowManager.ActionList)
    {
    //倒计时命令
    case ActionWaitting:
        //Do nothing7
        break;

    //自动起飞命令
    case ActionTakeOff:
        sdk_takeoff(60);
        break;

    //悬停命令
    case ActionHoverStartPoint:
        //起飞
        {
        }
        break;

    case ActionGoRight:
    case ActionGoRight2:
    case ActionGoRight4:
    case ActionGoRight3:
        sdk_velocity_reset();
        sdk_velociyt_y_set(-10);
        break;
    case ActionFindoneDimensionalCodeStep3:
//        sdk_velocity_reset();
//        if(FollowManager.FrontOpenmvFramePtr2->FormType == 201)
//        {
//            HoldYawAngle(dt);
//        }
        break;
    case ActionFindoneDimensionalCodeStep1:
        sdk_velociyt_y_set(-10);
        break;
    case ActionFindoneDimensionalCodeStep2:
        sdk_velocity_reset();
        break;
    case ActionFindPole:
        sdk_velocity_reset();
        sdk_yaw_set(180);
        last_yaw = sdk_manager.yaw_mark;
        break;
    case ActionCloseToPole:
        sdk_velocity_reset();
        sdk_yaw_stop();
        sdk_velociyt_y_set(-10);
        break;
    case ActionTurnRound:
        sdk_velocity_reset();
        sdk_yaw_set(180);
        break;
    case ActionPreLand:
        sdk_velocity_reset();
        break;
    case ActionGoLeft:

        break;
    case ActionGoBack:
        sdk_velocity_reset();
        sdk_velociyt_x_set(-10);
        break;
    //自动降落
    case ActionLand:
        sdk_velocity_reset();
        //降落命令
        sdk_land();
        break;

    //上锁动作
    case ActionLock:
        //        g_UAVinfo.FMUflg->unlock = 0;
        break;
    default:
        break;
    }
}

void HoldCurrentPostion(float dt)
{
    static float OldPos[2];
    //对输入做LPF

    //更新测量点
    PIDGroup[emPID_FolloLinePosVertically].measured = FollowManager.GroundOpenmvFramePtr->CentPoint.y1;
    PIDGroup[emPID_FolloLinePosHorizontally].measured = FollowManager.GroundOpenmvFramePtr->CentPoint.x1;

    PIDGroup[emPID_FolloLineSpdVertically].measured = (FollowManager.GroundOpenmvFramePtr->CentPoint.y1 - OldPos[0]);
    PIDGroup[emPID_FolloLineSpdHorizontally].measured = (FollowManager.GroundOpenmvFramePtr->CentPoint.x1 - OldPos[1]);

    OldPos[0] = FollowManager.GroundOpenmvFramePtr->CentPoint.y1;
    OldPos[1] = FollowManager.GroundOpenmvFramePtr->CentPoint.x1;

    UpdatePID(FollowManager.ptrPIDInfoH, dt); //PID
    UpdatePID(FollowManager.ptrPIDInfoV, dt); //PID

    PIDGroup[emPID_FolloLineSpdVertically].desired = FollowManager.ptrPIDInfoV->out;
    PIDGroup[emPID_FolloLineSpdHorizontally].desired = FollowManager.ptrPIDInfoH->out;

    UpdatePID(&PIDGroup[emPID_FolloLineSpdHorizontally], dt); //PID
    UpdatePID(&PIDGroup[emPID_FolloLineSpdVertically], dt);   //PID

    sdk_velocity_set(PIDGroup[emPID_FolloLineSpdVertically].out, PIDGroup[emPID_FolloLineSpdHorizontally].out);
}

bool ActionFormChange(int8_t HoldTime, FormType_t TargetFormType, FSMList_t NextAction)
{
    static int cnt = 0;
    bool ChangeFinished = false;

    if (FollowManager.GroundOpenmvFramePtr->FormType == TargetFormType)
    {
        cnt++;

        if (cnt > HoldTime)
        {
            cnt = 0;
            FollowManager.ActionList = NextAction;
            ChangeFinished = true;
        }
    }
    else
    {
        cnt = 0;
    }

    return ChangeFinished;
}

void ActionHoldPoint(int8_t Err, int16_t HoldTime, FSMList_t NextAction)
{
    static bool Enter = true;
    static uint16_t CountDown = 0;

    if (Enter)
    {
        CountDown = HoldTime;
        Enter = false;
    }
    else
    {
        CountDown--;
    }

    if (CountDown == 0)
    {
        Enter = true;
        FollowManager.ActionList = NextAction;
    }
}

void UpdateButton()
{
    if(!(GPIOE->IDR >> 4 & 0x01))
    {
        static bool CloseGate = true;

        //动作线进入倒计时状态
        if (CloseGate)
        {
            CloseGate = false;
            FollowManager.ActionList = ActionCountdown;
        }
    }
}

extern float PIDGroup_desired_yaw_pos_tmp;
void HoldYawAngle(float dt)
{
    //更新测量点
    PIDGroup[emPID_FollowSpdYaw].measured = FollowManager.FrontOpenmvFramePtr2->CentPoint.x1;
    UpdatePID(FollowManager.ptrPIDInfoY, dt);

    //    PIDGroup_desired_yaw_pos_tmp = -FollowManager.ptrPIDInfoY->out;
    sdk_yaw_little(FollowManager.ptrPIDInfoY->out);
    //    sdk_yaw_little(FollowManager.ptrPIDInfoY->out);
}

float update_yaw_info_in_360()
{
    float angle = 0;

    angle = g_Attitude.yaw;

    if (angle < 0)
    {
        angle += 360;
    }

    return angle;
}

bool is_set_round(float first_angle, float current_angle)
{
    static uint8_t step = 0;
    bool result = false;

    switch (step)
    {
    case 0:
        //waiting
        step = 1;
        break;
    case 1:
        //mark
        step = 2;
        break;
    case 2:
        if (absFloat(current_angle - first_angle) / 2 > 90)
        {
            step = 3;
        }
        break;
    case 3:
        if (absFloat(current_angle - first_angle) / 2 < 5)
        {
            step = 0;
            result = true;
        }
        break;
    default:
        break;
    }

    return result;
}
