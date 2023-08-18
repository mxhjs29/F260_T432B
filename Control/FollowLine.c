#include "FollowLine.h"
#include "HARDWARE_uart.h"
#include "stdbool.h"
#include "pid.h"
//#include "timer_drv.h"
#include "myMath.h"
#include "gcs.h"
#include "program_ctrl.h"
#include "SDK.h"
#include "UWB.h"
#include "tim.h"

extern Usart_t UsartGroup[Num_USART];
extern PIDInfo_t PIDGroup[emNum_Of_PID_List];
extern UAV_info_t g_UAVinfo;
extern u16 val, spd;
extern _program_ctrl_st program_ctrl;
bool FollowLine = false;
bool FollowApriTag = false;

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


static uint8_t gate_1 = 1;
FollowManager_t FollowManager;
SonarManager_t SonarManager;
uint8_t fire_fighting_flag;
uint8_t back_to_patrol_flag;
static uint8_t patrol_point[10];
//patrol_goal_t patrol_goal[30]={{80,70},{300,80} ,{500,83},
//									 {497,172}, {310,168},{81,153},
//									 {115,270},{283,223}, {472,214},
//									 {495,347},{303,337}, {98,325},
//									 {95,390}, {303,390},{462,440}
//									};
patrol_goal_t patrol_goal[30]={{125,100} ,{530,110},
								{530,190},{150,200},
								{140,275},{540,280},
									 {550,360},{140,360},
									 {150,445},{550,444},
									};
patrol_goal_t start_position;
patrol_goal_t fire_position;
/* 飞控坐标系
        |+X
        |
        |
+Y------------- -Y
        |
        |
        |-X
        
   像素坐标系     
   |+Y
   |
   |
	------- +X

*/
float absolute(float x)
{
	if(x >= 0)
		return x;
	if(x < 0)
		return -x;
}
void Follow_Init()
{
    FollowManager.ptrPIDInfoV = &PIDGroup[emPID_FolloLinePosVertically];
    FollowManager.ptrPIDInfoH = &PIDGroup[emPID_FolloLinePosHorizontally];
	FollowManager.ptrPIDInfoY = &PIDGroup[emPID_FollowSpdYaw];

    FollowManager.ptrPIDInfoV->kp = 1.5f;
    FollowManager.ptrPIDInfoH->kp = 1.5f;
	FollowManager.ptrPIDInfoY->kp = 0.01f;
	FollowManager.ptrPIDInfoY->ki = 0.001f;

    FollowManager.ptrPIDInfoH->DeathArea = 3;
    FollowManager.ptrPIDInfoV->DeathArea = 3;

    PIDGroup[emPID_FolloLineSpdVertically].kp = 0.45f;
    PIDGroup[emPID_FolloLineSpdVertically].ki = 0.13f;
    PIDGroup[emPID_FolloLineSpdVertically].kd = 0.014f;
    PIDGroup[emPID_FolloLineSpdVertically].OutLimitHigh = 15;
	PIDGroup[emPID_FolloLineSpdVertically].OutLimitLow = -15;
	
    PIDGroup[emPID_FolloLineSpdHorizontally].kp = 0.45f;
    PIDGroup[emPID_FolloLineSpdHorizontally].ki = 0.13f;
    PIDGroup[emPID_FolloLineSpdHorizontally].kd = 0.014f;
    PIDGroup[emPID_FolloLineSpdHorizontally].OutLimitHigh = 15;
    PIDGroup[emPID_FolloLineSpdHorizontally].OutLimitLow  = -15;


    PIDGroup[emPID_FolloLinePosVertically].desired = 240.0f / 2;
    PIDGroup[emPID_FolloLinePosHorizontally].desired = 320.0f / 2;
	PIDGroup[emPID_FollowSpdYaw].desired = 160.0f / 2;
	
    FollowManager.ptrPIDInfoH->OutLimitHigh = 20;
    FollowManager.ptrPIDInfoH->OutLimitLow = -20;
    FollowManager.ptrPIDInfoV->OutLimitHigh = 20;
    FollowManager.ptrPIDInfoV->OutLimitLow = -20;
	FollowManager.ptrPIDInfoY->OutLimitHigh = 0.5;
	FollowManager.ptrPIDInfoY->OutLimitLow = -0.5;
	
	FollowManager.ptrPIDposition_x->desired = 1;
	FollowManager.ptrPIDposition_x->kp = 10;
	FollowManager.ptrPIDposition_x->ki = 0.1;
	FollowManager.ptrPIDposition_x->kd = 0;
	FollowManager.ptrPIDposition_x->OutLimitHigh = 15;
	FollowManager.ptrPIDposition_x->OutLimitLow = -15;
	FollowManager.ptrPIDposition_x->IntegLimitHigh = 15;
	FollowManager.ptrPIDposition_x->IntegLimitLow = -15;
	
	FollowManager.ptrPIDposition_y->desired = 1;
	FollowManager.ptrPIDposition_y->kp = 10;
	FollowManager.ptrPIDposition_y->ki = 0.1;
	FollowManager.ptrPIDposition_y->kd = 0;
	FollowManager.ptrPIDposition_y->OutLimitHigh = 15;
	FollowManager.ptrPIDposition_y->OutLimitLow = -15;
	FollowManager.ptrPIDposition_y->IntegLimitHigh = 15;
	FollowManager.ptrPIDposition_y->IntegLimitLow = -15;
	
	
	sdk_reset_Location();
	
    FollowManager.CountDownNumMs = MAX_COUNTDOWN;
    FollowManager.TargetAltitudeCM = TARGETALTITUDECM;
    FollowManager.ptrUAVInfo = &g_UAVinfo;

    for (int i = 0; i < 3; i++)
    {
        StandardControl.StandardControlDirction[i].Speed = 0;
    }
	
	start_position.x= patrol_goal[0].x;
	start_position.y= patrol_goal[0].y;	
}

//以100hz的速度轮询 10ms
void UpdateCentControl(float dt)
{
//    //判断OpenMV返回的数据是否可用，有的时候OpenMV会返回无效数据
//    if (FollowManager.ptrFrame->CentPoint.x1 > 200 || FollowManager.ptrFrame->CentPoint.y1 > 200)
//        return;

    //更新按钮控制实践
    UpdateButton();

    //更新程控状态线
    UpdateStatus();

    //更新程控动作线
    UpdateAction(dt);
}

//此函数只做状态判断和状态更新
void UpdateStatus()
{
    //根据ActionList的内容，进入不同的状态
    switch (FollowManager.ActionList)
    {
    //判断
    case ActionWaitting:
        //Do nothing;
        break;
    case ActionCountdown:
        FollowManager.CountDownNumMs--;

        if (FollowManager.CountDownNumMs <= 0)
        {
            FollowManager.ActionList = ActionTakeOff;
        }
        break;
    case ActionTakeOff:
    {
        //起飞后悬停到悬停点:
        if (FollowApriTag)
        {
            ActionHoldPoint(MAX_HOVER_ERR, 1000, ActionHoverStartPoint);
        }
    }
		break;
    case ActionHoverStartPoint:
          if(patrol_point[0] == 1)
			  FollowManager.ActionList = patrol;
        break;
    case patrol:
    {
		if(patrol_point[9] == 1 )
			FollowManager.ActionList = back;
    }
		break;
	case back:
	{
		if(absolute(sdk_manager.location_x - patrol_goal[0].x) < 20 && absolute(sdk_manager.location_y - patrol_goal[0].y) < 20)
			FollowManager.ActionList = ActionLand;
	}
		break;
    default:
        break;
    }
}

static bool flag = false;
static uint8_t find = 0;
static int fire_fighting_time;
//只执行动作
void UpdateAction(float dt)
{
    switch (FollowManager.ActionList)
    {
    case ActionWaitting:
        break;
    case ActionTakeOff:
		  sdk_takeoff(180);
		  //sdk_velocity_reset();
	//sdk_pos_set(patrol_goal[0].x, patrol_goal[0].y);
        break;
    case ActionHoverStartPoint:
	{
		sdk_pos_set(patrol_goal[0].x, patrol_goal[0].y);
		static uint8_t gate = 1;
		if(absolute(sdk_manager.location_x - patrol_goal[0].x) < 30 && absolute(sdk_manager.location_y - patrol_goal[0].y) < 30 && gate==1)
		{
			gate = 0;
			patrol_point[0] = 1;
		}
		if(gate == 0)
			sdk_velocity_reset();
			
	}
        break;
	case patrol:
	{
			
			//基础部分，只是巡逻
//			sdk_takeoff(180);
//			static uint8_t temp_goal = 0;
//			uint8_t i;
//			for(i=0;i<10;i++)
//			{
//				if(patrol_point[i] == 0)
//				{
//					temp_goal = i;
//					break;
//				}
//			}
//			sdk_pos_set(patrol_goal[temp_goal].x, patrol_goal[temp_goal].y);
//			if(absolute(sdk_manager.location_x - patrol_goal[temp_goal].x) < 30  && absolute(sdk_manager.location_y - patrol_goal[temp_goal].y) < 30 )
//				patrol_point[temp_goal] = 1;	
//			if(temp_goal != 0)
//				sdk_yaw_reset();
			
			

//			//发挥部分
			static uint8_t temp_goal = 0;
			uint8_t i;
			for(i=0;i<10;i++)
			{
				if(patrol_point[i] == 0)
				{
					temp_goal = i;
					break;
				}
			}

			if(FollowManager.GroundOpenmvFramePtr->FormType == fire && gate_1 == 1 && temp_goal != 1 && temp_goal != 0)
			{
				HoldCurrentPostion(dt);
			}
			else 
			{			
				__HAL_TIM_SetCompare(&htim2, TIM_CHANNEL_2, 600);
				sdk_pos_set(patrol_goal[temp_goal].x, patrol_goal[temp_goal].y);
				sdk_takeoff(180);
			}

			if(absolute(sdk_manager.location_x - patrol_goal[temp_goal].x) < 30  && absolute(sdk_manager.location_y - patrol_goal[temp_goal].y) < 30 )
				patrol_point[temp_goal] = 1;	
			if(temp_goal != 0)
				sdk_yaw_reset();
	}
		break;
	case back:
	{
		sdk_pos_set(patrol_goal[0].x, patrol_goal[0].y);
	
	}
		break;
    case ActionLand:
        //降落
		sdk_yaw_reset();
		sdk_velocity_reset();
		sdk_land();  
        break;
    case ActionLock:
        g_UAVinfo.FMUflg->unlock = 0;
        break;
    default:
        break;
    }

    { //此处为Debug信息
        static int cnt = 0;
        cnt++;

        if (cnt % 4 == 0)
        {
            UpdateDebugInfo();
        }
    }
}

static float vel_filter[2],vel_last_filter[2];


static uint16_t time = 2000;
static uint32_t time_2 = 2000;
static uint8_t time_flag = 0;
static float vel[2],vel_last[2];
void HoldCurrentPostion(float dt)
{
    static float OldPos[2];
	
    //更新测量点
    FollowManager.ptrPIDInfoV->measured = FollowManager.GroundOpenmvFramePtr->CentPoint.y1;
    FollowManager.ptrPIDInfoH->measured = (float)(FollowManager.GroundOpenmvFramePtr->CentPoint.x1);

	
    PIDGroup[emPID_FolloLineSpdVertically].measured = (float)(FollowManager.GroundOpenmvFramePtr->CentPoint.y1 - OldPos[0]);
    PIDGroup[emPID_FolloLineSpdHorizontally].measured = -(float)(FollowManager.GroundOpenmvFramePtr->CentPoint.x1 - OldPos[1]);

    OldPos[0] = FollowManager.GroundOpenmvFramePtr->CentPoint.y1;
    OldPos[1] = FollowManager.GroundOpenmvFramePtr->CentPoint.x1;

    UpdatePID(FollowManager.ptrPIDInfoH, dt); //PID
    UpdatePID(FollowManager.ptrPIDInfoV, dt); //PID

    PIDGroup[emPID_FolloLineSpdVertically].desired = FollowManager.ptrPIDInfoV->out;
    PIDGroup[emPID_FolloLineSpdHorizontally].desired = -FollowManager.ptrPIDInfoH->out;

    UpdatePID(&PIDGroup[emPID_FolloLineSpdHorizontally], dt); //PID11
    UpdatePID(&PIDGroup[emPID_FolloLineSpdVertically], dt);   //PID10
	
	vel[0] = 0.3 * (-PIDGroup[emPID_FolloLineSpdVertically].out) + 0.7 * vel_last[0];
	vel[1] = 0.3 * (PIDGroup[emPID_FolloLineSpdHorizontally].out) + 0.7 * vel_last[1];
	sdk_velocity_set(vel[0], vel[1]);
	
	vel_last[0] = vel[0];
	vel_last[1] = vel[1]; 
	
	if(absolute(FollowManager.ptrPIDInfoV->Err) < 100 && absolute(FollowManager.ptrPIDInfoH->Err) < 100)
	{
		sdk_takeoff(100);
		sdk_velocity_reset();
		if(gate_1 == 1 && absolute(FollowManager.distance - 100) < 5)
		{
			time--;
		}
		if(gate_1 == 1 && time == 0 && time_2 !=0)
		{
			time_2--;
			fire_position.x = UWB_Manager.pos_x;
			fire_position.y = UWB_Manager.pos_y;
			__HAL_TIM_SetCompare(&htim2, TIM_CHANNEL_2, 1500);
			gate_1 = 0;
		}
	}
	
}

static uint16_t x[2];
void HoldYawAngle(float dt)
{
	x[0] = 0.5 * (FollowManager.FrontOpenmvFramePtr->point_1.x1 + FollowManager.FrontOpenmvFramePtr->point_2.x1);
    PIDGroup[emPID_FollowSpdYaw].measured = 0.2 * x[0] + 0.8 * x[1];
	x[1] = x[0];
    UpdatePID(FollowManager.ptrPIDInfoY, dt);

    sdk_yaw_little(-FollowManager.ptrPIDInfoY->out);

}

#include "Ano_OF.h"
extern _ano_of_st ANO_OF;
extern HeightInfo_t HeightInfo;
void UpdateDebugInfo()
{
    UpdateToGCSLine2(PIDGroup[emPID_FolloLinePosVertically].measured, PIDGroup[emPID_FolloLinePosHorizontally].measured, PIDGroup[emPID_FolloLinePosHorizontally].desired, PIDGroup[emPID_FolloLinePosVertically].desired, 0, 0, 0, 0);
}

bool ActionFormChange(int8_t HoldTime, FormType_t TargetFormType, FSMList_t NextAction)
{
    static int cnt = 0;
    bool ChangeFinished = false;

    if (FollowManager.ptrFrame->FormType == TargetFormType)
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

//规定：参数Distance,Speed中，左为正方向，右为负方向
void proControl(int16_t Distance, int16_t Speed)
{
    Distance = LIMIT(Distance, -8, 8);
    Speed = LIMIT(Speed, -10, 10);

    if (Distance > 0 && Speed > 0)
    {
        UpdateCMD(Distance, Speed, CmdLeft);
    }
    else if (Distance < 0 && Speed < 0)
    {
        UpdateCMD(Distance, Speed, CmdRight);
    }
}

//超时限制
void TimeoutCheck()
{
    FSMList_t tmpAction;
    static int Cnt = 0;

    if (tmpAction == FollowManager.ActionList)
    {
        Cnt++;

        if (Cnt > MAX_TIMEOUT2)
        {
            tmpAction = ActionEmergencyLand;
        }
        else if (Cnt > MAX_TIMEOUT1)
        {
            tmpAction = ActionLand;
        }
    }
    else
    {
        Cnt = 0;
        tmpAction = FollowManager.ActionList;
    }
}

static uint8_t input = 0;
static uint8_t input2 = 0;
void UpdateButton()
{
    //判定两个输入是否有效，其实是判断左右两个按键
	input = HAL_GPIO_ReadPin(GPIOE, GPIO_PIN_3); //左 按下为0
	input2 = HAL_GPIO_ReadPin(GPIOE, GPIO_PIN_4);  //右 按下为0
    //判断巡线按钮是否按下
    if (input)
    {
    }
    else
    {
        FollowLine = true;
    }

    //判断寻找ApriTag按钮是否按下
    if (input2)
    {
    }
    else
    {
        FollowApriTag = true;
    }

    //判断当前是否被多按
    if (FollowApriTag == false && FollowLine == false)
    {
        return;
    }
    else
    {
        static uint8_t CloseGate = 1;

        //动作线进入倒计时状态
        if (CloseGate)
        {
            CloseGate = 0;
            FollowManager.ActionList = ActionCountdown;
        }
    }
}
