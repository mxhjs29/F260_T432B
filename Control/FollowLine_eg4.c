/*
���������һ���������̶�����
1.����ʱ��
2.�Զ���ɣ�
3.��ͣ��
4.Ѱ��ApriTag��ǣ�����15S���䣻
5.������


����û���Ϊ�Ѿ����ո��ļ�ʹ�÷�������ɾ�����ļ���Ȼ������FollowLine.c�ļ�
*/

#include "FollowLine.h"
#include "HARDWARE_uart.h"
#include "stdbool.h"
#include "pid.h"
#include "myMath.h"
#include "gcs.h"
//#include "program_ctrl.h"
#include "sdk.h"

#define GREEN_LED_ON P2OUT |= GPIO_PIN1
#define GREEN_LED_OFF P2OUT &= ~GPIO_PIN1

extern Usart_t UsartGroup[Num_USART];
extern PIDInfo_t PIDGroup[emNum_Of_PID_List];
extern UAV_info_t g_UAVinfo;

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
bool ActionGoY(int16_t Speed, int16_t Position);
bool ActionGoX(int16_t Speed, int16_t Position);
bool ActionGoZ(int16_t Angle);
void Reset_speed(void);

FollowManager_t FollowManager;
SonarManager_t SonarManager;

/*
        |+X ��ͷ����
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

    PIDGroup[emPID_FolloLinePosVertically].desired = 120 / 2;
    PIDGroup[emPID_FolloLinePosHorizontally].desired = 160 / 2;

    FollowManager.ptrPIDInfoH->OutLimitHigh = 20;
    FollowManager.ptrPIDInfoH->OutLimitLow = -20;
    FollowManager.ptrPIDInfoV->OutLimitHigh = 20;
    FollowManager.ptrPIDInfoV->OutLimitLow = -20;

    FollowManager.ActionComplete = false;

    FollowManager.CountDownNumMs = MAX_COUNTDOWN;
    FollowManager.TargetAltitudeCM = TARGETALTITUDECM;

    FollowManager.ptrFrame = (OpenMVFrame_t *)UsartGroup[LookDownOpenmv].RxBuff;
    FollowManager.ptrUAVInfo = &g_UAVinfo;
}

//��100hz���ٶ���ѯ 10ms
void UpdateCentControl(float dt)
{
    //�ж�OpenMV���ص������Ƿ���ã��е�ʱ��OpenMV�᷵����Ч����
    if (FollowManager.ptrFrame->CentPoint.x1 > 200 || FollowManager.ptrFrame->CentPoint.y1 > 200)
        return;

    //���°�ť����ʵ��
    UpdateButton();

    //���³̿�״̬��
    UpdateStatus();

    //���³̿ض�����
    UpdateAction(dt);
}

//�˺���ֻ��״̬�жϺ�״̬����
void UpdateStatus()
{
    //����ActionList�����ݣ����벻ͬ��״̬
    switch (FollowManager.ActionList)
    {
    //�ж�
    case ActionWaitting:
        //Do nothing;
        break;

    //����ʱ״̬
    case ActionCountdown:
    {
        //����ʱ�����ݳ�ʼ���λ��Follow_Init��
        FollowManager.CountDownNumMs--;

        //������ʱ����ʱ��״̬���ΪActionTakeOff
        if (FollowManager.CountDownNumMs <= 0)
        {
            FollowManager.ActionList = ActionTakeOff;
        }
    }
    break;

    //�Զ����״̬
    case ActionTakeOff:
    {
        //�Զ���ɶ�������ʱ��Ϊ5s��500 * 10ms = 5000ms = 5s����Ȼ������ActionHoverStartPoint������
        ActionHoldPoint(MAX_HOVER_ERR, 500, ActionHoverStartPoint);
    }
    break;

    //��ͣ������5S��Ȼ�������Զ�����״̬
    case ActionHoverStartPoint:
        ActionHoldPoint(MAX_HOVER_ERR, 300, ActionHoldApriTag);
        break;
    case ActionGoForward:
        ActionHoldPoint(MAX_HOVER_ERR, 500, ActionGoLeft);
        break;
    case ActionHoldApriTag:
        ActionHoldPoint(MAX_HOVER_ERR, 1800, ActionLand);
        break;
    case ActionGoBack:
        ActionHoldPoint(MAX_HOVER_ERR, 1500, ActionLand);
        break;
    case ActionGoLeft:
        ActionHoldPoint(MAX_HOVER_ERR, 500, ActionLand);
        break;

    //�Զ�����״̬����ʱ�����Ժ󣬽�����������
    case ActionLand:
    {
        //����ʱ�߼�
        static int Cnt = MAX_TIMEOUT1;

        if (Cnt-- < 0)
        {
            FollowManager.ActionList = ActionLock;
        }
    }
    break;

    //��������
    case ActionLock:
        FollowManager.ActionList = ActionWaitting;
        break;
    default:
        break;
    }
}

//ִֻ�ж���
void UpdateAction(float dt)
{
    switch (FollowManager.ActionList)
    {
    //����ʱ����
    case ActionWaitting:
        //Do nothing
        break;

    //�Զ��������
    case ActionTakeOff:
        sdk_takeoff(80);
        break;

    //��ͣ����
    case ActionHoverStartPoint:
        //���
        sdk_velocity_reset();
        break;
    case ActionHoldApriTag:
        if (FollowManager.ptrFrame->FormType == ApriTag)
        {
            PIDGroup[emPID_FolloLinePosVertically].desired = 120 / 2;
            PIDGroup[emPID_FolloLinePosHorizontally].desired = 160 / 2;

            HoldCurrentPostion(dt);
        }
        else
        {
            sdk_velocity_reset();
        }
        break;
    case ActionGoForward:
        sdk_velociyt_x_set(30);
        break;
    case ActionGoLeft:
        sdk_velociyt_x_set(0);
        sdk_velociyt_y_set(30);
        break;
    case ActionGoBack:
        sdk_velociyt_y_set(0);
        sdk_velociyt_x_set(-30);
        break;
    //�Զ�����
    case ActionLand:
        //��������
        sdk_land();
        break;

    //��������
    case ActionLock:
        g_UAVinfo.FMUflg->unlock = 0;
        break;
    default:
        sdk_velocity_reset();
        break;
    }
}

void Reset_speed()
{
    sdk_velocity_reset();
}

void HoldCurrentPostion(float dt)
{
    static float OldPos[2];
    //��������LPF

    //���²�����
    PIDGroup[emPID_FolloLinePosVertically].measured = FollowManager.ptrFrame->CentPoint.y1;
    PIDGroup[emPID_FolloLinePosHorizontally].measured = FollowManager.ptrFrame->CentPoint.x1;

    PIDGroup[emPID_FolloLineSpdVertically].measured = (FollowManager.ptrFrame->CentPoint.y1 - OldPos[0]);
    PIDGroup[emPID_FolloLineSpdHorizontally].measured = (FollowManager.ptrFrame->CentPoint.x1 - OldPos[1]);

    OldPos[0] = FollowManager.ptrFrame->CentPoint.y1;
    OldPos[1] = FollowManager.ptrFrame->CentPoint.x1;

    UpdatePID(FollowManager.ptrPIDInfoH, dt); //PID
    UpdatePID(FollowManager.ptrPIDInfoV, dt); //PID

    PIDGroup[emPID_FolloLineSpdVertically].desired = FollowManager.ptrPIDInfoV->out;
    PIDGroup[emPID_FolloLineSpdHorizontally].desired = FollowManager.ptrPIDInfoH->out;

    UpdatePID(&PIDGroup[emPID_FolloLineSpdHorizontally], dt); //PID
    UpdatePID(&PIDGroup[emPID_FolloLineSpdVertically], dt);   //PID

    //���������ٶȿ��Ƶ�
    sdk_velocity_set(PIDGroup[emPID_FolloLineSpdVertically].out, PIDGroup[emPID_FolloLineSpdHorizontally].out);
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
        Reset_speed();
        FollowManager.ActionList = NextAction;
    }
}

void UpdateButton()
{
    if(!(GPIOE->IDR >> 4 & 0x01))
    {
        static bool CloseGate = true;

        //�����߽��뵹��ʱ״̬
        if (CloseGate)
        {
            CloseGate = false;
            FollowManager.ActionList = ActionCountdown;
        }
    }
}

#include "math.h"
extern float PIDGroup_desired_yaw_pos_tmp;
bool ActionGoZ(int16_t Angle)
{
    static bool Enter = true;
    static uint16_t CountDown = 0;

    if (Enter)
    {
        CountDown = absFloat((Angle / 0.3f));
        Enter = false;
    }
    else
    {
        CountDown--;
        PIDGroup_desired_yaw_pos_tmp += 0.3f;
    }

    if (CountDown == 0)
    {
        Enter = true;
    }

    return Enter;
}