/*
���������һ���������̶�����
1.����ʱ��
2.�Զ���ɣ�
3.��ͣ��
4.Ѱ��ApriTag��ǣ�����15S���䣻
5.������


����û���Ϊ�Ѿ����ո��ļ�ʹ�÷�������ɾ�����ļ���Ȼ�����FollowLine.c�ļ�
*/

#include "FollowLine.h"
#include "HARDWARE_uart.h"
#include "stdbool.h"
#include "pid.h"
//#include "timer_drv.h"
#include "myMath.h"
#include "gcs.h"
//#include "program_ctrl.h"
#include "sdk.h"

extern Usart_t UsartGroup[Num_USART];
extern PIDInfo_t PIDGroup[emNum_Of_PID_List];
extern UAV_info_t g_UAVinfo;

enSearch_state_t enSearch_state;
Point_t last_openmv;
bool FollowLine = false;
bool FollowApriTag = false;
enSearch_state_t lost_find(float dt);
void proControl(int16_t Distance, int16_t Speed);
void TimeoutCheck(void);
void UpdateStatus(void);
void UpdateAction(float dt);
void UpdateButton(void);
void ActionHoldPoint(int8_t Err, int16_t HoldTime, FSMList_t NextAction);
bool ActionFormChange(int8_t HoldTime, FormType_t TargetFormType, FSMList_t NextAction);
void UpdateDebugInfo(void);
void HoldCurrentPostion(float dt);

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

    FollowManager.CountDownNumMs = MAX_COUNTDOWN;
    FollowManager.TargetAltitudeCM = TARGETALTITUDECM;

    FollowManager.ptrFrame = (OpenMVFrame_t *)UsartGroup[UART_6].RxBuff;
    FollowManager.ptrUAVInfo = &g_UAVinfo;

//    P1DIR &= ~(1 << BIT1);
//    MAP_GPIO_setAsInputPinWithPullUpResistor(GPIO_PORT_P1, GPIO_PIN1);
//    P1DIR &= ~(1 << BIT4);
//    MAP_GPIO_setAsInputPinWithPullUpResistor(GPIO_PORT_P1, GPIO_PIN4);
}

//��100hz���ٶ���ѯ 10ms
void UpdateCentControl(float dt)
{
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
            ActionHoldPoint(MAX_HOVER_ERR, 300, ActionGoForward);
            break;

        case ActionGoForward:
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

    case ActionGoForward:
        sdk_velociyt_x_set(15);
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
        break;
    }
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
    
    UpdatePID(FollowManager.ptrPIDInfoH, dt);  //PID
    UpdatePID(FollowManager.ptrPIDInfoV, dt);  //PID
    
    PIDGroup[emPID_FolloLineSpdVertically].desired = FollowManager.ptrPIDInfoV->out;
    PIDGroup[emPID_FolloLineSpdHorizontally].desired = FollowManager.ptrPIDInfoH->out;
    
    UpdatePID(&PIDGroup[emPID_FolloLineSpdHorizontally], dt);  //PID
    UpdatePID(&PIDGroup[emPID_FolloLineSpdVertically], dt);  //PID

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
        FollowManager.ActionList = NextAction;
    }
}

void UpdateButton()
{
    if(!(GPIOE->IDR >> 3 & 0x01))
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

#define MAX_FINDING 300
#define MAX_LOST_COUNT 300
enSearch_state_t lost_find(float dt)
{
    //�ж��Ƿ�������OpenMV���ݲ�����ɵĶ�ʧ
    static uint16_t cnt = 0;
    enSearch_state_t enSearch_state;
    cnt++;

    if (cnt > MAX_LOST_COUNT)
    {
    }
    else
    {
        enSearch_state = frame_err;
        return enSearch_state;
    }

    cnt = 0;
    enSearch_state = searching;

    
    //������һ��OpenMV�ķ�������ֵ
    PIDGroup[emPID_FolloLinePosVertically].desired = last_openmv.y1;
    PIDGroup[emPID_FolloLinePosHorizontally].desired = last_openmv.x1;

    if (!(FollowManager.ptrFrame->FormType == 255))
    {
        enSearch_state = done;
        return enSearch_state;
    }
    else
    {
        cnt++;

        if (cnt > MAX_FINDING)
        {
            enSearch_state = fail;
        }
    }
}


