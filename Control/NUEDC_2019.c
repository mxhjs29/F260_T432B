#include "FollowLine.h"

void UpdateStatus(void);
void UpdateAction(float dt);
extern void ActionHoldPoint(int16_t HoldTime, FSMList_t NextAction);

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
        ActionHoldPoint(500, ActionGoRight);
        break;

    //此逻辑：检测第一个杆，并调整距离
    //Step1 激光距离是否小于阈值
    case ActionGoRight:
        if (FollowManager.distance < MAX_THR)
        {
            FollowManager.ActionList = ActionFindPoleStep1;
        }
        break;

    //Step2 激光距离是否小于安全值
    case ActionFindPoleStep1:
        if (FollowManager.distance < MAX_SAFE_THR)
        {
            FollowManager.ActionList = ActionGoRight;
        }
        break;

    //Step3 给黄色一维码拍照
    case ActionFindone_dimensional_code:
        break;
    default:
        break;
    }
}

void UpdateAction(dt)
{
}