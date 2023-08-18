#ifndef __FOLLOW_LINE_H
#define __FOLLOW_LINE_H

#include "stdint.h"
#include "stdbool.h"
#include "pid.h"
#include "gcs.h"
#include "HARDWARE_uart.h"

//单位:10ms
#define MAX_COUNTDOWN 100 * 6
#define TARGETALTITUDECM 50
#define MAX_HOVER_ERR 10
#define MAX_FORMCHANGE_TIME 3
#define MAX_HOVER_TIME 300
#define MAX_TIMEOUT1 500
#define MAX_TIMEOUT2 1500
#define MAX_ALT_ERR 10

#define set_position_mode 1
#define LookDownOpenmv  UART_3
#define FrontViewOpenmv  UART_4
/*

*/

typedef enum
{
    CmdNone = 0,
    CmdTakeOff,
    CmdLand,
    CmdUp,
    CmdDown,
    CmdForward,
    CmdBack,
    CmdLeft,
    CmdRight,
    CmdCCW,
    CmdCW,
    CmdEmergencyShutDown,
    CmdSpeeedControl,

    NumofCmd,
} FMUCmd_t;

typedef enum
{
    Vertical = 0,        //竖线
    Horizontal,          //横线
    Cross,               //十字
    Ttype,               //T字形
    TurnTtype,           //倒T字形
    Ltype,               //L字形
    MirrorFlipLtype,     //镜像翻转L字形
    TurnLtype,           //倒L字形
    MirrorFlipTurnLtype, //镜像翻转倒L字形
    LeftTtype,
    ApriTag = 100,
    Pole = 200,
	BLOB = 201,
    Cirle = 202,
    FirstOrdCode = 203,
	fire = 204,
    SecondOrdCode,

    NumofForm,
} FormType_t;

typedef enum
{
    ActionWaitting = 0,
    ActionCountdown,
    ActionTakeOff,
    ActionHoverStartPoint,

    //飞行动作
    ActionGoForward,
    ActionGoRight,
    ActionGoLeft,
    ActionGoBack,
    ActionGoRound,
    ActionGoForward2,
    ActionFindPole,
    ActionCloseToPole,
    ActionTurnRound,
    ActionFindLandSpace,
    ActionResetAngle,
    ActionPreLand,
    
    //悬停动作
    ActionHoldLtype,
    ActionHoldMirrorFlipLtype,
    ActionHoldTurnLtype,
    ActionHoldMirrorFlipTurnLtype,
    ActionHoldCross,
    ActionHoldTtype,
    ActionHoldTurnTtype,
    ActionHoldFeaturePoint,
    ActionHoldLeftTtype,
    ActionHoldApriTag,

 
    ActionStop,
    ActionGoRight2,
    ActionGoRight3,
    ActionGoRight4,

    //寻杆步骤
    ActionFindPoleStep1,
    ActionFindPoleStep2,
    ActionFindPoleStep3,
    ActionFindPoleStep4,
    ActionFindPoleStep5,
    ActionFindPoleStep6,

    //给黄色一维码拍照
    ActionFindoneDimensionalCodeStep1,
    ActionFindoneDimensionalCodeStep2,
    ActionFindoneDimensionalCodeStep3,

    ActionRotateServer,
    ActionHoverStopPoint,
    ActionFollowTarget,
    ActionLostTargetInfo,
    ActionLand,
    ActionLock,
    ActionTest,
    ActionSonar,
    
    //Follow Point
    ActionWayPoint0,
    ActionWayPoint1,
    ActionWayPoint2,
    ActionWayPoint3,

    ActionEmergencyLand,
	
	//巡逻状态
	patrol,
	fire_fighting,
	back,
	//灭火状态
	go,
	stop,
	recover,
    NumofActionList,
} FSMList_t;

typedef enum
{
    ActionTimes1 = 0,
    ActionTimes2,
    ActionTimes3,
    ActionTimes4,
} ActionTimes_t;

typedef struct
{
    int16_t x1;
    int16_t y1;
    int16_t x2;
    int16_t y2;
} Line_t;

typedef struct
{
    uint16_t x1;
    uint16_t y1;
} Point_t;

//数据结构声明
#pragma pack(1)
typedef struct
{
    uint8_t Start;  //1
    FormType_t FormType; //1
    Point_t CentPoint;   //4
    uint8_t End;    //1
} Ground_OpenMVFrame_t;  //俯视openmv

typedef struct
{
    uint8_t Start;
    FormType_t FormType;
    Point_t point_1;
	Point_t point_2;
	uint16_t pole_dist;
    uint8_t End;
} Front_OpenMVFrame_t;  //前视openmv
#pragma pack()

typedef struct
{
    uint16_t SonarF;
    uint16_t SonarB;
    uint16_t SonarL;
    uint16_t SonarR;
} SonarManager_t;

typedef enum
{
    searching = 0,
    done,
    fail,
    frame_err,

    list_searching,
} enSearch_state_t;

typedef struct
{
	long  x:24;
	long  y:24;
}patrol_goal_t;


typedef struct
{
    //OpenMV数据帧   图像的形状、位置
    Front_OpenMVFrame_t *FrontOpenmvFramePtr;
    Ground_OpenMVFrame_t *GroundOpenmvFramePtr;
    Front_OpenMVFrame_t *ptrFrame;
    //飞机的状态
    UAV_info_t *ptrUAVInfo;

    //动作序列
    FSMList_t ActionList;

    //CountDownNumMs倒计时数据
    int16_t CountDownNumMs;
    int16_t TargetAltitudeCM;
    int16_t WatchDogCnt;

    //3个PID控制器
    PIDInfo_t *ptrPIDInfoV;
    PIDInfo_t *ptrPIDInfoH;
    PIDInfo_t *ptrPIDInfoY;

	PIDInfo_t *ptrPIDposition_x;
	PIDInfo_t *ptrPIDposition_y;
    float distance;
	

	
    bool ActionComplete;
} FollowManager_t;
extern uint8_t fire_fighting_flag;
extern bool FollowApriTag;
extern FollowManager_t FollowManager;
extern SonarManager_t SonarManager;
extern Point_t last_openmv;
void UpdateCentControl(float dt);
void Follow_Init(void);
extern void HoldCurrentPostion(float dt);
extern float absolute(float x);
extern patrol_goal_t start_position;
extern patrol_goal_t fire_position;
#endif
