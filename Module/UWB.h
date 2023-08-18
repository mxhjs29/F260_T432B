#ifndef _UWB_H
#define _UWB_H

#include "stdint.h"
#include "stdbool.h"
#include "myMath.h"
//#include "MoudleBase.h"

#define USE_NOOPLOOP_UWB

#pragma pack (1)  
typedef struct
{
    uint8_t frame_head;
    uint8_t function_mark;
	uint16_t frame_length;
	uint8_t role;
    uint8_t id;
    uint32_t system_time;
    long pos_x:24;
    long pos_y:24;
    long pos_z:24;
    
} UWB_Manager_t;
#pragma pack ()

//typedef union
//{
//    uint8_t raw_buff[200];
//    UWB_Buff2_t *UWB_Buff2Ptr;
//}UWB_Buff2union_t;

//typedef struct
//{
//    Moudle_t moudle;
//    
//    Vector3f_t fuse_location;
//    Vector3i_t current_coordinate;
//    Vector3i_t vel;
//    Vector3f_t current_coordinate_LPF;
//    Vector3f_t last_coordinate;
//    uint8_t Count;
//    uint8_t single_lost_countdown;
//    bool update_handle;

//    UWB_Buff2union_t UWB_Buff2union;
//    
//} UWB_Manager_t;

//extern UWB_Manager_t UWB_Manager;
extern UWB_Manager_t UWB_Manager;
bool uwb_init(void);
void update_uwbhandle(void);

#endif
