#ifndef _DATA_TRANSFER_H
#define	_DATA_TRANSFER_H

#include "stdint.h"

typedef struct
{
    uint8_t msg_id;
    uint8_t msg_data;
    uint8_t send_check;
    uint8_t send_version;
    uint8_t send_status;
    uint8_t send_senser;
    uint8_t send_senser2;
    uint8_t send_rcdata;
    uint8_t send_offset;
    uint8_t send_motopwm;
    uint8_t send_power;
    uint8_t send_user;
    uint8_t send_speed;
    uint8_t send_location;
	uint8_t send_vef;
	uint16_t send_parame;
	uint16_t paraToSend;
} dt_flag_t;

#define PAR_PID_1_P		1
#define PAR_PID_1_I		2
#define PAR_PID_1_D		3
#define PAR_PID_2_P		4
#define PAR_PID_2_I		5
#define PAR_PID_2_D		6
#define PAR_PID_3_P		7
#define PAR_PID_3_I		8
#define PAR_PID_3_D		9
#define PAR_PID_4_P		10
#define PAR_PID_4_I		11
#define PAR_PID_4_D		12
#define PAR_PID_5_P		13
#define PAR_PID_5_I		14
#define PAR_PID_5_D		15
#define PAR_PID_6_P		16
#define PAR_PID_6_I		17
#define PAR_PID_6_D		18
#define PAR_PID_7_P		19
#define PAR_PID_7_I		20
#define PAR_PID_7_D		21
#define PAR_PID_8_P		22
#define PAR_PID_8_I		23
#define PAR_PID_8_D		24
#define PAR_PID_9_P		25
#define PAR_PID_9_I		26
#define PAR_PID_9_D		27
#define PAR_PID_10_P		28
#define PAR_PID_10_I		29
#define PAR_PID_10_D		30
#define PAR_PID_11_P		31
#define PAR_PID_11_I		32
#define PAR_PID_11_D		33
#define PAR_PID_12_P		34
#define PAR_PID_12_I		35
#define PAR_PID_12_D		36
#define PAR_PID_13_P		37
#define PAR_PID_13_I		38
#define PAR_PID_13_D		39
#define PAR_PID_14_P		40
#define PAR_PID_14_I		41
#define PAR_PID_14_D		42
#define PAR_PID_15_P		43
#define PAR_PID_15_I		44
#define PAR_PID_15_D		45
#define PAR_PID_16_P		46
#define PAR_PID_16_I		47
#define PAR_PID_16_D		48
#define PAR_PID_17_P		49
#define PAR_PID_17_I		50
#define PAR_PID_17_D		51
#define PAR_PID_18_P		52
#define PAR_PID_18_I		53
#define PAR_PID_18_D		54

#define PAR_RCINMODE		61
#define PAR_UNLOCKPWM		62
#define PAR_LVWARN			63
#define PAR_LVRETN			64
#define PAR_LVDOWN			65
#define PAR_CENPOSX			66
#define PAR_CENPOSY			67
#define PAR_CENPOSZ			68
#define PAR_TAKEOFFHIGH		69
#define PAR_TAKEOFFSPEED	70
#define PAR_LANDSPEED		71
#define PAR_HEATSWITCH		72


extern int32_t ParValList[100];

extern dt_flag_t f;
void ANO_DT_Data_Receive_Anl_Task(void);
void ANO_DT_Send_VER(void);
void ANO_DT_Data_Exchange(void);
void ANO_DT_Data_Receive_Prepare(uint8_t data);
void ANO_DT_Data_Receive_Anl(uint8_t *data_buf,uint8_t num);
void ANO_DT_Send_Version(uint8_t hardware_type, uint16_t hardware_ver,uint16_t software_ver,uint16_t protocol_ver,uint16_t bootloader_ver);
void ANO_DT_Send_Status(float angle_rol, float angle_pit, float angle_yaw, int32_t alt, uint8_t fly_model, uint8_t armed);
void ANO_DT_Send_Senser(int16_t a_x,int16_t a_y,int16_t a_z,int16_t g_x,int16_t g_y,int16_t g_z,int16_t m_x,int16_t m_y,int16_t m_z);
void ANO_DT_Send_Senser2(int32_t bar_alt,int32_t csb_alt, int16_t sensertmp);
void ANO_DT_Send_RCData(uint16_t thr,uint16_t yaw,uint16_t rol,uint16_t pit,uint16_t aux1,uint16_t aux2,uint16_t aux3,uint16_t aux4,uint16_t aux5,uint16_t aux6);
void ANO_DT_Send_Power(uint16_t votage, uint16_t current);
void ANO_DT_Send_MotoPWM(uint16_t m_1,uint16_t m_2,uint16_t m_3,uint16_t m_4,uint16_t m_5,uint16_t m_6,uint16_t m_7,uint16_t m_8);
void ANO_DT_Send_PID(uint8_t group,float p1_p,float p1_i,float p1_d,float p2_p,float p2_i,float p2_d,float p3_p,float p3_i,float p3_d);
void ANO_DT_Send_User(void);
void ANO_DT_Send_Speed(float,float,float);
void ANO_DT_Send_Location(uint8_t state,uint8_t sat_num,int32_t lon,int32_t lat,float back_home_angle,float back_home_dist);
void ANO_DT_SendCenterPos(float x,float y,float z);
void ANO_DT_SendParame(uint16_t num);
void ANO_DT_GetParame(uint16_t num,int32_t data);
void ANO_DT_SendCmd(uint8_t dest, uint8_t fun, uint16_t cmd1, uint16_t cmd2, uint16_t cmd3, uint16_t cmd4, uint16_t cmd5);
void ANO_DT_SendString(const char *str);
void ANO_DT_SendStrVal(const char *str, int32_t val);

#endif

