#include "UWB.h"
#include "usart.h"
#include "string.h"

UWB_Manager_t UWB_Manager;
//理论上，现在只要数据发送过来，就能直接生成坐标
bool uwb_init(void)
{
//    HAL_UART_Receive_DMA(&huart1, (uint8_t*)UWB_Manager, 128);
    UWB_Manager.pos_x = 0;
	UWB_Manager.pos_y = 0;
	UWB_Manager.pos_z = 0;
    return true;
}

////此函数就是检查UWB模块是否在线
//void get_buff_offset(void)
//{

//}



//void update_uwbhandle()
//{
//    static uint8_t offset = 0;
//    uint8_t buff[30];
//    static uint8_t last_data = 0;
//    static uint32_t count = 0;
//    static float UWB_x = 0;
//    static float UWB_y = 0;
//    
//    //上次的偏移是0吗？
//    if(offset == 0 && UWB_Manager.UWB_Buff2union.raw_buff[0] != 0x55)
//    {
//        //先找到0x55
//        for(int i = 0;i < 129;i++)
//        {
//            //判断开头和帧结构
//            if(UWB_Manager.UWB_Buff2union.raw_buff[i] == 0x55 && (UWB_Manager.UWB_Buff2union.raw_buff[i + 1] == 1))
//            {
//                //再计算偏移
//                offset = i;
//                break;
//            }
//        }
//    }
//    
//    if(UWB_Manager.UWB_Buff2union.raw_buff[4 + offset] - last_data == 0)
//    {
//        count++;
//    }else
//    {
//        count = 0;
//    }
//    
//    last_data = UWB_Manager.UWB_Buff2union.raw_buff[4 + offset];
//    
//    if(count > 50)
//    {
//        count = 60;
//        UWB_Manager.moudle.bCheck = false;
//    }else
//    {
//        UWB_Manager.moudle.bCheck = true;
//    }
//    
//    //再计算超过129的部分
//    if(offset + 4 > 129)
//    {
//        int offset2 = 129 - 4 - offset;
//        memcpy(UWB_Manager.UWB_Buff2union.raw_buff + 129, UWB_Manager.UWB_Buff2union.raw_buff, offset2);
//    }
//    
//    memcpy(buff, UWB_Manager.UWB_Buff2union.raw_buff + 4 + offset, 30);
//    
//    //再输出坐标
//    UWB_Manager.current_coordinate.x = buff[2] << 16| buff[1] << 8| buff[0];
//    UWB_Manager.current_coordinate.y = buff[5] << 16| buff[4] << 8| buff[3];
//    UWB_Manager.current_coordinate.z = buff[8] << 16| buff[7] << 8| buff[6];
//    
//    UWB_Manager.vel.x = buff[2 + 7] << 16| buff[1 + 7] << 8| buff[0 + 7];
//    UWB_Manager.vel.y = buff[5 + 7] << 16| buff[4 + 7] << 8| buff[3 + 7];
//    UWB_Manager.vel.z = buff[8 + 7] << 16| buff[7 + 7] << 8| buff[6 + 7];
//    
//    //Unit:cm
//    UWB_Manager.current_coordinate.x /= 10;
//    UWB_Manager.current_coordinate.y /= 10;
//    UWB_Manager.current_coordinate.z /= 10;
//    
//    UWB_Manager.current_coordinate_LPF.x += 0.5 * (UWB_Manager.current_coordinate.x - UWB_Manager.current_coordinate_LPF.x);
//    UWB_Manager.current_coordinate_LPF.y += 0.5 * (UWB_Manager.current_coordinate.y - UWB_Manager.current_coordinate_LPF.y);

//    //融合UWB
//    status_estimate.imu_gnd_position_uwb.x += 0.4f * (UWB_Manager.current_coordinate_LPF.x - status_estimate.imu_gnd_position_uwb.x);
//    status_estimate.imu_gnd_position_uwb.y += 0.4f * (UWB_Manager.current_coordinate_LPF.y - status_estimate.imu_gnd_position_uwb.y);
//}

