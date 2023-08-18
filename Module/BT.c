#include "bt.h"
#include "main.h"
#include "usart.h"
#include "sdk.h"
#include "HARDWARE_uart.h"

u_bt_t bt;

typedef enum
{
    bt_watting = 0,
    bt_enter_bt_mode,
    bt_unlock,
    bt_running,
    bt_land,
    bt_stop,
}bt_running_step_t;

void bt_init()
{
    //����ط���Ҫ����Ϊѭ������

}

// ��ȭ�Ǵ�ֵ
// �ſ���Сֵ
#define WOQUAN  1200
#define ZHANGKAI 1800
bt_running_step_t bt_running_step;
float target_alt = 0;
float target_y = 0;
float target_x = 0;
void bt_update()
{
    memcpy(bt.rx_buff, UsartGroup[UART_4].RxBuff, sizeof(bt));
    
    switch (bt_running_step)
    {
        case bt_watting:
            //do nothing
            if(bt.bt.start == 0xAAAA && bt.bt.end == 0x5555)
            {
                bt_running_step = bt_enter_bt_mode;
            }
            break;
        
        case bt_enter_bt_mode:
            {
                static int count = 0;
                //�����Ĵָ
                if(bt.bt.finger[0] > ZHANGKAI && 
                    bt.bt.finger[1] < WOQUAN &&
                    bt.bt.finger[2] < WOQUAN &&
                    bt.bt.finger[3] < WOQUAN &&
                    bt.bt.finger[4] < WOQUAN &&
                    bt.bt.R_y < -50
                    )
                
               
                {
                    count++;

                    if(count > 60)
                    {
                        count = 0;
                         target_alt = 0;
                        bt_running_step = bt_unlock;
                    }
                }else 
                {
                    count = 0;
                    /* code */
                }
            }
            break;

        case bt_unlock:
            sdk_unlock();
            if(bt.bt.finger[0] > ZHANGKAI && 
                    bt.bt.finger[1] > ZHANGKAI &&
                    bt.bt.finger[2] > ZHANGKAI &&
                    bt.bt.finger[3] > ZHANGKAI &&
                    bt.bt.finger[4] > ZHANGKAI)
                {
                    bt_running_step = bt_running;
                }
            break;
            
        case bt_running:
            {
                //climb
                if(bt.bt.finger[0] > 900 && 
                    bt.bt.finger[1] < WOQUAN &&
                    bt.bt.finger[2] < WOQUAN &&
                    bt.bt.finger[3] < WOQUAN &&
                    bt.bt.finger[4] < WOQUAN)
                {
                    
                    if(target_alt < 120)
                    {
                        target_alt += 0.03f;
                        sdk_alititude_set(target_alt);
                    }
                }

                //�½�
                if(bt.bt.finger[0] < WOQUAN && 
                    bt.bt.finger[1] < WOQUAN &&
                    bt.bt.finger[2] < WOQUAN &&
                    bt.bt.finger[3] < WOQUAN &&
                    bt.bt.finger[4] < WOQUAN)
                {
                    
                    if(target_alt > 30)
                    {
                        target_alt -= 0.01f;
                        sdk_alititude_set(target_alt);
                    }
                }

                //����
                if(bt.bt.finger[0] < WOQUAN && 
                    bt.bt.finger[1] > ZHANGKAI &&
                    bt.bt.finger[2] > ZHANGKAI &&
//                    bt.bt.finger[3] < WOQUAN &&
                    bt.bt.finger[4] < WOQUAN)
                {
                    if(bt.bt.R_y > 50)
                    {
                        target_x += 0.1f;
                        sdk_velociyt_x_set(10);
                    }else if (bt.bt.R_y < -50)
                    {
                        sdk_velociyt_x_set(-10);
                        target_x -= 0.1f;
                    }
                    

                }
                
                //ǰ��
                if(bt.bt.finger[0] > ZHANGKAI && 
                    bt.bt.finger[1] > ZHANGKAI &&
                    bt.bt.finger[2] > ZHANGKAI &&
                    bt.bt.finger[4] < WOQUAN)
                {
                    if(bt.bt.R_x > 50)
                    {
                        target_y += 0.1f;
                        sdk_velociyt_y_set(10);
                    }else if (bt.bt.R_x < -50)
                    {
                        target_y -= 0.1f;
                        sdk_velociyt_y_set(-10);
                    }
                }

                //����
                if(bt.bt.finger[0] > ZHANGKAI && 
                    bt.bt.finger[1] < WOQUAN &&
                    bt.bt.finger[2] < WOQUAN &&
                    bt.bt.finger[3] < WOQUAN &&
                    bt.bt.finger[4] < WOQUAN &&
                    bt.bt.R_y > 50)
                {
                    bt_running_step = bt_land;
                }
                
                //����ͣ��
                static int8_t emergecy_stop = 0;
                static int16_t timeout_count = 0;
                
                if(emergecy_stop > 0)
                {
                    timeout_count++;
                    
                    //3s�ڱ�����ɲ���
                    if(timeout_count > 100)
                    {
                        emergecy_stop = 0;
                    }
                }
                switch(emergecy_stop)
                {
                    case 0:
                        //��ָ��ȭ
                        if(bt.bt.finger[0] < WOQUAN && 
                            bt.bt.finger[1] < WOQUAN &&
                            bt.bt.finger[2] < WOQUAN &&
                            bt.bt.finger[3] < WOQUAN &&
                            bt.bt.finger[4] < WOQUAN)
                        {
                            emergecy_stop++;
                        }
                        break;
                        
                    case 1:
                        //��ָ�ſ�
                        if(bt.bt.finger[0] > ZHANGKAI && 
                            bt.bt.finger[1] > ZHANGKAI &&
                            bt.bt.finger[2] > ZHANGKAI &&
                            bt.bt.finger[3] > ZHANGKAI &&
                            bt.bt.finger[4] > ZHANGKAI)
                        {
                            emergecy_stop++;
                        }
                        break;
                    case 2:
                        //��ָ��ȭ
                        if(bt.bt.finger[0] < WOQUAN && 
                            bt.bt.finger[1] < WOQUAN &&
                            bt.bt.finger[2] < WOQUAN &&
                            bt.bt.finger[3] < WOQUAN &&
                            bt.bt.finger[4] < WOQUAN)
                        {
                            emergecy_stop++;
                        }
                        break;
                    case 3:
                        //��ָ�ſ�
                        if(bt.bt.finger[0] > ZHANGKAI && 
                            bt.bt.finger[1] > ZHANGKAI &&
                            bt.bt.finger[2] > ZHANGKAI &&
                            bt.bt.finger[3] > ZHANGKAI &&
                            bt.bt.finger[4] > ZHANGKAI)
                        {
                            emergecy_stop++;
                        }
                        break;
                    case 4:
                        //��ָ��ȭ
                        if(bt.bt.finger[0] < WOQUAN && 
                            bt.bt.finger[1] < WOQUAN &&
                            bt.bt.finger[2] < WOQUAN &&
                            bt.bt.finger[3] < WOQUAN &&
                            bt.bt.finger[4] < WOQUAN)
                        {
                            emergecy_stop = 0;
                            bt_running_step = bt_stop;
                        }
                        break;
                }
                
                    

            }
            break;
        case bt_land:
            sdk_land();
       
            bt_running_step = bt_watting;
            break;
        case bt_stop:
            sdk_lock();
            break;
        default:
            break;
    }
}