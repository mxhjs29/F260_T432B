#include "power.h"
#include "main.h"
//#include "timer_drv.h"
//#include "led.h"

#define IS_POWER_BUTTON_ENABLE (P4IN & BIT1)
#define POWER_MANAGER_ON (P4OUT |= GPIO_PIN6)
#define POWER_MANAGER_OFF (P4OUT &= ~GPIO_PIN6)

typedef enum
{
    power_on = 0,
    power_off,
} pwoer_status_t;

bool power_update_enable = false;
pwoer_status_t pwoer_status;
void power_init(void)
{
    //p4.1为输入,内部拉高
    P4DIR &= ~(1 << BIT1);
    MAP_GPIO_setAsInputPinWithPullUpResistor(GPIO_PORT_P4, GPIO_PIN1);

    //P4.6为输出
    P4DIR |= GPIO_PIN6;
    pwoer_status = power_on;
    
    if (!IS_POWER_BUTTON_ENABLE)
    {
        POWER_MANAGER_ON;
        while(!IS_POWER_BUTTON_ENABLE);
        power_update_enable = true;
    }
}

static int cnt = 0;
void power_update(void)
{
    if(!power_update_enable)
        return ;
    
        if (!IS_POWER_BUTTON_ENABLE)
        {
            cnt++;
            
            //2s后关机
            if(cnt > 2000)
            {
                cnt = 0;
                LED_RGB_R_ON;
                Delay_ms(100);
                LED_RGB_R_OFF;
                Delay_ms(100);
                LED_RGB_R_ON;
                Delay_ms(100);
                LED_RGB_R_OFF;
                Delay_ms(100);
                POWER_MANAGER_OFF;
                while(1);
            }
        }else
        {
            cnt = 0;
        }
}
