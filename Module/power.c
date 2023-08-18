#include "power.h"
#include "main.h"
//#include "timer_drv.h"
#include "led.h"

#define IS_POWER_BUTTON_ENABLE (GPIOE->IDR & GPIO_PIN_6)
#define POWER_MANAGER_ON (GPIOE->ODR |= GPIO_PIN_5)
#define POWER_MANAGER_OFF (GPIOE->ODR &= ~GPIO_PIN_5)

typedef enum
{
    power_on = 0,
    power_off,
} pwoer_status_t;

bool power_update_enable = false;
pwoer_status_t pwoer_status;
void power_init(void)
{
    //B15为输入,内部拉高
//    P4DIR &= ~(1 << BIT1);
//    MAP_GPIO_setAsInputPinWithPullUpResistor(GPIO_PORT_P4, GPIO_PIN1);

    //E2为输出
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
            if(cnt > 100)
            {
                cnt = 0;
                LED_RGB_R_ON;
                HAL_Delay(100);
                LED_RGB_R_OFF;
                HAL_Delay(100);
                LED_RGB_R_ON;
                HAL_Delay(100);
                LED_RGB_R_OFF;
                HAL_Delay(100);
                POWER_MANAGER_OFF;
                while(1);
            }
        }else
        {
            cnt = 0;
        }
}
