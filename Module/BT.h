#include "stdio.h"
#include "stdint.h"
#include "stdbool.h"

typedef struct
{
    uint16_t start;
    int16_t R_x;
    int16_t R_y;
    uint16_t finger[5];
    uint16_t end;
}bt_t;

typedef union
{
    bt_t bt;
    uint8_t rx_buff[sizeof(bt_t)];
}u_bt_t;

extern u_bt_t bt;

void bt_init(void);
void bt_update(void);
