
#ifndef __GPIO_DEBC_KEY
#define __GPIO_DEBC_KEY

#include "gpio.h"

typedef enum{
    GDKEY_PRESS = 0,
    GDKEY_RELEASE = 1,
} gdkey_evt_e;

typedef void (*gdkey_cb_t)(gpio_pin_e, gdkey_evt_e);


typedef struct
{
    gpio_pin_e pin;
    uint8_t    pin_state; //0 or 1
    uint8_t    debouce_state; //0: idle;  1: debouncing
    uint8_t    pull_set;   //gpio_pupd_e
}gdkey_t;

int gdkey_register(gdkey_t* key, int num, gdkey_cb_t cb);
void gdkey_init(uint8 task_id);
uint16 gdkey_processevent( uint8 task_id, uint16 events );

#endif /*__GPIO_DEBC_KEY*/

