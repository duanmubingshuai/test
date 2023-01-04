
#include "rom_sym_def.h"
#include "mcu.h"
#include "bus_dev.h"
#include "gpio.h"
#include "gpio_debounce_key.h"
#include "log.h"

void gpio_interrupt_disable(gpio_pin_e pin);
void gpio_interrupt_enable(gpio_pin_e pin, gpio_polarity_e type);

#define GDK_NUM_MAX 8

typedef struct{
    uint8_t init_flg;
    uint8_t key_num;
    gdkey_cb_t cb;
    gdkey_t* key;
}gdkey_ctx_t;

gdkey_ctx_t s_gdk_ctx = {
    .init_flg = 0,
    .cb = NULL,
    .key_num = 0,
    .key = NULL,
};

uint8_t gdkey_task_id = 0;

#define GDK_DEBOUNCE_INTV  20
#define GDK_PERIOD_CHK_INTV  200

#define GDK_PERIOD_CHK_EVT  BIT(GDK_NUM_MAX)

void gdkey_gpioin_hdl(gpio_pin_e pin,gpio_polarity_e type)
{
    uint8_t i;
    gdkey_t* key = s_gdk_ctx.key;
    //get index
    _HAL_CS_ALLOC_(); HAL_ENTER_CRITICAL_SECTION();
    for(i = 0; i < s_gdk_ctx.key_num; i++ )
        if(pin == key[i].pin) break;

    if(i == s_gdk_ctx.key_num)
    {
        LOG("gdkey unexpect trigger!!!\n");
        while(1) {;}
    }

    LOG("hdl %d,%d\n", key[i].debouce_state, type);

    key[i].debouce_state = 1;
    key[i].pin_state = ( type == POL_FALLING ) ? 0 : 1;

    //do debounce
    osal_start_timerEx(gdkey_task_id, BIT(i), GDK_DEBOUNCE_INTV);
    osal_clear_event(gdkey_task_id, GDK_PERIOD_CHK_EVT);

    //disable pin interrupt and sleep lock
    gpio_interrupt_disable(pin);
    pwrmgr_lock(MOD_GPIO);

    HAL_EXIT_CRITICAL_SECTION();
    
}


int gdkey_debounce_chk(uint8_t idx)
{
    uint8_t i;
    gdkey_t* pkey = &(s_gdk_ctx.key[idx]);
    int key_val = gpio_read(pkey->pin);
    LOG("chk %d, %d\n", key_val, pkey->pin_state);

    if(key_val == pkey->pin_state){
        gdkey_evt_e evt = (pkey->pin_state == 0) ? GDKEY_PRESS:GDKEY_RELEASE;
        s_gdk_ctx.cb(pkey->pin, evt);
    }

    pkey->debouce_state = 0;
    pkey->pin_state = key_val;
    osal_start_timerEx(gdkey_task_id, GDK_PERIOD_CHK_EVT, GDK_PERIOD_CHK_INTV);
    osal_clear_event(gdkey_task_id, GDK_PERIOD_CHK_EVT);

    gpio_polarity_e polarity = (key_val==0) ? POL_RISING : POL_FALLING;
    gpio_interrupt_enable(pkey->pin, polarity);
    for(i = 0; i< s_gdk_ctx.key_num; i++){
        if(s_gdk_ctx.key[i].debouce_state)
            return PPlus_SUCCESS;
    }
    pwrmgr_unlock(MOD_GPIO);
    return PPlus_SUCCESS;
}

int gdkey_period_chk(void)
{
    uint8_t i;
    int key_val;
    gdkey_t* key = s_gdk_ctx.key;
    for(i = 0; i< s_gdk_ctx.key_num; i++){
        key_val = gpio_read(key[i].pin);
        if(key_val != key[i].pin_state)
        {
            gdkey_evt_e evt = (key_val == 0) ? GDKEY_PRESS:GDKEY_RELEASE;
            s_gdk_ctx.cb(key[i].pin, evt);
            key[i].debouce_state = 0;
            key[i].pin_state = key_val;
        }
    }
    osal_start_timerEx(gdkey_task_id, GDK_PERIOD_CHK_EVT, GDK_PERIOD_CHK_INTV);
	return PPlus_SUCCESS;
}


int gdkey_register(gdkey_t* key, int num, gdkey_cb_t cb)
{
    uint8_t i;
    if(s_gdk_ctx.init_flg != 0)
        return PPlus_ERR_INVALID_STATE;
    if(num > GDK_NUM_MAX)
        return PPlus_ERR_INVALID_PARAM;
    s_gdk_ctx.key = key;
    s_gdk_ctx.key_num = num;
    s_gdk_ctx.cb = cb;

    //init key state
    for(i = 0; i< num; i++)
    {
        gpio_dir(key[i].pin, GPIO_INPUT);
        gpio_pull_set(key[i].pin, (gpio_pupd_e)(key[i].pull_set));
    }
    return PPlus_SUCCESS;

}

void gdkey_init(uint8 task_id)
{
    uint8_t i;
    gdkey_task_id = task_id;
    s_gdk_ctx.init_flg = 1;
    gdkey_t* key = s_gdk_ctx.key;

    for(i = 0; i< s_gdk_ctx.key_num; i++){
        key[i].pin_state = (uint8_t)gpio_read(key[i].pin);
        key[i].debouce_state = 0;
        int ret = gpioin_register(key[i].pin, gdkey_gpioin_hdl, gdkey_gpioin_hdl);
		LOG("ret = %d\n", ret);
    }

    osal_start_timerEx(gdkey_task_id, GDK_PERIOD_CHK_EVT, GDK_PERIOD_CHK_INTV);
	
}


uint16 gdkey_processevent( uint8 task_id, uint16 events )
{
    int i;
	if(task_id != gdkey_task_id){
		return 0;
	}
    if ( events & GDK_PERIOD_CHK_EVT) 
    {
        //LOG("GDK_PERIOD_CHK_EVT\n", events);
        gdkey_period_chk();
        return ( events ^ GDK_PERIOD_CHK_EVT);
    }
    for(i = 0; i< s_gdk_ctx.key_num; i++){
        if ( events & BIT(i) )
        {
            LOG("gdk evt %x\n", events);
            gdkey_debounce_chk(i);
            return ( events ^ BIT(i));
        }
    }
    return 0;
}


