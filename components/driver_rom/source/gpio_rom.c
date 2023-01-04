/*******************************************************************************
* @file		gpio.c
* @brief	Contains all functions support for gpio driver
* @version	0.0
* @date		27. Nov. 2017
* @author	qing.han
* 
* Copyright(C) 2016, PhyPlus Semiconductor
* All rights reserved.
*
*******************************************************************************/
#include "jump_function.h"
#include "bus_dev.h"
#include "gpio.h"
#include "pwrmgr.h"
#include <string.h>

typedef struct{
    gpio_pin_e pin;
    comm_cb_t cb;
}gpio_in_cfg_t;

gpio_in_cfg_t s_gpio_in_cfg = {GPIO_DUMMY,NULL};

gpio_Ctx_t m_gpioCtx = 
{
	.state = FALSE,
	.retention_map = 0,
	.gpioin_nums = 0,
	.gpioin_ctx = NULL,
};

/*
 * rom use:
 * *_r
*/
void gpio_init_r(void)
{
	AP_GPIO->swporta_dr = 0;
	AP_GPIO->swporta_ddr = 0;
	AP_GPIO->swporta_ctl = 0;

	memset(&m_gpioCtx, 0, sizeof(m_gpioCtx));	
 }

void gpio_dir_r(gpio_pin_e pin, gpio_dir_t dir)
{
	if((pin == P2) || (pin == P3))
    {
        gpio_fmux_set(pin,FMUX_P2_GPIO);
    }
    else
    {
        gpio_fmux_set(pin,FMUX_GPIO);
    }
	
	//gpio_fmux_control(pin,Bit_DISABLE); 		
	if(dir == GPIO_OUTPUT)
	{
		AP_GPIO->swporta_ddr |= BIT(pin);
	}
	else
	{
		AP_GPIO->swporta_ddr &= ~BIT(pin);
	}	
}

int gpio_in_trigger_r(gpio_pin_e pin, comm_cb_t in_cb)
{
    s_gpio_in_cfg.cb = in_cb;
    s_gpio_in_cfg.pin = pin;
    
    gpio_dir_r(pin, GPIO_INPUT);

    AP_GPIO->inttype_level |= BIT(pin);         //edge sensitive
    AP_GPIO->intmask &= ~ BIT(pin);             //unmask interrupt
    if (gpio_read(pin))                         //polarity
    {
        AP_GPIO->int_polarity &= ~ BIT(pin);    //fall edge
    }
    else
    {
        AP_GPIO->int_polarity |= BIT(pin);      //raise edge
    }
    AP_GPIO->inten |= BIT(pin);                 //interrupt enable
    
    NVIC_EnableIRQ(GPIO_IRQn);
    NVIC_SetPriority(GPIO_IRQn, IRQ_PRIO_HAL);

    return PPlus_SUCCESS;
}

void gpio_interrupt_set_r(gpio_pin_e pin, bit_action_e en)//rom use
{
    if(en)
    {
        AP_GPIO->inten |= BIT(pin);                 //interrupt enable
    }
    else
    {
        AP_GPIO->inten &= ~ BIT(pin);                 //interrupt disable
    }    
}

void GPIO_IRQHandler_r(void)
{
    uint32 st = AP_GPIO->int_status;
    
    //clear interrupt
    AP_GPIO->porta_eoi = st;
    if(s_gpio_in_cfg.cb){
        if(st & BIT(s_gpio_in_cfg.pin)){
            comm_evt_t evt;
            if((AP_GPIO->ext_porta) & BIT(s_gpio_in_cfg.pin))
            {
                evt.type = GPIO_EVT_EDGE_RISING;
                AP_GPIO->int_polarity &= ~ BIT(s_gpio_in_cfg.pin);
            }
            else
            {
                evt.type = GPIO_EVT_EDGE_FALLING;
                AP_GPIO->int_polarity |= BIT(s_gpio_in_cfg.pin);
            }
            
            s_gpio_in_cfg.cb(&evt);
        }
    }
}

// /*make hw IO retention active or deactive*/
void gpio_retention_active(gpio_pin_e pin, bool en)
{
    if(en)
    {
        AP_AON->IOCTL[2] |= BIT(pin); 
    }
    else
    {
        AP_AON->IOCTL[2] &= ~BIT(pin); 
    }
}

/*
 * */
void gpio_wakeup_control(gpio_pin_e pin, bit_action_e value)
{
    if (value) 
    {
        AP_AON->REG_S9 |= BIT(pin);	
    }
    else
    {
        AP_AON->REG_S9 &= ~BIT(pin);	
    }
}

void gpio_write(gpio_pin_e pin, bit_action_e en)
{   	
	if((pin == P2) || (pin == P3))
    {
        gpio_fmux_set(pin,FMUX_P2_GPIO);
    }
    else
    {
        gpio_fmux_set(pin,FMUX_GPIO);
    }

    if(en)
    {
        AP_GPIO->swporta_dr |= BIT(pin);
    }
    else
    {
        AP_GPIO->swporta_dr &= ~BIT(pin);
    }

    gpio_dir(pin,GPIO_OUTPUT);
}

//void gpio_fast_write(gpio_pin_e pin, uint8_t en)
//{
//    if(en)
//    {
//        AP_GPIO->swporta_dr |= BIT(pin);
//    }
//    else
//    {
//        AP_GPIO->swporta_dr &= ~BIT(pin);
//    }
//}

bool gpio_read(gpio_pin_e pin)
{
    uint32_t r;

    if(AP_GPIO->swporta_ddr & BIT(pin))
    {
        r = AP_GPIO->swporta_dr;
    }
    else
    {
        r = AP_GPIO->ext_porta;
    }

    return (int)((r>> pin) & 1);
}

void gpio_dir(gpio_pin_e pin, gpio_dir_t dir)
{	
    if((pin == P2) || (pin == P3))
    {
        gpio_fmux_set(pin,FMUX_P2_GPIO);
    }
    else
    {
        gpio_fmux_set(pin,FMUX_GPIO);
    }

    if(dir == GPIO_OUTPUT)
    {
        AP_GPIO->swporta_ddr |= BIT(pin);
    }
    else
    {
        AP_GPIO->swporta_ddr &= ~BIT(pin);
        gpio_retention(pin, false);
    }
}

void gpio_pull_set(gpio_pin_e pin, gpio_pupd_e type)
{
    uint8_t h = 0,l = 0;
    uint32_t reg_index = 0;
    uint32_t bit_index = 0;

    if(pin > P9)
    {
        reg_index = 1;
    }

    bit_index = pin % 10;
    l = bit_index *3 + 1;
    h = l+1;

    subWriteReg(&(AP_AON->IOCTL[reg_index]),h,l,type);
}

void gpio_wakeup_set(gpio_pin_e pin, gpio_polarity_e type)
{
    uint8_t p = 0;
	uint32_t reg_index = 0;
	uint32_t bit_index = 0;

    if (m_gpioCtx.retention_map & BIT(pin))
       return;

     if(pin > P9)
     {
         reg_index = 1;
     }
     bit_index = pin % 10;
     p = bit_index *3;

    AP_GPIO->inttype_level |= BIT(pin);//edge sensitive

    if(POL_FALLING == type)
    {
        AP_AON->IOCTL[reg_index] |= BIT(p);
    }       
    else
    {
        AP_AON->IOCTL[reg_index] &= ~BIT(p);
    }

   gpio_wakeup_control(pin,Bit_ENABLE);//enable wakeup function
}

void gpio_ds_control(gpio_pin_e pin, bit_action_e value)
{
    if(value)	
    {
        AP_IOMUX->pad_ps0 |= BIT(pin);	
    }
    else	
    {
        AP_IOMUX->pad_ps0 &= ~ BIT(pin);	
    }
}

void gpio_fmux_control(gpio_pin_e pin, bit_action_e value)
{
    if(value)
    {
        AP_IOMUX->full_mux0_en |= BIT(pin);
    }
    else
    {
        AP_IOMUX->full_mux0_en &= ~BIT(pin);
    }
}

void gpio_fmux_set(gpio_pin_e pin,gpio_fmux_e type)
{
	uint8_t h = 0,l = 0;
	uint32_t reg_index;
	uint32_t bit_index;
	
    reg_index = pin / 4;
    bit_index = pin % 4;
    l = 8 * bit_index;

	if((pin == P2) || (pin == P3) || (pin == P4) || (pin == P5)) 
	{
		h = l + 2;
	}
    else
    {
    	h = l + 1;
    }

    subWriteReg(&(AP_IOMUX->gpio_sel[reg_index]),h,l,type);
    gpio_fmux_control(pin,Bit_ENABLE);
}

void gpio_retention(gpio_pin_e pin,bool en)
{
    if(en)
    {
        m_gpioCtx.retention_map |= BIT((int)pin);
    }
    else
    {
        m_gpioCtx.retention_map &= ~ BIT((int)pin);
    }
}

// /**********************************/
/*static*/ void gpio_interrupt_disable(gpio_pin_e pin)
{
    subWriteReg(&(AP_GPIO->intmask),pin,pin,1);
    subWriteReg(&(AP_GPIO->inten),pin,pin,0);
}

void gpio_interrupt_enable(gpio_pin_e pin, gpio_polarity_e type)
{
    uint32_t gpio_tmp;

    gpio_tmp = AP_GPIO->inttype_level;
    gpio_tmp |= (1 << pin); //edge sensitive
    AP_GPIO->inttype_level = gpio_tmp;
    gpio_tmp = AP_GPIO->intmask;
    gpio_tmp &= ~(1 << pin); //unmask interrupt
    AP_GPIO->intmask = gpio_tmp;
    gpio_tmp = AP_GPIO->int_polarity;

    if (type == POL_RISING )
        gpio_tmp |= (1 << pin);
    else
        gpio_tmp &= ~(1 << pin);

    AP_GPIO->int_polarity = gpio_tmp;
    gpio_tmp = AP_GPIO->inten;
    gpio_tmp |= (1 << pin); //enable interrupt
    AP_GPIO->inten = gpio_tmp;
}

int gpioin_disable(gpioin_t* p_gpioin_ctx)
{
    if(p_gpioin_ctx){
        p_gpioin_ctx->enable = FALSE;
        //m_gpioCtx.retention_map = GPIO_PIN_ASSI_NONE;
        //gpio_dir(pin, GPIO_INPUT);
        gpio_interrupt_disable(p_gpioin_ctx->pin);
        return PPlus_SUCCESS;
    }
    return PPlus_ERR_NOT_FOUND;
}

int gpioin_enable(gpioin_t* p_gpioin_ctx)
{
    gpio_polarity_e type = POL_FALLING;

    if(p_gpioin_ctx){
        gpio_pin_e pin = p_gpioin_ctx->pin;
        if (p_gpioin_ctx->posedgeHdl == NULL && p_gpioin_ctx->negedgeHdl == NULL)
            return PPlus_ERR_NOT_REGISTED;

        p_gpioin_ctx->enable = TRUE;
        gpio_dir(pin, GPIO_INPUT);

        if (p_gpioin_ctx->posedgeHdl && p_gpioin_ctx->negedgeHdl) //both raise and fall
        {
            uint32 pinVal = 0;
            pinVal = gpio_read(pin);
            type = pinVal ? POL_FALLING : POL_RISING;
        }
        else if (p_gpioin_ctx->posedgeHdl) //raise
        {
            type = POL_RISING ;
        }
        else if (p_gpioin_ctx->negedgeHdl) //fall
        {
            type = POL_FALLING;
        }

        gpio_interrupt_enable(pin, type);

        return PPlus_SUCCESS;
    }

    return PPlus_ERR_NOT_FOUND;
}

void gpioin_event_pin(gpioin_t* p_gpioin_ctx, gpio_polarity_e type)
{
    if (p_gpioin_ctx->posedgeHdl && (type == POL_RISING ))
    {
        p_gpioin_ctx->posedgeHdl(p_gpioin_ctx->pin,POL_RISING );//LOG("POS\n");
    }
    else if (p_gpioin_ctx->negedgeHdl && (type == POL_FALLING))
    {
        p_gpioin_ctx->negedgeHdl(p_gpioin_ctx->pin,POL_FALLING);//LOG("NEG\n");
    }
}

void gpioin_event(uint32 int_status, uint32 polarity)
{
    int i;
    gpio_pin_e pin = 0;
    gpioin_t* p_gpioin_ctx = m_gpioCtx.gpioin_ctx;

    for (i = 0; i < m_gpioCtx.gpioin_nums; i++)
    {
        pin = p_gpioin_ctx[i].pin;
        if(pin == GPIO_DUMMY)
            continue;

        if(int_status & BIT(pin))
        {
            gpio_polarity_e type = (polarity & BIT(pin)) ? POL_RISING : POL_FALLING;
            gpioin_event_pin(p_gpioin_ctx+i, type);

            //reconfig interrupt
            if (p_gpioin_ctx[i].posedgeHdl && p_gpioin_ctx[i].negedgeHdl) //both raise and fall
            {
                type = (type == POL_RISING) ? POL_FALLING : POL_RISING ;
                gpio_interrupt_enable(pin, type);
            }
            else if (p_gpioin_ctx[i].posedgeHdl) //raise
            {
                gpio_interrupt_enable(pin, POL_RISING);
            }
            else if (p_gpioin_ctx[i].negedgeHdl) //fall
            {
                gpio_interrupt_enable(pin, POL_FALLING);
            }
        }
    }
}

void gpio_sleep_handler(void)
{
    int i;
    gpioin_t* p_gpioin_ctx = m_gpioCtx.gpioin_ctx;
    gpio_polarity_e pol;

//    for (i = 0; i < GPIO_NUM; i++)
//    {
//        //config retenstion setting
//        if (m_gpioCtx.retention_map & BIT(i))
//        {
//            gpio_retention_active((gpio_pin_e)i, true);
//        }
//    }
    AP_AON->IOCTL[2] = m_gpioCtx.retention_map;
    //config wakeup setting
    for (i = 0; i < m_gpioCtx.gpioin_nums; i++)
    {
        pol = gpio_read(p_gpioin_ctx[i].pin) ? POL_FALLING : POL_RISING ;
        gpio_wakeup_set(p_gpioin_ctx[i].pin, pol);
        m_gpioCtx.gpioin_ctx[i].pin_state = gpio_read(p_gpioin_ctx[i].pin);
    }
}

void gpioin_wakeup_trigger(gpioin_t* p_gpioin_ctx)
{
    gpio_polarity_e type = p_gpioin_ctx->pin_state ?  POL_FALLING : POL_RISING;
    int pin_state = gpio_read(p_gpioin_ctx->pin);

    gpioin_event_pin(p_gpioin_ctx, type);

    //below means gpio toggle is very shot, when wakeuped, 
    //the pin level have been same as before sleeping
    //so we need make up the missed edge
    if(p_gpioin_ctx->pin_state == pin_state){
        type = pin_state ? POL_FALLING : POL_RISING; //order reverted!!!
        gpioin_event_pin(p_gpioin_ctx, type);
    }
}

void gpio_wakeup_handler(void)
{
   int i;

   gpioin_t* p_gpioin_ctx = m_gpioCtx.gpioin_ctx;
   NVIC_SetPriority(GPIO_IRQn, IRQ_PRIO_HAL);
   NVIC_EnableIRQ(GPIO_IRQn);

   for (i = 0; i < GPIO_NUM; i++)
   {
        if((i == P2) || (i == P3))
        {
            gpio_fmux_set(i,FMUX_P3_GPIO);
        }
       if (m_gpioCtx.retention_map & BIT(i))
       {
           bool pol = gpio_read((gpio_pin_e)i);
           gpio_write((gpio_pin_e)i,pol);
       }
       gpio_retention_active((gpio_pin_e)i, false);
   }

   //get wakeup source
   for (i = 0; i < m_gpioCtx.gpioin_nums; i++)
   {
       if (p_gpioin_ctx[i].enable)
       {
           gpioin_enable(p_gpioin_ctx+i); //resume gpio irq
          if(s_gpio_wakeup_src & BIT(p_gpioin_ctx[i].pin))
			{
               gpioin_wakeup_trigger(p_gpioin_ctx+i);//trigger gpio irq manually
			}
       }
   }
}

void GPIO_IRQHandler(void)
{
    uint32 polarity = AP_GPIO->int_polarity;
    uint32 st = AP_GPIO->int_status;

    AP_GPIO->porta_eoi = st;//clear interrupt
    gpioin_event(st, polarity);
}

int gpioin_init(gpioin_t* p_gpioin_ctx, int pin_num)
{
    int i;

    if(m_gpioCtx.gpioin_nums)
        return PPlus_ERR_HAVE_REGISTED;

    for(i = 0; i< pin_num; i++){
        p_gpioin_ctx[i].enable = false;
        p_gpioin_ctx[i].pin = GPIO_DUMMY;
        p_gpioin_ctx[i].pin_state = 0;
        p_gpioin_ctx[i].posedgeHdl = NULL;
        p_gpioin_ctx[i].negedgeHdl = NULL;
    }

    m_gpioCtx.gpioin_ctx = p_gpioin_ctx;
    m_gpioCtx.gpioin_nums = pin_num;
    return PPlus_SUCCESS;  
}

int gpioin_register(gpio_pin_e pin, gpioin_Hdl_t posedgeHdl, gpioin_Hdl_t negedgeHdl)
{
    int i;
    gpioin_t* p_gpioin_ctx = m_gpioCtx.gpioin_ctx;

    for(i = 0; i< m_gpioCtx.gpioin_nums ; i++){
        if(p_gpioin_ctx[i].pin == pin)
            return PPlus_ERR_HAVE_REGISTED;
    }

    for(i = 0; i< m_gpioCtx.gpioin_nums ; i++){
        if(p_gpioin_ctx[i].pin == GPIO_DUMMY){
            p_gpioin_ctx[i].pin = pin;
            p_gpioin_ctx[i].pin_state = 0;
            p_gpioin_ctx[i].posedgeHdl = posedgeHdl;
            p_gpioin_ctx[i].negedgeHdl = negedgeHdl;
            return gpioin_enable(&p_gpioin_ctx[i]);
        }
    }
    return PPlus_ERR_NO_MEM;
}

int gpioin_unregister(gpio_pin_e pin)
{
    int i;
    gpioin_t* p_gpioin_ctx = m_gpioCtx.gpioin_ctx;

    for(i = 0; i< m_gpioCtx.gpioin_nums ; i++){
        if(p_gpioin_ctx[i].pin == pin){
            gpioin_disable(&p_gpioin_ctx[i]);
            p_gpioin_ctx[i].pin = GPIO_DUMMY;
            p_gpioin_ctx[i].pin_state = 0;
            p_gpioin_ctx[i].posedgeHdl = NULL;
            p_gpioin_ctx[i].negedgeHdl = NULL;
            return PPlus_SUCCESS;
        }
    }

    return PPlus_ERR_NOT_REGISTED;
}

int gpio_init(void)
{
    if (m_gpioCtx.state)
        return PPlus_ERR_INVALID_STATE;

    if(JUMP_FUNCTION_GET(GPIO_IRQ_HANDLER) == JUMP_BASE_ADDR)
        return PPlus_ERR_NOT_SUPPORTED;

    memset(&m_gpioCtx, 0, sizeof(m_gpioCtx));
    m_gpioCtx.state = TRUE;

    //disable all channel irq,unmask all channel
    AP_GPIO->inten = 0;
    AP_GPIO->intmask = 0;

    //disable all wakeup pin
    AP_WAKEUP->io_wu_mask_31_0 = 0;
    AP_WAKEUP->io_wu_mask_34_32 = 0;

    NVIC_SetPriority(GPIO_IRQn, IRQ_PRIO_HAL);
    NVIC_EnableIRQ(GPIO_IRQn);

    pwrmgr_register(MOD_GPIO, gpio_sleep_handler, gpio_wakeup_handler);

    return PPlus_SUCCESS;
}
