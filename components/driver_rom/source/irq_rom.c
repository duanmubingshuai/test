
#include "bus_dev.h"
#include "mcu.h"
#include "jump_function.h"

int m_in_critical_region = 0;

void drv_irq_init0(void)
{
  m_in_critical_region = 0;
  __disable_irq();    
  //disable all channel
  HAL_ICER = 0xffffffff;
  __enable_irq();
  #if(PHY_MCU_TYPE==MCU_BUMBEE_CK802)
  __enable_excp_irq();//CK802 need enable excp irq,
  #endif
}
#if 0

#else
extern int cpu_intrpt_save();
extern int cpu_intrpt_restore(int);
int drv_disable_irq0(void)
{
  return cpu_intrpt_save();
}

int drv_enable_irq0(int cs)
{
  return cpu_intrpt_restore(cs);
}
//-----------------------------------------------------------
// only used for program.c 
int drv_disable_irqx(void)
{
  __disable_irq();    
  m_in_critical_region++;
  return m_in_critical_region;
}

int drv_enable_irqx(int cs)
{
  (void)cs;//reserved for irq restore 
  m_in_critical_region--;
  if (m_in_critical_region == 0)
  {
    __enable_irq();
  }
  return m_in_critical_region;
}
#endif

