/*
 * Copyright (C) 2017 C-SKY Microsystems Co., Ltd. All rights reserved.
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *   http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

/******************************************************************************
 * @file     gcc_csky.ld
 * @brief    csky linker file
 * @version  V1.0
 * @date     02. June 2017
 ******************************************************************************/
MEMORY
{
	SRAM : ORIGIN = 0x1FFF0800 , LENGTH = 0x1800   /* SRAM  8KB */
	XIPROM : ORIGIN = 0x1FFF8400 , LENGTH = 0x3c00   /* XIPROM  16KB */
}

__min_heap_size = 0x10;
PROVIDE (__ram_end  = 0x1FFF2000);
PROVIDE (__heap_end = __ram_end);

REGION_ALIAS("REGION_ENTRY",   SRAM);
REGION_ALIAS("REGION_TEXT",    XIPROM);
REGION_ALIAS("REGION_RODATA",  XIPROM);
REGION_ALIAS("REGION_DATA",    SRAM);
REGION_ALIAS("REGION_BSS",     SRAM);

ENTRY(Reset_Handler)
SECTIONS
{

  .entry : {
  . = ALIGN(0x4) ;
  KEEP(*startup.o(.vectors))
  *startup*.o(.text*);
 } > REGION_ENTRY

  .rodata : {
  . = ALIGN(0x4) ;

  __srodata = .;
  *(.rdata)
  *(.rdata*)
  *(.rdata1)
  *(.rdata.*)
  *(.rodata)
  *(.rodata1)
  *(.rodata*)
  *(.rodata.*)
  *(.rodata.str1.4)
  . = ALIGN(0x4) ;
  __erodata = .;
  __rodata_end__ = .;
 } > REGION_RODATA

 .data : {
  . = ALIGN(0x4) ;
  __sdata = . ;
  __data_start__ = . ;
  data_start = . ;
  *jump*.o(.text* );
  *rf*.o(.text* );
  *source_main*.o(.text* );
  *pwrmgr*.o(.text* );
  *watchdog*.o(.text* );
  *clock*.o(.text* );
  *flash*.o(.text* );
  *my_printf*.o(.text* );
  *patch*.o(.text* );
  *(.data)
  *(.data*)
  *(.data1)
  *(.data.*)
  *(.data1)
  *(.sdata)
  *(.sdata.*)
  __edata = .;
  __data_end__ = .;
  . = ALIGN(0x4) ;
 } > REGION_DATA AT > REGION_RODATA

 .text : {
  . = ALIGN(0x4) ;
  /*
  KEEP(*startup.o(.vectors))
  */
  __stext = . ;
  *patch*.o(.text.init_config );
  *(.text)
  *(.text*)
  *(.text.*)
  *(.gnu.warning)
  *(.stub)
  *(.gnu.linkonce.t*)
  *(.glue_7t)
  *(.glue_7)
  *(.jcr)
  KEEP (*(.init))
  KEEP (*(.fini))
  . = ALIGN (4) ;
  PROVIDE(__ctbp = .);
  *(.call_table_data)
  *(.call_table_text)
  . = ALIGN(0x10) ;
  __etext = . ;
 } > REGION_TEXT
 
 
 .bss : {
  . = ALIGN(0x4) ;
  __sbss = ALIGN(0x4) ;
  __bss_start__ = . ;
  *(.sbss)
  *(.sbss.*)
  *(.scommon)
  *(.dynbss)
  *(.bss)
  *(.bss.*)
  *(COMMON)
  . = ALIGN(0x4) ;
  __ebss = . ;
  __bss_end__ = .;
  __end = . ;
  end = . ;
 } > REGION_BSS
 
 
 ._user_heap : {
  . = ALIGN(0x4) ;
  __heap_start = .;
  . += __min_heap_size;
  . = ALIGN(0x4) ;
 } > REGION_BSS

}
