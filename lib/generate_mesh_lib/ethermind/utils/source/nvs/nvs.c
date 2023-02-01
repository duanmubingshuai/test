
/**
    \file nvs.c


*/

/*
    Copyright (C) 2019. Phyplus Limited.
    All rights reserved.
*/

/* --------------------------------------------- Header File Inclusion */
#include "nvs.h"
#include "flash.h"
#include "EM_platform.h"
#include "MS_net_api.h"
#include "access_internal.h"
#include "access_extern.h"

#define NVS_ENABLE

//#ifdef printf
//#undef printf
//#define printf(...)  /*{printf (__VA_ARGS__); printf("\r\n"); fflush(stdout);}*/
//#endif /* printf */

extern NET_SEQ_NUMBER_STATE net_seq_number_state;

extern uint16_t crc16(uint16_t seed, const volatile void* p_data, uint32_t size);

/* --------------------------------------------- Global Definitions */
UINT32 nvs_flash_base1,nvs_flash_base2;
UINT32 nvs_crc_pos;


/* --------------------------------------------- External Global Variables */

/* --------------------------------------------- Exported Global Variables */

/* --------------------------------------------- Static Global Variables */
#ifdef NVS_ENABLE
    /** Flash Base */
    DECL_STATIC UINT8* nvs_base[NVS_NUM_BANKS];

    /* NVS bank size */
    DECL_STATIC UINT16 nvs_size[NVS_NUM_BANKS];

    /* NVS bank current offset */
    DECL_STATIC UINT16 nvs_offset[NVS_NUM_BANKS];

    /* NVS Bank current state */
    DECL_STATIC UCHAR nvs_state[NVS_NUM_BANKS];

    /* Non-volatile write buffer for sector write */
    //__attribute((aligned))
    //DECL_STATIC UCHAR mem[NVS_FLASH_SIZE];
#endif /* NVS_ENABLE */

/* --------------------------------------------- Functions */
#ifdef NVS_ENABLE
static void NV_Erase (unsigned int addr)
{
    if(NVS_FLASH_SIZE > 0x1000)
    {
        hal_flash_erase_sector(addr);
        hal_flash_erase_sector(addr + 0x1000);
    }
    else
    {
        hal_flash_erase_sector(addr);
    }
}

static unsigned int NV_Read (unsigned int addr, uint8_t* buffer, unsigned int size)
{
    /*  UINT32 i;

        for (i = 0; i < size; i ++)
        {
         (buffer + i) = ReadFlash ((unsigned int)(addr + i));
        }

        return i;*/
    hal_flash_read(addr,buffer,size);
    return size;
}

static unsigned int NV_Write
(
    unsigned int start_addr,
    unsigned int length,
    unsigned int* data
)
{
    unsigned int ret;
//    uint32_t i,j;
//    printf("[NV_Write]MS_PS_RECORD_CORE_MODULES_OFFSET = 0x%08X\n",MS_PS_RECORD_CORE_MODULES_OFFSET);
//    j = (length & 0x03) ? 1 : 0;
//    j = (length >> 2);
//    /* Read to local static */
//    NV_Read((unsigned int)(nvs_base[0]), mem, (j+2)<<2);
//    /* Update local static */
//    memcpy(&mem[nvs_offset[0]], data, length);
//    /* Erase the flash */
//    NV_Erase();
    ret = hal_flash_write_by_dma(((unsigned int)start_addr),(unsigned char*)data,length);

    if (0 != ret)
    {
        printf ("WriteFlash Failed!");
    }

//    for (i = 0; i < j; i++)
//    {
//        //HAL_DISABLE_INTERRUPTS();
//        ret = flash_write_word
//              (
//                  (((unsigned int)start_addr) + (i * sizeof(uint32_t))),
//                  (uint32_t)(*(data + i))
//              );
//        //HAL_ENABLE_INTERRUPTS();
//        if (0 != ret)
//        {
//            printf ("WriteFlash Failed! - %d", i);
//            break;
//        }
//    }
    return ret;
}

static UINT8 NV_Get_Free_Sector (UINT8 bank,UINT8 mode)
{
    UINT32  flag1,flag2;
    UINT8  count1,count2;
    /* Read from flash */
    NV_Read((unsigned int)(nvs_flash_base1+nvs_crc_pos), (void*)&flag1, sizeof(flag1));
    NV_Read((unsigned int)(nvs_flash_base2+nvs_crc_pos), (void*)&flag2, sizeof(flag2));
//  printf("flag1 %x\n",flag1);
//  printf("flag2 %x\n",flag2);
    count1 = (flag1>>8)&0xf;
    count2 = (flag2>>8)&0xf;
//  printf("count1 %x\n",count1);
//  printf("count2 %x\n",count2);

    if(mode == NVS_ACCESS_WRITE)
    {
        if((((flag1 & 0x07) == NVS_FLASH_READY)&&((flag2 & 0x07) == NVS_FLASH_BAD))
                || ((((flag1 & 0x07) == NVS_FLASH_BAD)&&((flag2 & 0x07) == NVS_FLASH_BAD))
                    &&(MS_access_ps_crc_check(NVS_FLASH_FIRST_OK) == API_SUCCESS)))
        {
            /* Assign the base */
            nvs_base[bank] = (UINT8*)nvs_flash_base2;
            return NVS_FLASH_SECOND_OK;
        }
        else
        {
            /* Assign the base */
            nvs_base[bank] = (UINT8*)nvs_flash_base1;
            return NVS_FLASH_FIRST_OK;
        }
    }
    else
    {
        if(((flag1 & 0x07) == NVS_FLASH_READY)&&((flag2 & 0x07) == NVS_FLASH_READY))
        {
            if((((count1 == 0)&&(count2 == 0xf))||(count1 >= count2))
                    &&(MS_access_ps_crc_check(NVS_FLASH_FIRST_OK) == API_SUCCESS))
            {
                nvs_base[bank] = (UINT8*)nvs_flash_base2;
                nvs_write_header(bank,0xfffffff8);
                /* Assign the base */
                ms_ps_count = count1;
                nvs_base[bank] = (UINT8*)nvs_flash_base1;
                return NVS_FLASH_FIRST_OK;
            }
            else if((((count1 == 0)&&(count2 == 0xf))||(count1 >= count2))
                    &&(MS_access_ps_crc_check(NVS_FLASH_SECOND_OK) == API_SUCCESS))
            {
                nvs_base[bank] = (UINT8*)nvs_flash_base1;
                nvs_write_header(bank,0xfffffff8);
                /* Assign the base */
                ms_ps_count = count2;
                nvs_base[bank] = (UINT8*)nvs_flash_base2;
                return NVS_FLASH_SECOND_OK;
            }
            else if((((count2 == 0)&&(count1 == 0xf))||(count2 > count1))
                    &&(MS_access_ps_crc_check(NVS_FLASH_SECOND_OK) == API_SUCCESS))
            {
                nvs_base[bank] = (UINT8*)nvs_flash_base1;
                nvs_write_header(bank,0xfffffff8);
                /* Assign the base */
                ms_ps_count = count2;
                nvs_base[bank] = (UINT8*)nvs_flash_base2;
                return NVS_FLASH_SECOND_OK;
            }
            else if((((count2 == 0)&&(count1 == 0xf))||(count2 > count1))
                    &&(MS_access_ps_crc_check(NVS_FLASH_FIRST_OK) == API_SUCCESS))
            {
                nvs_base[bank] = (UINT8*)nvs_flash_base2;
                nvs_write_header(bank,0xfffffff8);
                /* Assign the base */
                ms_ps_count = count1;
                nvs_base[bank] = (UINT8*)nvs_flash_base1;
                return NVS_FLASH_FIRST_OK;
            }
            else
            {
                ms_ps_count = 0;
                return NVS_FLASH_INVAILD;
            }
        }
        else if((((flag1 & 0x07) == NVS_FLASH_READY)&&((flag2 & 0x07) == NVS_FLASH_BAD))
                &&(MS_access_ps_crc_check(NVS_FLASH_FIRST_OK) == API_SUCCESS))
        {
            /* Assign the base */
            nvs_base[bank] = (UINT8*)nvs_flash_base1;
            ms_ps_count = count1;
            return NVS_FLASH_FIRST_OK;
        }
        else if((((flag1 & 0x07) == NVS_FLASH_READY)&&((flag2 & 0x07) == NVS_FLASH_BAD))
                &&(MS_access_ps_crc_check(NVS_FLASH_SECOND_OK) == API_SUCCESS))
        {
            /* Assign the base */
            nvs_base[bank] = (UINT8*)nvs_flash_base1;
            nvs_write_header(bank,0xfffffff8);
            nvs_base[bank] = (UINT8*)nvs_flash_base2;
            ms_ps_count = count2;
            return NVS_FLASH_SECOND_OK;
        }
        else if((((flag2 & 0x07) == NVS_FLASH_READY)&&((flag1 & 0x07) == NVS_FLASH_BAD))
                &&(MS_access_ps_crc_check(NVS_FLASH_SECOND_OK) == API_SUCCESS))
        {
            /* Assign the base */
            nvs_base[bank] = (UINT8*)nvs_flash_base2;
            ms_ps_count = count2;
            return NVS_FLASH_SECOND_OK;
        }
        else if((((flag2 & 0x07) == NVS_FLASH_READY)&&((flag1 & 0x07) == NVS_FLASH_BAD))
                &&(MS_access_ps_crc_check(NVS_FLASH_FIRST_OK) == API_SUCCESS))
        {
            /* Assign the base */
            nvs_base[bank] = (UINT8*)nvs_flash_base2;
            nvs_write_header(bank,0xfffffff8);
            nvs_base[bank] = (UINT8*)nvs_flash_base1;
            ms_ps_count = count1;
            return NVS_FLASH_FIRST_OK;
        }
        else if(((flag1 & 0x07) == NVS_FLASH_BAD)&&((flag2 & 0x07) == NVS_FLASH_BAD))
        {
            if(MS_access_ps_crc_check(NVS_FLASH_FIRST_OK) == API_SUCCESS)
            {
                /* Assign the base */
                ms_ps_count = count1;
                nvs_base[bank] = (UINT8*)nvs_flash_base1;
                return NVS_FLASH_FIRST_OK;
            }
            else if(MS_access_ps_crc_check(NVS_FLASH_SECOND_OK) == API_SUCCESS)
            {
                /* Assign the base */
                ms_ps_count = count2;
                nvs_base[bank] = (UINT8*)nvs_flash_base2;
                return NVS_FLASH_SECOND_OK;
            }
            else
            {
                ms_ps_count = 0;
                return NVS_FLASH_INVAILD;
            }
        }
        else
        {
            ms_ps_count = 0;
            return NVS_FLASH_INVAILD;
        }
    }
}
#endif /* NVS_ENABLE */

UINT16 nvs_init (UINT8 bank)
{
    #ifdef NVS_ENABLE
    /* Assign the base */
    nvs_base[bank] = (UINT8*)nvs_flash_base1;
    /* Get the maximum allowed size limit */
//    nvs_size[bank] = (UINT16)(MS_PS_RECORD_PROXY_FILTER_ADDR_OFFSET+MS_PS_RECORD_PROXY_FILTER_ADDR_MAXSIZE);
    nvs_size[bank] = (UINT16)(NVS_FLASH_SIZE);
//  printf ("Initializing Storage... %x bytes\r\n", nvs_size[bank]);
    /* Set initial state to closed */
    nvs_state[bank] = NVS_CLOSE;
    return nvs_size[bank];
    #else /* NVS_ENABLE */
    return 0;
    #endif
}

void nvs_shutdown (UINT8 bank)
{
}

void nvs_reset (UINT8 bank)
{
    #ifdef NVS_ENABLE
    NV_Erase((unsigned int)nvs_base[bank]);
    nvs_offset[bank] = 0;
    #endif /* NVS_ENABLE */
}

INT16 nvs_open (UINT8 bank, UINT8 mode, UINT16 offset)
{
    UINT8  select_sector;
    #ifdef NVS_ENABLE

    /* Check if state is closed. Only then open */
    if (NVS_CLOSE != nvs_state[bank])
    {
        return (INT16)-1;
    }

    /* Initialize access offset */
    nvs_offset[bank] = 0;
    nvs_offset[bank] += offset;
    /* Set persistant state to open */
    nvs_state[bank] = (NVS_ACCESS_READ == mode)? NVS_RDOPEN: NVS_WROPEN;
    select_sector = NV_Get_Free_Sector(bank,mode);

    if(select_sector == NVS_FLASH_INVAILD)
    {
        return (INT16)-1;
    }

    #endif /* NVS_ENABLE */
    return select_sector;
}

INT16 nvs_close (UINT8 bank)
{
    #ifdef NVS_ENABLE
    /* Seek to the start of the bank */
    nvs_seek(bank, 0);
    nvs_state[bank] = NVS_CLOSE;
//    printf ("Storage Closed");
    #endif /* NVS_ENABLE */
    return 0;
}

INT16 nvs_write (UINT8 bank, void* buffer, UINT16 size)
{
    #ifdef NVS_ENABLE
//    UINT16 ret;

    if (NVS_WROPEN != nvs_state[bank])
    {
//      printf("close\n");
        return -1;
    }

    /* Write to flash */
    NV_Write
    (
        (unsigned int)nvs_offset[bank] + (unsigned int)nvs_base[bank],
        size,
        buffer
    );
    /* Update access offset */
    nvs_offset[bank] += size;
//    printf ("Written 0x%08X bytes. Offset at 0x%08X\n", size, nvs_offset[bank]);
    #endif /* NVS_ENABLE */
    return size;
}

INT16 nvs_read (UINT8 bank, void* buffer, UINT16 size)
{
    #ifdef NVS_ENABLE

    if (NVS_RDOPEN != nvs_state[bank])
    {
//      printf("close\n");
        return -1;
    }

    /* Read from flash */
    NV_Read((unsigned int)((unsigned int)(nvs_base[bank]) + nvs_offset[bank]), buffer, size);
    /* Update access offset */
    nvs_offset[bank] += size;
//    printf ("Read %d bytes. Offset at %d", size, nvs_offset[bank]);
    #endif /* NVS_ENABLE */
    return size;
}

INT16 nvs_read_crc16 (UINT8 bank, UINT16* buffer, UINT16 size)
{
    #ifdef NVS_ENABLE
    hal_flash_read((unsigned int)nvs_base[bank]+0x11000002 + nvs_crc_pos,(UINT8*)buffer,2);
    #endif /* NVS_ENABLE */
    return 0;
}

INT8 nvs_read_index (UINT8 bank, UINT8* buffer, UINT16 size)
{
    #ifdef NVS_ENABLE
//    UINT16 crc16_code=0;
    //Calc crc code
    hal_flash_read((unsigned int)nvs_base[bank]+0x11000001 + nvs_crc_pos,(UINT8*)buffer,1);
//  printf("[NVS]index:0x%02x\n",(unsigned int)(nvs_base[bank]+0x11000001 + nvs_crc_pos));
//  hal_flash_read((unsigned int)nvs_base[bank]+0x11000002 + NVS_FLASH_SIZE,(UINT8*)buffer,2);
//  printf("[NVS]read:0x%08x\n",(unsigned int)(nvs_base[bank]+0x11000002 + NVS_FLASH_SIZE));
//    crc16_code = crc16(crc16_code, (UINT8 *)((unsigned int)nvs_base[bank]+0x11000000), NVS_FLASH_SIZE);
//    * buffer = crc16_code;
    #endif /* NVS_ENABLE */
    return 0;
}



INT16 nvs_seek (UINT8 bank, UINT32 offset)
{
    #ifdef NVS_ENABLE

    /* Check if flash is not closed */
    if (NVS_CLOSE == nvs_state[bank])
    {
        return -1;
    }

    /* Update the access offset */
    nvs_offset[bank] = offset;
//    printf ("Seek 0x%08X offset. Offset at 0x%08X\n", offset, nvs_offset[bank]);
    #endif /* NVS_ENABLE */
    return offset;
}

INT16 nvs_erase (UINT8 bank)
{
    #ifdef NVS_ENABLE

    /* Check if flash is not closed */
    if (NVS_CLOSE == nvs_state[bank])
    {
//      printf("close\n");
        return -1;
    }

    NV_Erase((unsigned int)nvs_base[bank]);
    #endif /* NVS_ENABLE */
    return 0;
}

INT16 nvs_write_header (UINT8 bank,UINT32 svalue)
{
    #ifdef NVS_ENABLE
    unsigned int ret;

    /* Check if flash is not closed */
    if (NVS_CLOSE == nvs_state[bank])
    {
//      printf("close\n");
        return -1;
    }

//    printf("[NVS]Writer:0x%08x,value:0x%08x\n",(unsigned int)(nvs_base[bank]+NVS_FLASH_SIZE),svalue);
//    ret=flash_write_word((unsigned int)(nvs_base[bank]+NVS_FLASH_SIZE),svalue);
//  printf("[NVS]Writer:0x%08x,value:0x%08x\n",(unsigned int)(nvs_base[bank]+nvs_crc_pos),svalue);
    ret=flash_write_word((unsigned int)(nvs_base[bank]+nvs_crc_pos),svalue);

//  printf("ret %x\n",ret);
    if(ret != 0)
        return -1;

    if((svalue&0x07) == NVS_FLASH_READY)
    {
        UINT32 rword_add,rword_value;
        rword_add = (nvs_base[bank] == (UINT8*)nvs_flash_base1)?nvs_flash_base2:nvs_flash_base1;
        rword_add += nvs_crc_pos;
        hal_flash_read((unsigned int)rword_add,(uint8*)&rword_value,4);
        rword_value &= 0xfffffff8;
        ret = flash_write_word((unsigned int)rword_add,rword_value);

        if(ret != 0)
            return -1;
    }

    #endif /* NVS_ENABLE */
    return 0;
}



#include "nvsto.h"

void nvs_test (void)
{
    UINT32 base1 = 0x3c000,base2 = 0x3e000;
    NVSTO_HANDLE shdl;
    volatile INT32 nbytes;
    char wtest[] = "EtherMind Mesh Storage Test";
    char rtest[sizeof(wtest)];
//    UCHAR test[8] = {0xde, 0xad, 0xbe, 0xef, 0xde, 0xad, 0xbe, 0xef};
//    static UCHAR tt[8] = {0};
    printf ("Storage Testing...\r\n");
    printf ("Initializing Storage...\r\n")
    nvsto_init(base1,base2);
    nvsto_register_ps(sizeof(wtest), &shdl);
    printf ("Opening Storage for Read...\r\n")
    nbytes = nvsto_open_psread(shdl);
    printf ("Retval - 0x%04X\r\n", nbytes);

    if (0 == nbytes)
    {
        printf ("Read...\r\n")
        nbytes = nvsto_read_ps(shdl, rtest, sizeof(rtest));
        printf ("Retval - 0x%04X\r\n", nbytes);

        if (0 < nbytes)
        {
            printf ("Read from Flash - %s\r\n", rtest);
        }

        printf ("Closing Storage...\r\n");
        nvsto_close_ps (shdl);
    }
    else
    {
        printf ("Opening Storage for Write...\r\n")
        nbytes = nvsto_open_pswrite(shdl);
        printf ("Retval - 0x%04X\r\n", nbytes);

        if (0 == nbytes)
        {
            printf ("Write...\r\n")
            nbytes = nvsto_write_ps(shdl, wtest, sizeof(wtest));
            printf ("Retval - 0x%04X\r\n", nbytes);

            if (0 < nbytes)
            {
                printf ("Write to Flash - %s\r\n", wtest);
            }

            printf ("Closing Storage...\r\n");
            nvsto_close_ps (shdl);
        }
    }
}

