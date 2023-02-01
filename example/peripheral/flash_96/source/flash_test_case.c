
#define FLASH_TEST

#ifdef FLASH_TEST

#include "log.h"
#include "flash.h"

extern int hal_flash_erase_all(void);
extern int spif_erase_chip(void);
//extern bool large_flash;

//test config
#define FLASH_TEST_TIME         250
//50//5000//5000

#define FLASH_START_ADDR        0x11000000
#define FLASH_STAGE_DELAY       1000

#define FLASH_PRINT_EN

typedef struct
{
    uint8_t manufacturerID;
    uint8_t memoryType;
    uint8_t Capacity;
    uint32_t capacity;
} FLASH_INFO;

FLASH_INFO flash = {0,0,0,0};

void  spif_read_id(void)
{
    uint8_t id[3];
    spif_cmd(FCMD_RDID, 0, 3, 0, 0, 0);
    spif_rddata(id, 3);
    flash.manufacturerID = id[0];
    flash.memoryType = id[1];
    flash.Capacity = id[2];

    if(flash.Capacity >= 0x11)
    {
        flash.capacity = (1ul << (flash.Capacity));
    }
    else
    {
        flash.capacity = 512*1024;
    }

    LOG("func:%s\n",__func__);
    LOG("flashid:0x%x 0x%x 0x%x\n",flash.manufacturerID,flash.memoryType,flash.Capacity);
    LOG("flash capacity:0x%x %dKByte\n",flash.capacity,flash.capacity/1024);
    LOG("\n");

    if(flash.capacity > 512*1024)
    {
        //large_flash = TRUE;
    }
}

#define FLASH_TEST_BUFFER_LEN 1024*16
uint8_t temp_r_buf[FLASH_TEST_BUFFER_LEN];
volatile unsigned int read_data;

void flash_set(unsigned int data)
{
    volatile uint8_t buf[4];
    buf[0] = data & 0x000000FF;
    buf[1] = (data & 0x0000FF00)>>8;
    buf[2] = (data & 0x00FF0000)>>16;
    buf[3] = (data & 0x0FF000000)>>24;

    for(unsigned int i = 0; i < FLASH_TEST_BUFFER_LEN; i++)
    {
        switch(i%4)
        {
        case 0:
            temp_r_buf[i] = buf[0];
            break;

        case 1:
            temp_r_buf[i] = buf[1];
            break;

        case 2:
            temp_r_buf[i] = buf[2];
            break;

        case 3:
            temp_r_buf[i] = buf[3];
            break;

        default:
            break;
        }
    }
}

void flash_check(unsigned int data)
{
    for(unsigned int i = 0; i < FLASH_TEST_BUFFER_LEN; i=i+4)
    {
        read_data = temp_r_buf[i] | \
                    (temp_r_buf[i+1]<<8)| \
                    (temp_r_buf[i+2]<<16)| \
                    (temp_r_buf[i+3]<<24);

        if(read_data != data)
        {
            while(1)
            {
                LOG("value:0x%x value:0x%x\n",data,read_data);
                WaitMs(5000);
            }
        }
    }
}

/*
    erase
    erase ret check
    erase data check

    prog
    prog ret check

    read
    read ret check
    read data check
*/
bool stop_when_error = TRUE;
void flash_test(void)
{
    volatile unsigned int i = 0,j = 0;
    int ret = 0;
    unsigned int flash_size = 0;
    static volatile unsigned int er_err_couner = 0;
    static volatile unsigned int rd_err_couner1 = 0;
    static volatile unsigned int pg_err_couner = 0;
    static volatile unsigned int rd_err_couner2 = 0;
    spif_read_id();
    flash_size = FLASH_START_ADDR + flash.capacity;
    LOG("test times:%d\n",FLASH_TEST_TIME);
    LOG("start addr:0x%x\n",FLASH_START_ADDR);
    LOG("end addr:0x%x\n",flash_size);
    LOG("\n");

    for(i = 0; i<FLASH_TEST_TIME; i++)
    {
        LOG("\n\n-------------->test time:%d\n",i);
        LOG("------->erase:\n");
        //ret = hal_flash_erase_all();//0x11002000~size
        ret = spif_erase_chip();//0x11000000~size

        if(ret != PPlus_SUCCESS)
        {
            er_err_couner++;

            while(stop_when_error)
            {
                LOG("[error %d %d] ",ret,__LINE__);
                WaitMs(5000);
            }
        }

        WaitMs(FLASH_STAGE_DELAY);
        LOG("------->erase check:\n");

        for(j = FLASH_START_ADDR; j < flash_size; j += FLASH_TEST_BUFFER_LEN)
        {
            flash_set(0x00000000);
            ret = hal_flash_read(j,temp_r_buf,FLASH_TEST_BUFFER_LEN);

            if(ret != PPlus_SUCCESS)
            {
                rd_err_couner1++;

                while(stop_when_error)
                {
                    LOG("[error %d %d] ",ret,__LINE__);
                    WaitMs(5000);
                }
            }

            LOG("[0]------->addr:0x%x\n",j);
            flash_check(0xffffffff);
        }

        WaitMs(FLASH_STAGE_DELAY);
        LOG("------->prog:\n");

        for(j = FLASH_START_ADDR; j < flash_size; j += FLASH_TEST_BUFFER_LEN)
        {
            flash_set(0x12345678);
            gpio_write(P1,1);
            //ret = hal_flash_write_by_dma(j,temp_r_buf,FLASH_TEST_BUFFER_LEN);//todo
            ret = hal_flash_write(j,temp_r_buf,FLASH_TEST_BUFFER_LEN);
            //ret = spif_write_dma(j,temp_r_buf,FLASH_TEST_BUFFER_LEN);
            gpio_write(P1,0);

            if(ret != PPlus_SUCCESS)
            {
                pg_err_couner++;

                while(1)
                {
                    LOG("[error %d %d] ",ret,__LINE__);
                    WaitMs(5000);
                }
            }

            LOG("[1]------->addr:0x%x\n",j);
        }

        WaitMs(FLASH_STAGE_DELAY);
        LOG("------->prog check:\n");

        for(j = FLASH_START_ADDR; j < flash_size; j += FLASH_TEST_BUFFER_LEN)
        {
            flash_set(0x00000000);
            ret = hal_flash_read(j,temp_r_buf,FLASH_TEST_BUFFER_LEN);

            if(ret != PPlus_SUCCESS)
            {
                rd_err_couner2++;

                while(1)
                {
                    LOG("[error %d %d] ",ret,__LINE__);
                    WaitMs(5000);
                }
            }

            LOG("[2]------->addr:0x%x\n",j);
            flash_check(0x12345678);
        }
    }

    while(1)
    {
        LOG("\ntest finish:\n");
        LOG("test time:%d\n",FLASH_TEST_TIME);
        LOG("erase error:%d\n",er_err_couner);
        LOG("read error:%d\n",rd_err_couner1);
        LOG("prog error:%d\n",pg_err_couner);
        LOG("read error:%d\n",rd_err_couner2);

        if((er_err_couner == 0)&&(rd_err_couner1 == 0)&&(pg_err_couner == 0)&&(rd_err_couner2 == 0))
        {
            LOG("\nno problem\n");
        }
        else
        {
            LOG("\nerror check\n");
        }

        WaitMs(5000);
    }
}



void flash_test2(void)
{
    volatile unsigned int i = 0,j = 0,k=0;
    int ret = 0;
    unsigned int flash_size = 0;
    static volatile unsigned int er_err_couner = 0;
    static volatile unsigned int rd_err_couner1 = 0;
    static volatile unsigned int pg_err_couner = 0;
    static volatile unsigned int rd_err_couner2 = 0;
    spif_read_id();
    flash_size = FLASH_START_ADDR + flash.capacity;
    LOG("test times:%d\n",FLASH_TEST_TIME);
    LOG("start addr:0x%x\n",FLASH_START_ADDR);
    LOG("end addr:0x%x\n",flash_size);
    LOG("\n");

    //erase:1
    //prog:1
    //read:500
    for(k=0; k<2; k++)
    {
        LOG("\n\n-------------->test time:%d\n",i);
        LOG("------->erase:\n");
        //ret = hal_flash_erase_all();//0x11002000~size
        ret = spif_erase_chip();//0x11000000~size

        if(ret != PPlus_SUCCESS)
        {
            er_err_couner++;

            while(stop_when_error)
            {
                LOG("[error %d %d] ",ret,__LINE__);
                WaitMs(5000);
            }
        }

        WaitMs(FLASH_STAGE_DELAY);
        LOG("------->erase check:\n");

        for(j = FLASH_START_ADDR; j < flash_size; j += FLASH_TEST_BUFFER_LEN)
        {
            flash_set(0x00000000);
            ret = hal_flash_read(j,temp_r_buf,FLASH_TEST_BUFFER_LEN);

            if(ret != PPlus_SUCCESS)
            {
                rd_err_couner1++;

                while(stop_when_error)
                {
                    LOG("[error %d %d] ",ret,__LINE__);
                    WaitMs(5000);
                }
            }

            LOG("[0]->addr:0x%x\n",j);
            flash_check(0xffffffff);
        }

        WaitMs(FLASH_STAGE_DELAY);
        LOG("------->prog:\n");

        for(j = FLASH_START_ADDR; j < flash_size; j += FLASH_TEST_BUFFER_LEN)
        {
            switch(k)
            {
            case 0:
                flash_set(0x5a5a5a5a);
                break;

            case 1:
                flash_set(0xa5a5a5a5);
                break;

            default:
                break;
            }

            //gpio_write(P1,1);
            //ret = hal_flash_write_by_dma(j,temp_r_buf,FLASH_TEST_BUFFER_LEN);//todo
            ret = hal_flash_write(j,temp_r_buf,FLASH_TEST_BUFFER_LEN);
            //ret = spif_write_dma(j,temp_r_buf,FLASH_TEST_BUFFER_LEN);
            //gpio_write(P1,0);

            if(ret != PPlus_SUCCESS)
            {
                pg_err_couner++;

                while(1)
                {
                    LOG("[error %d %d] ",ret,__LINE__);
                    WaitMs(5000);
                }
            }

            LOG("[1]->addr:0x%x\n",j);
        }

        WaitMs(FLASH_STAGE_DELAY);

        for(i = 0; i<FLASH_TEST_TIME; i++)
        {
            LOG("------->prog check:%d\n",i);

            for(j = FLASH_START_ADDR; j < flash_size; j += FLASH_TEST_BUFFER_LEN)
            {
                flash_set(0x00000000);
                ret = hal_flash_read(j,temp_r_buf,FLASH_TEST_BUFFER_LEN);

                if(ret != PPlus_SUCCESS)
                {
                    rd_err_couner2++;

                    while(1)
                    {
                        LOG("[error %d %d] ",ret,__LINE__);
                        WaitMs(5000);
                    }
                }

                LOG("[2]>addr:0x%x\n",j);

                switch(k)
                {
                case 0:
                    flash_set(0x5a5a5a5a);
                    break;

                case 1:
                    flash_set(0xa5a5a5a5);
                    break;

                default:
                    break;
                }
            }
        }
    }

    while(1)
    {
        LOG("\ntest finish:\n");
        LOG("test time:%d\n",FLASH_TEST_TIME);
        LOG("erase error:%d\n",er_err_couner);
        LOG("read error:%d\n",rd_err_couner1);
        LOG("prog error:%d\n",pg_err_couner);
        LOG("read error:%d\n",rd_err_couner2);

        if((er_err_couner == 0)&&(rd_err_couner1 == 0)&&(pg_err_couner == 0)&&(rd_err_couner2 == 0))
        {
            LOG("\nno problem\n");
        }
        else
        {
            LOG("\nerror check\n");
        }

        WaitMs(5000);
    }
}

#endif
