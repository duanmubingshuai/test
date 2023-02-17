/*
    Copyright 2018 Embedded Microprocessor Benchmark Consortium (EEMBC)

    Licensed under the Apache License, Version 2.0 (the "License");
    you may not use this file except in compliance with the License.
    You may obtain a copy of the License at

    http://www.apache.org/licenses/LICENSE-2.0

    Unless required by applicable law or agreed to in writing, software
    distributed under the License is distributed on an "AS IS" BASIS,
    WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
    See the License for the specific language governing permissions and
    limitations under the License.

    Original Author: Shay Gal-on
*/

#include "coremark.h"
/*  Function: get_seed
    Get a values that cannot be determined at compile time.

    Since different embedded systems and compilers are used, 3 different methods are provided:
    1 - Using a volatile variable. This method is only valid if the compiler is forced to generate code that
    reads the value of a volatile variable from memory at run time.
    Please note, if using this method, you would need to modify core_portme.c to generate training profile.
    2 - Command line arguments. This is the preferred method if command line arguments are supported.
    3 - System function. If none of the first 2 methods is available on the platform,
    a system function which is not a stub can be used.

    e.g. read the value on GPIO pins connected to switches, or invoke special simulator functions.
*/
ee_u16 crcu8_7(ee_u8 data, ee_u16 crc )
{
    ee_u8 i=0,x16=0,carry=0;

    for (i = 0; i < 8; i++)
    {
        x16 = (ee_u8)((data & 1) ^ ((ee_u8)crc & 1));
        data >>= 1;

        if (x16 == 1)
        {
            crc ^= 0x4002;
            carry = 1;
        }
        else
            carry = 0;

        crc >>= 1;

        if (carry)
            crc |= 0x8000;
        else
            crc &= 0x7fff;
    }

    return crc;
}

ee_s16 matrix_sum_7(ee_u32 N, MATRES* C, MATDAT clipval)
{
    MATRES tmp=0,prev=0,cur=0;
    ee_s16 ret=0;
    ee_u32 i,j;

    for (i=0; i<N; i++)
    {
        for (j=0; j<N; j++)
        {
            cur=C[i*N+j];
            tmp+=cur;

            if (tmp>clipval)
            {
                ret+=10;
                tmp=0;
            }
            else
            {
                ret += (cur>prev) ? 1 : 0;
            }

            prev=cur;
        }
    }

    return ret;
}
void matrix_mul_matrix_7(ee_u32 N, MATRES* C, MATDAT* A, MATDAT* B)
{
    ee_u32 i,j,k;

    for (i=0; i<N; i++)
    {
        for (j=0; j<N; j++)
        {
            C[i*N+j]=0;

            for(k=0; k<N; k++)
            {
                C[i*N+j]+=(MATRES)A[i*N+k] * (MATRES)B[k*N+j];
            }
        }
    }
}
void matrix_add_const_7(ee_u32 N, MATDAT* A, MATDAT val)
{
    ee_u32 i,j;

    for (i=0; i<N; i++)
    {
        for (j=0; j<N; j++)
        {
            A[i*N+j] += val;
        }
    }
}
void matrix_mul_vect_7(ee_u32 N, MATRES* C, MATDAT* A, MATDAT* B)
{
    ee_u32 i,j;

    for (i=0; i<N; i++)
    {
        C[i]=0;

        for (j=0; j<N; j++)
        {
            C[i]+=(MATRES)A[i*N+j] * (MATRES)B[j];
        }
    }
}

ee_s16 calc_func_7(ee_s16* pdata, core_results* res)
{
    ee_s16 data=*pdata;
    ee_s16 retval;
    ee_u8 optype=(data>>7) & 1; /* bit 7 indicates if the function result has been cached */

    if (optype) /* if cached, use cache */
        return (data & 0x007f);
    else   /* otherwise calculate and cache the result */
    {
        ee_s16 flag=data & 0x7; /* bits 0-2 is type of function to perform */
        ee_s16 dtype=((data>>3) & 0xf); /* bits 3-6 is specific data for the operation */
        dtype |= dtype << 4; /* replicate the lower 4 bits to get an 8b value */

        switch (flag)
        {
        case 0:
            if (dtype<0x22) /* set min period for bit corruption */
                dtype=0x22;

            retval=core_bench_state(res->size,res->memblock[3],res->seed1,res->seed2,dtype,res->crc);

            if (res->crcstate==0)
                res->crcstate=retval;

            break;

        case 1:
            retval=core_bench_matrix(&(res->mat),dtype,res->crc);

            if (res->crcmatrix==0)
                res->crcmatrix=retval;

            break;

        default:
            retval=data;
            break;
        }

        res->crc=crcu16(retval,res->crc);
        retval &= 0x007f;
        *pdata = (data & 0xff00) | 0x0080 | retval; /* cache the result */
        return retval;
    }
}

