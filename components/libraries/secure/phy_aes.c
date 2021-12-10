/**************************************************************************************************
 
  Phyplus Microelectronics Limited confidential and proprietary. 
  All rights reserved.

  IMPORTANT: All rights of this software belong to Phyplus Microelectronics 
  Limited ("Phyplus"). Your use of this Software is limited to those 
  specific rights granted under  the terms of the business contract, the 
  confidential agreement, the non-disclosure agreement and any other forms 
  of agreements as a customer or a partner of Phyplus. You may not use this 
  Software unless you agree to abide by the terms of these agreements. 
  You acknowledge that the Software may not be modified, copied, 
  distributed or disclosed unless embedded on a Phyplus Bluetooth Low Energy 
  (BLE) integrated circuit, either as a product or is integrated into your 
  products.  Other than for the aforementioned purposes, you may not use, 
  reproduce, copy, prepare derivative works of, modify, distribute, perform, 
  display or sell this Software and/or its documentation for any purposes.

  YOU FURTHER ACKNOWLEDGE AND AGREE THAT THE SOFTWARE AND DOCUMENTATION ARE
  PROVIDED AS IS WITHOUT WARRANTY OF ANY KIND, EITHER EXPRESS OR IMPLIED,
  INCLUDING WITHOUT LIMITATION, ANY WARRANTY OF MERCHANTABILITY, TITLE,
  NON-INFRINGEMENT AND FITNESS FOR A PARTICULAR PURPOSE. IN NO EVENT SHALL
  PHYPLUS OR ITS SUBSIDIARIES BE LIABLE OR OBLIGATED UNDER CONTRACT,
  NEGLIGENCE, STRICT LIABILITY, CONTRIBUTION, BREACH OF WARRANTY, OR OTHER
  LEGAL EQUITABLE THEORY ANY DIRECT OR INDIRECT DAMAGES OR EXPENSES
  INCLUDING BUT NOT LIMITED TO ANY INCIDENTAL, SPECIAL, INDIRECT, PUNITIVE
  OR CONSEQUENTIAL DAMAGES, LOST PROFITS OR LOST DATA, COST OF PROCUREMENT
  OF SUBSTITUTE GOODS, TECHNOLOGY, SERVICES, OR ANY CLAIMS BY THIRD PARTIES
  (INCLUDING BUT NOT LIMITED TO ANY DEFENSE THEREOF), OR OTHER SIMILAR COSTS.
  
**************************************************************************************************/

#include <string.h>
#include "ll_def.h"
#include "ll_enc.h"
#include "phy_aes.h"
#include "error.h"
//#include "flash.h"
//#include "ll_def.h"
//#include "ll_enc.h"
extern void LL_ENC_AES128_Encrypt0( uint8 *key,uint8 *plaintext,uint8 *ciphertext );
#if 0
static const char* s_company_id = "PHY+622XPLUS0504";
uint8_t  s_sBox[256] =
{ /*  0    1    2    3    4    5    6    7    8    9    a    b    c    d    e    f */
    0x63,0x7c,0x77,0x7b,0xf2,0x6b,0x6f,0xc5,0x30,0x01,0x67,0x2b,0xfe,0xd7,0xab,0x76, /*0*/
    0xca,0x82,0xc9,0x7d,0xfa,0x59,0x47,0xf0,0xad,0xd4,0xa2,0xaf,0x9c,0xa4,0x72,0xc0, /*1*/
    0xb7,0xfd,0x93,0x26,0x36,0x3f,0xf7,0xcc,0x34,0xa5,0xe5,0xf1,0x71,0xd8,0x31,0x15, /*2*/
    0x04,0xc7,0x23,0xc3,0x18,0x96,0x05,0x9a,0x07,0x12,0x80,0xe2,0xeb,0x27,0xb2,0x75, /*3*/
    0x09,0x83,0x2c,0x1a,0x1b,0x6e,0x5a,0xa0,0x52,0x3b,0xd6,0xb3,0x29,0xe3,0x2f,0x84, /*4*/
    0x53,0xd1,0x00,0xed,0x20,0xfc,0xb1,0x5b,0x6a,0xcb,0xbe,0x39,0x4a,0x4c,0x58,0xcf, /*5*/
    0xd0,0xef,0xaa,0xfb,0x43,0x4d,0x33,0x85,0x45,0xf9,0x02,0x7f,0x50,0x3c,0x9f,0xa8, /*6*/
    0x51,0xa3,0x40,0x8f,0x92,0x9d,0x38,0xf5,0xbc,0xb6,0xda,0x21,0x10,0xff,0xf3,0xd2, /*7*/
    0xcd,0x0c,0x13,0xec,0x5f,0x97,0x44,0x17,0xc4,0xa7,0x7e,0x3d,0x64,0x5d,0x19,0x73, /*8*/
    0x60,0x81,0x4f,0xdc,0x22,0x2a,0x90,0x88,0x46,0xee,0xb8,0x14,0xde,0x5e,0x0b,0xdb, /*9*/
    0xe0,0x32,0x3a,0x0a,0x49,0x06,0x24,0x5c,0xc2,0xd3,0xac,0x62,0x91,0x95,0xe4,0x79, /*a*/
    0xe7,0xc8,0x37,0x6d,0x8d,0xd5,0x4e,0xa9,0x6c,0x56,0xf4,0xea,0x65,0x7a,0xae,0x08, /*b*/
    0xba,0x78,0x25,0x2e,0x1c,0xa6,0xb4,0xc6,0xe8,0xdd,0x74,0x1f,0x4b,0xbd,0x8b,0x8a, /*c*/
    0x70,0x3e,0xb5,0x66,0x48,0x03,0xf6,0x0e,0x61,0x35,0x57,0xb9,0x86,0xc1,0x1d,0x9e, /*d*/
    0xe1,0xf8,0x98,0x11,0x69,0xd9,0x8e,0x94,0x9b,0x1e,0x87,0xe9,0xce,0x55,0x28,0xdf, /*e*/
    0x8c,0xa1,0x89,0x0d,0xbf,0xe6,0x42,0x68,0x41,0x99,0x2d,0x0f,0xb0,0x54,0xbb,0x16  /*f*/
};
uint8_t  s_invsBox[256] =
{ /*  0    1    2    3    4    5    6    7    8    9    a    b    c    d    e    f  */
    0x52,0x09,0x6a,0xd5,0x30,0x36,0xa5,0x38,0xbf,0x40,0xa3,0x9e,0x81,0xf3,0xd7,0xfb, /*0*/
    0x7c,0xe3,0x39,0x82,0x9b,0x2f,0xff,0x87,0x34,0x8e,0x43,0x44,0xc4,0xde,0xe9,0xcb, /*1*/
    0x54,0x7b,0x94,0x32,0xa6,0xc2,0x23,0x3d,0xee,0x4c,0x95,0x0b,0x42,0xfa,0xc3,0x4e, /*2*/
    0x08,0x2e,0xa1,0x66,0x28,0xd9,0x24,0xb2,0x76,0x5b,0xa2,0x49,0x6d,0x8b,0xd1,0x25, /*3*/
    0x72,0xf8,0xf6,0x64,0x86,0x68,0x98,0x16,0xd4,0xa4,0x5c,0xcc,0x5d,0x65,0xb6,0x92, /*4*/
    0x6c,0x70,0x48,0x50,0xfd,0xed,0xb9,0xda,0x5e,0x15,0x46,0x57,0xa7,0x8d,0x9d,0x84, /*5*/
    0x90,0xd8,0xab,0x00,0x8c,0xbc,0xd3,0x0a,0xf7,0xe4,0x58,0x05,0xb8,0xb3,0x45,0x06, /*6*/
    0xd0,0x2c,0x1e,0x8f,0xca,0x3f,0x0f,0x02,0xc1,0xaf,0xbd,0x03,0x01,0x13,0x8a,0x6b, /*7*/
    0x3a,0x91,0x11,0x41,0x4f,0x67,0xdc,0xea,0x97,0xf2,0xcf,0xce,0xf0,0xb4,0xe6,0x73, /*8*/
    0x96,0xac,0x74,0x22,0xe7,0xad,0x35,0x85,0xe2,0xf9,0x37,0xe8,0x1c,0x75,0xdf,0x6e, /*9*/
    0x47,0xf1,0x1a,0x71,0x1d,0x29,0xc5,0x89,0x6f,0xb7,0x62,0x0e,0xaa,0x18,0xbe,0x1b, /*a*/
    0xfc,0x56,0x3e,0x4b,0xc6,0xd2,0x79,0x20,0x9a,0xdb,0xc0,0xfe,0x78,0xcd,0x5a,0xf4, /*b*/
    0x1f,0xdd,0xa8,0x33,0x88,0x07,0xc7,0x31,0xb1,0x12,0x10,0x59,0x27,0x80,0xec,0x5f, /*c*/
    0x60,0x51,0x7f,0xa9,0x19,0xb5,0x4a,0x0d,0x2d,0xe5,0x7a,0x9f,0x93,0xc9,0x9c,0xef, /*d*/
    0xa0,0xe0,0x3b,0x4d,0xae,0x2a,0xf5,0xb0,0xc8,0xeb,0xbb,0x3c,0x83,0x53,0x99,0x61, /*e*/
    0x17,0x2b,0x04,0x7e,0xba,0x77,0xd6,0x26,0xe1,0x69,0x14,0x63,0x55,0x21,0x0c,0x7d  /*f*/
};
#endif

/*
static void getBit(uint8_t  ci,int*b){
    for(int i=0;i<8;i++)
        b[i]=(ci>>i)&0x01;
}


static uint8_t  getByte(int*b){
    uint8_t  out=0;
    for(int i=0;i<8;i++)
            out=out+(b[i]<<i);

    return out;
}

static uint8_t  xtime(uint8_t  a,int xtm){
    int b[8]={0};
    int ao[8]={0};
    getBit(a,b);

    if(xtm==1){
        ao[0]=0   ^b[7];
        ao[1]=b[0]^b[7];
        ao[2]=b[1];
        ao[3]=b[2]^b[7];
        ao[4]=b[3]^b[7];
        ao[5]=b[4];
        ao[6]=b[5];
        ao[7]=b[6];
    }else if(xtm==2){
        ao[0]=b[6];
        ao[1]=b[6]^b[7];
        ao[2]=b[0]^b[7];
        ao[3]=b[1]^b[6];
        ao[4]=b[2]^(b[6]^b[7]);
        ao[5]=b[3]^b[7];
        ao[6]=b[4];
        ao[7]=b[5];
    }else if(xtm==3){
        ao[0]=b[5];
        ao[1]=b[5]^b[6];
        ao[2]=b[6]^b[7];
        ao[3]=b[0]^(b[5]^b[7]);
        ao[4]=b[1]^(b[5]^b[6]);
        ao[5]=b[2]^(b[6]^b[7]);
        ao[6]=b[3]^b[7];
        ao[7]=b[4];

    }

    a=getByte(ao);


    return a;
}

static void affTrans(int *a,int sel){
    int t[8]={0};
    int A,B,C,D;
    for (int i=0;i<8;i++)
        t[i]=a[i];

    if(sel==0){
        A=a[0]^a[1];
        B=a[2]^a[3];
        C=a[4]^a[5];
        D=a[6]^a[7];

        a[0] = (1^t[0]) ^ C ^ D;
        a[1] = (1^t[5]) ^ A ^ D;
        a[2] = (0^t[2]) ^ A ^ D;
        a[3] = (0^t[7]) ^ A ^ B;
        a[4] = (0^t[4]) ^ A ^ B;
        a[5] = (1^t[1]) ^ B ^ C;
        a[6] = (1^t[6]) ^ B ^ C;
        a[7] = (0^t[3]) ^ C ^ D;

    }else if(sel==-1){
        A=a[0]^a[5];
        B=a[1]^a[4];
        C=a[2]^a[7];
        D=a[3]^a[6];

        a[0] = (1^t[5]) ^ C ;
        a[1] = (0^t[0]) ^ D ;
        a[2] = (1^t[7]) ^ B ;
        a[3] = (0^t[2]) ^ A ;
        a[4] = (0^t[1]) ^ D ;
        a[5] = (0^t[4]) ^ C ;
        a[6] = (0^t[3]) ^ A ;
        a[7] = (0^t[6]) ^ B ;
    }
}
static void SubBytes(uint8_t  state[][4],int mode)
{


    int r,c;
    for(r=0; r<4; r++)
    {
        for(c=0; c<4; c++)
        {

            state[r][c] = mode==0 ? s_sBox[state[r][c]] :s_invsBox[state[r][c]];
        }
    }
}
static void KeyExpansion(const uint8_t * key, uint8_t  w[][4][4],int mode)
{
    int i,j,r,c;
    //uint8_t  rc[] = {0x01, 0x02, 0x04, 0x08, 0x10, 0x20, 0x40, 0x80, 0x1b, 0x36};
    int rcSft[8]       = {1,0,0,0,0,0,0,0};
    uint8_t  rcOut[10];
    uint8_t  rcOutInv[10];

    for(i=1;i<=10;i++){

        int feedback=0;
        rcOut[i-1]=getByte(rcSft);
        feedback = rcSft[7];
        rcSft[7] = rcSft[6];
        rcSft[6] = rcSft[5];
        rcSft[5] = rcSft[4];
        rcSft[4] = rcSft[3]^feedback;
        rcSft[3] = rcSft[2]^feedback;
        rcSft[2] = rcSft[1];
        rcSft[1] = rcSft[0]^feedback;
        rcSft[0] = feedback;

        //printf("%2x ",rcOut[i-1]);
    }

    getBit(0x36,rcSft);
    for(i=1;i<=10;i++){

        int feedback=0;
        rcOutInv[i-1]=getByte(rcSft);
        feedback = rcSft[0];
        rcSft[0] = rcSft[1]^feedback;
        rcSft[1] = rcSft[2];
        rcSft[2] = rcSft[3]^feedback;
        rcSft[3] = rcSft[4]^feedback;
        rcSft[4] = rcSft[5];
        rcSft[5] = rcSft[6];
        rcSft[6] = rcSft[7];
        rcSft[7] = feedback;

//        printf("%2x ",rcOutInv[i-1]);
    }

    uint8_t  t[4];

    for(r=0; r<4; r++)
    {
        for(c=0; c<4; c++)
        {
            w[0][r][c] = key[r+c*4];
        }
    }
    for(i=1; i<=10; i++)
    {
        for(j=0; j<4; j++)
        {

            for(r=0; r<4; r++)
            {
                t[r] = j ? w[i][r][j-1] : w[i-1][r][3];
            }
            if(j == 0)
            {
                uint8_t  temp = t[0];
                for(r=0; r<3; r++)
                {
                    t[r] = s_sBox[t[(r+1)%4]];
                }
                t[3] = s_sBox[temp];
                //t[0] ^= rc[i-1];
                t[0] ^= rcOut[i-1];
            }
            for(r=0; r<4; r++)
            {
                w[i][r][j] = w[i-1][r][j] ^ t[r];
            }
        }
    }

    if(mode==1){

        for(int i=1;i<11;i++){

            for(r=0;r<4;r++){
                t[r]=w[10-i][r][3];
            }

            uint8_t  temp = t[0];
            for(r=0; r<3; r++)
            {
                t[r] = s_sBox[t[(r+1)%4]];
            }
            t[3] = s_sBox[temp];
            //t[0] ^= rc[i-1];
            t[0] ^= rcOutInv[i-1];


            for(r=0;r<4;r++){
                w[10-i][r][3]=w[10-i+1][r][3] ^ w[10-i+1][r][2];
                w[10-i][r][2]=w[10-i+1][r][2] ^ w[10-i+1][r][1];
                w[10-i][r][1]=w[10-i+1][r][1] ^ w[10-i+1][r][0];
                w[10-i][r][0]=w[10-i+1][r][0] ^ t[r];
            }
        }
    }


}
*/

/*
static uint8_t  FFmul(uint8_t  a, uint8_t  b)
{
    uint8_t  bw[4];
    uint8_t  res=0;
    int i;
    bw[0] = b;
    for(i=1; i<4; i++)
    {
        bw[i] = bw[i-1]<<1;
        if(bw[i-1]&0x80)
        {
            bw[i]^=0x1b;
        }
    }
    for(i=0; i<4; i++)
    {
        if((a>>i)&0x01)
        {
            res ^= bw[i];
        }
    }
    return res;
}

static void ShiftRows(uint8_t  state[][4],int mode)
{
    uint8_t  t[4];
    int r,c;
    for(r=1; r<4; r++)
    {
        for(c=0; c<4; c++)
        {
            if(mode==0)
                t[c] = state[r][(c+r)%4];// --> 1 2 3
            else
                t[c] = state[r][(c+4-r)%4];//<--1 2 3
        }
        for(c=0; c<4; c++)
        {
            state[r][c] = t[c];
        }
    }
}

static void MixColumns(uint8_t  state[][4],int mode)
{
    uint8_t  t[4];
    int  c;
    for(c=0; c< 4; c++)
    {
        t[0]=state[0][c];
        t[1]=state[1][c];
        t[2]=state[2][c];
        t[3]=state[3][c];

        state[0][c]= xtime(t[0]^t[1],1) ^ (t[2]^t[3]) ^ t[1];
        state[1][c]= xtime(t[1]^t[2],1) ^ (t[0]^t[3]) ^ t[2];
        state[2][c]= xtime(t[2]^t[3],1) ^ (t[0]^t[1]) ^ t[3];
        state[3][c]= xtime(t[0]^t[3],1) ^ (t[1]^t[2]) ^ t[0];

    if(mode==1){

        state[0][c]= state[0][c] ^ xtime(t[0]^t[2],2) ^ xtime( (t[0]^t[2]) ^ (t[1]^t[3]) ,3);
        state[1][c]= state[1][c] ^ xtime(t[1]^t[3],2) ^ xtime( (t[0]^t[2]) ^ (t[1]^t[3]) ,3);
        state[2][c]= state[2][c] ^ xtime(t[0]^t[2],2) ^ xtime( (t[0]^t[2]) ^ (t[1]^t[3]) ,3);
        state[3][c]= state[3][c] ^ xtime(t[1]^t[3],2) ^ xtime( (t[0]^t[2]) ^ (t[1]^t[3]) ,3);

    }

    }
}
static void AddRoundKey(uint8_t  state[][4], uint8_t  k[][4])
{
    int r,c;
    for(c=0; c<4; c++)
    {
        for(r=0; r<4; r++)
        {
            state[r][c] ^= k[r][c];
        }
    }
}

static void printfStats(uint8_t  state[][4] ,int mode){

#if(AES_DEBUG)
    int c,r;
    if(mode==1){
        for(r=0;r<4;r++){
            for(c=0;c<4;c++){
                printf("%2x ",state[r][c]);
            }
            printf("\n");
        }
        printf("\n");
    }
#endif

}

*/


static void xor128bit(uint8_t * x,uint8_t  *y,uint8_t  *z)
{
    for(int i=0;i<16;i++)
    {
        z[i]=x[i]^y[i];
    }
}

                                    
/****************************************************************************
* Function Name  : aes_ccm_phyplus_dec
* Description    : decrypt by phyplus defined aes ccm mode
* Input          :
                 : *iv,  13 byte
                 : *din, dLen byte
                 : dLen, input data length, exclude mic
                 : *mic, 4 byte, Message Identify Code
* Output         : (dout,dLen byte
* Return         : PPlus_SUCCESS, according to the MIC check result
                   PPlus_ERR_CRYPTO, error
                   
****************************************************************************/


int pplus_aesccm_dec( const uint8_t * key, const uint8_t * iv,
            uint8_t * din, int dLen, uint8_t * micIn, uint8_t *dout)
{
    //uint8_t  key_o[16];
	int i = 0;
    uint8_t  ax[16];
    uint8_t  bx[16];
    uint8_t  y[16];
    uint8_t  s[16];
    uint8_t  ti[16];
    uint8_t  to[16];
    uint8_t  dummy_out[16];

    int loopNum = (dLen+15)/16;
    int resLen  = dLen-(loopNum-1)*16;
    unsigned short cnt=0;   

    memset(ax, 0, 16);
    memset(bx, 0, 16);
    ax[0]=0x02;bx[0]=0x62;
    if(iv){
        for(i =0;i<13;i++)
        {
            ax[i+1]=iv[i];
            bx[i+1]=iv[i];
        }
    }
    bx[14]=((dLen/16)&0xff00)>>8;
    bx[15]=(dLen/16)&0xff;


    for(i=0;i<loopNum;i++)
    {
        uint8_t * pout = (dout == NULL) ? dummy_out : (dout+i*16);
        //update counter in ccm
        cnt+=13;
        ax[14]=(cnt&0xff00)>>8;
        ax[15]=cnt&0x00ff;

        //aes((const uint8_t *)s_company_id,key_o,ax,s,AES_ENC);
        LL_ENC_AES128_Encrypt0((uint8_t *)key, ax,s);

        if(i<loopNum-1)
        {
            xor128bit(s,din+i*16,pout);
        }
        else //process for the last16byte
        {
            for(int j=0;j<resLen;j++)
                ti[j]=din[i*16+j];
            for(int j=resLen;j<16;j++)
                ti[j]=0x00;

            xor128bit(s,ti,to);

            for(int j=0;j<resLen;j++)
                pout[j]=to[j];
            for(int j=resLen;j<16;j++)
                to[j]=0x00;

        }

        //cbc
        //aes((const uint8_t *)s_company_id,key_o,bx,y,AES_ENC);
        LL_ENC_AES128_Encrypt0((uint8_t *)key,bx,y);

        if(i<loopNum-1)
        {
            xor128bit(y,pout,bx);
        }
        else
        {
            xor128bit(y,to,bx);
        }
        
    }

    //check mic
    //aes((const uint8_t *)s_company_id,key_o,bx,y,AES_ENC);
   LL_ENC_AES128_Encrypt0((uint8_t *)key,bx,y);

    ax[14]=0;
    ax[15]=0;

    //aes((const uint8_t *)s_company_id,key_o,ax,s,AES_ENC);
    LL_ENC_AES128_Encrypt0((uint8_t *)key,ax,s);
   
    if(micIn == NULL)
      return PPlus_SUCCESS;

    for(i=0;i<4;i++)
    {
        if((y[i]^s[i])!=micIn[i])
        {
            return PPlus_ERR_SECURE_CRYPTO;
        }
    }
    
    return PPlus_SUCCESS;

    
}

/****************************************************************************
* Function Name  : aes_ccm_phyplus_enc
* Description    : encrypt by phyplus defined aes ccm mode
* Input          : *key, 16 byte
                 : *iv,  13 byte
                 : *din, dLen byte
                 : dLen, input data length, exclude mic
                 
* Output         : *dout,dLen byte
                 : *mic, 4 byte, Message Identify Code
* Return         : PPlus_SUCCESS 
****************************************************************************/
int pplus_aesccm_enc(const uint8_t * key, const uint8_t * iv, 
                    uint8_t * din,int dLen, uint8_t *dout, uint8_t * micOut)
{
    int i;
    uint8_t  ax[16];
    uint8_t  bx[16];
    uint8_t  y[16];
    uint8_t  s[16];
    uint8_t  ti[16];
    uint8_t  to[16];

    int loopNum = (dLen+15)/16;
    int resLen  = dLen-(loopNum-1)*16;
    unsigned short cnt=0;

    memset(ax, 0, 16);
    memset(bx, 0, 16);
    ax[0]=0x02;bx[0]=0x62;
    if(iv){
        for(i =0;i<13;i++)
        {
            ax[i+1]=iv[i];
            bx[i+1]=iv[i];
        }
    }
    bx[14]=((dLen/16)&0xff00)>>8;
    bx[15]=(dLen/16)&0xff;

    for(int i=0;i<loopNum;i++)
    {
        //update counter in ccm
        cnt+=13;
        ax[14]=(cnt&0xff00)>>8;
        ax[15]=cnt&0x00ff;

        //aes(key,key_o,ax,s,AES_ENC);
        LL_ENC_AES128_Encrypt0(( uint8_t *)key,ax,s);

        if(dout){
            if(i<loopNum-1)
                xor128bit(s,din+i*16,dout+16*i);
        }
        else //process for the last16byte
        {
            for(int j=0;j<resLen;j++)
                ti[j]=din[i*16+j];
            for(int j=resLen;j<16;j++)
                ti[j]=0x00;

            xor128bit(s,ti,to);

            if(dout){
                for(int j=0;j<resLen;j++)
                    dout[i*16+j]=to[j];
            }
        }

        //cbc
        //aes(key,key_o,bx,y,AES_ENC);
        LL_ENC_AES128_Encrypt0(( uint8_t *)key,bx,y);

        if(i<loopNum-1)
        {
            xor128bit(y,din+i*16,bx);
        }
        else
        {
            xor128bit(y,ti,bx);
        }
        
    }

    //generat mic
    ax[14]=0;
    ax[15]=0;
    //aes(key,key_o,bx,y,AES_ENC);
    //aes(key,key_o,ax,s,AES_ENC);
    LL_ENC_AES128_Encrypt0(( uint8_t *)key,bx,y);
    LL_ENC_AES128_Encrypt0(( uint8_t *)key,ax,s);

    if(micOut){
        for(int i=0;i<4;i++)
            micOut[i]=y[i]^s[i];
    }
    return PPlus_SUCCESS;

}

                             
