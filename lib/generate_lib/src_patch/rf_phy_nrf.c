#include "rf_phy_driver.h"
#include "global_config.h"
#include "OSAL.h"
#include "OSAL_PwrMgr.h"
#include "log.h"
#include "timer.h"
//#include "phy_plus_phy.h"
#include "ll.h"
#include "ll_hw_drv.h"
#include "clock.h"
#include "gpio.h"
#include "rf_phy_nrf.h"


#define NRF_DBG(...)            //dbg_printf(__VA_ARGS__)
#define NRF_DUMP(...)           //my_dump_byte(__VA_ARGS__)
#define RF_PHY_NRF_DBG  0
#define RF_PHY_NRF_VERSION   (0x010001)

static uint16 s_nrf_lastcrc=0xffff;
static uint8  s_nrf_lastpid=0xff;
nrfPhy_t nrfConfig;

extern uint8_t  phyBufRx[44];
extern uint8_t  phyBufTx[44];

/*********************************************************************
    LOCAL FUNCTIONS
*/

void  cic16_itu_bitshift(uint8_t* dataIn,int bitLen,uint16_t crcSeed,uint8_t* crcCode)
{
    uint8_t bitOut[8]= {0};
    uint8_t i=0;
    uint8_t j=0;
    uint8_t feedback = 0;
    uint8_t reg[16]= {0};
    //uint8_t seed[16];

    for(i=0; i<16; i++)
    {
        reg[i] = (crcSeed>>i)&0x01;
    }
    int byteLen = (bitLen+7)/8;
    int resBit  = bitLen;
    int bitloop=8;
    for(i=0; i<byteLen; i++)
    {
        byte_to_bit(dataIn[i],bitOut);
        bitloop = (resBit>=8) ? 8:resBit;
        for(j=0; j<bitloop; j++)
        {
            feedback = reg[0]^bitOut[j];
            reg[0]   = reg[1];
            reg[1]   = reg[2];
            reg[2]   = reg[3];
            reg[3]   = reg[4]^feedback;
            reg[4]   = reg[5];
            reg[5]   = reg[6];
            reg[6]   = reg[7];
            reg[7]   = reg[8];
            reg[8]   = reg[9];
            reg[9]   = reg[10];
            reg[10]  = reg[11]^feedback;
            reg[11]  = reg[12];
            reg[12]  = reg[13];
            reg[13]  = reg[14];
            reg[14]  = reg[15];
            reg[15]  = feedback;
        }
        resBit-=8;
    }

    bit_to_byte(reg,crcCode);
    bit_to_byte(reg+8,crcCode+1);
}

uint8_t bit_rev_8(uint8_t b)
{
    b = (((uint32_t)b * 0x0802LU & 0x22110LU) |
         ((uint32_t)b * 0x8020LU & 0x88440LU)) * 0x10101LU >> 16;
    return b;
}

uint32_t swapByteToWord(uint8_t* b)
{
    return(BUILD_UINT32(    bit_rev_8(b[3]) ,
                            bit_rev_8(b[2]) ,
                            bit_rev_8(b[1]) ,
                            bit_rev_8(b[0]) )
                            );
    
}

const uint8_t crc8_table_rev[16]={
    0x0,0x1c,0x38,0x24,0x70,0x6c,0x48,0x54,
    0xe0,0xfc,0xd8,0xc4,0x90,0x8c,0xa8,0xb4
};
const uint16_t crc_table_rev[16] ={
0x0000,0x1081,0x2102,0x3183,0x4204,0x5285,0x6306,0x7387,
0x8408,0x9489,0xa50a,0xb58b,0xc60c,0xd68d,0xe70e,0xf78f
};


void cic16_itu_table(uint16_t seed,uint8_t *data, int len,uint8_t* crcCode)
{
    uint16_t crc16 = seed;
    uint16_t crc_4, crc_12;
    while( len-- ) {
        crc_4  = (uint16)(crc16 &0x0f);
        crc_12 = (uint16)((crc16>>4) &0x0fff);
        
        crc16 = (uint16)(crc_12 ^ crc_table_rev[crc_4 ^ (*data & 0x0f)]);

        crc_4  = (uint16)(crc16 &0x0f);
        crc_12 = (uint16)((crc16>>4) &0x0fff);
       
        crc16 = (uint16)(crc_12 ^ crc_table_rev[crc_4 ^ (*data >> 4)]);

        data++;
    }

    crcCode[0]= (crc16&0xff);
    crcCode[1]= (crc16>>8)&0xff;
}
void cic8_itu_table(uint8_t seed,uint8_t *data, int len,uint8_t* crcCode)
{
    uint8_t crc8 = seed;
    uint8_t crc_4, crc_12;
    while( len-- ) {
        crc_4  = (uint8_t)(crc8 &0x0f);
        crc_12 = (uint8_t)((crc8>>4) &0x0fff);
        
        crc8 = (uint8_t)(crc_12 ^ crc8_table_rev[crc_4 ^ (*data & 0x0f)]);

        crc_4  = (uint8_t)(crc8 &0x0f);
        crc_12 = (uint8_t)((crc8>>4) &0x0fff);
       
        crc8 = (uint8_t)(crc_12 ^ crc8_table_rev[crc_4 ^ (*data >> 4)]);

        data++;
    }

    crcCode[0]= crc8;
}
void cic16_itu_poly(uint16 crcSeed,unsigned char *data, int bitLen,uint8_t*crcCode)
{
	unsigned short wCRCin = crcSeed;
	unsigned short wCPoly = 0x8408;//0x8408;
	unsigned char wChar = 0;
	int byteLen = (bitLen+7)>>3;
	//InvertUint16(&wCPoly,&wCPoly);
	while (byteLen--) 	
	{
		wChar= *(data++);
		for(int i = 0;i < (bitLen>8 ?8:bitLen);i++)
		{
            wCRCin^= ((wChar>>i)&0x01);
			if(wCRCin & 0x01)
				wCRCin = (wCRCin >> 1) ^ wCPoly;
			else
				wCRCin = wCRCin >> 1;
		}
		bitLen-=8;
	}
	crcCode[0]= (wCRCin&0xff);
    crcCode[1]= (wCRCin>>8)&0xff;
}

void cic8_itu_poly(uint8 crcSeed,unsigned char *data, int bitLen,uint8_t*crcCode)
{
	unsigned char wCRCin = crcSeed;
	unsigned char wCPoly = 0xe0;//x^8+x^2+x+1;
	unsigned char wChar = 0;
	int byteLen = (bitLen+7)>>3;
	//InvertUint16(&wCPoly,&wCPoly);
	while (byteLen--) 	
	{
		wChar= *(data++);
		for(int i = 0;i < (bitLen>8 ?8:bitLen);i++)
		{
            wCRCin^= ((wChar>>i)&0x01);
			if(wCRCin & 0x01)
				wCRCin = (wCRCin >> 1) ^ wCPoly;
			else
				wCRCin = wCRCin >> 1;
		}
		bitLen-=8;
	}
	crcCode[0]= (wCRCin&0xff);
}
void cic_itu_bitmode(uint16_t seed,uint8_t *data, int bitLen,uint8_t* crcCode)
{
    int byteLen = bitLen/8;
    int resBit  = bitLen - (byteLen*8);
    uint16_t crcOut=0;

    if(nrfConfig.crcByte==CRC_ITU_16_LEN_2BYTE)
    {
        cic16_itu_table(seed,data,byteLen,(uint8_t*)(&crcOut));
        //cic16_itu_bitshift(data+byteLen,resBit,crcOut,crcCode);
        cic16_itu_poly(crcOut,data+byteLen,resBit,crcCode);
    }
    else
    {
        cic8_itu_table((uint8_t)seed,data,byteLen,(uint8_t*)(&crcOut));
        //cic16_itu_bitshift(data+byteLen,resBit,crcOut,crcCode);
        cic8_itu_poly((uint8_t)crcOut,data+byteLen,resBit,crcCode);
    }
    
}


nrfPkt_t nrfTxBuf;
nrfPkt_t nrfRxBuf;
nrfPkt_t nrfAckBuf;

uint32_t nrf_pkt_init(uint8_t* addr,uint8_t addrLen,uint8_t pduLen,uint8_t crcByte,uint8_t nrfMode)
{
    uint32_t syncWord=0;
    uint8_t tmp;

    nrfConfig.addrLen = addrLen;
    nrfConfig.mode    = nrfMode;
    nrfConfig.crcByte = crcByte;
    nrfConfig.version = RF_PHY_NRF_VERSION;
    
    nrfTxBuf.pldLen=pduLen;
    nrfTxBuf.pid=0;
    nrfTxBuf.noAckBit=0;
    
    nrfRxBuf.pldLen=pduLen;
    nrfRxBuf.pid=0;
    nrfRxBuf.noAckBit=0;
    
    nrfAckBuf.pldLen=0;
    nrfAckBuf.pid=0;
    nrfAckBuf.noAckBit=1;
    
    s_nrf_lastcrc = 0xffff;
    s_nrf_lastpid = 0xff;
    //get addr0-add4
    for(int i=0;i<addrLen;i++)
    {
        tmp=bit_rev_8(addr[addrLen-1-i]);// when addrLen <5, LSBtye(NRF addr fmt) is used
        nrfTxBuf.addr[i] = tmp;//
        nrfAckBuf.addr[i]= tmp;
        //phyBufRx[i]=tmp;
        if(i<4)
            syncWord |= (tmp<<(8*i)); 
    }
    if(addrLen==3)
    {
        syncWord = (syncWord<<8)|((syncWord&0x01) ? 0x55 :0xAA);
        osal_memcpy(phyBufRx+1, &nrfTxBuf.addr[0],addrLen);
        phyBufRx[0] = 0xff&syncWord;
    }
    else
    {
        osal_memcpy(phyBufRx, &nrfTxBuf.addr[0],addrLen);
    }
    //NRF_DBG("NRF PKT INIT :MODE %d PDU %d SW[%08x]ADDR L %d : ",nrfMode,nrfTxBuf.pldLen,syncWord,addrLen);
    //NRF_DUMP(&nrfTxBuf.addr[0], addrLen);
    
    return syncWord;
    
}
uint8_t nrf_pkt_crc_check(uint8_t* din)
{   
    uint8_t ret;
    uint8_t *pData;
    uint16 crcCode=0;
    
    if(nrfConfig.addrLen==NRF_ADDR_LEN_3BYTE)
        pData = din+1;
    else
        pData = din;

    cic_itu_bitmode(/*crcSeed*/NRF_CRC_SEED,pData,/*bitLen*/NRF_PHY_BUF_LEN(nrfRxBuf.pldLen),(uint8*)&crcCode);
     
    
    if(nrfConfig.addrLen==NRF_ADDR_LEN_5BYTE)
    {
        ret = (crcCode==0 && nrfTxBuf.addr[4]==phyBufRx[4]);
    }
    else
    {
        ret = (crcCode==0);
    }

    #if 1
    NRF_DBG("RX:ADDR0 tx%02x rx%02x\n",nrfTxBuf.addr[4],phyBufRx[4]);
    NRF_DUMP(phyBufRx,32+4);
    NRF_DBG("CRC Lb%d C%04x\n",NRF_PHY_BUF_LEN(nrfTxBuf.pldLen),crcCode);
    #endif
                
    return ret;
    
}
uint8_t nrf_pkt_dec(uint8_t* din, uint8_t diLen, uint8_t pduLen,nrfPkt_t *p_pkt)
{
    uint8_t ret = 0;
    uint8_t i,tmp0,tmp1;
    uint8_t bof=0;

    //get addr0-add4
    if(nrfConfig.addrLen==NRF_ADDR_LEN_3BYTE)
    {
        bof++;//don't care the first byte, not involved in crc
    }
    
    for(i=0;i<nrfConfig.addrLen;i++)
    {
        p_pkt->addr[i] = (din[bof++]);
    }

    if(nrfConfig.mode == NRF_MODE_ENHANCE_SHOCKBURST)
    {
        //get pkt header
        tmp0 = bit_rev_8(din[bof++]);
        p_pkt->pldLen = (pduLen==0) ?(tmp0>>2) : pduLen; //pduLen=0 for dynamic pkt length
        p_pkt->pid    = (tmp0&0x03);
        NRF_DBG("pldlen = %d\n",tmp0>>2);
        //get pkt no_ack_bit
        tmp0 = din[bof++];
        p_pkt->noAckBit  = (tmp0&0x01);

        //get pkt
        for(i=0;i<p_pkt->pldLen;i++)
        {
            tmp1 = din[bof++];
            p_pkt->pdu[i]=bit_rev_8( (tmp0>>1)|((tmp1&0x01)<<7) );
            tmp0 = tmp1;
        }
        
        //get crc
        for(i=0;i<2;i++)
        {
            tmp1 = din[bof++];
            p_pkt->crc[i]=bit_rev_8( (tmp0>>1)|((tmp1&0x01)<<7) );
            tmp0 = tmp1;
        }
    }
    else
    {
        p_pkt->pldLen = pduLen; //pduLen=0 for dynamic pkt length
        p_pkt->pid    = 0;
        //get pkt
        for(i=0;i<p_pkt->pldLen;i++)
        {
            p_pkt->pdu[i]=bit_rev_8(din[bof++] );
        }
       
        //get crc
        for(i=0;i<nrfConfig.crcByte;i++)
        {
            p_pkt->crc[i]=bit_rev_8(din[bof++]);
        }
    }

    return ret;
}
uint8_t nrf_pkt_enc(nrfPkt_t * p_pkt,uint8_t *dOut, uint8_t* doLen)
{
    uint8_t ret = PPlus_SUCCESS;
    uint8_t i,tmp0,tmp1;
    uint8_t bof=0;
    uint16_t crcSeed = 0xFFFF;
    uint8_t crc[2];
    //fill addr0-5
    if(nrfConfig.addrLen==NRF_ADDR_LEN_3BYTE)
    {
        dOut[bof++]=0xFF;//first byte not be involved in crc , don't care
    }

    for(i=0;i<nrfConfig.addrLen;i++)
    {
        dOut[bof++]= (p_pkt->addr[i]);
    }
    

    if(nrfConfig.mode == NRF_MODE_ENHANCE_SHOCKBURST)
    {
        //fill pcf
        tmp0 = ((p_pkt->pldLen)<<2)| (p_pkt->pid);
        
        dOut[bof++]= bit_rev_8(tmp0);
        //pre-load pkt no_ack_bit
        tmp0 = p_pkt->noAckBit;

        //fill pkt
        for(i=0;i<p_pkt->pldLen;i++)
        {
            tmp1 = p_pkt->pdu[i];
            dOut[bof++]=bit_rev_8( ((tmp0&0x01)<<7) | (tmp1>>1) );
            tmp0 = tmp1;
        }
        //need fill the last bit before do crc
        dOut[bof]=bit_rev_8( ((tmp0&0x01)<<7));
        int bitLen = NRF_PHY_BUF_LEN(p_pkt->pldLen)-nrfConfig.crcByte*8;
        
        if(nrfConfig.addrLen==NRF_ADDR_LEN_3BYTE)
            cic_itu_bitmode(crcSeed,dOut+1,bitLen,crc);
        else
            cic_itu_bitmode(crcSeed,dOut,bitLen,crc);
            
        NRF_DBG("[CRC GEN] %02x %02x\n",crc[0],crc[1]);
        
        dOut[bof] = (crc[0]<<1) | (dOut[bof]   &0x01);bof++;
        dOut[bof] = (crc[1]<<1) | ((crc[0]>>7) &0x01);bof++;
        //fill the last bit
        if(nrfConfig.crcByte==CRC_ITU_16_LEN_2BYTE)
        {    dOut[bof] = (0        ) | ((crc[1]>>7) &0x01);bof++; }
        
        *doLen = bof;
    }
    else
    {
        //fill pkt
        for(i=0;i<p_pkt->pldLen;i++)
        {
            dOut[bof++]=bit_rev_8( p_pkt->pdu[i] );
        }
        int bitLen = NRF_PHY_BUF_LEN(p_pkt->pldLen)-nrfConfig.crcByte*8;
        
        if(nrfConfig.addrLen==NRF_ADDR_LEN_3BYTE)
            cic_itu_bitmode(crcSeed,dOut+1,bitLen,crc);
        else
            cic_itu_bitmode(crcSeed,dOut,bitLen,crc);
            
        NRF_DBG("[CRC GEN] %02x %02x\n",crc[0],crc[1]);
        
        dOut[bof++] = crc[0];
        if(nrfConfig.crcByte==CRC_ITU_16_LEN_2BYTE)
            dOut[bof++] = crc[1];       
        *doLen = bof;
    }

    return ret;
}

uint8_t nrf_pkt_gen(uint8_t *din, uint8_t dLen,uint8_t *dout)
{
    int i;
    uint8_t oLen;
    nrfTxBuf.pid=(nrfTxBuf.pid+1)&0x03;
   

    if(dLen == 0 || dLen> nrfTxBuf.pldLen)
    {
        oLen=nrfTxBuf.pldLen;
    }
    else
    {
        oLen=dLen;
    }

    for(i=0;i<oLen;i++)
        nrfTxBuf.pdu[i]=din[i];
    for(i=oLen;i<0x20;i++)
        nrfTxBuf.pdu[i]=0;
    

    nrf_pkt_enc(&nrfTxBuf,dout,&oLen);
    return oLen;

}

uint8_t nrf_rxdata_check(uint8_t *din)
{
    uint8_t ret = PPlus_ERR_FATAL;
       
    nrf_pkt_dec(din,/*bitLen*/NRF_PHY_BUF_LEN(nrfRxBuf.pldLen),nrfRxBuf.pldLen,&nrfRxBuf);

    if( nrfConfig.addrLen== NRF_ADDR_LEN_5BYTE 
      &&  nrfRxBuf.addr[4]!=nrfTxBuf.addr[4]) 
    {
        NRF_DBG("[addr]: rx%02x tx%02x\n",nrfRxBuf.addr[4],nrfTxBuf.addr[4]);
        ret = PPlus_ERR_INVALID_ADDR;
        return ret;
    }

    if(nrfConfig.mode==NRF_MODE_SHOCKBURST)
        return PPlus_SUCCESS;

    uint16 crc = BUILD_UINT16(nrfRxBuf.crc[0],nrfRxBuf.crc[1]);
    if(nrfRxBuf.pid==s_nrf_lastpid && crc==s_nrf_lastcrc)
    {
        NRF_DBG("[pid]: rx%02x last %02x\n",nrfRxBuf.pid,s_nrf_lastpid);
        NRF_DBG("[crc]: rx%04x last %04x\n",crc,s_nrf_lastcrc);
        ret = PPlus_ERR_INVALID_FLAGS;
        return ret;
    }
    s_nrf_lastpid = nrfRxBuf.pid;
    s_nrf_lastcrc = crc;

    return PPlus_SUCCESS;
    
}

uint8_t nrf_txack_check(uint8_t *din)
{
    uint8_t ret = PPlus_ERR_FATAL;
    uint16 crcCode=0;
    uint8_t *pData;
    
    if(nrfConfig.addrLen==NRF_ADDR_LEN_3BYTE)
        pData=din+1;
    else
        pData = din;
        
    //uint16_t crcSeed = 0xFFFF;
    //int bitLen = (5+1+0+2)*8+1;//crc include the addr + pcf + pdu + crc
    cic_itu_bitmode(/*crcSeed*/NRF_CRC_SEED,pData,/*bitLen*/NRF_TX_ACK_BIT_LEN,(uint8_t*)&crcCode);
    NRF_DBG("[ACK]:crc %04x ",crcCode);
    NRF_DUMP(din,10);

    if(crcCode!=0)
        return ret;
        
    nrf_pkt_dec(din,/*bitLen*/NRF_TX_ACK_BIT_LEN,0,&nrfAckBuf);

    if( nrfConfig.addrLen== NRF_ADDR_LEN_5BYTE
        && nrfAckBuf.addr[4]!=nrfTxBuf.addr[4]) 
    {
        ret = PPlus_ERR_INVALID_ADDR;
        NRF_DBG("[addr]: rx%02x tx%02x\n",nrfAckBuf.addr[4],nrfTxBuf.addr[4]);
        return ret;
    }
    if(nrfConfig.mode == NRF_MODE_ENHANCE_SHOCKBURST &&
        nrfAckBuf.pid!=nrfTxBuf.pid)
    {
        NRF_DBG("[pid]: rx%02x tx%02x\n",nrfAckBuf.pid,nrfTxBuf.pid);
        ret = PPlus_ERR_INVALID_DATA;
        return ret;
    }
    

    return PPlus_SUCCESS;
    
}


#if RF_PHY_NRF_DBG
uint8_t tst_pkt_dec(uint8_t* din,uint8_t diLen)
{
    uint8_t ret=0;
    uint8_t dlen = diLen;
    //uint16_t crcSeed = 0xFFFF;
    //uint8_t crcCode[2];

    NRF_DBG("%08x:------\n",din);
    NRF_DUMP(din, dlen);
//    int bitLen = NRF_PHY_BUF_LEN(32);//crc include the addr + pcf + pdu + crc
//
//
//    cic16_itu_bitmode(crcSeed,din,bitLen,crcCode);
//    NRF_DBG("[PKT DEC]ITU CRC bL%3d %02x %02x \n",bitLen,crcCode[0],crcCode[1]);
    ret =nrf_pkt_crc_check(din);

    nrf_pkt_dec(din,diLen,nrfRxBuf.pldLen,&nrfRxBuf);
    NRF_DBG("[PKT DEC] ADDR:");
    NRF_DUMP(&nrfRxBuf.addr[0], 5);
    NRF_DBG("[PKT DEC] PCF: l%2x pid %2x no_ack %2x\n",nrfRxBuf.pldLen,nrfRxBuf.pid,nrfRxBuf.noAckBit);
    NRF_DBG("[PKT DEC] PDU:");
    NRF_DUMP(&nrfRxBuf.pdu[0], nrfRxBuf.pldLen);
    NRF_DBG("[PKT DEC] CRC:%02x %02x\n",nrfRxBuf.crc[0],nrfRxBuf.crc[1]);
    return ret;

}
#endif

#if RF_PHY_NRF_DBG
uint8_t tst_pkt_enc(uint8_t addrLen,uint8_t pduLen,uint8_t crcByte, uint8_t mode)
{
    uint8_t ret=0;
    uint8_t din[32];
    uint8_t addr[5]={0x01,0xd6,0x89,0xbe,0x8e};//LSByte lsb send first
    //uint8_t addr[5]={0xd6,0x89,0xbe,0x8e,0xc7};//LSByte lsb send first
    int i;
    for(i=0;i<32;i++)
        din[i]=0x10+i;
    din[0]=7;
    din[1]=0;
    din[2]=0;
    din[3]=0;

    nrf_pkt_init(addr,addrLen,pduLen,crcByte,mode);//NRF_MODE_ENHANCE_SHOCKBURST

    uint8_t doLen;
    for(i=0;i<1;i++)
    {
        din[0]++;
        doLen=nrf_pkt_gen(din, pduLen,phyBufTx);
      
        NRF_DBG("#%d :------------------------\n",i);
        NRF_DBG("[DIN]");
        NRF_DUMP(din, pduLen);
        NRF_DBG("[DOUT]");
        NRF_DUMP(phyBufTx, doLen);

        if(tst_pkt_dec(phyBufTx,doLen)==0)
            ret++;
        
    }

    return ret;
}
#endif
#if RF_PHY_NRF_DBG
void test_crc(void)
{
    uint8_t dataIn[45];
    uint8_t dLen = sizeof(dataIn);
    for(int i=0;i<dLen;i++)
    {
        dataIn[i]=i;
    }
    dataIn[dLen-5]=0xff;
    dLen -=5;
    int bitLen = dLen*8 + 3;

    uint16_t crcSeed = 0xFFFF;

    
    uint8_t crcCode[2]={0,0};
    uint32_t t0,t1;
    t0=read_current_fine_time();
    cic16_itu_bitshift(dataIn,dLen*8,crcSeed,crcCode);
    t1=read_current_fine_time();
    NRF_DBG("zbcrc L%3d T%4d %02x %02x \n",dLen,t1-t0,crcCode[0],crcCode[1]);
    
    t0=read_current_fine_time();
    cic16_itu_poly(crcSeed,dataIn,dLen*8,crcCode);
    t1=read_current_fine_time();
    NRF_DBG("ITU-T L%3d T%4d %02x %02x \n",dLen,t1-t0,crcCode[0],crcCode[1]);

    dataIn[dLen]=crcCode[0];
    dataIn[dLen+1]=crcCode[1];
    
    dLen=dLen+2;
    t0=read_current_fine_time();
    cic16_itu_poly(crcSeed,dataIn,dLen*8,crcCode);
    t1=read_current_fine_time();
    NRF_DBG("ITU-T L%3d T%4d %02x %02x \n",dLen,t1-t0,crcCode[0],crcCode[1]);

    t0=read_current_fine_time();
    cic16_itu_table(crcSeed,dataIn,dLen,crcCode);
    t1=read_current_fine_time();
    NRF_DBG("CRC-T L%3d T%4d %02x %02x \n",dLen,t1-t0,crcCode[0],crcCode[1]);

    t0=read_current_fine_time();
    cic16_itu_poly(crcSeed,dataIn,bitLen,crcCode);
    t1=read_current_fine_time();
    NRF_DBG("ITU-T bL%3d T%4d %02x %02x \n",bitLen,t1-t0,crcCode[0],crcCode[1]);

    t0=read_current_fine_time();
    cic16_itu_bitshift(dataIn,bitLen,crcSeed,crcCode);
    t1=read_current_fine_time();
    NRF_DBG("zbcrc bL%3d T%4d %02x %02x \n",bitLen,t1-t0,crcCode[0],crcCode[1]);

#if 0
    bitLen = 32;
    for (int i=0;i<128;i++)
    {
        NRF_DBG("#%d ",bitLen);
        cic16_itu_poly(crcSeed,dataIn,bitLen,crcCode);
        NRF_DBG("ply %02x %02x |",crcCode[0],crcCode[1]);
        cic16_itu_bitshift(dataIn,bitLen,crcSeed,crcCode);
        NRF_DBG("sft %02x %02x \n",crcCode[0],crcCode[1]);
        bitLen++;
    }
#endif
    uint8_t addrLen[]={5,4,3};
    uint8_t pduLen[] ={32,31,20,19,8,7,6};
    uint8_t crcByte[]={2,1};
    uint8_t nrfMode[]={1,0};

    int num=0,fcnt=0,i=0,j=0,k=0,h=0;
    uint8_t ret;
    for( i=0;i<sizeof(addrLen);i++)
    {
        for(j=0;j<sizeof(pduLen);j++)
        {
            for(h=0;h<sizeof(crcByte);h++)
            {
                for(k=0;k<sizeof(nrfMode);k++)
                {
                    
                    ret=tst_pkt_enc(addrLen[i],pduLen[j],crcByte[h],nrfMode[k]);
                    if(ret)
                        fcnt++;
                    num++;
                    NRF_DBG("[#TC%3d]ret %d AddrL %d,pduL %d CrcByte %d Mode %d,\n",num,ret,addrLen[i],pduLen[j],crcByte[h],nrfMode[k]);
                }
            }
        }
    }
    NRF_DBG("----------------------------------------------------\n");
    NRF_DBG("Test Case All Done: FailRate %d/%d\n",fcnt,num);
    NRF_DBG("----------------------------------------------------\n");

    while(1);
#if 0
    tst_pkt_dec(rxTmp0,sizeof(rxTmp0));
    tst_pkt_dec(rxTmp1,sizeof(rxTmp1));
        tst_pkt_dec(rxTmp2,sizeof(rxTmp0));
    tst_pkt_dec(rxTmp3,sizeof(rxTmp1));
        tst_pkt_dec(rxTmp4,sizeof(rxTmp0));
#endif
}
#endif

