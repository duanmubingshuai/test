/*******************************************************************************
    Filename:       aoaEstAlgo.c
    Revised:
    Revision:

    Description:    This file contains the BLE CTE AOA Estimation Algorithms


*******************************************************************************/

/*******************************************************************************
    INCLUDES
*/

#include "types.h"
#include "aoaEstAlgo.h"
#include "aoaComFun.h"
#include <stdio.h>
//#include "FileOpera.h"
/*******************************************************************************
    AOA EST BUILD CONFIG
*/

#define _AOA_EST_ANT_MODE_        CTE_AOA_ANT_URA
/*******************************************************************************
    MACROS
*/


/*******************************************************************************
    CONSTANTS
*/
#define CTE_REF_PHASE_IQ                                                     \
    {                                                                        \
        {1, 0}, {0, -1}, {-1, 0}, {0, 1}, {1, 0}, {0, -1}, {-1, 0}, { 0, 1 } \
    }
// (exp(-j*0.5*pi*(0:7)))

#define AOA_MF_COARSE_SEARCH_TLUT                                             \
    {-80, -64, -48, -32, -16,   0,  16,  32,  48,  64,  80} //theta lUT for Match Filter Coarse Search

#define AOA_MF_COARSE_SEARCH_TLUT_INTV10                                             \
    {-80, -70, -60, -50, -40, -30,-20,-10, 0, 10, 20, 30, 40, 50, 60, 70, 80} //theta lUT for Match Filter Coarse Search


#define CTE_GUARD_LEN                   4
#define CTE_REF_LEN                     8
#define CTE_GUARD_REF_LEN               ( CTE_GUARD_LEN + CTE_REF_LEN )
#define CTE_MAX_SLOT_SAMPLE             74
#define CTE_MAX_SAMPLE_LEN              ( CTE_REF_LEN + CTE_MAX_SLOT_SAMPLE )   // only process the ref + ant IQ sample

#define CTE_F0_EST_LEN                  4

#define MFPF_PEAK_FINDING_NUM           4           //MF Coarse Search inteval -> 2^MFPF_PEAK_FINDING_NUM

#define URA8_COL_ROW_NUM                (3)

#define RMFS_MAX_ZETA_LUT_LEN           (5)
#define RMFS_MAX_THETA_LUT_LEN          (21)


#define MF_GEN_MATCH_ULA                (0x01)
#define MF_GEN_MATCH_URA8_XAXIS         (0x02)
#define MF_GEN_MATCH_URA8_ZAXIS         (0x03)
#define MF_GEN_MATCH_URA8_MTRIX         (0x04)

/*******************************************************************************
    TYPEDEFS
*/
typedef struct uxaEstCtx
{
    cplxInt di[CTE_MAX_SLOT_SAMPLE];
    uint8_t sampPerAnt;
    uint8_t iqLen;
} uxaEstCtx_t;

typedef struct uxaEstStats
{
    cplxInt mfIQ;
    uint32_t mVal;
    int8_t aEst;
    int8_t zEst;            //for URA only
} uxaEstStats_t;

typedef struct mfGenCtx
{
    int8_t theta ;
    int8_t zeta;
    uint8_t match;
} mfGenCtx_t;
/*******************************************************************************
    LOCAL VARIABLES
*/
static uxaEstCtx_t  s_uxa;
static mfGenCtx_t   s_mfCfg;
/*******************************************************************************
    GLOBAL VARIABLES
*/

int cte_ref_freq_est(cplxInt* di, uint8 nf0d)
{
    uint8 i = 0;
    cplxInt refPhy[CTE_REF_LEN] = CTE_REF_PHASE_IQ;
    cplxInt rtRes[CTE_REF_LEN];
    cplxInt fnd[CTE_F0_EST_LEN];
    int phy[CTE_F0_EST_LEN];
    int fEst0 = 0;
    int nSum = 0;
    CPLX_DOT(/*in*/ di, /*in*/ refPhy, /*out*/ rtRes, /*len*/ CTE_REF_LEN, /*rSftb*/ 0);

    // for (i = 0; i < 8; i++)
    // {
    //     AOA_DBG("[ #%d di%4d dq%4d ri%4d rq%4d rti%4d rtq%4d]\n",
    //             i, di[i].I, di[i].Q, refPhy[i].I, refPhy[i].Q, rtRes[i].I, rtRes[i].Q);
    // }

    for (i = 0; i < nf0d; i++)
    {
        cplxMatch(rtRes + i + 1, rtRes, fnd + i, /*len*/ (CTE_REF_LEN-1-i), /*inc*/1,/*rSftb*/7);
        FIXPOINT_GET_ANGLE(fnd[i].I, fnd[i].Q, phy + i, NULL);
        fEst0 += phy[i];
        nSum += i + 1;
        AOA_DBG("[FREQ0 #%d i%7d q%7d p%7d]\n", i + 1, fnd[i].I, fnd[i].Q, phy[i]);
    }

    fEst0 = (fEst0 + (nSum >> 1)) / nSum;
    fEst0 +=(1<<(PI_QUNT_BIT-1) );  //add hidx=0.5 250KHz offset
    AOA_DBG("[FREQ0 %d %f MHz]\n", fEst0,(fEst0*1.0)/(1<<(PI_QUNT_BIT+1)));
    return fEst0;
}

int cte_uxa_fine_freq_est(cplxInt* di, uint8 sampPerAnt,uint8 antNum)
{
    uint8 i;
    cplxInt fAntD[antNum];
    int phy[antNum];
    int fEst1=0;

    for(i=0; i<antNum; i++)
    {
        cplxMatch(di + i + antNum, di+i, fAntD + i,
                  /*len*/ (sampPerAnt-1)*antNum,/*inc*/antNum, /*rSftb*/6);
        FIXPOINT_GET_ANGLE(fAntD[i].I, fAntD[i].Q, phy + i, NULL);
        fEst1 += phy[i];
        AOA_DBG("[FREQ1 #%d i%7d q%7d p%7d]\n", i, fAntD[i].I, fAntD[i].Q, phy[i]);
    }

    fEst1= fEst1/antNum; // mean over n ant
    fEst1 = (fEst1+(antNum>>1))/antNum; // fest1 per sample slot
    AOA_DBG("[FREQ1 %d]\n", fEst1);
    return fEst1;
}

void angle_mf_gen(mfGenCtx_t mf,aoaEstCtx_t ctx,uint8 oLen,cplxInt* mOut)
{
    int tmpSft,phyTmp,phyTmpZ;

    if(mf.match==MF_GEN_MATCH_ULA)
    {
        //-------------------------------------------------------------------
        //mOut   = exp(1j * 2 * pi * antPattern'*phyTmp);
        //phyTmp = (antD/lambda) * sin( theta *pi/180 )
        tmpSft = (PI_QUNT_BIT + 1 - (TQA_AMP_BIT - 1));
        phyTmp = ((ctx.antDist * TqaSIN(((int)mf.theta * (1 << PI_QUNT_BIT) + 90) / 180) << tmpSft) + (ctx.lambda >> 1)) / ctx.lambda;
        //AOA_DBG("[MF_GEN %4d antD%6d lbd%6d p%6d Tqa %d]\n", theta,ctx.antDist,ctx.lambda,phyTmp);
    }
    else if(mf.match==MF_GEN_MATCH_URA8_XAXIS)
    {
        //-------------------------------------------------------------------
        //mOut   = exp(1j * 2 * pi * antPattern'*phyTmp);
        //phyTmp(n,m) = n*(antD/lambda) * sin( theta *pi/180 ) * cos( zeta *pi/180 ) @x axis
        //            + m*(antD/lambda) * sin( zeta *pi/180 )                        @z axis
        // only cal x axis phy diff
        tmpSft = (PI_QUNT_BIT + 1 - (TQA_AMP_BIT - 1));
        phyTmp = ((ctx.antDist * TqaSIN(((int)mf.theta * (1 << PI_QUNT_BIT) + 90) / 180) << tmpSft) + (ctx.lambda >> 1)) / ctx.lambda;
        phyTmp = bROUND((phyTmp       * TqaCOS(((int)mf.zeta * (1 << PI_QUNT_BIT) + 90) / 180)),(TQA_AMP_BIT - 1));
        //AOA_DBG("[MF_GEN %4d antD%6d lbd%6d p%6d Tqa %d]\n", theta,ctx.antDist,ctx.lambda,phyTmp);
    }
    else if(mf.match==MF_GEN_MATCH_URA8_ZAXIS)
    {
        //-------------------------------------------------------------------
        //mOut   = exp(1j * 2 * pi * antPattern'*phyTmp);
        //phyTmp(n,m) = n*(antD/lambda) * sin( theta *pi/180 ) * cos( zeta *pi/180 ) @x axis
        //            + m*(antD/lambda) * sin( zeta *pi/180 )                        @z axis
        // only cal z axis phy diff
        tmpSft = (PI_QUNT_BIT + 1 - (TQA_AMP_BIT - 1));
        phyTmp = ((ctx.antDist * TqaSIN(((int)mf.zeta * (1 << PI_QUNT_BIT) + 90) / 180) << tmpSft) + (ctx.lambda >> 1)) / ctx.lambda;
        //AOA_DBG("[MF_GEN %4d antD%6d lbd%6d p%6d Tqa %d]\n", theta,ctx.antDist,ctx.lambda,phyTmp);
    }
    else if(mf.match==MF_GEN_MATCH_URA8_MTRIX)
    {
        //-------------------------------------------------------------------
        //mOut   = exp(1j * 2 * pi * antPattern'*phyTmp);
        //phyTmp(n,m) = n*(antD/lambda) * sin( theta *pi/180 ) * cos( zeta *pi/180 ) @x axis
        //            + m*(antD/lambda) * sin( zeta *pi/180 )                        @z axis
        // only cal x axis phy diff
        tmpSft = (PI_QUNT_BIT + 1 - (TQA_AMP_BIT - 1));
        phyTmp = ((ctx.antDist * TqaSIN(((int)mf.theta * (1 << PI_QUNT_BIT) + 90) / 180) << tmpSft) + (ctx.lambda >> 1)) / ctx.lambda;
        phyTmp = bROUND((phyTmp       * TqaCOS(((int)mf.zeta * (1 << PI_QUNT_BIT) + 90) / 180)),(TQA_AMP_BIT - 1));
        phyTmpZ = ((ctx.antDist * TqaSIN(((int)mf.zeta * (1 << PI_QUNT_BIT) + 90) / 180) << tmpSft) + (ctx.lambda >> 1)) / ctx.lambda;
    }

    //--------------------------------------------------------------------------
    //generate mf

    if(mf.match==MF_GEN_MATCH_URA8_MTRIX)
    {
        //ura8 012
        for(uint8_t i = 0; i<3; i++)
        {
            TqaNCO(phyTmp * (i + 1), &(mOut[i].I), &(mOut[i].Q), /*tqaComp*/ 1);
        }

        //ura8 3 4
        TqaNCO(phyTmp * (0 + 1)+phyTmpZ, &(mOut[3].I), &(mOut[3].Q), /*tqaComp*/ 1);
        TqaNCO(phyTmp * (2 + 1)+phyTmpZ, &(mOut[4].I), &(mOut[4].Q), /*tqaComp*/ 1);

        //ura 567
        for(uint8_t i = 0; i<3; i++)
        {
            TqaNCO(phyTmp * (i + 1) + phyTmpZ*2, &(mOut[5+i].I), &(mOut[5+i].Q), /*tqaComp*/ 1);
        }
    }
    else
    {
        //uxa antPattern[0:1:antNum]
        for (uint8_t i = 0; i < oLen; i++)
        {
            TqaNCO(phyTmp * (i + 1), &(mOut[i].I), &(mOut[i].Q), /*tqaComp*/ 1);
            //AOA_DBG("[%6d %6d] ", mOut[i].I,mOut[i].Q);
        }

        /* code */
    }

    //AOA_DBG("\n ");
}

void mf_cplx_match(aoaEstCtx_t ctx,cplxInt* sumAnt,cplxInt* antSearch,int rSftbMf,uxaEstStats_t* xMtx)
{
    uint32_t mVal[URA8_COL_ROW_NUM];
    uint8_t i;

    if (s_mfCfg.match == MF_GEN_MATCH_ULA)
    {
        s_mfCfg.theta = xMtx->aEst;
        angle_mf_gen(s_mfCfg, ctx, ctx.antNum, antSearch);
        cplxMatch(sumAnt, antSearch, &(xMtx->mfIQ), /*len*/ ctx.antNum, /*inc*/ 1, /*rSftb*/ rSftbMf);
        xMtx->mVal = FIXPOINT_XABS(xMtx->mfIQ);
    }
    else if(s_mfCfg.match == MF_GEN_MATCH_URA8_XAXIS ||
            s_mfCfg.match == MF_GEN_MATCH_URA8_ZAXIS )
    {
        //curAntN = (s_mfCfg.match==MF_GEN_MATCH_URA8_XAXIS) ? ctx.antNumCol :ctx.antNumRow;
        //only support 3x3-1 URA
        s_mfCfg.theta = xMtx->aEst;
        s_mfCfg.zeta  = s_mfCfg.match == MF_GEN_MATCH_URA8_ZAXIS ? xMtx->aEst : s_mfCfg.zeta ;
        angle_mf_gen(s_mfCfg, ctx, URA8_COL_ROW_NUM, antSearch);

        for(i=0; i<URA8_COL_ROW_NUM; i++)
        {
            cplxMatch(&(sumAnt[i*URA8_COL_ROW_NUM]), antSearch, &(xMtx->mfIQ), /*len*/ URA8_COL_ROW_NUM, /*inc*/ 1, /*rSftb*/ rSftbMf);
            mVal[i] = FIXPOINT_XABS(xMtx->mfIQ);
        }

        xMtx->mVal = (3*mVal[0] + 2*mVal[1] + 3*mVal[2])>>3;// for ura8
        //xMtx->mVal = (mVal[0]);
        //int idx = FindMaxDec((int*)mVal,3);
        //xMtx->mVal=mVal[idx];
    }
    else if(s_mfCfg.match == MF_GEN_MATCH_URA8_MTRIX)
    {
        s_mfCfg.theta = xMtx->aEst;
        s_mfCfg.zeta  = xMtx->zEst;
        angle_mf_gen(s_mfCfg, ctx, ctx.antNum, antSearch);
        cplxMatch(sumAnt, antSearch, &(xMtx->mfIQ), /*len*/ ctx.antNum, /*inc*/ 1, /*rSftb*/ rSftbMf);
        xMtx->mVal = FIXPOINT_XABS(xMtx->mfIQ);
    }
}

int8_t cte_ula_mf_coarse_fine(cplxInt* di, uint8_t sampPerAnt,aoaEstCtx_t ctx)
{
    int8_t thetaLut[] = AOA_MF_COARSE_SEARCH_TLUT_INTV10;
    uint8_t schLen = sizeof(thetaLut) / sizeof(int8_t);
    cplxInt antSearch[ctx.antNum];
    uint8_t i,j,k;
    uxaEstStats_t aMtx0,aMtx1,bMtx[schLen];
    int val[schLen];
    int mIdx=0;
    cplxInt sumAnt[ctx.antNum];

    for (j = 0; j < ctx.antNum; j++)
    {
        sumAnt[j].I = 0;
        sumAnt[j].Q = 0;

        for (k = 0; k < sampPerAnt; k++)
        {
            sumAnt[j].I += di[j + k * ctx.antNum].I;
            sumAnt[j].Q += di[j + k * ctx.antNum].Q;
        }
    }

    int rSftbMf = (/*diB*/AOA_DI_INPUT_BITWTH + /*antN Gain*/ 3) \
                  + (/*antSearch*/(TQA_AMP_BIT-1) +/*antN Gain */3) \
                  -/*outB*/ AOA_DI_ANT_MATCH_OUT_BITWTH;
    s_mfCfg.match = MF_GEN_MATCH_ULA;

    //Step 1 matching filter coarse search
    for(i=0; i<schLen; i++)
    {
        bMtx[i].aEst =thetaLut[i];
        s_mfCfg.theta = bMtx[i].aEst;
        angle_mf_gen(s_mfCfg,ctx,ctx.antNum,antSearch);
        cplxMatch(sumAnt, antSearch, &(bMtx[i].mfIQ), /*len*/ ctx.antNum, /*inc*/ 1, /*rSftb*/ rSftbMf);
        bMtx[i].mVal = FIXPOINT_XABS(bMtx[i].mfIQ);
        val[i] = bMtx[i].mVal;
        AOA_DBG("[MFCS %4d %6d ]\n", bMtx[i].aEst,bMtx[i].mVal);
    }

    mIdx = FindMaxDec(val,schLen);
    AOA_DBG("\n[MFCS{%4d %6d} ]\n", bMtx[mIdx].aEst,bMtx[mIdx].mVal);

    //Step 2 cal fine search starting point
    if(mIdx==0)
    {
        aMtx1.aEst =-90;
        s_mfCfg.theta = aMtx1.aEst;
        angle_mf_gen(s_mfCfg,ctx,ctx.antNum,antSearch);
        // uxa_angle_match(sumAnt,antSearch,sampPerAnt,ctx,&(aMtx1));
        cplxMatch(sumAnt, antSearch, &(aMtx1.mfIQ), /*len*/ ctx.antNum, /*inc*/ 1, /*rSftb*/ rSftbMf);
        aMtx1.mVal = FIXPOINT_XABS(aMtx1.mfIQ);

        if (aMtx1.mVal> bMtx[mIdx + 1].mVal)
        {
            aMtx1.aEst = -90;
        }
        else
        {
            aMtx1.aEst = bMtx[mIdx].aEst+1;
        }
    }
    else if(mIdx==schLen-1)
    {
        aMtx0.aEst =90;
        s_mfCfg.theta = aMtx0.aEst;
        angle_mf_gen(s_mfCfg,ctx,ctx.antNum,antSearch);
        // uxa_angle_match(sumAnt,antSearch,sampPerAnt,ctx,&(aMtx1));
        cplxMatch(sumAnt, antSearch, &(aMtx1.mfIQ), /*len*/ ctx.antNum, /*inc*/ 1, /*rSftb*/ rSftbMf);
        aMtx1.mVal = FIXPOINT_XABS(aMtx1.mfIQ);

        if (aMtx1.mVal > bMtx[mIdx - 1].mVal)
        {
            aMtx1.aEst = 81;
        }
        else
        {
            aMtx1.aEst = bMtx[mIdx-1].aEst+1;
        }
    }
    else
    {
        if(bMtx[mIdx-1].mVal>bMtx[mIdx+1].mVal)
        {
            aMtx1.aEst = bMtx[mIdx-1].aEst+1;
        }
        else
        {
            aMtx1.aEst = bMtx[mIdx].aEst+1;
        }
    }

    //Step 3 fine search
    uint8_t fineSchLen = bMtx[1].aEst-bMtx[0].aEst-1;
    aMtx0.mVal = 0;

    for(i=0; i<fineSchLen; i++)
    {
        s_mfCfg.theta = aMtx1.aEst;
        angle_mf_gen(s_mfCfg,ctx,ctx.antNum,antSearch);
        // uxa_angle_match(sumAnt,antSearch,sampPerAnt,ctx,&(aMtx1));
        cplxMatch(sumAnt, antSearch, &(aMtx1.mfIQ), /*len*/ ctx.antNum, /*inc*/ 1, /*rSftb*/ rSftbMf);
        aMtx1.mVal = FIXPOINT_XABS(aMtx1.mfIQ);
        AOA_DBG("[FINE %4d %6d i%6d q%6d ]\n", aMtx1.aEst,aMtx1.mVal,aMtx1.mfIQ.I,aMtx1.mfIQ.Q);

        if(aMtx0.mVal<aMtx1.mVal)
        {
            aMtx0.mVal = aMtx1.mVal;
            aMtx0.aEst = aMtx1.aEst;
        }

        aMtx1.aEst =aMtx1.aEst+1;
    }

    if(bMtx[mIdx].mVal> aMtx0.mVal)
    {
        aMtx0.mVal = bMtx[mIdx].mVal;
        aMtx0.aEst = bMtx[mIdx].aEst;
    }

    AOA_DBG("\n[MFFS{%4d %6d} ]\n", aMtx0.aEst,aMtx0.mVal);
    return aMtx0.aEst;
}

int8_t mf_corase_serach_peak_finding(aoaEstCtx_t ctx,cplxInt* sumAnt,int8_t* thetaLut,uint8_t schLen,uint8 mfLen,int rSftbMf)
{
    cplxInt antSearch[mfLen];
    uint8_t i;
    int8_t nextT;
    uxaEstStats_t aMtx0, aMtx1, bMtx;
    //Step 1 matching filter coarse search
    aMtx0.mVal = 0;

    for(i=0; i<schLen; i++)
    {
        aMtx1.aEst =thetaLut[i];
        mf_cplx_match(ctx,/*din*/sumAnt,/*antLut*/antSearch,/*bitSft*/rSftbMf,/*out*/&(aMtx1));
        AOA_DBG("[MFCS %4d %6d ]\n", aMtx1.aEst,aMtx1.mVal);

        if(aMtx0.mVal<aMtx1.mVal)
        {
            aMtx0.mVal = aMtx1.mVal;
            aMtx0.aEst = aMtx1.aEst;
        }
    }

    AOA_DBG("\n[MFCS{%4d %6d} ]\n", aMtx0.aEst,aMtx0.mVal);

    //Step 2 Peak Finding
    for(i=0; i<MFPF_PEAK_FINDING_NUM+1; i++)
    {
        if (i == 0)
        {
            aMtx1.aEst = aMtx0.aEst + 1;
            mf_cplx_match(ctx,/*din*/sumAnt,/*antLut*/antSearch,/*bitSft*/rSftbMf,/*out*/&(aMtx1));
        }
        else
        {
            aMtx0.aEst = nextT;
            mf_cplx_match(ctx,/*din*/sumAnt,/*antLut*/antSearch,/*bitSft*/rSftbMf,/*out*/&(aMtx0));
            aMtx1.aEst = aMtx0.aEst + 1;
            mf_cplx_match(ctx,/*din*/sumAnt,/*antLut*/antSearch,/*bitSft*/rSftbMf,/*out*/&(aMtx1));
        }

        if(aMtx0.mVal>aMtx1.mVal)
        {
            nextT = (   aMtx0.aEst - (1<<(MFPF_PEAK_FINDING_NUM -1 - i)) > -90 ) ?
                    aMtx0.aEst - (1<<(MFPF_PEAK_FINDING_NUM -1 - i)) : -90;
            bMtx.aEst = aMtx0.aEst;
            bMtx.mVal = aMtx0.mVal;
        }
        else if(aMtx0.mVal<aMtx1.mVal)
        {
            nextT = (   aMtx0.aEst + (1 << (MFPF_PEAK_FINDING_NUM - 1 - i)) < 90) ?
                    aMtx0.aEst + (1 << (MFPF_PEAK_FINDING_NUM - 1 - i)) : 90;
            bMtx.aEst = aMtx1.aEst;
            bMtx.mVal = aMtx1.mVal;
        }
        else
        {
            bMtx.aEst = aMtx1.aEst;
            bMtx.mVal = aMtx1.mVal;
            break;
        }

        AOA_DBG("[MFPF%4d %6d %4d %6d nextT%3d ]\n", aMtx0.aEst,aMtx0.mVal,aMtx1.aEst,aMtx1.mVal,nextT);
    }

    AOA_DBG("\n[MFPF{%4d %6d} ]\n", bMtx.aEst,bMtx.mVal);
    return bMtx.aEst;
}
void mf_matrix_fine_search(/*in*/ aoaEstCtx_t ctx,cplxInt* sumAnt,\
                                  /*search lut*/ int8_t* thetaLut,uint8_t tLen,int8_t* zetaLut,uint8_t zLen,\
                                  /*mf*/uint8 mfLen,int rSftbMf,
                                  /*out*/uxaEstStats_t* uxaRes)
{
    cplxInt antSearch[mfLen];
    uint8_t i,j;
    uxaEstStats_t aMtx0, aMtx1;
    //Step 1 matching filter coarse search
    aMtx0.mVal = 0;

    for (i = 0; i < zLen; i++)
    {
        aMtx1.zEst = zetaLut[i];

        for (j = 0; j < tLen; j++)
        {
            aMtx1.aEst = thetaLut[j];
            mf_cplx_match(ctx, /*din*/ sumAnt, /*antLut*/ antSearch, /*bitSft*/ rSftbMf, /*out*/ &(aMtx1));
            AOA_DBG("[RMFS z%3d t%3d %6d ]\n", aMtx1.zEst, aMtx1.aEst, aMtx1.mVal);

            if (aMtx0.mVal < aMtx1.mVal)
            {
                aMtx0.mVal = aMtx1.mVal;
                aMtx0.aEst = aMtx1.aEst;
                aMtx0.zEst = aMtx1.zEst;
            }
        }
    }

    AOA_DBG("\n[RMFS{z%3d t%4d %6d} ]\n", aMtx0.zEst, aMtx0.aEst,aMtx0.mVal);
    uxaRes->aEst = aMtx0.aEst;
    uxaRes->zEst = aMtx0.zEst;
}

int8_t cte_ula_mf_peak_finding(cplxInt* di, uint8_t sampPerAnt,aoaEstCtx_t ctx)
{
    int8_t thetaLut[] = AOA_MF_COARSE_SEARCH_TLUT;
    uint8_t schLen = sizeof(thetaLut) / sizeof(int8_t);
    uint8_t j,k;
    cplxInt sumAnt[ctx.antNum];

    for (j = 0; j < ctx.antNum; j++)
    {
        sumAnt[j].I = 0;
        sumAnt[j].Q = 0;

        for (k = 0; k < sampPerAnt; k++)
        {
            sumAnt[j].I += di[j + k * ctx.antNum].I;
            sumAnt[j].Q += di[j + k * ctx.antNum].Q;
        }
    }

    int rSftbMf = (/*diB*/AOA_DI_INPUT_BITWTH + /*antN Gain*/ 3) \
                  + (/*antSearch*/(TQA_AMP_BIT-1) +/*antN Gain */3) \
                  -/*outB*/ AOA_DI_ANT_MATCH_OUT_BITWTH;
    //mf corase_search_peak_finding config
    s_mfCfg.match = MF_GEN_MATCH_ULA;
    return (mf_corase_serach_peak_finding(ctx,sumAnt,thetaLut,schLen,/*match len*/ctx.antNum,rSftbMf));
}



void cte_ura_rmtrx_fine_search(cplxInt* di, uint8_t sampPerAnt,aoaEstCtx_t ctx,aoaEstRes_t* res)
{
    int8_t thetaLut[] = AOA_MF_COARSE_SEARCH_TLUT;//AOA_MF_COARSE_SEARCH_TLUT_INTV10;
    uint8_t schLen = sizeof(thetaLut) / sizeof(int8_t);
    uint8_t j,k;
    uxaEstStats_t uraMtx0;
    cplxInt sumAnt[8];
    cplxInt sumAntZ[9];      //3*3 ura, sumAntZ[4]=0;
    cplxInt sumAntX[9];      //3*3 ura, sumAntX[4]=0;

    for (j = 0; j < 8; j++)
    {
        sumAnt[j].I = 0;
        sumAnt[j].Q = 0;

        for (k = 0; k < sampPerAnt; k++)
        {
            sumAnt[j].I += di[j + k * 8].I;
            sumAnt[j].Q += di[j + k * 8].Q;
        }
    }

    //gen x axis ant arrary
    for (j = 0; j < 4; j++)
    {
        sumAntX[j].I = sumAnt[j].I;
        sumAntX[j].Q = sumAnt[j].Q;
    }

    sumAntX[4].I = 0;
    sumAntX[4].Q = 0;

    for (j = 0; j < 4; j++)
    {
        sumAntX[j+5].I = sumAnt[j+4].I;
        sumAntX[j+5].Q = sumAnt[j+4].Q;
    }

    //gen z axis ant arrary
    for(j=0; j<URA8_COL_ROW_NUM; j++)
    {
        for(k=0; k<URA8_COL_ROW_NUM; k++)
        {
            sumAntZ[j*URA8_COL_ROW_NUM+k].I = sumAntX[j+k*URA8_COL_ROW_NUM].I;
            sumAntZ[j*URA8_COL_ROW_NUM+k].Q = sumAntX[j+k*URA8_COL_ROW_NUM].Q;
        }
    }

    int rSftbMf = (/*diB*/AOA_DI_INPUT_BITWTH + /*antN Gain*/ 3) \
                  + (/*antSearch*/(TQA_AMP_BIT-1) +/*antN Gain */3) \
                  -/*outB*/ AOA_DI_ANT_MATCH_OUT_BITWTH;
    //------------------------------------------------------------------------------------------------
    // Corase Search Zeta
    s_mfCfg.theta = 0;
    s_mfCfg.zeta  = 0;
    s_mfCfg.match=MF_GEN_MATCH_URA8_ZAXIS;
    uraMtx0.zEst=mf_corase_serach_peak_finding(ctx,sumAntZ,thetaLut,schLen,/*match len*/ctx.antNumCol,rSftbMf);
    AOA_DBG("[URA zEst] %d \n", uraMtx0.zEst);
    //------------------------------------------------------------------------------------------------
    // Corase Search theta
    s_mfCfg.zeta = uraMtx0.zEst;
    s_mfCfg.match=MF_GEN_MATCH_URA8_XAXIS;
    uraMtx0.aEst=mf_corase_serach_peak_finding(ctx,sumAntX,thetaLut,schLen,/*match len*/ctx.antNumRow,rSftbMf);
    AOA_DBG("[URA aEst] %d \n", uraMtx0.aEst);
    //------------------------------------------------------------------------------------------------
    // Matrix Fine Search
    #if(RMFS_MAX_ZETA_LUT_LEN>0 && RMFS_MAX_THETA_LUT_LEN>0 )
    int8_t zLut[RMFS_MAX_ZETA_LUT_LEN];
    uint8_t zLen=0;
    int8_t tLut[RMFS_MAX_THETA_LUT_LEN];
    uint8_t tLen=0;
    ctx.est.rmfsNN = ctx.est.rmfsNN<RMFS_MAX_ZETA_LUT_LEN ? ctx.est.rmfsNN : RMFS_MAX_ZETA_LUT_LEN;
    ctx.est.rmfsMM = ctx.est.rmfsMM<RMFS_MAX_THETA_LUT_LEN ?ctx.est.rmfsMM : RMFS_MAX_THETA_LUT_LEN;
    AOA_DBG("[ZETA LUT] \n");

    for(int8_t i=-ctx.est.rmfsNN/2; i<=ctx.est.rmfsNN/2; i++)
    {
        if(uraMtx0.zEst+i>-90 && uraMtx0.zEst+i<90)
        {
            zLut[zLen]=uraMtx0.zEst+i;
            AOA_DBG("#%2d %4d, ",zLen,zLut[zLen]);
            zLen++;
        }
    }

    AOA_DBG("\n[THETA LUT] \n");

    for(int8_t i=-ctx.est.rmfsMM/2; i<=ctx.est.rmfsMM/2; i++)
    {
        if(uraMtx0.aEst+i>-90 && uraMtx0.aEst+i<90)
        {
            tLut[tLen]=uraMtx0.aEst+i;
            AOA_DBG("#%2d %4d, ",tLen,tLut[tLen]);
            tLen++;
        }
    }

    AOA_DBG("\n");

    if(ctx.est.rmfsNN>0 || ctx.est.rmfsMM>0)
    {
        s_mfCfg.theta = 0;
        s_mfCfg.zeta = 0;
        s_mfCfg.match = MF_GEN_MATCH_URA8_MTRIX;
        mf_matrix_fine_search(/*in*/ ctx, sumAnt, tLut, tLen, zLut, zLen, ctx.antNum, rSftbMf,
                                     /*out*/ &(uraMtx0));
        AOA_DBG("[URA RMFS] z%d  t%d\n", uraMtx0.zEst, uraMtx0.aEst);
    }

    #endif
    //------------------------------------------------------------------------------------------------
    // return est result
    res->aEst=uraMtx0.aEst;
    res->zEst=uraMtx0.zEst;
}

uint8 aoa_est_uxa(aoaEstCtx_t ctx, aoaEstRes_t* res)
{
    int i;

    for (i = 0; i < CTE_REF_LEN; i++)
    {
        s_uxa.di[i].I = ctx.iSample[i+CTE_GUARD_LEN];
        s_uxa.di[i].Q = ctx.qSample[i+CTE_GUARD_LEN];
    }

    res->fEst0 = cte_ref_freq_est(s_uxa.di, CTE_F0_EST_LEN);
    //res->fEst0 = (1<<(PI_QUNT_BIT-1));
    s_uxa.iqLen = ((ctx.iqCnt - CTE_GUARD_REF_LEN) / ctx.swSlot);
    s_uxa.iqLen = s_uxa.iqLen < CTE_MAX_SLOT_SAMPLE ? s_uxa.iqLen : CTE_MAX_SLOT_SAMPLE;
    s_uxa.sampPerAnt = s_uxa.iqLen/ctx.antNum;
    AOA_DBG("[iqLen%d ANT %d SampPerAnt%d ]\n", s_uxa.iqLen,ctx.antNum,s_uxa.sampPerAnt);
    int ncoPhy = 0 ;//
    uint8 cIdx = 0;
    int tCos,tSin;

    for (i = 0; i < s_uxa.iqLen; i++)
    {
        ncoPhy = phyWrap(ncoPhy-ctx.swSlot*res->fEst0,PI_QUNT_BIT);
        TqaNCO(ncoPhy,&tCos,&tSin,/*tqaComp*/1);
        cIdx   = CTE_GUARD_REF_LEN + (ctx.swSlot>>1) + i * ctx.swSlot;  //guard(4us)+ref (8us)+ 0.5 slot
        s_uxa.di[i].I = ctx.iSample[cIdx]* tCos - ctx.qSample[cIdx]* tSin;
        s_uxa.di[i].Q = ctx.iSample[cIdx]* tSin + ctx.qSample[cIdx]* tCos;
        s_uxa.di[i].I = bROUND(s_uxa.di[i].I,(TQA_AMP_BIT-1));
        s_uxa.di[i].Q = bROUND(s_uxa.di[i].Q,(TQA_AMP_BIT-1));
    }

    #if(_AOA_LOG_LEVEL_ == AOA_LOG_LEVEL_ALL)
    //WriteCplxIntFile("zz_dif0",s_uxa.iqLen, s_uxa.di);
    #endif
    //------------------------------------------------------------------------
    // fine foff est
    res->fEst1 = cte_uxa_fine_freq_est( s_uxa.di,s_uxa.sampPerAnt,ctx.antNum);
    //res->fEst1 = 0;
    AOA_DBG("[FEST %d %f MHz]\n", res->fEst0+res->fEst1/ctx.swSlot,(res->fEst0+res->fEst1*1.0/ctx.swSlot)/(1<<(PI_QUNT_BIT+1)));
    //------------------------------------------------------------------------
    ncoPhy = 0;
    int tmpI,tmpQ;

    for (i = 0; i < s_uxa.iqLen; i++)
    {
        ncoPhy = phyWrap(ncoPhy-res->fEst1,PI_QUNT_BIT);
        TqaNCO(ncoPhy,&tCos,&tSin,/*tqaComp*/1);
        tmpI = s_uxa.di[i].I;
        tmpQ = s_uxa.di[i].Q;
        s_uxa.di[i].I = tmpI* tCos - tmpQ* tSin;
        s_uxa.di[i].Q = tmpI* tSin + tmpQ* tCos;
        s_uxa.di[i].I = bROUND(s_uxa.di[i].I,(TQA_AMP_BIT-1));
        s_uxa.di[i].Q = bROUND(s_uxa.di[i].Q,(TQA_AMP_BIT-1));
    }

    #if(_AOA_LOG_LEVEL_ == AOA_LOG_LEVEL_ALL)
    //WriteCplxIntFile("zz_dif1",s_uxa.iqLen,s_uxa.di);
    #endif
    #if(_AOA_EST_ANT_MODE_ == CTE_AOA_ANT_ULA)
    //------------------------------------------------------------------------
    // angle estimation algorithm MFPF
    res->aEst = cte_ula_mf_peak_finding(s_uxa.di,s_uxa.sampPerAnt,ctx);
    //res->aEst = cte_ula_mf_coarse_fine(s_uxa.di,s_uxa.sampPerAnt,ctx);
#elif(_AOA_EST_ANT_MODE_ == CTE_AOA_ANT_URA)
    //------------------------------------------------------------------------
    // angle estimation algorithm URA RMFS
    cte_ura_rmtrx_fine_search(s_uxa.di,s_uxa.sampPerAnt,ctx,/*out*/res);
    #endif
    return 0;
}
