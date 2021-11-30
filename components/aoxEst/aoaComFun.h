#ifndef AOA_COMFUN_H_
#define AOA_COMFUN_H_
#include "types.h"
#include "log.h"
/*******************************************************************************
    COMM FUNC CONFIG
*/
#define     _PHASE_LUT_USE_                     PHASE_LUT_USE_TQALUT
#define     _CTE_GET_ANGLE_USE_                 CTE_GET_ANGLE_USE_CODIC
#define     _XABS_ALGO_USE_                     XABS_USE_ABSCODIC

/*******************************************************************************
    Debug Log Config
*/
#define     _AOA_LOG_LEVEL_                     AOA_LOG_LEVEL_INFO


/*******************************************************************************
    CONSTANTS
*/


#define     PHASE_LUT_USE_TQALUT                0x01
#define     PHASE_LUT_USE_TQAFIXPOINT           0x02

#define     CTE_GET_ANGLE_USE_ANG_LUT           0x01
#define     CTE_GET_ANGLE_USE_CODIC             0x02

#define     XABS_USE_ABSHWX                     0x01
#define     XABS_USE_ABSCODIC                   0x02

#define     AOA_LOG_LEVEL_ALL                   0x01
#define     AOA_LOG_LEVEL_INFO                  0x02
#define     AOA_LOG_LEVEL_ERR                   0x03




#define     COMFUN_CODIC_AMP_GAIN_SFT           (10)
#define     COMFUN_CODIC_AMP_GAIN               (622)   //aGain= for i=0:14 { aGain*sqrt(1+2^(-2*(i-1))) }; 
//1/aGain = 0.607252936517010
//abs(I+jQ) =  x[i]/aGain = x[i]*622/1024

#define     COMFUN_CORDIC_ITR_MAX               (14)
#define     COMFUN_CODIC_RSFT                   (8) // !!! COMFUN_CODIC_RSFT+di < 31


/*******************************************************************************
    MACROS
*/
#if(_AOA_LOG_LEVEL_ == AOA_LOG_LEVEL_ALL)
    #define     AOA_DBG(...) dbg_printf(__VA_ARGS__)
    #define     AOA_ERR(...) dbg_printf("[ERR]");dbg_printf(__VA_ARGS__)
    #define     AOA_TIM(...) dbg_printf("[TC ]");dbg_printf(__VA_ARGS__)
    #elif(_AOA_LOG_LEVEL_ == AOA_LOG_LEVEL_INFO)
    #define     AOA_DBG(...) //dbg_printf(__VA_ARGS__)
    #define     AOA_ERR(...) dbg_printf("[ERR]");dbg_printf(__VA_ARGS__)
    #define     AOA_TIM(...) dbg_printf("[TC ]");dbg_printf(__VA_ARGS__)
    #define     AOA_LOG(...) dbg_printf("[LOG]");dbg_printf(__VA_ARGS__)
    #elif(_AOA_LOG_LEVEL_ == AOA_LOG_LEVEL_ERR)
    #define     AOA_DBG(...) //dbg_printf(__VA_ARGS__)
    #define     AOA_ERR(...) dbg_printf("[ERR]");dbg_printf(__VA_ARGS__)
    #define     AOA_TIM(...) //dbg_printf("[TC ]");printf(__VA_ARGS__)
#endif

#ifndef SIGN
    #define SIGN(x) (x < 0 ? -1 : 1)
#endif

#ifndef ABS
    #define ABS(n) (((n) < 0) ? -(n) : (n))
#endif

#define CPLX_DOT_CONJ(a, b, c, d, e) cplxDot((a), (b), (c), (d), (e), 1)
#define CPLX_DOT(a, b, c, d, e) cplxDot((a), (b), (c), (d), (e), 0)

#if (_PHASE_LUT_USE_ == PHASE_LUT_USE_TQAFIXPOINT)
    #define PI_QUNT_BIT (14)
    #define TQA_AMP_BIT (12)
    #define FIXPOINT_TQA(a, b, c, d, e, f) TqaFP((a), (b), (c), (d), (e), (f))
#else
    #define PI_QUNT_BIT (13)
    #define TQA_AMP_BIT (13)
    #define FIXPOINT_TQA(a, b, c, d, e, f) TqaLut((a), (b), (c), (d), (e), (f))
#endif

#if (_XABS_ALGO_USE_ == XABS_USE_ABSHWX)
    #define FIXPOINT_XABS(a) absHWX(a)
#else
    #define FIXPOINT_XABS(a) absHWCodic(a)
#endif

#if (_CTE_GET_ANGLE_USE_ == CTE_GET_ANGLE_USE_ANG_LUT)
    #define FIXPOINT_GET_ANGLE(a, b, c, d) get_angle((a), (b), (c), (d))
#else
    #define FIXPOINT_GET_ANGLE(a, b, c, d) atan_Codic((a), (b), COMFUN_CODIC_RSFT, COMFUN_CORDIC_ITR_MAX, (c), (d))
#endif

/*******************************************************************************
    TYPEDEFS
*/
/*******************************************************************************
    LOCAL VARIABLES
*/

/*******************************************************************************
    GLOBAL VARIABLES
*/
typedef struct
{
    double I;
    double Q;
} cplx;

typedef struct
{
    int I;
    int Q;
} cplxInt;

typedef struct
{
    int16 I;
    int16 Q;
} cplxInt16;


////declearion of the function
void  iCopy(int* pDO, int* pDI, int len);
int iCopy_xInt(cplxInt* pDO,cplxInt* pDI, int len);

int  FindMaxDec(int* data,int length);
int  FindMinDec(int* data,int length);

int iSum(int* data,int length);
int bCLIP(int di, int bw);

void cplxMatch(cplxInt* a, cplxInt* b, cplxInt* c, int len,int inc, int rSftb);
void cplxDot(cplxInt* a,cplxInt* b, cplxInt* c,int len,int rSftb,int conjMode);

int bROUND(int di,int rSft);
int phyWrap(int di,int piBit);
int bWIDTH(int di);
#if(_CTE_GET_ANGLE_USE_ == CTE_GET_ANGLE_USE_ANG_LUT )
    void getAngle(int I,int Q,int* phy, int* rShift);
#endif
void atan_Codic (int freq_i, int freq_q, int rSft,int itr,int* phyOut,uint32_t* ampOut);

void TqaNCO(int tqaIn,int* I,int* Q,int tqaComp);
int TqaCOS(int tqaIn);
int TqaSIN(int tqaIn);

uint32_t absHW(int I,int Q);
uint32_t absHWX(cplxInt a);
uint32_t absHWCodic(cplxInt a);

void time_cost_dbg_tic(void);
void time_cost_dbg_toc(char* a);

#endif /* AOA_COMFUN_H_ */

