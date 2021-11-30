#ifndef AoaEstAlgo_H
#define AoaEstAlgo_H

/*******************************************************************************
    Filename:       aoaEstAlgo.h
    Revised:
    Revision:

    Description:    This file contains the BLE CTE AOA Estimation Algorithms


*******************************************************************************/

/*******************************************************************************
    INCLUDES
*/
#include    "types.h"
#include    "aoaComFun.h"



/*******************************************************************************
    MACROS
*/

/*******************************************************************************
    CONSTANTS
*/
#define CTE_AOA_ANT_ULA             0x01
#define CTE_AOA_ANT_URA             0x02

#define AOA_DI_ANT_MATCH_OUT_BITWTH (20)
#define AOA_DI_INPUT_BITWTH         (10)

/*******************************************************************************
    TYPEDEFS
*/

typedef struct aoaEstPara
{
    uint8_t rmfsNN;
    uint8_t rmfsMM;
} aoaEstPara_t;

typedef struct aoaEstCtx
{
    uint8_t       antMode;
    uint8_t       antNum;
    uint8_t       antNumCol;
    uint8_t       antNumRow;
    uint16_t      antDist;
    uint16_t      lambda;
    uint8_t       swSlot;
    int16_t*      iSample;
    int16_t*      qSample;
    uint8_t       iqCnt;
    aoaEstPara_t  est;
} aoaEstCtx_t;

typedef struct aoaEstRes
{
    int fEst0;
    int fEst1;
    int8_t  aEst;
    int8_t  zEst;
} aoaEstRes_t;


/*******************************************************************************
    LOCAL VARIABLES
*/

/*******************************************************************************
    GLOBAL VARIABLES
*/


/**************************************************************************************
    @fn          aoa_est_uxa

    @brief       This function process for rf phy baseband tx and rx config.

    input parameters

    @param       ctx,.

    output parameters

    @param       res.

    @return      None.
*/
uint8_t aoa_est_uxa(aoaEstCtx_t ctx,aoaEstRes_t* res);

#endif // AoaEstAlgo_H
