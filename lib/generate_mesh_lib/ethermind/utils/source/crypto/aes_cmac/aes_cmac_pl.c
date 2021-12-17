
/**
    \file aes_cmac_pl.c


*/

/*
    Copyright (C) 2013. Mindtree Ltd.
    All rights reserved.
*/

/* --------------------------------------------- Header File Inclusion */
#include "aes_cmac_pl.h"

#ifdef AES_CMAC

/* --------------------------------------------- External Global Variables */

/* --------------------------------------------- Exported Global Variables */

/* --------------------------------------------- Static Global Variables */

/* --------------------------------------------- Functions */

void aes_cmac_init_pl (void)
{
}

/**
    \fn aes_cmac_128B_encrypt_pl

    \brief

    \Description


    \param key
    \param data
    \param encout

    \return
*/
EM_RESULT aes_cmac_128B_encrypt_pl (UCHAR* key, UCHAR* data, UCHAR* encout)
{
    INT32 ret;
    cry_aes_128_encrypt_le (data, key, encout, ret);
    return (0 <= ret)? EM_SUCCESS: EM_FAILURE;
}

#ifdef AES_CMAC_PL_AES128_NONBLOCKING
void aes_cmac_128B_encrypt_complete_pl (UCHAR status, UCHAR* data, UINT16 length)
{
    aes_cmac_aes_128_encryption_complete (status, data, length);
}
#endif /* AES_CMAC_PL_AES128_NONBLOCKING */
#endif /* AES_CMAC */

