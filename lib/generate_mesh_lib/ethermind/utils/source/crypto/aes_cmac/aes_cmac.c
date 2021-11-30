
/**
    \file aes_cmac.c

    Implementation of AES-CMAC algorithm, with reference to RFC-4493.

*/

/*
    Copyright (C) 2013. Mindtree Ltd.
    All rights reserved.
*/

/* --------------------------------------------- Header File Inclusion */
#include "aes_cmac.h"
#include "aes_cmac_pl.h"

#ifdef AES_CMAC

/* --------------------------------------------- External Global Variables */

/* --------------------------------------------- Exported Global Variables */

/* --------------------------------------------- Static Global Variables */

static EM_RESULT aes_cmac_generate_subkeys
(
    AES_CMAC_CONTEXT* context
);

static EM_RESULT aes_cmac_generate_mac
(
    AES_CMAC_CONTEXT* context
);

static void aes_cmac_create_subkeys
(
    AES_CMAC_CONTEXT* context,
    UCHAR* key
);

static EM_RESULT aes_cmac_finalize_mac (AES_CMAC_CONTEXT* context, UCHAR* premac);

static void aes_cmac_handle_process_completion
(
    AES_CMAC_CONTEXT* context,
    UCHAR* data
);

static void aes_cmac_xor_16octet (UCHAR* in1, UCHAR* in2, UCHAR* out);
static void aes_cmac_1bit_leftshift_16octet_stream (UCHAR* inout);

static EM_RESULT aes_cmac_add_context (AES_CMAC_CONTEXT* context);

/* Constant zero data */
DECL_CONST UCHAR cmac_zero_data[AES_CMAC_KEY_SIZE] =
{
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00
};

DECL_CONST UCHAR cmac_rb_data[AES_CMAC_KEY_SIZE] =
{
    0x87, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00
};

#ifdef AES_CMAC_PL_AES128_NONBLOCKING
    /* Global context. TODO: Update to handle multiple */
    DECL_STATIC AES_CMAC_CONTEXT* g_context;
    DECL_STATIC UCHAR aes_cmac_state;
#endif

/* State Handling Macros */
#define AES_CMAC_SET_STATE(x, s)          (x)->state |= (s)
#define AES_CMAC_RESET_STATE(x, s)        (x)->state &= (UCHAR)(~(s))

/* --------------------------------------------- Functions */

/**
    \fn aes_cmac_init

    \brief

    \Description


    \param None

    \return
*/
EM_RESULT aes_cmac_init (void)
{
    /* Initialize module */
    #ifdef AES_CMAC_PL_AES128_NONBLOCKING
    g_context = NULL;
    aes_cmac_state = 0x00;
    #endif
    /* Initialize Platform module */
    aes_cmac_init_pl ();
    return EM_SUCCESS;
}


/**
    \fn aes_cmac_context_init

    \brief

    \Description


    \param context

    \return
*/
EM_RESULT aes_cmac_context_init (AES_CMAC_CONTEXT* context)
{
    /* Initialize context data */
    context->key          = NULL;
    context->data         = NULL;
    context->mac          = NULL;
    context->cb           = NULL;
    context->datalen      = 0;
    context->maclen       = 0;
    context->num_blocks   = 0;
    context->processed    = 0;
    context->action       = 0x00;
    /* Initialize context state */
    context->state        = AES_CMAC_STATE_INIT;
    return EM_SUCCESS;
}


/**
    \fn aes_cmac

    \brief

    \Description


    \param context
    \param action

    \return
*/
EM_RESULT aes_cmac (AES_CMAC_CONTEXT* context)
{
    EM_RESULT retval;
    UCHAR action;
    /* TODO: Validate context */
    /* Add context to table */
    retval = aes_cmac_add_context (context);

    if (EM_SUCCESS != retval)
    {
        /* TODO: Log error and correct error code */
        return EM_FAILURE;
    }

    /* Get the action */
    action = context->action;
    #ifdef AES_CMAC_PL_AES128_NONBLOCKING
    /* Set global state */
    aes_cmac_state = 0x01;
    #endif
    /* Start sub-key generation */
    retval = aes_cmac_generate_subkeys (context);

    /* Is successful or pending */
    if ((EM_SUCCESS == retval) || (AES_CMAC_PROC_PENDING == retval))
    {
        /* Update context states */
        if (AES_CMAC_MAC_GENERATE == action)
        {
            AES_CMAC_SET_STATE(context, AES_CMAC_STATE_MAC_GENERATE);
        }
        else if (AES_CMAC_MAC_VERIFY == action)
        {
            AES_CMAC_SET_STATE(context, AES_CMAC_STATE_MAC_VERIFY);
        }
    }

    /* Is successful? */
    if (EM_SUCCESS != retval)
    {
        /* TODO: Log */
        return retval;
    }

    /* Proceed with subkeys handling */
    retval = aes_cmac_generate_mac
             (
                 context
             );

    /* Is successful */
    if (EM_SUCCESS == retval)
    {
        /* Update context states */
        if (AES_CMAC_MAC_GENERATE == action)
        {
            AES_CMAC_RESET_STATE(context, AES_CMAC_STATE_MAC_GENERATE);
        }
        else if (AES_CMAC_MAC_VERIFY == action)
        {
            AES_CMAC_RESET_STATE(context, AES_CMAC_STATE_MAC_VERIFY);
        }
    }

    return retval;
}


/**
    \fn aes_cmac_generate_subkeys

    \brief

    \Description


    \param context
    \param k1
    \param k2

    \return
*/
static EM_RESULT aes_cmac_generate_subkeys
(
    AES_CMAC_CONTEXT* context
)
{
    UCHAR enc[AES_CMAC_KEY_SIZE];
    EM_RESULT retval;
    /* Run AES-128 encryption on the zero data with input key */
    retval = aes_cmac_128B_encrypt_pl (context->key, (UCHAR*)cmac_zero_data, enc);
    #ifdef AES_CMAC_PL_AES128_NONBLOCKING

    /* Is pending? */
    if (AES_CMAC_PROC_PENDING == retval)
    {
        /* Update state */
        AES_CMAC_SET_STATE
        (context, (AES_CMAC_STATE_SUBKEY_GEN | AES_CMAC_STATE_IN_ENCRYPT));
    }
    else
    #endif /* AES_CMAC_PL_AES128_NONBLOCKING */
        if (EM_SUCCESS == retval)
        {
            /* Create subkeys from AES-128 encrypted output */
            aes_cmac_create_subkeys (context, enc);
        }

    return retval;
}


/**
    \fn aes_cmac_generate_mac

    \brief

    \Description


    \param context

    \return
*/
static EM_RESULT aes_cmac_generate_mac
(
    AES_CMAC_CONTEXT* context
)
{
    UCHAR y[AES_CMAC_KEY_SIZE];
    UCHAR x[AES_CMAC_KEY_SIZE];
    EM_RESULT retval;
    INT16 i;
    UCHAR remainder, offset, limit;
    offset = 0;
    /* Calculate the number of data blocks. Assuming AES_CMAC_BLOCK_SIZE to be ^2 */
    context->num_blocks = context->datalen >> AES_CMAC_BLOCK_EXPONENT;
    remainder = (context->datalen & (AES_CMAC_BLOCK_SIZE - 1));

    /* Include the incomplete last block to the count, if any */
    if (0 != remainder)
    {
        context->num_blocks ++;
    }

    /* Initialize return */
    retval = EM_FAILURE;
    /* Generate the zero data */
    EM_mem_set (x, 0x00, AES_CMAC_KEY_SIZE);
    /* Update to operating state */
    AES_CMAC_SET_STATE(context, AES_CMAC_STATE_OPERATING);

    /*
        Loop through the data blocks but for the last
        Since data is taken from LSB to MSB: Blocks start from LAST
        i.e. Block starts from (context->datalen - AES_CMAC_BLOCK_SIZE)
    */
    if (context->num_blocks > 1)
    {
        /* Calculate the offset and limit for the data blocks */
        offset = (0 == remainder)? (context->num_blocks - 1):
                 (context->num_blocks - 2);
        limit = (0 == remainder)? 1: 0;

        for (i = offset; i >= limit; i--)
        {
            /* XOR data block with AES scalar */
            aes_cmac_xor_16octet
            (
                (context->data + remainder + (i * AES_CMAC_KEY_SIZE)),
                x,
                y
            );
            /* Run AES-128 encryption on the zero data with input key */
            retval = aes_cmac_128B_encrypt_pl (context->key, y, x);
            #ifdef AES_CMAC_PL_AES128_NONBLOCKING

            /* Is pending? */
            if (EM_SUCCESS != retval)
            {
                /* Update number of processed blocks */
                context->processed ++;
                /* Update state */
                AES_CMAC_SET_STATE(context, AES_CMAC_STATE_IN_ENCRYPT);
                break;
            }

            #endif /* AES_CMAC_PL_AES128_NONBLOCKING */
        }
    }

    /* Is last data block to be prepared? */
    if (!(context->state & AES_CMAC_STATE_IN_ENCRYPT))
    {
        /* Call to finalize the MAC calculation */
        retval = aes_cmac_finalize_mac (context, x);
    }

    return retval;
}


/**
    \fn aes_cmac_create_subkeys

    \brief

    \Description


    \param context
    \param key

    \return
*/
static void aes_cmac_create_subkeys
(
    AES_CMAC_CONTEXT* context,
    UCHAR* key
)
{
    UCHAR* k1;
    UCHAR* k2;
    /* Get reference to context subkeys */
    k1 = context->subkey1;
    k2 = context->subkey2;
    /* k1 is key << by 1 bit */
    EM_mem_copy (k1, key, AES_CMAC_KEY_SIZE);
    aes_cmac_1bit_leftshift_16octet_stream (k1);

    /* Is MSB(key) not equals 0? */
    if (key[AES_CMAC_KEY_SIZE - 1] & 0x80)
    {
        /* k1 = k1 xor cmac_rb_data */
        aes_cmac_xor_16octet ((UCHAR*)cmac_rb_data, k1, k1);
    }

    /* k2 is k1 << by 1 bit */
    EM_mem_copy (k2, k1, AES_CMAC_KEY_SIZE);
    aes_cmac_1bit_leftshift_16octet_stream (k2);

    /* Is MSB(key) not equals 0? */
    if (k1[AES_CMAC_KEY_SIZE - 1] & 0x80)
    {
        /* k2 = k2 xor cmac_rb_data */
        aes_cmac_xor_16octet ((UCHAR*)cmac_rb_data, k2, k2);
    }

    return;
}


/**
    \fn aes_cmac_finalize_mac

    \brief

    \Description


    \param context
    \param premac

    \return
*/
static EM_RESULT aes_cmac_finalize_mac (AES_CMAC_CONTEXT* context, UCHAR* premac)
{
    UCHAR data_blk[AES_CMAC_BLOCK_SIZE];
    UCHAR last_blk[AES_CMAC_BLOCK_SIZE];
    EM_RESULT retval;
    UCHAR last_block_complete;
    UCHAR remainder;

    /* Check if any data blocks are present */
    if (0 == context->num_blocks)
    {
        /* No data blocks */
        context->num_blocks = 1;
        remainder = (UCHAR)context->datalen;
        last_block_complete = 0x00;
    }
    else
    {
        /* Is (len) mod (AES_CMAC_BLOCK_SIZE) equals 0? */
        remainder = (context->datalen & (AES_CMAC_BLOCK_SIZE - 1));
        last_block_complete = (!remainder)? 0x01: 0x00;
    }

    /* Form the last message block based on whether it is complete or not */
    if (0x01 == last_block_complete)
    {
        /* Copy the last data block locally */
        EM_mem_copy
        (
            data_blk,
            (context->data),
            AES_CMAC_BLOCK_SIZE
        );
        /* XOR with subkey 1 */
        aes_cmac_xor_16octet
        (
            data_blk,
            context->subkey1,
            data_blk
        );
    }
    else
    {
        /* Pad incomplete last data block with 10^i, i = 128-8*remainder-1 */
        EM_mem_set (data_blk, 0x00, AES_CMAC_BLOCK_SIZE);
        data_blk[AES_CMAC_BLOCK_SIZE - remainder - 1] = 0x80;

        if (NULL != context->data)
        {
            EM_mem_copy
            (
                data_blk + (AES_CMAC_BLOCK_SIZE - remainder),
                (context->data),
                remainder
            );
        }

        /* XOR with subkey 1 */
        aes_cmac_xor_16octet
        (
            data_blk,
            context->subkey2,
            data_blk
        );
    }

    /* XOR data block with AES scalar */
    aes_cmac_xor_16octet
    (
        data_blk,
        premac,
        last_blk
    );
    /* Run AES-128 encryption on the data with input key */
    retval = aes_cmac_128B_encrypt_pl (context->key, last_blk, data_blk);
    #ifdef AES_CMAC_PL_AES128_NONBLOCKING

    /* Is pending? */
    if (AES_CMAC_PROC_PENDING == retval)
    {
        /* Update number of processed blocks */
        context->processed ++;
        /* Update state */
        AES_CMAC_SET_STATE(context, AES_CMAC_STATE_IN_ENCRYPT);
    }
    else
    #endif /* AES_CMAC_PL_AES128_NONBLOCKING */
        if (EM_SUCCESS == retval)
        {
            /* Handle process completion */
            aes_cmac_handle_process_completion (context, data_blk);
        }

    return retval;
}


/**
    \fn aes_cmac_xor_16octet

    \brief

    \Description


    \param in1
    \param in2
    \param out

    \return None
*/
static void aes_cmac_xor_16octet (UCHAR* in1, UCHAR* in2, UCHAR* out)
{
    INT32  index;
    /* Initialize */
    index = 15;

    do
    {
        out[index] = in1[index] ^ in2[index];
    }
    while (--index >= 0);
}


/**
    \fn aes_cmac_1bit_leftshift_16octet_stream

    \brief

    \Description


    \param inout

    \return None
*/
static void aes_cmac_1bit_leftshift_16octet_stream (UCHAR* inout)
{
    INT32 index;
    UCHAR temp;
    /* Initialize */
    index = 15;
    inout[index--] <<= 1;

    do
    {
        temp = (inout[index] & 0x80)? 0x01: 0x00;
        inout[index + 1] |= temp;
        inout[index] <<= 1;
    }
    while (--index >= 0);
}


/**
    \fn aes_cmac_add_context

    \brief

    \Description


    \param context

    \return
*/
static EM_RESULT aes_cmac_add_context (AES_CMAC_CONTEXT* context)
{
    #ifdef AES_CMAC_PL_AES128_NONBLOCKING
    /* Add to single global context. TODO: Update for multiple */
    g_context = context;
    #endif
    return EM_SUCCESS;
}


/**
    \fn aes_cmac_handle_process_completion

    \brief

    \Description


    \param context

    \return
*/
static void aes_cmac_handle_process_completion
(
    AES_CMAC_CONTEXT* context,
    UCHAR* data
)
{
    /* Update context state to status of operation */
    context->state = 0x00;
    /*
        Truncate the generated CMAC.

        As per RFC-4493 on truncation -
        By default, the length of the output of AES-CMAC is 128 bits.
        It is possible to truncate the MAC.The result of the truncation
        should be taken in most significant bits first order.
    */
    data += (AES_CMAC_KEY_SIZE - context->maclen);

    if (AES_CMAC_MAC_GENERATE == context->action)
    {
        /* Copy the generated MAC to the context */
        EM_mem_copy (context->mac, data, context->maclen);
    }
    else if (AES_CMAC_MAC_VERIFY == context->action)
    {
        /* Compare the generated and original MACs */
        if (EM_mem_cmp (data, context->mac, context->maclen))
        {
            /* Copy the generated MAC to the context */
            EM_mem_copy (context->mac, data, context->maclen);
            /* Update context state to status */
            context->state = 0xFF;
        }
    }

    #ifdef AES_CMAC_PL_AES128_NONBLOCKING
    aes_cmac_state = 0x00;
    #endif
}


#ifdef AES_CMAC_PL_AES128_NONBLOCKING
/**
    \fn aes_cmac_aes_128_encryption_complete

    \brief

    \Description


    \param data
    \param datalen

    \return None
*/
void aes_cmac_aes_128_encryption_complete (UCHAR status, UCHAR* data, UINT16 datalen)
{
    AES_CMAC_CONTEXT* context;
    UCHAR y[AES_CMAC_BLOCK_SIZE];
    EM_RESULT retval;
    UCHAR state;
    UCHAR remainder, offset;
    AES_CMAC_CB cb;
    offset = 0;
    EM_debug_trace(EM_MODULE_ID_AES_CMAC, "[AES_CMAC] Enc Output.\n");
    EM_debug_dump_bytes(EM_MODULE_ID_AES_CMAC, data, datalen);
    /* TODO: Not checking for status */
    EM_IGNORE_UNUSED_PARAM(status);

    /* TODO: Maintain proper state and checks */
    if (0x01 != aes_cmac_state)
    {
        return;
    }

    /*
        TODO: Search for correct context, when multiple contexts exists. Now
        take the single global context
    */
    context = g_context;
    /* Reference the context state */
    state = context->state;

    /* Check the state of context */
    if (state & AES_CMAC_STATE_SUBKEY_GEN)
    {
        /* Update state */
        AES_CMAC_RESET_STATE(context, AES_CMAC_STATE_SUBKEY_GEN);

        /* Is encryption was being waited for? */
        if (context->state & AES_CMAC_STATE_IN_ENCRYPT)
        {
            /* Update state */
            AES_CMAC_RESET_STATE(context, AES_CMAC_STATE_IN_ENCRYPT);
        }

        /* Handle subkey generation */
        aes_cmac_create_subkeys (context, data);
        /* Now handle the subkeys created to proceed */
        aes_cmac_generate_mac
        (
            context
        );
    }
    else if ((state & AES_CMAC_STATE_OPERATING) &&
             (state & AES_CMAC_STATE_IN_ENCRYPT))
    {
        /* Update state */
        AES_CMAC_RESET_STATE(context, AES_CMAC_STATE_IN_ENCRYPT);

        if (context->processed == (context->num_blocks - 1))
        {
            /* Call to calculate the final MAC */
            aes_cmac_finalize_mac (context, data);
            return;
        }
        else if (context->processed == context->num_blocks)
        {
            /* Update state */
            AES_CMAC_RESET_STATE(context, AES_CMAC_STATE_OPERATING);
            /* Handle process completion */
            aes_cmac_handle_process_completion (context, data);
            /* Call the context callback */
            cb = context->cb;
            cb ();
            return;
        }

        /*
            Calculate the offset to find the next block of data.
            First block of data starts from (context-datalen - AES_CMAC_BLOCK_SIZE).
            During subsequent calculations, check with the remainder.
        */
        remainder = (context->datalen & (AES_CMAC_BLOCK_SIZE - 1));
        offset = (context->datalen - (context->processed * AES_CMAC_BLOCK_SIZE)) - remainder;

        if (0 != offset)
        {
            offset = context->datalen - ((context->processed + 1) * AES_CMAC_BLOCK_SIZE);
        }

        /* XOR data block with AES scalar */
        aes_cmac_xor_16octet
        (
            (context->data + offset),
            data,
            y
        );
        /* Run AES-128 encryption on the zero data with input key */
        retval = aes_cmac_128B_encrypt_pl (context->key, y, data);

        /* Is pending? */
        if (AES_CMAC_PROC_PENDING == retval)
        {
            /* Update number of processed blocks */
            context->processed ++;
            /* Update state */
            AES_CMAC_SET_STATE(context, AES_CMAC_STATE_IN_ENCRYPT);
        }
    }
}
#endif /* AES_CMAC_PL_AES128_NONBLOCKING */

#endif /* AES_CMAC */
