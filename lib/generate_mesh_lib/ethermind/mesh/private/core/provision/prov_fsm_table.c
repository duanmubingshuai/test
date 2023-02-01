
/**
    \file prov_fsm_table.c

    This file contains PROV FSM - state/event transitions.
*/

/*
    Copyright (C) 2017. Mindtree Ltd.
    All rights reserved.
*/

/* --------------------------------------------- Header File Inclusion */
#include "prov_internal.h"
#include "prov_fsm_handlers.h"
#include "prov_extern.h"


#ifdef PRINT_STATES
/* Get the State Name based upon the Bit Mask */
void print_state_name(UINT32 state_bitmap, UINT16 level)
{
    printf("S[%d] = %x ", level, state_bitmap);

    switch (level)
    {
    case 0: /* State Level 0 */
    {
        if ((state_bitmap & SL_0_PROV_IDLE) == SL_0_PROV_IDLE)
        {
            printf("(SL_0_PROV_IDLE)");
        }

        if ((state_bitmap & SL_0_PROV_W4CAPAB) == SL_0_PROV_W4CAPAB)
        {
            printf("(SL_0_PROV_W4CAPAB)");
        }

        if ((state_bitmap & SL_0_PROV_INCAPAB) == SL_0_PROV_INCAPAB)
        {
            printf("(SL_0_PROV_INCAPAB)");
        }

        if ((state_bitmap & SL_0_PROV_W4START) == SL_0_PROV_W4START)
        {
            printf("(SL_0_PROV_W4START)");
        }

        if ((state_bitmap & SL_0_PROV_INSTART) == SL_0_PROV_INSTART)
        {
            printf("(SL_0_PROV_INSTART)");
        }

        if ((state_bitmap & SL_0_PROV_W4PUBKEY) == SL_0_PROV_W4PUBKEY)
        {
            printf("(SL_0_PROV_W4PUBKEY)");
        }

        if ((state_bitmap & SL_0_PROV_INPUBKEY) == SL_0_PROV_INPUBKEY)
        {
            printf("(SL_0_PROV_INPUBKEY)");
        }

        if ((state_bitmap & SL_0_PROV_W4INPCOM) == SL_0_PROV_W4INPCOM)
        {
            printf("(SL_0_PROV_W4INPCOM)");
        }

        if ((state_bitmap & SL_0_PROV_ININPCOM) == SL_0_PROV_ININPCOM)
        {
            printf("(SL_0_PROV_ININPCOM)");
        }

        if ((state_bitmap & SL_0_PROV_W4CONF) == SL_0_PROV_W4CONF)
        {
            printf("(SL_0_PROV_W4CONF)");
        }

        if ((state_bitmap & SL_0_PROV_INCONF) == SL_0_PROV_INCONF)
        {
            printf("(SL_0_PROV_INCONF)");
        }

        if ((state_bitmap & SL_0_PROV_W4RAND) == SL_0_PROV_W4RAND)
        {
            printf("(SL_0_PROV_W4RAND)");
        }

        if ((state_bitmap & SL_0_PROV_INRAND) == SL_0_PROV_INRAND)
        {
            printf("(SL_0_PROV_INRAND)");
        }

        if ((state_bitmap & SL_0_PROV_W4DATA) == SL_0_PROV_W4DATA)
        {
            printf("(SL_0_PROV_W4DATA)");
        }

        if ((state_bitmap & SL_0_PROV_INDATA) == SL_0_PROV_INDATA)
        {
            printf("(SL_0_PROV_INDATA)");
        }

        if ((state_bitmap & SL_0_PROV_W4COMPLETE) == SL_0_PROV_W4COMPLETE)
        {
            printf("(SL_0_PROV_W4COMPLETE)");
        }

        if ((state_bitmap & SL_0_PROV_INCOMPLETE) == SL_0_PROV_INCOMPLETE)
        {
            printf("(SL_0_PROV_INCOMPLETE)");
        }

        if ((state_bitmap & SL_0_PROV_ERROR) == SL_0_PROV_ERROR)
        {
            printf("(SL_0_PROV_ERROR)");
        }
    }
    break;
    }

    printf("\n");
    return;
}
#endif /* PRINT_STATES */

DECL_CONST DECL_CONST_QUALIFIER STATE_EVENT_TABLE prov_state_event_table[] =
{
    {
        /*0*/ ev_prov_invite,
        1,
        0
    },
    {
        /*1*/ ev_prov_capabilities,
        3,
        2
    },
    {
        /*2*/ ev_prov_start,
        5,
        4
    },
    {
        /*3*/ ev_prov_pubkey,
        7,
        6
    },
    {
        /*4*/ ev_prov_inputcom,
        9,
        8
    },
    {
        /*5*/ ev_prov_confirmation,
        11,
        10
    },
    {
        /*6*/ ev_prov_random,
        13,
        12
    },
    {
        /*7*/ ev_prov_data,
        15,
        14
    },
    {
        /*8*/ ev_prov_complete,
        17,
        16
    }
};

DECL_CONST DECL_CONST_QUALIFIER EVENT_HANDLER_TABLE prov_event_handler_table[] =
{
    {
        /*0*/ SL_0_PROV_IDLE,
        (SE_HNDLR_T)se_prov_invite_handler
    },
    {
        /*1*/ SL_0_PROV_W4CAPAB | SL_0_PROV_INCAPAB | SL_0_PROV_W4START | SL_0_PROV_INSTART | SL_0_PROV_W4PUBKEY | SL_0_PROV_INPUBKEY | SL_0_PROV_W4INPCOM | SL_0_PROV_ININPCOM | SL_0_PROV_W4CONF | SL_0_PROV_INCONF | SL_0_PROV_W4RAND | SL_0_PROV_INRAND | SL_0_PROV_W4DATA | SL_0_PROV_INDATA | SL_0_PROV_W4COMPLETE | SL_0_PROV_INCOMPLETE | SL_0_PROV_ERROR,
        (SE_HNDLR_T)se_prov_error_handler
    },
    {
        /*2*/ SL_0_PROV_W4CAPAB | SL_0_PROV_INCAPAB,
        (SE_HNDLR_T)se_prov_capabilities_handler
    },
    {
        /*3*/ SL_0_PROV_IDLE | SL_0_PROV_W4START | SL_0_PROV_INSTART | SL_0_PROV_W4PUBKEY | SL_0_PROV_INPUBKEY | SL_0_PROV_W4INPCOM | SL_0_PROV_ININPCOM | SL_0_PROV_W4CONF | SL_0_PROV_INCONF | SL_0_PROV_W4RAND | SL_0_PROV_INRAND | SL_0_PROV_W4DATA | SL_0_PROV_INDATA | SL_0_PROV_W4COMPLETE | SL_0_PROV_INCOMPLETE | SL_0_PROV_ERROR,
        (SE_HNDLR_T)se_prov_error_handler
    },
    {
        /*4*/ SL_0_PROV_W4START | SL_0_PROV_INSTART,
        (SE_HNDLR_T)se_prov_start_handler
    },
    {
        /*5*/ SL_0_PROV_IDLE | SL_0_PROV_W4CAPAB | SL_0_PROV_INCAPAB | SL_0_PROV_W4PUBKEY | SL_0_PROV_INPUBKEY | SL_0_PROV_W4INPCOM | SL_0_PROV_ININPCOM | SL_0_PROV_W4CONF | SL_0_PROV_INCONF | SL_0_PROV_W4RAND | SL_0_PROV_INRAND | SL_0_PROV_W4DATA | SL_0_PROV_INDATA | SL_0_PROV_W4COMPLETE | SL_0_PROV_INCOMPLETE | SL_0_PROV_ERROR,
        (SE_HNDLR_T)se_prov_error_handler
    },
    {
        /*6*/ SL_0_PROV_W4PUBKEY | SL_0_PROV_INPUBKEY,
        (SE_HNDLR_T)se_prov_pubkey_handler
    },
    {
        /*7*/ SL_0_PROV_IDLE | SL_0_PROV_W4CAPAB | SL_0_PROV_INCAPAB | SL_0_PROV_W4START | SL_0_PROV_INSTART | SL_0_PROV_W4INPCOM | SL_0_PROV_ININPCOM | SL_0_PROV_W4CONF | SL_0_PROV_INCONF | SL_0_PROV_W4RAND | SL_0_PROV_INRAND | SL_0_PROV_W4DATA | SL_0_PROV_INDATA | SL_0_PROV_W4COMPLETE | SL_0_PROV_INCOMPLETE | SL_0_PROV_ERROR,
        (SE_HNDLR_T)se_prov_error_handler
    },
    {
        /*8*/ SL_0_PROV_W4INPCOM | SL_0_PROV_ININPCOM,
        (SE_HNDLR_T)se_prov_inputcom_handler
    },
    {
        /*9*/ SL_0_PROV_IDLE | SL_0_PROV_W4CAPAB | SL_0_PROV_INCAPAB | SL_0_PROV_W4START | SL_0_PROV_INSTART | SL_0_PROV_W4CONF | SL_0_PROV_INCONF | SL_0_PROV_W4RAND | SL_0_PROV_INRAND | SL_0_PROV_W4DATA | SL_0_PROV_INDATA | SL_0_PROV_W4COMPLETE | SL_0_PROV_INCOMPLETE | SL_0_PROV_ERROR,
        (SE_HNDLR_T)se_prov_error_handler
    },
    {
        /*10*/ SL_0_PROV_W4CONF | SL_0_PROV_INCONF,
        (SE_HNDLR_T)se_prov_confirmation_handler
    },
    {
        /*11*/ SL_0_PROV_IDLE | SL_0_PROV_W4CAPAB | SL_0_PROV_INCAPAB | SL_0_PROV_W4START | SL_0_PROV_INSTART | SL_0_PROV_W4PUBKEY | SL_0_PROV_INPUBKEY | SL_0_PROV_W4INPCOM | SL_0_PROV_ININPCOM | SL_0_PROV_W4RAND | SL_0_PROV_INRAND | SL_0_PROV_W4DATA | SL_0_PROV_INDATA | SL_0_PROV_W4COMPLETE | SL_0_PROV_INCOMPLETE | SL_0_PROV_ERROR,
        (SE_HNDLR_T)se_prov_error_handler
    },
    {
        /*12*/ SL_0_PROV_W4RAND | SL_0_PROV_INRAND,
        (SE_HNDLR_T)se_prov_random_handler
    },
    {
        /*13*/ SL_0_PROV_IDLE | SL_0_PROV_W4CAPAB | SL_0_PROV_INCAPAB | SL_0_PROV_W4START | SL_0_PROV_INSTART | SL_0_PROV_W4PUBKEY | SL_0_PROV_INPUBKEY | SL_0_PROV_W4INPCOM | SL_0_PROV_ININPCOM | SL_0_PROV_W4CONF | SL_0_PROV_INCONF | SL_0_PROV_W4DATA | SL_0_PROV_INDATA | SL_0_PROV_W4COMPLETE | SL_0_PROV_INCOMPLETE | SL_0_PROV_ERROR,
        (SE_HNDLR_T)se_prov_error_handler
    },
    {
        /*14*/ SL_0_PROV_W4DATA | SL_0_PROV_INDATA,
        (SE_HNDLR_T)se_prov_data_handler
    },
    {
        /*15*/ SL_0_PROV_IDLE | SL_0_PROV_W4CAPAB | SL_0_PROV_INCAPAB | SL_0_PROV_W4START | SL_0_PROV_INSTART | SL_0_PROV_W4PUBKEY | SL_0_PROV_INPUBKEY | SL_0_PROV_W4INPCOM | SL_0_PROV_ININPCOM | SL_0_PROV_W4CONF | SL_0_PROV_INCONF | SL_0_PROV_W4RAND | SL_0_PROV_INRAND | SL_0_PROV_W4COMPLETE | SL_0_PROV_INCOMPLETE | SL_0_PROV_ERROR,
        (SE_HNDLR_T)se_prov_error_handler
    },
    {
        /*16*/ SL_0_PROV_W4COMPLETE | SL_0_PROV_INCOMPLETE,
        (SE_HNDLR_T)se_prov_complete_handler
    },
    {
        /*17*/ SL_0_PROV_IDLE | SL_0_PROV_W4CAPAB | SL_0_PROV_INCAPAB | SL_0_PROV_W4START | SL_0_PROV_INSTART | SL_0_PROV_W4PUBKEY | SL_0_PROV_INPUBKEY | SL_0_PROV_W4INPCOM | SL_0_PROV_ININPCOM | SL_0_PROV_W4CONF | SL_0_PROV_INCONF | SL_0_PROV_W4RAND | SL_0_PROV_INRAND | SL_0_PROV_W4DATA | SL_0_PROV_INDATA | SL_0_PROV_ERROR,
        (SE_HNDLR_T)se_prov_error_handler
    }
};

