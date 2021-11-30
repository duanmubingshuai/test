
/**
    \file MS_common.c

    This File contains Common Routines such as System Start-up Init Call.
*/

/*
    Copyright (C) 2016. Mindtree Ltd.
    All rights reserved.
*/

/* ------------------------------------------- Header File Inclusion */
#include "MS_common.h"
#include "brr.h"
#include "net.h"
#include "ltrn.h"
#include "trn.h"
#include "access.h"
#include "prov.h"
#include "timer.h"     //HZF

static inline uint32 clock_time_rtc(void)
{
    return (*(volatile unsigned int*)0x4000f028) & 0xffffff;
}


#include "fsm_engine.h"

#ifdef MS_SUPPORT_STACK_VERSION_INFO
    #include "MS_version.h"
#endif /* MS_SUPPORT_STACK_VERSION_INFO */

/* ------------------------------------------- Global Definitions */


/* ------------------------------------------- Static Global Variables */
/* Flag - whether Stack is Initialized */
DECL_STATIC UCHAR ms_stack_init_state = MS_STACK_INIT_UNDEFINED;

#ifdef MS_HAVE_DYNAMIC_CONFIG
    /* Global Configuration for Mesh Stack */
    MS_CONFIG ms_global_config;
#endif /* MS_HAVE_DYNAMIC_CONFIG */

/* ------------------------------------------- Functions */
/**
    \brief To initialize Mesh Stack.

    \par Description
    API to initialize Mesh Stack. This is the first API that the
    application should call before any other API. This function
    initializes all the internal stack modules and data structures.

    \param [in] blob
           If 'MS_HAVE_DYNAMIC_CONFIG' defined,
               application shall provide the desired dynamic configuration
               using a pointer to MS_CONFIG data structure instance.
           else,
               this parameter shall be NULL and ignored by the API.

    \return API_SUCCESS or an error code indicating reason for failure
*/
void MS_init
(
    /* IN */ void* blob
)
{
    /* Is EtherMind-Init done already? */
    if (MS_STACK_INIT_UNDEFINED != ms_stack_init_state)
    {
        return;
    }

    /* Check dynamic configuration */
    #ifdef MS_HAVE_DYNAMIC_CONFIG

    if (NULL == blob)
    {
        MS_COMMON_ERR(
            "Dynamic Configuration can not be NULL. Returning\n");
        return;
    }
    else
    {
        /* Save Dynamic Configuration */
        ms_global_config = *(MS_CONFIG*)blob;
    }

    #endif /* MS_HAVE_DYNAMIC_CONFIG */
    /* Initialize the platform */
    ms_init_pl();
    /* Debug Library Initialization */
    /* BT_Init_Debug_Library(); */
    /* Bearer Initialization */
    ms_brr_init();
    /* Network Initialization */
    ms_net_init();
    /* Lower Transport Initialization */
    ms_ltrn_init();
    /* Upper Transport Initialization */
    ms_trn_init();
    /* Access Initialization */
    ms_access_init();
    /* FSM Initialization */
    ms_fsm_init();
    /* Provisioning layer initialization */
    ms_prov_init();
    /* Transition Timer initialization */
    ms_common_init_transition_timer();
    UINT32  T1, T2;
    T1 = clock_time_rtc();//read_current_fine_time();
    // HZF, init ecdh when mesh profile init
    /* Initialize the ECDH for a new pair of public/private keys */
    cry_ecdh_init();
    T2 = clock_time_rtc();//read_current_fine_time();
    printf("consume time of function cry_ecdh_init: %d RTC tick\r\n", (T2 - T1));//(T2 > T1) ? (T2 - T1) : (BASE_TIME_UNITS - T1 + T2));
    /* EtherMind-Init done */
    ms_stack_init_state = MS_STACK_INIT_ETHERMIND_INIT;
    #ifdef MS_SUPPORT_STACK_VERSION_INFO
    MS_COMMON_TRC (
        "EtherMind Mesh Stack Initialization Complete. Stack Version: %03d.%03d.%03d.\n",
        MS_MAJOR_VERSION_NUMBER, MS_MINOR_VERSION_NUMBER, MS_SUB_MINOR_VERSION_NUMBER);
    #endif /* MS_SUPPORT_STACK_VERSION_INFO */
    return;
}


#ifndef MS_NO_SHUTDOWN

/*
    Function Name:
     MS_shutdown

    Description:
      This function shuts down all the initialized modules in the
    reverse order of their initialization.

    Input Parameters:
      None.

    Return value:
      API_RESULT
*/
API_RESULT MS_shutdown
(
    void
)
{
    /* Shutdown MUST happen in the reverse order of Initialization */
    /* Provisioning layer shutdown */
    ms_prov_shutdown();
    /* Access - Shutdown */
    ms_access_shutdown();
    /* Upper Transport - Shutdown */
    ms_trn_shutdown();
    /* Lower Transport - Shutdown */
    ms_ltrn_shutdown();
    /* Network - Shutdown */
    ms_net_shutdown();
    /* Bearer - Shutdown */
    ms_brr_shutdown();
    /* Set Stack Init State as Undefined */
    ms_stack_init_state = MS_STACK_INIT_UNDEFINED;
    return API_SUCCESS;
}

#endif /* MS_NO_SHUTDOWN */


#ifdef MS_SUPPORT_STACK_VERSION_INFO
/*
    Function Name:
      MS_get_version_number

    Description:
      Routine to get  the version number of the stack as return value.
      The version number consists of 3 fields:
           Major Number
           Minor Number
           Sub-Minor Number

    Input Parameters:
      None.

    Output Parameters:
      version_number containing version number of the stack.

    Return value:
      None.
*/
void MS_get_version_number
(
    /* OUT */ MS_VERSION_NUMBER*   version_number
)
{
    version_number->major = MS_MAJOR_VERSION_NUMBER;
    version_number->minor = MS_MINOR_VERSION_NUMBER;
    version_number->subminor = MS_SUB_MINOR_VERSION_NUMBER;
}



API_RESULT ms_internal_verificaiton_check(void)
{
    uint32_t efuseReg = (*(volatile unsigned int*)0x4000f058)&0x3f;
    uint32_t flashReg = (*(volatile unsigned int*)0x11000a00);

    if(efuseReg==0x00 && flashReg==0xffffffff)
    {
        return API_SUCCESS;
    }
    else
    {
        printf("=================================\n");
        printf("====  [DO NOT SUPPORT MESH]  ====\n");
        printf("=================================\n");

        while(1) {};
    }
}
#endif /* MS_SUPPORT_STACK_VERSION_INFO */

