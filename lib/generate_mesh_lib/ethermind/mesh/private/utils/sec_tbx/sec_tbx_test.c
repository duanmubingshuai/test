
/**
    \file sec_tbx_test.c


*/

/*
    Copyright (C) 2013. Mindtree Limited.
    All rights reserved.
*/

/* --------------------------------------------- Header File Inclusion */
#include "sec_tbx.h"

/* --------------------------------------------- External Global Variables */

/* --------------------------------------------- Exported Global Variables */

/* --------------------------------------------- Static Global Variables */
DECL_STATIC CHAR tbx_menu[] = " \n\
================ T O O L B O X   M E N U ================ \n\
   0.  Exit. \n\
   1.  Refresh this Menu. \n\
 \n\
   2.  Test S1. \n\
   3.  Test K1. \n\
   4.  Test K2. \n\
   5.  Test K3. \n\
   6.  Test K4. \n\
 \n\
Your Option ? \0";

/* --------------------------------------------- Functions */
void sec_tbx_test_s1 (void)
{
    UCHAR s1[STBX_128B_KEY_SIZE];
    UCHAR m[] = {'t', 's', 'e', 't'};
    ms_stbx_s1(m, sizeof(m), s1);
}

void sec_tbx_test_k1 (void)
{
    UCHAR n[16] = { 0x98, 0x7f, 0x87, 0x2b, 0x79, 0x41, 0x85, 0x24,
                    0x33, 0xb5, 0x84, 0x98, 0x50, 0xd1, 0x16, 0x32
                  };
    UCHAR p[16] = { 0x0d, 0x2a, 0x35, 0xb3, 0x9d, 0xa5, 0xad, 0x8a,
                    0x47, 0xb4, 0xee, 0x97, 0x07, 0xd6, 0x09, 0x5a
                  };
    UCHAR salt[STBX_128B_KEY_SIZE] = { 0xb4, 0xca, 0x76, 0xd2, 0x57, 0x8d, 0x93, 0x31,
                                       0x28, 0x4a, 0xf8, 0x0d, 0xfa, 0x4f, 0xa1, 0x2b
                                     };
    UCHAR k1[STBX_128B_KEY_SIZE];
    ms_stbx_k1(n, sizeof(n), salt, p, sizeof(p), k1);
}

void sec_tbx_test_k2 (void)
{
    #if 0
    UCHAR n[16] = { 0x00, 0x2b, 0x1e, 0xdc, 0x3d, 0x17, 0x4f, 0x06,
                    0x29, 0x80, 0x8a, 0x8e, 0x4f, 0xa4, 0xa2, 0xf7
                  };
    #else /* 0 */
    UCHAR n[16] = { 0xf7, 0xa2, 0xa4, 0x4f, 0x8e, 0x8a, 0x80, 0x29,
                    0x06, 0x4f, 0x17, 0x3d, 0xdc, 0x1e, 0x2b, 0x00
                  };
    #endif /* 0 */
    UCHAR p[1] = { 0x00 };
    UCHAR k2[33];
    ms_stbx_k2(n, sizeof(n), p, sizeof(p), k2);
}

void sec_tbx_test_k3 (void)
{
    #if 0
    UCHAR n[16] = { 0x00, 0x2b, 0x1e, 0xdc, 0x3d, 0x17, 0x4f, 0x06,
                    0x29, 0x80, 0x8a, 0x8e, 0x4f, 0xa4, 0xa2, 0xf7
                  };
    #else /* 0 */
    UCHAR n[16] = { 0xf7, 0xa2, 0xa4, 0x4f, 0x8e, 0x8a, 0x80, 0x29,
                    0x06, 0x4f, 0x17, 0x3d, 0xdc, 0x1e, 0x2b, 0x00
                  };
    #endif /* 0 */
    UCHAR k3[8];
    ms_stbx_k3(n, sizeof(n), k3);
}

void sec_tbx_test_k4 (void)
{
    #if 0
    UCHAR n[16] = { 0x98, 0x7f, 0x87, 0x2b, 0x79, 0x41, 0x85, 0x24,
                    0x33, 0xb5, 0x84, 0x98, 0x50, 0xd1, 0x16, 0x32
                  };
    #else
    UCHAR n[16] = { 0x32, 0x16, 0xd1, 0x50, 0x98, 0x84, 0xb5, 0x33,
                    0x24, 0x85, 0x41, 0x79, 0x2b, 0x87, 0x7f, 0x98
                  };
    #endif
    UCHAR k4;
    ms_stbx_k4(n, sizeof(n), &k4);
}

void sec_tbx_test_va (void)
{
    #if 0
    UCHAR l[] = {0x00, 0x73, 0xE7, 0xE4, 0xD8, 0xB9, 0x44, 0x0F,
                 0xAF, 0x84, 0x15, 0xDF, 0x4C, 0x56, 0xC0, 0xE1
                };
    #else
    UCHAR l[] = {0xF4, 0xA0, 0x02, 0xC7, 0xFB, 0x1E, 0x4C, 0xA0,
                 0xA4, 0x69, 0xA0, 0x21, 0xDE, 0x0D, 0xB8, 0x75
                };
    #endif /* 0 */
    UINT16 va;
    ms_stbx_va(l, sizeof(l), &va);
}

void main_sec_tbx_test_operations(void)
{
    int choice;

    while (1)
    {
        printf ("%s", tbx_menu);
        scanf ("%d", &choice);

        switch (choice)
        {
        case 0:
            return;

        case 1:
            break;

        case 2:
            sec_tbx_test_s1();
            break;

        case 3:
            sec_tbx_test_k1();
            break;

        case 4:
            sec_tbx_test_k2();
            break;

        case 5:
            sec_tbx_test_k3();
            break;

        case 6:
            sec_tbx_test_k4();
            break;

        case 7:
            sec_tbx_test_va();
            break;

        default:
            break;
        }
    }
}

