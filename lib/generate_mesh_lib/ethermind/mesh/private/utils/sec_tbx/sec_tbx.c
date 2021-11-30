
/**
    \file sec_tbx.c


*/

/*
    Copyright (C) 2013. Mindtree Limited.
    All rights reserved.
*/

/* --------------------------------------------- Header File Inclusion */
#include "sec_tbx_internal.h"

/* --------------------------------------------- External Global Variables */

/* --------------------------------------------- Exported Global Variables */

/* --------------------------------------------- Static Global Variables */

/* --------------------------------------------- Functions */
void ms_stbx_s1
(
    /* IN */  UCHAR* m,
    /* IN */  UINT16  mlen,
    /* OUT */ UCHAR* s1
)
{
    UCHAR key[STBX_128B_KEY_SIZE];
    INT32 ret;
    STBX_TRC ("[STBX] M: [LSB - MSB]\n");
    STBX_debug_dump_bytes((UCHAR*)m, mlen);
    EM_mem_set(key, 0x0, STBX_128B_KEY_SIZE);
    cry_aes_128_cmac_sign_be
    (
        m,
        mlen,
        key,
        s1,
        STBX_128B_KEY_SIZE,
        ret
    );
    STBX_TRC ("[STBX] S1: [LSB - MSB]\n");
    STBX_debug_dump_bytes(s1, STBX_128B_KEY_SIZE);
}

void ms_stbx_k1
(
    /* IN */  UCHAR* n,
    /* IN */  UINT16  nlen,
    /* IN */  UCHAR* salt,
    /* IN */  UCHAR* p,
    /* IN */  UINT16  plen,
    /* OUT */ UCHAR* k1
)
{
    UCHAR t[STBX_128B_KEY_SIZE];
    INT32 ret;
    STBX_TRC ("[STBX] N: [LSB - MSB]\n");
    STBX_debug_dump_bytes((UCHAR*)n, nlen);
    STBX_TRC ("[STBX] Salt: [LSB - MSB]\n");
    STBX_debug_dump_bytes((UCHAR*)salt, STBX_128B_KEY_SIZE);
    STBX_TRC ("[STBX] P: [LSB - MSB]\n");
    STBX_debug_dump_bytes((UCHAR*)p, plen);
    cry_aes_128_cmac_sign_be
    (
        n,
        nlen,
        salt,
        t,
        STBX_128B_KEY_SIZE,
        ret
    );
    STBX_TRC ("[STBX] T: [LSB - MSB]\n");
    STBX_debug_dump_bytes(t, STBX_128B_KEY_SIZE);
    cry_aes_128_cmac_sign_be
    (
        p,
        plen,
        t,
        k1,
        STBX_128B_KEY_SIZE,
        ret
    );
    STBX_TRC ("[STBX] k1: [LSB - MSB]\n");
    STBX_debug_dump_bytes(k1, STBX_128B_KEY_SIZE);
}

void ms_stbx_k2
(
    /* IN */  UCHAR* n,
    /* IN */  UINT16  nlen,
    /* IN */  UCHAR* p,
    /* IN */  UINT16  plen,
    /* OUT */ UCHAR* k2
)
{
    UCHAR plain[STBX_TEMP_PLAINTEXT_SIZE];
    UCHAR salt[STBX_128B_KEY_SIZE];
    UCHAR t[STBX_128B_KEY_SIZE];
    UCHAR t1[STBX_128B_KEY_SIZE];
    UCHAR t2[STBX_128B_KEY_SIZE];
    UCHAR t3[STBX_128B_KEY_SIZE];
    UCHAR smk2[] = { 's', 'm', 'k', '2' };
    INT32 ret;

    if (STBX_TEMP_PLAINTEXT_SIZE < (plen + STBX_128B_KEY_SIZE + 1))
    {
        /* Error */
        return;
    }

    STBX_TRC ("[STBX] smk2: [LSB - MSB]\n");
    STBX_debug_dump_bytes(smk2, sizeof(smk2));
    /* Compute the SALT */
    ms_stbx_s1 (smk2, sizeof(smk2), salt);
    STBX_TRC ("[STBX] Salt: [LSB - MSB]\n");
    STBX_debug_dump_bytes((UCHAR*)salt, STBX_128B_KEY_SIZE);
    STBX_TRC ("[STBX] N: [LSB - MSB]\n");
    STBX_debug_dump_bytes((UCHAR*)n, nlen);
    /* Compute the Key T */
    cry_aes_128_cmac_sign_be
    (
        n,
        nlen,
        salt,
        t,
        STBX_128B_KEY_SIZE,
        ret
    );
    STBX_TRC ("[STBX] Computed key T(t): [LSB - MSB]\n");
    STBX_debug_dump_bytes(t, STBX_128B_KEY_SIZE);
    /* Compute T1 */
    EM_mem_copy (plain, p, plen);
    plain[plen] = 0x01;
    STBX_TRC ("[STBX] plain: [LSB - MSB]\n");
    STBX_debug_dump_bytes((UCHAR*)plain, (plen + 1));
    STBX_TRC ("[STBX] T: [LSB - MSB]\n");
    STBX_debug_dump_bytes(t, STBX_128B_KEY_SIZE);
    cry_aes_128_cmac_sign_be
    (
        plain,
        (plen + 1),
        t,
        t1,
        STBX_128B_KEY_SIZE,
        ret
    );
    STBX_TRC ("[STBX] Computed T1(t1): [LSB - MSB]\n");
    STBX_debug_dump_bytes(t1, STBX_128B_KEY_SIZE);
    /* Compute T2 */
    EM_mem_copy(plain, t1, STBX_128B_KEY_SIZE);
    EM_mem_copy((plain + STBX_128B_KEY_SIZE), p, plen);
    plain[STBX_128B_KEY_SIZE + plen] = 0x02;
    STBX_TRC ("[STBX] plain: [LSB - MSB]\n");
    STBX_debug_dump_bytes((UCHAR*)plain, (plen + STBX_128B_KEY_SIZE + 1));
    STBX_TRC ("[STBX] T: [LSB - MSB]\n");
    STBX_debug_dump_bytes(t, STBX_128B_KEY_SIZE);
    cry_aes_128_cmac_sign_be
    (
        plain,
        (plen + STBX_128B_KEY_SIZE + 1),
        t,
        t2,
        STBX_128B_KEY_SIZE,
        ret
    );
    STBX_TRC ("[STBX] Computed T2(t2): [LSB - MSB]\n");
    STBX_debug_dump_bytes(t2, STBX_128B_KEY_SIZE);
    /* Compute T3 */
    EM_mem_copy(plain, t2, STBX_128B_KEY_SIZE);
    EM_mem_copy((plain + STBX_128B_KEY_SIZE), p, plen);
    plain[STBX_128B_KEY_SIZE + plen] = 0x03;
    STBX_TRC ("[STBX] plain: [LSB - MSB]\n");
    STBX_debug_dump_bytes((UCHAR*)plain, (plen + STBX_128B_KEY_SIZE + 1));
    STBX_TRC ("[STBX] T: [LSB - MSB]\n");
    STBX_debug_dump_bytes(t, STBX_128B_KEY_SIZE);
    cry_aes_128_cmac_sign_be
    (
        plain,
        (plen + STBX_128B_KEY_SIZE + 1),
        t,
        t3,
        STBX_128B_KEY_SIZE,
        ret
    );
    STBX_TRC ("[STBX] Computed T3(t3): [LSB - MSB]\n");
    STBX_debug_dump_bytes(t3, STBX_128B_KEY_SIZE);
    /*
        Compute the K2. (T1 || T2 || T3) mod 2^263.
        i.e., Have the least significant 263 bits
    */
    k2[0] = (t1[STBX_128B_KEY_SIZE - 1] & 0x7F);
    EM_mem_copy((k2 + 1), t2, STBX_128B_KEY_SIZE);
    EM_mem_copy((k2 + (1 + STBX_128B_KEY_SIZE)), t3, STBX_128B_KEY_SIZE);
}

void ms_stbx_k3
(
    /* IN */  UCHAR* n,
    /* IN */  UINT16  nlen,
    /* OUT */ UCHAR* k3
)
{
    UCHAR salt[STBX_128B_KEY_SIZE];
    UCHAR t[STBX_128B_KEY_SIZE];
    UCHAR tmpk3[STBX_128B_KEY_SIZE];
    UCHAR smk3[] = { 's', 'm', 'k', '3' };
    UCHAR id64[] = { 'i', 'd', '6', '4', 0x01};
    INT32 ret;
    STBX_TRC ("[STBX] smk3: [LSB - MSB]\n");
    STBX_debug_dump_bytes(smk3, sizeof(smk3));
    /* Compute the SALT */
    ms_stbx_s1(smk3, sizeof(smk3), salt);
    STBX_TRC ("[STBX] Salt: [LSB - MSB]\n");
    STBX_debug_dump_bytes((UCHAR*)salt, STBX_128B_KEY_SIZE);
    STBX_TRC ("[STBX] N: [LSB - MSB]\n");
    STBX_debug_dump_bytes((UCHAR*)n, nlen);
    /* Compute the Key T */
    cry_aes_128_cmac_sign_be
    (
        n,
        nlen,
        salt,
        t,
        STBX_128B_KEY_SIZE,
        ret
    );
    STBX_TRC ("[STBX] Computed key T(t): [LSB - MSB]\n");
    STBX_debug_dump_bytes(t, STBX_128B_KEY_SIZE);
    STBX_TRC ("[STBX] id64: [LSB - MSB]\n");
    STBX_debug_dump_bytes((UCHAR*)id64, sizeof(id64));
    /* Compute the K3 */
    cry_aes_128_cmac_sign_be
    (
        id64,
        sizeof(id64),
        t,
        tmpk3,
        STBX_128B_KEY_SIZE,
        ret
    );
    STBX_TRC ("[STBX] Computed K3(tmpk3): [LSB - MSB]\n");
    STBX_debug_dump_bytes(tmpk3, STBX_128B_KEY_SIZE);
    /*
        Do a K3 mod 2^64
        i.e., Have the least significant 64 bits
    */
    EM_mem_copy(k3, (tmpk3 + (STBX_128B_KEY_SIZE >> 1)), (STBX_128B_KEY_SIZE >> 1));
}

void ms_stbx_k4
(
    /* IN */  UCHAR* n,
    /* IN */  UINT16  nlen,
    /* OUT */ UCHAR* k4
)
{
    UCHAR salt[STBX_128B_KEY_SIZE];
    UCHAR t[STBX_128B_KEY_SIZE];
    UCHAR tmpk4[STBX_128B_KEY_SIZE];
    UCHAR smk4[] = { 's', 'm', 'k', '4' };
    UCHAR id6[] = {'i', 'd', '6', 0x01};
    INT32 ret;
    STBX_TRC ("[STBX] smk4: [LSB - MSB]\n");
    STBX_debug_dump_bytes(smk4, sizeof(smk4));
    /* Compute the SALT */
    ms_stbx_s1(smk4, sizeof(smk4), salt);
    STBX_TRC ("[STBX] Salt: [LSB - MSB]\n");
    STBX_debug_dump_bytes((UCHAR*)salt, STBX_128B_KEY_SIZE);
    STBX_TRC ("[STBX] N: [LSB - MSB]\n");
    STBX_debug_dump_bytes((UCHAR*)n, nlen);
    /* Compute the Key T */
    cry_aes_128_cmac_sign_be
    (
        n,
        nlen,
        salt,
        t,
        STBX_128B_KEY_SIZE,
        ret
    );
    STBX_TRC ("[STBX] Computed key T(t) : [LSB - MSB]\n");
    STBX_debug_dump_bytes(t, STBX_128B_KEY_SIZE);
    STBX_TRC ("[STBX] id5: [LSB - MSB]\n");
    STBX_debug_dump_bytes((UCHAR*)id6, sizeof(id6));
    /* Compute the K4 */
    cry_aes_128_cmac_sign_be
    (
        id6,
        sizeof(id6),
        t,
        tmpk4,
        STBX_128B_KEY_SIZE,
        ret
    );
    STBX_TRC ("[STBX] Computed K4(tmpk4): [LSB - MSB]\n");
    STBX_debug_dump_bytes(tmpk4, STBX_128B_KEY_SIZE);
    /*
        Do a K4 mod 2^6
        i.e., Have the least significant 6 bits
    */
    *k4 = (tmpk4[STBX_128B_KEY_SIZE - 1] & 0x3F);
}

void ms_stbx_va
(
    /* IN */  UCHAR*   l,
    /* IN */  UINT16   llen,
    /* OUT */ UINT16* va
)
{
    UCHAR salt[STBX_128B_KEY_SIZE];
    UCHAR tmph[STBX_128B_KEY_SIZE];
    UCHAR vtad[] = { 'v', 't', 'a', 'd' };
    UINT16 tmpva;
    INT32 ret;
    STBX_TRC ("[STBX] vtad: [LSB - MSB]\n");
    STBX_debug_dump_bytes(vtad, sizeof(vtad));
    /* Compute the SALT */
    ms_stbx_s1(vtad, sizeof(vtad), salt);
    STBX_TRC ("[STBX] Salt: [LSB - MSB]\n");
    STBX_debug_dump_bytes((UCHAR*)salt, STBX_128B_KEY_SIZE);
    STBX_TRC ("[STBX] L: [LSB - MSB]\n");
    STBX_debug_dump_bytes((UCHAR*)l, llen);
    /* Compute the Hash */
    cry_aes_128_cmac_sign_be
    (
        l,
        llen,
        salt,
        tmph,
        STBX_128B_KEY_SIZE,
        ret
    );
    STBX_TRC ("[STBX] Computed Hash H(h) : [LSB - MSB]\n");
    STBX_debug_dump_bytes(tmph, STBX_128B_KEY_SIZE);
    /*
        Do a h mod 2^14
        i.e., Have the least significant 14 bits

        Set bit 15 to 1 and bit 14 to 0.
    */
    tmpva = (((tmph[STBX_128B_KEY_SIZE - 2]) & 0x3F) | (0x80));
    tmpva <<= 8;
    tmpva |= tmph[STBX_128B_KEY_SIZE - 1];
    *va = tmpva;
    STBX_TRC ("[STBX] Computed Virtual Address 0x%04X\n", *va);
}

