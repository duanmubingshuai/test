
#ifndef __PHY_SECURE_H
#define __PHY_SECURE_H

#include "types.h"

enum{
  PHY_SEC_KEY_NULL = 0,
  PHY_SEC_KEY_CCM = 1,
  PHY_SEC_KEY_UNINIT = 0xfe,
  PHY_SEC_KEY_FAIL = 0xff,
};

uint8_t phy_sec_key_valid(void);
uint8_t* phy_sec_app_key(void);
uint8_t* phy_sec_app_iv(void);
int phy_sec_encrypt(const uint8_t* key, const uint8_t * iv,
                uint8_t * din, uint32_t len, uint8_t * mic_out, uint8_t *dout);
int phy_sec_decrypt(const uint8_t* key, const uint8_t * iv,
                uint8_t * din, uint32_t len, uint8_t * mic_in, uint8_t *dout);
int phy_sec_efuse_lock(void);
int phy_sec_init(void);
void phy_sec_key_destory(void);

#endif

