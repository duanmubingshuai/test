
#include <string.h>
#include "types.h"
#include "phy_secure.h"
#include "ll_enc.h"
#include "phy_aes.h"
#include "spif.h"
#include "otp.h"
#include "gpio.h"
#include "error.h"


uint8_t     g_phy_sec_key_valid= PHY_SEC_KEY_NULL;
uint32_t    g_phy_sec_key[4]= {0,0,0,0};
uint8_t*    p_phy_sec_iv = NULL;
//uint8_t     g_efuseFlg = 0;
uint8_t 	g_bootFlag = 0;

extern void LL_ENC_AES128_Encrypt0( uint8 *key,uint8 *plaintext,uint8 *ciphertext );

uint8_t phy_sec_key_valid(void)
{
  return g_phy_sec_key_valid;
}
uint8_t* phy_sec_app_key(void) //g_phy_sec_key<--cyr_out
{
  return (uint8_t*)g_phy_sec_key;
}
uint8_t* phy_sec_app_iv(void)
{
  return p_phy_sec_iv;
}
void phy_sec_key_destory(void)
{
  g_phy_sec_key[0]=0xff;
  g_phy_sec_key[1]=0xff;
  g_phy_sec_key[2]=0xff;
  g_phy_sec_key[3]=0xff;

}
int phy_sec_encrypt(const uint8_t* key, const uint8_t * iv,
                        uint8_t * din, uint32_t len, uint8_t *dout,  uint8_t * mic_out)
{
    return pplus_aesccm_enc(key, iv, din, len, dout, mic_out);
}

int phy_sec_decrypt(const uint8_t* key, const uint8_t * iv,
                        uint8_t * din, uint32_t len, uint8_t * mic_in, uint8_t *dout)
{
    return pplus_aesccm_dec(key, iv, din, len, mic_in, dout);
}
int phy_sec_otp_lock(void)
{
    //otp lock method to do
	return PPlus_SUCCESS;
}

int phy_sec_init(void)
{
	uint8_t cnt = 0;

	int ret,i;
	uint32_t temp_sec_key[4];
    uint32_t cyr_out[4];    //phase1: validate phy_sec_mic, phase2: generate g_phy_sec_key
	uint32_t temp_plaintext[4]={0x11223344,0x12345678,0x13151719,0x22242628};
//	uint32_t OTP_SECURE_USE_SELECT,OTP_SECURE_KEY_L,OTP_SECURE_KEY_H;
//	uint8_t pOTP_SECURE_MIC[16];
//	uint8_t* pOTP_SECURE_IV = (OTP_BASE_ADDR + OTP_SECURE_AREA_ADDR + 0x20);
	
	otp_read_data((OTP_BASE_ADDR + OTP_SECURE_AREA_ADDR + 0x4), (uint32_t*)(&OTP_SECURE_USE_SELECT), 1, OTP_USER_READ_MODE);
	if((OTP_SECURE_USE_SELECT != 0x0) &&(cnt<=16)){		
		if((OTP_SECURE_USE_SELECT & (0x3<<(2*cnt)))== 0x0){
			while((OTP_SECURE_USE_SELECT & (0x3<<(2*cnt)))== 0){
				cnt++;
				if((OTP_SECURE_USE_SELECT & (0x3<<(2*cnt))) == (0x3<<(2*cnt)))
				{
					g_bootFlag = 1;
					break;
				}
				else if(((OTP_SECURE_USE_SELECT & (0x1<<(2*cnt))) == (0x1<<(2*cnt)))||((OTP_SECURE_USE_SELECT & (0x2<<(2*cnt))) == (0x2<<(2*cnt)))){
					g_bootFlag = 0;
					gpio_fmux_set(P35,FMUX_MISO_0); //J11(6)(29)SI
					gpio_fmux_set(P36,FMUX_CLK);	//J11(5)(30)SCLK
					gpio_fmux_set(P37,FMUX_CSN);	//J11(16)(19)CS
					gpio_fmux_set(P38,FMUX_MISO_1); //J11(12)(23)SO			
					init_spif();
					break;
				}
			}			
		}
		else if((OTP_SECURE_USE_SELECT & (0x3<<(2*cnt))) == (0x3<<(2*cnt)))
		{
			g_bootFlag = 1;			
		}
		else{
			g_bootFlag = 0;
			gpio_fmux_set(P35,FMUX_MISO_0); //J11(6)(29)SI
			gpio_fmux_set(P36,FMUX_CLK);	//J11(5)(30)SCLK
			gpio_fmux_set(P37,FMUX_CSN);	//J11(16)(19)CS
			gpio_fmux_set(P38,FMUX_MISO_1); //J11(12)(23)SO
			
			init_spif();			
		}
	}
	else{
		g_bootFlag = 1;
	}
    
	if(g_bootFlag){
		//otp default plaintext value
//		otp_read_data((OTP_BASE_ADDR + OTP_SECURE_AREA_ADDR + 0x8), (uint32_t*)(&OTP_SECURE_KEY_L), 1, OTP_USER_READ_MODE);
//		otp_read_data((OTP_BASE_ADDR + OTP_SECURE_AREA_ADDR + 0xc), (uint32_t*)(&OTP_SECURE_KEY_H), 1, OTP_USER_READ_MODE);
		if(OTP_SECURE_KEY_L == 0xffffffff && OTP_SECURE_KEY_H == 0xffffffff){
			return PPlus_SUCCESS;
		}
		temp_sec_key[0] = OTP_SECURE_KEY_L;
    	temp_sec_key[1] = OTP_SECURE_KEY_H;
		//default value
    	temp_sec_key[2] = 0x7068794B; //PHY+
    	temp_sec_key[3] = 0x06027878; //62XX

		//validate phy_sec_mic
	    LL_ENC_AES128_Encrypt0((uint8_t * )temp_sec_key, (uint8_t * ) temp_plaintext, (uint8_t * ) cyr_out);

//		otp_read_data_byte((OTP_BASE_ADDR + OTP_SECURE_AREA_ADDR + 0x10), pOTP_SECURE_MIC, 16, OTP_USER_READ_MODE);
		
	    if(memcmp((uint8_t * ) cyr_out, (const void*)pOTP_SECURE_MIC, 128/8) != 0){
	        g_phy_sec_key_valid = PHY_SEC_KEY_FAIL;
	        return PPlus_SUCCESS;
	    }

	    //generate g_phy_sec_key
	    g_phy_sec_key_valid = PHY_SEC_KEY_CCM;
	    LL_ENC_AES128_Encrypt0((uint8_t * )temp_sec_key, (uint8_t * ) pOTP_SECURE_MIC, (uint8_t * ) g_phy_sec_key);
		  
		
		for(i=0; i<4; i++){
			temp_sec_key[i] = 0;
		}
			
	    //load iv
//	    otp_read_data_byte((OTP_BASE_ADDR + OTP_SECURE_AREA_ADDR + 0x20), ((uint8_t*)pOTP_SECURE_IV), 16, OTP_USER_READ_MODE);
		
	    for(i = 0; i < 16; i++){
	        if(pOTP_SECURE_IV[i] == 0 || pOTP_SECURE_IV[i] == 0xff )
	            continue;
	        p_phy_sec_iv = (uint8_t*)pOTP_SECURE_IV;
	        break;
	    }
	}
	else{
		
		if(SPIF_SECURE_KEY_L == 0xffffffff && SPIF_SECURE_KEY_H == 0xffffffff){
			return PPlus_SUCCESS;
		}
		temp_sec_key[0] = SPIF_SECURE_KEY_L;
    	temp_sec_key[1] = SPIF_SECURE_KEY_H;
    	//default value
    	temp_sec_key[2] = 0x7068794B; //PHY+
    	temp_sec_key[3] = 0x06027878; //62XX

		//validate phy_sec_mic
	    LL_ENC_AES128_Encrypt0((uint8_t * )temp_sec_key, (uint8_t * ) temp_plaintext, (uint8_t * ) cyr_out);
		
	    if(memcmp((uint8_t * ) cyr_out, (const void*)pSPIF_SECURE_MIC, 128/8) != 0){
	        g_phy_sec_key_valid = PHY_SEC_KEY_FAIL;
	        return PPlus_SUCCESS;
	    }

	    //generate g_phy_sec_key
	    g_phy_sec_key_valid = PHY_SEC_KEY_CCM;
	    LL_ENC_AES128_Encrypt0((uint8_t * )temp_sec_key, (uint8_t * ) pSPIF_SECURE_MIC, (uint8_t * ) g_phy_sec_key);
		  
		for(i=0; i<4; i++){
			temp_sec_key[i] = 0;
		}
			
	    //load iv
	    for(i = 0; i < 16; i++){
	        if(pSPIF_SECURE_IV[i] == 0 || pSPIF_SECURE_IV[i] == 0xff )
	            continue;
	        p_phy_sec_iv = (uint8_t*)pSPIF_SECURE_IV;
	        break;
	    }
	}

	
	return PPlus_SUCCESS;
}


