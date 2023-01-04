/*******************************************************************************
  Filename:       ll_enc.c
  Revised:         
  Revision:        

  Description:    This file contains the BLE encryption API for the LL.


*******************************************************************************/

/*******************************************************************************
 * INCLUDES
 */
#include "ll_common.h"
#include "ll.h"
#include "ll_enc.h"
#include "string.h"
#include "jump_function.h"
#include "phy_aes.h"

/*******************************************************************************
 * MACROS
 */
#define ADJUST_PKT_LEN( len )                                                  \
  (((len)<=LL_ENC_BLOCK_LEN)?LL_ENC_BLOCK_LEN-(len):(2*LL_ENC_BLOCK_LEN)-(len))
  
#define LL_ENC_BASE         0x40040000               // LL HW AES engine Base address  

#define LL_ENC_ENCRYPT_DONE_MASK        0x0001
#define LL_ENC_DECRYPT_FAIL_MASK        0x0002
#define LL_ENC_DECRYPT_SUCC_MASK        0x0004
#define LL_ENC_SINGLE_MODE_DONE_MASK    0x0008

/*******************************************************************************
 * CONSTANTS
 */

#define LL_ENC_MAX_RF_ADC_WAIT_COUNT  250

/*******************************************************************************
 * TYPEDEFS
 */

/*******************************************************************************
 * LOCAL VARIABLES
 */

// generic packet for cyrptography: length, hdr, payload including MIC
// Note: One extra byte so besides length and header, there is a multiple of
//       two 16 byte blocks available.
//uint8 dataPkt[2*LL_ENC_BLOCK_LEN];

/*******************************************************************************
 * GLOBAL VARIABLES
 */

// cache of FIPS compliant true random numbers for IV and SKD formation
uint8 cachedTRNGdata[ LL_ENC_TRUE_RAND_BUF_SIZE ];

#ifdef DEBUG_ENC
const uint8 testSKD[] = { 0x79, 0x68, 0x57, 0x46, 0x35, 0x24, 0x13, 0x02,   // slave
                          0x13, 0x02, 0xF1, 0xE0, 0xDF, 0xCE, 0xBD, 0xAC }; // master
const uint8 testIV[]  = { 0xBE, 0xBA, 0xAF, 0xDE,    // slave
                          0x24, 0xAB, 0xDC, 0xBA };  // master
#endif  // DEBUG_ENC


// generic zero block used for encryption
//const uint8 zeroBlock[32] = {0};

/*******************************************************************************
 * PROTOTYPES
 */

//void LL_ENC_StartAES( uint8 aesCmd );

/*******************************************************************************
 * API
 */


/*******************************************************************************
 * @fn          LL_ENC_ReverseBytes API
 *
 * @brief       This function is used to reverse the order of the bytes in
 *              an array in place.
 *
 *              Note: The max length is 128 bytes; the min length is > 0.
 *
 * input parameters
 *
 * @param       buf - Pointer to buffer containing bytes to be reversed.
 * @param       len - Number of bytes in buffer.
 *
 * output parameters
 *
 * @param       buf - Pointer to buffer containing reversed bytes.
 *
 * @return      None.
 */
void LL_ENC_ReverseBytes( uint8 *buf,
                          uint8 len )
{
  uint8 temp;
  uint8 index = (uint8)(len - 1);
  uint8 i;

  // adjust length as only half the operations are needed
  len >>= 1;

  // reverse the order of the bytes
  for (i=0; i<len; i++)
  {
    temp           = buf[i];
    buf[i]         = buf[index - i];
    buf[index - i] = temp;
  }

  return;
}


/*******************************************************************************
 * @fn          LL_ENC_GeneratePseudoRandNum API
 *
 * @brief       This function is used to generate a pseudo random byte.
 *
 * input parameters
 *
 * @param       None.
 *
 * output parameters
 *
 * @param       None.
 *
 * @return      A pseudo random byte.
 */
uint8 LL_ENC_GeneratePseudoRandNum( void )
{
    uint8  rand_char;

    LL_Rand(&rand_char, 1);
	return (rand_char);
}

/*******************************************************************************
 * @fn          LL_ENC_AES128_Encrypt API
 *
 * @brief       This function takes a key, plaintext, and generates ciphertext
 *              by AES128 encryption. This function is used to generate the
 *              BLE Session Key (SK) from the Long Term Key (LTK) and Session
 *              Key Diversifier (SKD).
 *
 *              Note: The array indexes are MSO..LSO for LTK, SKD, and SK.
 *
 * input parameters
 *
 * @param       key       - The 128 bit key to be used for encryption.
 * @param       plaintext - The 128 bit plain text before encryption.
 *
 * output parameters
 *
 * @param       ciphertext - The 128 bit cipher text after encryption.
 *
 * @return      None.
 */
void LL_ENC_AES128_Encrypt( uint8 *key,
                            uint8 *plaintext,
                            uint8 *ciphertext )
{ 
    volatile int delay;
    uint32_t temp;
    AP_PCR->SW_CLK    |= BIT(MOD_AES);
    // disable AES engine
    *(volatile uint32_t *) LL_ENC_BASE = 0x0;
    
	//config KEY, the reg write order is recommend by Xiao Guijun
    *(volatile uint32_t *)(LL_ENC_BASE + 0x2c) = key[0] << 24 | key[1] << 16 | key[2] << 8 | key[3];
    key += 4;    
    *(volatile uint32_t *)(LL_ENC_BASE + 0x28) = key[0] << 24 | key[1] << 16 | key[2] << 8 | key[3];
    key += 4;
    *(volatile uint32_t *)(LL_ENC_BASE + 0x24) = key[0] << 24 | key[1] << 16 | key[2] << 8 | key[3];
    key += 4;    
    *(volatile uint32_t *)(LL_ENC_BASE + 0x20) = key[0] << 24 | key[1] << 16 | key[2] << 8 | key[3];
 
	//config data in, the reg write order is recommend by Xiao Guijun
    *(volatile uint32_t *)(LL_ENC_BASE + 0x3c) = plaintext[0] << 24 | plaintext[1] << 16 | plaintext[2] << 8 | plaintext[3];
    plaintext += 4;    
    *(volatile uint32_t *)(LL_ENC_BASE + 0x38) = plaintext[0] << 24 | plaintext[1] << 16 | plaintext[2] << 8 | plaintext[3];
    plaintext += 4;
    *(volatile uint32_t *)(LL_ENC_BASE + 0x34) = plaintext[0] << 24 | plaintext[1] << 16 | plaintext[2] << 8 | plaintext[3];
    plaintext += 4;    
    *(volatile uint32_t *)(LL_ENC_BASE + 0x30) = plaintext[0] << 24 | plaintext[1] << 16 | plaintext[2] << 8 | plaintext[3];
    	
	//set AES ctrl reg
	*(volatile uint32_t *)(LL_ENC_BASE + 0x04) = 0xf10;         // single mode, encrypt, revert bytes in word
	
	//set interrupt enable
//	*(int *) 0x40040010 = 0xf;
	
	//set AES en
	*(volatile uint32_t *) LL_ENC_BASE = 0x1;
    
    // need delay 10-20 cycle for single mode encrypt
    delay = 20;
    while (delay --) ;	
    
    // read ciphertext
    temp = *(volatile uint32_t *)(LL_ENC_BASE + 0x5c);
    ciphertext[0] = (temp >> 24) & 0xff;    
    ciphertext[1] = (temp >> 16) & 0xff;    
    ciphertext[2] = (temp >> 8) & 0xff;    
    ciphertext[3] = temp & 0xff;   
    ciphertext += 4;    
    
    temp = *(volatile uint32_t *)(LL_ENC_BASE + 0x58);
    ciphertext[0] = (temp >> 24) & 0xff;    
    ciphertext[1] = (temp >> 16) & 0xff;    
    ciphertext[2] = (temp >> 8) & 0xff;    
    ciphertext[3] = temp & 0xff;   
    ciphertext += 4;  
    
    temp = *(volatile uint32_t *)(LL_ENC_BASE + 0x54);
    ciphertext[0] = (temp >> 24) & 0xff;    
    ciphertext[1] = (temp >> 16) & 0xff;    
    ciphertext[2] = (temp >> 8) & 0xff;    
    ciphertext[3] = temp & 0xff;   
    ciphertext += 4;  
    
    temp = *(volatile uint32_t *)(LL_ENC_BASE + 0x50);  
    ciphertext[0] = (temp >> 24) & 0xff;    
    ciphertext[1] = (temp >> 16) & 0xff;    
    ciphertext[2] = (temp >> 8) & 0xff;    
    ciphertext[3] = temp & 0xff;  

    // disable AES
	*(int *) 0x40040000 = 0x0;    
	AP_PCR->SW_CLK	  &= ~BIT(MOD_AES);

    return;
}

// Note: The hardware RNG is failed in some boards, need investigate the issue 
// before using this function
/*******************************************************************************
 * @fn          LL_ENC_GenerateTrueRandNum API
 *
 * @brief       This function is used to generate a sequence of true random
 *              bytes. The result is FIPS compliant. This routine also re-seeds
 *              the PRNG, ensuring an improper seed isn't used.
 *
 *              Note: This is done using the radio. Care must be taken when
 *                    using this function to ensure that there are no conflicts
 *                    with RF.
 *
 *              Note: This algorithm was not checked for FIPS compliance over
 *                    voltage change.
 *
 * input parameters
 *
 * @param       len  - The number of true random bytes desired (1..255).
 *
 * output parameters
 *
 * @param       buf - A pointer to a buffer for storing true random bytes.
 *
 * @return      LL_STATUS_SUCCESS, LL_STATUS_ERROR_BAD_PARAMETER,
 *              LL_STATUS_ERROR_RNG_FAILURE
 */
uint8 LL_ENC_GenerateTrueRandNum( uint8 *buf,
                                  uint8 len )
{
    //need to be replaced by TRNG_Rand
    return( LL_Rand(buf,len) );
}

/*******************************************************************************
 * @fn          LL_ENC_GenDeviceSKD API
 *
 * @brief       This function is used to generate a device's half of the SKD.
 *
 * input parameters
 *
 * @param       None.
 *
 * output parameters
 *
 * @param       SKD - A pointer to a buffer in which to place the device's SKD.
 *
 * @return      None.
 */
void LL_ENC_GenDeviceSKD( uint8 *SKD )
{
  uint8 i;

  // used cached FIPS compliant TRNG data
  // Note: SKD (master or slave) ues first LL_ENC_SKD_LEN/2 of the bytes.
  // Note: Calling routine will schedule a cache update as a postRF operation.
  for (i=0; i<LL_ENC_SKD_LINK_LEN; i++)
  {
    SKD[i] = cachedTRNGdata[i];
  }

  return;
}



/*******************************************************************************
 * @fn          LL_ENC_GenDeviceIV API
 *
 * @brief       This function is used to generate a device's half of the IV.
 *
 * input parameters
 *
 * @param       None.
 *
 * output parameters
 *
 * @param       IV - A pointer to a buffer in which to place the device's IV.
 *
 * @return      None.
 */
void LL_ENC_GenDeviceIV( uint8 *IV )
{
  uint8 i;

  // used cached FIPS compliant TRNG data
  // Note: SKD (master or slave) takes first LL_ENC_SKD_LEN/2 of the bytes.
  // Note: Calling routine will schedule a cache update as a postRF operation.
  for (i=0; i<LL_ENC_IV_LINK_LEN; i++)
  {
    IV[i] = cachedTRNGdata[i+(LL_ENC_SKD_LEN / 2)];
  }

  return;
}

/*******************************************************************************
 * @fn          LL_ENC_GenerateNonce API
 *
 * @brief       This function is used to form the Nonce.
 *
 *              Note: This routine assumes that the nonce already contains the
 *                    IV in bytes 5..13.
 * input parameters
 *
 * @param       pktCnt    - The packet count.
 * @param       direction - LL_ENC_TX_DIRECTION or LL_ENC_RX_DIRECTION.
 * @param       nonce     - Pointer to Nonce to be updated.
 *
 * output parameters
 *
 * @param       nonce - Pointer to Nonce that has been updated.
 *
 * @return      None.
 */
void LL_ENC_GenerateNonce( uint32 pktCnt,
                           uint8  direction,
                           uint8  *nonce )
{
  // add packet count to the nonce
  // Note: Packet count is supposed to be 39 bits; assume 32 for now.
  nonce[0] = ((uint8 *)&pktCnt)[0]; //(pktCnt >>  0) & 0xFF;
  nonce[1] = ((uint8 *)&pktCnt)[1]; //(pktCnt >>  8) & 0xFF;
  nonce[2] = ((uint8 *)&pktCnt)[2]; //(pktCnt >> 16) & 0xFF;
  nonce[3] = ((uint8 *)&pktCnt)[3]; //(pktCnt >> 24) & 0xFF;

  // add direction bit to MSO
  // Note: Last 7 bits of 39-bit packet count are assumed to be zero as only
  //       a 32 bit counter is currently supported.
  nonce[4] = (direction << 7);

  return;
}



/*******************************************************************************
 * @fn          LL_ENC_LoadKey API
 *
 * @brief       This function is used to load the cryptography key into the
 *              AES engine.
 *
 * input parameters
 *
 * @param       key - Pointer to 16 byte cryptography key.
 *
 * output parameters
 *
 * @param       None.
 *
 * @return      None.
 */
//void LL_ENC_LoadKey( uint8 *key )
//{
//	//config KEY
//    *(volatile uint32_t *)(LL_ENC_BASE + 0x2c) = key[0] << 24 | key[1] << 16 | key[2] << 8 | key[3];
//    key += 4;    
//    *(volatile uint32_t *)(LL_ENC_BASE + 0x28) = key[0] << 24 | key[1] << 16 | key[2] << 8 | key[3];
//    key += 4;
//    *(volatile uint32_t *)(LL_ENC_BASE + 0x24) = key[0] << 24 | key[1] << 16 | key[2] << 8 | key[3];
//    key += 4;    
//    *(volatile uint32_t *)(LL_ENC_BASE + 0x20) = key[0] << 24 | key[1] << 16 | key[2] << 8 | key[3];
//
//    return;
//}


/*******************************************************************************
 * @fn          LL_ENC_Encrypt API
 *
 * @brief       This function is used to encrypt a packet. It loads the key,
 *              generates the nonce, generates the MIC, and encrypts the data
 *              and MIC.
 *
 *              Note: This routine assumes that the buffer is 32 bytes and has
 *                    already been zero padded.
 *
 * input parameters
 *
 * @param       connPtr - A pointer to this connection's information.
 * @param       pktHdr  - Packet header, LLID only.
 * @param       pktLen  - Length of buffer to be encrypted.
 * @param       pBuf    - Pointer to buffer to be encrypted.
 *
 * output parameters
 *
 * @param       pBuf   - Pointer to encrypted buffer.
 *
 * @return      None.
 */
//static void xor128bit(uint8_t * x,uint8_t  *y,uint8_t  *z)
//{
//    for(int i=0;i<16;i++)
//    {
//        z[i]=x[i]^y[i];
//    }
//}
void LL_ENC_Encrypt( llConnState_t *connPtr,
                     uint8          pktHdr,
                     uint8          pktLen,
                     uint8         *pBuf )
{
	// input pktLen not include header
//    uint8 loopNum = ( pktLen + 15 ) / 16;
    // generate the nonce based on packet count, IV, and direction
    LL_ENC_GenerateNonce( connPtr->encInfo.txPktCount,
                          LL_ENC_TX_DIRECTION_SLAVE,
                          connPtr->encInfo.nonce );

//	LOG("%s\n",__func__);
	uint8 aad[1] = {0xE3 & pktHdr};

	aes_ccm_encrypt(	AES_CCM_MOD_NORMAL,				/*normal mode or phyplus mode*/
						connPtr->encInfo.SK, 			/*key*/
						connPtr->encInfo.nonce, 		/*nounce*/
						aad, 							/*aad variable in B1 field : header mask SN,NESN,MD */
						0x1,							/*aadLen = 1*/
						0x49,							/*bFlg : B0 LSB Byte0 = 0x49*/
						0x1,							/*aFlg : A0 LSB Byte0 = 0x01*/
						pBuf,							/*din*/ 
						pktLen,							/*dLen : data length*/
						pBuf,							/*dout*/
						&pBuf[pktLen]);					/*micOut*/

    // up the count for the next TX'ed data packet
    // Note: This is supposed to be 39 bit counter, but for now, we don't
    //       envision receiving 550 billion packets during a connection!
    connPtr->encInfo.txPktCount++;
    return;
}


/*******************************************************************************
 * @fn          LL_ENC_Decrypt API
 *
 * @brief       This function is used to decrypt a packet. It loads the key,
 *              generates the nonce, decrypts the data and MIC, generates a
 *              MIC based on the decrypted data, and authenticates the packet
 *              by comparing the two MIC values.
 *
 *              Note: This routine assumes that the buffer is 32 bytes,
 *                    regardless of the number of bytes of data.
 *
 * input parameters
 *
 * @param       connPtr - A pointer to this connection's information.
 * @param       pktHdr  - Packet header, LLID only.
 * @param       pktLen  - Length of buffer to be decrypted.
 * @param       pBuf    - Pointer to buffer to be decrypted.
 *
 * output parameters
 *
 * @param       pBuf   - Pointer to decrypted buffer.
 *
 * @return      None.
 */
//	1. If Clen¡Ü Tlen, then return INVALID.
//	2. Apply the counter generation function to generate the counter blocks Ctr0, Ctr1,
//												¡­, Ctrm, where m = (Clen - Tlen)/128 (bits)
//	3. For j=0 to m, do Sj = CIPHK(Ctrj).
//	4. Set S= S1 || S2 || ¡­|| Sm.
//	5. Set P=MSBClen-Tlen(C) ¨�? MSBClen-Tlen(S).
//	6. Set T=LSBTlen(C) ¨�? MSBBTlen(S0).
//	7. If N, A, or P is not valid, as discussed in Section 5.4, then return INVALID, else
//	apply the formatting function to (N, A, P) to produce the blocks BB0, B1 B , ¡­, BBr.
//	8. Set Y0= CIPHK(BB0).
//	9. For i = 1 to r, do Yj = CIPHK(BBi ¨�? Yi-1).
//	10. If T ¡Ù MSBTlen(Yr), then return INVALID, else return P.
uint8 LL_ENC_Decrypt( llConnState_t *connPtr,
                      uint8          pktHdr,
                      uint8          pktLen,
                      uint8         *pBuf )
{	
	LL_ENC_GenerateNonce( connPtr->encInfo.rxPktCount,
				          LL_ENC_RX_DIRECTION_SLAVE,
				          connPtr->encInfo.nonce );

	uint8 aad[1] = {0xE3 & pktHdr};
	int ret = aes_ccm_decrypt(	AES_CCM_MOD_NORMAL,				/*normal mode or phyplus mode*/
								connPtr->encInfo.SK, 			/*key*/
								connPtr->encInfo.nonce, 		/*nounce*/
								aad, 							/*aad variable in B1 field : header mask SN,NESN,MD */
								0x1,							/*aadLen = 1*/
								0x49,							/*bFlg : B0 LSB Byte0 = 0x49*/
								0x1,							/*aFlg : A0 LSB Byte0 = 0x01*/
								pBuf,							/*din*/ 
								pktLen,							/*dLen : data length*/
								pBuf,							/*dout*/
                				&pBuf[pktLen]);					/*micIn*/

    connPtr->encInfo.rxPktCount++;  
	return ((uint8)ret);
}


#define LEN_24BIT                 3 // Number of bytes in a 24 bit number
#define PRAND_SIZE                LEN_24BIT // PRAND size in the Private Resolvable Address calculation

/*********************************************************************
 * @fn          LL_ENC_sm_ah
 *
 * @brief       Random address hash function.
 *
 * @param       pK - key is 128 bits, LSByte first
 * @param       pR - 24 bit, LSByte first
 * @param       pAh - pointer to 24 bit results space
 *
 * @return      None
 */
 #if 0
void LL_ENC_sm_ah( uint8 *pK, uint8 *pR, uint8 *pAh )
{
    uint8 key[KEYLEN];              // Key to encrypt with
    uint8 plainTextData[KEYLEN];    // Plain Text data to encrypt
    uint8 result[KEYLEN];           // Result of encrypt (key + Plain Text data)    

	memset(plainTextData, 0, KEYLEN);        
    // Copy the encrypt variables
    VOID osal_revmemcpy( key, pK, KEYLEN );
    VOID osal_revmemcpy( &(plainTextData[KEYLEN - LEN_24BIT]),
                      pR, LEN_24BIT );
  
    LL_ENC_AES128_Encrypt(key, plainTextData, result);
  
    // Copy the results
    VOID osal_revmemcpy( pAh, &(result[KEYLEN - LEN_24BIT]), LEN_24BIT );
}
#endif
/*******************************************************************************
 */

