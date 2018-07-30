/******************************************************************************
 * @file    sgfx_credentials_template.c
 * @author  MCD Application Team
 * @version V1.1.0
 * @date    30-April-2018
 * @brief   manages keys and encryption algorithm
 ******************************************************************************
 * @attention
 *
 * <h2><center>&copy; Copyright (c) 2017 STMicroelectronics International N.V.
 * All rights reserved.</center></h2>
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted, provided that the following conditions are met:
 *
 * 1. Redistribution of source code must retain the above copyright notice,
 *    this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright notice,
 *    this list of conditions and the following disclaimer in the documentation
 *    and/or other materials provided with the distribution.
 * 3. Neither the name of STMicroelectronics nor the names of other
 *    contributors to this software may be used to endorse or promote products
 *    derived from this software without specific written permission.
 * 4. This software, including modifications and/or derivative works of this
 *    software, must execute solely and exclusively on microcontroller or
 *    microprocessor devices manufactured by or for STMicroelectronics.
 * 5. Redistribution and use of this software other than as permitted under
 *    this license is void and will automatically terminate your rights under
 *    this license.
 *
 * THIS SOFTWARE IS PROVIDED BY STMICROELECTRONICS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS, IMPLIED OR STATUTORY WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY, FITNESS FOR A
 * PARTICULAR PURPOSE AND NON-INFRINGEMENT OF THIRD PARTY INTELLECTUAL PROPERTY
 * RIGHTS ARE DISCLAIMED TO THE FULLEST EXTENT PERMITTED BY LAW. IN NO EVENT
 * SHALL STMICROELECTRONICS OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
 * LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA,
 * OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF
 * LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING
 * NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
 * EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 ******************************************************************************
 */
#include <stdint.h>
#include <string.h>
#include "aes.h"
#include "st_sigfox_api.h"
#include "sgfx_credentials.h"
#include "se_nvm.h"
/* Private typedef -----------------------------------------------------------*/


#define MANUF_DEVICE_ID_LENGTH     4
#define MANUF_SIGNATURE_LENGTH     16
#define MANUF_VER_LENGTH           1
#define MANUF_SPARE_1              3
#define MANUF_DEVICE_KEY_LENGTH    16
#define MANUF_PAC_LENGTH           8
#define MANUF_SPARE_2   14
#define MANUF_CRC_LENGTH           2


typedef struct manuf_device_info_s
{
    /* 16bits block 1 */
    sfx_u8 dev_id[MANUF_DEVICE_ID_LENGTH];
    sfx_u8 pac[MANUF_PAC_LENGTH];
    sfx_u8 ver[MANUF_VER_LENGTH];
    sfx_u8 spare1[MANUF_SPARE_1];
    /* 16bits block 2 */
    sfx_u8 dev_key[MANUF_DEVICE_KEY_LENGTH];
    /* 16bits block 3 */
     sfx_u8 spare2[MANUF_SPARE_2];
    sfx_u8 crc[MANUF_CRC_LENGTH];
} manuf_device_info_t;

                                      
/*PREPROCESSOR CONVERSION*/
#define DECIMAL2STRING_DEF(s) #s
#define DECIMAL2STRING(s) DECIMAL2STRING_DEF(s)
                                      
#ifndef UNUSED
#define UNUSED(x) ((void) x)
#endif
#ifndef ALIGN
#define ALIGN(n)             __attribute__((aligned(n)))
#endif

/*SIGfox defines*/


#define SIGNATURE_LEN 16 /*bytes*/

#define SIGFOX_DATA_LEN 48 /*bytes*/

#define CREDENTIALS_VERSION 11

#define PUBLIC_KEY    {0x00, 0x11, 0x22, 0x33, 0x44, 0x55, 0x66, 0x77, 0x88, 0x99, 0xAA, 0xBB, 0xCC, 0xDD, 0xEE, 0xFF}
/*CREDENTIAL_KEY may be used to encrypt sigfox_data
  CREDENTIAL_KEY must be aligned with the sigfox tool generating and encrypting the sigfox_data*/
/*
#define CREDENTIAL_KEY {0x01, 0x01, 0x01, 0x01, 0x01, 0x01, 0x01, 0x01, 0x01, 0x01, 0x01, 0x01, 0x01, 0x01, 0x01, 0x01} 
*/
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
static sfx_bool encrypt_flag = SFX_FALSE;

static aes_context AesContext;

extern sfx_u8 encrypted_sigfox_data[SIGFOX_DATA_LEN];

static uint8_t device_public_key[]=PUBLIC_KEY;

static const char sgfxSeeLibVersion[]="." DECIMAL2STRING(CREDENTIALS_VERSION);

static uint8_t session_key[SIGNATURE_LEN]={0};

/* Private function prototypes -----------------------------------------------*/

static void CREDENTIALS_get_key (uint8_t* key, sfx_key_type_t KeyType );

static sfx_error_t CREDENTIALS_get_cra(sfx_u8 *decrypted_data, sfx_u8 *data_to_decrypt, sfx_u8 data_len);

/* Public function definition -----------------------------------------------*/

sfx_error_t CREDENTIALS_aes_128_cbc_encrypt(uint8_t* encrypted_data, uint8_t* data_to_encrypt, uint8_t blocks)
{
  uint8_t iv[N_BLOCK] = {0x00};
  
  uint8_t key[AES_KEY_LEN];
  
  sfx_key_type_t KeyType =SE_NVM_get_key_type();
  
  CREDENTIALS_get_key ( key, KeyType );

  aes_set_key( key, AES_KEY_LEN,  &AesContext);
  
  memset(key, 0, AES_KEY_LEN);

  aes_cbc_encrypt( data_to_encrypt,
                     encrypted_data,
                     blocks,
                     iv,
                     &AesContext );
  
  return SFX_ERR_NONE;
}

sfx_error_t CREDENTIALS_aes_128_cbc_encrypt_with_session_key(uint8_t* encrypted_data, uint8_t* data_to_encrypt, uint8_t blocks)
{
  uint8_t iv[N_BLOCK] = {0x00};

  aes_set_key( session_key, AES_KEY_LEN,  &AesContext);

  aes_cbc_encrypt( data_to_encrypt,
                     encrypted_data,
                     blocks,
                     iv,
                     &AesContext );
  
  return SFX_ERR_NONE;
}

sfx_error_t CREDENTIALS_wrap_session_key( uint8_t* data, uint8_t blocks)
{
  uint8_t iv[N_BLOCK] = {0x00};
  
  uint8_t key[AES_KEY_LEN];
  
  CREDENTIALS_get_key ( key, CREDENTIALS_KEY_PRIVATE);

  aes_set_key( key, AES_KEY_LEN,  &AesContext);
  
  memset(key, 0, AES_KEY_LEN);

  aes_cbc_encrypt( data,
                   session_key,
                   blocks,
                   iv,
                   &AesContext );
  
  return SFX_ERR_NONE;
}

const char* CREDENTIALS_get_version( void )
{
  return sgfxSeeLibVersion;
}

void CREDENTIALS_get_dev_id( uint8_t* dev_id)
{
    manuf_device_info_t DeviceInfo;
    
    CREDENTIALS_get_cra( (uint8_t*) &DeviceInfo, encrypted_sigfox_data, sizeof(manuf_device_info_t) );

    memcpy(dev_id, DeviceInfo.dev_id, MANUF_DEVICE_ID_LENGTH);
  
    /*clear key*/
    memset( DeviceInfo.dev_key, 0, AES_KEY_LEN);
}

void CREDENTIALS_get_initial_pac( uint8_t* pac)
{
    manuf_device_info_t DeviceInfo;
    
    CREDENTIALS_get_cra( (uint8_t*) &DeviceInfo, encrypted_sigfox_data, sizeof(manuf_device_info_t) );
    /*clear key*/
    memcpy(pac, DeviceInfo.pac, MANUF_PAC_LENGTH);
    /*clear key*/
    memset( DeviceInfo.dev_key, 0, AES_KEY_LEN);
}

sfx_bool CREDENTIALS_get_payload_encryption_flag(void)
{
    return encrypt_flag;
}

/* Private function definition -----------------------------------------------*/

static void CREDENTIALS_get_key(uint8_t* key, sfx_key_type_t KeyType)
{
  switch (KeyType)
  {
    case CREDENTIALS_KEY_PUBLIC:
    {
      memcpy(key, device_public_key, AES_KEY_LEN);
      
      break;
    }
    case CREDENTIALS_KEY_PRIVATE:
    {
      manuf_device_info_t DeviceInfo;
    
      CREDENTIALS_get_cra( (uint8_t*) &DeviceInfo, encrypted_sigfox_data, sizeof(manuf_device_info_t) );

      memcpy(key, DeviceInfo.dev_key, AES_KEY_LEN);
    
      memset( DeviceInfo.dev_key, 0, AES_KEY_LEN);
      
      break;
    }
    default:
      break;
  }
}


static sfx_error_t CREDENTIALS_get_cra(sfx_u8 *decrypted_data, sfx_u8 *data_to_decrypt, sfx_u8 data_len)
{
#ifdef CREDENTIAL_KEY
  uint8_t iv[N_BLOCK] = {0x00};
  
  uint8_t CredentialKey[AES_KEY_LEN]=CREDENTIAL_KEY;
  
  /*device is provisioned with sigfox_data.h   */
  /*encrypted with CREDENTIAL_KEY in Sigfox Tool*/
  aes_set_key( CredentialKey, AES_KEY_LEN,  &AesContext);
  
  memset( CredentialKey, 0, AES_KEY_LEN);

  aes_cbc_decrypt( data_to_decrypt,
                   decrypted_data,
                   sizeof(manuf_device_info_t) / AES_KEY_LEN,
                   iv,
                   &AesContext );
#else
  /* default sigfox_data.h provided, sigfox_data.h is not encrypted*/
  memcpy( (uint8_t*) decrypted_data, (uint8_t*) data_to_decrypt, sizeof(manuf_device_info_t) );

#endif

  return SFX_ERR_NONE;
}





/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
