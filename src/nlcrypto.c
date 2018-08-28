/*
 *
 *    Copyright (c) 2013-2018 Nest Labs, Inc.
 *    All rights reserved.
 *
 *    Licensed under the Apache License, Version 2.0 (the "License");
 *    you may not use this file except in compliance with the License.
 *    You may obtain a copy of the License at
 *
 *        http://www.apache.org/licenses/LICENSE-2.0
 *
 *    Unless required by applicable law or agreed to in writing, software
 *    distributed under the License is distributed on an "AS IS" BASIS,
 *    WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 *    See the License for the specific language governing permissions and
 *    limitations under the License.
 */

/*
 *    Description:
 *      This file implements the NRF52x cryptography accelerator.
 *
 */

#include <errno.h>
#include <string.h>
#include <stdint.h>
#include <sys/types.h>

#include <nlplatform.h>
#include <nlplatform/nlcrypto.h>

#include <nrf52x/device/nrf52840.h>
#include <nrf52x/device/nrf52840_bitfields.h>

/* CryptoCell 310 */
#ifdef NL_BOOTLOADER
#include <nrf_cc310_bl_init.h>
#include <nrf_cc310_bl_hash_sha256.h>
#include <nrf_cc310_bl_ecdsa_verify_secp224r1.h>
#else
#include <crys_ecpki_build.h>
#include <crys_ecpki_domain.h>
#include <crys_ecpki_ecdsa.h>
#endif
#include <crys_hash.h>
#include <crys_rnd.h>
#include <sns_silib.h>

// cryptocell sha256 hangs if given a buffer 64KB or above.
// from testing, we can go up to about 63KB but we'll limit
// ourselves to 32KB chunks
#define MAX_HASH_CHUNK_SIZE 0x8000

#ifdef STATIC_HASH_RAM_BUFFER_SIZE

/* Bootloader has more RAM to use and cares more about speed for
 * image verification so we use a large static RAM buffer.
 */
static uint8_t temp_buf[STATIC_HASH_RAM_BUFFER_SIZE] __attribute__((aligned(4))) NL_SYMBOL_AT_PLATFORM_DATA_SECTION(".noinit");
#define HASH_RAM_BUFFER_SIZE STATIC_HASH_RAM_BUFFER_SIZE

#else /* STATIC_HASH_RAM_BUFFER_SIZE */

/* HASH_RAM_BUFFER_SIZE will be the size of a temporary buffer on the
 * stack used to copy non-DataRAM arguments before passing to the
 * CryptoCell HASH APIs. CryptoCell APIs can take DataRAM addresses.
 * HASH_RAM_BUFFER_SIZE is set to NLCRYPTO_STACK_RAM_BUFFER_SIZE if the
 * latter is defined. Otherwise, HASH_RAM_BUFFER_SIZE is set to a
 * default size of 256 bytes. The size is a tradeoff of efficiency
 * vs stack usage. The bigger the better for speed, with the sweet
 * spot at around 4KB, but that's too big for a stack buffer. Should
 * be at least 32-bytes, the size of SHA256 block.
 */
#ifdef NLCRYPTO_STACK_RAM_BUFFER_SIZE
#define HASH_RAM_BUFFER_SIZE NLCRYPTO_STACK_RAM_BUFFER_SIZE
#else
#define HASH_RAM_BUFFER_SIZE 256
#endif /* NLCRYPTO_STACK_RAM_BUFFER_SIZE */

#endif /* STATIC_HASH_RAM_BUFFER_SIZE */

#ifndef NL_BOOTLOADER
static CRYS_RND_Context_t s_rnd_context;
static CRYS_RND_WorkBuff_t s_rnd_work_buffer;
static CRYS_RND_Context_t *s_rnd_context_ptr = &s_rnd_context;
static CRYS_RND_WorkBuff_t *s_rnd_work_buffer_ptr = &s_rnd_work_buffer;

#include <FreeRTOS.h>
#include <semphr.h>

static StaticSemaphore_t s_lock_obj;
static SemaphoreHandle_t s_lock;
#endif

typedef struct {
    SaSiAesUserContext_t aes_ctx;
    uint8_t last_block[SASI_AES_BLOCK_SIZE_IN_BYTES];
    uint32_t last_block_len;
} nlplatform_aes_cmac_internal_t;

/* No string to keep this as small as possible for bootloader */
_Static_assert(sizeof(CRYS_HASHUserContext_t) == sizeof(nlplatform_sha256_t), "");
_Static_assert(sizeof(nlplatform_aes_cmac_internal_t) == sizeof(nlplatform_aes_cmac_t), "");

void nlplatform_crypto_init(void)
{
#ifndef NL_BOOTLOADER
    assert(s_lock == NULL);
    s_lock = xSemaphoreCreateMutexStatic(&s_lock_obj);
#endif

    NRF_CRYPTOCELL->ENABLE = 1;
#ifdef NL_BOOTLOADER
    nrf_cc310_bl_init();
#else
    SaSi_LibInit(s_rnd_context_ptr, s_rnd_work_buffer_ptr);
#endif
    NRF_CRYPTOCELL->ENABLE = 0;
}

static void cryptocell_enable(void)
{
#ifndef NL_BOOTLOADER
    xSemaphoreTake(s_lock, portMAX_DELAY);
#endif
    NRF_CRYPTOCELL->ENABLE = 1;
}

static void cryptocell_disable(void)
{
    NRF_CRYPTOCELL->ENABLE = 0;
#ifndef NL_BOOTLOADER
    xSemaphoreGive(s_lock);
#endif
}

/*****************************************************************************/
/* AES                                                                       */
/*****************************************************************************/

// Block cipher function in nrf52x does not expect the expanded key, in
// fact, the key expansion appears to take place in the memory of the
// crypto module.  As a result, the set key operation is a simple
// copy.
void nlplatform_AES128ECB_set_encrypt_key(const uint8_t *userKey, uint8_t *key)
{
    memcpy(key, userKey, SASI_AES_BLOCK_SIZE_IN_BYTES);
}

static void aes_init(SaSiAesUserContext_t *ctx, const uint8_t *key, SaSiAesOperationMode_t mode)
{
    int result;
    SaSiAesUserKeyData_t key_data;
    uint8_t key_buffer[SASI_AES_BLOCK_SIZE_IN_BYTES];

    (void)result; // to avoid compiler unused variable error in release builds

    // SaSi_AesInit() and SaSi_AesSetKey() only modify the SaSiAesUserContext_t and
    // don't modify any HW registers, so no need to call cryptocell_enable().
    result = SaSi_AesInit(ctx, SASI_AES_ENCRYPT, mode, SASI_AES_PADDING_NONE);
    assert(result == SA_SILIB_RET_OK);

    // key must be in RAM. since we're not sure key is RAM or
    // FLASH and it's pretty small, just copy it to a stack buffer to be sure
    memcpy(key_buffer, key, sizeof(key_buffer));

    key_data.pKey = key_buffer;
    key_data.keySize = sizeof(key_buffer);

    result = SaSi_AesSetKey(ctx, SASI_AES_USER_KEY, &key_data, sizeof(key_data));
    assert(result == SA_SILIB_RET_OK);

    // Zero out the stack data to avoid leaking anything
    memset(key_buffer, 0, sizeof(key_buffer));
}

void nlplatform_AES128ECB_encrypt(const uint8_t *inBlock, uint8_t *outBlock, const uint8_t *key)
{
    int result;
    SaSiAesUserContext_t context;
    uint8_t in_block_buf[SASI_AES_BLOCK_SIZE_IN_BYTES];
    uint8_t *in_buf;

    (void)result; // to avoid compiler unused variable error in release builds

    aes_init(&context, key, SASI_AES_MODE_ECB);

    // if inBlock is not in DataRAM, copy to a stack buffer because CryptoCell can
    // only operate on DataRAM addresses
    if (nlplatform_in_data_ram(inBlock, SASI_AES_BLOCK_SIZE_IN_BYTES))
    {
        in_buf = (uint8_t *)inBlock;
    }
    else
    {
        memcpy(in_block_buf, inBlock, sizeof(in_block_buf));
        in_buf = in_block_buf;
    }

    cryptocell_enable();
    result = SaSi_AesBlock(&context, in_buf, SASI_AES_BLOCK_SIZE_IN_BYTES, outBlock);
    cryptocell_disable();
    assert(result == SA_SILIB_RET_OK);

    // Zero out the stack data to avoid leaking anything
    memset(in_block_buf, 0, sizeof(in_block_buf));
}

void nlplatform_AES_CMAC_init(nlplatform_aes_cmac_t *ctx, const uint8_t *key)
{
    nlplatform_aes_cmac_internal_t *cmac_ctx = (nlplatform_aes_cmac_internal_t *)ctx;
    aes_init(&cmac_ctx->aes_ctx, key, SASI_AES_MODE_CMAC);
    cmac_ctx->last_block_len = 0;
    return;
}

void nlplatform_AES_CMAC_update(nlplatform_aes_cmac_t *ctx, const uint8_t *inData, size_t dataLen)
{
    nlplatform_aes_cmac_internal_t *cmac_ctx = (nlplatform_aes_cmac_internal_t *)ctx;
    int result;
    uint8_t *tmp_buf;

    (void)result; // to avoid compiler unused variable error in release builds

    if (!nlplatform_in_data_ram(inData, dataLen))
    {
        // need to always use RAM buffer
        tmp_buf = cmac_ctx->last_block;
    }
    else
    {
        tmp_buf = NULL;
    }

    if (dataLen == 0)
    {
        goto out;
    }

    // copy into last_block buffer if it wasn't full
    if (cmac_ctx->last_block_len)
    {
        size_t bytes_to_copy = ARRAY_SIZE(cmac_ctx->last_block) - cmac_ctx->last_block_len;
        if (bytes_to_copy > dataLen)
        {
            bytes_to_copy = dataLen;
        }
        memcpy(cmac_ctx->last_block + cmac_ctx->last_block_len, inData, bytes_to_copy);
        dataLen -= bytes_to_copy;
        cmac_ctx->last_block_len += bytes_to_copy;
        if (dataLen == 0)
        {
            goto out;
        }
        inData += bytes_to_copy;

        cryptocell_enable();
        result = SaSi_AesBlock(&cmac_ctx->aes_ctx, cmac_ctx->last_block, SASI_AES_BLOCK_SIZE_IN_BYTES, NULL);
        assert(result == SA_SILIB_RET_OK);
    }
    else
    {
        cryptocell_enable();
    }
    // Encrypt full blocks except last one
    while (dataLen > SASI_AES_BLOCK_SIZE_IN_BYTES)
    {
        if (tmp_buf == cmac_ctx->last_block)
        {
            // have to use last_block for all transfers because inData is not in DataRAM
            memcpy(tmp_buf, inData, SASI_AES_BLOCK_SIZE_IN_BYTES);
        }
        else
        {
            tmp_buf = (uint8_t *)inData;
        }
        result = SaSi_AesBlock(&cmac_ctx->aes_ctx, tmp_buf, SASI_AES_BLOCK_SIZE_IN_BYTES, NULL);
        assert(result == SA_SILIB_RET_OK);
        inData += SASI_AES_BLOCK_SIZE_IN_BYTES;
        dataLen -= SASI_AES_BLOCK_SIZE_IN_BYTES;
    }
    cryptocell_disable();
    // Copy any leftover data to lastBlock buffer
    memcpy(cmac_ctx->last_block, inData, dataLen);
    cmac_ctx->last_block_len = dataLen;
out:
    return;
}

void nlplatform_AES_CMAC_finish(nlplatform_aes_cmac_t *ctx, uint8_t *macBuf)
{
    int result;
    nlplatform_aes_cmac_internal_t *cmac_ctx = (nlplatform_aes_cmac_internal_t *)ctx;
    size_t mac_size = SASI_AES_IV_SIZE_IN_BYTES;

    (void)result; // to avoid compiler unused variable error in release builds
    cryptocell_enable();
    result = SaSi_AesFinish(&cmac_ctx->aes_ctx,
                            cmac_ctx->last_block_len,
                            cmac_ctx->last_block,
                            cmac_ctx->last_block_len,
                            macBuf,
                            &mac_size);
    cryptocell_disable();
    assert(result == SA_SILIB_RET_OK);
}

/*****************************************************************************/
/* SHA                                                                       */
/*****************************************************************************/

#ifdef NL_BOOTLOADER
static void hash_update(nrf_cc310_bl_hash_context_sha256_t *ctx, const uint8_t *data, size_t len)
#else
static void hash_update(CRYS_HASHUserContext_t *ctx, const uint8_t *data, size_t len)
#endif
{
#ifndef STATIC_HASH_RAM_BUFFER_SIZE
    uint8_t temp_buf[HASH_RAM_BUFFER_SIZE];
#endif
    uint8_t *hash_buf;
    const uint8_t *src_buf;
    size_t bytes_to_hash;
    size_t max_hash_size;
    CRYSError_t result;
    (void)result; // to avoid compiler unused variable error in release builds

    cryptocell_enable();

    if (nlplatform_in_data_ram(data, len))
    {
        max_hash_size = MAX_HASH_CHUNK_SIZE;
        src_buf = NULL;
    }
    else
    {
        // copy data to a temp_buf in RAM and pass that to CRYS_HASH_Update()
        max_hash_size = HASH_RAM_BUFFER_SIZE;
        src_buf = data;
        // set hash_buf to temp_buf, which won't change in loop below
        hash_buf = temp_buf;
    }
    bytes_to_hash = max_hash_size;
    while (len)
    {
        if (len < max_hash_size)
        {
            bytes_to_hash = len;
        }

        if (src_buf)
        {
            memcpy(hash_buf, src_buf, bytes_to_hash);
            src_buf += bytes_to_hash;
        }
        else
        {
            // CRYS_HASH_Update() takes a non-const data ptr.
            // Since we already checked that data is a pointer into
            // DataRAM, it's safe to cast the const away
            hash_buf = (uint8_t *)data;
        }
#ifdef NL_BOOTLOADER
        result = nrf_cc310_bl_hash_sha256_update(ctx, hash_buf, bytes_to_hash);
#else
        result = CRYS_HASH_Update(ctx, hash_buf, bytes_to_hash);
#endif
        assert(result == CRYS_OK);
        len -= bytes_to_hash;
        data += bytes_to_hash;
    }
    cryptocell_disable();
}

#ifdef NL_BOOTLOADER
static void hash_finish(nrf_cc310_bl_hash_context_sha256_t *ctx, uint8_t *digest)
#else
static void hash_finish(CRYS_HASHUserContext_t *ctx, uint8_t *digest, size_t digest_len)
#endif
{
#ifdef NL_BOOTLOADER
    cryptocell_enable();
    nrf_cc310_bl_hash_sha256_finalize(ctx, (nrf_cc310_bl_hash_digest_sha256_t*)digest);
    cryptocell_disable();
#else
    /* CRYS_HASH_Finish() insists on copying back 64 bytes of digest,
     * even if the requested hash mode would result in a smaller
     * one. To avoid memory corruption, we allocate a temp buffer
     * on the stack of the right size, pass that to CRYS_HASH_Finish(),
     * then copy out from that to the user provided buffer.
     */
    CRYSError_t result;
    CRYS_HASH_Result_t result_buf;

    (void)result; // to avoid compiler unused variable error in release builds

    /* There may be a partial block that wasn't processed previously
     * and will be now, so we still need to enable the crytpocell hw
     */
    cryptocell_enable();
    result = CRYS_HASH_Finish(ctx, result_buf);
    cryptocell_disable();

    assert(result == CRYS_OK);

    memcpy(digest, &result_buf, digest_len);
#endif
}

#ifdef NL_BOOTLOADER
void nlplatform_SHA256_init(nlplatform_sha256_t *ctx)
{
    CRYSError_t result = nrf_cc310_bl_hash_sha256_init((nrf_cc310_bl_hash_context_sha256_t*)ctx);
    (void)result; // to avoid compiler unused variable error in release builds
    assert(result == CRYS_OK);
}

void nlplatform_SHA256_update(nlplatform_sha256_t *ctx, const uint8_t *data, size_t len)
{
    hash_update((nrf_cc310_bl_hash_context_sha256_t*)ctx, data, len);
}

void nlplatform_SHA256_finish(nlplatform_sha256_t *ctx, uint8_t *digest)
{
    hash_finish((nrf_cc310_bl_hash_context_sha256_t*)ctx, digest);
}

void nlplatform_SHA256_hash(const uint8_t *data, uint8_t *digest, size_t len)
{
    nrf_cc310_bl_hash_context_sha256_t ctx;
    nlplatform_SHA256_init((nlplatform_sha256_t *)&ctx);
    hash_update(&ctx, data, len);
    hash_finish(&ctx, digest);
}

#else /* NL_BOOTLOADER */
void nlplatform_SHA1_init(nlplatform_sha1_t *ctx)
{
    CRYSError_t result = CRYS_HASH_Init((CRYS_HASHUserContext_t*)ctx, CRYS_HASH_SHA1_mode);
    (void)result; // to avoid compiler unused variable error in release builds
    assert(result == CRYS_OK);
}

void nlplatform_SHA1_update(nlplatform_sha1_t *ctx, const uint8_t *data, size_t len)
{
    hash_update((CRYS_HASHUserContext_t*)ctx, data, len);
}

void nlplatform_SHA1_finish(nlplatform_sha1_t *ctx, uint8_t *digest)
{
    hash_finish((CRYS_HASHUserContext_t*)ctx, digest, CRYS_HASH_SHA1_DIGEST_SIZE_IN_BYTES);
}

void nlplatform_SHA1_hash(const uint8_t *data, uint8_t *digest, size_t len)
{
    CRYS_HASHUserContext_t ctx;
    nlplatform_SHA1_init((nlplatform_sha1_t *)&ctx);
    hash_update(&ctx, data, len);
    hash_finish(&ctx, digest, CRYS_HASH_SHA1_DIGEST_SIZE_IN_BYTES);
}

void nlplatform_SHA256_init(nlplatform_sha256_t *ctx)
{
    CRYSError_t result = CRYS_HASH_Init((CRYS_HASHUserContext_t*)ctx, CRYS_HASH_SHA256_mode);
    (void)result; // to avoid compiler unused variable error in release builds
    assert(result == CRYS_OK);
}

void nlplatform_SHA256_update(nlplatform_sha256_t *ctx, const uint8_t *data, size_t len)
{
    hash_update((CRYS_HASHUserContext_t*)ctx, data, len);
}

void nlplatform_SHA256_finish(nlplatform_sha256_t *ctx, uint8_t *digest)
{
    hash_finish((CRYS_HASHUserContext_t*)ctx, digest, CRYS_HASH_SHA256_DIGEST_SIZE_IN_BYTES);
}

void nlplatform_SHA256_hash(const uint8_t *data, uint8_t *digest, size_t len)
{
    CRYS_HASHUserContext_t ctx;
    nlplatform_SHA256_init((nlplatform_sha256_t *)&ctx);
    hash_update(&ctx, data, len);
    hash_finish(&ctx, digest, CRYS_HASH_SHA256_DIGEST_SIZE_IN_BYTES);
}

#endif /* #ifdef NL_BOOTLOADER */


/*****************************************************************************/
/* ECDSA                                                                     */
/*****************************************************************************/

#ifndef NL_BOOTLOADER
/* These are internal routines in CrytpoCell for now so no header definitions
 * were provided. We declare them ourselves.
 */
extern CRYSError_t CRYS_ECDSA_VerifyInit(CRYS_ECDSA_VerifyUserContext_t  *pVerifyUserContext,
                                         CRYS_ECPKI_UserPublKey_t        *pSignerPublKey,
                                         CRYS_ECPKI_HASH_OpMode_t         hashMode);
extern CRYSError_t CRYS_ECDSA_VerifyUpdate(CRYS_ECDSA_VerifyUserContext_t *pVerifyUserContext,
                                           uint8_t                        *pMessageDataIn,
                                           uint32_t                        dataInSize);
extern CRYSError_t CRYS_ECDSA_VerifyFinish(CRYS_ECDSA_VerifyUserContext_t  *pVerifyUserContext,
                                           uint8_t                         *pSignatureIn,
                                           uint32_t                         SignatureSizeBytes);

/* Key/signature length for SECP224R1 */
#define SECP224R1_MODULUS_SIZE 56

/* To reduce peak stack usage, use helper routines
 */
static void ecdsa_init(CRYS_ECDSA_VerifyUserContext_t *ctx, const uint8_t *public_key) __attribute__((noinline));
static void ecdsa_init(CRYS_ECDSA_VerifyUserContext_t *ctx, const uint8_t *public_key)
{
    int result;
    CRYS_ECPKI_UserPublKey_t user_public_key;
    uint8_t cryptocell_public_key[SECP224R1_MODULUS_SIZE + 1]; // +1 for point control

    (void)result; // to avoid compiler unused variable error in release builds

    // Cryptocell wants public key with a one byte point control
    // header, so we have to convert
    cryptocell_public_key[0] = CRYS_EC_PointUncompressed;
    memcpy(cryptocell_public_key + 1, public_key, SECP224R1_MODULUS_SIZE);

    // have to convert the raw key to the format CryptoCell wants.
    CRYS_ECPKI_BuildPublKey(CRYS_ECPKI_GetEcDomain(CRYS_ECPKI_DomainID_secp224r1),
                            cryptocell_public_key, sizeof(cryptocell_public_key),
                            &user_public_key);

    result = CRYS_ECDSA_VerifyInit(ctx, &user_public_key, CRYS_ECPKI_HASH_SHA256_mode);
    assert(result == CRYS_OK);
}

static void ecdsa_hash_using_temp_buf(CRYS_ECDSA_VerifyUserContext_t *ctx, const uint8_t *message, size_t length) __attribute__((noinline));
static void ecdsa_hash_using_temp_buf(CRYS_ECDSA_VerifyUserContext_t *ctx, const uint8_t *message, size_t length)
{
    int result;
#ifndef STATIC_HASH_RAM_BUFFER_SIZE
    uint8_t temp_buf[HASH_RAM_BUFFER_SIZE];
#endif
    size_t bytes_to_hash = HASH_RAM_BUFFER_SIZE;

    (void)result; // to avoid compiler unused variable error in release builds

    while (length > 0)
    {
        if (length < HASH_RAM_BUFFER_SIZE)
        {
            bytes_to_hash = length;
        }
        memcpy(temp_buf, message, bytes_to_hash);
        result = CRYS_ECDSA_VerifyUpdate(ctx, temp_buf, bytes_to_hash);
        assert(result == CRYS_OK);
        length -= bytes_to_hash;
        message += bytes_to_hash;
    }
}
#endif

int nlplatform_ecdsa_verify(ecdsa_signature_t signatureType, const uint8_t *public_key, const uint8_t *message, size_t length, const uint8_t *signature)
{
    int result;
    int retval = ECDSA_VERIFY_SUCCESS;

    switch (signatureType) {
    case ECDSA_SIGNATURE_TYPE_SHA256_SECP224R1:
    {
#ifdef NL_BOOTLOADER
        nrf_cc310_bl_ecdsa_verify_context_secp224r1_t verify_context;
        nrf_cc310_bl_hash_digest_sha256_t hash_digest;

        nlplatform_SHA256_hash(message, (uint8_t*)&hash_digest, length);

        cryptocell_enable();

        result = nrf_cc310_bl_ecdsa_verify_init_secp224r1(&verify_context,
                                                          (nrf_cc310_bl_ecc_public_key_secp224r1_t*)public_key);
        if (result == SASI_SUCCESS)
        {
            result = nrf_cc310_bl_ecdsa_verify_hash_secp224r1(&verify_context,
                                                              (nrf_cc310_bl_ecc_signature_secp224r1_t*)signature,
                                                              (uint8_t*)&hash_digest,
                                                              sizeof(hash_digest));
        }
#else /* NL_BOOTLOADER */
        CRYS_ECDSA_VerifyUserContext_t verify_context;

        cryptocell_enable();

        ecdsa_init(&verify_context, public_key);

        // This is typically used for FLASH image verification.  We cannot pass
        // FLASH addresses directly to the CrytpoCell (can't DMA from FLASH) so
        // we have to hash in chunks that we copy form FLASH into a temp buffer.
        ecdsa_hash_using_temp_buf(&verify_context, message, length);

        // cast away const of signature because CRYS_ECDSA_VerifyFinish() didn't
        // declare it as a const ptr, but it doesn't modify it and doesn't
        // need to DMA it.
        result = CRYS_ECDSA_VerifyFinish(&verify_context,
                                         (uint8_t *)signature, SECP224R1_MODULUS_SIZE);
#endif  /* NL_BOOTLOADER */
        cryptocell_disable();
        if (result != 0)
        {
            retval = ECDSA_VERIFY_INVALID_SIGNATURE;
        }
        break;
    }
    case ECDSA_SIGNATURE_TYPE_NONE:
        retval = ECDSA_VERIFY_NO_SIGNATURE;
        // No signature present
        break;
    default:
        // Unhandled signature type
        retval = ECDSA_VERIFY_INVALID_SIGNATURE_TYPE;
        break;
    }

    return retval;
}

bool nlplatform_AES_available_in_isr(void)
{
    // It will always return false, since the AES provided by CC310 could not support
    // interrupt context usage for now (PLATFORM-468).
    return false;
}
