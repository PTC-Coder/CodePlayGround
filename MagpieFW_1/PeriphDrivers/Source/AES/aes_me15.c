/**
 * @file
 * @brief   Trust Protection Unit driver.
 */

/******************************************************************************
 *
 * Copyright (C) 2022-2023 Maxim Integrated Products, Inc. All Rights Reserved.
 * (now owned by Analog Devices, Inc.),
 * Copyright (C) 2023 Analog Devices, Inc. All Rights Reserved. This software
 * is proprietary to Analog Devices, Inc. and its licensors.
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *
 ******************************************************************************/

#include "mxc_device.h"
#include "mxc_errors.h"
#include "mxc_assert.h"
#include "mxc_sys.h"
#include "aes_revb.h"
#include "trng.h"
#include "flc.h"
#include "string.h"

#define KEY_ADDR 0x10802008
#define FMV_ADDR 0x10802000
static const uint32_t fmv[2] = { 0x2B86D479, 0x2B86D479 };

static void reverse_key(const void *key, uint8_t *keyr, int len)
{
    int i;
    uint8_t tmp;
    uint8_t *k = (uint8_t *)key;
    for (i = 0; i < len; i++) {
        tmp = k[i];
        k[i] = keyr[len - i - 1];
        keyr[len - i - 1] = tmp;
    }
}

/* ************************************************************************* */
/* Global Control/Configuration functions                                    */
/* ************************************************************************* */

int MXC_AES_Init(void)
{
#ifndef MSDK_NO_GPIO_CLK_INIT
    MXC_SYS_ClockEnable(MXC_SYS_PERIPH_CLOCK_AES);
    MXC_SYS_ClockEnable(MXC_SYS_PERIPH_CLOCK_TRNG);
#endif

    MXC_AES->ctrl = 0x00;
    // Start with a randomly generated key.
    MXC_AES_GenerateKey();

    MXC_AES_RevB_Init((mxc_aes_revb_regs_t *)MXC_AES);

    return E_NO_ERROR;
}

void MXC_AES_EnableInt(uint32_t interrupt)
{
    MXC_AES_RevB_EnableInt((mxc_aes_revb_regs_t *)MXC_AES, interrupt);
}

void MXC_AES_DisableInt(uint32_t interrupt)
{
    MXC_AES_RevB_DisableInt((mxc_aes_revb_regs_t *)MXC_AES, interrupt);
}

int MXC_AES_IsBusy(void)
{
    return MXC_AES_RevB_IsBusy((mxc_aes_revb_regs_t *)MXC_AES);
}

int MXC_AES_Shutdown(void)
{
    int error = MXC_AES_RevB_Shutdown((mxc_aes_revb_regs_t *)MXC_AES);

    MXC_SYS_ClockDisable(MXC_SYS_PERIPH_CLOCK_AES);

    return error;
}

void MXC_AES_DMACallback(int ch, int error)
{
    MXC_AES_RevB_DMACallback(ch, error);
}

void MXC_AES_GenerateKey(void)
{
    // Generating a random key is part of the TRNG block
    MXC_TRNG_GenerateKey();
}

void MXC_AES_SetKeySize(mxc_aes_keys_t key)
{
    MXC_AES_RevB_SetKeySize((mxc_aes_revb_regs_t *)MXC_AES, (mxc_aes_revb_keys_t)key);
}

mxc_aes_keys_t MXC_AES_GetKeySize(void)
{
    return MXC_AES_RevB_GetKeySize((mxc_aes_revb_regs_t *)MXC_AES);
}

void MXC_AES_FlushInputFIFO(void)
{
    MXC_AES_RevB_FlushInputFIFO((mxc_aes_revb_regs_t *)MXC_AES);
}

void MXC_AES_FlushOutputFIFO(void)
{
    MXC_AES_RevB_FlushOutputFIFO((mxc_aes_revb_regs_t *)MXC_AES);
}

void MXC_AES_Start(void)
{
    MXC_AES_RevB_Start((mxc_aes_revb_regs_t *)MXC_AES);
}

uint32_t MXC_AES_GetFlags(void)
{
    return MXC_AES_RevB_GetFlags((mxc_aes_revb_regs_t *)MXC_AES);
}

void MXC_AES_ClearFlags(uint32_t flags)
{
    MXC_AES_RevB_ClearFlags((mxc_aes_revb_regs_t *)MXC_AES, flags);
}

int MXC_AES_Generic(mxc_aes_req_t *req)
{
    return MXC_AES_RevB_Generic((mxc_aes_revb_regs_t *)MXC_AES, (mxc_aes_revb_req_t *)req);
}

int MXC_AES_Encrypt(mxc_aes_req_t *req)
{
    return MXC_AES_RevB_Encrypt((mxc_aes_revb_regs_t *)MXC_AES, (mxc_aes_revb_req_t *)req);
}

int MXC_AES_Decrypt(mxc_aes_req_t *req)
{
    return MXC_AES_RevB_Decrypt((mxc_aes_revb_regs_t *)MXC_AES, (mxc_aes_revb_req_t *)req);
}

int MXC_AES_TXDMAConfig(void *src_addr, int len)
{
    return MXC_AES_RevB_TXDMAConfig(src_addr, len);
}

int MXC_AES_RXDMAConfig(void *dest_addr, int len)
{
    return MXC_AES_RevB_RXDMAConfig(dest_addr, len);
}

int MXC_AES_GenericAsync(mxc_aes_req_t *req, uint8_t enc)
{
    return MXC_AES_RevB_GenericAsync((mxc_aes_revb_regs_t *)MXC_AES, (mxc_aes_revb_req_t *)req,
                                     enc);
}

int MXC_AES_EncryptAsync(mxc_aes_req_t *req)
{
    return MXC_AES_RevB_EncryptAsync((mxc_aes_revb_regs_t *)MXC_AES, (mxc_aes_revb_req_t *)req);
}

int MXC_AES_DecryptAsync(mxc_aes_req_t *req)
{
    return MXC_AES_RevB_DecryptAsync((mxc_aes_revb_regs_t *)MXC_AES, (mxc_aes_revb_req_t *)req);
}

void MXC_AES_SetExtKey(const void *key, mxc_aes_keys_t len)
{
    MXC_AES_RevB_SetExtKey((mxc_aeskeys_revb_regs_t *)MXC_AESKEYS, key, len);
}

int MXC_AES_SetPORKey(const void *key, mxc_aes_keys_t len)
{
    int err = E_BAD_PARAM;
    uint8_t keyr[32];

    // Make the key location readable/writable
    MXC_FLC_UnlockInfoBlock(KEY_ADDR);

    // Write the key
    switch (len) {
    case MXC_AES_128BITS:
        reverse_key(key, keyr, 16);
        err = MXC_FLC_Write(KEY_ADDR, 16, (uint32_t *)keyr);
        break;
    case MXC_AES_192BITS:
        reverse_key(key, keyr, 24);
        err = MXC_FLC_Write(KEY_ADDR, 24, (uint32_t *)keyr);
        break;
    case MXC_AES_256BITS:
        reverse_key(key, keyr, 32);
        err = MXC_FLC_Write(KEY_ADDR, 32, (uint32_t *)keyr);
        break;
    }

    if (err == E_NO_ERROR) {
        // Write the magic value to activate the key
        err = MXC_FLC_Write(FMV_ADDR, sizeof(fmv), (uint32_t *)fmv);

        // Lock the key region from reads/writes
        MXC_FLC_LockInfoBlock(KEY_ADDR);
        return err;
    }

    // Lock the key region from reads/writes
    MXC_FLC_LockInfoBlock(KEY_ADDR);
    return err;
}

int MXC_AES_ClearPORKey()
{
    int err;

    // The first 40 bytes of the page contain the FMV and AES key, no
    //  need to save those.
    uint8_t page[MXC_FLASH_PAGE_SIZE - 40];

    // Make the key location readable/writable
    MXC_FLC_UnlockInfoBlock(FMV_ADDR);

    // Copy the current memory contents
    memcpy(page, (uint8_t *)FMV_ADDR + 40, MXC_FLASH_PAGE_SIZE - 40);

    err = MXC_FLC_PageErase(FMV_ADDR);
    if (err != E_NO_ERROR) {
        // Couldn't erase the memory.  Abort.
        // Lock the key region from reads/writes
        MXC_FLC_LockInfoBlock(FMV_ADDR);
        return err;
    }

    // Write the old contents (minus the fmv and key) back to the part
    err = MXC_FLC_Write(FMV_ADDR + 40, MXC_FLASH_PAGE_SIZE - 40, (uint32_t *)page);

    // Lock the key region from reads/writes
    MXC_FLC_LockInfoBlock(FMV_ADDR);

    return err;
}

void MXC_AES_CopyPORKeyToKeyRegisters(mxc_aes_keys_t len)
{
    // Make the key location readable/writable
    MXC_FLC_UnlockInfoBlock(KEY_ADDR);

    // Copy the values to the key register
    switch (len) {
    case MXC_AES_128BITS:
        memcpy(MXC_AESKEYS, (uint8_t *)KEY_ADDR, 16);
        break;
    case MXC_AES_192BITS:
        memcpy(MXC_AESKEYS, (uint8_t *)KEY_ADDR, 24);
        break;
    case MXC_AES_256BITS:
        memcpy(MXC_AESKEYS, (uint8_t *)KEY_ADDR, 32);
        break;
    }

    // Lock the key region from reads/writes
    MXC_FLC_LockInfoBlock(KEY_ADDR);
}

int MXC_AES_HasPORKey()
{
    int res;

    // Make the key location readable/writable
    MXC_FLC_UnlockInfoBlock(FMV_ADDR);

    // Look for the magic value.
    res = memcmp((uint8_t *)FMV_ADDR, (uint8_t *)fmv, 8);

    // Lock the key region from reads/writes
    MXC_FLC_LockInfoBlock(FMV_ADDR);

    return !res;
}
