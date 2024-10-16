/**
 * @file flc_me15.c
 * @brief      Flash Controler driver.
 * @details    This driver can be used to operate on the embedded flash memory.
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

/* **** Includes **** */
#include <string.h>
#include "mxc_device.h"
#include "mxc_assert.h"
#include "mxc_sys.h"
#include "flc.h"
#include "flc_reva.h"
#include "flc_common.h"
#include "ecc_regs.h" // For ECCEN registers.
#include "mcr_regs.h" // For ECCEN registers.

//******************************************************************************
void MXC_FLC_ME15_Flash_Operation(void)
{
    /* Flush all instruction caches */
    MXC_GCR->sysctrl |= MXC_F_GCR_SYSCTRL_ICC0_FLUSH;

    /* Wait for flush to complete */
    while (MXC_GCR->sysctrl & MXC_F_GCR_SYSCTRL_ICC0_FLUSH) {}
}

//******************************************************************************
int MXC_FLC_ME15_GetByAddress(mxc_flc_regs_t **flc, uint32_t addr)
{
    if ((addr >= MXC_FLASH_MEM_BASE) && (addr < (MXC_FLASH_MEM_BASE + MXC_FLASH_MEM_SIZE))) {
        *flc = MXC_FLC0;
    } else if ((addr >= MXC_INFO_MEM_BASE) && (addr < (MXC_INFO_MEM_BASE + MXC_INFO_MEM_SIZE))) {
        *flc = MXC_FLC0;
    } else {
        return E_BAD_PARAM;
    }

    return E_NO_ERROR;
}

//******************************************************************************
int MXC_FLC_ME15_GetPhysicalAddress(uint32_t addr, uint32_t *result)
{
    if ((addr >= MXC_FLASH_MEM_BASE) && (addr < (MXC_FLASH_MEM_BASE + MXC_FLASH_MEM_SIZE))) {
        *result = addr - MXC_FLASH_MEM_BASE;
    } else if ((addr >= MXC_INFO_MEM_BASE) && (addr < (MXC_INFO_MEM_BASE + MXC_INFO_MEM_SIZE))) {
        /* For ME15, the info block base was located at the next power of 2 address beyond the main flash.
           The ME15 ends at 0x5FFFF, so the info block starts at 0x80000. */
        *result = (addr & (MXC_INFO_MEM_SIZE - 1)) + 0x80000;
    } else {
        return E_BAD_PARAM;
    }

    return E_NO_ERROR;
}

//******************************************************************************
int MXC_FLC_Init()
{
    return E_NO_ERROR;
}

//******************************************************************************
#if IAR_PRAGMAS
#pragma section = ".flashprog"
#else
__attribute__((section(".flashprog")))
#endif
int MXC_FLC_Busy(void)
{
    return MXC_FLC_RevA_Busy();
}

//******************************************************************************
#if IAR_PRAGMAS
#pragma section = ".flashprog"
#else
__attribute__((section(".flashprog")))
#endif
int MXC_FLC_PageErase(uint32_t address)
{
    int err;
    uint32_t addr;
    mxc_flc_regs_t *flc = NULL;

    // Get FLC Instance
    if ((err = MXC_FLC_ME15_GetByAddress(&flc, address)) != E_NO_ERROR) {
        return err;
    }

    if ((err = MXC_FLC_ME15_GetPhysicalAddress(address, &addr)) < E_NO_ERROR) {
        return err;
    }

    err = MXC_FLC_RevA_PageErase((mxc_flc_reva_regs_t *)flc, addr);
    // Flush the cache
    MXC_FLC_ME15_Flash_Operation();

    return err;
}

//******************************************************************************
#if IAR_PRAGMAS
#pragma section = ".flashprog"
#else
__attribute__((section(".flashprog")))
#endif
// make sure to disable ICC with ICC_Disable(); before Running this function
int MXC_FLC_Write128(uint32_t address, uint32_t *data)
{
    int err;
    mxc_flc_regs_t *flc = NULL;
    uint32_t addr;

    // Address checked if it is 128-bit aligned
    if (address & 0xF) {
        return E_BAD_PARAM;
    }

    // Get FLC Instance
    if ((err = MXC_FLC_ME15_GetByAddress(&flc, address)) != E_NO_ERROR) {
        return err;
    }

    if ((err = MXC_FLC_ME15_GetPhysicalAddress(address, &addr)) < E_NO_ERROR) {
        return err;
    }

    if ((err = MXC_FLC_RevA_Write128((mxc_flc_reva_regs_t *)flc, addr, data)) != E_NO_ERROR) {
        return err;
    }

    // Flush the cache
    MXC_FLC_ME15_Flash_Operation();

    if ((err = MXC_FLC_Com_VerifyData(address, 4, data)) != E_NO_ERROR) {
        return err;
    }

    return E_NO_ERROR;
}

//******************************************************************************
int MXC_FLC_Write32(uint32_t address, uint32_t data)
{
    uint32_t addr, aligned;
    int err;
    mxc_flc_regs_t *flc = NULL;

    // Address checked if it is byte addressable
    if (address & 0x3) {
        return E_BAD_PARAM;
    }

    // Align address to 128-bit word
    aligned = address & 0xfffffff0;

    // Get FLC Instance
    if ((err = MXC_FLC_ME15_GetByAddress(&flc, address)) != E_NO_ERROR) {
        return err;
    }

    if ((err = MXC_FLC_ME15_GetPhysicalAddress(aligned, &addr)) < E_NO_ERROR) {
        return err;
    }

    if (MXC_ECC->en & MXC_F_ECC_EN_FLASH) {
        return E_BAD_STATE;
    }

    return MXC_FLC_RevA_Write32Using128((mxc_flc_reva_regs_t *)flc, address, data, addr);
}

//******************************************************************************
int MXC_FLC_MassErase(void)
{
    int err, i;
    mxc_flc_regs_t *flc;

    for (i = 0; i < MXC_FLC_INSTANCES; i++) {
        flc = MXC_FLC_GET_FLC(i);
        err = MXC_FLC_RevA_MassErase((mxc_flc_reva_regs_t *)flc);

        if (err != E_NO_ERROR) {
            return err;
        }

        MXC_FLC_ME15_Flash_Operation();
    }

    return E_NO_ERROR;
}

//******************************************************************************
int MXC_FLC_UnlockInfoBlock(uint32_t address)
{
    int err;
    mxc_flc_regs_t *flc;

    if ((err = MXC_FLC_ME15_GetByAddress(&flc, address)) != E_NO_ERROR) {
        return err;
    }

    return MXC_FLC_RevA_UnlockInfoBlock((mxc_flc_reva_regs_t *)flc, address);
}

//******************************************************************************
int MXC_FLC_LockInfoBlock(uint32_t address)
{
    int err;
    mxc_flc_regs_t *flc;

    if ((err = MXC_FLC_ME15_GetByAddress(&flc, address)) != E_NO_ERROR) {
        return err;
    }

    return MXC_FLC_RevA_LockInfoBlock((mxc_flc_reva_regs_t *)flc, address);
}

//******************************************************************************
int MXC_FLC_Write(uint32_t address, uint32_t length, uint32_t *buffer)
{
    return MXC_FLC_Com_Write(address, length, buffer);
}

//******************************************************************************
void MXC_FLC_Read(int address, void *buffer, int len)
{
    MXC_FLC_Com_Read(address, buffer, len);
}

//******************************************************************************
int MXC_FLC_EnableInt(uint32_t flags)
{
    return MXC_FLC_RevA_EnableInt(flags);
}

//******************************************************************************
int MXC_FLC_DisableInt(uint32_t flags)
{
    return MXC_FLC_RevA_DisableInt(flags);
}

//******************************************************************************
int MXC_FLC_GetFlags(void)
{
    return MXC_FLC_RevA_GetFlags();
}

//******************************************************************************
int MXC_FLC_ClearFlags(uint32_t flags)
{
    return MXC_FLC_RevA_ClearFlags(flags);
}

//******************************************************************************
int MXC_FLC_BlockPageWrite(uint32_t address)
{
    if (address < MXC_FLASH_MEM_BASE || address > (MXC_FLASH_MEM_BASE + MXC_FLASH_MEM_SIZE)) {
        return E_INVALID;
    }

    return MXC_FLC_RevA_BlockPageWrite(address, MXC_FLASH_MEM_BASE);
}

//******************************************************************************
int MXC_FLC_BlockPageRead(uint32_t address)
{
    if (address < MXC_FLASH_MEM_BASE || address > (MXC_FLASH_MEM_BASE + MXC_FLASH_MEM_SIZE)) {
        return E_INVALID;
    }

    return MXC_FLC_RevA_BlockPageRead(address, MXC_FLASH_MEM_BASE);
}

//******************************************************************************
volatile uint32_t *MXC_FLC_GetWELR(uint32_t address, uint32_t page_num)
{
    uint32_t reg_num;
    reg_num = page_num >>
              5; // Divide by 32 to get WELR register number containing the page lock bit

    if (address < MXC_FLASH_MEM_BASE || address > (MXC_FLASH_MEM_BASE + MXC_FLASH_MEM_SIZE)) {
        return NULL;
    }

    switch (reg_num) {
    case 0:
        return &(MXC_FLC0->welr0);
    case 1:
        return &(MXC_FLC0->welr1);
    }

    return NULL;
}

//******************************************************************************
volatile uint32_t *MXC_FLC_GetRLR(uint32_t address, uint32_t page_num)
{
    uint32_t reg_num;
    reg_num = page_num >> 5; // Divide by 32 to get RLR register number containing the page lock bit

    if (address < MXC_FLASH_MEM_BASE || address > (MXC_FLASH_MEM_BASE + MXC_FLASH_MEM_SIZE)) {
        return NULL;
    }

    switch (reg_num) {
    case 0:
        return &(MXC_FLC0->rlr0);
    case 1:
        return &(MXC_FLC0->rlr1);
    }

    return NULL;
}
