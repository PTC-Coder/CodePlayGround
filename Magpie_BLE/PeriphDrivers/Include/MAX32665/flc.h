/**
 * @file       flc.h
 * @brief      Flash Controller driver.
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

#ifndef LIBRARIES_PERIPHDRIVERS_INCLUDE_MAX32665_FLC_H_
#define LIBRARIES_PERIPHDRIVERS_INCLUDE_MAX32665_FLC_H_

/* **** Includes **** */
#include "flc_regs.h"
#include "mxc_sys.h"
#include "mxc_errors.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @defgroup flc Flash Controller  (FLC)
 * @ingroup periphlibs
 * @{
 */

/***** Definitions *****/

/// Bit mask that can be used to find the starting address of a page in flash
#define MXC_FLASH_PAGE_MASK ~(MXC_FLASH_PAGE_SIZE - 1)

/// Calculate the address of a page in flash from the page number
#define MXC_FLASH_PAGE_ADDR(page) (MXC_FLASH_MEM_BASE + ((uint32_t)page * MXC_FLASH_PAGE_SIZE))

/***** Function Prototypes *****/

/**
 * @brief      Initializes the Flash Controller for erase/write operations
 * @return     #E_NO_ERROR if successful.
 */
int MXC_FLC_Init();

/**
 * @brief      Checks if Flash Controller is busy.
 * @details    Reading or executing from flash is not possible if flash is busy
 *             with an erase or write operation.
 * @return     If non-zero, flash operation is in progress
 */
int MXC_FLC_Busy(void);

/**
 * @brief      Erases the entire flash array.
 * @note       This function must be executed from RAM.
 * @return     #E_NO_ERROR If function is successful.
 */
int MXC_FLC_MassErase(void);

/**
 * @brief      Erases the page of flash at the specified address.
 * @note       This function must be executed from RAM.
 * @param      address  Any address within the page to erase.
 * @return     #E_NO_ERROR If function is successful.
 */
int MXC_FLC_PageErase(uint32_t address);

/**
 * @brief      Read Data out of Flash from an address
 *
 * @param[in]  address  The address to read from
 * @param      buffer   The buffer to read the data into
 * @param[in]  len      The length of the buffer
 *
 */
void MXC_FLC_Read(int address, void *buffer, int len);

/**
 * @brief      Writes data to flash.
 * @note       This function must be executed from RAM.
 * @param      address  Address in flash to start writing from.
 * @param      length   Number of bytes to be written.
 * @param      buffer     Pointer to data to be written to flash.
 * @return     #E_NO_ERROR If function is successful.
 * @note       make sure to disable ICC with ICC_Disable(); before Running this function
 */
int MXC_FLC_Write(uint32_t address, uint32_t length, uint32_t *buffer);

/**
 * @brief      Writes 32 bits of data to flash.
 * @note       This function must be executed from RAM.
 * @param      address  Address in flash to start writing from.
 * @param      data     Pointer to data to be written to flash.
 * @return     #E_NO_ERROR If function is successful.
 * @note       make sure to disable ICC with ICC_Disable(); before Running this function
 */
int MXC_FLC_Write32(uint32_t address, uint32_t data);

/**
 * @brief      Writes 128 bits of data to flash.
 * @note       This function must be executed from RAM.
 * @param      address  Address in flash to start writing from.
 * @param      data     Pointer to data to be written to flash.
 * @return     #E_NO_ERROR If function is successful.
 * @note       make sure to disable ICC with ICC_Disable(); before Running this function
 */
int MXC_FLC_Write128(uint32_t address, uint32_t *data);

/**
 * @brief      Set FLC Instance used for Interrupts functions.
 * @note       Default is set to MXC_FLC0 if you don't run this.
 * @param      flc   Pointer to FLC instance.
 */
void MXC_FLC_SetFLCInt(mxc_flc_regs_t *flc);

/**
 * @brief      Receive the currently set FLC instances used for Interrupts.
 * @return     Pointer to set FLC instance.
 */
mxc_flc_regs_t *MXC_FLC_GetFLCInt(void);

/**
 * @brief      Enable flash interrupts
 * @param      flags   Interrupts to enable
 * @return     #E_NO_ERROR If function is successful.
 */
int MXC_FLC_EnableInt(uint32_t flags);

/**
 * @brief      Disable flash interrupts
 * @param      flags   Interrupts to disable
 * @return     #E_NO_ERROR If function is successful.
 */
int MXC_FLC_DisableInt(uint32_t flags);

/**
 * @brief      Retrieve flash interrupt flags
 * @return     Interrupt flags registers
 */
int MXC_FLC_GetFlags(void);

/**
 * @brief      Clear flash interrupt flags
 * @note       Provide the bit position to clear, even if the flag is write-0-to-clear
 * @param      flags Flag bit(s) to clear
 * @return     #E_NO_ERROR If function is successful.
 */
int MXC_FLC_ClearFlags(uint32_t flags);

/**
 * @brief      Unlock info block
 *
 * @param[in]  address  The address in the info block needing written to
 *
 * @return     #E_NO_ERROR If function is successful.
 */
int MXC_FLC_UnlockInfoBlock(uint32_t address);

/**
 * @brief      Lock info block
 *
 * @param[in]  address  The address in the info block that was written to
 * @return     #E_NO_ERROR If function is successful.
 */
int MXC_FLC_LockInfoBlock(uint32_t address);

/**@} end of group flc */

#ifdef __cplusplus
}
#endif

#endif // LIBRARIES_PERIPHDRIVERS_INCLUDE_MAX32665_FLC_H_
