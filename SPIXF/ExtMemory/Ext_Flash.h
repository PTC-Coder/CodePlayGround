/**
 * @file
 * @brief BSP driver to communicate via SPI/QPI with an External Flash Memory.
 */

/******************************************************************************
 *
 * Copyright (C) 2022-2024 Maxim Integrated Products, Inc. All Rights Reserved.
 * (now owned by Analog Devices, Inc.),
 * Copyright (C) 2023-2024 Analog Devices, Inc. All Rights Reserved. This software
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

#ifndef EXTMEMORY_EXT_FLASH_H_
#define EXTMEMORY_EXT_FLASH_H_

/* **** Includes **** */
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @ingroup bsp
 * @defgroup Ext_Flash_driver Ext_Flash SPI Multi-I/O Flash Memory Driver
 * @{
 */

#define EXT_FLASH_NAME "W25N02KVxxIR"

#define EXT_FLASH_READ_DUMMY \
    8 /**< Dummy byte sent on a standard read command per the W25 datasheet.         */
#define EXT_FLASH_DREAD_DUMMY \
    4 /**< Dummy data sent on a fast-read (Dual) read command per the W25 datasheet. */
#define EXT_FLASH_QREAD_DUMMY \
    6 /**< Dummy data sent on a fast-read (Quad) read command per the W25 datasheet. */

#define EXT_FLASH_EXP_ID 0xEFAA22

#define EXT_FLASH_CMD_READ 0x0B /**< Read                           */
#define EXT_FLASH_CMD_DREAD 0xBB /**< Dual SPI Read                  */
#define EXT_FLASH_CMD_QREAD 0xEB /**< Quad SPI Read                  */


#ifdef EXT_FLASH_READ_DUMMY
// Deprecated name. Please use EXT_FLASH_READ_DUMMY
#define EXT_FLASH_Read_DUMMY EXT_FLASH_READ_DUMMY
#endif


/**
 * Enumeration type to select the size for an Erase command.
 */
typedef enum {
    EF_E_SUCCESS = 0,
    EF_E_ERROR = -1,
    EF_E_BUSY = -99,
    EF_E_BAD_STATE = -98,
    EF_E_TIME_OUT = -97,
    EF_E_BAD_PARAM = -96,
    EF_E_ERASE_FAILED = -95,
} Ext_Flash_Error_t;

/**
 * Enumeration type to select the size for an Erase command.
 */
typedef enum {
    Ext_Flash_Erase_4K, /**< 4KB Sector Erase  */
    Ext_Flash_Erase_32K, /**< 32KB Block Erase */
    Ext_Flash_Erase_64K, /**< 64KB Block Erase */
    Ext_Flash_Erase_128K, /**< 128KB Block Erase */
} Ext_Flash_Erase_t;

/**
 * Enumeration type to select status register.
 */
typedef enum {
    Ext_Flash_StatusReg_1, /**< Status Register 1 */
    Ext_Flash_StatusReg_2, /**< Status Register 2 */
    Ext_Flash_StatusReg_3, /**< Status Register 3 */
} Ext_Flash_StatusReg_t;

/**
 * Enumeration type to specify data width.
 */
typedef enum {
    Ext_Flash_DataLine_Single, /**< 1 Data Line.  */
    Ext_Flash_DataLine_Dual, /**< 2 Data Lines (x2). */
    Ext_Flash_DataLine_Quad /**< 4 Data Lines (x4). */
} Ext_Flash_DataLine_t;

/**
 * Struct used to hold the start and end addresses of flash currently available to write
 */
typedef struct {
    uint32_t start_addr; /**< Start address of flash available to write */
    uint32_t end_addr; /**< End address of flash available to write */
} Ext_Flash_Unblk_t;

/**
 * Struct definition to configure physical communication layer.
 */
typedef struct {
    int (*init)(void);
    int (*read)(uint8_t *read, unsigned len, unsigned deassert, Ext_Flash_DataLine_t d_line);
    int (*write)(const uint8_t *write, unsigned len, unsigned deassert,
                 Ext_Flash_DataLine_t d_line);
    int (*clock)(unsigned len, unsigned deassert);
} Ext_Flash_Config_t;

/* *** Globals **** */

/* **** Function Prototypes **** */

/**
 * @brief      Configure Ext_Flash communication line
 * @param      cfg    Configuration structure
 * @retval     0         Success
 * @retval     Non-zero  Error condition
 */
Ext_Flash_Error_t Ext_Flash_Configure(Ext_Flash_Config_t *cfg);

/**
 * @brief      Initialize SPI configuration and reset Ext_Flash
 * @retval     0         Success
 * @retval     Non-zero  Error condition
 */
Ext_Flash_Error_t Ext_Flash_Init(void);

/**
 * @brief       Reset the Ext_Flash flash memory.
 * @retval      0         Success
 * @retval      Non-zero  Error condition
 */
Ext_Flash_Error_t Ext_Flash_Reset(void);

/**
 * @brief       Read manufacturer ID.
 * @retval      ID of the device, or 0 if an error occurred
 */
uint32_t Ext_Flash_ID(void);

/**
 * @brief       Enable/Disable the Quad Enable(QE) bit in the status register.
 * @param       enable    @arg @b 1 enables Quad Mode. @arg @b 0 disables Quad Mode.
 * @retval      0         Success
 * @retval      Non-zero  Error condition
 */
Ext_Flash_Error_t Ext_Flash_Quad(int enable);

/**
 * @brief       Read data out from the Flash Memory to Flash buffer
 * @param       address         Start address to read from
 * @retval      0         Success
 * @retval      Non-zero  Error condition
 */
Ext_Flash_Error_t Ext_Flash_DataRead(uint32_t address);


/**
 * @brief       Read data out from the Flash data buffer by using 4-wire SPI mode.
 * @param       address         Start address to read from
 * @param       rx_buf          Pointer to the buffer of receiving data
 * @param       rx_len          Size of the data to read
 * @param       d_line          #Ext_Flash_DataLine_t for how many data lines to use
 * @retval      0         Success
 * @retval      Non-zero  Error condition
 */
Ext_Flash_Error_t Ext_Flash_Read(uint32_t address, uint8_t *rx_buf, uint32_t rx_len, Ext_Flash_DataLine_t d_line);

/**
 * @brief       Program the memory to @p tx_buf and length @p tx_len, applies to both SPI and QPI modes.
 * @details
 *        - SPI mode: All operations are in 4-wire SPI mode.
 *        - QPI mode: All operations are in quad SPI mode.
 * @param       address         Start address to program.
 * @param       tx_buf          Pointer to the buffer of data to write.
 * @param       tx_len          Size of the data to write.
 * @param       d_line          #Ext_Flash_DataLine_t for how many data lines to use.
 * @retval      0         Success
 * @retval      Non-zero  Error condition
 */
Ext_Flash_Error_t Ext_Flash_Program_Page(uint32_t address, uint8_t *tx_buf, uint32_t tx_len,
                           Ext_Flash_DataLine_t d_line);


/**
 * @brief       Unprotect status register so it can be changed
 * @warning     Status Register is no longer protected
 * @retval      0         Success
 * @retval      Non-zero  Error condition
 */
Ext_Flash_Error_t Ext_Flash_Unprotect_StatusRegister(void);

/**
 * @brief       Bulk erase the Ext_Flash flash memory.
 * @warning     Bulk erase typically takes between 100 to 150 seconds.
 * @retval      0         Success
 * @retval      Non-zero  Error condition
 */
Ext_Flash_Error_t Ext_Flash_Bulk_Erase(void);

/**
 * @brief       Erase memory segments
 * @param       address  Start address to begin erasing.
 * @param       size     Size to erase, see #Ext_Flash_Erase_t.
 * @retval      0         Success
 * @retval      Non-zero  Error condition
 */
Ext_Flash_Error_t Ext_Flash_Erase(uint32_t address, Ext_Flash_Erase_t size);

/**
 * @brief       Read status register.
 * @param       buf      Pointer to store the value of the status register.
 * @param       reg_num  Selects which status register to read (see #Ext_Flash_StatusReg_t for valid values)
 */
Ext_Flash_Error_t Ext_Flash_Read_SR(uint8_t *buf, Ext_Flash_StatusReg_t reg_num);

/**
 * @brief       Write status register
 * @param       value  Value to write to the status register.
 * @param       reg_num  Selects which status register to write (see #Ext_Flash_StatusReg_t for valid values)
 */
Ext_Flash_Error_t Ext_Flash_Write_SR(uint8_t value, Ext_Flash_StatusReg_t reg_num);

/**
 * @brief       Execute command to move data from buffer to flash memory
 * @details     After the data is written to the buffer it needs to be transferred to the flash memory.
 *              Executing the command to do the transfer.
 * @param       address         Start address to program.
 * 
 * @retval      0           Success
 * @retval      Non-zero    Error condition    
 */
Ext_Flash_Error_t Ext_Flash_Prog_Execute(uint32_t address);

/**
 * @brief       Configures write protection scheme to protect data stored in the flash block in which "addr" is located
 * @details     The Ext_Flash write protection scheme protects sections of memory either starting at the beginning of memory 
 *              up through block selcted by "addr" or from the end of memory down through the memory block selected by "addr". 
 * 
 * @param       addr        Address to protect from being modified, passing a 0 for this argument clears all write protection 
 * @param       start       True - protect memory from start of flash up through the protected address
 *                          False - protect memory from end of flash down trhough the protected address 
 * @retval      0           Success
 * @retval      Non-zero    Error condition    
 */
Ext_Flash_Error_t Ext_Flash_Block_WP(uint32_t addr, uint32_t begin);

/**
 * @brief       Returns the start and end address of the available flash memory based on the current write protection scheme
 * 
 * @returns     Struct containing the start and addresses of available flash
 */
Ext_Flash_Unblk_t Ext_Flash_GetAvailableFlash(void);

/**@} end of group Ext_Flash_driver */
#ifdef __cplusplus
}
#endif

#endif // LIBRARIES_MISCDRIVERS_EXTMEMORY_EXT_FLASH_H_
