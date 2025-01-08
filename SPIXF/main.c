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

/**
 * @file    main.c
 * @brief   SPIX example using the external flash.
 * @details Uses the external flash on the EvKit to show the SPIX. Erases, writes, and then
 *          verifies the data. EXT_FLASH_BAUD, EXT_FLASH_ADDR, and EXT_FLASH_SPIXFC_WIDTH
 *          can be changed to alter the communication between the devices. Refer
 *          to the schematic for the pinout and ensure that there are no switches
 *          blocking the communication to the external flash.
 */

/***** Includes *****/
#include <stdio.h>
#include <string.h>
#include "mxc_sys.h"
#include "mxc_errors.h"
#include "max32665.h"
#include "Ext_Flash.h"
#include "spixf.h"
#include "board.h"
#include "led.h"
#include "lfs.h"
#include "lfs_util.h" 
#include "time.h"

/***** Definitions *****/
#define EXT_FLASH_BAUD 8000000 /* SPI clock rate to communicate with the external flash */
#define EXT_FLASH_ADDR 0
#define EXT_FLASH_SPIXFC_WIDTH Ext_Flash_DataLine_Single
#define EXT_FLASH_SIZE 268435456//1048576

#define BUFF_SIZE  2112

/***** Functions *****/

// These are set in the linkerfile and give the starting and ending address of xip_section
#if defined(__GNUC__)
extern uint8_t __load_start_xip, __load_length_xip;
#endif

#if defined(__CC_ARM)
// Note: This demo has not been tested under IAR and should be considered non-functional
extern int Image$$RW_IRAM2$$Length;
extern char Image$$RW_IRAM2$$Base[];
uint8_t *__xip_addr;
#endif

#define NAND_PAGE_SIZE 2048
#define NAND_BLOCK_SIZE (64 * NAND_PAGE_SIZE)  // Block size = 64 pages × 2048 bytes = 128KB
#define NAND_TOTAL_SIZE 268435456  // 256 MB
#define NAND_BLOCK_COUNT 2048

int block_device_read(const struct lfs_config *c, lfs_block_t block, lfs_off_t off, void *buffer, lfs_size_t size) {
    uint32_t addr = block * c->block_size + off;

    //First fill data buffer
    Ext_Flash_DataRead(addr);
    //then read from data buffer
    return Ext_Flash_Read(addr, buffer, size, EXT_FLASH_SPIXFC_WIDTH);
}

int block_device_prog(const struct lfs_config *c, lfs_block_t block, lfs_off_t off, const void *buffer, lfs_size_t size) {
    uint32_t addr = block * c->block_size + off;
    
    return Ext_Flash_Program_Page(addr, buffer, size, EXT_FLASH_SPIXFC_WIDTH);
}

int block_device_erase(const struct lfs_config *c, lfs_block_t block) {
    uint32_t addr = block * c->block_size;

    return Ext_Flash_Bulk_Erase();
    //return Ext_Flash_Erase(addr, Ext_Flash_Erase_128K);
}

int block_device_sync(const struct lfs_config *c) {
    return Ext_Flash_SyncFlash();
}

// Configure LittleFS
const struct lfs_config cfg = {
    .read  = block_device_read,
    .prog  = block_device_prog,
    .erase = block_device_erase,
    .sync  = block_device_sync,

    .block_size = NAND_BLOCK_SIZE,
    .block_count = NAND_BLOCK_COUNT,
    .cache_size = NAND_PAGE_SIZE,
    .lookahead_size = NAND_PAGE_SIZE,
    .block_cycles = 100000,
    .read_size = NAND_BLOCK_COUNT,
    .prog_size = NAND_BLOCK_COUNT,
};

lfs_t lfs;

/******************************************************************************/
static int ext_flash_module_init(void)
{
    int err;

    err = MXC_SPIXF_Init(0x0B, EXT_FLASH_BAUD);

    if (err == E_NO_ERROR) {
        MXC_SPIXF_Enable();
    }
    return err;
}

/******************************************************************************/
//SPIFX read write setup communication functions (not the actual flash read/write)
static int ext_flash_module_read(uint8_t *read, unsigned len, unsigned deassert,
                                Ext_Flash_DataLine_t width)
{
    mxc_spixf_req_t req = { deassert, 0, NULL, read, (mxc_spixf_width_t)width, len, 0, 0, NULL };

    if (MXC_SPIXF_Transaction(&req) != len) {
        return E_COMM_ERR;
    }
    return E_NO_ERROR;
}

/******************************************************************************/
static int ext_flash_module_write(const uint8_t *write, unsigned len, unsigned deassert,
                                 Ext_Flash_DataLine_t width)
{
    mxc_spixf_req_t req = { deassert, 0, write, NULL, (mxc_spixf_width_t)width, len, 0, 0, NULL };

    if (MXC_SPIXF_Transaction(&req) != len) {
        return E_COMM_ERR;
    }
    return E_NO_ERROR;
}

/******************************************************************************/
static int ext_flash_clock(unsigned len, unsigned deassert)
{
    return MXC_SPIXF_Clocks(len, deassert);
}

int SetupExtFlash(void)
{
    int err;
    Ext_Flash_Config_t exf_cfg = { .init = ext_flash_module_init,
                                   .read = ext_flash_module_read,
                                   .write = ext_flash_module_write,
                                   .clock = ext_flash_clock };

    if ((err = Ext_Flash_Configure(&exf_cfg)) != E_NO_ERROR) {
        return err;
    }
    return E_NO_ERROR;
}
/******************************************************************************/
void spixf_cfg_setup()
{
    // Disable the SPIXFC before setting the SPIXF
    MXC_SPIXF_Disable();
    MXC_SPIXF_SetSPIFrequency(EXT_FLASH_BAUD);
    MXC_SPIXF_SetMode(MXC_SPIXF_MODE_0);
    MXC_SPIXF_SetSSPolActiveLow();
    MXC_SPIXF_SetSSActiveTime(MXC_SPIXF_SYS_CLOCKS_2);
    MXC_SPIXF_SetSSInactiveTime(MXC_SPIXF_SYS_CLOCKS_3);

    if (EXT_FLASH_SPIXFC_WIDTH == Ext_Flash_DataLine_Single) {
        MXC_SPIXF_SetCmdValue(EXT_FLASH_CMD_READ);
        MXC_SPIXF_SetCmdWidth(MXC_SPIXF_SINGLE_SDIO);
        MXC_SPIXF_SetAddrWidth(MXC_SPIXF_SINGLE_SDIO);
        MXC_SPIXF_SetDataWidth(MXC_SPIXF_WIDTH_1);
        MXC_SPIXF_SetModeClk(EXT_FLASH_Read_DUMMY);
    } else {

        MXC_SPIXF_SetCmdValue(EXT_FLASH_CMD_DREAD);
        MXC_SPIXF_SetCmdWidth(MXC_SPIXF_DUAL_SDIO);
        MXC_SPIXF_SetAddrWidth(MXC_SPIXF_DUAL_SDIO);
        MXC_SPIXF_SetDataWidth(MXC_SPIXF_WIDTH_2);
        MXC_SPIXF_SetModeClk(EXT_FLASH_DREAD_DUMMY);


        // MXC_SPIXF_SetCmdValue(EXT_FLASH_CMD_QREAD);
        // MXC_SPIXF_SetCmdWidth(MXC_SPIXF_SINGLE_SDIO);
        // MXC_SPIXF_SetAddrWidth(MXC_SPIXF_QUAD_SDIO);
        // MXC_SPIXF_SetDataWidth(MXC_SPIXF_WIDTH_4);
        // MXC_SPIXF_SetModeClk(EXT_FLASH_QREAD_DUMMY);
    }

    MXC_SPIXF_Set3ByteAddr();
    MXC_SPIXF_SCKFeedbackEnable();
    MXC_SPIXF_SetSCKNonInverted();
}

/******************************************************************************/
int main(void)
{
    int fail = 0;
    uint32_t id;
    //void (*func)(void);
    uint8_t rx_buf[BUFF_SIZE];
    int rx_len = (uint32_t)(&__load_length_xip);
    int remain = rx_len;

    printf("\n\n********************* SPIX Example *********************\n");
    printf("This example communicates with an %s flash on the FTHR\n", EXT_FLASH_NAME);
    //printf("loads code onto it and then executes that code using the \n");
    //printf("SPIX execute-in-place peripheral\n\n");

    printf("SPI Clock: %d Hz\n\n", EXT_FLASH_BAUD);


    printf("Configuring SPI communication with external flash ...\n");
    // Setup SPIX
    
    spixf_cfg_setup();

    if(E_NO_ERROR != SetupExtFlash())
    {
        printf("[ERROR]--> Setting up external flash failed.\n\n");
        printf("Example Failed\n");
        return E_FAIL;
    }
    printf("[SUCCESS]--> External flash SPI communication configured.\n\n");

    printf("Initializing External flash ...\n");
    // Initialize the SPIXFC registers and set the appropriate output pins
    if (E_NO_ERROR != Ext_Flash_Init()) {
        printf("[ERROR]--> External flash module init failed\n\n");
        printf("Example Failed\n");
        return E_FAIL;
    }
    printf("[SUCCESS]--> External flash Initialized.\n\n");

    printf("Resetting External Flash ...\n");
    if (E_NO_ERROR != Ext_Flash_Reset()){
        printf("[ERROR]--> External Flash reset failed\n\n");
        printf("Example Failed\n");
        return E_FAIL;
    }
    printf("[SUCCESS]--> External flash reset successfully.\n\n");

    printf("Checking External Flash ID ...\n");
    // Get the ID of the external flash
    if ((id = Ext_Flash_ID()) == EXT_FLASH_EXP_ID) {
        printf("[SUCCESS]--> ID Found: 0x%x. \n[Mnf: Winbond]\n[Type: SpiNAND]\n[Device: W25N02KV]\nExternal flash ID verified.\n\n", id);
    } else {
        printf("[ERROR]--> Error verifying external flash ID.\nUnknown ID: 0x%x\n\n", id);
        printf("Example Failed\n");
        return E_FAIL;
    }

    int err;

    //Unprotect status register and write protection
    printf("Unlocking Flash Write Protection ...\n");
    if(E_NO_ERROR != Ext_Flash_Unprotect_StatusRegister()){
        printf("[ERROR]--> Failed to unlock write protection.\n\n");
        return E_FAIL;
    } else {
        printf("[SUCCESS]--> Write protection unlocked.\n\n");
    }

    // Erase Flash
    // printf("Bulk erasing flash\n");
    // if(E_NO_ERROR != Ext_Flash_Bulk_Erase()){
    //     printf("[ERROR]--> Flash failed to bulk erase\n\n");
    //     return E_FAIL;
    // } else {
    //     printf("[SUCCESS]--> Flash succesfully bulk erased.\n\n");
    // }


/* 
    //Unprotect status register and write protection
    printf("Unlocking Write Protection ...\n");
    if(E_NO_ERROR != Ext_Flash_Unprotect_StatusRegister()){
        printf("[ERROR]--> Failed to unlock write protection.\n\n");
        return E_FAIL;
    } else {
        printf("[SUCCESS]--> Write protection unlocked.\n\n");
    }

    // Erase Test Sector
    printf("Erasing first 128k sector\n");
    if(E_NO_ERROR != Ext_Flash_Erase(0x00000, Ext_Flash_Erase_128K)){
        printf("[ERROR]--> First 128k sector failed to erase\n\n");
        return E_FAIL;
    } else {
        printf("[SUCCESS]--> First 128k sector succesfully erased.\n\n");
    }

    
    err = Ext_Flash_Read(0x000000, rx_buf, 200, EXT_FLASH_SPIXFC_WIDTH);

    printf("===== Address [0x%06xh], Chunk [%d] =====\n", 0x000000, 200);
    for (int i = 0; i < 200; i++) {
        printf("%02xh\t", rx_buf[i]); 
    }
    printf("\n\n");
    printf("...");
    printf("\n\n");
    
    err = Ext_Flash_Read(0x01F338, rx_buf, 200, EXT_FLASH_SPIXFC_WIDTH);

    printf("===== Address [0x%06xh], Chunk [%d] =====\n", 0x01F338, 200);
    for (int i = 0; i < 200; i++) {
        printf("%02xh\t", rx_buf[i]); 
    }
    printf("\n\n");
    

    // // Enable Quad mode if we are using quad
    // if (EXT_FLASH_SPIXFC_WIDTH == Ext_Flash_DataLine_Quad) {
    //     if (Ext_Flash_Quad(1) != E_NO_ERROR) {
    //         printf("Error enabling quad mode\n\n");
    //         fail++;
    //     } else {
    //         printf("Quad mode enabled\n\n");
    //     }
    // } else {
    //     if (Ext_Flash_Quad(0) != E_NO_ERROR) {
    //         printf("Error disabling quad mode\n\n");
    //         fail++;
    //     } else {
    //         printf("Quad mode disabled\n\n");
    //     }
    // }

    // Program the external flash
    printf("Programming function (%d bytes @ 0x%08x) into external flash address 0x%08x ...\n",
           (uint32_t)(&__load_length_xip), &__load_start_xip, EXT_FLASH_ADDR);

    
    uint8_t* arr = &__load_start_xip;
    for (int i = 0; i < rx_len; i++) {
        printf("%02xh\t", *(arr + i)); // Equivalent to arr[i]
    }
    printf("\n");


    if ((err = Ext_Flash_Program_Page(EXT_FLASH_ADDR, &__load_start_xip,
                                      (uint32_t)(&__load_length_xip), EXT_FLASH_SPIXFC_WIDTH)) !=
        E_NO_ERROR) {
        printf("[ERROR]--> Error Programming: %d\n", err);
        fail++;
    } else {
        printf("[SUCCESS]--> External Flash Programmed\n\n");
    }
    
    while (remain) {
        int chunk = ((remain > BUFF_SIZE) ? BUFF_SIZE : remain);

        printf("Verifying External Flash Written Memory from flash address 0x%08x ...\n", EXT_FLASH_ADDR + rx_len - remain);
        err = Ext_Flash_DataRead(EXT_FLASH_ADDR + rx_len - remain);
        if(E_NO_ERROR != err){
            printf("[ERROR]--> Error reading data into buffer: %d\n", err);
            break;
        }

        err = Ext_Flash_Read(EXT_FLASH_ADDR + rx_len - remain, rx_buf, chunk,
                                  EXT_FLASH_SPIXFC_WIDTH);
        if(E_NO_ERROR != err){
            printf("[ERROR]--> Error reading data from buffer: %d\n", err);
            break;
        }

        printf("===== Chunk [%d] =====\n", chunk);
        for (int i = 0; i < rx_len; i++) {
            printf("%02xh\t", rx_buf[i]); 
        }
        printf("\n\n");

        if (E_NO_ERROR != err) {
            printf("[ERROR]--> Error verifying data %d\n", err);
            fail++;
            break;
        } else if (memcmp(rx_buf, &__load_start_xip + rx_len - remain, chunk) != E_NO_ERROR) {
            printf("[ERROR]--> Error invalid data\n");
            fail++;
            break;
        } else if (remain == chunk) {
            printf("[SUCCESS]--> Programmed flash memory verified\n\n");
        }
        remain -= chunk;
    }
 */

// ----   This section doesn't work unless the Flash is a NOR type.  Required to do execution in place (XIP)
/*     if (fail != 0) {
        printf("\nExample Failed\n");
        return E_FAIL;
    } else {
        spixf_cfg_setup();

        printf("Jumping to external flash (@ 0x%08x), watch for blinking LED.\n\n",
            (MXC_XIP_MEM_BASE | 0x1));
        func = (void (*)(void))(MXC_XIP_MEM_BASE | 0x1);
        func();
        printf("Returned from external flash\n\n");
    } */

/*
// Littlefs error codes
enum lfs_error {
    LFS_ERR_OK          = 0,    // No error
    LFS_ERR_IO          = -1,   // Error during device operation
    LFS_ERR_CORRUPT     = -84,  // Corrupted
    LFS_ERR_NOENT      = -2,    // No directory entry
    LFS_ERR_EXIST      = -17,   // Entry already exists
    LFS_ERR_NOTDIR     = -20,   // Entry is not a dir
    LFS_ERR_ISDIR      = -21,   // Entry is a dir
    LFS_ERR_NOTEMPTY   = -39,   // Dir is not empty
    LFS_ERR_BADF       = -9,    // Bad file number
    LFS_ERR_FBIG       = -27,   // File too large
    LFS_ERR_INVAL      = -22,   // Invalid parameter
    LFS_ERR_NOSPC      = -28,   // No space left on device
    LFS_ERR_NOMEM      = -12,   // No more memory available
};
*/


    // Initialize LittleFS
    lfs_file_t file;

    // Mount the filesystem
    err = lfs_mount(&lfs, &cfg);
    if (err) {
        printf("Failed to mount filesystem.\n");

        //Format the flash drive if the file system is not mountable
        //Might not want to do this everytime.
        printf("Formatting...\n");
        err = lfs_format(&lfs, &cfg);
        if (err) {
            printf("Failed to format filesystem: error %d\n", err);
            return err;
        }
        printf("Filesystem formatted. Mounting...\n");
        err = lfs_mount(&lfs, &cfg);
        if (err) {
            printf("Failed to mount formatted filesystem: error %d\n", err);
            return err;
        }
    }
    printf("File system mounted successfully\n");

    // Define your struct
    typedef struct {
        int id;
        char name[20];
        float temperature_c;
        struct tm setDateTime;
    } MyData;

    struct tm newTime = {
		.tm_year = 2025 - 1900U,
		.tm_mon = 1 - 1U,
		.tm_mday = 8U,
		.tm_hour = 3U,
		.tm_min = 52U,
		.tm_sec = 0U
	};

    MyData data = {1, "Magpie Recorder", 25.2, newTime};


    // Write the struct to the file
    err = lfs_file_open(&lfs, &file, "setup.bin", LFS_O_RDWR | LFS_O_CREAT);
    if (err) {
        printf("Failed to open file for writing: error %d\n", err);
        lfs_unmount(&lfs);
        return err;
    }

    //****Note that the data type here is lfs_size_t and NOT the lfs_ssize_t we used for writing just the string data.
    lfs_size_t struct_written = lfs_file_write(&lfs, &file, &data, sizeof(MyData));
    if (struct_written != sizeof(MyData)) {
        printf("Failed to write struct data to file: error %d\n", struct_written);
        lfs_file_close(&lfs, &file);
        lfs_unmount(&lfs);
        return struct_written;
    }
    printf("setup.bin written successfully. %d written\n", struct_written);

    
    // // Write to a file
    // err = lfs_file_open(&lfs, &file, "test.txt", LFS_O_RDWR | LFS_O_CREAT);
    // if (err) {
    //     printf("Failed to open file for writing: error %d\n", err);
    //     lfs_unmount(&lfs);
    //     return err;
    // }
    // printf("test.txt opened for write successfully\n");

    // char *data = "Hello, LittleFS on MAX32666!  This is a test writing of a file.\n Hope it works!\n";
    // lfs_ssize_t written = lfs_file_write(&lfs, &file, data, strlen(data));
    // if (written < 0) {
    //     printf("Failed to write to file: error %d\n", written);
    //     lfs_file_close(&lfs, &file);
    //     lfs_unmount(&lfs);
    //     return written;
    // }
    // printf("test.txt written successfully. %d written\n", written);

    //Sync Flash after write
    err = lfs_file_sync(&lfs, &file);
    if (err) {
        printf("Failed to sync file: error %d\n", err);
        lfs_file_close(&lfs, &file);
        lfs_unmount(&lfs);
        return err;
    }
    printf("File sync successfully.\n");

    lfs_file_close(&lfs, &file);
    
    //List directory 
    lfs_dir_t dir;
    struct lfs_info info;

    err = lfs_dir_open(&lfs, &dir, "/");
    if (err) {
        printf("Failed to open root directory: error %d\n", err);
        return err;
    }

    while (true) {
        int res = lfs_dir_read(&lfs, &dir, &info);
        if (res < 0) {
            printf("Error reading directory: %d\n", res);
            break;
        }
        if (res == 0) {
            break;
        }
        printf("Found file: %s\n", info.name);
    }

    lfs_dir_close(&lfs, &dir);


    // Read from the file
    printf("Opening test.txt for read only mode.\n");

    char buf[NAND_BLOCK_SIZE] = {0};
    err = lfs_file_open(&lfs, &file, "test.txt", LFS_O_RDONLY);
    if (err == LFS_ERR_NOENT){
         printf("File does not exist\n");
    }
    else if (err) {
        printf("Failed to open file for reading: error %d\n", err);
        lfs_unmount(&lfs);
        return err;
    }

    lfs_ssize_t read = lfs_file_read(&lfs, &file, buf, sizeof(buf) - 1);
    if (read < 0) {
        printf("Failed to read from file: error %d\n", read);
        lfs_file_close(&lfs, &file);
        lfs_unmount(&lfs);
        return read;
    }
    lfs_file_close(&lfs, &file);    
    printf("Reading content from test.txt file:\n %s\n", buf);



    //////  Read struct from setup.prm file
    printf("Opening setup.bin for read only mode.\n");    

    err = lfs_file_open(&lfs, &file, "setup.bin", LFS_O_RDONLY);
    if (err == LFS_ERR_NOENT){
         printf("File does not exist\n");
    }
    else if (err) {
        printf("Failed to open file for reading: error %d\n", err);
        lfs_unmount(&lfs);
        return err;
    }

    MyData readData;

    //Note that the data type here is lfs_size_t and not the lfs_ssize_t we used for reading just the string data.
    lfs_size_t struct_read = lfs_file_read(&lfs, &file, &readData, sizeof(MyData));
    if (struct_read != sizeof(MyData)) {
        printf("Failed to retrieve struct data from file: error %d\n", read);
        lfs_file_close(&lfs, &file);
        lfs_unmount(&lfs);
        return struct_read;
    }

    static uint8_t dateTimeStr[128];

    //Convert datetime type to string
    strftime((char*)dateTimeStr, 128, "%F %TZ", &readData.setDateTime);

    printf("ID: %d, Name: %s, Temperature(°C): %f, Set Date/Time: %s", readData.id, readData.name, readData.temperature_c, dateTimeStr);
	
    lfs_file_close(&lfs, &file);

    // Unmount the filesystem
    lfs_unmount(&lfs);
    
   

    printf("\n ************** Example Succeeded ****************\n");
    return E_NO_ERROR;
}
