/**
 * @file    sd_card.h
 * @brief   A software module for initializing and writing to the SD card is represented here.
 * @details This is a thin wrapper over the existing Elm-chan FatFS library and MSDK SDHC functions.
 */

#ifndef SD_CARD_H_
#define SD_CARD_H_

/* Includes ----------------------------------------------------------------------------------------------------------*/

#include "ff.h"
#include <stdbool.h>
#include <stdint.h>

/* Public enumerations -----------------------------------------------------------------------------------------------*/

/**
 * @brief Enumerated SD card errors are represented here.
 */
typedef enum
{
    SD_CARD_ERROR_ALL_OK,
    SD_CARD_NOT_INSERTED_ERROR,
    SD_CARD_INIT_ERROR,
    SD_CARD_MOUNT_ERROR,
    SD_CARD_DIRECTORY_ERROR,
    SD_CARD_FILE_IO_ERROR,
} SD_Card_Error_t;

/**
 * @brief Enumerated POSIX file modes are represented here. Only file modes we actually use are present, not all modes.
 */
typedef enum
{
    POSIX_FILE_MODE_READ = FA_READ,
    POSIX_FILE_MODE_WRITE = FA_CREATE_ALWAYS | FA_WRITE,
    POSIX_FILE_MODE_APPEND = FA_OPEN_APPEND | FA_WRITE,
} POSIX_FileMode_t;

/* Public function declarations --------------------------------------------------------------------------------------*/

/**
 * @brief `sd_card_init()` initializes the SD card. This must be called each time a card is powered up.
 *
 * @pre An SD card previously formatted to exFAT or FAT32 is physically inserted in the slot.
 *
 * @post The SD card is initialized and ready to be mounted.
 *
 * @return `SD_CARD_ERROR_ALL_OK` if initialization was successful, else an error.
 */
SD_Card_Error_t sd_card_init();

/**
 * @brief `sd_card_mount()` mounts the SD card.
 *
 * @pre `sd_card_init()` must have been successfully called prior to mounting.
 *
 * @post The SD card is mounted and ready for file IO operations.
 *
 * @return `SD_CARD_ERROR_ALL_OK` if card mounting was successful, else an error.
 */
SD_Card_Error_t sd_card_mount();

/**
 * @brief `sd_card_unmount()` unmounts the SD card if it was previously mounted.
 *
 * @post The SD card is unmounted.
 *
 * @return `SD_CARD_ERROR_ALL_OK` if unmounting was successful, else an error.
 */
SD_Card_Error_t sd_card_unmount();

/**
 * @brief `sd_card_is_mounted()` is true iff the SD card is mounted.
 *
 * @return true if the card is mounted, else false.
 */
bool sd_card_is_mounted();

/**
 * @brief `sd_card_disk_size_bytes()` is the size of the currently mounted SD card in bytes.
 *
 * @return the size of the mounted SD card in bytes, or zero if there is no card mounted or an error occurs.
 */
uint32_t sd_card_disk_size_bytes();

/**
 * @brief `sd_card_free_space_bytes()` is the number of bytes of free space on the currently mounted SD card.
 *
 * @return the number of free bytes on the mounted SD card, or zero if there is no card mounted or an error occurs.
 */
uint32_t sd_card_free_space_bytes();

/**
 * @brief `sd_card_mkdir(p)` creates a new directory on the currently mounted SD card at directory path `p`.
 *
 * @param path the path of the new directory to create.
 *
 * @pre The SD card is mounted.
 *
 * @post A new directory at path `p` is created.
 *
 * @return `SD_CARD_ERROR_ALL_OK` if the operation was successful, else an error.
 */
SD_Card_Error_t sd_card_mkdir(const char *path);

/**
 * `sd_card_cd(p)` changes the directory of the currently mounted SD card to path `p`.
 *
 * @param path the path to change directory into.
 *
 * @pre The SD card is mounted and path `p` exists.
 *
 * @post The current directory is changed to `p`.
 *
 * @return `SD_CARD_ERROR_ALL_OK` if the operation was successful, else an error.
 */
SD_Card_Error_t sd_card_cd(const char *path);

/**
 * @brief `sd_card_fopen(f, m)` opens file at path `f` with file mode `m` on the currently mounted SD card.
 *
 * @param file_name the name of the file to open.
 *
 * @param mode the enumerated POSIX file mode to use.
 *
 * @pre The SD card is mounted.
 *
 * @post A file is opened with the given file mode.
 *
 * @return `SD_CARD_ERROR_ALL_OK` if the operation was successful, else an error.
 */
SD_Card_Error_t sd_card_fopen(const char *file_name, POSIX_FileMode_t mode);

/**
 * @brief `sd_card_fclose()` closes any open file on the currently mounted SD card.
 *
 * @pre The SD card is mounted.
 *
 * @post If a file was open, it is closed.
 *
 * @return `SD_CARD_ERROR_ALL_OK` if the operation was successful, else an error.
 */
SD_Card_Error_t sd_card_fclose();

/**
 * @brief `sd_card_fwrite(b, s, w)` writes `s` bytes of buffer `b` to the currently mounted SD card and stores the number
 * of bytes written in integer pointer `w`.
 *
 * @param buff pointer to the buffer to write from.
 *
 * @param size the number of bytes from `buff` to write to the card.
 *
 * @param written pointer to an integer to store the number of bytes actually written into.
 *
 * @pre The SD card is mounted and a file is opened for writing or appending.
 *
 * @post Bytes from the buffer are written to the previously opened file on the SD card.
 *
 * @return `SD_CARD_ERROR_ALL_OK` if the operation was successful, else an error. If the number of bytes in `written`
 * does not match `size` after this function returns, this indicates some write error occurred.
 */
SD_Card_Error_t sd_card_fwrite(const void *buff, uint32_t size, uint32_t *written);

/**
 * @brief `sd_card_lseek(o)` offsets the file pointer `o` bytes from the top of the currently opened file.
 *
 * @param offset the number of bytes to offset from the top of the file.
 *
 * @pre The SD card is mounted and a file is opened.
 *
 * @post The read/write pointer of the currently open file is moved to `o` bytes from the top of the file.
 *
 * @return `SD_CARD_ERROR_ALL_OK` if the operation was successful, else an error.
 */
SD_Card_Error_t sd_card_lseek(uint32_t offset);

/**
 * @brief `sd_card_fsize()` is the size in bytes of the currently opened file.
 *
 * @pre The SD card is mounted and a file is opened.
 *
 * @return The size of the currently open file in bytes, invalid if a file is not open.
 */
uint32_t sd_card_fsize();

#endif /* SD_CARD_H_ */