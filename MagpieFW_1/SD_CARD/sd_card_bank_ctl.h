/**
 * @file      sd_card_bank.h
 * @brief     A software interface for controlling the SD card bank is represented here.
 * @details   This module is responsible for selecting the active SD card, turning the SD cards on and off, and
 * querying whether an SD card is inserted or not. This module is NOT responsible for actually reading/writing to
 * the SD cards.
 */
#ifndef SD_CARD_BANK_CTL_H_
#define SD_CARD_BANK_CTL_H_

/* Includes ----------------------------------------------------------------------------------------------------------*/

#include <stdbool.h>
#include "i2c.h"

/* Public defines ----------------------------------------------------------------------------------------------------*/

#define SD_CARD_BANK_CTL_NUM_CARDS (6u)

/* Public enumerations -----------------------------------------------------------------------------------------------*/

/**
 * @brief Available card slots for the SD card bank are represented here.
 */
typedef enum
{
    SD_CARD_BANK_CARD_SLOT_0 = 0u,
    SD_CARD_BANK_CARD_SLOT_1 = 1u,
    SD_CARD_BANK_CARD_SLOT_2 = 2u,
    SD_CARD_BANK_CARD_SLOT_3 = 3u,
    SD_CARD_BANK_CARD_SLOT_4 = 4u,
    SD_CARD_BANK_CARD_SLOT_5 = 5u,
    SD_CARD_BANK_ALL_SLOTS_DISABLED,
} SD_Card_Bank_Card_Slot_t;

/**
 * @brief Enumerated SD card bank errors are represented here.
 */
typedef enum
{
    SD_CARD_BANK_CTL_ERROR_ALL_OK,
    SD_CARD_BANK_CTL_I2C_ERROR,
    SD_CARD_BANK_CTL_INVALID_INPUT_ERROR
} SD_Card_Bank_Ctl_Error_t;

/* Public function declarations --------------------------------------------------------------------------------------*/

/**
 * @brief `sd_card_bank_ctl_init(i2c)` initializes the SD card bank with I2C handle `i2c` and configures all of the I2C
 * GPIO port expander pins used to control the SD card bank.
 *
 * @param hi2c: pointer to the I2C handle to use for I2C transactions.
 *
 * @post All input/output pins of the port expander are configured and the SD card bank is powered down.
 *
 * @return `SD_CARD_BANK_ERROR_ALL_OK` if the function succeeded, else an error code
 *
 * This must be performed before any other SD card bank functions are called.
 */
SD_Card_Bank_Ctl_Error_t sd_card_bank_ctl_init(mxc_i2c_regs_t *hi2c);

/**
 * @brief `sd_card_bank_ctl_disable_all()` deselects any active card slot and powers down the SD card bank.
 *
 * @pre `sd_card_bank_init()` has been called.
 *
 * @post all SD cards are powered down and no card slot is selected as active.
 *
 * @return `SD_CARD_BANK_ERROR_ALL_OK` if the function succeeded, else an error code.
 */
SD_Card_Bank_Ctl_Error_t sd_card_bank_ctl_disable_all();

/**
 * @brief `sd_card_bank_ctl_enable_slot(s)` powers on the SD card bank and selects card slot `s` as the active card.
 *
 * @param slot: the card slot to select as the active card, or `SD_CARD_BANK_ALL_SLOTS_DISABLED` to power down the SD
 * card bank. Must be a valid `SD_Card_Bank_Card_Slot_t` enumeration.
 *
 * @pre `sd_card_bank_init()` has been called.
 *
 * @post a single SD card slot is powered on and the communication lines are routed to that card slot, or the SD card
 * bank is disabled in the case of `SD_CARD_BANK_ALL_SLOTS_DISABLED`. If the slot given is out of range, this function
 * has no effect and simply returns `SD_CARD_BANK_INVALID_INPUT_ERROR`.
 *
 * @return `SD_CARD_BANK_ERROR_ALL_OK` if the function succeeded, else an error code.
 */
SD_Card_Bank_Ctl_Error_t sd_card_bank_ctl_enable_slot(SD_Card_Bank_Card_Slot_t slot);

/**
 * @brief `sd_card_bank_ctl_get_active_slot()` is the current enumerated active card slot.
 *
 * @pre `sd_card_bank_init()` has been called.
 *
 * @return the active card slot, or `SD_CARD_BANK_ALL_SLOTS_DISABLED` if there is no active card slot.
 */
SD_Card_Bank_Card_Slot_t sd_card_bank_ctl_get_active_slot();

/**
 * @brief `sd_card_bank_ctl_read_and_cache_detect_pins()` reads the SD card detect pins and caches the result.
 *
 * @pre `sd_card_bank_init()` has been called.
 *
 * @post the value of all SD card detect pins is cached. The status of the active SD card can be queried via the
 * `sd_card_bank_active_card_is_inserted()` boolean function.
 *
 * @return `SD_CARD_BANK_ERROR_ALL_OK` if the function succeeded, else an error code.
 *
 */
SD_Card_Bank_Ctl_Error_t sd_card_bank_ctl_read_and_cache_detect_pins();

/**
 * @brief `sd_card_bank_ctl_active_card_is_inserted()` is true iff a card in the active slot is physically detected.
 *
 * To be detected the bank must not be disabled and the active card must physically close the mechanical switch on
 * the SD card holder that pulls the detect pin to ground.
 *
 * This does not guarantee that the card is mounted or has no errors, only that it is physically inserted. It is even
 * possible to "trick" this function by jamming something other than a card into the slot.
 *
 * You must call `sd_card_bank_read_and_cache_detect_pins()` to cache the value of the detect pins immediately before
 * calling this function.
 *
 * @pre `sd_card_bank_init()` has been called. To be valid, `sd_card_bank_read_and_cache_detect_pins()` has been called.
 *
 * @return true if a card is physically detected in the current active card slot, otherwise false.
 */
bool sd_card_bank_ctl_active_card_is_inserted();

#endif /* SD_CARD_BANK_H_ */