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
 * @file    board.h
 * @brief   Board support package API.
 */

#include <stdio.h>

#ifndef LIBRARIES_BOARDS_MAX32665_EVKIT_V1_INCLUDE_BOARD_H_
#define LIBRARIES_BOARDS_MAX32665_EVKIT_V1_INCLUDE_BOARD_H_

#ifdef __cplusplus
extern "C" {
#endif

/* 
    Define board name:
    Use as #if defined(BOARD_EVKIT_V1)
    Not as #if BOARD_EVKIT_V1
*/
#define BOARD_EVKIT_V1 1

#ifndef CONSOLE_UART
#define CONSOLE_UART 1 /// UART instance to use for console
#endif

#ifndef CONSOLE_BAUD
#define CONSOLE_BAUD 115200 /// Console baud rate
#endif

#ifdef LED_OFF
#undef LED_OFF
#endif
#define LED_OFF 1 /// Override inactive state of LEDs

#ifdef LED_ON
#undef LED_ON
#endif
#define LED_ON 0 /// Override active state of LEDs

#ifndef EXT_FLASH_BAUD
#define EXT_FLASH_BAUD 4000000
#endif

#ifndef HCI_UART
#define HCI_UART 0
#endif

#ifndef TERMINAL_UART
#define TERMINAL_UART CONSOLE_UART
#endif

#ifndef USER_UART
#define USER_UART 2
#endif

#ifndef HCI_UART_MAP
#define HCI_UART_MAP MAP_A
#endif

#ifndef TERMINAL_UART_MAP
#define TERMINAL_UART_MAP MAP_A
#endif

#ifndef USER_UART_MAP
#define USER_UART_MAP MAP_A
#endif

/**
 *  A reference to LED1 (RED LED in the RGB LED) of the board.
 *  Can be used with the LED_On, LED_Off, and LED_Toggle functions.
 */
#define LED1 0
#define LED_RED LED1

/**
 *  A reference to LED2 (GREEN LED in the RGB LED) of the board.
 *  Can be used with the LED_On, LED_Off, and LED_Toggle functions.
 */
#define LED2 1
#define LED_GREEN LED2

/**
 * \brief   Initialize the BSP and board interfaces.
 * \returns #E_NO_ERROR if everything is successful
 */
int Board_Init(void);

/**
 * \brief   Initialize or reinitialize the console. This may be necessary if the
 *          system clock rate is changed.
 * \returns #E_NO_ERROR if everything is successful
 */
int Console_Init(void);

/**
 * @brief   Initialize or reinitialize the console. This may be necessary if the
 *          system clock rate is changed.
 * @returns #E_NO_ERROR if everything is successful
 */
int Console_Shutdown(void);

#ifdef __cplusplus
}
#endif

#endif // LIBRARIES_BOARDS_MAX32665_EVKIT_V1_INCLUDE_BOARD_H_
