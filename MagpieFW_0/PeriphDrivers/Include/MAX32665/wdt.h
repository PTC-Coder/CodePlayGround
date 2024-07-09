/**
 * @file    wdt.h
 * @brief   Watchdog timer (WDT) function prototypes and data types.
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

/* Define to prevent redundant inclusion */
#ifndef LIBRARIES_PERIPHDRIVERS_INCLUDE_MAX32665_WDT_H_
#define LIBRARIES_PERIPHDRIVERS_INCLUDE_MAX32665_WDT_H_

/* **** Includes **** */
#include <stdint.h>
#include "mxc_device.h"
#include "wdt_regs.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @defgroup wdt Watchdog Timer (WDT)
 * @ingroup periphlibs
 * @{
 */

/* **** Definitions **** */

/** @brief Watchdog period enumeration.
    Used to configure the period of the watchdog interrupt */
typedef enum {
    MXC_WDT_PERIOD_2_31 = MXC_S_WDT_CTRL_INT_PERIOD_WDT2POW31, ///< Period 2^31
    MXC_WDT_PERIOD_2_30 = MXC_S_WDT_CTRL_INT_PERIOD_WDT2POW30, ///< Period 2^30
    MXC_WDT_PERIOD_2_29 = MXC_S_WDT_CTRL_INT_PERIOD_WDT2POW29, ///< Period 2^29
    MXC_WDT_PERIOD_2_28 = MXC_S_WDT_CTRL_INT_PERIOD_WDT2POW28, ///< Period 2^28
    MXC_WDT_PERIOD_2_27 = MXC_S_WDT_CTRL_INT_PERIOD_WDT2POW27, ///< Period 2^27
    MXC_WDT_PERIOD_2_26 = MXC_S_WDT_CTRL_INT_PERIOD_WDT2POW26, ///< Period 2^26
    MXC_WDT_PERIOD_2_25 = MXC_S_WDT_CTRL_INT_PERIOD_WDT2POW25, ///< Period 2^25
    MXC_WDT_PERIOD_2_24 = MXC_S_WDT_CTRL_INT_PERIOD_WDT2POW24, ///< Period 2^24
    MXC_WDT_PERIOD_2_23 = MXC_S_WDT_CTRL_INT_PERIOD_WDT2POW23, ///< Period 2^23
    MXC_WDT_PERIOD_2_22 = MXC_S_WDT_CTRL_INT_PERIOD_WDT2POW22, ///< Period 2^22
    MXC_WDT_PERIOD_2_21 = MXC_S_WDT_CTRL_INT_PERIOD_WDT2POW21, ///< Period 2^21
    MXC_WDT_PERIOD_2_20 = MXC_S_WDT_CTRL_INT_PERIOD_WDT2POW20, ///< Period 2^20
    MXC_WDT_PERIOD_2_19 = MXC_S_WDT_CTRL_INT_PERIOD_WDT2POW19, ///< Period 2^19
    MXC_WDT_PERIOD_2_18 = MXC_S_WDT_CTRL_INT_PERIOD_WDT2POW18, ///< Period 2^18
    MXC_WDT_PERIOD_2_17 = MXC_S_WDT_CTRL_INT_PERIOD_WDT2POW17, ///< Period 2^17
    MXC_WDT_PERIOD_2_16 = MXC_S_WDT_CTRL_INT_PERIOD_WDT2POW16, ///< Period 2^16
} mxc_wdt_period_t;

/* **** Function Prototypes **** */

/**
 * @brief       Initialize the Watchdog Timer
 * @note        On default this function enables WDT peripheral clock.
 *              if you wish to manage clock and gpio related things in upper level instead of here.
 *              Define MSDK_NO_GPIO_CLK_INIT flag in project.mk file. 
 *              By this flag this function will remove clock and gpio related codes from file.
 * 
 * @param       wdt      Pointer to the watchdog registers, has no effect incase of
 *                       MSDK_NO_GPIO_CLK_INIT has been defiend.
 * @return      See \ref MXC_Error_Codes for the list of error codes.
 */
int MXC_WDT_Init(mxc_wdt_regs_t *wdt);

/**
 * @brief Shutdown the Watchdog Timer
 * @param       wdt     Pointer to the watchdog registers
 * @return      See \ref MXC_Error_Codes for the list of error codes.
 */
int MXC_WDT_Shutdown(mxc_wdt_regs_t *wdt);

/**
 * @brief       Set the period of the watchdog interrupt.
 * @param       wdt     Pointer to watchdog registers.
 * @param       period  Enumeration of the desired watchdog period.
 */
void MXC_WDT_SetIntPeriod(mxc_wdt_regs_t *wdt, mxc_wdt_period_t period);

/**
 * @brief       Set the period of the watchdog reset.
 * @param       wdt     Pointer to watchdog registers.
 * @param       period  Enumeration of the desired watchdog period.
 */
void MXC_WDT_SetResetPeriod(mxc_wdt_regs_t *wdt, mxc_wdt_period_t period);

/**
 * @brief       Enable the watchdog timer.
 * @param       wdt     Pointer to watchdog registers.
 */
void MXC_WDT_Enable(mxc_wdt_regs_t *wdt);

/**
 * @brief       Disable the watchdog timer.
 * @param       wdt     Pointer to watchdog registers.
 */
void MXC_WDT_Disable(mxc_wdt_regs_t *wdt);

/**
 * @brief       Enable the watchdog interrupt.
 * @param       wdt     Pointer to watchdog registers.
 */
void MXC_WDT_EnableInt(mxc_wdt_regs_t *wdt);

/**
 * @brief       Enable the watchdog reset.
 * @param       wdt     Pointer to watchdog registers.
 */
void MXC_WDT_EnableReset(mxc_wdt_regs_t *wdt);

/**
 * @brief       Enable the watchdog interrupt.
 * @param       wdt     Pointer to watchdog registers.
 */
void MXC_WDT_DisableInt(mxc_wdt_regs_t *wdt);

/**
 * @brief       Enable the watchdog reset.
 * @param       wdt     Pointer to watchdog registers.
 */
void MXC_WDT_DisableReset(mxc_wdt_regs_t *wdt);

/**
 * @brief       Reset the watchdog timer.
 * @param       wdt     Pointer to watchdog registers.
 */
void MXC_WDT_ResetTimer(mxc_wdt_regs_t *wdt);

/**
 * @brief       Get the status of the reset flag.
 * @param       wdt     Pointer to watchdog registers.
 * @returns     1 if the previous reset was caused by the watchdog, 0 otherwise.
 */
int MXC_WDT_GetResetFlag(mxc_wdt_regs_t *wdt);

/**
 * @brief       Clears the reset flag.
 * @param       wdt     Pointer to watchdog registers.
 */
void MXC_WDT_ClearResetFlag(mxc_wdt_regs_t *wdt);

/**
 * @brief       Get the status of the interrupt flag.
 * @param       wdt     Pointer to watchdog registers.
 * @returns     1 if the interrupt is pending, 0 otherwise.
 */
int MXC_WDT_GetIntFlag(mxc_wdt_regs_t *wdt);

/**
 * @brief       Clears the interrupt flag.
 * @param       wdt     Pointer to watchdog registers.
 */
void MXC_WDT_ClearIntFlag(mxc_wdt_regs_t *wdt);

/**@} end of group wdt */

#ifdef __cplusplus
}
#endif

#endif // LIBRARIES_PERIPHDRIVERS_INCLUDE_MAX32665_WDT_H_
