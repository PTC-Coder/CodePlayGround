/**
 * @file    lpcmp_reva_regs.h
 * @brief   Registers, Bit Masks and Bit Positions for the LPCMP_REVA Peripheral Module.
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

#ifndef LIBRARIES_PERIPHDRIVERS_SOURCE_LPCMP_LPCMP_REVA_REGS_H_
#define LIBRARIES_PERIPHDRIVERS_SOURCE_LPCMP_LPCMP_REVA_REGS_H_

/* **** Includes **** */
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

#if defined (__ICCARM__)
  #pragma system_include
#endif

#if defined (__CC_ARM)
  #pragma anon_unions
#endif
/// @cond
/*
    If types are not defined elsewhere (CMSIS) define them here
*/
#ifndef __IO
#define __IO volatile
#endif
#ifndef __I
#define __I  volatile const
#endif
#ifndef __O
#define __O  volatile
#endif
#ifndef __R
#define __R  volatile const
#endif
/// @endcond

/* **** Definitions **** */

/**
 * @ingroup     lpcmp_reva
 * @defgroup    lpcmp_reva_registers LPCMP_REVA_Registers
 * @brief       Registers, Bit Masks and Bit Positions for the LPCMP_REVA Peripheral Module.
 * @details Low Power Comparator
 */

/**
 * @ingroup lpcmp_reva_registers
 * Structure type to access the LPCMP_REVA Registers.
 */
typedef struct {
    __IO uint32_t ctrl[3];              /**< <tt>\b 0x00:</tt> LPCMP_REVA CTRL Register */
} mxc_lpcmp_reva_regs_t;

/* Register offsets for module LPCMP_REVA */
/**
 * @ingroup    lpcmp_reva_registers
 * @defgroup   LPCMP_REVA_Register_Offsets Register Offsets
 * @brief      LPCMP_REVA Peripheral Register Offsets from the LPCMP_REVA Base Peripheral Address.
 * @{
 */
#define MXC_R_LPCMP_REVA_CTRL              ((uint32_t)0x00000000UL) /**< Offset from LPCMP_REVA Base Address: <tt> 0x0000</tt> */
/**@} end of group lpcmp_reva_registers */

/**
 * @ingroup  lpcmp_reva_registers
 * @defgroup LPCMP_REVA_CTRL LPCMP_REVA_CTRL
 * @brief    Comparator Control Register.
 * @{
 */
#define MXC_F_LPCMP_REVA_CTRL_EN_POS                   0 /**< CTRL_EN Position */
#define MXC_F_LPCMP_REVA_CTRL_EN                       ((uint32_t)(0x1UL << MXC_F_LPCMP_REVA_CTRL_EN_POS)) /**< CTRL_EN Mask */

#define MXC_F_LPCMP_REVA_CTRL_POL_POS                  5 /**< CTRL_POL Position */
#define MXC_F_LPCMP_REVA_CTRL_POL                      ((uint32_t)(0x1UL << MXC_F_LPCMP_REVA_CTRL_POL_POS)) /**< CTRL_POL Mask */

#define MXC_F_LPCMP_REVA_CTRL_INT_EN_POS               6 /**< CTRL_INT_EN Position */
#define MXC_F_LPCMP_REVA_CTRL_INT_EN                   ((uint32_t)(0x1UL << MXC_F_LPCMP_REVA_CTRL_INT_EN_POS)) /**< CTRL_INT_EN Mask */

#define MXC_F_LPCMP_REVA_CTRL_OUT_POS                  14 /**< CTRL_OUT Position */
#define MXC_F_LPCMP_REVA_CTRL_OUT                      ((uint32_t)(0x1UL << MXC_F_LPCMP_REVA_CTRL_OUT_POS)) /**< CTRL_OUT Mask */

#define MXC_F_LPCMP_REVA_CTRL_INT_FL_POS               15 /**< CTRL_INT_FL Position */
#define MXC_F_LPCMP_REVA_CTRL_INT_FL                   ((uint32_t)(0x1UL << MXC_F_LPCMP_REVA_CTRL_INT_FL_POS)) /**< CTRL_INT_FL Mask */

/**@} end of group LPCMP_REVA_CTRL_Register */

#ifdef __cplusplus
}
#endif

#endif  // LIBRARIES_PERIPHDRIVERS_SOURCE_LPCMP_LPCMP_REVA_REGS_H_

