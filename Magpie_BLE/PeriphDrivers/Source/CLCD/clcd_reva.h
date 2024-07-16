/**
 * @file    clcd_reva.h
 * @brief   Color LCD function prototypes and data types.
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
#ifndef LIBRARIES_PERIPHDRIVERS_SOURCE_CLCD_CLCD_REVA_H_
#define LIBRARIES_PERIPHDRIVERS_SOURCE_CLCD_CLCD_REVA_H_

/* **** Includes **** */
#include "clcd_reva_regs.h"
#include "clcd.h"
#include "mxc_sys.h"

#ifdef __cplusplus
extern "C" {
#endif

/* **** Definitions **** */

/* **** Function Prototypes **** */
int MXC_CLCD_RevA_Init(mxc_clcd_reva_regs_t *clcd, mxc_clcd_cfg_t *cfg);
int MXC_CLCD_RevA_Shutdown(mxc_clcd_reva_regs_t *clcd);
int MXC_CLCD_RevA_ConfigPanel(mxc_clcd_reva_regs_t *clcd, mxc_clcd_cfg_t *clcd_cfg);
int MXC_CLCD_RevA_Enable(mxc_clcd_reva_regs_t *clcd);
int MXC_CLCD_RevA_Disable(mxc_clcd_reva_regs_t *clcd);
int MXC_CLCD_RevA_SetFrameAddr(mxc_clcd_reva_regs_t *clcd, void *addr);

#ifdef __cplusplus
}
#endif

#endif // LIBRARIES_PERIPHDRIVERS_SOURCE_CLCD_CLCD_REVA_H_
