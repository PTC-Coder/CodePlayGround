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

#ifndef LIBRARIES_PERIPHDRIVERS_SOURCE_SFE_SFE_REVA_H_
#define LIBRARIES_PERIPHDRIVERS_SOURCE_SFE_SFE_REVA_H_

#include <stdio.h>
#include <stddef.h>
#include <stdint.h>
#include "mxc_device.h"
#include "mxc_assert.h"
#include "mxc_sys.h"
#include "mxc_delay.h"
#include "sfe.h"
#include "sfe_reva_regs.h"

int MXC_SFE_RevA_Init(void);
int MXC_SFE_RevA_Shutdown(void);
int MXC_SFE_RevA_ReadEnable(mxc_sfe_reva_regs_t *sfe);
int MXC_SFE_RevA_WriteEnable(mxc_sfe_reva_regs_t *sfe);
int MXC_SFE_RevA_SetFlashAddress(mxc_sfe_reva_regs_t *sfe, uint32_t baseAdd, uint32_t topAdd);
int MXC_SFE_RevA_SetRAMAddress(mxc_sfe_reva_regs_t *sfe, uint32_t baseAdd, uint32_t topAdd);
int MXC_SFE_RevA_SetHostAddress(mxc_sfe_reva_regs_t *sfe, uint32_t RAMAdd, uint32_t FLASHAdd);

#endif // LIBRARIES_PERIPHDRIVERS_SOURCE_SFE_SFE_REVA_H_
