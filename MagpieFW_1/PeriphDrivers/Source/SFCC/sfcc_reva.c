/******************************************************************************
 *
 * Copyright (C) 2024 Analog Devices, Inc. All Rights Reserved. This software
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

/* **** Includes **** */
#include <stddef.h>
#include "mxc_device.h"
#include "mxc_errors.h"
#include "mxc_assert.h"
#include "mxc_sys.h"
#include "sfcc.h"
#include "sfcc_reva.h"
#include "sfcc_reva_regs.h"

/* **** Definitions **** */

/* **** Globals **** */

/* **** Functions **** */
static int MXC_SFCC_Ready(mxc_sfcc_reva_regs_t *sfcc)
{
    return (sfcc->ctrl & MXC_F_SFCC_REVA_CTRL_RDY);
}

int MXC_SFCC_RevA_ID(mxc_sfcc_reva_regs_t *sfcc, mxc_sfcc_info_t cid)
{
    if (sfcc == NULL) {
        return E_NULL_PTR;
    }

    switch (cid) {
    case SFCC_INFO_RELNUM:
        return ((sfcc->info & MXC_F_SFCC_REVA_INFO_RELNUM) >> MXC_F_SFCC_REVA_INFO_RELNUM_POS);

    case SFCC_INFO_PARTNUM:
        return ((sfcc->info & MXC_F_SFCC_REVA_INFO_PARTNUM) >> MXC_F_SFCC_REVA_INFO_PARTNUM_POS);

    case SFCC_INFO_ID:
        return ((sfcc->info & MXC_F_SFCC_REVA_INFO_ID) >> MXC_F_SFCC_REVA_INFO_ID_POS);

    default:
        return E_BAD_PARAM;
    }
}

void MXC_SFCC_RevA_Enable(mxc_sfcc_reva_regs_t *sfcc)
{
    // Invalidate cache and wait until ready
    sfcc->ctrl &= ~MXC_F_SFCC_REVA_CTRL_EN;
    sfcc->invalidate = 1;

    while (!(MXC_SFCC_Ready(sfcc))) {}

    // Enable Cache
    sfcc->ctrl |= MXC_F_SFCC_REVA_CTRL_EN;
    while (!(MXC_SFCC_Ready(sfcc))) {}
}

void MXC_SFCC_RevA_Disable(mxc_sfcc_reva_regs_t *sfcc)
{
    // Disable Cache
    sfcc->ctrl &= ~MXC_F_SFCC_REVA_CTRL_EN;
}
