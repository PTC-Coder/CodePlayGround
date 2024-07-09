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

#include "tmr.h"
#include "tmr_reva.h"
#include "tmr_common.h"

void MXC_TMR_Init(mxc_tmr_regs_t *tmr, mxc_tmr_cfg_t *cfg)
{
#ifndef MSDK_NO_GPIO_CLK_INIT
    int tmr_id = MXC_TMR_GET_IDX(tmr);
    MXC_ASSERT(tmr_id >= 0);

    //enable peripheral clock and configure gpio pins
    switch (tmr_id) {
    case 0:
        MXC_SYS_Reset_Periph(MXC_SYS_RESET_TMR0);
        MXC_SYS_ClockEnable(MXC_SYS_PERIPH_CLOCK_TMR0);
        MXC_GPIO_Config(&gpio_cfg_tmr0);
        break;

    case 1:
        MXC_SYS_Reset_Periph(MXC_SYS_RESET_TMR1);
        MXC_SYS_ClockEnable(MXC_SYS_PERIPH_CLOCK_TMR1);
        MXC_GPIO_Config(&gpio_cfg_tmr1);
        break;

    case 2:
        MXC_SYS_Reset_Periph(MXC_SYS_RESET_TMR2);
        MXC_SYS_ClockEnable(MXC_SYS_PERIPH_CLOCK_TMR2);
        MXC_GPIO_Config(&gpio_cfg_tmr2);
        break;

    case 3:
        MXC_SYS_Reset_Periph(MXC_SYS_RESET_TMR3);
        MXC_SYS_ClockEnable(MXC_SYS_PERIPH_CLOCK_TMR3);
        MXC_GPIO_Config(&gpio_cfg_tmr3);
        break;
    }
#endif

    MXC_TMR_RevA_Init((mxc_tmr_reva_regs_t *)tmr, cfg);
}

void MXC_TMR_Shutdown(mxc_tmr_regs_t *tmr)
{
    int tmr_id = MXC_TMR_GET_IDX(tmr);
    MXC_ASSERT(tmr_id >= 0);

    MXC_TMR_RevA_Shutdown((mxc_tmr_reva_regs_t *)tmr);

    // System settigns
    // disable peripheral clock
    switch (tmr_id) {
    case 0:
        MXC_SYS_ClockDisable(MXC_SYS_PERIPH_CLOCK_TMR0);
        break;

    case 1:
        MXC_SYS_ClockDisable(MXC_SYS_PERIPH_CLOCK_TMR1);
        break;

    case 2:
        MXC_SYS_ClockDisable(MXC_SYS_PERIPH_CLOCK_TMR2);
        break;

    case 3:
        MXC_SYS_ClockDisable(MXC_SYS_PERIPH_CLOCK_TMR3);
        break;
    }
}

void MXC_TMR_Start(mxc_tmr_regs_t *tmr)
{
    MXC_TMR_RevA_Start((mxc_tmr_reva_regs_t *)tmr);
}

void MXC_TMR_Stop(mxc_tmr_regs_t *tmr)
{
    MXC_TMR_RevA_Stop((mxc_tmr_reva_regs_t *)tmr);
}

int MXC_TMR_SetPWM(mxc_tmr_regs_t *tmr, uint32_t pwm)
{
    return MXC_TMR_RevA_SetPWM((mxc_tmr_reva_regs_t *)tmr, pwm);
}

uint32_t MXC_TMR_GetCompare(mxc_tmr_regs_t *tmr)
{
    return MXC_TMR_RevA_GetCompare((mxc_tmr_reva_regs_t *)tmr);
}

uint32_t MXC_TMR_GetCapture(mxc_tmr_regs_t *tmr)
{
    return MXC_TMR_RevA_GetCapture((mxc_tmr_reva_regs_t *)tmr);
}

uint32_t MXC_TMR_GetCount(mxc_tmr_regs_t *tmr)
{
    return MXC_TMR_RevA_GetCount((mxc_tmr_reva_regs_t *)tmr);
}

void MXC_TMR_ClearFlags(mxc_tmr_regs_t *tmr)
{
    MXC_TMR_RevA_ClearFlags((mxc_tmr_reva_regs_t *)tmr);
}

uint32_t MXC_TMR_GetFlags(mxc_tmr_regs_t *tmr)
{
    return MXC_TMR_RevA_GetFlags((mxc_tmr_reva_regs_t *)tmr);
}

void MXC_TMR_SetCompare(mxc_tmr_regs_t *tmr, uint32_t cmp_cnt)
{
    MXC_TMR_RevA_SetCompare((mxc_tmr_reva_regs_t *)tmr, cmp_cnt);
}

void MXC_TMR_SetCount(mxc_tmr_regs_t *tmr, uint32_t cnt)
{
    MXC_TMR_RevA_SetCount((mxc_tmr_reva_regs_t *)tmr, cnt);
}

void MXC_TMR_Delay(mxc_tmr_regs_t *tmr, uint32_t us)
{
    MXC_TMR_Common_Delay(tmr, us);
}

void MXC_TMR_TO_Start(mxc_tmr_regs_t *tmr, uint32_t us)
{
    MXC_TMR_RevA_TO_Start((mxc_tmr_reva_regs_t *)tmr, us);
}

int MXC_TMR_TO_Check(mxc_tmr_regs_t *tmr)
{
    return MXC_TMR_Common_TO_Check(tmr);
}

void MXC_TMR_TO_Stop(mxc_tmr_regs_t *tmr)
{
    MXC_TMR_Common_TO_Stop(tmr);
}

void MXC_TMR_TO_Clear(mxc_tmr_regs_t *tmr)
{
    MXC_TMR_Common_TO_Clear(tmr);
}

unsigned int MXC_TMR_TO_Elapsed(mxc_tmr_regs_t *tmr)
{
    return MXC_TMR_Common_TO_Elapsed(tmr);
}

unsigned int MXC_TMR_TO_Remaining(mxc_tmr_regs_t *tmr)
{
    return MXC_TMR_Common_TO_Remaining(tmr);
}

void MXC_TMR_SW_Start(mxc_tmr_regs_t *tmr)
{
    return MXC_TMR_Common_SW_Start(tmr);
}

unsigned int MXC_TMR_SW_Stop(mxc_tmr_regs_t *tmr)
{
    return MXC_TMR_Common_SW_Stop(tmr);
}

int MXC_TMR_GetTime(mxc_tmr_regs_t *tmr, uint32_t ticks, uint32_t *time, mxc_tmr_unit_t *units)
{
    return MXC_TMR_RevA_GetTime((mxc_tmr_reva_regs_t *)tmr, ticks, time, units);
}
