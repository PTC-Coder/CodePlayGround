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

/* **** Includes **** */
#include <stddef.h>
#include "mxc_errors.h"
#include "mxc_assert.h"
#include "mxc_sys.h"
#include "gpio.h"
#include "gpio_common.h"
#include "gpio_reva.h"

/* **** Definitions **** */

/* **** Globals **** */

// static void (*callback[MXC_CFG_GPIO_INSTANCES][MXC_CFG_GPIO_PINS_PORT])(void *);
// static void *cbparam[MXC_CFG_GPIO_INSTANCES][MXC_CFG_GPIO_PINS_PORT];

/* **** Functions **** */

int MXC_GPIO_Init(uint32_t port)
{
    if (port == MXC_GPIO_PORT_0) {
        MXC_SYS_ClockEnable(MXC_SYS_PERIPH_CLOCK_GPIO0);
    } else if (port == MXC_GPIO_PORT_1) {
        MXC_SYS_ClockEnable(MXC_SYS_PERIPH_CLOCK_GPIO1);
    } else if (port == MXC_GPIO_PORT_2) {
        MXC_SYS_ClockEnable(MXC_SYS_PERIPH_CLOCK_GPIO2);
    } else if (port == MXC_GPIO_PORT_3) {
        MXC_SYS_ClockEnable(MXC_SYS_PERIPH_CLOCK_GPIO3);
    } else {
        return E_BAD_PARAM;
    }
    return MXC_GPIO_Common_Init(port);
}

/* ************************************************************************** */
int MXC_GPIO_Shutdown(uint32_t port)
{
    if (port == MXC_GPIO_PORT_0) {
        MXC_SYS_ClockDisable(MXC_SYS_PERIPH_CLOCK_GPIO0);
    } else if (port == MXC_GPIO_PORT_1) {
        MXC_SYS_ClockDisable(MXC_SYS_PERIPH_CLOCK_GPIO1);
    } else if (port == MXC_GPIO_PORT_2) {
        MXC_SYS_ClockDisable(MXC_SYS_PERIPH_CLOCK_GPIO2);
    } else if (port == MXC_GPIO_PORT_3) {
        MXC_SYS_ClockDisable(MXC_SYS_PERIPH_CLOCK_GPIO3);
    } else {
        return E_BAD_PARAM;
    }

    return E_NO_ERROR;
}

/* ************************************************************************** */
int MXC_GPIO_Reset(uint32_t port)
{
    if (port == MXC_GPIO_PORT_0) {
        MXC_SYS_Reset_Periph(MXC_SYS_RESET_GPIO0);
    } else if (port == MXC_GPIO_PORT_1) {
        MXC_SYS_Reset_Periph(MXC_SYS_RESET_GPIO1);
    } else if (port == MXC_GPIO_PORT_2) {
        MXC_SYS_Reset_Periph(MXC_SYS_RESET_GPIO2);
    } else if (port == MXC_GPIO_PORT_3) {
        MXC_SYS_Reset_Periph(MXC_SYS_RESET_GPIO3);
    } else {
        return E_BAD_PARAM;
    }

    return E_NO_ERROR;
}

/* ************************************************************************** */
/*
 *       GPIO_EN2 |  GPIO_EN1           |  GPIO_EN            |   Function
 *  --------------|---------------------|---------------------|----------------------
 *     0          |          0          |          0          |     Alternative 1
 *     0          |          1          |          0          |     Alternative 2
 *     0          |          0          |          1          |     GPIO (default)
*/
int MXC_GPIO_Config(const mxc_gpio_cfg_t *cfg)
{
    int err;
    mxc_gpio_regs_t *gpio = cfg->port;

    // Configure the vssel
    err = MXC_GPIO_SetVSSEL(gpio, cfg->vssel, cfg->mask);
    if (err != E_NO_ERROR) {
        return err;
    }

    // Set the GPIO type
    if ((err = MXC_GPIO_RevA_SetAF((mxc_gpio_reva_regs_t *)gpio, cfg->func, cfg->mask)) !=
        E_NO_ERROR) {
        return err;
    }

    // Configure the pad
    switch (cfg->pad) {
    case MXC_GPIO_PAD_NONE:
        gpio->pdpu_sel0 &= ~cfg->mask;
        gpio->pdpu_sel1 &= ~cfg->mask;
        gpio->pssel &= ~cfg->mask;
        break;
    case MXC_GPIO_PAD_WEAK_PULL_UP:
        gpio->pdpu_sel0 |= cfg->mask;
        gpio->pdpu_sel1 &= ~cfg->mask;
        gpio->pssel |= cfg->mask;
        break;
    case MXC_GPIO_PAD_WEAK_PULL_DOWN:
        gpio->pdpu_sel0 &= ~cfg->mask;
        gpio->pdpu_sel1 |= cfg->mask;
        gpio->pssel |= cfg->mask;
        break;
    case MXC_GPIO_PAD_STRONG_PULL_UP:
        gpio->pdpu_sel0 |= cfg->mask;
        gpio->pdpu_sel1 &= ~cfg->mask;
        gpio->pssel &= ~cfg->mask;
        break;
    case MXC_GPIO_PAD_STRONG_PULL_DOWN:
        gpio->pdpu_sel0 &= ~cfg->mask;
        gpio->pdpu_sel1 |= cfg->mask;
        gpio->pssel &= ~cfg->mask;
        break;
    default:
        return E_BAD_PARAM;
    }

    // Configure the drive strength
    if (cfg->func == MXC_GPIO_FUNC_IN) {
        return E_NO_ERROR;
    } else {
        return MXC_GPIO_SetDriveStrength(gpio, cfg->drvstr, cfg->mask);
    }
}

/* ************************************************************************** */
uint32_t MXC_GPIO_InGet(mxc_gpio_regs_t *port, uint32_t mask)
{
    return MXC_GPIO_RevA_InGet((mxc_gpio_reva_regs_t *)port, mask);
}

/* ************************************************************************** */
void MXC_GPIO_OutSet(mxc_gpio_regs_t *port, uint32_t mask)
{
    MXC_GPIO_RevA_OutSet((mxc_gpio_reva_regs_t *)port, mask);
}

/* ************************************************************************** */
void MXC_GPIO_OutClr(mxc_gpio_regs_t *port, uint32_t mask)
{
    MXC_GPIO_RevA_OutClr((mxc_gpio_reva_regs_t *)port, mask);
}

/* ************************************************************************** */
uint32_t MXC_GPIO_OutGet(mxc_gpio_regs_t *port, uint32_t mask)
{
    return MXC_GPIO_RevA_OutGet((mxc_gpio_reva_regs_t *)port, mask);
}

/* ************************************************************************** */
void MXC_GPIO_OutPut(mxc_gpio_regs_t *port, uint32_t mask, uint32_t val)
{
    return MXC_GPIO_RevA_OutPut((mxc_gpio_reva_regs_t *)port, mask, val);
}

/* ************************************************************************** */
void MXC_GPIO_OutToggle(mxc_gpio_regs_t *port, uint32_t mask)
{
    MXC_GPIO_RevA_OutToggle((mxc_gpio_reva_regs_t *)port, mask);
}

/* ************************************************************************** */
int MXC_GPIO_IntConfig(const mxc_gpio_cfg_t *cfg, mxc_gpio_int_pol_t pol)
{
    return MXC_GPIO_RevA_IntConfig(cfg, pol);
}

/* ************************************************************************** */
void MXC_GPIO_EnableInt(mxc_gpio_regs_t *port, uint32_t mask)
{
    MXC_GPIO_RevA_EnableInt((mxc_gpio_reva_regs_t *)port, mask);
}

/* ************************************************************************** */
void MXC_GPIO_DisableInt(mxc_gpio_regs_t *port, uint32_t mask)
{
    MXC_GPIO_RevA_DisableInt((mxc_gpio_reva_regs_t *)port, mask);
}

/* ************************************************************************** */
uint32_t MXC_GPIO_GetFlags(mxc_gpio_regs_t *port)
{
    return MXC_GPIO_RevA_GetFlags((mxc_gpio_reva_regs_t *)port);
}

/* ************************************************************************** */
void MXC_GPIO_ClearFlags(mxc_gpio_regs_t *port, uint32_t flags)
{
    MXC_GPIO_RevA_ClearFlags((mxc_gpio_reva_regs_t *)port, flags);
}

/* ************************************************************************** */
void MXC_GPIO_RegisterCallback(const mxc_gpio_cfg_t *cfg, mxc_gpio_callback_fn func, void *cbdata)
{
    MXC_GPIO_Common_RegisterCallback(cfg, func, cbdata);
}

/* ************************************************************************** */
void MXC_GPIO_Handler(unsigned int port)
{
    MXC_GPIO_Common_Handler(port);
}

/* ************************************************************************** */
int MXC_GPIO_SetVSSEL(mxc_gpio_regs_t *port, mxc_gpio_vssel_t vssel, uint32_t mask)
{
    return MXC_GPIO_RevA_SetVSSEL((mxc_gpio_reva_regs_t *)port, vssel, mask);
}

/* ************************************************************************** */
void MXC_GPIO_SetWakeEn(mxc_gpio_regs_t *port, uint32_t mask)
{
    MXC_GPIO_RevA_SetWakeEn((mxc_gpio_reva_regs_t *)port, mask);
}

/* ************************************************************************** */
void MXC_GPIO_ClearWakeEn(mxc_gpio_regs_t *port, uint32_t mask)
{
    MXC_GPIO_RevA_ClearWakeEn((mxc_gpio_reva_regs_t *)port, mask);
}

/* ************************************************************************** */
uint32_t MXC_GPIO_GetWakeEn(mxc_gpio_regs_t *port)
{
    return MXC_GPIO_RevA_GetWakeEn((mxc_gpio_reva_regs_t *)port);
}

/* ************************************************************************** */
int MXC_GPIO_SetDriveStrength(mxc_gpio_regs_t *port, mxc_gpio_drvstr_t drvstr, uint32_t mask)
{
    return MXC_GPIO_RevA_SetDriveStrength((mxc_gpio_reva_regs_t *)port, drvstr, mask);
}
