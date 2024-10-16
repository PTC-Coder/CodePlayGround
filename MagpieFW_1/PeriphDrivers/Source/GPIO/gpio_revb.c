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
#include "mxc_device.h"
#include "mxc_assert.h"
#include "gpio.h"
#include "gpio_revb.h"
#include "gpio_common.h"

/* **** Functions **** */
int MXC_GPIO_RevB_Config(const mxc_gpio_cfg_t *cfg, uint8_t psMask)
{
    mxc_gpio_regs_t *gpio = cfg->port;

    // Set the GPIO type
    switch (cfg->func) {
    case MXC_GPIO_FUNC_IN:
        gpio->outen_clr = cfg->mask;
        gpio->en0_set = cfg->mask;
        gpio->en1_clr = cfg->mask;
        gpio->en2_clr = cfg->mask;
        break;

    case MXC_GPIO_FUNC_OUT:
        gpio->outen_set = cfg->mask;
        gpio->en0_set = cfg->mask;
        gpio->en1_clr = cfg->mask;
        gpio->en2_clr = cfg->mask;
        break;

    case MXC_GPIO_FUNC_ALT1:
        gpio->en0_clr = cfg->mask;
        gpio->en1_clr = cfg->mask;
        gpio->en2_clr = cfg->mask;
        break;

    case MXC_GPIO_FUNC_ALT2:
        gpio->en0_clr = cfg->mask;
        gpio->en1_set = cfg->mask;
        gpio->en2_clr = cfg->mask;
        break;

    case MXC_GPIO_FUNC_ALT3:
        gpio->en0_set = cfg->mask;
        gpio->en1_set = cfg->mask;
        gpio->en2_clr = cfg->mask;
        break;

    default:
        return E_BAD_PARAM;
    }

    // Configure the pad
    switch (cfg->pad) {
    case MXC_GPIO_PAD_NONE:
        gpio->padctrl0 &= ~cfg->mask;
        gpio->padctrl1 &= ~cfg->mask;
        if (psMask == MXC_GPIO_PS_PULL_SELECT) {
            gpio->ps &= ~cfg->mask;
        }
        break;

    case MXC_GPIO_PAD_PULL_UP:
        gpio->padctrl0 |= cfg->mask;
        gpio->padctrl1 &= ~cfg->mask;
        if (psMask == MXC_GPIO_PS_PULL_SELECT) {
            gpio->ps |= cfg->mask;
        }
        break;

    case MXC_GPIO_PAD_PULL_DOWN:
        gpio->padctrl0 &= ~cfg->mask;
        gpio->padctrl1 |= cfg->mask;
        if (psMask == MXC_GPIO_PS_PULL_SELECT) {
            gpio->ps &= ~cfg->mask;
        }
        break;

    default:
        return E_BAD_PARAM;
    }

    // Configure the vssel
    MXC_GPIO_SetVSSEL(gpio, cfg->vssel, cfg->mask);

    return E_NO_ERROR;
}

uint32_t MXC_GPIO_RevB_InGet(mxc_gpio_regs_t *port, uint32_t mask)
{
    return (port->in & mask);
}

void MXC_GPIO_RevB_OutSet(mxc_gpio_regs_t *port, uint32_t mask)
{
    port->out_set = mask;
}

void MXC_GPIO_RevB_OutClr(mxc_gpio_regs_t *port, uint32_t mask)
{
    port->out_clr = mask;
}

uint32_t MXC_GPIO_RevB_OutGet(mxc_gpio_regs_t *port, uint32_t mask)
{
    return (port->out & mask);
}

void MXC_GPIO_RevB_OutPut(mxc_gpio_regs_t *port, uint32_t mask, uint32_t val)
{
    port->out = (port->out & ~mask) | (val & mask);
}

void MXC_GPIO_RevB_OutToggle(mxc_gpio_regs_t *port, uint32_t mask)
{
    port->out ^= mask;
}

int MXC_GPIO_RevB_IntConfig(const mxc_gpio_cfg_t *cfg, mxc_gpio_int_pol_t pol)
{
    mxc_gpio_regs_t *gpio = cfg->port;

    switch (pol) {
    case MXC_GPIO_INT_HIGH:
        gpio->intpol &= ~cfg->mask;
        gpio->dualedge &= ~cfg->mask;
        gpio->intmode &= ~cfg->mask;
        break;

    case MXC_GPIO_INT_FALLING: /* MXC_GPIO_INT_HIGH */
        gpio->intpol &= ~cfg->mask;
        gpio->dualedge &= ~cfg->mask;
        gpio->intmode |= cfg->mask;
        break;

    case MXC_GPIO_INT_LOW: /* MXC_GPIO_INT_LOW */
        gpio->intpol |= cfg->mask;
        gpio->dualedge &= ~cfg->mask;
        gpio->intmode &= ~cfg->mask;
        break;

    case MXC_GPIO_INT_RISING: /* MXC_GPIO_INT_LOW */
        gpio->intpol |= cfg->mask;
        gpio->dualedge &= ~cfg->mask;
        gpio->intmode |= cfg->mask;
        break;

    case MXC_GPIO_INT_BOTH:
        gpio->dualedge |= cfg->mask;
        gpio->intmode |= cfg->mask;
        break;

    default:
        return E_BAD_PARAM;
    }

    return E_NO_ERROR;
}

void MXC_GPIO_RevB_EnableInt(mxc_gpio_regs_t *port, uint32_t mask)
{
    port->inten_set = mask;
}

void MXC_GPIO_RevB_DisableInt(mxc_gpio_regs_t *port, uint32_t mask)
{
    port->inten_clr = mask;
}

void MXC_GPIO_RevB_ClearFlags(mxc_gpio_regs_t *port, uint32_t flags)
{
    port->intfl_clr = flags;
}

uint32_t MXC_GPIO_RevB_GetFlags(mxc_gpio_regs_t *port)
{
    return port->intfl;
}

int MXC_GPIO_RevB_SetVSSEL(mxc_gpio_regs_t *port, mxc_gpio_vssel_t vssel, uint32_t mask)
{
    // Configure the vssel
    switch (vssel) {
    case MXC_GPIO_VSSEL_VDDIO:
        port->vssel &= ~mask;
        break;

    case MXC_GPIO_VSSEL_VDDIOH:
        port->vssel |= mask;
        break;

    default:
        return E_BAD_PARAM;
    }

    return E_NO_ERROR;
}
