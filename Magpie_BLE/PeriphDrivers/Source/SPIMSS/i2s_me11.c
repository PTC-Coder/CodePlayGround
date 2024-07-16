/**
 * @file    i2s.c
 * @brief   Inter-Integrated Sound (I2S) driver implementation.
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

#include <stddef.h>
#include <stdint.h>
#include "mxc_device.h"
#include "mxc_assert.h"
#include "mxc_lock.h"
#include "mxc_sys.h"
#include "mxc_pins.h"
#include "dma.h"
#include "i2s.h"
#include "i2s_reva.h"
#include "spimss.h"
#include "spimss_regs.h"

#define I2S_CHANNELS 2
#define I2S_WIDTH 16

int dma_channel = -1;

int MXC_I2S_Init(const mxc_i2s_config_t *config, void (*dma_ctz_cb)(int, int))
{
#ifndef MSDK_NO_GPIO_CLK_INIT
    if (config->map == I2S_MAP_A) {
        MXC_GPIO_Config(&gpio_cfg_spi1a); // SPIMSS: I2S and SPI share pins
    } else if (config->map == I2S_MAP_B) {
        MXC_GPIO_Config(&gpio_cfg_spi1b);
    } else {
        return E_BAD_PARAM;
    }

    MXC_SYS_ClockEnable(MXC_SYS_PERIPH_CLOCK_SPI1); // SPI1 clock used for SPIMSS
#endif

    return MXC_I2S_RevA_Init((mxc_spimss_reva_regs_t *)MXC_SPIMSS, config, dma_ctz_cb);
}

int MXC_I2S_Shutdown(void)
{
    MXC_SYS_ClockDisable(MXC_SYS_PERIPH_CLOCK_SPI1); // SPI1 used for SPIMSS
    return MXC_I2S_RevA_Shutdown((mxc_spimss_reva_regs_t *)MXC_SPIMSS);
}

int MXC_I2S_Mute(void)
{
    return MXC_I2S_RevA_Mute((mxc_spimss_reva_regs_t *)MXC_SPIMSS);
}

int MXC_I2S_Unmute(void)
{
    return MXC_I2S_RevA_Unmute((mxc_spimss_reva_regs_t *)MXC_SPIMSS);
}

int MXC_I2S_Pause(void)
{
    return MXC_I2S_RevA_Pause((mxc_spimss_reva_regs_t *)MXC_SPIMSS);
}

int MXC_I2S_Unpause(void)
{
    return MXC_I2S_RevA_Unpause((mxc_spimss_reva_regs_t *)MXC_SPIMSS);
}

int MXC_I2S_Stop(void)
{
    return MXC_I2S_RevA_Stop((mxc_spimss_reva_regs_t *)MXC_SPIMSS);
}

int MXC_I2S_Start(void)
{
    return MXC_I2S_RevA_Start((mxc_spimss_reva_regs_t *)MXC_SPIMSS);
}

int MXC_I2S_DMA_ClearFlags(void)
{
    return MXC_I2S_RevA_DMA_ClearFlags();
}

int MXC_I2S_DMA_SetAddrCnt(void *src_addr, void *dst_addr, unsigned int count)
{
    return MXC_I2S_RevA_DMA_SetAddrCnt(src_addr, dst_addr, count);
}

int MXC_I2S_DMA_SetReload(void *src_addr, void *dst_addr, unsigned int count)
{
    return MXC_I2S_RevA_DMA_SetReload(src_addr, dst_addr, count);
}
