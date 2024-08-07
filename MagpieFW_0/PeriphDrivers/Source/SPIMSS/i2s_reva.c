/**
 * @file    i2s.c
 * @brief   Inter-Integrated Sound(I2S) driver implementation.
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
#include "mxc_errors.h"
#include "mxc_device.h"
#include "mxc_assert.h"
#include "mxc_lock.h"
#include "mxc_sys.h"
#include "dma.h"
#include "i2s.h"
#include "i2s_reva.h"
#include "spimss.h"
#include "spimss_reva.h"

#define I2S_CHANNELS 2
#define I2S_WIDTH 16

static int tx_dma_channel = -1;
static int rx_dma_channel = -1;

int MXC_I2S_RevA_Init(mxc_spimss_reva_regs_t *spimss, const mxc_i2s_config_t *config,
                      void (*dma_ctz_cb)(int, int))
{
    unsigned int baud;
    uint16_t clkdiv;
    uint8_t ctz_en;
    int err;

    mxc_dma_config_t dma_config;
    mxc_dma_srcdst_t srcdst;

    /* Setup SPI_MSS as master, mode 0, 16 bit transfers as I2S Requires */
    spimss->ctrl = MXC_F_SPIMSS_REVA_CTRL_MMEN;
    spimss->mode = MXC_S_SPIMSS_REVA_MODE_NUMBITS_BITS16 | MXC_F_SPIMSS_REVA_MODE_SS_IO;

    spimss->dma = (1 << MXC_F_SPIMSS_DMA_TX_FIFO_LVL_POS) | /* TX DMA request FIFO level */
                  MXC_F_SPIMSS_DMA_TX_FIFO_CLR | /* Clear TX FIFO */
                  (1 << MXC_F_SPIMSS_DMA_RX_FIFO_LVL_POS) | /* RX DMA request FIFO level */
                  MXC_F_SPIMSS_DMA_RX_FIFO_CLR; /* Clear RX FIFO */

    /* Setup I2S register from i2s_cfg_t */
    spimss->i2s_ctrl = config->justify << MXC_F_SPIMSS_REVA_I2S_CTRL_I2S_LJ_POS |
                       config->audio_mode << MXC_F_SPIMSS_I2S_CTRL_I2S_MONO_POS;

    /* Determine divisor for baud rate generator */
    baud = config->sample_rate * I2S_CHANNELS * I2S_WIDTH;

    if ((PeripheralClock / 4) < baud) {
        return E_BAD_PARAM;
    }

    clkdiv = PeripheralClock / (2 * baud); // Peripheral clock in system_max*****.h

    if (clkdiv < 2) {
        return E_BAD_PARAM;
    }

    spimss->brg = clkdiv;

    /* Prepare SPIMSS DMA register for DMA setup */
    if (dma_ctz_cb == NULL) {
        ctz_en = 0;
    } else {
        ctz_en = 1;
    }

    /* Initialize DMA */
    if (config->audio_direction % 2) {
        spimss->dma |= MXC_F_SPIMSS_REVA_DMA_TX_DMA_EN | MXC_F_SPIMSS_REVA_DMA_TX_FIFO_CLR;
        if ((err = MXC_DMA_Init()) != E_NO_ERROR) {
            if (err != E_BAD_STATE) {
                return err;
            }
        }

        if ((err = MXC_DMA_AcquireChannel()) < 0) {
            return err;
        }

        tx_dma_channel = err;

        dma_config.ch = tx_dma_channel;

        dma_config.srcwd = MXC_DMA_WIDTH_WORD;
        dma_config.dstwd = MXC_DMA_WIDTH_HALFWORD;
#if TARGET_NUM == 32650
        dma_config.reqsel = MXC_DMA_REQUEST_SPIMSSTX;
#endif
#if TARGET_NUM == 32660
        dma_config.reqsel = MXC_DMA_REQUEST_SPI1TX;
#endif

        dma_config.srcinc_en = 1;
        dma_config.dstinc_en = 0;

        srcdst.ch = tx_dma_channel;
        srcdst.source = config->src_addr;
        srcdst.dest = NULL;
        srcdst.len = config->length;

        MXC_DMA_ConfigChannel(dma_config, srcdst);
        MXC_DMA_SetChannelInterruptEn(tx_dma_channel, 0, 1);

        MXC_DMA->ch[tx_dma_channel].cfg &= ~MXC_F_DMA_CFG_BRST;
        MXC_DMA->ch[tx_dma_channel].cfg |= (3 << MXC_F_DMA_CFG_BRST_POS);

        if (ctz_en) {
            MXC_DMA_SetCallback(tx_dma_channel, dma_ctz_cb);
            MXC_DMA_EnableInt(tx_dma_channel);
        }
    }
    if (config->audio_direction / 2) {
        spimss->dma |= MXC_F_SPIMSS_REVA_DMA_RX_DMA_EN | MXC_F_SPIMSS_REVA_DMA_RX_FIFO_CLR;
        if ((err = MXC_DMA_Init()) != E_NO_ERROR) {
            if (err != E_BAD_STATE) { //DMA already initialized
                return err;
            }
        }

        if ((err = MXC_DMA_AcquireChannel()) < 0) {
            return err;
        }

        rx_dma_channel = err;

        dma_config.ch = rx_dma_channel;

        dma_config.srcwd = MXC_DMA_WIDTH_HALFWORD;
        dma_config.dstwd = MXC_DMA_WIDTH_WORD;
#if TARGET_NUM == 32650
        dma_config.reqsel = MXC_DMA_REQUEST_SPIMSSRX;
#endif
#if TARGET_NUM == 32660
        dma_config.reqsel = MXC_DMA_REQUEST_SPI1RX;
#endif

        dma_config.srcinc_en = 0;
        dma_config.dstinc_en = 1;

        srcdst.ch = rx_dma_channel;
        srcdst.source = NULL;
        srcdst.dest = config->dst_addr;
        srcdst.len = config->length;

        MXC_DMA_ConfigChannel(dma_config, srcdst);
        MXC_DMA_SetChannelInterruptEn(rx_dma_channel, 0, 1);

        MXC_DMA->ch[rx_dma_channel].cfg &= ~MXC_F_DMA_CFG_BRST;
        MXC_DMA->ch[rx_dma_channel].cfg |= (3 << MXC_F_DMA_CFG_BRST_POS);

        if (ctz_en) {
            MXC_DMA_SetCallback(rx_dma_channel, dma_ctz_cb);
            MXC_DMA_EnableInt(rx_dma_channel);
        }
    }

    MXC_I2S_DMA_SetAddrCnt(config->src_addr, config->dst_addr, config->length);
    if (config->dma_reload_en) {
        MXC_I2S_DMA_SetReload(config->src_addr, config->dst_addr, config->length);
    }

    if (config->start_immediately) {
        return MXC_I2S_Start();
    }
    return E_NO_ERROR;
}

int MXC_I2S_RevA_Shutdown(mxc_spimss_reva_regs_t *spimss)
{
    int retTx = E_NO_ERROR;
    int retRx = E_NO_ERROR;

    spimss->ctrl = 0;
    spimss->i2s_ctrl = 0;
    spimss->brg = 0;
    spimss->mode = 0;
    spimss->dma = 0;

    if (tx_dma_channel != -1) {
        retTx = MXC_DMA_ReleaseChannel(tx_dma_channel);
    }

    if (rx_dma_channel != -1) {
        retRx = MXC_DMA_ReleaseChannel(rx_dma_channel);
    }

    if (retTx != E_NO_ERROR) {
        return retTx;
    } else {
        return retRx;
    }
}

int MXC_I2S_RevA_Mute(mxc_spimss_reva_regs_t *spimss)
{
    spimss->i2s_ctrl |= MXC_F_SPIMSS_REVA_I2S_CTRL_I2S_MUTE;
    return E_NO_ERROR;
}

int MXC_I2S_RevA_Unmute(mxc_spimss_reva_regs_t *spimss)
{
    spimss->i2s_ctrl &= ~MXC_F_SPIMSS_REVA_I2S_CTRL_I2S_MUTE;
    return E_NO_ERROR;
}

int MXC_I2S_RevA_Pause(mxc_spimss_reva_regs_t *spimss)
{
    spimss->i2s_ctrl |= MXC_F_SPIMSS_REVA_I2S_CTRL_I2S_PAUSE;
    return E_NO_ERROR;
}

int MXC_I2S_RevA_Unpause(mxc_spimss_reva_regs_t *spimss)
{
    spimss->i2s_ctrl &= ~MXC_F_SPIMSS_REVA_I2S_CTRL_I2S_PAUSE;
    return E_NO_ERROR;
}

int MXC_I2S_RevA_Stop(mxc_spimss_reva_regs_t *spimss)
{
    int retTx = E_NO_ERROR;
    int retRx = E_NO_ERROR;

    spimss->ctrl &= ~MXC_F_SPIMSS_REVA_CTRL_ENABLE;
    spimss->i2s_ctrl &= ~MXC_F_SPIMSS_REVA_I2S_CTRL_I2S_EN;

    if (tx_dma_channel != -1) {
        retTx = MXC_DMA_Stop(tx_dma_channel);
    }
    if (rx_dma_channel != -1) {
        retRx = MXC_DMA_Stop(rx_dma_channel);
    }

    if (retTx != E_NO_ERROR) {
        return retTx;
    } else {
        return retRx;
    }
}

int MXC_I2S_RevA_Start(mxc_spimss_reva_regs_t *spimss)
{
    int retTx = E_NO_ERROR;
    int retRx = E_NO_ERROR;

    spimss->ctrl |= MXC_F_SPIMSS_REVA_CTRL_ENABLE;
    spimss->i2s_ctrl |= MXC_F_SPIMSS_REVA_I2S_CTRL_I2S_EN;

    if (tx_dma_channel != -1) {
        retTx = MXC_DMA_Start(tx_dma_channel);
    }
    if (rx_dma_channel != -1) {
        retRx = MXC_DMA_Start(rx_dma_channel);
    }

    if (retTx != E_NO_ERROR) {
        return retTx;
    } else {
        return retRx;
    }
}

int MXC_I2S_RevA_DMA_ClearFlags(void)
{
    int retTx = E_NO_ERROR;
    int retRx = E_NO_ERROR;

    int flags;

    if (tx_dma_channel != -1) {
        flags = MXC_DMA_ChannelGetFlags(tx_dma_channel);
        retTx = MXC_DMA_ChannelClearFlags(tx_dma_channel, flags);
    }
    if (rx_dma_channel != -1) {
        flags = MXC_DMA_ChannelGetFlags(rx_dma_channel);
        retRx = MXC_DMA_ChannelClearFlags(rx_dma_channel, flags);
    }

    if (retTx != E_NO_ERROR) {
        return retTx;
    } else {
        return retRx;
    }
}

int MXC_I2S_RevA_DMA_SetAddrCnt(void *src_addr, void *dst_addr, unsigned int count)
{
    int retTx = E_NO_ERROR;
    int retRx = E_NO_ERROR;

    mxc_dma_srcdst_t srcdst;

    if (tx_dma_channel != -1) {
        srcdst.ch = tx_dma_channel;
        srcdst.source = src_addr;
        srcdst.dest = dst_addr;
        srcdst.len = count;
        retTx = MXC_DMA_SetSrcDst(srcdst);
    }
    if (rx_dma_channel != -1) {
        srcdst.ch = rx_dma_channel;
        srcdst.source = src_addr;
        srcdst.dest = dst_addr;
        srcdst.len = count;
        retRx = MXC_DMA_SetSrcDst(srcdst);
    }

    if (retTx != E_NO_ERROR) {
        return retTx;
    } else {
        return retRx;
    }
}

int MXC_I2S_RevA_DMA_SetReload(void *src_addr, void *dst_addr, unsigned int count)
{
    int retTx = E_NO_ERROR;
    int retRx = E_NO_ERROR;

    mxc_dma_srcdst_t srcdst;

    if (tx_dma_channel != -1) {
        srcdst.ch = tx_dma_channel;
        srcdst.source = src_addr;
        srcdst.dest = dst_addr;
        srcdst.len = count;
        retTx = MXC_DMA_SetSrcReload(srcdst);
    }
    if (rx_dma_channel != -1) {
        srcdst.ch = rx_dma_channel;
        srcdst.source = src_addr;
        srcdst.dest = dst_addr;
        srcdst.len = count;
        retRx = MXC_DMA_SetSrcReload(srcdst);
    }

    if (retTx != E_NO_ERROR) {
        return retTx;
    } else {
        return retRx;
    }
}
