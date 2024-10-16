/******************************************************************************
 *
 * Copyright (C) 2022-2023 Maxim Integrated Products, Inc. All Rights Reserved.
 * (now owned by Analog Devices, Inc.),
 * Copyright (C) 2023-2024 Analog Devices, Inc. All Rights Reserved. This software
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

#include <stdio.h>
#include "mxc_device.h"
#include "mxc_assert.h"
#include "uart.h"
#include "uart_revb.h"
#include "dma.h"
#ifdef __arm__
#include "nvic_table.h"
#endif

/* **** Definitions **** */
#define MXC_UART_REVB_ERRINT_EN \
    (MXC_F_UART_REVB_INT_EN_RX_FERR | MXC_F_UART_REVB_INT_EN_RX_PAR | MXC_F_UART_REVB_INT_EN_RX_OV)

#define MXC_UART_REVB_ERRINT_FL \
    (MXC_F_UART_REVB_INT_FL_RX_FERR | MXC_F_UART_REVB_INT_FL_RX_PAR | MXC_F_UART_REVB_INT_FL_RX_OV)

/* **** Variable Declaration **** */
static void *AsyncTxRequests[MXC_UART_INSTANCES];
static void *AsyncRxRequests[MXC_UART_INSTANCES];

typedef struct {
    mxc_uart_revb_req_t *tx_req;
    mxc_uart_revb_req_t *rx_req;
    int channelTx;
    int channelRx;
    bool auto_dma_handlers;
} uart_revb_req_state_t;

uart_revb_req_state_t states[MXC_UART_INSTANCES];

/* **** Function Prototypes **** */

/* ************************************************************************* */
/* Control/Configuration functions                                           */
/* ************************************************************************* */
int MXC_UART_RevB_Init(mxc_uart_revb_regs_t *uart, unsigned int baud, mxc_uart_revb_clock_t clock)
{
    int err;

    MXC_ASSERT(MXC_UART_GET_IDX((mxc_uart_regs_t *)uart) >= 0)

    // Initialize UART
    if ((err = MXC_UART_SetRXThreshold((mxc_uart_regs_t *)uart, 1)) !=
        E_NO_ERROR) { // Set RX threshold to 1 byte
        return err;
    }

    if ((err = MXC_UART_SetDataSize((mxc_uart_regs_t *)uart, 8)) !=
        E_NO_ERROR) { // Set Datasize to 8 bits
        return err;
    }

    if ((err = MXC_UART_SetParity((mxc_uart_regs_t *)uart, MXC_UART_PARITY_DISABLE)) !=
        E_NO_ERROR) {
        return err;
    }

    if ((err = MXC_UART_SetStopBits((mxc_uart_regs_t *)uart, MXC_UART_STOP_1)) != E_NO_ERROR) {
        return err;
    }

    if ((err = MXC_UART_SetFrequency((mxc_uart_regs_t *)uart, baud, (mxc_uart_clock_t)clock)) <
        E_NO_ERROR) {
        return err;
    }

    // Initialize state struct
    unsigned int i = MXC_UART_GET_IDX((mxc_uart_regs_t *)uart);
    states[i].channelRx = -1;
    states[i].channelTx = -1;
    states[i].tx_req = NULL;
    states[i].rx_req = NULL;
    states[i].auto_dma_handlers = false;

    return E_NO_ERROR;
}

int MXC_UART_RevB_ReadyForSleep(mxc_uart_revb_regs_t *uart)
{
    if (AsyncTxRequests[MXC_UART_GET_IDX((mxc_uart_regs_t *)uart)] != NULL) {
        return E_BUSY;
    }

    if (AsyncRxRequests[MXC_UART_GET_IDX((mxc_uart_regs_t *)uart)] != NULL) {
        return E_BUSY;
    }

    return MXC_UART_GetActive((mxc_uart_regs_t *)uart);
}

int MXC_UART_RevB_SetFrequency(mxc_uart_revb_regs_t *uart, unsigned int baud,
                               mxc_uart_revb_clock_t clock)
{
    unsigned clkDiv = 0, mod = 0;

    // OSR default value
    uart->osr = 5;

    switch (clock) {
    case MXC_UART_REVB_APB_CLK:
        clkDiv = (PeripheralClock / baud);
        mod = (PeripheralClock % baud);
        break;

    case MXC_UART_REVB_EXT_CLK:
        uart->ctrl |= MXC_S_UART_REVB_CTRL_BCLKSRC_EXTERNAL_CLOCK;
        clkDiv = UART_EXTCLK_FREQ / baud;
        mod = UART_EXTCLK_FREQ % baud;
        break;

    //case MXC_UART_IBRO_CLK:
    case MXC_UART_REVB_CLK2:
        clkDiv = (IBRO_FREQ / baud);
        mod = (IBRO_FREQ % baud);

        uart->ctrl |= MXC_S_UART_REVB_CTRL_BCLKSRC_CLK2;
        break;

    //case MXC_UART_ERFO:
    case MXC_UART_REVB_CLK3:
#if (TARGET_NUM == 78000 || TARGET_NUM == 78002)
        return E_BAD_PARAM;
#else
        clkDiv = (ERFO_FREQ / baud);
        mod = (ERFO_FREQ % baud);
#endif

        uart->ctrl |= MXC_S_UART_REVB_CTRL_BCLKSRC_CLK3;
        break;

    default:
        return E_BAD_PARAM;
    }

    if (!clkDiv || mod > (baud / 2)) {
        clkDiv++;
    }
    uart->clkdiv = clkDiv;
    return MXC_UART_GetFrequency((mxc_uart_regs_t *)uart);
}

int MXC_UART_RevB_GetFrequency(mxc_uart_revb_regs_t *uart)
{
    int periphClock = 0;

    if ((uart->ctrl & MXC_F_UART_REVB_CTRL_BCLKSRC) ==
        MXC_S_UART_REVB_CTRL_BCLKSRC_EXTERNAL_CLOCK) {
        periphClock = UART_EXTCLK_FREQ;
    } else if ((uart->ctrl & MXC_F_UART_REVB_CTRL_BCLKSRC) ==
               MXC_S_UART_REVB_CTRL_BCLKSRC_PERIPHERAL_CLOCK) {
        periphClock = PeripheralClock;
    } else if ((uart->ctrl & MXC_F_UART_REVB_CTRL_BCLKSRC) == MXC_S_UART_REVB_CTRL_BCLKSRC_CLK2) {
        periphClock = IBRO_FREQ;
    } else if ((uart->ctrl & MXC_F_UART_REVB_CTRL_BCLKSRC) == MXC_S_UART_REVB_CTRL_BCLKSRC_CLK3) {
#if (TARGET_NUM == 78000 || TARGET_NUM == 78002)
        return E_BAD_PARAM;
#else
        periphClock = ERFO_FREQ;
#endif
    } else {
        return E_BAD_PARAM;
    }

    return (periphClock / uart->clkdiv);
}

int MXC_UART_RevB_SetDataSize(mxc_uart_revb_regs_t *uart, int dataSize)
{
    if (dataSize < 5 || dataSize > 8) {
        return E_BAD_PARAM;
    }

    dataSize = (dataSize - 5) << MXC_F_UART_REVB_CTRL_CHAR_SIZE_POS;

    MXC_SETFIELD(uart->ctrl, MXC_F_UART_REVB_CTRL_CHAR_SIZE, dataSize);

    return E_NO_ERROR;
}

int MXC_UART_RevB_SetStopBits(mxc_uart_revb_regs_t *uart, mxc_uart_stop_t stopBits)
{
    switch (stopBits) {
    case MXC_UART_STOP_1:
        MXC_SETFIELD(uart->ctrl, MXC_F_UART_REVB_CTRL_STOPBITS,
                     0 << MXC_F_UART_REVB_CTRL_STOPBITS_POS);
        break;

    case MXC_UART_STOP_2:
        MXC_SETFIELD(uart->ctrl, MXC_F_UART_REVB_CTRL_STOPBITS,
                     1 << MXC_F_UART_REVB_CTRL_STOPBITS_POS);
        break;

    default:
        return E_BAD_PARAM;
        break;
    }

    return E_NO_ERROR;
}

int MXC_UART_RevB_SetParity(mxc_uart_revb_regs_t *uart, mxc_uart_parity_t parity)
{
    switch (parity) {
    case MXC_UART_PARITY_DISABLE:
        MXC_SETFIELD(uart->ctrl, MXC_F_UART_REVB_CTRL_PAR_EN, 0 << MXC_F_UART_REVB_CTRL_PAR_EN_POS);
        break;

    case MXC_UART_PARITY_EVEN_0:
        MXC_SETFIELD(uart->ctrl, MXC_F_UART_REVB_CTRL_PAR_EN, 1 << MXC_F_UART_REVB_CTRL_PAR_EN_POS);
        MXC_SETFIELD(uart->ctrl, MXC_F_UART_REVB_CTRL_PAR_EO, 0 << MXC_F_UART_REVB_CTRL_PAR_EO_POS);
        MXC_SETFIELD(uart->ctrl, MXC_F_UART_REVB_CTRL_PAR_MD, 0 << MXC_F_UART_REVB_CTRL_PAR_MD_POS);
        break;

    case MXC_UART_PARITY_EVEN_1:
        MXC_SETFIELD(uart->ctrl, MXC_F_UART_REVB_CTRL_PAR_EN, 1 << MXC_F_UART_REVB_CTRL_PAR_EN_POS);
        MXC_SETFIELD(uart->ctrl, MXC_F_UART_REVB_CTRL_PAR_EO, 0 << MXC_F_UART_REVB_CTRL_PAR_EO_POS);
        MXC_SETFIELD(uart->ctrl, MXC_F_UART_REVB_CTRL_PAR_MD, 1 << MXC_F_UART_REVB_CTRL_PAR_MD_POS);
        break;

    case MXC_UART_PARITY_ODD_0:
        MXC_SETFIELD(uart->ctrl, MXC_F_UART_REVB_CTRL_PAR_EN, 1 << MXC_F_UART_REVB_CTRL_PAR_EN_POS);
        MXC_SETFIELD(uart->ctrl, MXC_F_UART_REVB_CTRL_PAR_EO, 1 << MXC_F_UART_REVB_CTRL_PAR_EO_POS);
        MXC_SETFIELD(uart->ctrl, MXC_F_UART_REVB_CTRL_PAR_MD, 0 << MXC_F_UART_REVB_CTRL_PAR_MD_POS);
        break;

    case MXC_UART_PARITY_ODD_1:
        MXC_SETFIELD(uart->ctrl, MXC_F_UART_REVB_CTRL_PAR_EN, 1 << MXC_F_UART_REVB_CTRL_PAR_EN_POS);
        MXC_SETFIELD(uart->ctrl, MXC_F_UART_REVB_CTRL_PAR_EO, 1 << MXC_F_UART_REVB_CTRL_PAR_EO_POS);
        MXC_SETFIELD(uart->ctrl, MXC_F_UART_REVB_CTRL_PAR_MD, 1 << MXC_F_UART_REVB_CTRL_PAR_MD_POS);
        break;

    default:
        return E_BAD_PARAM;
        break;
    }

    return E_NO_ERROR;
}

int MXC_UART_RevB_SetFlowCtrl(mxc_uart_revb_regs_t *uart, mxc_uart_flow_t flowCtrl,
                              int rtsThreshold)
{
    switch (flowCtrl) {
    case MXC_UART_FLOW_DIS:
        MXC_SETFIELD(uart->ctrl, MXC_F_UART_REVB_CTRL_HFC_EN, 0 << MXC_F_UART_REVB_CTRL_HFC_EN_POS);
        break;

    case MXC_UART_FLOW_EN:
        MXC_SETFIELD(uart->ctrl, MXC_F_UART_REVB_CTRL_HFC_EN, 1 << MXC_F_UART_REVB_CTRL_HFC_EN_POS);
        break;

    default:
        return E_BAD_PARAM;
        break;
    }

    //FIXME: Fix missing code for CTS threshhold.

    return E_NO_ERROR;
}

int MXC_UART_RevB_SetClockSource(mxc_uart_revb_regs_t *uart, mxc_uart_revb_clock_t clock)
{
    switch (clock) {
    case MXC_UART_REVB_APB_CLK:
        break;

    case MXC_UART_REVB_EXT_CLK:
        uart->ctrl |= MXC_S_UART_REVB_CTRL_BCLKSRC_EXTERNAL_CLOCK;
        break;

    case MXC_UART_REVB_CLK2:
        uart->ctrl |= MXC_S_UART_REVB_CTRL_BCLKSRC_CLK2;
        break;

    case MXC_UART_REVB_CLK3:
        uart->ctrl |= MXC_S_UART_REVB_CTRL_BCLKSRC_CLK3;
        break;

    default:
        return E_BAD_PARAM;
    }

    return E_NO_ERROR;
}

int MXC_UART_RevB_GetActive(mxc_uart_revb_regs_t *uart)
{
    if (uart->status & (MXC_F_UART_REVB_STATUS_TX_BUSY | MXC_F_UART_REVB_STATUS_RX_BUSY)) {
        return E_BUSY;
    }

    return E_NO_ERROR;
}

int MXC_UART_RevB_AbortTransmission(mxc_uart_revb_regs_t *uart)
{
    MXC_UART_ClearTXFIFO((mxc_uart_regs_t *)uart);
    int uart_num = MXC_UART_GET_IDX((mxc_uart_regs_t *)uart);

    if (states[uart_num].channelTx >= 0) {
        MXC_DMA_Stop(states[uart_num].channelTx);
    }
    if (states[uart_num].channelRx >= 0) {
        MXC_DMA_Stop(states[uart_num].channelRx);
    }

    if (states[uart_num].auto_dma_handlers) {
        MXC_DMA_ReleaseChannel(states[uart_num].channelTx);
        states[uart_num].channelTx = -1;
        MXC_DMA_ReleaseChannel(states[uart_num].channelRx);
        states[uart_num].channelRx = -1;
    }

    return E_NO_ERROR;
}

int MXC_UART_RevB_ReadCharacterRaw(mxc_uart_revb_regs_t *uart)
{
    if (uart->status & MXC_F_UART_REVB_STATUS_RX_EM) {
        return E_UNDERFLOW;
    }

    return uart->fifo;
}

int MXC_UART_RevB_WriteCharacterRaw(mxc_uart_revb_regs_t *uart, uint8_t character)
{
    // Require the TX FIFO to be empty, so that we write out the expected character
    // Return error if the FIFO is full
    if (uart->status & MXC_F_UART_REVB_STATUS_TX_FULL) {
        return E_OVERFLOW;
    }

    uart->fifo = character;

    return E_NO_ERROR;
}

int MXC_UART_RevB_ReadCharacter(mxc_uart_revb_regs_t *uart)
{
    if (uart->status & MXC_F_UART_REVB_STATUS_RX_EM) {
        return E_UNDERFLOW;
    }

    return uart->fifo;
}

int MXC_UART_RevB_WriteCharacter(mxc_uart_revb_regs_t *uart, uint8_t character)
{
    // Require the TX FIFO to be empty, so that we write out the expected character
    // Return error if the FIFO is full
    if (uart->status & MXC_F_UART_REVB_STATUS_TX_FULL) {
        return E_OVERFLOW;
    }

    uart->fifo = character;

    return E_NO_ERROR;
}

int MXC_UART_RevB_Read(mxc_uart_revb_regs_t *uart, uint8_t *buffer, int *len)
{
    int read = 0;
    int retVal;

    if (buffer == NULL) {
        return E_NULL_PTR;
    }

    if (len == NULL) {
        return E_NULL_PTR;
    }

    for (; read < *len; read++) {
        retVal = MXC_UART_ReadCharacter((mxc_uart_regs_t *)uart);

        if (retVal < 0) {
            *len = read;
            return retVal;
        } else {
            buffer[read] = retVal;
        }
    }

    *len = read;
    return E_NO_ERROR;
}

int MXC_UART_RevB_Write(mxc_uart_revb_regs_t *uart, const uint8_t *byte, int *len)
{
    int written = 0;
    int retVal;

    if (byte == NULL) {
        return E_NULL_PTR;
    }

    if (len == NULL) {
        return E_NULL_PTR;
    }

    for (; written < *len; written++) {
        retVal = MXC_UART_WriteCharacterRaw((mxc_uart_regs_t *)uart, byte[written]);

        if (retVal != E_NO_ERROR) {
            *len = written;
            return retVal;
        }
    }

    *len = written;
    return E_NO_ERROR;
}

unsigned int MXC_UART_RevB_ReadRXFIFO(mxc_uart_revb_regs_t *uart, unsigned char *bytes,
                                      unsigned int len)
{
    unsigned read = 0;

    for (; read < len; read++) {
        if (uart->status & MXC_F_UART_REVB_STATUS_RX_EM) {
            break;
        }

        bytes[read] = uart->fifo;
    }

    return read;
}

unsigned int MXC_UART_RevB_GetRXFIFOAvailable(mxc_uart_revb_regs_t *uart)
{
    return (uart->status & MXC_F_UART_REVB_STATUS_RX_LVL) >> MXC_F_UART_REVB_STATUS_RX_LVL_POS;
}

unsigned int MXC_UART_RevB_WriteTXFIFO(mxc_uart_revb_regs_t *uart, const unsigned char *bytes,
                                       unsigned int len)
{
    unsigned written = 0;

    for (; written < len; written++) {
        if (uart->status & MXC_F_UART_REVB_STATUS_TX_FULL) {
            break;
        }

        uart->fifo = bytes[written];
    }

    return written;
}

unsigned int MXC_UART_RevB_GetTXFIFOAvailable(mxc_uart_revb_regs_t *uart)
{
    int txCnt = (uart->status & MXC_F_UART_REVB_STATUS_TX_LVL) >> MXC_F_UART_REVB_STATUS_TX_LVL_POS;
    return MXC_UART_FIFO_DEPTH - txCnt;
}

int MXC_UART_RevB_ClearRXFIFO(mxc_uart_revb_regs_t *uart)
{
    uart->ctrl |= MXC_F_UART_REVB_CTRL_RX_FLUSH;
    while (!(uart->status & MXC_F_UART_REVB_STATUS_RX_EM)) {}

    return E_NO_ERROR;
}

int MXC_UART_RevB_ClearTXFIFO(mxc_uart_revb_regs_t *uart)
{
    uart->ctrl |= MXC_F_UART_REVB_CTRL_TX_FLUSH;
    while (uart->ctrl & MXC_F_UART_REVB_CTRL_TX_FLUSH) {}

    return E_NO_ERROR;
}

int MXC_UART_RevB_SetRXThreshold(mxc_uart_revb_regs_t *uart, unsigned int numBytes)
{
    if (numBytes < 1 || numBytes > MXC_UART_FIFO_DEPTH) {
        return E_BAD_PARAM;
    }

    numBytes <<= MXC_F_UART_REVB_CTRL_RX_THD_VAL_POS;
    MXC_SETFIELD(uart->ctrl, MXC_F_UART_REVB_CTRL_RX_THD_VAL, numBytes);

    return E_NO_ERROR;
}

unsigned int MXC_UART_RevB_GetRXThreshold(mxc_uart_revb_regs_t *uart)
{
    return ((uart->ctrl & MXC_F_UART_REVB_CTRL_RX_THD_VAL) >> MXC_F_UART_REVB_CTRL_RX_THD_VAL_POS);
}

unsigned int MXC_UART_RevB_GetFlags(mxc_uart_revb_regs_t *uart)
{
    return uart->int_fl;
}

int MXC_UART_RevB_ClearFlags(mxc_uart_revb_regs_t *uart, unsigned int flags)
{
    uart->int_fl = flags;

    return E_NO_ERROR;
}

int MXC_UART_RevB_EnableInt(mxc_uart_revb_regs_t *uart, unsigned int intEn)
{
    uart->int_en |= intEn;

    return E_NO_ERROR;
}

int MXC_UART_RevB_DisableInt(mxc_uart_revb_regs_t *uart, unsigned int intDis)
{
    uart->int_en &= ~intDis;

    return E_NO_ERROR;
}

unsigned int MXC_UART_RevB_GetStatus(mxc_uart_revb_regs_t *uart)
{
    return uart->status;
}

int MXC_UART_RevB_Transaction(mxc_uart_revb_req_t *req)
{
    uint32_t numToWrite, numToRead;

    if (MXC_UART_GET_IDX((mxc_uart_regs_t *)(req->uart)) < 0) {
        return E_BAD_PARAM;
    }

    MXC_UART_DisableInt((mxc_uart_regs_t *)(req->uart), 0xFFFFFFFF);
    MXC_UART_ClearFlags((mxc_uart_regs_t *)(req->uart), 0xFFFFFFFF);

    req->txCnt = 0;
    req->rxCnt = 0;

    if (req->rxLen) {
        if (req->rxData == NULL) {
            return E_BAD_PARAM;
        }
    }

    if (req->txLen) {
        if (req->txData == NULL) {
            return E_BAD_PARAM;
        }

        numToWrite = MXC_UART_GetTXFIFOAvailable((mxc_uart_regs_t *)(req->uart));
        numToWrite = numToWrite > (req->txLen - req->txCnt) ? req->txLen - req->txCnt : numToWrite;
        req->txCnt += MXC_UART_WriteTXFIFO((mxc_uart_regs_t *)(req->uart), &req->txData[req->txCnt],
                                           numToWrite);

        while (req->txCnt < req->txLen) {
            while (!(MXC_UART_GetFlags((mxc_uart_regs_t *)(req->uart)) &
                     MXC_F_UART_REVB_INT_FL_TX_HE) &&
                   !(req->uart->status & MXC_F_UART_REVB_STATUS_TX_EM)) {}

            numToWrite = MXC_UART_GetTXFIFOAvailable((mxc_uart_regs_t *)(req->uart));
            numToWrite = numToWrite > (req->txLen - req->txCnt) ? req->txLen - req->txCnt :
                                                                  numToWrite;
            req->txCnt += MXC_UART_WriteTXFIFO((mxc_uart_regs_t *)(req->uart),
                                               &req->txData[req->txCnt], numToWrite);
            MXC_UART_ClearFlags((mxc_uart_regs_t *)(req->uart), MXC_F_UART_REVB_INT_FL_TX_HE);
        }
    }

    if (req->rxLen) {
        numToRead = MXC_UART_GetRXFIFOAvailable((mxc_uart_regs_t *)(req->uart));
        numToRead = numToRead > (req->rxLen - req->rxCnt) ? req->rxLen - req->rxCnt : numToRead;
        req->rxCnt += MXC_UART_ReadRXFIFO((mxc_uart_regs_t *)(req->uart), &req->rxData[req->rxCnt],
                                          numToRead);

        while (req->rxCnt < req->rxLen) {
            while (!(MXC_UART_GetFlags((mxc_uart_regs_t *)(req->uart)) &
                     MXC_F_UART_REVB_INT_FL_RX_THD)) {}

            numToRead = MXC_UART_GetRXFIFOAvailable((mxc_uart_regs_t *)(req->uart));
            numToRead = numToRead > (req->rxLen - req->rxCnt) ? req->rxLen - req->rxCnt : numToRead;
            req->rxCnt += MXC_UART_ReadRXFIFO((mxc_uart_regs_t *)(req->uart),
                                              &req->rxData[req->rxCnt], numToRead);
            MXC_UART_ClearFlags((mxc_uart_regs_t *)(req->uart), MXC_F_UART_REVB_INT_FL_RX_THD);
        }
    }

    return E_NO_ERROR;
}

int MXC_UART_RevB_TransactionAsync(mxc_uart_revb_req_t *req)
{
    uint32_t numToWrite, numToRead;
    int uart_num = MXC_UART_GET_IDX((mxc_uart_regs_t *)(req->uart));

    if (!AsyncTxRequests[uart_num] && !AsyncRxRequests[uart_num]) {
        /* No requests pending, clear the interrupt state */
        MXC_UART_DisableInt((mxc_uart_regs_t *)(req->uart), 0xFFFFFFFF);
        MXC_UART_ClearFlags((mxc_uart_regs_t *)(req->uart), 0xFFFFFFFF);

    } else if (AsyncRxRequests[uart_num] && req->rxLen) {
        /* RX request pending */
        return E_BUSY;
    } else if (AsyncTxRequests[uart_num] && req->txLen) {
        /* TX request pending */
        return E_BUSY;
    }

    req->txCnt = 0;
    req->rxCnt = 0;

    if (req->txLen) {
        if (req->txData == NULL) {
            return E_BAD_PARAM;
        }

        // Save TX Request
        AsyncTxRequests[MXC_UART_GET_IDX((mxc_uart_regs_t *)(req->uart))] = (void *)req;

        MXC_UART_EnableInt((mxc_uart_regs_t *)(req->uart), MXC_F_UART_REVB_INT_EN_TX_HE);
        numToWrite = MXC_UART_GetTXFIFOAvailable((mxc_uart_regs_t *)(req->uart));
        numToWrite = numToWrite > (req->txLen - req->txCnt) ? req->txLen - req->txCnt : numToWrite;
        req->txCnt += MXC_UART_WriteTXFIFO((mxc_uart_regs_t *)(req->uart), &req->txData[req->txCnt],
                                           numToWrite);

        /* If we're finished writing to the TX FIFO and it's less than half+1 full, pend the interrupt */
        if ((MXC_UART_GetTXFIFOAvailable((mxc_uart_regs_t *)(req->uart)) >=
             (MXC_UART_FIFO_DEPTH / 2)) &&
            (req->txCnt == req->txLen)) {
            NVIC_SetPendingIRQ(MXC_UART_GET_IRQ(uart_num));
        }
    }

    if (req->rxLen) {
        if (req->rxData == NULL) {
            MXC_UART_DisableInt((mxc_uart_regs_t *)(req->uart), 0xFFFFFFFF);
            MXC_UART_ClearTXFIFO((mxc_uart_regs_t *)(req->uart));
            return E_BAD_PARAM;
        }

        // Save RX Request
        AsyncRxRequests[MXC_UART_GET_IDX((mxc_uart_regs_t *)(req->uart))] = (void *)req;

        // All error interrupts are related to RX
        MXC_UART_EnableInt((mxc_uart_regs_t *)(req->uart), MXC_UART_REVB_ERRINT_EN);

        MXC_UART_EnableInt((mxc_uart_regs_t *)(req->uart), MXC_F_UART_REVB_INT_EN_RX_THD);
        numToRead = MXC_UART_GetRXFIFOAvailable((mxc_uart_regs_t *)(req->uart));
        numToRead = numToRead > (req->rxLen - req->rxCnt) ? req->rxLen - req->rxCnt : numToRead;
        req->rxCnt += MXC_UART_ReadRXFIFO((mxc_uart_regs_t *)(req->uart), &req->rxData[req->rxCnt],
                                          numToRead);
        MXC_UART_ClearFlags((mxc_uart_regs_t *)(req->uart), MXC_F_UART_REVB_INT_FL_RX_THD);
    }

    return E_NO_ERROR;
}

int MXC_UART_RevB_AsyncTxCallback(mxc_uart_revb_regs_t *uart, int retVal)
{
    int uart_num = MXC_UART_GET_IDX((mxc_uart_regs_t *)uart);

    mxc_uart_req_t *req = (mxc_uart_req_t *)AsyncTxRequests[uart_num];
    if ((req != NULL) && (req->callback != NULL)) {
        AsyncTxRequests[uart_num] = NULL;
        req->callback(req, retVal);
    }

    return E_NO_ERROR;
}

int MXC_UART_RevB_AsyncRxCallback(mxc_uart_revb_regs_t *uart, int retVal)
{
    int uart_num = MXC_UART_GET_IDX((mxc_uart_regs_t *)uart);

    mxc_uart_req_t *req = (mxc_uart_req_t *)AsyncRxRequests[uart_num];
    if ((req != NULL) && (req->callback != NULL)) {
        AsyncRxRequests[uart_num] = NULL;
        req->callback(req, retVal);
    }

    return E_NO_ERROR;
}

int MXC_UART_RevB_AsyncCallback(mxc_uart_revb_regs_t *uart, int retVal)
{
    int uart_num = MXC_UART_GET_IDX((mxc_uart_regs_t *)uart);

    MXC_UART_RevB_AsyncTxCallback(uart, retVal);

    /* Only call the callback once if it's for the same request */
    if (AsyncRxRequests[uart_num] != AsyncTxRequests[uart_num]) {
        MXC_UART_RevB_AsyncRxCallback(uart, retVal);
    }

    return E_NO_ERROR;
}

int MXC_UART_RevB_AsyncStopTx(mxc_uart_revb_regs_t *uart)
{
    MXC_UART_DisableInt((mxc_uart_regs_t *)uart, MXC_F_UART_REVB_INT_EN_TX_HE);

    return E_NO_ERROR;
}

int MXC_UART_RevB_AsyncStopRx(mxc_uart_revb_regs_t *uart)
{
    MXC_UART_DisableInt((mxc_uart_regs_t *)uart, MXC_UART_REVB_ERRINT_EN);

    return E_NO_ERROR;
}

int MXC_UART_RevB_AsyncStop(mxc_uart_revb_regs_t *uart)
{
    MXC_UART_DisableInt((mxc_uart_regs_t *)uart, 0xFFFFFFFF);

    return E_NO_ERROR;
}

int MXC_UART_RevB_AbortAsync(mxc_uart_revb_regs_t *uart)
{
    MXC_UART_AsyncStop((mxc_uart_regs_t *)uart);
    MXC_UART_AsyncCallback((mxc_uart_regs_t *)uart, E_ABORT);

    return E_NO_ERROR;
}

int MXC_UART_RevB_AsyncHandler(mxc_uart_revb_regs_t *uart)
{
    uint32_t numToWrite, numToRead, flags;
    mxc_uart_req_t *req;

    int uart_num = MXC_UART_GET_IDX((mxc_uart_regs_t *)uart);

    flags = MXC_UART_GetFlags((mxc_uart_regs_t *)uart);

    /* Unexpected interrupt */
    if (!AsyncTxRequests[uart_num] && !AsyncRxRequests[uart_num]) {
        MXC_UART_ClearFlags((mxc_uart_regs_t *)uart, uart->int_fl);
        return E_INVALID;
    }

    if (flags & MXC_UART_REVB_ERRINT_FL & uart->int_en) {
        MXC_UART_AsyncStop((mxc_uart_regs_t *)uart);
        MXC_UART_AsyncCallback((mxc_uart_regs_t *)uart, E_COMM_ERR);
        return E_INVALID;
    }

    req = (mxc_uart_req_t *)AsyncTxRequests[uart_num];
    if ((req != NULL) && (req->txLen)) {
        numToWrite = MXC_UART_GetTXFIFOAvailable((mxc_uart_regs_t *)(req->uart));
        numToWrite = numToWrite > (req->txLen - req->txCnt) ? req->txLen - req->txCnt : numToWrite;
        req->txCnt += MXC_UART_WriteTXFIFO((mxc_uart_regs_t *)(req->uart), &req->txData[req->txCnt],
                                           numToWrite);
        MXC_UART_ClearFlags(req->uart, MXC_F_UART_REVB_INT_FL_TX_HE);
    }

    req = (mxc_uart_req_t *)AsyncRxRequests[uart_num];
    if ((req != NULL) && (flags & MXC_F_UART_REVB_INT_FL_RX_THD) && (req->rxLen)) {
        numToRead = MXC_UART_GetRXFIFOAvailable((mxc_uart_regs_t *)(req->uart));
        numToRead = numToRead > (req->rxLen - req->rxCnt) ? req->rxLen - req->rxCnt : numToRead;
        req->rxCnt += MXC_UART_ReadRXFIFO((mxc_uart_regs_t *)(req->uart), &req->rxData[req->rxCnt],
                                          numToRead);

        if ((req->rxLen - req->rxCnt) < MXC_UART_GetRXThreshold((mxc_uart_regs_t *)(req->uart))) {
            MXC_UART_SetRXThreshold((mxc_uart_regs_t *)(req->uart), req->rxLen - req->rxCnt);
        }

        MXC_UART_ClearFlags((mxc_uart_regs_t *)(req->uart), MXC_F_UART_REVB_INT_FL_RX_THD);
    }

    if (AsyncRxRequests[uart_num] == AsyncTxRequests[uart_num]) {
        if ((req != NULL) && (req->rxCnt == req->rxLen) && (req->txCnt == req->txLen)) {
            MXC_UART_AsyncStop((mxc_uart_regs_t *)uart);
            MXC_UART_AsyncCallback((mxc_uart_regs_t *)uart, E_NO_ERROR);
        }
        return E_NO_ERROR;
    }

    req = (mxc_uart_req_t *)AsyncRxRequests[uart_num];
    if ((req != NULL) && (req->rxCnt == req->rxLen)) {
        MXC_UART_RevB_AsyncStopRx(uart);
        MXC_UART_RevB_AsyncRxCallback(uart, E_NO_ERROR);
        return E_NO_ERROR;
    }

    req = (mxc_uart_req_t *)AsyncTxRequests[uart_num];
    if ((req != NULL) && (req->txCnt == req->txLen)) {
        MXC_UART_RevB_AsyncStopTx(uart);
        MXC_UART_RevB_AsyncTxCallback(uart, E_NO_ERROR);
        return E_NO_ERROR;
    }

    return E_NO_ERROR;
}

int MXC_UART_RevB_SetAutoDMAHandlers(mxc_uart_revb_regs_t *uart, bool enable)
{
    int n = MXC_UART_GET_IDX((mxc_uart_regs_t *)uart);
    MXC_ASSERT(n >= 0);

    states[n].auto_dma_handlers = enable;

    return E_NO_ERROR;
}

void MXC_UART_RevB_DMA_SetupAutoHandlers(unsigned int channel)
{
#ifdef __arm__
    NVIC_EnableIRQ(MXC_DMA_CH_GET_IRQ(channel));
    MXC_NVIC_SetVector(MXC_DMA_CH_GET_IRQ(channel), MXC_DMA_Handler);
#else
    // TODO(JC): RISC-V

#endif // __arm__
}

int MXC_UART_RevB_SetTXDMAChannel(mxc_uart_revb_regs_t *uart, unsigned int channel)
{
    int n = MXC_UART_GET_IDX((mxc_uart_regs_t *)uart);

    states[n].channelTx = channel;

    return E_NO_ERROR;
}

int MXC_UART_RevB_GetTXDMAChannel(mxc_uart_revb_regs_t *uart)
{
    int n = MXC_UART_GET_IDX((mxc_uart_regs_t *)uart);

    return states[n].channelTx;
}

int MXC_UART_RevB_SetRXDMAChannel(mxc_uart_revb_regs_t *uart, unsigned int channel)
{
    int n = MXC_UART_GET_IDX((mxc_uart_regs_t *)uart);

    states[n].channelRx = channel;

    return E_NO_ERROR;
}

int MXC_UART_RevB_GetRXDMAChannel(mxc_uart_revb_regs_t *uart)
{
    int n = MXC_UART_GET_IDX((mxc_uart_regs_t *)uart);

    return states[n].channelRx;
}

int MXC_UART_RevB_ReadRXFIFODMA(mxc_uart_revb_regs_t *uart, unsigned char *bytes, unsigned int len,
                                mxc_uart_dma_complete_cb_t callback, mxc_dma_config_t config)
{
    uint8_t channel;
    mxc_dma_srcdst_t srcdst;

    int uart_num = MXC_UART_GET_IDX((mxc_uart_regs_t *)uart);

    if (bytes == NULL) {
        return E_NULL_PTR;
    }

    if (states[uart_num].auto_dma_handlers && states[uart_num].channelRx < 0) {
        /* Acquire channel if we don't have one already */
        channel = MXC_DMA_AcquireChannel();
        MXC_UART_RevB_SetRXDMAChannel(uart, channel);
        MXC_UART_RevB_DMA_SetupAutoHandlers(channel);
    } else {
        if (states[uart_num].channelRx < 0)
            return E_BAD_STATE;
        channel = MXC_UART_RevB_GetRXDMAChannel(uart);
    }

    config.ch = channel;

    config.srcwd = MXC_DMA_WIDTH_BYTE;
    config.dstwd = MXC_DMA_WIDTH_BYTE;

    config.srcinc_en = 0;
    config.dstinc_en = 1;

    srcdst.ch = channel;
    srcdst.dest = bytes;
    srcdst.len = len;

    states[uart_num].channelRx = channel;
    MXC_DMA_ConfigChannel(config, srcdst);
    MXC_DMA_SetCallback(channel, MXC_UART_DMACallback);
    MXC_DMA_EnableInt(channel);
    MXC_DMA_Start(channel);
    //MXC_DMA->ch[channel].ctrl |= MXC_F_DMA_CTRL_CTZ_IE;
    MXC_DMA_SetChannelInterruptEn(channel, 0, 1);
    uart->dma |= MXC_F_UART_REVB_DMA_RX_EN;

    return E_NO_ERROR;
}

int MXC_UART_RevB_WriteTXFIFODMA(mxc_uart_revb_regs_t *uart, const unsigned char *bytes,
                                 unsigned int len, mxc_uart_dma_complete_cb_t callback,
                                 mxc_dma_config_t config)
{
    uint8_t channel;
    mxc_dma_srcdst_t srcdst;

    int uart_num = MXC_UART_GET_IDX((mxc_uart_regs_t *)uart);

    if (bytes == NULL) {
        return E_NULL_PTR;
    }

    if (states[uart_num].auto_dma_handlers && states[uart_num].channelTx < 0) {
        /* Acquire channel if we don't have one already */
        channel = MXC_DMA_AcquireChannel();
        MXC_UART_RevB_SetTXDMAChannel(uart, channel);
        MXC_UART_RevB_DMA_SetupAutoHandlers(channel);
    } else {
        if (states[uart_num].channelTx < 0)
            return E_BAD_STATE;
        channel = MXC_UART_RevB_GetTXDMAChannel(uart);
    }

    config.ch = channel;

    config.srcwd = MXC_DMA_WIDTH_BYTE;
    config.dstwd = MXC_DMA_WIDTH_BYTE;

    config.srcinc_en = 1;
    config.dstinc_en = 0;

    srcdst.ch = channel;
    srcdst.source = (void *)bytes;
    srcdst.len = len;

    states[uart_num].channelTx = channel;
    MXC_DMA_ConfigChannel(config, srcdst);
    MXC_DMA_SetCallback(channel, MXC_UART_DMACallback);
    MXC_DMA_EnableInt(channel);
    MXC_DMA_Start(channel);
    //MXC_DMA->ch[channel].ctrl |= MXC_F_DMA_CTRL_CTZ_IE;
    MXC_DMA_SetChannelInterruptEn(channel, 0, 1);
    uart->dma |= MXC_F_UART_REVB_DMA_TX_EN;

    return E_NO_ERROR;
}

int MXC_UART_RevB_TransactionDMA(mxc_uart_revb_req_t *req)
{
    int uart_num = MXC_UART_GET_IDX((mxc_uart_regs_t *)(req->uart));

    if (req->txLen) {
        if (req->txData == NULL) {
            return E_BAD_PARAM;
        }
    }

    if (req->rxLen) {
        if (req->rxData == NULL) {
            return E_BAD_PARAM;
        }
    }

    MXC_UART_DisableInt((mxc_uart_regs_t *)(req->uart), 0xFFFFFFFF);
    MXC_UART_ClearFlags((mxc_uart_regs_t *)(req->uart), 0xFFFFFFFF);

    // Clearing the RX FIFOs here makes RX-only or TX-only
    // transactions half-duplex.  Commenting out for now.
    // MXC_UART_ClearTXFIFO((mxc_uart_regs_t *)(req->uart));
    // MXC_UART_ClearRXFIFO((mxc_uart_regs_t *)(req->uart));

    //Set DMA FIFO threshold
    (req->uart)->dma |= (1 << MXC_F_UART_REVB_DMA_RX_THD_VAL_POS);
    (req->uart)->dma |= (2 << MXC_F_UART_REVB_DMA_TX_THD_VAL_POS);

    MXC_DMA_Init();

    // Reset rx/tx counters,
    req->rxCnt = 0;
    req->txCnt = 0;

    //tx
    if ((req->txData != NULL) && (req->txLen)) {
        /* Save TX req, the DMA handler will use this later. */
        states[uart_num].tx_req = req;
        if (MXC_UART_WriteTXFIFODMA((mxc_uart_regs_t *)(req->uart), req->txData, req->txLen,
                                    NULL) != E_NO_ERROR) {
            return E_BAD_PARAM;
        }
    }

    //rx
    if ((req->rxData != NULL) && (req->rxLen)) {
        states[uart_num].rx_req = req;
        if (MXC_UART_ReadRXFIFODMA((mxc_uart_regs_t *)(req->uart), req->rxData, req->rxLen, NULL) !=
            E_NO_ERROR) {
            return E_BAD_PARAM;
        }
    }

    return E_NO_ERROR;
}

void MXC_UART_RevB_DMACallback(int ch, int error)
{
    mxc_uart_revb_req_t *temp_req;

    for (int i = 0; i < MXC_UART_INSTANCES; i++) {
        if (states[i].channelTx == ch) {
            /* Populate txLen.  The number of "remainder" bytes is what's left on the 
            DMA channel's count register. */
            states[i].tx_req->txCnt = states[i].tx_req->txLen - MXC_DMA->ch[ch].cnt;

            temp_req = states[i].tx_req;

            if (states[i].auto_dma_handlers) {
                /* Release channel _before_ running callback in case 
                user wants to start another transaction inside it */
                MXC_DMA_ReleaseChannel(ch);
                states[i].channelTx = -1;
            }

            if (temp_req->callback != NULL &&
                ((states[i].tx_req->rxCnt == states[i].tx_req->rxLen) ||
                 states[i].tx_req->rxData == NULL)) {
                /* Only call TX callback if RX component is complete/disabled. Note that
                we are checking the request associated with the _channel_ assignment, not
                the other side of the state struct. */
                temp_req->callback((mxc_uart_req_t *)temp_req, E_NO_ERROR);
            }
            break;
        } else if (states[i].channelRx == ch) {
            /* Same as above, but for RX */
            states[i].rx_req->rxCnt = states[i].rx_req->rxLen - MXC_DMA->ch[ch].cnt;
            temp_req = states[i].rx_req;
            if (states[i].auto_dma_handlers) {
                MXC_DMA_ReleaseChannel(ch);
                states[i].channelRx = -1;
            }

            if (temp_req->callback != NULL &&
                ((states[i].rx_req->txCnt == states[i].rx_req->txLen) ||
                 states[i].rx_req->txData == NULL)) {
                temp_req->callback((mxc_uart_req_t *)temp_req, E_NO_ERROR);
            }
            break;
        }
    }
}
