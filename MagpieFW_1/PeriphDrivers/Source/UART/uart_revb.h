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

#ifndef LIBRARIES_PERIPHDRIVERS_SOURCE_UART_UART_REVB_H_
#define LIBRARIES_PERIPHDRIVERS_SOURCE_UART_UART_REVB_H_

#include "mxc_device.h"
#include "mxc_assert.h"
#include "dma.h"
#include "uart_revb_regs.h"
#include "uart_regs.h"

typedef struct _mxc_uart_revb_req_t mxc_uart_revb_req_t;

/**
 * @brief      Clock settings */
typedef enum {
    MXC_UART_REVB_APB_CLK = 0, // PCLK
    MXC_UART_REVB_EXT_CLK = 1,
    MXC_UART_REVB_CLK2 = 2,
    MXC_UART_REVB_CLK3 = 3,
    // For UART3, ERTCO and INRO clocks
    MXC_UART_REVB_LPCLK2 = 4,
    MXC_UART_REVB_LPCLK3 = 5
} mxc_uart_revb_clock_t;

struct _mxc_uart_revb_req_t {
    mxc_uart_revb_regs_t *uart;
    uint8_t *txData;
    uint8_t *rxData;
    uint32_t txLen;
    uint32_t rxLen;
    uint32_t txCnt;
    uint32_t rxCnt;
    mxc_uart_complete_cb_t callback;
};

int MXC_UART_RevB_Init(mxc_uart_revb_regs_t *uart, unsigned int baud, mxc_uart_revb_clock_t clock);
int MXC_UART_RevB_Shutdown(mxc_uart_revb_regs_t *uart);
int MXC_UART_RevB_ReadyForSleep(mxc_uart_revb_regs_t *uart);
int MXC_UART_RevB_SetFrequency(mxc_uart_revb_regs_t *uart, unsigned int baud,
                               mxc_uart_revb_clock_t clock);
int MXC_UART_RevB_GetFrequency(mxc_uart_revb_regs_t *uart);
int MXC_UART_RevB_SetDataSize(mxc_uart_revb_regs_t *uart, int dataSize);
int MXC_UART_RevB_SetStopBits(mxc_uart_revb_regs_t *uart, mxc_uart_stop_t stopBits);
int MXC_UART_RevB_SetParity(mxc_uart_revb_regs_t *uart, mxc_uart_parity_t parity);
int MXC_UART_RevB_SetFlowCtrl(mxc_uart_revb_regs_t *uart, mxc_uart_flow_t flowCtrl,
                              int rtsThreshold);
int MXC_UART_RevB_SetClockSource(mxc_uart_revb_regs_t *uart, mxc_uart_revb_clock_t clock);
int MXC_UART_RevB_GetActive(mxc_uart_revb_regs_t *uart);
int MXC_UART_RevB_AbortTransmission(mxc_uart_revb_regs_t *uart);
int MXC_UART_RevB_ReadCharacterRaw(mxc_uart_revb_regs_t *uart);
int MXC_UART_RevB_WriteCharacterRaw(mxc_uart_revb_regs_t *uart, uint8_t character);
int MXC_UART_RevB_ReadCharacter(mxc_uart_revb_regs_t *uart);
int MXC_UART_RevB_WriteCharacter(mxc_uart_revb_regs_t *uart, uint8_t character);
int MXC_UART_RevB_Read(mxc_uart_revb_regs_t *uart, uint8_t *buffer, int *len);
int MXC_UART_RevB_Write(mxc_uart_revb_regs_t *uart, const uint8_t *byte, int *len);
unsigned int MXC_UART_RevB_ReadRXFIFO(mxc_uart_revb_regs_t *uart, unsigned char *bytes,
                                      unsigned int len);
int MXC_UART_RevB_ReadRXFIFODMA(mxc_uart_revb_regs_t *uart, unsigned char *bytes, unsigned int len,
                                mxc_uart_dma_complete_cb_t callback, mxc_dma_config_t config);
unsigned int MXC_UART_RevB_GetRXFIFOAvailable(mxc_uart_revb_regs_t *uart);
unsigned int MXC_UART_RevB_WriteTXFIFO(mxc_uart_revb_regs_t *uart, const unsigned char *bytes,
                                       unsigned int len);
int MXC_UART_RevB_WriteTXFIFODMA(mxc_uart_revb_regs_t *uart, const unsigned char *bytes,
                                 unsigned int len, mxc_uart_dma_complete_cb_t callback,
                                 mxc_dma_config_t config);
unsigned int MXC_UART_RevB_GetTXFIFOAvailable(mxc_uart_revb_regs_t *uart);
int MXC_UART_RevB_ClearRXFIFO(mxc_uart_revb_regs_t *uart);
int MXC_UART_RevB_ClearTXFIFO(mxc_uart_revb_regs_t *uart);
int MXC_UART_RevB_SetRXThreshold(mxc_uart_revb_regs_t *uart, unsigned int numBytes);
unsigned int MXC_UART_RevB_GetRXThreshold(mxc_uart_revb_regs_t *uart);
int MXC_UART_RevB_SetTXThreshold(mxc_uart_revb_regs_t *uart, unsigned int numBytes);
unsigned int MXC_UART_RevB_GetTXThreshold(mxc_uart_revb_regs_t *uart);
unsigned int MXC_UART_RevB_GetFlags(mxc_uart_revb_regs_t *uart);
int MXC_UART_RevB_ClearFlags(mxc_uart_revb_regs_t *uart, unsigned int flags);
int MXC_UART_RevB_EnableInt(mxc_uart_revb_regs_t *uart, unsigned int mask);
int MXC_UART_RevB_DisableInt(mxc_uart_revb_regs_t *uart, unsigned int mask);
unsigned int MXC_UART_RevB_GetStatus(mxc_uart_revb_regs_t *uart);
int MXC_UART_RevB_Busy(mxc_uart_revb_regs_t *uart);
int MXC_UART_RevB_Transaction(mxc_uart_revb_req_t *req);
int MXC_UART_RevB_TransactionAsync(mxc_uart_revb_req_t *req);
int MXC_UART_RevB_TransactionDMA(mxc_uart_revb_req_t *req);
int MXC_UART_RevB_AbortAsync(mxc_uart_revb_regs_t *uart);
int MXC_UART_RevB_AsyncHandler(mxc_uart_revb_regs_t *uart);
int MXC_UART_RevB_AsyncStop(mxc_uart_revb_regs_t *uart);
int MXC_UART_RevB_AsyncCallback(mxc_uart_revb_regs_t *uart, int retVal);
void MXC_UART_RevB_DMACallback(int ch, int error);

int MXC_UART_RevB_SetAutoDMAHandlers(mxc_uart_revb_regs_t *uart, bool enable);
int MXC_UART_RevB_SetTXDMAChannel(mxc_uart_revb_regs_t *uart, unsigned int channel);
int MXC_UART_RevB_GetTXDMAChannel(mxc_uart_revb_regs_t *uart);
int MXC_UART_RevB_SetRXDMAChannel(mxc_uart_revb_regs_t *uart, unsigned int channel);
int MXC_UART_RevB_GetRXDMAChannel(mxc_uart_revb_regs_t *uart);

#endif // LIBRARIES_PERIPHDRIVERS_SOURCE_UART_UART_REVB_H_
