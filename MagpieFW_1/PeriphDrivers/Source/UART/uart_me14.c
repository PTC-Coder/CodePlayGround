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

#include "uart.h"
#include "mxc_device.h"
#include "mxc_pins.h"
#include "mxc_assert.h"
#include "uart_reva.h"
#include "uart_common.h"
#include "dma.h"

void MXC_UART_DMACallback(int ch, int error)
{
    MXC_UART_RevA_DMACallback(ch, error);
}

int MXC_UART_AsyncCallback(mxc_uart_regs_t *uart, int retVal)
{
    return MXC_UART_RevA_AsyncCallback((mxc_uart_reva_regs_t *)uart, retVal);
}

int MXC_UART_TxAsyncCallback(mxc_uart_regs_t *uart, int retVal)
{
    return MXC_UART_RevA_TxAsyncCallback((mxc_uart_reva_regs_t *)uart, retVal);
}

int MXC_UART_RxAsyncCallback(mxc_uart_regs_t *uart, int retVal)
{
    return MXC_UART_RevA_RxAsyncCallback((mxc_uart_reva_regs_t *)uart, retVal);
}

int MXC_UART_AsyncStop(mxc_uart_regs_t *uart)
{
    return MXC_UART_RevA_AsyncStop((mxc_uart_reva_regs_t *)uart);
}

int MXC_UART_TxAsyncStop(mxc_uart_regs_t *uart)
{
    return MXC_UART_RevA_TxAsyncStop((mxc_uart_reva_regs_t *)uart);
}

int MXC_UART_RxAsyncStop(mxc_uart_regs_t *uart)
{
    return MXC_UART_RevA_RxAsyncStop((mxc_uart_reva_regs_t *)uart);
}

int MXC_UART_Init(mxc_uart_regs_t *uart, unsigned int baud, sys_map_t map)
{
#ifndef MSDK_NO_GPIO_CLK_INIT
    int retval;

    retval = MXC_UART_Shutdown(uart);

    if (retval) {
        return retval;
    }

    switch (MXC_UART_GET_IDX(uart)) {
    case 0:
        if (map == MAP_A) {
            MXC_GPIO_Config(&gpio_cfg_uart0a);
        } else {
            MXC_GPIO_Config(&gpio_cfg_uart0b);
        }
        MXC_SYS_ClockEnable(MXC_SYS_PERIPH_CLOCK_UART0);
        break;

    case 1:
        if (map == MAP_A) {
            MXC_GPIO_Config(&gpio_cfg_uart1a);
        } else {
            MXC_GPIO_Config(&gpio_cfg_uart1b);
        }
        MXC_SYS_ClockEnable(MXC_SYS_PERIPH_CLOCK_UART1);
        break;

    case 2:
        if (map == MAP_A) {
            MXC_GPIO_Config(&gpio_cfg_uart2a);
        } else {
            MXC_GPIO_Config(&gpio_cfg_uart2b);
        }
        MXC_SYS_ClockEnable(MXC_SYS_PERIPH_CLOCK_UART2);
        break;
    }
#else
    (void)map;
#endif // MSDK_NO_GPIO_CLK_INIT

    return MXC_UART_RevA_Init((mxc_uart_reva_regs_t *)uart, baud);
}

int MXC_UART_Shutdown(mxc_uart_regs_t *uart)
{
    switch (MXC_UART_GET_IDX(uart)) {
    case 0:
        MXC_SYS_Reset_Periph(MXC_SYS_RESET_UART0);
        MXC_SYS_ClockDisable(MXC_SYS_PERIPH_CLOCK_UART0);
        break;

    case 1:
        MXC_SYS_Reset_Periph(MXC_SYS_RESET_UART1);
        MXC_SYS_ClockDisable(MXC_SYS_PERIPH_CLOCK_UART1);
        break;

    case 2:
        MXC_SYS_Reset_Periph(MXC_SYS_RESET_UART2);
        MXC_SYS_ClockDisable(MXC_SYS_PERIPH_CLOCK_UART2);
        break;
    }

    return E_NO_ERROR;
}

int MXC_UART_ReadyForSleep(mxc_uart_regs_t *uart)
{
    return MXC_UART_RevA_ReadyForSleep((mxc_uart_reva_regs_t *)uart);
}

int MXC_UART_SetFrequency(mxc_uart_regs_t *uart, unsigned int baud)
{
    return MXC_UART_RevA_SetFrequency((mxc_uart_reva_regs_t *)uart, baud);
}

int MXC_UART_GetFrequency(mxc_uart_regs_t *uart)
{
    return MXC_UART_RevA_GetFrequency((mxc_uart_reva_regs_t *)uart);
}

int MXC_UART_SetDataSize(mxc_uart_regs_t *uart, int dataSize)
{
    return MXC_UART_RevA_SetDataSize((mxc_uart_reva_regs_t *)uart, dataSize);
}

int MXC_UART_SetStopBits(mxc_uart_regs_t *uart, mxc_uart_stop_t stopBits)
{
    return MXC_UART_RevA_SetStopBits((mxc_uart_reva_regs_t *)uart, stopBits);
}

int MXC_UART_SetParity(mxc_uart_regs_t *uart, mxc_uart_parity_t parity)
{
    return MXC_UART_RevA_SetParity((mxc_uart_reva_regs_t *)uart, parity);
}

int MXC_UART_SetFlowCtrl(mxc_uart_regs_t *uart, mxc_uart_flow_t flowCtrl, int rtsThreshold,
                         sys_map_t map)
{
    switch (MXC_UART_GET_IDX(uart)) {
    case 0:
        if (map == MAP_A) {
            MXC_GPIO_Config(&gpio_cfg_uart0a_flow);
        } else {
            MXC_GPIO_Config(&gpio_cfg_uart0b_flow);
        }
        break;

    case 1:
        if (map == MAP_A) {
            MXC_GPIO_Config(&gpio_cfg_uart1a_flow);
        } else {
            MXC_GPIO_Config(&gpio_cfg_uart1b_flow);
        }
        break;

    case 2:
        if (map == MAP_A) {
            MXC_GPIO_Config(&gpio_cfg_uart2a_flow);
        } else {
            MXC_GPIO_Config(&gpio_cfg_uart2b_flow);
        }
        break;
    }

    return MXC_UART_RevA_SetFlowCtrl((mxc_uart_reva_regs_t *)uart, flowCtrl, rtsThreshold);
}

int MXC_UART_SetClockSource(mxc_uart_regs_t *uart, int usePCLK)
{
    return MXC_UART_RevA_SetClockSource((mxc_uart_reva_regs_t *)uart, usePCLK);
}

int MXC_UART_SetNullModem(mxc_uart_regs_t *uart, int nullModem)
{
    return MXC_UART_RevA_SetNullModem((mxc_uart_reva_regs_t *)uart, nullModem);
}

int MXC_UART_SendBreak(mxc_uart_regs_t *uart)
{
    return MXC_UART_RevA_SendBreak((mxc_uart_reva_regs_t *)uart);
}

int MXC_UART_GetActive(mxc_uart_regs_t *uart)
{
    return MXC_UART_RevA_GetActive((mxc_uart_reva_regs_t *)uart);
}

int MXC_UART_AbortTransmission(mxc_uart_regs_t *uart)
{
    return MXC_UART_RevA_AbortTransmission((mxc_uart_reva_regs_t *)uart);
}

int MXC_UART_ReadCharacterRaw(mxc_uart_regs_t *uart)
{
    return MXC_UART_RevA_ReadCharacterRaw((mxc_uart_reva_regs_t *)uart);
}

int MXC_UART_WriteCharacterRaw(mxc_uart_regs_t *uart, uint8_t character)
{
    return MXC_UART_RevA_WriteCharacterRaw((mxc_uart_reva_regs_t *)uart, character);
}

int MXC_UART_ReadCharacter(mxc_uart_regs_t *uart)
{
    return MXC_UART_Common_ReadCharacter(uart);
}

int MXC_UART_WriteCharacter(mxc_uart_regs_t *uart, uint8_t character)
{
    return MXC_UART_Common_WriteCharacter(uart, character);
}

int MXC_UART_Read(mxc_uart_regs_t *uart, uint8_t *buffer, int *len)
{
    return MXC_UART_RevA_Read((mxc_uart_reva_regs_t *)uart, buffer, len);
}

int MXC_UART_Write(mxc_uart_regs_t *uart, uint8_t *byte, int *len)
{
    return MXC_UART_RevA_Write((mxc_uart_reva_regs_t *)uart, byte, len);
}

unsigned int MXC_UART_ReadRXFIFO(mxc_uart_regs_t *uart, unsigned char *bytes, unsigned int len)
{
    return MXC_UART_RevA_ReadRXFIFO((mxc_uart_reva_regs_t *)uart, bytes, len);
}

int MXC_UART_ReadRXFIFODMA(mxc_uart_regs_t *uart, mxc_dma_regs_t *dma, unsigned char *bytes,
                           unsigned int len, mxc_uart_dma_complete_cb_t callback)
{
    mxc_dma_config_t config;

    if (MXC_DMA_GET_IDX(dma) == -1) {
        return E_BAD_PARAM;
    }

    int uart_num = MXC_UART_GET_IDX(uart);
    switch (uart_num) {
    case 0:
        config.reqsel = MXC_DMA_REQUEST_UART0RX;
        break;

    case 1:
        config.reqsel = MXC_DMA_REQUEST_UART1RX;
        break;

    case 2:
        config.reqsel = MXC_DMA_REQUEST_UART2RX;
        break;

    default:
        return E_BAD_PARAM;
        break;
    }

    return MXC_UART_RevA_ReadRXFIFODMA((mxc_uart_reva_regs_t *)uart, dma, bytes, len, callback,
                                       config);
}

unsigned int MXC_UART_GetRXFIFOAvailable(mxc_uart_regs_t *uart)
{
    return MXC_UART_RevA_GetRXFIFOAvailable((mxc_uart_reva_regs_t *)uart);
}

unsigned int MXC_UART_WriteTXFIFO(mxc_uart_regs_t *uart, unsigned char *bytes, unsigned int len)
{
    return MXC_UART_RevA_WriteTXFIFO((mxc_uart_reva_regs_t *)uart, bytes, len);
}

int MXC_UART_WriteTXFIFODMA(mxc_uart_regs_t *uart, mxc_dma_regs_t *dma, unsigned char *bytes,
                            unsigned int len, mxc_uart_dma_complete_cb_t callback)
{
    mxc_dma_config_t config;

    if (MXC_DMA_GET_IDX(dma) == -1) {
        return E_BAD_PARAM;
    }

    int uart_num = MXC_UART_GET_IDX(uart);
    switch (uart_num) {
    case 0:
        config.reqsel = MXC_DMA_REQUEST_UART0TX;
        break;

    case 1:
        config.reqsel = MXC_DMA_REQUEST_UART1TX;
        break;

    case 2:
        config.reqsel = MXC_DMA_REQUEST_UART2TX;
        break;

    default:
        return E_BAD_PARAM;
        break;
    }

    return MXC_UART_RevA_WriteTXFIFODMA((mxc_uart_reva_regs_t *)uart, dma, bytes, len, callback,
                                        config);
}

unsigned int MXC_UART_GetTXFIFOAvailable(mxc_uart_regs_t *uart)
{
    return MXC_UART_RevA_GetTXFIFOAvailable((mxc_uart_reva_regs_t *)uart);
}

void MXC_UART_ClearRXFIFO(mxc_uart_regs_t *uart)
{
    MXC_UART_RevA_ClearRXFIFO((mxc_uart_reva_regs_t *)uart);
}

void MXC_UART_ClearTXFIFO(mxc_uart_regs_t *uart)
{
    MXC_UART_RevA_ClearTXFIFO((mxc_uart_reva_regs_t *)uart);
}

int MXC_UART_SetRXThreshold(mxc_uart_regs_t *uart, unsigned int numBytes)
{
    return MXC_UART_RevA_SetRXThreshold((mxc_uart_reva_regs_t *)uart, numBytes);
}

unsigned int MXC_UART_GetRXThreshold(mxc_uart_regs_t *uart)
{
    return MXC_UART_RevA_GetRXThreshold((mxc_uart_reva_regs_t *)uart);
}

int MXC_UART_SetTXThreshold(mxc_uart_regs_t *uart, unsigned int numBytes)
{
    return MXC_UART_RevA_SetTXThreshold((mxc_uart_reva_regs_t *)uart, numBytes);
}

unsigned int MXC_UART_GetTXThreshold(mxc_uart_regs_t *uart)
{
    return MXC_UART_RevA_GetTXThreshold((mxc_uart_reva_regs_t *)uart);
}

unsigned int MXC_UART_GetFlags(mxc_uart_regs_t *uart)
{
    return MXC_UART_RevA_GetFlags((mxc_uart_reva_regs_t *)uart);
}

void MXC_UART_ClearFlags(mxc_uart_regs_t *uart, unsigned int flags)
{
    MXC_UART_RevA_ClearFlags((mxc_uart_reva_regs_t *)uart, flags);
}

void MXC_UART_EnableInt(mxc_uart_regs_t *uart, unsigned int intEn)
{
    MXC_UART_RevA_EnableInt((mxc_uart_reva_regs_t *)uart, intEn);
}

void MXC_UART_DisableInt(mxc_uart_regs_t *uart, unsigned int intDis)
{
    MXC_UART_RevA_DisableInt((mxc_uart_reva_regs_t *)uart, intDis);
}

unsigned int MXC_UART_GetStatus(mxc_uart_regs_t *uart)
{
    return MXC_UART_RevA_GetStatus((mxc_uart_reva_regs_t *)uart);
}

int MXC_UART_Transaction(mxc_uart_req_t *req)
{
    return MXC_UART_RevA_Transaction((mxc_uart_reva_req_t *)req);
}

int MXC_UART_TransactionAsync(mxc_uart_req_t *req)
{
    return MXC_UART_RevA_TransactionAsync((mxc_uart_reva_req_t *)req);
}

int MXC_UART_TransactionDMA(mxc_uart_req_t *req, mxc_dma_regs_t *dma)
{
    if (MXC_DMA_GET_IDX(dma) == -1) {
        return E_BAD_PARAM;
    }

    return MXC_UART_RevA_TransactionDMA((mxc_uart_reva_req_t *)req, dma);
}

int MXC_UART_AbortAsync(mxc_uart_regs_t *uart)
{
    return MXC_UART_RevA_AbortAsync((mxc_uart_reva_regs_t *)uart);
}

int MXC_UART_TxAbortAsync(mxc_uart_regs_t *uart)
{
    return MXC_UART_RevA_TxAbortAsync((mxc_uart_reva_regs_t *)uart);
}

int MXC_UART_RxAbortAsync(mxc_uart_regs_t *uart)
{
    return MXC_UART_RevA_RxAbortAsync((mxc_uart_reva_regs_t *)uart);
}

void MXC_UART_AsyncHandler(mxc_uart_regs_t *uart)
{
    MXC_UART_RevA_AsyncHandler((mxc_uart_reva_regs_t *)uart);
}

uint32_t MXC_UART_GetAsyncTXCount(mxc_uart_req_t *req)
{
    return req->txCnt;
}

uint32_t MXC_UART_GetAsyncRXCount(mxc_uart_req_t *req)
{
    return req->rxCnt;
}

int MXC_UART_SetAutoDMAHandlers(mxc_uart_regs_t *uart, bool enable)
{
    return MXC_UART_RevA_SetAutoDMAHandlers((mxc_uart_reva_regs_t *)uart, enable);
}
int MXC_UART_SetTXDMAChannel(mxc_uart_regs_t *uart, unsigned int channel)
{
    return MXC_UART_RevA_SetTXDMAChannel((mxc_uart_reva_regs_t *)uart, channel);
}
int MXC_UART_GetTXDMAChannel(mxc_uart_regs_t *uart)
{
    return MXC_UART_RevA_GetTXDMAChannel((mxc_uart_reva_regs_t *)uart);
}
int MXC_UART_SetRXDMAChannel(mxc_uart_regs_t *uart, unsigned int channel)
{
    return MXC_UART_RevA_SetRXDMAChannel((mxc_uart_reva_regs_t *)uart, channel);
}
int MXC_UART_GetRXDMAChannel(mxc_uart_regs_t *uart)
{
    return MXC_UART_RevA_GetRXDMAChannel((mxc_uart_reva_regs_t *)uart);
}
