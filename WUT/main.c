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

/**
 * @file    main.c
 * @brief   Hello World!
 *
 * @details This example uses the UART to print to a terminal and flashes an LED(P1.14).
 */

/***** Includes *****/
#include <stdio.h>
#include <stdint.h>
#include "mxc_device.h"
#include "mxc_sys.h"
#include "mxc_delay.h"
#include "mxc_errors.h"
#include "pb.h"
#include "board.h"
#include "wut.h"
#include "lp.h"
#include "led.h"
//#include "uart.h"

/***** Definitions *****/
#define MILLISECONDS_WUT 5000

/***** Globals *****/

/***** Functions *****/
void WUT_IRQHandler()
{
    MXC_WUT_IntClear();
}

// *****************************************************************************
int main(void)
{
    mxc_wut_cfg_t cfg;
    uint32_t ticks;

    // printf("/************** Wakeup timer example ********************/\n");
    // printf("This example is to show how the Wakeup timer is used and configured\n");
    // printf("Press push button 0 to put the chip into sleep and then the wakeup timer will wake up "
    //        "in %d Miliseconds \n",
    //        MILLISECONDS_WUT);

    // Get ticks based off of milliseconds
    MXC_WUT_GetTicks(MILLISECONDS_WUT, MXC_WUT_UNIT_MILLISEC, &ticks);

    // config structure for one shot timer to trigger in a number of ticks
    cfg.mode = MXC_WUT_MODE_ONESHOT;
    cfg.cmp_cnt = ticks;

    // Init WUT
    MXC_WUT_Init(MXC_WUT_PRES_1);

    //Config WUT
    MXC_WUT_Config(&cfg);
    MXC_LP_EnableWUTAlarmWakeup();
    

    while (1) {

        LED_On(0);
        MXC_Delay(50000);
        LED_Off(0);
        MXC_Delay(50000);

        if (PB_Get(0) == TRUE) {
            // printf("Entering SLEEP mode.\n");

            LED_On(1);
            MXC_Delay(500000);
            LED_Off(1);
            MXC_Delay(500000);
            
            NVIC_EnableIRQ(WUT_IRQn);
            MXC_WUT_Enable();

            // wait until UART transmit
            // while (MXC_UART_ReadyForSleep(MXC_UART_GET_UART(CONSOLE_UART)) != E_NO_ERROR) {}

            MXC_LP_EnterSleepMode();
            //
            // printf("Waking up from SLEEP mode.\n");

            // for (size_t i = 0; i < 5; i++)
            // {
            //     LED_On(1);
            //     MXC_Delay(50000);
            //     LED_Off(1);
            //     MXC_Delay(50000);
            // }
            
        }
    }
}
