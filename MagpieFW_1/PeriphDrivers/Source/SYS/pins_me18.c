/**
 * @file    mxc_pins.c
 * @brief   This file contains constant pin configurations for the peripherals.
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

#include "gpio.h"
#include "mxc_device.h"

/***** Definitions *****/

/***** Global Variables *****/

// clang-format off
const mxc_gpio_cfg_t gpio_cfg_extclk = { MXC_GPIO0, (MXC_GPIO_PIN_23), MXC_GPIO_FUNC_IN,
                                         MXC_GPIO_PAD_NONE, MXC_GPIO_VSSEL_VDDIO, MXC_GPIO_DRVSTR_0 };

const mxc_gpio_cfg_t gpio_cfg_i2c0 = { MXC_GPIO2, (MXC_GPIO_PIN_7 | MXC_GPIO_PIN_8), MXC_GPIO_FUNC_ALT1,
                                       MXC_GPIO_PAD_NONE, MXC_GPIO_VSSEL_VDDIO, MXC_GPIO_DRVSTR_0 };
const mxc_gpio_cfg_t gpio_cfg_i2c1 = { MXC_GPIO0, (MXC_GPIO_PIN_11 | MXC_GPIO_PIN_12), MXC_GPIO_FUNC_ALT1,
                                       MXC_GPIO_PAD_NONE, MXC_GPIO_VSSEL_VDDIO, MXC_GPIO_DRVSTR_0 };
const mxc_gpio_cfg_t gpio_cfg_i2c2 = { MXC_GPIO1, (MXC_GPIO_PIN_7 | MXC_GPIO_PIN_8), MXC_GPIO_FUNC_ALT3,
                                       MXC_GPIO_PAD_NONE, MXC_GPIO_VSSEL_VDDIO, MXC_GPIO_DRVSTR_0 };

const mxc_gpio_cfg_t gpio_cfg_uart0 = { MXC_GPIO2, (MXC_GPIO_PIN_11 | MXC_GPIO_PIN_12), MXC_GPIO_FUNC_ALT1,
                                        MXC_GPIO_PAD_PULL_UP, MXC_GPIO_VSSEL_VDDIO, MXC_GPIO_DRVSTR_0 };
const mxc_gpio_cfg_t gpio_cfg_uart0_flow = { MXC_GPIO2, (MXC_GPIO_PIN_9 | MXC_GPIO_PIN_10), MXC_GPIO_FUNC_ALT1,
                                             MXC_GPIO_PAD_PULL_UP, MXC_GPIO_VSSEL_VDDIO, MXC_GPIO_DRVSTR_0 };
const mxc_gpio_cfg_t gpio_cfg_uart0_flow_disable = { MXC_GPIO2, (MXC_GPIO_PIN_9 | MXC_GPIO_PIN_10), MXC_GPIO_FUNC_IN,
                                                     MXC_GPIO_PAD_PULL_UP, MXC_GPIO_VSSEL_VDDIO, MXC_GPIO_DRVSTR_0 };
const mxc_gpio_cfg_t gpio_cfg_uart1 = { MXC_GPIO2, (MXC_GPIO_PIN_14 | MXC_GPIO_PIN_16), MXC_GPIO_FUNC_ALT1,
                                        MXC_GPIO_PAD_PULL_UP, MXC_GPIO_VSSEL_VDDIO, MXC_GPIO_DRVSTR_0 };
const mxc_gpio_cfg_t gpio_cfg_uart1_flow = { MXC_GPIO2, (MXC_GPIO_PIN_13 | MXC_GPIO_PIN_15),
                                             MXC_GPIO_FUNC_ALT1, MXC_GPIO_PAD_PULL_UP,
                                             MXC_GPIO_VSSEL_VDDIO, MXC_GPIO_DRVSTR_0 };
const mxc_gpio_cfg_t gpio_cfg_uart1_flow_disable = { MXC_GPIO2, (MXC_GPIO_PIN_13 | MXC_GPIO_PIN_15),
                                                     MXC_GPIO_FUNC_IN, MXC_GPIO_PAD_PULL_UP,
                                                     MXC_GPIO_VSSEL_VDDIO, MXC_GPIO_DRVSTR_0 };
const mxc_gpio_cfg_t gpio_cfg_uart2 = { MXC_GPIO1, (MXC_GPIO_PIN_9 | MXC_GPIO_PIN_10),
                                        MXC_GPIO_FUNC_ALT1, MXC_GPIO_PAD_PULL_UP,
                                        MXC_GPIO_VSSEL_VDDIO, MXC_GPIO_DRVSTR_0 };
const mxc_gpio_cfg_t gpio_cfg_uart2_flow = { MXC_GPIO1, (MXC_GPIO_PIN_7 | MXC_GPIO_PIN_8),
                                             MXC_GPIO_FUNC_ALT1, MXC_GPIO_PAD_PULL_UP,
                                             MXC_GPIO_VSSEL_VDDIO, MXC_GPIO_DRVSTR_0 };
const mxc_gpio_cfg_t gpio_cfg_uart2_flow_disable = { MXC_GPIO1, (MXC_GPIO_PIN_7 | MXC_GPIO_PIN_8),
                                                     MXC_GPIO_FUNC_IN, MXC_GPIO_PAD_PULL_UP,
                                                     MXC_GPIO_VSSEL_VDDIO, MXC_GPIO_DRVSTR_0 };
const mxc_gpio_cfg_t gpio_cfg_uart3 = { MXC_GPIO3, (MXC_GPIO_PIN_0 | MXC_GPIO_PIN_1),
                                        MXC_GPIO_FUNC_ALT2, MXC_GPIO_PAD_PULL_UP,
                                        MXC_GPIO_VSSEL_VDDIO, MXC_GPIO_DRVSTR_0 };
const mxc_gpio_cfg_t gpio_cfg_uart3_flow = { MXC_GPIO3, (MXC_GPIO_PIN_2 | MXC_GPIO_PIN_3),
                                             MXC_GPIO_FUNC_ALT2, MXC_GPIO_PAD_PULL_UP,
                                             MXC_GPIO_VSSEL_VDDIO, MXC_GPIO_DRVSTR_0 };
const mxc_gpio_cfg_t gpio_cfg_uart3_flow_disable = { MXC_GPIO3, (MXC_GPIO_PIN_2 | MXC_GPIO_PIN_3),
                                                     MXC_GPIO_FUNC_IN, MXC_GPIO_PAD_PULL_UP,
                                                     MXC_GPIO_VSSEL_VDDIO, MXC_GPIO_DRVSTR_0 };

const mxc_gpio_cfg_t antenna_ctrl0 = { MXC_GPIO2, (MXC_GPIO_PIN_17), MXC_GPIO_FUNC_ALT2,
                                       MXC_GPIO_PAD_NONE, MXC_GPIO_VSSEL_VDDIO, MXC_GPIO_DRVSTR_0 };
const mxc_gpio_cfg_t antenna_ctrl1 = { MXC_GPIO2, (MXC_GPIO_PIN_18), MXC_GPIO_FUNC_ALT2,
                                       MXC_GPIO_PAD_NONE, MXC_GPIO_VSSEL_VDDIO, MXC_GPIO_DRVSTR_0 };
const mxc_gpio_cfg_t antenna_ctrl2 = { MXC_GPIO2, (MXC_GPIO_PIN_20), MXC_GPIO_FUNC_ALT2,
                                       MXC_GPIO_PAD_NONE, MXC_GPIO_VSSEL_VDDIO, MXC_GPIO_DRVSTR_0 };
const mxc_gpio_cfg_t antenna_ctrl3 = { MXC_GPIO2, (MXC_GPIO_PIN_21), MXC_GPIO_FUNC_ALT2,
                                       MXC_GPIO_PAD_NONE, MXC_GPIO_VSSEL_VDDIO, MXC_GPIO_DRVSTR_0 };

// Timers are only defined once, depending on package, each timer could be mapped to other pins
const mxc_gpio_cfg_t gpio_cfg_tmr0 = { MXC_GPIO0, (MXC_GPIO_PIN_4 | MXC_GPIO_PIN_5), MXC_GPIO_FUNC_ALT3,
                                       MXC_GPIO_PAD_NONE, MXC_GPIO_VSSEL_VDDIO, MXC_GPIO_DRVSTR_0 };
const mxc_gpio_cfg_t gpio_cfg_tmr0b = { MXC_GPIO0, (MXC_GPIO_PIN_13 | MXC_GPIO_PIN_14), MXC_GPIO_FUNC_ALT2,
                                        MXC_GPIO_PAD_NONE, MXC_GPIO_VSSEL_VDDIO, MXC_GPIO_DRVSTR_0 };
const mxc_gpio_cfg_t gpio_cfg_tmr1 = { MXC_GPIO0, (MXC_GPIO_PIN_7 | MXC_GPIO_PIN_8), MXC_GPIO_FUNC_ALT2,
                                       MXC_GPIO_PAD_NONE, MXC_GPIO_VSSEL_VDDIO, MXC_GPIO_DRVSTR_0 };
const mxc_gpio_cfg_t gpio_cfg_tmr1b = { MXC_GPIO0, (MXC_GPIO_PIN_15 | MXC_GPIO_PIN_17), MXC_GPIO_FUNC_ALT3,
                                        MXC_GPIO_PAD_NONE, MXC_GPIO_VSSEL_VDDIO, MXC_GPIO_DRVSTR_0 };
const mxc_gpio_cfg_t gpio_cfg_tmr2 = { MXC_GPIO1, (MXC_GPIO_PIN_4 | MXC_GPIO_PIN_5), MXC_GPIO_FUNC_ALT2,
                                       MXC_GPIO_PAD_NONE, MXC_GPIO_VSSEL_VDDIO, MXC_GPIO_DRVSTR_0 };
const mxc_gpio_cfg_t gpio_cfg_tmr2b = { MXC_GPIO2, (MXC_GPIO_PIN_20 | MXC_GPIO_PIN_21), MXC_GPIO_FUNC_ALT3,
                                        MXC_GPIO_PAD_NONE, MXC_GPIO_VSSEL_VDDIO, MXC_GPIO_DRVSTR_0 };
const mxc_gpio_cfg_t gpio_cfg_tmr3 = { MXC_GPIO1, (MXC_GPIO_PIN_13 | MXC_GPIO_PIN_14), MXC_GPIO_FUNC_ALT1,
                                       MXC_GPIO_PAD_NONE, MXC_GPIO_VSSEL_VDDIO, MXC_GPIO_DRVSTR_0 };
const mxc_gpio_cfg_t gpio_cfg_tmr3b = { MXC_GPIO2, (MXC_GPIO_PIN_30 | MXC_GPIO_PIN_31), MXC_GPIO_FUNC_ALT3,
                                        MXC_GPIO_PAD_NONE, MXC_GPIO_VSSEL_VDDIO, MXC_GPIO_DRVSTR_0 };
const mxc_gpio_cfg_t gpio_cfg_tmr4 = { MXC_GPIO3, (MXC_GPIO_PIN_4), MXC_GPIO_FUNC_ALT2,
                                       MXC_GPIO_PAD_NONE, MXC_GPIO_VSSEL_VDDIO, MXC_GPIO_DRVSTR_0 };
const mxc_gpio_cfg_t gpio_cfg_tmr5 = { MXC_GPIO3, (MXC_GPIO_PIN_7), MXC_GPIO_FUNC_ALT2,
                                       MXC_GPIO_PAD_NONE, MXC_GPIO_VSSEL_VDDIO, MXC_GPIO_DRVSTR_0 };

const mxc_gpio_cfg_t gpio_cfg_i2s0 = { MXC_GPIO2, (MXC_GPIO_PIN_26 | MXC_GPIO_PIN_27 | MXC_GPIO_PIN_28 | MXC_GPIO_PIN_29),
                                       MXC_GPIO_FUNC_ALT3, MXC_GPIO_PAD_NONE, MXC_GPIO_VSSEL_VDDIO, MXC_GPIO_DRVSTR_0 };
const mxc_gpio_cfg_t gpio_cfg_i2s0_clkext = { MXC_GPIO0, (MXC_GPIO_PIN_23), MXC_GPIO_FUNC_ALT2,
                                              MXC_GPIO_PAD_NONE, MXC_GPIO_VSSEL_VDDIO, MXC_GPIO_DRVSTR_0 };

const mxc_gpio_cfg_t gpio_cfg_spi0 = { MXC_GPIO2, (MXC_GPIO_PIN_27 | MXC_GPIO_PIN_28 | MXC_GPIO_PIN_29),
                                       MXC_GPIO_FUNC_ALT2, MXC_GPIO_PAD_NONE, MXC_GPIO_VSSEL_VDDIO, MXC_GPIO_DRVSTR_0 };
const mxc_gpio_cfg_t gpio_cfg_spi0_ss0 = { MXC_GPIO0, (MXC_GPIO_PIN_22), MXC_GPIO_FUNC_ALT1,
                                           MXC_GPIO_PAD_NONE, MXC_GPIO_VSSEL_VDDIO, MXC_GPIO_DRVSTR_0 };
const mxc_gpio_cfg_t gpio_cfg_spi0_ss1 = { MXC_GPIO2, (MXC_GPIO_PIN_26), MXC_GPIO_FUNC_ALT2,
                                           MXC_GPIO_PAD_NONE, MXC_GPIO_VSSEL_VDDIO, MXC_GPIO_DRVSTR_0 };

const mxc_gpio_cfg_t gpio_cfg_spi1 = { MXC_GPIO1, (MXC_GPIO_PIN_26 | MXC_GPIO_PIN_28 | MXC_GPIO_PIN_29),
                                       MXC_GPIO_FUNC_ALT1, MXC_GPIO_PAD_NONE, MXC_GPIO_VSSEL_VDDIO, MXC_GPIO_DRVSTR_0 };
const mxc_gpio_cfg_t gpio_cfg_spi1_ss0 = { MXC_GPIO1, (MXC_GPIO_PIN_23), MXC_GPIO_FUNC_ALT1,
                                           MXC_GPIO_PAD_NONE, MXC_GPIO_VSSEL_VDDIO, MXC_GPIO_DRVSTR_0 };
const mxc_gpio_cfg_t gpio_cfg_spi1_ss1 = { MXC_GPIO1, (MXC_GPIO_PIN_25), MXC_GPIO_FUNC_ALT1,
                                           MXC_GPIO_PAD_NONE, MXC_GPIO_VSSEL_VDDIO, MXC_GPIO_DRVSTR_0 };
const mxc_gpio_cfg_t gpio_cfg_spi1_ss2 = { MXC_GPIO1, (MXC_GPIO_PIN_24), MXC_GPIO_FUNC_ALT1,
                                           MXC_GPIO_PAD_NONE, MXC_GPIO_VSSEL_VDDIO, MXC_GPIO_DRVSTR_0 };

const mxc_gpio_cfg_t gpio_cfg_spi2 = { MXC_GPIO2, (MXC_GPIO_PIN_2 | MXC_GPIO_PIN_3 | MXC_GPIO_PIN_4),
                                       MXC_GPIO_FUNC_ALT1, MXC_GPIO_PAD_NONE, MXC_GPIO_VSSEL_VDDIO, MXC_GPIO_DRVSTR_0 };
const mxc_gpio_cfg_t gpio_cfg_spi2_ss0 = { MXC_GPIO2, (MXC_GPIO_PIN_5), MXC_GPIO_FUNC_ALT1,
                                           MXC_GPIO_PAD_NONE, MXC_GPIO_VSSEL_VDDIO, MXC_GPIO_DRVSTR_0 };
const mxc_gpio_cfg_t gpio_cfg_spi2_ss1 = { MXC_GPIO2, (MXC_GPIO_PIN_1), MXC_GPIO_FUNC_ALT1,
                                           MXC_GPIO_PAD_NONE, MXC_GPIO_VSSEL_VDDIO, MXC_GPIO_DRVSTR_0 };
const mxc_gpio_cfg_t gpio_cfg_spi2_ss2 = { MXC_GPIO2, (MXC_GPIO_PIN_0), MXC_GPIO_FUNC_ALT1,
                                           MXC_GPIO_PAD_NONE, MXC_GPIO_VSSEL_VDDIO, MXC_GPIO_DRVSTR_0 };

const mxc_gpio_cfg_t gpio_cfg_spi3 = { MXC_GPIO0, (MXC_GPIO_PIN_16 | MXC_GPIO_PIN_20 | MXC_GPIO_PIN_21),
                                       MXC_GPIO_FUNC_ALT1, MXC_GPIO_PAD_NONE, MXC_GPIO_VSSEL_VDDIO, MXC_GPIO_DRVSTR_0 };
const mxc_gpio_cfg_t gpio_cfg_spi3_ss0 = { MXC_GPIO0, (MXC_GPIO_PIN_19), MXC_GPIO_FUNC_ALT1,
                                           MXC_GPIO_PAD_NONE, MXC_GPIO_VSSEL_VDDIO, MXC_GPIO_DRVSTR_0 };
const mxc_gpio_cfg_t gpio_cfg_spi3_ss1 = { MXC_GPIO0, (MXC_GPIO_PIN_13), MXC_GPIO_FUNC_ALT1,
                                           MXC_GPIO_PAD_NONE, MXC_GPIO_VSSEL_VDDIO, MXC_GPIO_DRVSTR_0 };
const mxc_gpio_cfg_t gpio_cfg_spi3_ss2 = { MXC_GPIO0, (MXC_GPIO_PIN_14), MXC_GPIO_FUNC_ALT1,
                                           MXC_GPIO_PAD_NONE, MXC_GPIO_VSSEL_VDDIO, MXC_GPIO_DRVSTR_0 };

const mxc_gpio_cfg_t gpio_cfg_spi4 = { MXC_GPIO1, (MXC_GPIO_PIN_1 | MXC_GPIO_PIN_2 | MXC_GPIO_PIN_3),
                                       MXC_GPIO_FUNC_ALT1, MXC_GPIO_PAD_NONE, MXC_GPIO_VSSEL_VDDIO, MXC_GPIO_DRVSTR_0 };
const mxc_gpio_cfg_t gpio_cfg_spi4_ss0 = { MXC_GPIO1, (MXC_GPIO_PIN_0), MXC_GPIO_FUNC_ALT1,
                                           MXC_GPIO_PAD_NONE, MXC_GPIO_VSSEL_VDDIO, MXC_GPIO_DRVSTR_0 };
const mxc_gpio_cfg_t gpio_cfg_spi4_ss1 = { MXC_GPIO1, (MXC_GPIO_PIN_6), MXC_GPIO_FUNC_ALT1,
                                           MXC_GPIO_PAD_NONE, MXC_GPIO_VSSEL_VDDIO, MXC_GPIO_DRVSTR_0 };
const mxc_gpio_cfg_t gpio_cfg_spi4_ss2 = { MXC_GPIO1, (MXC_GPIO_PIN_11), MXC_GPIO_FUNC_ALT1,
                                           MXC_GPIO_PAD_NONE, MXC_GPIO_VSSEL_VDDIO, MXC_GPIO_DRVSTR_0 };

const mxc_gpio_cfg_t gpio_cfg_spixr = { MXC_GPIO0, (MXC_GPIO_PIN_1 | MXC_GPIO_PIN_2 | MXC_GPIO_PIN_3 | MXC_GPIO_PIN_4 | MXC_GPIO_PIN_5 |
                                        MXC_GPIO_PIN_6), MXC_GPIO_FUNC_ALT1, MXC_GPIO_PAD_NONE, MXC_GPIO_VSSEL_VDDIO, MXC_GPIO_DRVSTR_0 };

const mxc_gpio_cfg_t gpio_cfg_spixf = { MXC_GPIO0, (MXC_GPIO_PIN_1 | MXC_GPIO_PIN_2 | MXC_GPIO_PIN_3 | MXC_GPIO_PIN_4 | MXC_GPIO_PIN_5 |
                                        MXC_GPIO_PIN_6), MXC_GPIO_FUNC_ALT2, MXC_GPIO_PAD_NONE, MXC_GPIO_VSSEL_VDDIO, MXC_GPIO_DRVSTR_0 };

const mxc_gpio_cfg_t gpio_cfg_rtcsqw = { MXC_GPIO4, MXC_GPIO_PIN_1, MXC_GPIO_FUNC_ALT1,
                                         MXC_GPIO_PAD_NONE, MXC_GPIO_VSSEL_VDDIO, MXC_GPIO_DRVSTR_0 };

const mxc_gpio_cfg_t gpio_cfg_hpb = { MXC_GPIO1, (MXC_GPIO_PIN_12 | MXC_GPIO_PIN_13 | MXC_GPIO_PIN_14 | MXC_GPIO_PIN_15 |
                                      MXC_GPIO_PIN_16 | MXC_GPIO_PIN_18 | MXC_GPIO_PIN_19 | MXC_GPIO_PIN_20 | MXC_GPIO_PIN_21),
                                      MXC_GPIO_FUNC_ALT3, MXC_GPIO_PAD_NONE, MXC_GPIO_VSSEL_VDDIO, MXC_GPIO_DRVSTR_0 };
const mxc_gpio_cfg_t gpio_cfg_hpb_cs0 = { MXC_GPIO1, MXC_GPIO_PIN_11, MXC_GPIO_FUNC_ALT3,
                                          MXC_GPIO_PAD_NONE, MXC_GPIO_VSSEL_VDDIO, MXC_GPIO_DRVSTR_0 };
const mxc_gpio_cfg_t gpio_cfg_hpb_cs1 = { MXC_GPIO1, MXC_GPIO_PIN_17, MXC_GPIO_FUNC_ALT3,
                                          MXC_GPIO_PAD_NONE, MXC_GPIO_VSSEL_VDDIO, MXC_GPIO_DRVSTR_0 };

const mxc_gpio_cfg_t gpio_cfg_pt0 = { MXC_GPIO2, MXC_GPIO_PIN_29, MXC_GPIO_FUNC_ALT1,
                                      MXC_GPIO_PAD_NONE, MXC_GPIO_VSSEL_VDDIO, MXC_GPIO_DRVSTR_0 };
const mxc_gpio_cfg_t gpio_cfg_pt1 = { MXC_GPIO1, MXC_GPIO_PIN_7, MXC_GPIO_FUNC_ALT2,
                                      MXC_GPIO_PAD_NONE, MXC_GPIO_VSSEL_VDDIO, MXC_GPIO_DRVSTR_0 };
const mxc_gpio_cfg_t gpio_cfg_pt2 = { MXC_GPIO1, MXC_GPIO_PIN_8, MXC_GPIO_FUNC_ALT2,
                                      MXC_GPIO_PAD_NONE, MXC_GPIO_VSSEL_VDDIO, MXC_GPIO_DRVSTR_0 };
const mxc_gpio_cfg_t gpio_cfg_pt3 = { MXC_GPIO1, MXC_GPIO_PIN_9, MXC_GPIO_FUNC_ALT2,
                                      MXC_GPIO_PAD_NONE, MXC_GPIO_VSSEL_VDDIO, MXC_GPIO_DRVSTR_0 };
const mxc_gpio_cfg_t gpio_cfg_pt4 = { MXC_GPIO1, MXC_GPIO_PIN_10, MXC_GPIO_FUNC_ALT2,
                                      MXC_GPIO_PAD_NONE, MXC_GPIO_VSSEL_VDDIO, MXC_GPIO_DRVSTR_0 };
const mxc_gpio_cfg_t gpio_cfg_pt5 = { MXC_GPIO1, MXC_GPIO_PIN_12, MXC_GPIO_FUNC_ALT1,
                                      MXC_GPIO_PAD_NONE, MXC_GPIO_VSSEL_VDDIO, MXC_GPIO_DRVSTR_0 };
const mxc_gpio_cfg_t gpio_cfg_pt6 = { MXC_GPIO1, MXC_GPIO_PIN_18, MXC_GPIO_FUNC_ALT1,
                                      MXC_GPIO_PAD_NONE, MXC_GPIO_VSSEL_VDDIO, MXC_GPIO_DRVSTR_0 };
const mxc_gpio_cfg_t gpio_cfg_pt7 = { MXC_GPIO1, MXC_GPIO_PIN_19, MXC_GPIO_FUNC_ALT1,
                                      MXC_GPIO_PAD_NONE, MXC_GPIO_VSSEL_VDDIO, MXC_GPIO_DRVSTR_0 };
const mxc_gpio_cfg_t gpio_cfg_pt8 = { MXC_GPIO1, MXC_GPIO_PIN_21, MXC_GPIO_FUNC_ALT1,
                                      MXC_GPIO_PAD_NONE, MXC_GPIO_VSSEL_VDDIO, MXC_GPIO_DRVSTR_0 };
const mxc_gpio_cfg_t gpio_cfg_pt9 = { MXC_GPIO1, MXC_GPIO_PIN_17, MXC_GPIO_FUNC_ALT1,
                                      MXC_GPIO_PAD_NONE, MXC_GPIO_VSSEL_VDDIO, MXC_GPIO_DRVSTR_0 };
const mxc_gpio_cfg_t gpio_cfg_pt10 = { MXC_GPIO2, MXC_GPIO_PIN_24, MXC_GPIO_FUNC_ALT1,
                                       MXC_GPIO_PAD_NONE, MXC_GPIO_VSSEL_VDDIO, MXC_GPIO_DRVSTR_0 };
const mxc_gpio_cfg_t gpio_cfg_pt11 = { MXC_GPIO2, MXC_GPIO_PIN_25, MXC_GPIO_FUNC_ALT1,
                                       MXC_GPIO_PAD_NONE, MXC_GPIO_VSSEL_VDDIO, MXC_GPIO_DRVSTR_0 };
const mxc_gpio_cfg_t gpio_cfg_pt12 = { MXC_GPIO2, MXC_GPIO_PIN_26, MXC_GPIO_FUNC_ALT1,
                                       MXC_GPIO_PAD_NONE, MXC_GPIO_VSSEL_VDDIO, MXC_GPIO_DRVSTR_0 };
const mxc_gpio_cfg_t gpio_cfg_pt13 = { MXC_GPIO2, MXC_GPIO_PIN_27, MXC_GPIO_FUNC_ALT1,
                                       MXC_GPIO_PAD_NONE, MXC_GPIO_VSSEL_VDDIO, MXC_GPIO_DRVSTR_0 };
const mxc_gpio_cfg_t gpio_cfg_pt14 = { MXC_GPIO2, MXC_GPIO_PIN_28, MXC_GPIO_FUNC_ALT1,
                                       MXC_GPIO_PAD_NONE, MXC_GPIO_VSSEL_VDDIO, MXC_GPIO_DRVSTR_0 };
const mxc_gpio_cfg_t gpio_cfg_pt15 = { MXC_GPIO2, MXC_GPIO_PIN_12, MXC_GPIO_FUNC_ALT2,
                                       MXC_GPIO_PAD_NONE, MXC_GPIO_VSSEL_VDDIO, MXC_GPIO_DRVSTR_0 };

// 1-Wire pins need to be at 3.3V so that MXC_GPIO_VSSEL_VDDIOH is selected.
const mxc_gpio_cfg_t gpio_cfg_owm = { MXC_GPIO0, (MXC_GPIO_PIN_7 | MXC_GPIO_PIN_8), MXC_GPIO_FUNC_ALT1,
                                      MXC_GPIO_PAD_NONE, MXC_GPIO_VSSEL_VDDIOH, MXC_GPIO_DRVSTR_0 };
const mxc_gpio_cfg_t gpio_cfg_owmb = { MXC_GPIO1, (MXC_GPIO_PIN_30 | MXC_GPIO_PIN_31), MXC_GPIO_FUNC_ALT1,
                                      MXC_GPIO_PAD_NONE, MXC_GPIO_VSSEL_VDDIOH, MXC_GPIO_DRVSTR_0 };

const mxc_gpio_cfg_t gpio_cfg_adc_ain0 = { MXC_GPIO3, MXC_GPIO_PIN_0, MXC_GPIO_FUNC_ALT1,
                                           MXC_GPIO_PAD_NONE, MXC_GPIO_VSSEL_VDDIO, MXC_GPIO_DRVSTR_0 };
const mxc_gpio_cfg_t gpio_cfg_adc_ain1 = { MXC_GPIO3, MXC_GPIO_PIN_1, MXC_GPIO_FUNC_ALT1,
                                           MXC_GPIO_PAD_NONE, MXC_GPIO_VSSEL_VDDIO, MXC_GPIO_DRVSTR_0 };
const mxc_gpio_cfg_t gpio_cfg_adc_ain2 = { MXC_GPIO3, MXC_GPIO_PIN_2, MXC_GPIO_FUNC_ALT1,
                                           MXC_GPIO_PAD_NONE, MXC_GPIO_VSSEL_VDDIO, MXC_GPIO_DRVSTR_0 };
const mxc_gpio_cfg_t gpio_cfg_adc_ain3 = { MXC_GPIO3, MXC_GPIO_PIN_3, MXC_GPIO_FUNC_ALT1,
                                           MXC_GPIO_PAD_NONE, MXC_GPIO_VSSEL_VDDIO, MXC_GPIO_DRVSTR_0 };
const mxc_gpio_cfg_t gpio_cfg_adc_ain4 = { MXC_GPIO3, MXC_GPIO_PIN_4, MXC_GPIO_FUNC_ALT1,
                                           MXC_GPIO_PAD_NONE, MXC_GPIO_VSSEL_VDDIO, MXC_GPIO_DRVSTR_0 };
const mxc_gpio_cfg_t gpio_cfg_adc_ain5 = { MXC_GPIO3, MXC_GPIO_PIN_5, MXC_GPIO_FUNC_ALT1,
                                           MXC_GPIO_PAD_NONE, MXC_GPIO_VSSEL_VDDIO, MXC_GPIO_DRVSTR_0 };
const mxc_gpio_cfg_t gpio_cfg_adc_ain6 = { MXC_GPIO3, MXC_GPIO_PIN_6, MXC_GPIO_FUNC_ALT1,
                                           MXC_GPIO_PAD_NONE, MXC_GPIO_VSSEL_VDDIO, MXC_GPIO_DRVSTR_0 };
const mxc_gpio_cfg_t gpio_cfg_adc_ain7 = { MXC_GPIO3, MXC_GPIO_PIN_7, MXC_GPIO_FUNC_ALT1,
                                           MXC_GPIO_PAD_NONE, MXC_GPIO_VSSEL_VDDIO, MXC_GPIO_DRVSTR_0 };

const mxc_gpio_cfg_t gpio_cfg_adc_trig_p0_10 = { MXC_GPIO0, MXC_GPIO_PIN_10, MXC_GPIO_FUNC_ALT1,
                                                 MXC_GPIO_PAD_NONE, MXC_GPIO_VSSEL_VDDIO, MXC_GPIO_DRVSTR_0 };
const mxc_gpio_cfg_t gpio_cfg_adc_trig_p1_0 = { MXC_GPIO1, MXC_GPIO_PIN_0, MXC_GPIO_FUNC_ALT2,
                                                MXC_GPIO_PAD_NONE, MXC_GPIO_VSSEL_VDDIO, MXC_GPIO_DRVSTR_0 };
const mxc_gpio_cfg_t gpio_cfg_adc_trig_p2_15 = { MXC_GPIO2, MXC_GPIO_PIN_15, MXC_GPIO_FUNC_ALT2,
                                                 MXC_GPIO_PAD_NONE, MXC_GPIO_VSSEL_VDDIO, MXC_GPIO_DRVSTR_0 };

const mxc_gpio_cfg_t gpio_cfg_cmp0 = { MXC_GPIO3, (MXC_GPIO_PIN_0 | MXC_GPIO_PIN_1), MXC_GPIO_FUNC_ALT1,
                                       MXC_GPIO_PAD_NONE, MXC_GPIO_VSSEL_VDDIO, MXC_GPIO_DRVSTR_0 };
const mxc_gpio_cfg_t gpio_cfg_cmp1 = { MXC_GPIO3, (MXC_GPIO_PIN_2 | MXC_GPIO_PIN_3), MXC_GPIO_FUNC_ALT1,
                                       MXC_GPIO_PAD_NONE, MXC_GPIO_VSSEL_VDDIO, MXC_GPIO_DRVSTR_0 };
const mxc_gpio_cfg_t gpio_cfg_cmp2 = { MXC_GPIO3, (MXC_GPIO_PIN_4 | MXC_GPIO_PIN_5), MXC_GPIO_FUNC_ALT1,
                                       MXC_GPIO_PAD_NONE, MXC_GPIO_VSSEL_VDDIO, MXC_GPIO_DRVSTR_0 };
const mxc_gpio_cfg_t gpio_cfg_cmp3 = { MXC_GPIO3, (MXC_GPIO_PIN_6 | MXC_GPIO_PIN_7), MXC_GPIO_FUNC_ALT1,
                                       MXC_GPIO_PAD_NONE, MXC_GPIO_VSSEL_VDDIO, MXC_GPIO_DRVSTR_0 };

const mxc_gpio_cfg_t gpio_cfg_rv_jtag = { MXC_GPIO0, (MXC_GPIO_PIN_19 | MXC_GPIO_PIN_20 | MXC_GPIO_PIN_21 | MXC_GPIO_PIN_22),
                                          MXC_GPIO_FUNC_ALT2, MXC_GPIO_PAD_NONE, MXC_GPIO_VSSEL_VDDIO, MXC_GPIO_DRVSTR_0 };

const mxc_gpio_cfg_t gpio_cfg_can0 = { MXC_GPIO2, (MXC_GPIO_PIN_22 | MXC_GPIO_PIN_23), MXC_GPIO_FUNC_ALT2,
                                       MXC_GPIO_PAD_NONE, MXC_GPIO_VSSEL_VDDIOH, MXC_GPIO_DRVSTR_0 };
const mxc_gpio_cfg_t gpio_cfg_can1 = { MXC_GPIO2, (MXC_GPIO_PIN_24 | MXC_GPIO_PIN_25), MXC_GPIO_FUNC_ALT2,
                                       MXC_GPIO_PAD_NONE, MXC_GPIO_VSSEL_VDDIOH, MXC_GPIO_DRVSTR_0 };

// SPI v2 Pin Definitions
const mxc_gpio_cfg_t gpio_cfg_spi0_standard = { MXC_GPIO2, (MXC_GPIO_PIN_27 | MXC_GPIO_PIN_28 | MXC_GPIO_PIN_29),
                                                MXC_GPIO_FUNC_ALT1, MXC_GPIO_PAD_NONE, MXC_GPIO_VSSEL_VDDIO, MXC_GPIO_DRVSTR_0 };
const mxc_gpio_cfg_t gpio_cfg_spi0_3wire = { MXC_GPIO2, (MXC_GPIO_PIN_28 | MXC_GPIO_PIN_29), MXC_GPIO_FUNC_ALT2,
                                             MXC_GPIO_PAD_NONE, MXC_GPIO_VSSEL_VDDIO, MXC_GPIO_DRVSTR_0 };
const mxc_gpio_cfg_t gpio_cfg_spi0_dual = { MXC_GPIO2, (MXC_GPIO_PIN_27 | MXC_GPIO_PIN_28 | MXC_GPIO_PIN_29), MXC_GPIO_FUNC_ALT1,
                                            MXC_GPIO_PAD_NONE, MXC_GPIO_VSSEL_VDDIO, MXC_GPIO_DRVSTR_0 };
const mxc_gpio_cfg_t gpio_cfg_spi0_quad = { MXC_GPIO2, (MXC_GPIO_PIN_27 | MXC_GPIO_PIN_28 | MXC_GPIO_PIN_29 | MXC_GPIO_PIN_30 | MXC_GPIO_PIN_31),
                                            MXC_GPIO_FUNC_ALT2, MXC_GPIO_PAD_NONE, MXC_GPIO_VSSEL_VDDIO, MXC_GPIO_DRVSTR_0 };

const mxc_gpio_cfg_t gpio_cfg_spi1_standard = { MXC_GPIO1, (MXC_GPIO_PIN_26 | MXC_GPIO_PIN_28 | MXC_GPIO_PIN_29),
                                                MXC_GPIO_FUNC_ALT1, MXC_GPIO_PAD_NONE, MXC_GPIO_VSSEL_VDDIO, MXC_GPIO_DRVSTR_0 };
const mxc_gpio_cfg_t gpio_cfg_spi1_3wire = { MXC_GPIO1, (MXC_GPIO_PIN_26 | MXC_GPIO_PIN_29), MXC_GPIO_FUNC_ALT1,
                                             MXC_GPIO_PAD_NONE, MXC_GPIO_VSSEL_VDDIO, MXC_GPIO_DRVSTR_0 };
const mxc_gpio_cfg_t gpio_cfg_spi1_dual = { MXC_GPIO1, (MXC_GPIO_PIN_26 | MXC_GPIO_PIN_28 | MXC_GPIO_PIN_29),
                                            MXC_GPIO_FUNC_ALT1, MXC_GPIO_PAD_NONE, MXC_GPIO_VSSEL_VDDIO, MXC_GPIO_DRVSTR_0 };
const mxc_gpio_cfg_t gpio_cfg_spi1_quad_0 = { MXC_GPIO1, (MXC_GPIO_PIN_26 | MXC_GPIO_PIN_28 | MXC_GPIO_PIN_29),
                                              MXC_GPIO_FUNC_ALT1, MXC_GPIO_PAD_NONE, MXC_GPIO_VSSEL_VDDIO, MXC_GPIO_DRVSTR_0 };
const mxc_gpio_cfg_t gpio_cfg_spi1_quad_1 = { MXC_GPIO1, (MXC_GPIO_PIN_30 | MXC_GPIO_PIN_11), MXC_GPIO_FUNC_ALT2,
                                              MXC_GPIO_PAD_NONE, MXC_GPIO_VSSEL_VDDIO, MXC_GPIO_DRVSTR_0 };

const mxc_gpio_cfg_t gpio_cfg_spi2_standard = { MXC_GPIO2, (MXC_GPIO_PIN_2 | MXC_GPIO_PIN_3 | MXC_GPIO_PIN_4),
                                                MXC_GPIO_FUNC_ALT1, MXC_GPIO_PAD_NONE, MXC_GPIO_VSSEL_VDDIO, MXC_GPIO_DRVSTR_0 };
const mxc_gpio_cfg_t gpio_cfg_spi2_3wire = { MXC_GPIO2, (MXC_GPIO_PIN_2 | MXC_GPIO_PIN_4), MXC_GPIO_FUNC_ALT1,
                                             MXC_GPIO_PAD_NONE, MXC_GPIO_VSSEL_VDDIO, MXC_GPIO_DRVSTR_0 };
const mxc_gpio_cfg_t gpio_cfg_spi2_dual = { MXC_GPIO2, (MXC_GPIO_PIN_2 | MXC_GPIO_PIN_3 | MXC_GPIO_PIN_4),
                                            MXC_GPIO_FUNC_ALT1, MXC_GPIO_PAD_NONE, MXC_GPIO_VSSEL_VDDIO, MXC_GPIO_DRVSTR_0 };
const mxc_gpio_cfg_t gpio_cfg_spi2_quad_0 = { MXC_GPIO0, (MXC_GPIO_PIN_2 | MXC_GPIO_PIN_3 | MXC_GPIO_PIN_4),
                                              MXC_GPIO_FUNC_ALT1, MXC_GPIO_PAD_NONE, MXC_GPIO_VSSEL_VDDIO, MXC_GPIO_DRVSTR_0 };
const mxc_gpio_cfg_t gpio_cfg_spi2_quad_1 = { MXC_GPIO0, (MXC_GPIO_PIN_6 | MXC_GPIO_PIN_7), MXC_GPIO_FUNC_ALT2,
                                              MXC_GPIO_PAD_NONE, MXC_GPIO_VSSEL_VDDIO, MXC_GPIO_DRVSTR_0 };

const mxc_gpio_cfg_t gpio_cfg_spi3_standard = { MXC_GPIO0, (MXC_GPIO_PIN_16 | MXC_GPIO_PIN_20 | MXC_GPIO_PIN_21), MXC_GPIO_FUNC_ALT1,
                                                MXC_GPIO_PAD_NONE, MXC_GPIO_VSSEL_VDDIO, MXC_GPIO_DRVSTR_0 };
const mxc_gpio_cfg_t gpio_cfg_spi3_3wire = { MXC_GPIO0, (MXC_GPIO_PIN_16 | MXC_GPIO_PIN_21), MXC_GPIO_FUNC_ALT1,
                                             MXC_GPIO_PAD_NONE, MXC_GPIO_VSSEL_VDDIO, MXC_GPIO_DRVSTR_0 };
const mxc_gpio_cfg_t gpio_cfg_spi3_dual = { MXC_GPIO0, (MXC_GPIO_PIN_16 | MXC_GPIO_PIN_20 | MXC_GPIO_PIN_21),
                                            MXC_GPIO_FUNC_ALT1, MXC_GPIO_PAD_NONE, MXC_GPIO_VSSEL_VDDIO, MXC_GPIO_DRVSTR_0 };
const mxc_gpio_cfg_t gpio_cfg_spi3_quad = { MXC_GPIO0, (MXC_GPIO_PIN_15 | MXC_GPIO_PIN_16 | MXC_GPIO_PIN_17 | MXC_GPIO_PIN_20 | MXC_GPIO_PIN_21),
                                            MXC_GPIO_FUNC_ALT1, MXC_GPIO_PAD_NONE, MXC_GPIO_VSSEL_VDDIO, MXC_GPIO_DRVSTR_0 };

const mxc_gpio_cfg_t gpio_cfg_spi4_standard = { MXC_GPIO1, (MXC_GPIO_PIN_1 | MXC_GPIO_PIN_2 | MXC_GPIO_PIN_3),
                                                MXC_GPIO_FUNC_ALT1, MXC_GPIO_PAD_NONE, MXC_GPIO_VSSEL_VDDIO, MXC_GPIO_DRVSTR_0 };
const mxc_gpio_cfg_t gpio_cfg_spi4_3wire = { MXC_GPIO1, (MXC_GPIO_PIN_1 | MXC_GPIO_PIN_3), MXC_GPIO_FUNC_ALT1,
                                             MXC_GPIO_PAD_NONE, MXC_GPIO_VSSEL_VDDIO, MXC_GPIO_DRVSTR_0 };
const mxc_gpio_cfg_t gpio_cfg_spi4_dual = { MXC_GPIO1, (MXC_GPIO_PIN_1 | MXC_GPIO_PIN_2 | MXC_GPIO_PIN_3),
                                            MXC_GPIO_FUNC_ALT1, MXC_GPIO_PAD_NONE, MXC_GPIO_VSSEL_VDDIO, MXC_GPIO_DRVSTR_0 };
const mxc_gpio_cfg_t gpio_cfg_spi4_quad = { MXC_GPIO1, (MXC_GPIO_PIN_1 | MXC_GPIO_PIN_2 | MXC_GPIO_PIN_3 | MXC_GPIO_PIN_4 | MXC_GPIO_PIN_5),
                                            MXC_GPIO_FUNC_ALT1, MXC_GPIO_PAD_NONE, MXC_GPIO_VSSEL_VDDIO, MXC_GPIO_DRVSTR_0 };

// SPI v2 Target Selects Pin Definitions
const mxc_gpio_cfg_t gpio_cfg_spi0_ts0 = { MXC_GPIO0, MXC_GPIO_PIN_22, MXC_GPIO_FUNC_ALT1,
                                           MXC_GPIO_PAD_NONE, MXC_GPIO_VSSEL_VDDIO, MXC_GPIO_DRVSTR_0 };
const mxc_gpio_cfg_t gpio_cfg_spi0_ts1 = { MXC_GPIO2, MXC_GPIO_PIN_26, MXC_GPIO_FUNC_ALT2,
                                           MXC_GPIO_PAD_NONE, MXC_GPIO_VSSEL_VDDIO, MXC_GPIO_DRVSTR_0 };

const mxc_gpio_cfg_t gpio_cfg_spi1_ts0 = { MXC_GPIO1, MXC_GPIO_PIN_23, MXC_GPIO_FUNC_ALT1,
                                           MXC_GPIO_PAD_NONE, MXC_GPIO_VSSEL_VDDIO, MXC_GPIO_DRVSTR_0 };
const mxc_gpio_cfg_t gpio_cfg_spi1_ts1 = { MXC_GPIO1, MXC_GPIO_PIN_25, MXC_GPIO_FUNC_ALT1,
                                           MXC_GPIO_PAD_NONE, MXC_GPIO_VSSEL_VDDIO, MXC_GPIO_DRVSTR_0 };
const mxc_gpio_cfg_t gpio_cfg_spi1_ts2 = { MXC_GPIO1, MXC_GPIO_PIN_24, MXC_GPIO_FUNC_ALT1,
                                           MXC_GPIO_PAD_NONE, MXC_GPIO_VSSEL_VDDIO, MXC_GPIO_DRVSTR_0 };

const mxc_gpio_cfg_t gpio_cfg_spi2_ts0 = { MXC_GPIO2, MXC_GPIO_PIN_5, MXC_GPIO_FUNC_ALT1,
                                           MXC_GPIO_PAD_NONE, MXC_GPIO_VSSEL_VDDIO, MXC_GPIO_DRVSTR_0 };
const mxc_gpio_cfg_t gpio_cfg_spi2_ts1 = { MXC_GPIO2, MXC_GPIO_PIN_1, MXC_GPIO_FUNC_ALT1,
                                           MXC_GPIO_PAD_NONE, MXC_GPIO_VSSEL_VDDIO, MXC_GPIO_DRVSTR_0 };
const mxc_gpio_cfg_t gpio_cfg_spi2_ts2 = { MXC_GPIO1, MXC_GPIO_PIN_27, MXC_GPIO_FUNC_ALT1,
                                           MXC_GPIO_PAD_NONE, MXC_GPIO_VSSEL_VDDIO, MXC_GPIO_DRVSTR_0 };

const mxc_gpio_cfg_t gpio_cfg_spi3_ts0 = { MXC_GPIO0, MXC_GPIO_PIN_19, MXC_GPIO_FUNC_ALT1,
                                           MXC_GPIO_PAD_NONE, MXC_GPIO_VSSEL_VDDIO, MXC_GPIO_DRVSTR_0 };
const mxc_gpio_cfg_t gpio_cfg_spi3_ts1 = { MXC_GPIO0, MXC_GPIO_PIN_13, MXC_GPIO_FUNC_ALT1,
                                           MXC_GPIO_PAD_NONE, MXC_GPIO_VSSEL_VDDIO, MXC_GPIO_DRVSTR_0 };
const mxc_gpio_cfg_t gpio_cfg_spi3_ts2 = { MXC_GPIO0, MXC_GPIO_PIN_14, MXC_GPIO_FUNC_ALT1,
                                           MXC_GPIO_PAD_NONE, MXC_GPIO_VSSEL_VDDIO, MXC_GPIO_DRVSTR_0 };

const mxc_gpio_cfg_t gpio_cfg_spi4_ts0 = { MXC_GPIO1, MXC_GPIO_PIN_0, MXC_GPIO_FUNC_ALT1,
                                           MXC_GPIO_PAD_NONE, MXC_GPIO_VSSEL_VDDIO, MXC_GPIO_DRVSTR_0 };
const mxc_gpio_cfg_t gpio_cfg_spi4_ts1 = { MXC_GPIO1, MXC_GPIO_PIN_6, MXC_GPIO_FUNC_ALT1,
                                           MXC_GPIO_PAD_NONE, MXC_GPIO_VSSEL_VDDIO, MXC_GPIO_DRVSTR_0 };
const mxc_gpio_cfg_t gpio_cfg_spi4_ts2 = { MXC_GPIO1, MXC_GPIO_PIN_11, MXC_GPIO_FUNC_ALT1,
                                           MXC_GPIO_PAD_NONE, MXC_GPIO_VSSEL_VDDIO, MXC_GPIO_DRVSTR_0 };
