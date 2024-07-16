/******************************************************************************
 *
 * Copyright (C) 2022-2023 Maxim Integrated Products, Inc. (now owned by 
 * Analog Devices, Inc.),
 * Copyright (C) 2023-2024 Analog Devices, Inc.
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

/*! \file cli.h
    \brief A CLI Implementation using a command table to store command string, function pointer and a help string is dispatched.

    Details. 

    1.  Line Accumlator 

    Reads the incoming bytes
    Accumulates into a line buffer
    Echos char back to the emulator
    Handles backspace

    2.  Prcoess Command

    Processes input into a series of tokens
    All tokes are seperated by whitespace characters 
    Lookup first token in a table of functions
    Dispatch to handler functions 
*/

#ifndef LIBRARIES_CLI_INC_CLI_H_
#define LIBRARIES_CLI_INC_CLI_H_

#include <stdint.h>
#include "uart.h"

/**
 * @brief Command handler function prototype. Once a command is entered in the CLI, it is parsed
 *        ("tokenized") into an argument vector. The argument counter and argument vector are
 *        passed to the command's handler function where the command is executed. 
 * 
 * @param argc      Number of tokens in the argument vector
 * @param argv[]    Array of arguments storing different tokens of the command string in the
 *                  same order as they were passed in the command line. (argv[0] is the command,
 *                  argv[1:argc] are the arguments (if any are passed), and the last element in the
 *                  argument vector is always a NULL pointer.)
 * 
 * @returns E_NO_ERROR if successful, otherwise an error code (error code must be a negative integer)
 */
typedef int (*command_handler_t)(int argc, char *argv[]);

/**
 * @brief Structure used to define the commands supported by the CLI
 */
typedef struct {
    const char *cmd; /**< name of the command (as it should be entered on the command line) */
    const char *usage; /**< string to show how the command should be entered on the command line */
    const char *description; /**< string describing what the command does */
    command_handler_t handler; /**< function pointer of the command handler function */
} command_t;

/**
 * @brief Initializes the CLI state variables and configures the uart for CLI operations.
 * 
 * @param uart          Pointer to UART instance to use for the CLI
 * @param commands      Pointer to the command table storing user-defined CLI commands
 * @param num_commands  Number of commands in the command table
 * 
 * @return E_NO_ERROR if successful, otherwise an error code.
 */
int MXC_CLI_Init(mxc_uart_regs_t *uart, const command_t *commands, unsigned int num_commands);

/**
 * @brief Shuts down the CLI. (UART will remain enabled.)
 * 
 * @return E_NO_ERROR if successful, otheriwse an error code.
 */
int MXC_CLI_Shutdown(void);

/**
 * @brief IRQ Handler for the CLI UART. This function should be called from the
 *        MXC_UARTx_Handler in the user application.
 */
void MXC_CLI_Handler(void);

#endif // LIBRARIES_CLI_INC_CLI_H_