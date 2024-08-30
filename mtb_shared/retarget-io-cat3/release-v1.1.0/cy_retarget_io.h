/***********************************************************************************************//**
 * \file cy_retarget_io.h
 *
 * \brief
 * Provides APIs for transmitting messages to or from the XMC™ board via standard
 * printf/scanf functions. Messages are transmitted over a UART connection which
 * is generally connected to a host machine. The UART peripheral instance and
 * Tx/Rx pins must be already configured before calling cy_retarget_io_init.
 * NOTE: If the application is built using newlib-nano, by default, floating
 * point format strings (%f) are not supported. To enable this support you must
 * add '-u _printf_float' to the linker command line.
 *
 ***************************************************************************************************
 * \copyright
 * Copyright 2021-2022 Cypress Semiconductor Corporation (an Infineon company) or
 * an affiliate of Cypress Semiconductor Corporation
 *
 * SPDX-License-Identifier: Apache-2.0
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
 **************************************************************************************************/

/**
 * \addtogroup group_board_libs Retarget IO
 * \{
 */

#pragma once

#include <stdio.h>
#include "cy_result.h"
#include "xmc_usic.h"

#if defined(__cplusplus)
extern "C" {
#endif

/** UART channel handle */
typedef struct
{
    XMC_USIC_CH_t* channel;
} cy_retarget_io_uart_t;

/** UART channel handle used by this library */
extern cy_retarget_io_uart_t cy_retarget_io_uart_obj;

#ifdef DOXYGEN

/** Defining this macro enables conversion of line feed (LF) into carriage
 * return followed by line feed (CR & LF) on the output direction (STDOUT). You
 * can define this macro through the DEFINES variable in the application
 * Makefile.
 */
#define CY_RETARGET_IO_CONVERT_LF_TO_CRLF

#endif // DOXYGEN

/**
 * \brief Initialization function for redirecting low level IO commands to allow
 * sending messages over a UART interface.
 *
 * \note This function doesn't perform configuration of the USIC peripheral
 * channel or Tx/Rx pins. The hardware resources must be already configured
 * before calling this function.
 *
 * In an RTOS environment, this function must be called after the RTOS has been
 * initialized.
 *
 * \param channel Pointer to USIC channel handler
 * \returns CY_RSLT_SUCCESS if successfully initialized, else an error about
 * what went wrong
 */
cy_rslt_t cy_retarget_io_init(XMC_USIC_CH_t* channel);

/**
 * \brief Checks whether the data is currently written to the serial console.
 * \returns true if there are pending TX transactions, otherwise false
 */
bool cy_retarget_io_is_tx_active();

/**
 * \brief Releases the UART interface allowing it to be used for other purposes.
 * After calling this, printf and related functions will no longer work.
 */
void cy_retarget_io_deinit(void);

#if defined(__cplusplus)
}
#endif

/** \} group_board_libs */
