/**
 * @file xmc_gpio.c
 *
 * @cond
 *****************************************************************************
 * XMClib - XMC Peripheral Driver Library
 *
 * Copyright (c) 2015-2020, Infineon Technologies AG
 * All rights reserved.
 *
 * Boost Software License - Version 1.0 - August 17th, 2003
 *
 * Permission is hereby granted, free of charge, to any person or organization
 * obtaining a copy of the software and accompanying documentation covered by
 * this license (the "Software") to use, reproduce, display, distribute,
 * execute, and transmit the Software, and to prepare derivative works of the
 * Software, and to permit third-parties to whom the Software is furnished to
 * do so, all subject to the following:
 *
 * The copyright notices in the Software and this entire statement, including
 * the above license grant, this restriction and the following disclaimer,
 * must be included in all copies of the Software, in whole or in part, and
 * all derivative works of the Software, unless such copies or derivative
 * works are solely in the form of machine-executable object code generated by
 * a source language processor.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE, TITLE AND NON-INFRINGEMENT. IN NO EVENT
 * SHALL THE COPYRIGHT HOLDERS OR ANYONE DISTRIBUTING THE SOFTWARE BE LIABLE
 * FOR ANY DAMAGES OR OTHER LIABILITY, WHETHER IN CONTRACT, TORT OR OTHERWISE,
 * ARISING FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER
 * DEALINGS IN THE SOFTWARE.
 *
 * To improve the quality of the software, users are encouraged to share
 * modifications, enhancements or bug fixes with Infineon Technologies AG
 * at XMCSupport@infineon.com.
 *****************************************************************************
 *
 * @endcond
 *
 */

/*******************************************************************************
 * HEADER FILES
 *******************************************************************************/

#include "xmc_gpio.h"

/*******************************************************************************
 * MACROS
 *******************************************************************************/

#define PORT_HWSEL_Msk PORT0_HWSEL_HW0_Msk

/*******************************************************************************
 * API IMPLEMENTATION
 *******************************************************************************/

void XMC_GPIO_SetMode(XMC_GPIO_PORT_t *const port, const uint8_t pin, const XMC_GPIO_MODE_t mode)
{
  XMC_ASSERT("XMC_GPIO_SetMode: Invalid port", XMC_GPIO_CHECK_PORT(port));
  XMC_ASSERT("XMC_GPIO_SetMode: Invalid mode", XMC_GPIO_IsModeValid(mode));
  uint32_t iocr;

  iocr = port->IOCR[(uint32_t)pin >> 2U] & (~(uint32_t)((uint32_t)PORT_IOCR_PC_Msk << ((uint32_t)PORT_IOCR_PC_Size * ((uint32_t)pin & 0x3U))));
  iocr |= (uint32_t)mode << ((uint32_t)PORT_IOCR_PC_Size * ((uint32_t)pin & 0x3U));
  port->IOCR[(uint32_t)pin >> 2U] = iocr;
}

void XMC_GPIO_SetHardwareControl(XMC_GPIO_PORT_t *const port, const uint8_t pin, const XMC_GPIO_HWCTRL_t hwctrl)
{
  XMC_ASSERT("XMC_GPIO_SetHardwareControl: Invalid port", XMC_GPIO_CHECK_PORT(port));
  XMC_ASSERT("XMC_GPIO_SetHardwareControl: Invalid hwctrl", XMC_GPIO_CHECK_HWCTRL(hwctrl));

  port->HWSEL &= ~(uint32_t)((uint32_t)PORT_HWSEL_Msk << ((uint32_t)pin << 1U));
  port->HWSEL |= (uint32_t)hwctrl << ((uint32_t)pin << 1U);
}
