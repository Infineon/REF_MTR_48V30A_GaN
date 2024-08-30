/*******************************************************************************
 * File Name: cycfg_system.c
 *
 * Description:
 * System configuration
 * This file was automatically generated and should not be modified.
 * Configurator Backend 3.20.0
 * device-db 4.11.1.5194
 * mtb-xmclib-cat3 4.3.0.4408
 *
 *******************************************************************************
 * Copyright 2024 Cypress Semiconductor Corporation (an Infineon company) or
 * an affiliate of Cypress Semiconductor Corporation.
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
 ******************************************************************************/

#include "cycfg_system.h"

#define CLOCK_CCUCLK_ENABLED 1U
#define CLOCK_CCUCLK_DIV 1U
#define CLOCK_CPUCLK_DIV 1U
#define CLOCK_FOFI_CALIBRATION_MODE XMC_SCU_CLOCK_FOFI_CALIBRATION_MODE_FACTORY
#define CLOCK_OSCHP_ENABLED 1U
#define CLOCK_OSCHP_MODE XMC_SCU_CLOCK_OSCHP_MODE_OSC
#define CLOCK_PERICLK_DIV 1U
#define CLOCK_SYSPLL_ENABLED 1U
#define CLOCK_SYSPLL_SEL XMC_SCU_CLOCK_SYSPLLCLKSRC_OSCHP
#define CLOCK_SYSPLL_MODE XMC_SCU_CLOCK_SYSPLL_MODE_NORMAL
#define CLOCK_SYSPLL_PDIV 1U
#define CLOCK_SYSPLL_NDIV 20U
#define CLOCK_SYSPLL_KDIV 2U
#define CLOCK_RTCCLK_SEL XMC_SCU_HIB_RTCCLKSRC_OSI
#define CLOCK_STDBYCLK_SEL XMC_SCU_HIB_STDBYCLKSRC_OSI
#define CLOCK_SYSCLK_SEL XMC_SCU_CLOCK_SYSCLKSRC_PLL
#define CLOCK_SYSCLK_DIV 2U

void SystemCoreClockSetup(void)
{
    XMC_SCU_CLOCK_CONFIG_t clock_config =
  {
  #if defined(CLOCK_SYSPLL_ENABLED)
    .syspll_config.n_div = CLOCK_SYSPLL_NDIV,
    .syspll_config.p_div = CLOCK_SYSPLL_PDIV,
    .syspll_config.k_div = CLOCK_SYSPLL_KDIV,
    .syspll_config.clksrc = CLOCK_SYSPLL_SEL,
    .syspll_config.mode = CLOCK_SYSPLL_MODE,
  #else
    .syspll_config.mode = XMC_SCU_CLOCK_SYSPLL_MODE_DISABLED,
  #endif
  #if defined(CLOCK_OSCHP_ENABLED)
    .enable_oschp = true,
  #endif
  #if defined(CLOCK_OSCLP_ENABLED)
    .enable_osculp = true,
  #endif
    .calibration_mode = CLOCK_FOFI_CALIBRATION_MODE,
    .fstdby_clksrc = CLOCK_STDBYCLK_SEL,
    .fsys_clksrc = CLOCK_SYSCLK_SEL,
    .fsys_clkdiv = CLOCK_SYSCLK_DIV,
    .fcpu_clkdiv = CLOCK_CPUCLK_DIV,
  #if defined(CLOCK_CCUCLK_ENABLED)
    .fccu_clkdiv = CLOCK_CCUCLK_DIV,
  #endif
    .fperipheral_clkdiv = CLOCK_PERICLK_DIV
  };
 
 #if defined(CLOCK_EXTCLK_ENABLED)
  /* External output source clock */
  XMC_SCU_CLOCK_SetExternalOutputClockSource(CLOCK_EXTCLK_SEL);
  /* External clock divider setting */
  XMC_SCU_CLOCK_SetExternalOutputClockDivider(CLOCK_EXTCLK_DIV);
 #endif
   
  XMC_SCU_CLOCK_Init(&clock_config);
 
  /* RTC source clock */
  XMC_SCU_HIB_SetRtcClockSource(CLOCK_RTCCLK_SEL);
 
 #if defined(CLOCK_USBCLK_ENABLED)
   /* USB/SDMMC source clock */
   XMC_SCU_CLOCK_SetUsbClockSource(CLOCK_USBCLK_SEL);
   /* USB/SDMMC divider setting */
   XMC_SCU_CLOCK_SetUsbClockDivider(CLOCK_USBCLK_DIV);
 #endif
 
 #if defined(CLOCK_USBPLL_ENABLED)
  XMC_SCU_CLOCK_EnableUsbPll();
  XMC_SCU_CLOCK_StartUsbPll(CLOCK_USBPLL_PDIV, CLOCK_USBPLL_NDIV);
 #endif  
 #if defined(CLOCK_ECATCLK_ENABLED)
  /* ECAT source clock */
  XMC_SCU_CLOCK_SetECATClockSource(CLOCK_ECATCLK_SEL);
  /* ECAT divider setting */
  XMC_SCU_CLOCK_SetECATClockDivider(CLOCK_ECATCLK_DIV);
 #endif
 
 #if defined(CLOCK_WDTCLK_ENABLED)
  /* WDT source clock */
  XMC_SCU_CLOCK_SetWdtClockSource(CLOCK_WDTCLK_SEL);
  /* WDT divider setting */
  XMC_SCU_CLOCK_SetWdtClockDivider(CLOCK_WDTCLK_DIV);
 #endif
 
 #if defined(CLOCK_EBUCLK_ENABLED)
  /* EBU divider setting */
  XMC_SCU_CLOCK_SetEbuClockDivider(CLOCK_EBUCLK_DIV);
 #endif 
  #if defined(CLOCK_USBCLK_ENABLED)
  XMC_SCU_CLOCK_EnableClock(XMC_SCU_CLOCK_USB);
 #endif
 #if defined(CLOCK_SDCLK_ENABLED)
  XMC_SCU_CLOCK_EnableClock(XMC_SCU_CLOCK_MMC);
 #endif
 #if defined(CLOCK_ETHCLK_ENABLED)
  XMC_SCU_CLOCK_EnableClock(XMC_SCU_CLOCK_ETH);
 #endif
 #if defined(CLOCK_EBUCLK_ENABLED)
  XMC_SCU_CLOCK_EnableClock(XMC_SCU_CLOCK_EBU);
 #endif
 #if defined(CLOCK_CCUCLK_ENABLED)
  XMC_SCU_CLOCK_EnableClock(XMC_SCU_CLOCK_CCU);
 #endif
 #if defined(CLOCK_WDTCLK_ENABLED)
  XMC_SCU_CLOCK_EnableClock(XMC_SCU_CLOCK_WDT);
 #endif
}
uint32_t OSCHP_GetFrequency(void)
{
    return 16000000U;
}
