/*******************************************************************************
 * File Name: cycfg_pins.c
 *
 * Description:
 * Pin configuration
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

#include "cycfg_pins.h"

const XMC_GPIO_CONFIG_t PWMUL_config =
{
    .mode = (XMC_GPIO_MODE_t)PWMUL_MODE,
    .output_level = XMC_GPIO_OUTPUT_LEVEL_LOW,
};
const XMC_GPIO_CONFIG_t PWMVL_config =
{
    .mode = (XMC_GPIO_MODE_t)PWMVL_MODE,
    .output_level = XMC_GPIO_OUTPUT_LEVEL_LOW,
};
const XMC_GPIO_CONFIG_t PWMWL_config =
{
    .mode = (XMC_GPIO_MODE_t)PWMWL_MODE,
    .output_level = XMC_GPIO_OUTPUT_LEVEL_LOW,
};
const XMC_GPIO_CONFIG_t PWMUH_config =
{
    .mode = (XMC_GPIO_MODE_t)PWMUH_MODE,
    .output_level = XMC_GPIO_OUTPUT_LEVEL_LOW,
};
const XMC_GPIO_CONFIG_t PWMVH_config =
{
    .mode = (XMC_GPIO_MODE_t)PWMVH_MODE,
    .output_level = XMC_GPIO_OUTPUT_LEVEL_LOW,
};
const XMC_GPIO_CONFIG_t PWMWH_config =
{
    .mode = (XMC_GPIO_MODE_t)PWMWH_MODE,
    .output_level = XMC_GPIO_OUTPUT_LEVEL_LOW,
};
const XMC_GPIO_CONFIG_t PRE_CHARGE_config =
{
    .mode = (XMC_GPIO_MODE_t)PRE_CHARGE_MODE,
    .output_level = XMC_GPIO_OUTPUT_LEVEL_LOW,
    .output_strength = XMC_GPIO_OUTPUT_STRENGTH_STRONG_SOFT_EDGE,
};
const XMC_GPIO_CONFIG_t TEST_PIN0_config =
{
    .mode = (XMC_GPIO_MODE_t)TEST_PIN0_MODE,
    .output_level = XMC_GPIO_OUTPUT_LEVEL_LOW,
    .output_strength = XMC_GPIO_OUTPUT_STRENGTH_STRONG_SOFT_EDGE,
};
const XMC_GPIO_CONFIG_t TEST_PIN1_config =
{
    .mode = (XMC_GPIO_MODE_t)TEST_PIN1_MODE,
    .output_level = XMC_GPIO_OUTPUT_LEVEL_LOW,
    .output_strength = XMC_GPIO_OUTPUT_STRENGTH_STRONG_SOFT_EDGE,
};
const XMC_GPIO_CONFIG_t IUP_config =
{
    .mode = (XMC_GPIO_MODE_t)IUP_MODE,
};
const XMC_GPIO_CONFIG_t IVP_config =
{
    .mode = (XMC_GPIO_MODE_t)IVP_MODE,
};
const XMC_GPIO_CONFIG_t IWP_config =
{
    .mode = (XMC_GPIO_MODE_t)IWP_MODE,
};
const XMC_GPIO_CONFIG_t T_POWER_config =
{
    .mode = (XMC_GPIO_MODE_t)T_POWER_MODE,
};
const XMC_GPIO_CONFIG_t VBUS_config =
{
    .mode = (XMC_GPIO_MODE_t)VBUS_MODE,
};
const XMC_GPIO_CONFIG_t TEST_PIN2_config =
{
    .mode = (XMC_GPIO_MODE_t)TEST_PIN2_MODE,
    .output_level = XMC_GPIO_OUTPUT_LEVEL_LOW,
    .output_strength = XMC_GPIO_OUTPUT_STRENGTH_STRONG_SOFT_EDGE,
};
const XMC_GPIO_CONFIG_t TEST_PIN3_config =
{
    .mode = (XMC_GPIO_MODE_t)TEST_PIN3_MODE,
    .output_level = XMC_GPIO_OUTPUT_LEVEL_LOW,
    .output_strength = XMC_GPIO_OUTPUT_STRENGTH_STRONG_SOFT_EDGE,
};
const XMC_GPIO_CONFIG_t N_HALL_EN_config =
{
    .mode = (XMC_GPIO_MODE_t)N_HALL_EN_MODE,
    .output_level = XMC_GPIO_OUTPUT_LEVEL_LOW,
    .output_strength = XMC_GPIO_OUTPUT_STRENGTH_STRONG_SOFT_EDGE,
};
const XMC_GPIO_CONFIG_t HALL_2_config =
{
    .mode = (XMC_GPIO_MODE_t)HALL_2_MODE,
};
const XMC_GPIO_CONFIG_t HALL_1_config =
{
    .mode = (XMC_GPIO_MODE_t)HALL_1_MODE,
};
const XMC_GPIO_CONFIG_t HALL_0_config =
{
    .mode = (XMC_GPIO_MODE_t)HALL_0_MODE,
};
const XMC_GPIO_CONFIG_t SB_CAN_config =
{
    .mode = (XMC_GPIO_MODE_t)SB_CAN_MODE,
    .output_level = XMC_GPIO_OUTPUT_LEVEL_LOW,
    .output_strength = XMC_GPIO_OUTPUT_STRENGTH_STRONG_SOFT_EDGE,
};
const XMC_GPIO_CONFIG_t CAN_RX_config =
{
    .mode = (XMC_GPIO_MODE_t)CAN_RX_MODE,
};
const XMC_GPIO_CONFIG_t CAN_TX_config =
{
    .mode = (XMC_GPIO_MODE_t)CAN_TX_MODE,
    .output_level = XMC_GPIO_OUTPUT_LEVEL_HIGH,
    .output_strength = XMC_GPIO_OUTPUT_STRENGTH_STRONG_SOFT_EDGE,
};
const XMC_GPIO_CONFIG_t CYBSP_DEBUG_UART_RX_config =
{
    .mode = (XMC_GPIO_MODE_t)CYBSP_DEBUG_UART_RX_MODE,
    .output_level = XMC_GPIO_OUTPUT_LEVEL_LOW,
};
const XMC_GPIO_CONFIG_t N_FAULT_HW_config =
{
    .mode = (XMC_GPIO_MODE_t)N_FAULT_HW_MODE,
};
const XMC_GPIO_CONFIG_t OCD1_config =
{
    .mode = (XMC_GPIO_MODE_t)OCD1_MODE,
};
const XMC_GPIO_CONFIG_t CYBSP_DEBUG_UART_TX_config =
{
    .mode = (XMC_GPIO_MODE_t)CYBSP_DEBUG_UART_TX_MODE,
    .output_level = XMC_GPIO_OUTPUT_LEVEL_LOW,
};

void init_cycfg_pins(void)
{
    XMC_GPIO_Init(PWMUL_PORT, PWMUL_PIN, &PWMUL_config);
    XMC_GPIO_SetHardwareControl(PWMUL_PORT, PWMUL_PIN, PWMUL_HWO);
    XMC_GPIO_Init(PWMVL_PORT, PWMVL_PIN, &PWMVL_config);
    XMC_GPIO_SetHardwareControl(PWMVL_PORT, PWMVL_PIN, PWMVL_HWO);
    XMC_GPIO_Init(PWMWL_PORT, PWMWL_PIN, &PWMWL_config);
    XMC_GPIO_SetHardwareControl(PWMWL_PORT, PWMWL_PIN, PWMWL_HWO);
    XMC_GPIO_Init(PWMUH_PORT, PWMUH_PIN, &PWMUH_config);
    XMC_GPIO_SetHardwareControl(PWMUH_PORT, PWMUH_PIN, PWMUH_HWO);
    XMC_GPIO_Init(PWMVH_PORT, PWMVH_PIN, &PWMVH_config);
    XMC_GPIO_SetHardwareControl(PWMVH_PORT, PWMVH_PIN, PWMVH_HWO);
    XMC_GPIO_Init(PWMWH_PORT, PWMWH_PIN, &PWMWH_config);
    XMC_GPIO_SetHardwareControl(PWMWH_PORT, PWMWH_PIN, PWMWH_HWO);
    XMC_GPIO_Init(PRE_CHARGE_PORT, PRE_CHARGE_PIN, &PRE_CHARGE_config);
    XMC_GPIO_SetHardwareControl(PRE_CHARGE_PORT, PRE_CHARGE_PIN, PRE_CHARGE_HWO);
    XMC_GPIO_Init(TEST_PIN0_PORT, TEST_PIN0_PIN, &TEST_PIN0_config);
    XMC_GPIO_SetHardwareControl(TEST_PIN0_PORT, TEST_PIN0_PIN, TEST_PIN0_HWO);
    XMC_GPIO_Init(TEST_PIN1_PORT, TEST_PIN1_PIN, &TEST_PIN1_config);
    XMC_GPIO_SetHardwareControl(TEST_PIN1_PORT, TEST_PIN1_PIN, TEST_PIN1_HWO);
    XMC_GPIO_Init(IUP_PORT, IUP_PIN, &IUP_config);
    XMC_GPIO_SetHardwareControl(IUP_PORT, IUP_PIN, IUP_HWO);
    XMC_GPIO_Init(IVP_PORT, IVP_PIN, &IVP_config);
    XMC_GPIO_SetHardwareControl(IVP_PORT, IVP_PIN, IVP_HWO);
    XMC_GPIO_Init(IWP_PORT, IWP_PIN, &IWP_config);
    XMC_GPIO_SetHardwareControl(IWP_PORT, IWP_PIN, IWP_HWO);
    XMC_GPIO_Init(T_POWER_PORT, T_POWER_PIN, &T_POWER_config);
    XMC_GPIO_SetHardwareControl(T_POWER_PORT, T_POWER_PIN, T_POWER_HWO);
    XMC_GPIO_Init(VBUS_PORT, VBUS_PIN, &VBUS_config);
    XMC_GPIO_SetHardwareControl(VBUS_PORT, VBUS_PIN, VBUS_HWO);
    XMC_GPIO_Init(TEST_PIN2_PORT, TEST_PIN2_PIN, &TEST_PIN2_config);
    XMC_GPIO_SetHardwareControl(TEST_PIN2_PORT, TEST_PIN2_PIN, TEST_PIN2_HWO);
    XMC_GPIO_Init(TEST_PIN3_PORT, TEST_PIN3_PIN, &TEST_PIN3_config);
    XMC_GPIO_SetHardwareControl(TEST_PIN3_PORT, TEST_PIN3_PIN, TEST_PIN3_HWO);
    XMC_GPIO_Init(N_HALL_EN_PORT, N_HALL_EN_PIN, &N_HALL_EN_config);
    XMC_GPIO_SetHardwareControl(N_HALL_EN_PORT, N_HALL_EN_PIN, N_HALL_EN_HWO);
    XMC_GPIO_Init(HALL_2_PORT, HALL_2_PIN, &HALL_2_config);
    XMC_GPIO_SetHardwareControl(HALL_2_PORT, HALL_2_PIN, HALL_2_HWO);
    XMC_GPIO_Init(HALL_1_PORT, HALL_1_PIN, &HALL_1_config);
    XMC_GPIO_SetHardwareControl(HALL_1_PORT, HALL_1_PIN, HALL_1_HWO);
    XMC_GPIO_Init(HALL_0_PORT, HALL_0_PIN, &HALL_0_config);
    XMC_GPIO_SetHardwareControl(HALL_0_PORT, HALL_0_PIN, HALL_0_HWO);
    XMC_GPIO_Init(SB_CAN_PORT, SB_CAN_PIN, &SB_CAN_config);
    XMC_GPIO_SetHardwareControl(SB_CAN_PORT, SB_CAN_PIN, SB_CAN_HWO);
    XMC_GPIO_Init(CAN_RX_PORT, CAN_RX_PIN, &CAN_RX_config);
    XMC_GPIO_SetHardwareControl(CAN_RX_PORT, CAN_RX_PIN, CAN_RX_HWO);
    XMC_GPIO_Init(CAN_TX_PORT, CAN_TX_PIN, &CAN_TX_config);
    XMC_GPIO_SetHardwareControl(CAN_TX_PORT, CAN_TX_PIN, CAN_TX_HWO);
    XMC_GPIO_Init(CYBSP_DEBUG_UART_RX_PORT, CYBSP_DEBUG_UART_RX_PIN, &CYBSP_DEBUG_UART_RX_config);
    XMC_GPIO_SetHardwareControl(CYBSP_DEBUG_UART_RX_PORT, CYBSP_DEBUG_UART_RX_PIN, CYBSP_DEBUG_UART_RX_HWO);
    XMC_GPIO_Init(N_FAULT_HW_PORT, N_FAULT_HW_PIN, &N_FAULT_HW_config);
    XMC_GPIO_SetHardwareControl(N_FAULT_HW_PORT, N_FAULT_HW_PIN, N_FAULT_HW_HWO);
    XMC_GPIO_Init(OCD1_PORT, OCD1_PIN, &OCD1_config);
    XMC_GPIO_SetHardwareControl(OCD1_PORT, OCD1_PIN, OCD1_HWO);
    XMC_GPIO_Init(CYBSP_DEBUG_UART_TX_PORT, CYBSP_DEBUG_UART_TX_PIN, &CYBSP_DEBUG_UART_TX_config);
    XMC_GPIO_SetHardwareControl(CYBSP_DEBUG_UART_TX_PORT, CYBSP_DEBUG_UART_TX_PIN, CYBSP_DEBUG_UART_TX_HWO);
}
