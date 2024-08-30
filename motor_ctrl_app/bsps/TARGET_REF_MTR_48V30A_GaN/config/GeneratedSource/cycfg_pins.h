/*******************************************************************************
 * File Name: cycfg_pins.h
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

#if !defined(CYCFG_PINS_H)
#define CYCFG_PINS_H

#include "cycfg_notices.h"
#include "xmc_gpio.h"
#include "cycfg_routing.h"

#if defined(__cplusplus)
extern "C" {
#endif /* defined(__cplusplus) */

#define PWMUL_ENABLED 1U
#define PWMUL_PORT XMC_GPIO_PORT0
#define PWMUL_PORT_NUM 0U
#define PWMUL_PIN 0U
#ifndef ioss_0_port_0_pin_0_ALT
    #define ioss_0_port_0_pin_0_ALT 0U
#endif
#define PWMUL_MODE (XMC_GPIO_MODE_OUTPUT_PUSH_PULL | ioss_0_port_0_pin_0_ALT)
#ifndef ioss_0_port_0_pin_0_HWO
    #define ioss_0_port_0_pin_0_HWO XMC_GPIO_HWCTRL_DISABLED
#endif
#define PWMUL_HWO ioss_0_port_0_pin_0_HWO
#define PWMVL_ENABLED 1U
#define PWMVL_PORT XMC_GPIO_PORT0
#define PWMVL_PORT_NUM 0U
#define PWMVL_PIN 1U
#ifndef ioss_0_port_0_pin_1_ALT
    #define ioss_0_port_0_pin_1_ALT 0U
#endif
#define PWMVL_MODE (XMC_GPIO_MODE_OUTPUT_PUSH_PULL | ioss_0_port_0_pin_1_ALT)
#ifndef ioss_0_port_0_pin_1_HWO
    #define ioss_0_port_0_pin_1_HWO XMC_GPIO_HWCTRL_DISABLED
#endif
#define PWMVL_HWO ioss_0_port_0_pin_1_HWO
#define PWMWL_ENABLED 1U
#define PWMWL_PORT XMC_GPIO_PORT0
#define PWMWL_PORT_NUM 0U
#define PWMWL_PIN 2U
#ifndef ioss_0_port_0_pin_2_ALT
    #define ioss_0_port_0_pin_2_ALT 0U
#endif
#define PWMWL_MODE (XMC_GPIO_MODE_OUTPUT_PUSH_PULL | ioss_0_port_0_pin_2_ALT)
#ifndef ioss_0_port_0_pin_2_HWO
    #define ioss_0_port_0_pin_2_HWO XMC_GPIO_HWCTRL_DISABLED
#endif
#define PWMWL_HWO ioss_0_port_0_pin_2_HWO
#define PWMUH_ENABLED 1U
#define PWMUH_PORT XMC_GPIO_PORT0
#define PWMUH_PORT_NUM 0U
#define PWMUH_PIN 3U
#ifndef ioss_0_port_0_pin_3_ALT
    #define ioss_0_port_0_pin_3_ALT 0U
#endif
#define PWMUH_MODE (XMC_GPIO_MODE_OUTPUT_PUSH_PULL | ioss_0_port_0_pin_3_ALT)
#ifndef ioss_0_port_0_pin_3_HWO
    #define ioss_0_port_0_pin_3_HWO XMC_GPIO_HWCTRL_DISABLED
#endif
#define PWMUH_HWO ioss_0_port_0_pin_3_HWO
#define PWMVH_ENABLED 1U
#define PWMVH_PORT XMC_GPIO_PORT0
#define PWMVH_PORT_NUM 0U
#define PWMVH_PIN 4U
#ifndef ioss_0_port_0_pin_4_ALT
    #define ioss_0_port_0_pin_4_ALT 0U
#endif
#define PWMVH_MODE (XMC_GPIO_MODE_OUTPUT_PUSH_PULL | ioss_0_port_0_pin_4_ALT)
#ifndef ioss_0_port_0_pin_4_HWO
    #define ioss_0_port_0_pin_4_HWO XMC_GPIO_HWCTRL_DISABLED
#endif
#define PWMVH_HWO ioss_0_port_0_pin_4_HWO
#define PWMWH_ENABLED 1U
#define PWMWH_PORT XMC_GPIO_PORT0
#define PWMWH_PORT_NUM 0U
#define PWMWH_PIN 5U
#ifndef ioss_0_port_0_pin_5_ALT
    #define ioss_0_port_0_pin_5_ALT 0U
#endif
#define PWMWH_MODE (XMC_GPIO_MODE_OUTPUT_PUSH_PULL | ioss_0_port_0_pin_5_ALT)
#ifndef ioss_0_port_0_pin_5_HWO
    #define ioss_0_port_0_pin_5_HWO XMC_GPIO_HWCTRL_DISABLED
#endif
#define PWMWH_HWO ioss_0_port_0_pin_5_HWO
#define PRE_CHARGE_ENABLED 1U
#define PRE_CHARGE_PORT XMC_GPIO_PORT0
#define PRE_CHARGE_PORT_NUM 0U
#define PRE_CHARGE_PIN 6U
#ifndef ioss_0_port_0_pin_6_ALT
    #define ioss_0_port_0_pin_6_ALT 0U
#endif
#define PRE_CHARGE_MODE (XMC_GPIO_MODE_OUTPUT_PUSH_PULL | ioss_0_port_0_pin_6_ALT)
#ifndef ioss_0_port_0_pin_6_HWO
    #define ioss_0_port_0_pin_6_HWO XMC_GPIO_HWCTRL_DISABLED
#endif
#define PRE_CHARGE_HWO ioss_0_port_0_pin_6_HWO
#define TEST_PIN0_ENABLED 1U
#define TEST_PIN0_PORT XMC_GPIO_PORT0
#define TEST_PIN0_PORT_NUM 0U
#define TEST_PIN0_PIN 8U
#ifndef ioss_0_port_0_pin_8_ALT
    #define ioss_0_port_0_pin_8_ALT 0U
#endif
#define TEST_PIN0_MODE (XMC_GPIO_MODE_OUTPUT_PUSH_PULL | ioss_0_port_0_pin_8_ALT)
#ifndef ioss_0_port_0_pin_8_HWO
    #define ioss_0_port_0_pin_8_HWO XMC_GPIO_HWCTRL_DISABLED
#endif
#define TEST_PIN0_HWO ioss_0_port_0_pin_8_HWO
#define TEST_PIN1_ENABLED 1U
#define TEST_PIN1_PORT XMC_GPIO_PORT14
#define TEST_PIN1_PORT_NUM 14U
#define TEST_PIN1_PIN 0U
#ifndef ioss_0_port_14_pin_0_ALT
    #define ioss_0_port_14_pin_0_ALT 0U
#endif
#define TEST_PIN1_MODE (XMC_GPIO_MODE_OUTPUT_PUSH_PULL | ioss_0_port_14_pin_0_ALT)
#ifndef ioss_0_port_14_pin_0_HWO
    #define ioss_0_port_14_pin_0_HWO XMC_GPIO_HWCTRL_DISABLED
#endif
#define TEST_PIN1_HWO ioss_0_port_14_pin_0_HWO
#define IUP_ENABLED 1U
#define IUP_PORT XMC_GPIO_PORT14
#define IUP_PORT_NUM 14U
#define IUP_PIN 3U
#ifndef ioss_0_port_14_pin_3_INPUT
    #define ioss_0_port_14_pin_3_INPUT 0U
#endif
#define IUP_MODE (XMC_GPIO_MODE_INPUT_TRISTATE | ioss_0_port_14_pin_3_INPUT)
#ifndef ioss_0_port_14_pin_3_HWO
    #define ioss_0_port_14_pin_3_HWO XMC_GPIO_HWCTRL_DISABLED
#endif
#define IUP_HWO ioss_0_port_14_pin_3_HWO
#define IVP_ENABLED 1U
#define IVP_PORT XMC_GPIO_PORT14
#define IVP_PORT_NUM 14U
#define IVP_PIN 4U
#ifndef ioss_0_port_14_pin_4_INPUT
    #define ioss_0_port_14_pin_4_INPUT 0U
#endif
#define IVP_MODE (XMC_GPIO_MODE_INPUT_TRISTATE | ioss_0_port_14_pin_4_INPUT)
#ifndef ioss_0_port_14_pin_4_HWO
    #define ioss_0_port_14_pin_4_HWO XMC_GPIO_HWCTRL_DISABLED
#endif
#define IVP_HWO ioss_0_port_14_pin_4_HWO
#define IWP_ENABLED 1U
#define IWP_PORT XMC_GPIO_PORT14
#define IWP_PORT_NUM 14U
#define IWP_PIN 5U
#ifndef ioss_0_port_14_pin_5_INPUT
    #define ioss_0_port_14_pin_5_INPUT 0U
#endif
#define IWP_MODE (XMC_GPIO_MODE_INPUT_TRISTATE | ioss_0_port_14_pin_5_INPUT)
#ifndef ioss_0_port_14_pin_5_HWO
    #define ioss_0_port_14_pin_5_HWO XMC_GPIO_HWCTRL_DISABLED
#endif
#define IWP_HWO ioss_0_port_14_pin_5_HWO
#define T_POWER_ENABLED 1U
#define T_POWER_PORT XMC_GPIO_PORT14
#define T_POWER_PORT_NUM 14U
#define T_POWER_PIN 6U
#ifndef ioss_0_port_14_pin_6_INPUT
    #define ioss_0_port_14_pin_6_INPUT 0U
#endif
#define T_POWER_MODE (XMC_GPIO_MODE_INPUT_TRISTATE | ioss_0_port_14_pin_6_INPUT)
#ifndef ioss_0_port_14_pin_6_HWO
    #define ioss_0_port_14_pin_6_HWO XMC_GPIO_HWCTRL_DISABLED
#endif
#define T_POWER_HWO ioss_0_port_14_pin_6_HWO
#define VBUS_ENABLED 1U
#define VBUS_PORT XMC_GPIO_PORT14
#define VBUS_PORT_NUM 14U
#define VBUS_PIN 7U
#ifndef ioss_0_port_14_pin_7_INPUT
    #define ioss_0_port_14_pin_7_INPUT 0U
#endif
#define VBUS_MODE (XMC_GPIO_MODE_INPUT_TRISTATE | ioss_0_port_14_pin_7_INPUT)
#ifndef ioss_0_port_14_pin_7_HWO
    #define ioss_0_port_14_pin_7_HWO XMC_GPIO_HWCTRL_DISABLED
#endif
#define VBUS_HWO ioss_0_port_14_pin_7_HWO
#define TEST_PIN2_ENABLED 1U
#define TEST_PIN2_PORT XMC_GPIO_PORT14
#define TEST_PIN2_PORT_NUM 14U
#define TEST_PIN2_PIN 8U
#ifndef ioss_0_port_14_pin_8_ALT
    #define ioss_0_port_14_pin_8_ALT 0U
#endif
#define TEST_PIN2_MODE (XMC_GPIO_MODE_OUTPUT_PUSH_PULL | ioss_0_port_14_pin_8_ALT)
#ifndef ioss_0_port_14_pin_8_HWO
    #define ioss_0_port_14_pin_8_HWO XMC_GPIO_HWCTRL_DISABLED
#endif
#define TEST_PIN2_HWO ioss_0_port_14_pin_8_HWO
#define TEST_PIN3_ENABLED 1U
#define TEST_PIN3_PORT XMC_GPIO_PORT14
#define TEST_PIN3_PORT_NUM 14U
#define TEST_PIN3_PIN 9U
#ifndef ioss_0_port_14_pin_9_ALT
    #define ioss_0_port_14_pin_9_ALT 0U
#endif
#define TEST_PIN3_MODE (XMC_GPIO_MODE_OUTPUT_PUSH_PULL | ioss_0_port_14_pin_9_ALT)
#ifndef ioss_0_port_14_pin_9_HWO
    #define ioss_0_port_14_pin_9_HWO XMC_GPIO_HWCTRL_DISABLED
#endif
#define TEST_PIN3_HWO ioss_0_port_14_pin_9_HWO
#define N_HALL_EN_ENABLED 1U
#define ENC_EN_ENABLED N_HALL_EN_ENABLED
#define N_HALL_EN_PORT XMC_GPIO_PORT1
#define ENC_EN_PORT N_HALL_EN_PORT
#define N_HALL_EN_PORT_NUM 1U
#define ENC_EN_PORT_NUM N_HALL_EN_PORT_NUM
#define N_HALL_EN_PIN 0U
#define ENC_EN_PIN N_HALL_EN_PIN
#ifndef ioss_0_port_1_pin_0_ALT
    #define ioss_0_port_1_pin_0_ALT 0U
#endif
#define N_HALL_EN_MODE (XMC_GPIO_MODE_OUTPUT_PUSH_PULL | ioss_0_port_1_pin_0_ALT)
#define ENC_EN_MODE N_HALL_EN_MODE
#ifndef ioss_0_port_1_pin_0_HWO
    #define ioss_0_port_1_pin_0_HWO XMC_GPIO_HWCTRL_DISABLED
#endif
#define N_HALL_EN_HWO ioss_0_port_1_pin_0_HWO
#define ENC_EN_HWO N_HALL_EN_HWO
#define HALL_2_ENABLED 1U
#define ENC_Z_ENABLED HALL_2_ENABLED
#define HALL_2_PORT XMC_GPIO_PORT1
#define ENC_Z_PORT HALL_2_PORT
#define HALL_2_PORT_NUM 1U
#define ENC_Z_PORT_NUM HALL_2_PORT_NUM
#define HALL_2_PIN 1U
#define ENC_Z_PIN HALL_2_PIN
#ifndef ioss_0_port_1_pin_1_INPUT
    #define ioss_0_port_1_pin_1_INPUT 0U
#endif
#define HALL_2_MODE (XMC_GPIO_MODE_INPUT_TRISTATE | ioss_0_port_1_pin_1_INPUT)
#define ENC_Z_MODE HALL_2_MODE
#ifndef ioss_0_port_1_pin_1_HWO
    #define ioss_0_port_1_pin_1_HWO XMC_GPIO_HWCTRL_DISABLED
#endif
#define HALL_2_HWO ioss_0_port_1_pin_1_HWO
#define ENC_Z_HWO HALL_2_HWO
#define HALL_1_ENABLED 1U
#define ENC_B_ENABLED HALL_1_ENABLED
#define HALL_1_PORT XMC_GPIO_PORT1
#define ENC_B_PORT HALL_1_PORT
#define HALL_1_PORT_NUM 1U
#define ENC_B_PORT_NUM HALL_1_PORT_NUM
#define HALL_1_PIN 2U
#define ENC_B_PIN HALL_1_PIN
#ifndef ioss_0_port_1_pin_2_INPUT
    #define ioss_0_port_1_pin_2_INPUT 0U
#endif
#define HALL_1_MODE (XMC_GPIO_MODE_INPUT_TRISTATE | ioss_0_port_1_pin_2_INPUT)
#define ENC_B_MODE HALL_1_MODE
#ifndef ioss_0_port_1_pin_2_HWO
    #define ioss_0_port_1_pin_2_HWO XMC_GPIO_HWCTRL_DISABLED
#endif
#define HALL_1_HWO ioss_0_port_1_pin_2_HWO
#define ENC_B_HWO HALL_1_HWO
#define HALL_0_ENABLED 1U
#define ENC_A_ENABLED HALL_0_ENABLED
#define HALL_0_PORT XMC_GPIO_PORT1
#define ENC_A_PORT HALL_0_PORT
#define HALL_0_PORT_NUM 1U
#define ENC_A_PORT_NUM HALL_0_PORT_NUM
#define HALL_0_PIN 3U
#define ENC_A_PIN HALL_0_PIN
#ifndef ioss_0_port_1_pin_3_INPUT
    #define ioss_0_port_1_pin_3_INPUT 0U
#endif
#define HALL_0_MODE (XMC_GPIO_MODE_INPUT_TRISTATE | ioss_0_port_1_pin_3_INPUT)
#define ENC_A_MODE HALL_0_MODE
#ifndef ioss_0_port_1_pin_3_HWO
    #define ioss_0_port_1_pin_3_HWO XMC_GPIO_HWCTRL_DISABLED
#endif
#define HALL_0_HWO ioss_0_port_1_pin_3_HWO
#define ENC_A_HWO HALL_0_HWO
#define SB_CAN_ENABLED 1U
#define SB_CAN_PORT XMC_GPIO_PORT1
#define SB_CAN_PORT_NUM 1U
#define SB_CAN_PIN 4U
#ifndef ioss_0_port_1_pin_4_ALT
    #define ioss_0_port_1_pin_4_ALT 0U
#endif
#define SB_CAN_MODE (XMC_GPIO_MODE_OUTPUT_PUSH_PULL | ioss_0_port_1_pin_4_ALT)
#ifndef ioss_0_port_1_pin_4_HWO
    #define ioss_0_port_1_pin_4_HWO XMC_GPIO_HWCTRL_DISABLED
#endif
#define SB_CAN_HWO ioss_0_port_1_pin_4_HWO
#define CAN_RX_ENABLED 1U
#define CAN_RX_PORT XMC_GPIO_PORT1
#define CAN_RX_PORT_NUM 1U
#define CAN_RX_PIN 5U
#ifndef ioss_0_port_1_pin_5_INPUT
    #define ioss_0_port_1_pin_5_INPUT 0U
#endif
#define CAN_RX_MODE (XMC_GPIO_MODE_INPUT_TRISTATE | ioss_0_port_1_pin_5_INPUT)
#ifndef ioss_0_port_1_pin_5_HWO
    #define ioss_0_port_1_pin_5_HWO XMC_GPIO_HWCTRL_DISABLED
#endif
#define CAN_RX_HWO ioss_0_port_1_pin_5_HWO
#define CAN_TX_ENABLED 1U
#define CAN_TX_PORT XMC_GPIO_PORT2
#define CAN_TX_PORT_NUM 2U
#define CAN_TX_PIN 0U
#ifndef ioss_0_port_2_pin_0_ALT
    #define ioss_0_port_2_pin_0_ALT 0U
#endif
#define CAN_TX_MODE (XMC_GPIO_MODE_OUTPUT_PUSH_PULL | ioss_0_port_2_pin_0_ALT)
#ifndef ioss_0_port_2_pin_0_HWO
    #define ioss_0_port_2_pin_0_HWO XMC_GPIO_HWCTRL_DISABLED
#endif
#define CAN_TX_HWO ioss_0_port_2_pin_0_HWO
#define CYBSP_DEBUG_UART_RX_ENABLED 1U
#define CYBSP_DEBUG_UART_RX_PORT XMC_GPIO_PORT2
#define CYBSP_DEBUG_UART_RX_PORT_NUM 2U
#define CYBSP_DEBUG_UART_RX_PIN 2U
#ifndef ioss_0_port_2_pin_2_ALT
    #define ioss_0_port_2_pin_2_ALT 0U
#endif
#define CYBSP_DEBUG_UART_RX_MODE (XMC_GPIO_MODE_OUTPUT_PUSH_PULL | ioss_0_port_2_pin_2_ALT)
#ifndef ioss_0_port_2_pin_2_HWO
    #define ioss_0_port_2_pin_2_HWO XMC_GPIO_HWCTRL_DISABLED
#endif
#define CYBSP_DEBUG_UART_RX_HWO ioss_0_port_2_pin_2_HWO
#define N_FAULT_HW_ENABLED 1U
#define N_FAULT_HW_PORT XMC_GPIO_PORT2
#define N_FAULT_HW_PORT_NUM 2U
#define N_FAULT_HW_PIN 3U
#ifndef ioss_0_port_2_pin_3_INPUT
    #define ioss_0_port_2_pin_3_INPUT 0U
#endif
#define N_FAULT_HW_MODE (XMC_GPIO_MODE_INPUT_PULL_UP | ioss_0_port_2_pin_3_INPUT)
#ifndef ioss_0_port_2_pin_3_HWO
    #define ioss_0_port_2_pin_3_HWO XMC_GPIO_HWCTRL_DISABLED
#endif
#define N_FAULT_HW_HWO ioss_0_port_2_pin_3_HWO
#define OCD1_ENABLED 1U
#define OCD1_PORT XMC_GPIO_PORT2
#define OCD1_PORT_NUM 2U
#define OCD1_PIN 4U
#ifndef ioss_0_port_2_pin_4_INPUT
    #define ioss_0_port_2_pin_4_INPUT 0U
#endif
#define OCD1_MODE (XMC_GPIO_MODE_INPUT_TRISTATE | ioss_0_port_2_pin_4_INPUT)
#ifndef ioss_0_port_2_pin_4_HWO
    #define ioss_0_port_2_pin_4_HWO XMC_GPIO_HWCTRL_DISABLED
#endif
#define OCD1_HWO ioss_0_port_2_pin_4_HWO
#define CYBSP_DEBUG_UART_TX_ENABLED 1U
#define CYBSP_DEBUG_UART_TX_PORT XMC_GPIO_PORT2
#define CYBSP_DEBUG_UART_TX_PORT_NUM 2U
#define CYBSP_DEBUG_UART_TX_PIN 5U
#ifndef ioss_0_port_2_pin_5_ALT
    #define ioss_0_port_2_pin_5_ALT 0U
#endif
#define CYBSP_DEBUG_UART_TX_MODE (XMC_GPIO_MODE_OUTPUT_PUSH_PULL | ioss_0_port_2_pin_5_ALT)
#ifndef ioss_0_port_2_pin_5_HWO
    #define ioss_0_port_2_pin_5_HWO XMC_GPIO_HWCTRL_DISABLED
#endif
#define CYBSP_DEBUG_UART_TX_HWO ioss_0_port_2_pin_5_HWO

extern const XMC_GPIO_CONFIG_t PWMUL_config;
extern const XMC_GPIO_CONFIG_t PWMVL_config;
extern const XMC_GPIO_CONFIG_t PWMWL_config;
extern const XMC_GPIO_CONFIG_t PWMUH_config;
extern const XMC_GPIO_CONFIG_t PWMVH_config;
extern const XMC_GPIO_CONFIG_t PWMWH_config;
extern const XMC_GPIO_CONFIG_t PRE_CHARGE_config;
extern const XMC_GPIO_CONFIG_t TEST_PIN0_config;
extern const XMC_GPIO_CONFIG_t TEST_PIN1_config;
extern const XMC_GPIO_CONFIG_t IUP_config;
extern const XMC_GPIO_CONFIG_t IVP_config;
extern const XMC_GPIO_CONFIG_t IWP_config;
extern const XMC_GPIO_CONFIG_t T_POWER_config;
extern const XMC_GPIO_CONFIG_t VBUS_config;
extern const XMC_GPIO_CONFIG_t TEST_PIN2_config;
extern const XMC_GPIO_CONFIG_t TEST_PIN3_config;
extern const XMC_GPIO_CONFIG_t N_HALL_EN_config;

#define ENC_EN_config N_HALL_EN_config

extern const XMC_GPIO_CONFIG_t HALL_2_config;

#define ENC_Z_config HALL_2_config

extern const XMC_GPIO_CONFIG_t HALL_1_config;

#define ENC_B_config HALL_1_config

extern const XMC_GPIO_CONFIG_t HALL_0_config;

#define ENC_A_config HALL_0_config

extern const XMC_GPIO_CONFIG_t SB_CAN_config;
extern const XMC_GPIO_CONFIG_t CAN_RX_config;
extern const XMC_GPIO_CONFIG_t CAN_TX_config;
extern const XMC_GPIO_CONFIG_t CYBSP_DEBUG_UART_RX_config;
extern const XMC_GPIO_CONFIG_t N_FAULT_HW_config;
extern const XMC_GPIO_CONFIG_t OCD1_config;
extern const XMC_GPIO_CONFIG_t CYBSP_DEBUG_UART_TX_config;

void init_cycfg_pins(void);

#if defined(__cplusplus)
}
#endif /* defined(__cplusplus) */

#endif /* CYCFG_PINS_H */
