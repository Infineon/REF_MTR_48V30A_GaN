/*******************************************************************************
 * File Name: cycfg_routing.h
 *
 * Description:
 * Establishes all necessary connections between hardware elements.
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

#if !defined(CYCFG_ROUTING_H)
#define CYCFG_ROUTING_H

#include "cycfg_notices.h"

#if defined(__cplusplus)
extern "C" {
#endif /* defined(__cplusplus) */

#define ioss_0_port_0_pin_0_ALT XMC_GPIO_MODE_OUTPUT_ALT3
#define ioss_0_port_0_pin_1_ALT XMC_GPIO_MODE_OUTPUT_ALT3
#define ioss_0_port_0_pin_2_ALT XMC_GPIO_MODE_OUTPUT_ALT3
#define ioss_0_port_0_pin_3_ALT XMC_GPIO_MODE_OUTPUT_ALT3
#define ioss_0_port_0_pin_4_ALT XMC_GPIO_MODE_OUTPUT_ALT3
#define ioss_0_port_0_pin_5_ALT XMC_GPIO_MODE_OUTPUT_ALT3
#define ioss_0_port_1_pin_5_INPUT XMC_GPIO_MODE_INPUT_TRISTATE
#define ioss_0_port_2_pin_0_ALT XMC_GPIO_MODE_OUTPUT_ALT1
#define ioss_0_port_2_pin_2_INPUT XMC_GPIO_MODE_INPUT_TRISTATE
#define ioss_0_port_2_pin_5_ALT XMC_GPIO_MODE_OUTPUT_ALT2
#define ADC_0_sr3_0_TRIGGER_IN 2
#define DMA_ADC_0_src_per_0_TRIGGER_OUT 7
#define CAN_NODE0_RECEIVE_INPUT (XMC_CAN_NODE_RECEIVE_INPUT_RXDCA)
#define USIC0_CH1_DX0CR_DSEL_VALUE 0
#define XMC_GPDMA0_CH0_SRC_PER 7

static inline void init_cycfg_routing(void) {}

#if defined(__cplusplus)
}
#endif /* defined(__cplusplus) */

#endif /* CYCFG_ROUTING_H */
