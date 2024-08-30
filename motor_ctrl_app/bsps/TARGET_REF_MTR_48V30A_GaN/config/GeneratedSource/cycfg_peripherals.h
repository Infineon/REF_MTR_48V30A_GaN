/*******************************************************************************
 * File Name: cycfg_peripherals.h
 *
 * Description:
 * Peripheral Hardware Block configuration
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

#if !defined(CYCFG_PERIPHERALS_H)
#define CYCFG_PERIPHERALS_H

#include "cycfg_notices.h"
#include "xmc_can.h"
#include "cycfg_routing.h"
#include "xmc_ccu4.h"
#include "xmc_ccu8.h"
#include "xmc_uart.h"
#include "xmc_vadc.h"

#if defined(__cplusplus)
extern "C" {
#endif /* defined(__cplusplus) */

#define can_0_ENABLED 1U
#define can_0_HW ((CAN_GLOBAL_TypeDef*)CAN)
#define CAN_FREQUENCY_VALUE ((uint32_t)80000000)
#define CAN_CLOCK_SOURCE ((XMC_CAN_CANCLKSRC_t)XMC_CAN_CANCLKSRC_FPERI)
#define can_0_0_INTERRUPT_HANDLER CAN0_0_IRQHandler
#define can_0_1_INTERRUPT_HANDLER CAN0_1_IRQHandler
#define can_0_2_INTERRUPT_HANDLER CAN0_2_IRQHandler
#define can_0_3_INTERRUPT_HANDLER CAN0_3_IRQHandler
#define can_0_4_INTERRUPT_HANDLER CAN0_4_IRQHandler
#define can_0_5_INTERRUPT_HANDLER CAN0_5_IRQHandler
#define can_0_6_INTERRUPT_HANDLER CAN0_6_IRQHandler
#define can_0_7_INTERRUPT_HANDLER CAN0_7_IRQHandler
#define can_0_0_IRQN CAN0_0_IRQn
#define can_0_1_IRQN CAN0_1_IRQn
#define can_0_2_IRQN CAN0_2_IRQn
#define can_0_3_IRQN CAN0_3_IRQn
#define can_0_4_IRQN CAN0_4_IRQn
#define can_0_5_IRQN CAN0_5_IRQn
#define can_0_6_IRQN CAN0_6_IRQn
#define can_0_7_IRQN CAN0_7_IRQn
#define can_0_node_0_ENABLED 1U
#define can_0_node_0_HW CAN_NODE0
#define can_0_node_0_TIME_CONFIG can_0_node_0_bit_time_config
#define can_0_node_0_LMO0_CONFIG can_0_node_0_LMO_0
#define can_0_node_0_LMO0_EVENT_ENABLE_TX false
#define can_0_node_0_LMO0_EVENT_ENABLE_RX false
#define can_0_node_0_NODE_NUMBER 0U
#define can_0_node_0_MO_COUNT 1U
#define can_0_node_0_ENABLE_LOOPBACK false
#define can_0_node_0_ENABLE_EVENT_ALERT false
#define can_0_node_0_ENABLE_EVENT_LAST_ERR_CODE false
#define can_0_node_0_ENABLE_EVENT_TXOK false
#define can_0_node_0_ENABLE_EVENT_FRAME_COUNT false
#define SYNC_ISRn_ENABLED 1U
#define SYNC_ISRn_NUM 0U
#define SYNC_ISRn_HW CCU40
#define SYNC_ISRn_SR3_INTERRUPT_HANDLER CCU40_3_IRQHandler
#define SYNC_ISRn_SR3_IRQN CCU40_3_IRQn
#define SYNC_ISR1_ENABLED 1U
#define SYNC_ISR1_NUM 0U
#define SYNC_ISR1_HW CCU40_CC40
#define SYNC_ISR1_TICK_NS 25U
#define ccu4_1_ENABLED 1U
#define ccu4_1_NUM 1U
#define ccu4_1_HW CCU41
#define EXE_TIMER_L_ENABLED 1U
#define EXE_TIMER_L_NUM 0U
#define EXE_TIMER_L_HW CCU41_CC40
#define EXE_TIMER_L_TICK_NS 25U
#define EXE_TIMER_H_ENABLED 1U
#define EXE_TIMER_H_NUM 1U
#define EXE_TIMER_H_HW CCU41_CC41
#define EXE_TIMER_H_TICK_NS 25U
#define HALL_TIMER_L_ENABLED 1U
#define HALL_TIMER_L_NUM 2U
#define HALL_TIMER_L_HW CCU41_CC42
#define HALL_TIMER_L_TICK_NS 25U
#define HALL_TIMER_H_ENABLED 1U
#define HALL_TIMER_H_NUM 3U
#define HALL_TIMER_H_HW CCU41_CC43
#define HALL_TIMER_H_TICK_NS 25U
#define PWM_UVW_ADCn_ISRn_ENABLED 1U
#define PWM_UVW_ADCn_ISRn_NUM 0U
#define PWM_UVW_ADCn_ISRn_HW CCU80
#define PWM_UVW_ADCn_ISRn_SR2_INTERRUPT_HANDLER CCU80_2_IRQHandler
#define PWM_UVW_ADCn_ISRn_SR3_INTERRUPT_HANDLER CCU80_3_IRQHandler
#define PWM_UVW_ADCn_ISRn_SR2_IRQN CCU80_2_IRQn
#define PWM_UVW_ADCn_ISRn_SR3_IRQN CCU80_3_IRQn
#define PWM_W_ENABLED 1U
#define PWM_W_NUM 0U
#define PWM_W_HW CCU80_CC80
#define PWM_W_TICK_NS 13U
#define PWM_V_ENABLED 1U
#define PWM_V_NUM 1U
#define PWM_V_HW CCU80_CC81
#define PWM_V_TICK_NS 13U
#define PWM_U_ENABLED 1U
#define PWM_U_NUM 2U
#define PWM_U_HW CCU80_CC82
#define PWM_U_TICK_NS 13U
#define ADC0_ISR0_ENABLED 1U
#define ADC1_ISR0_ENABLED ADC0_ISR0_ENABLED
#define ADC0_ISR0_NUM 3U
#define ADC1_ISR0_NUM ADC0_ISR0_NUM
#define ADC0_ISR0_HW CCU80_CC83
#define ADC1_ISR0_HW ADC0_ISR0_HW
#define ADC0_ISR0_TICK_NS 13U
#define ADC1_ISR0_TICK_NS ADC0_ISR0_TICK_NS
#define CYBSP_DEBUG_UART_ENABLED 1U
#define CYBSP_DEBUG_UART_NUM 1U
#define CYBSP_DEBUG_UART_HW XMC_UART0_CH1
#define CYBSP_DEBUG_UART_DX0_INPUT USIC0_CH1_DX0CR_DSEL_VALUE
#define vadc_0_ENABLED 1U
#define vadc_0_HW VADC
#define vadc_0_SR0_INTERRUPT_HANDLER VADC0_C0_0_IRQHandler
#define vadc_0_SR1_INTERRUPT_HANDLER VADC0_C0_1_IRQHandler
#define vadc_0_SR2_INTERRUPT_HANDLER VADC0_C0_2_IRQHandler
#define vadc_0_SR3_INTERRUPT_HANDLER VADC0_C0_3_IRQHandler
#define vadc_0_SR0_IRQN VADC0_C0_0_IRQn
#define vadc_0_SR1_IRQN VADC0_C0_1_IRQn
#define vadc_0_SR2_IRQN VADC0_C0_2_IRQn
#define vadc_0_SR3_IRQN VADC0_C0_3_IRQn
#define ADC_0_ENABLED 1U
#define ADC_0_HW VADC_G0
#define ADC_0_NUM 0U
#define ADC_0_SR0_INTERRUPT_HANDLER VADC0_G0_0_IRQHandler
#define ADC_0_SR1_INTERRUPT_HANDLER VADC0_G0_1_IRQHandler
#define ADC_0_SR2_INTERRUPT_HANDLER VADC0_G0_2_IRQHandler
#define ADC_0_SR3_INTERRUPT_HANDLER VADC0_G0_3_IRQHandler
#define ADC_0_SR0_IRQN VADC0_G0_0_IRQn
#define ADC_0_SR1_IRQN VADC0_G0_1_IRQn
#define ADC_0_SR2_IRQN VADC0_G0_2_IRQn
#define ADC_0_SR3_IRQN VADC0_G0_3_IRQn
#define ADC0_CH_IV_ENABLED 1U
#define ADC0_CH_IW_ENABLED 1U
#define ADC0_CH_TEMP_ENABLED 1U
#define ADC0_CH_VBUS_ENABLED 1U
#define ADC_1_ENABLED 1U
#define ADC_1_HW VADC_G1
#define ADC_1_NUM 1U
#define ADC_1_SR0_INTERRUPT_HANDLER VADC0_G1_0_IRQHandler
#define ADC_1_SR1_INTERRUPT_HANDLER VADC0_G1_1_IRQHandler
#define ADC_1_SR2_INTERRUPT_HANDLER VADC0_G1_2_IRQHandler
#define ADC_1_SR3_INTERRUPT_HANDLER VADC0_G1_3_IRQHandler
#define ADC_1_SR0_IRQN VADC0_G1_0_IRQn
#define ADC_1_SR1_IRQN VADC0_G1_1_IRQn
#define ADC_1_SR2_IRQN VADC0_G1_2_IRQn
#define ADC_1_SR3_IRQN VADC0_G1_3_IRQn
#define ADC1_CH_IU_ENABLED 1U

extern const XMC_CAN_NODE_NOMINAL_BIT_TIME_CONFIG_t can_0_node_0_bit_time_config;
extern XMC_CAN_MO_t can_0_node_0_LMO_0;
extern const XMC_CCU4_SLICE_COMPARE_CONFIG_t SYNC_ISR1_compare_config;
extern const XMC_CCU4_SLICE_EVENT_CONFIG_t SYNC_ISR1_event0_config;
extern const XMC_CCU4_SLICE_EVENT_CONFIG_t SYNC_ISR1_event1_config;
extern const XMC_CCU4_SLICE_EVENT_CONFIG_t SYNC_ISR1_event2_config;
extern const XMC_CCU4_SLICE_COMPARE_CONFIG_t EXE_TIMER_L_compare_config;
extern const XMC_CCU4_SLICE_EVENT_CONFIG_t EXE_TIMER_L_event0_config;
extern const XMC_CCU4_SLICE_EVENT_CONFIG_t EXE_TIMER_L_event1_config;
extern const XMC_CCU4_SLICE_EVENT_CONFIG_t EXE_TIMER_L_event2_config;
extern const XMC_CCU4_SLICE_COMPARE_CONFIG_t EXE_TIMER_H_compare_config;
extern const XMC_CCU4_SLICE_EVENT_CONFIG_t EXE_TIMER_H_event0_config;
extern const XMC_CCU4_SLICE_EVENT_CONFIG_t EXE_TIMER_H_event1_config;
extern const XMC_CCU4_SLICE_EVENT_CONFIG_t EXE_TIMER_H_event2_config;
extern const XMC_CCU4_SLICE_COMPARE_CONFIG_t HALL_TIMER_L_compare_config;
extern const XMC_CCU4_SLICE_EVENT_CONFIG_t HALL_TIMER_L_event0_config;
extern const XMC_CCU4_SLICE_EVENT_CONFIG_t HALL_TIMER_L_event1_config;
extern const XMC_CCU4_SLICE_EVENT_CONFIG_t HALL_TIMER_L_event2_config;
extern const XMC_CCU4_SLICE_COMPARE_CONFIG_t HALL_TIMER_H_compare_config;
extern const XMC_CCU4_SLICE_EVENT_CONFIG_t HALL_TIMER_H_event0_config;
extern const XMC_CCU4_SLICE_EVENT_CONFIG_t HALL_TIMER_H_event1_config;
extern const XMC_CCU4_SLICE_EVENT_CONFIG_t HALL_TIMER_H_event2_config;
extern const XMC_CCU8_SLICE_COMPARE_CONFIG_t PWM_W_compare_config;
extern const XMC_CCU8_SLICE_DEAD_TIME_CONFIG_t PWM_W_dead_time_config;
extern const XMC_CCU8_SLICE_EVENT_CONFIG_t PWM_W_event0_config;
extern const XMC_CCU8_SLICE_EVENT_CONFIG_t PWM_W_event1_config;
extern const XMC_CCU8_SLICE_EVENT_CONFIG_t PWM_W_event2_config;
extern const XMC_CCU8_SLICE_COMPARE_CONFIG_t PWM_V_compare_config;
extern const XMC_CCU8_SLICE_DEAD_TIME_CONFIG_t PWM_V_dead_time_config;
extern const XMC_CCU8_SLICE_EVENT_CONFIG_t PWM_V_event0_config;
extern const XMC_CCU8_SLICE_EVENT_CONFIG_t PWM_V_event1_config;
extern const XMC_CCU8_SLICE_EVENT_CONFIG_t PWM_V_event2_config;
extern const XMC_CCU8_SLICE_COMPARE_CONFIG_t PWM_U_compare_config;
extern const XMC_CCU8_SLICE_DEAD_TIME_CONFIG_t PWM_U_dead_time_config;
extern const XMC_CCU8_SLICE_EVENT_CONFIG_t PWM_U_event0_config;
extern const XMC_CCU8_SLICE_EVENT_CONFIG_t PWM_U_event1_config;
extern const XMC_CCU8_SLICE_EVENT_CONFIG_t PWM_U_event2_config;
extern const XMC_CCU8_SLICE_COMPARE_CONFIG_t ADC0_ISR0_compare_config;

#define ADC1_ISR0_compare_config ADC0_ISR0_compare_config

extern const XMC_CCU8_SLICE_EVENT_CONFIG_t ADC0_ISR0_event0_config;

#define ADC1_ISR0_event0_config ADC0_ISR0_event0_config

extern const XMC_CCU8_SLICE_EVENT_CONFIG_t ADC0_ISR0_event1_config;

#define ADC1_ISR0_event1_config ADC0_ISR0_event1_config

extern const XMC_CCU8_SLICE_EVENT_CONFIG_t ADC0_ISR0_event2_config;

#define ADC1_ISR0_event2_config ADC0_ISR0_event2_config

extern const XMC_UART_CH_CONFIG_t CYBSP_DEBUG_UART_config;
extern XMC_VADC_GROUP_CONFIG_t vadc_0_group0_init_config;
extern XMC_VADC_GROUP_CONFIG_t vadc_0_group1_init_config;
extern const XMC_VADC_GLOBAL_CONFIG_t vadc_0_config;
extern const XMC_VADC_BACKGROUND_CONFIG_t vadc_0_background_scan_config;
extern const XMC_VADC_QUEUE_ENTRY_t ADC_0_queue_entries_4;
extern const XMC_VADC_QUEUE_ENTRY_t ADC_0_queue_entries_5;
extern const XMC_VADC_QUEUE_ENTRY_t ADC_0_queue_entries_6;
extern const XMC_VADC_QUEUE_ENTRY_t ADC_0_queue_entries_7;
extern const XMC_VADC_RESULT_CONFIG_t ADC_0_result_0_config;
extern const XMC_VADC_RESULT_CONFIG_t ADC_0_result_1_config;
extern const XMC_VADC_RESULT_CONFIG_t ADC_0_result_2_config;
extern const XMC_VADC_RESULT_CONFIG_t ADC_0_result_3_config;
extern const XMC_VADC_QUEUE_CONFIG_t ADC_0_queue_config;
extern const XMC_VADC_GROUP_CLASS_t vadc_0_0_iclass_0;
extern const XMC_VADC_GROUP_CLASS_t vadc_0_0_iclass_1;
extern const XMC_VADC_CHANNEL_CONFIG_t ADC0_CH_IV_config;
extern const XMC_VADC_CHANNEL_CONFIG_t ADC0_CH_IW_config;
extern const XMC_VADC_CHANNEL_CONFIG_t ADC0_CH_TEMP_config;
extern const XMC_VADC_CHANNEL_CONFIG_t ADC0_CH_VBUS_config;
extern const XMC_VADC_QUEUE_ENTRY_t ADC_1_queue_entries_7;
extern const XMC_VADC_RESULT_CONFIG_t ADC_1_result_0_config;
extern const XMC_VADC_QUEUE_CONFIG_t ADC_1_queue_config;
extern const XMC_VADC_GROUP_CLASS_t vadc_0_1_iclass_0;
extern const XMC_VADC_GROUP_CLASS_t vadc_0_1_iclass_1;
extern const XMC_VADC_CHANNEL_CONFIG_t ADC1_CH_IU_config;

void init_cycfg_peripherals(void);

#if defined(__cplusplus)
}
#endif /* defined(__cplusplus) */

#endif /* CYCFG_PERIPHERALS_H */
