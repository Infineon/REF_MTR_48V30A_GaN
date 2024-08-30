/*******************************************************************************
 * File Name: cycfg_peripherals.c
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

#include "cycfg_peripherals.h"

const XMC_CAN_NODE_NOMINAL_BIT_TIME_CONFIG_t can_0_node_0_bit_time_config =
{
    .can_frequency = CAN_FREQUENCY_VALUE,
    .baudrate = (uint32_t)(1000 * 1000),
    .sample_point = (uint16_t)(80 * 100),
    .sjw = (uint16_t)1,
};
XMC_CAN_MO_t can_0_node_0_LMO_0 = 
{
    .can_mo_type = XMC_CAN_MO_TYPE_TRANSMSGOBJ,
    .can_id_mode = XMC_CAN_FRAME_TYPE_STANDARD_11BITS,
    .can_priority = XMC_CAN_ARBITRATION_MODE_IDE_DIR_BASED_PRIO_2,
    .can_identifier = 0x7FFU,
    .can_id_mask = 0x7FFU,
    .can_ide_mask = 1U,
    .can_mo_ptr = (CAN_MO_TypeDef*)CAN_MO0,
    .can_data_length = 8U,
    .can_data[1] = 0x0U,
    .can_data[0] = 0x0U,
};
const XMC_CCU4_SLICE_COMPARE_CONFIG_t SYNC_ISR1_compare_config =
{
    .timer_mode = XMC_CCU4_SLICE_TIMER_COUNT_MODE_EA,
    .monoshot = XMC_CCU4_SLICE_TIMER_REPEAT_MODE_REPEAT,
    .shadow_xfer_clear = false,
    .dither_timer_period = false,
    .dither_duty_cycle = false,
    .prescaler_mode = XMC_CCU4_SLICE_PRESCALER_MODE_NORMAL,
    .mcm_enable = false,
    .prescaler_initval = XMC_CCU4_SLICE_PRESCALER_2,
    .float_limit = XMC_CCU4_SLICE_PRESCALER_32768,
    .dither_limit = 0U,
    .passive_level = XMC_CCU4_SLICE_OUTPUT_PASSIVE_LEVEL_LOW,
    .timer_concatenation = false,
};
const XMC_CCU4_SLICE_EVENT_CONFIG_t SYNC_ISR1_event0_config =
{
    .mapped_input = CCU40_IN0_SCU_GSC40,
    .edge = XMC_CCU4_SLICE_EVENT_EDGE_SENSITIVITY_RISING_EDGE,
    .level = XMC_CCU4_SLICE_EVENT_LEVEL_SENSITIVITY_ACTIVE_HIGH,
    .duration = XMC_CCU4_SLICE_EVENT_FILTER_DISABLED,
};
const XMC_CCU4_SLICE_EVENT_CONFIG_t SYNC_ISR1_event1_config =
{
    .mapped_input = CCU40_IN0_SCU_GSC40,
    .edge = XMC_CCU4_SLICE_EVENT_EDGE_SENSITIVITY_FALLING_EDGE,
    .level = XMC_CCU4_SLICE_EVENT_LEVEL_SENSITIVITY_ACTIVE_LOW,
    .duration = XMC_CCU4_SLICE_EVENT_FILTER_DISABLED,
};
const XMC_CCU4_SLICE_EVENT_CONFIG_t SYNC_ISR1_event2_config =
{
    .edge = XMC_CCU4_SLICE_EVENT_EDGE_SENSITIVITY_NONE,
    .level = XMC_CCU4_SLICE_EVENT_LEVEL_SENSITIVITY_ACTIVE_HIGH,
    .duration = XMC_CCU4_SLICE_EVENT_FILTER_DISABLED,
};
const XMC_CCU4_SLICE_COMPARE_CONFIG_t EXE_TIMER_L_compare_config =
{
    .timer_mode = XMC_CCU4_SLICE_TIMER_COUNT_MODE_EA,
    .monoshot = XMC_CCU4_SLICE_TIMER_REPEAT_MODE_REPEAT,
    .shadow_xfer_clear = false,
    .dither_timer_period = false,
    .dither_duty_cycle = false,
    .prescaler_mode = XMC_CCU4_SLICE_PRESCALER_MODE_NORMAL,
    .mcm_enable = false,
    .prescaler_initval = XMC_CCU4_SLICE_PRESCALER_2,
    .float_limit = XMC_CCU4_SLICE_PRESCALER_32768,
    .dither_limit = 0U,
    .passive_level = XMC_CCU4_SLICE_OUTPUT_PASSIVE_LEVEL_LOW,
    .timer_concatenation = false,
};
const XMC_CCU4_SLICE_EVENT_CONFIG_t EXE_TIMER_L_event0_config =
{
    .edge = XMC_CCU4_SLICE_EVENT_EDGE_SENSITIVITY_NONE,
    .level = XMC_CCU4_SLICE_EVENT_LEVEL_SENSITIVITY_ACTIVE_HIGH,
    .duration = XMC_CCU4_SLICE_EVENT_FILTER_DISABLED,
};
const XMC_CCU4_SLICE_EVENT_CONFIG_t EXE_TIMER_L_event1_config =
{
    .edge = XMC_CCU4_SLICE_EVENT_EDGE_SENSITIVITY_NONE,
    .level = XMC_CCU4_SLICE_EVENT_LEVEL_SENSITIVITY_ACTIVE_HIGH,
    .duration = XMC_CCU4_SLICE_EVENT_FILTER_DISABLED,
};
const XMC_CCU4_SLICE_EVENT_CONFIG_t EXE_TIMER_L_event2_config =
{
    .edge = XMC_CCU4_SLICE_EVENT_EDGE_SENSITIVITY_NONE,
    .level = XMC_CCU4_SLICE_EVENT_LEVEL_SENSITIVITY_ACTIVE_HIGH,
    .duration = XMC_CCU4_SLICE_EVENT_FILTER_DISABLED,
};
const XMC_CCU4_SLICE_COMPARE_CONFIG_t EXE_TIMER_H_compare_config =
{
    .timer_mode = XMC_CCU4_SLICE_TIMER_COUNT_MODE_EA,
    .monoshot = XMC_CCU4_SLICE_TIMER_REPEAT_MODE_REPEAT,
    .shadow_xfer_clear = false,
    .dither_timer_period = false,
    .dither_duty_cycle = false,
    .prescaler_mode = XMC_CCU4_SLICE_PRESCALER_MODE_NORMAL,
    .mcm_enable = false,
    .prescaler_initval = XMC_CCU4_SLICE_PRESCALER_2,
    .float_limit = XMC_CCU4_SLICE_PRESCALER_32768,
    .dither_limit = 0U,
    .passive_level = XMC_CCU4_SLICE_OUTPUT_PASSIVE_LEVEL_LOW,
    .timer_concatenation = true,
};
const XMC_CCU4_SLICE_EVENT_CONFIG_t EXE_TIMER_H_event0_config =
{
    .edge = XMC_CCU4_SLICE_EVENT_EDGE_SENSITIVITY_NONE,
    .level = XMC_CCU4_SLICE_EVENT_LEVEL_SENSITIVITY_ACTIVE_HIGH,
    .duration = XMC_CCU4_SLICE_EVENT_FILTER_DISABLED,
};
const XMC_CCU4_SLICE_EVENT_CONFIG_t EXE_TIMER_H_event1_config =
{
    .edge = XMC_CCU4_SLICE_EVENT_EDGE_SENSITIVITY_NONE,
    .level = XMC_CCU4_SLICE_EVENT_LEVEL_SENSITIVITY_ACTIVE_HIGH,
    .duration = XMC_CCU4_SLICE_EVENT_FILTER_DISABLED,
};
const XMC_CCU4_SLICE_EVENT_CONFIG_t EXE_TIMER_H_event2_config =
{
    .edge = XMC_CCU4_SLICE_EVENT_EDGE_SENSITIVITY_NONE,
    .level = XMC_CCU4_SLICE_EVENT_LEVEL_SENSITIVITY_ACTIVE_HIGH,
    .duration = XMC_CCU4_SLICE_EVENT_FILTER_DISABLED,
};
const XMC_CCU4_SLICE_COMPARE_CONFIG_t HALL_TIMER_L_compare_config =
{
    .timer_mode = XMC_CCU4_SLICE_TIMER_COUNT_MODE_EA,
    .monoshot = XMC_CCU4_SLICE_TIMER_REPEAT_MODE_REPEAT,
    .shadow_xfer_clear = false,
    .dither_timer_period = false,
    .dither_duty_cycle = false,
    .prescaler_mode = XMC_CCU4_SLICE_PRESCALER_MODE_NORMAL,
    .mcm_enable = false,
    .prescaler_initval = XMC_CCU4_SLICE_PRESCALER_2,
    .float_limit = XMC_CCU4_SLICE_PRESCALER_32768,
    .dither_limit = 0U,
    .passive_level = XMC_CCU4_SLICE_OUTPUT_PASSIVE_LEVEL_LOW,
    .timer_concatenation = false,
};
const XMC_CCU4_SLICE_EVENT_CONFIG_t HALL_TIMER_L_event0_config =
{
    .edge = XMC_CCU4_SLICE_EVENT_EDGE_SENSITIVITY_NONE,
    .level = XMC_CCU4_SLICE_EVENT_LEVEL_SENSITIVITY_ACTIVE_HIGH,
    .duration = XMC_CCU4_SLICE_EVENT_FILTER_DISABLED,
};
const XMC_CCU4_SLICE_EVENT_CONFIG_t HALL_TIMER_L_event1_config =
{
    .edge = XMC_CCU4_SLICE_EVENT_EDGE_SENSITIVITY_NONE,
    .level = XMC_CCU4_SLICE_EVENT_LEVEL_SENSITIVITY_ACTIVE_HIGH,
    .duration = XMC_CCU4_SLICE_EVENT_FILTER_DISABLED,
};
const XMC_CCU4_SLICE_EVENT_CONFIG_t HALL_TIMER_L_event2_config =
{
    .edge = XMC_CCU4_SLICE_EVENT_EDGE_SENSITIVITY_NONE,
    .level = XMC_CCU4_SLICE_EVENT_LEVEL_SENSITIVITY_ACTIVE_HIGH,
    .duration = XMC_CCU4_SLICE_EVENT_FILTER_DISABLED,
};
const XMC_CCU4_SLICE_COMPARE_CONFIG_t HALL_TIMER_H_compare_config =
{
    .timer_mode = XMC_CCU4_SLICE_TIMER_COUNT_MODE_EA,
    .monoshot = XMC_CCU4_SLICE_TIMER_REPEAT_MODE_REPEAT,
    .shadow_xfer_clear = false,
    .dither_timer_period = false,
    .dither_duty_cycle = false,
    .prescaler_mode = XMC_CCU4_SLICE_PRESCALER_MODE_NORMAL,
    .mcm_enable = false,
    .prescaler_initval = XMC_CCU4_SLICE_PRESCALER_2,
    .float_limit = XMC_CCU4_SLICE_PRESCALER_32768,
    .dither_limit = 0U,
    .passive_level = XMC_CCU4_SLICE_OUTPUT_PASSIVE_LEVEL_LOW,
    .timer_concatenation = true,
};
const XMC_CCU4_SLICE_EVENT_CONFIG_t HALL_TIMER_H_event0_config =
{
    .edge = XMC_CCU4_SLICE_EVENT_EDGE_SENSITIVITY_NONE,
    .level = XMC_CCU4_SLICE_EVENT_LEVEL_SENSITIVITY_ACTIVE_HIGH,
    .duration = XMC_CCU4_SLICE_EVENT_FILTER_DISABLED,
};
const XMC_CCU4_SLICE_EVENT_CONFIG_t HALL_TIMER_H_event1_config =
{
    .edge = XMC_CCU4_SLICE_EVENT_EDGE_SENSITIVITY_NONE,
    .level = XMC_CCU4_SLICE_EVENT_LEVEL_SENSITIVITY_ACTIVE_HIGH,
    .duration = XMC_CCU4_SLICE_EVENT_FILTER_DISABLED,
};
const XMC_CCU4_SLICE_EVENT_CONFIG_t HALL_TIMER_H_event2_config =
{
    .edge = XMC_CCU4_SLICE_EVENT_EDGE_SENSITIVITY_NONE,
    .level = XMC_CCU4_SLICE_EVENT_LEVEL_SENSITIVITY_ACTIVE_HIGH,
    .duration = XMC_CCU4_SLICE_EVENT_FILTER_DISABLED,
};
const XMC_CCU8_SLICE_COMPARE_CONFIG_t PWM_W_compare_config =
{
    .timer_mode = XMC_CCU8_SLICE_TIMER_COUNT_MODE_CA,
    .monoshot = XMC_CCU8_SLICE_TIMER_REPEAT_MODE_REPEAT,
    .shadow_xfer_clear = 0,
    .dither_timer_period = 0,
    .dither_duty_cycle = 0,
    .prescaler_mode = XMC_CCU8_SLICE_PRESCALER_MODE_NORMAL,
    .mcm_ch1_enable = 0,
    .mcm_ch2_enable = 0,
    .slice_status = XMC_CCU8_SLICE_STATUS_CHANNEL_1_AND_2,
    .passive_level_out0 = XMC_CCU8_SLICE_OUTPUT_PASSIVE_LEVEL_LOW,
    .passive_level_out1 = XMC_CCU8_SLICE_OUTPUT_PASSIVE_LEVEL_LOW,
    .passive_level_out2 = XMC_CCU8_SLICE_OUTPUT_PASSIVE_LEVEL_LOW,
    .passive_level_out3 = XMC_CCU8_SLICE_OUTPUT_PASSIVE_LEVEL_LOW,
    .asymmetric_pwm = 1,
    .invert_out0 = true,
    .invert_out1 = false,
    .invert_out2 = false,
    .invert_out3 = false,
    .prescaler_initval = XMC_CCU8_SLICE_PRESCALER_1,
    .float_limit = XMC_CCU8_SLICE_PRESCALER_32768,
    .dither_limit = 0U,
    .timer_concatenation = 0,
};
const XMC_CCU8_SLICE_DEAD_TIME_CONFIG_t PWM_W_dead_time_config =
{
    .enable_dead_time_channel1 = true,
    .enable_dead_time_channel2 = true,
    .channel1_st_path = true,
    .channel2_st_path = true,
    .channel1_inv_st_path = true,
    .channel2_inv_st_path = true,
    .div = XMC_CCU8_SLICE_DTC_DIV_1,
    .channel1_st_rising_edge_counter = 4,
    .channel2_st_rising_edge_counter = 4,
    .channel1_st_falling_edge_counter = 4,
    .channel2_st_falling_edge_counter = 4,
};
const XMC_CCU8_SLICE_EVENT_CONFIG_t PWM_W_event0_config =
{
    .mapped_input = CCU80_IN0_SCU_GSC80,
    .edge = XMC_CCU8_SLICE_EVENT_EDGE_SENSITIVITY_RISING_EDGE,
    .level = XMC_CCU8_SLICE_EVENT_LEVEL_SENSITIVITY_ACTIVE_HIGH,
    .duration = XMC_CCU8_SLICE_EVENT_FILTER_DISABLED,
};
const XMC_CCU8_SLICE_EVENT_CONFIG_t PWM_W_event1_config =
{
    .mapped_input = CCU80_IN0_SCU_GSC80,
    .edge = XMC_CCU8_SLICE_EVENT_EDGE_SENSITIVITY_FALLING_EDGE,
    .level = XMC_CCU8_SLICE_EVENT_LEVEL_SENSITIVITY_ACTIVE_LOW,
    .duration = XMC_CCU8_SLICE_EVENT_FILTER_DISABLED,
};
const XMC_CCU8_SLICE_EVENT_CONFIG_t PWM_W_event2_config =
{
    .edge = XMC_CCU8_SLICE_EVENT_EDGE_SENSITIVITY_NONE,
    .level = XMC_CCU8_SLICE_EVENT_LEVEL_SENSITIVITY_ACTIVE_LOW,
    .duration = XMC_CCU8_SLICE_EVENT_FILTER_5_CYCLES,
};
const XMC_CCU8_SLICE_COMPARE_CONFIG_t PWM_V_compare_config =
{
    .timer_mode = XMC_CCU8_SLICE_TIMER_COUNT_MODE_CA,
    .monoshot = XMC_CCU8_SLICE_TIMER_REPEAT_MODE_REPEAT,
    .shadow_xfer_clear = 0,
    .dither_timer_period = 0,
    .dither_duty_cycle = 0,
    .prescaler_mode = XMC_CCU8_SLICE_PRESCALER_MODE_NORMAL,
    .mcm_ch1_enable = 0,
    .mcm_ch2_enable = 0,
    .slice_status = XMC_CCU8_SLICE_STATUS_CHANNEL_1_AND_2,
    .passive_level_out0 = XMC_CCU8_SLICE_OUTPUT_PASSIVE_LEVEL_LOW,
    .passive_level_out1 = XMC_CCU8_SLICE_OUTPUT_PASSIVE_LEVEL_LOW,
    .passive_level_out2 = XMC_CCU8_SLICE_OUTPUT_PASSIVE_LEVEL_LOW,
    .passive_level_out3 = XMC_CCU8_SLICE_OUTPUT_PASSIVE_LEVEL_LOW,
    .asymmetric_pwm = 1,
    .invert_out0 = true,
    .invert_out1 = false,
    .invert_out2 = false,
    .invert_out3 = false,
    .prescaler_initval = XMC_CCU8_SLICE_PRESCALER_1,
    .float_limit = XMC_CCU8_SLICE_PRESCALER_32768,
    .dither_limit = 0U,
    .timer_concatenation = 0,
};
const XMC_CCU8_SLICE_DEAD_TIME_CONFIG_t PWM_V_dead_time_config =
{
    .enable_dead_time_channel1 = true,
    .enable_dead_time_channel2 = true,
    .channel1_st_path = true,
    .channel2_st_path = true,
    .channel1_inv_st_path = true,
    .channel2_inv_st_path = true,
    .div = XMC_CCU8_SLICE_DTC_DIV_1,
    .channel1_st_rising_edge_counter = 4,
    .channel2_st_rising_edge_counter = 4,
    .channel1_st_falling_edge_counter = 4,
    .channel2_st_falling_edge_counter = 4,
};
const XMC_CCU8_SLICE_EVENT_CONFIG_t PWM_V_event0_config =
{
    .mapped_input = CCU80_IN1_SCU_GSC80,
    .edge = XMC_CCU8_SLICE_EVENT_EDGE_SENSITIVITY_RISING_EDGE,
    .level = XMC_CCU8_SLICE_EVENT_LEVEL_SENSITIVITY_ACTIVE_HIGH,
    .duration = XMC_CCU8_SLICE_EVENT_FILTER_DISABLED,
};
const XMC_CCU8_SLICE_EVENT_CONFIG_t PWM_V_event1_config =
{
    .mapped_input = CCU80_IN1_SCU_GSC80,
    .edge = XMC_CCU8_SLICE_EVENT_EDGE_SENSITIVITY_FALLING_EDGE,
    .level = XMC_CCU8_SLICE_EVENT_LEVEL_SENSITIVITY_ACTIVE_LOW,
    .duration = XMC_CCU8_SLICE_EVENT_FILTER_DISABLED,
};
const XMC_CCU8_SLICE_EVENT_CONFIG_t PWM_V_event2_config =
{
    .edge = XMC_CCU8_SLICE_EVENT_EDGE_SENSITIVITY_NONE,
    .level = XMC_CCU8_SLICE_EVENT_LEVEL_SENSITIVITY_ACTIVE_LOW,
    .duration = XMC_CCU8_SLICE_EVENT_FILTER_5_CYCLES,
};
const XMC_CCU8_SLICE_COMPARE_CONFIG_t PWM_U_compare_config =
{
    .timer_mode = XMC_CCU8_SLICE_TIMER_COUNT_MODE_CA,
    .monoshot = XMC_CCU8_SLICE_TIMER_REPEAT_MODE_REPEAT,
    .shadow_xfer_clear = 0,
    .dither_timer_period = 0,
    .dither_duty_cycle = 0,
    .prescaler_mode = XMC_CCU8_SLICE_PRESCALER_MODE_NORMAL,
    .mcm_ch1_enable = 0,
    .mcm_ch2_enable = 0,
    .slice_status = XMC_CCU8_SLICE_STATUS_CHANNEL_1_AND_2,
    .passive_level_out0 = XMC_CCU8_SLICE_OUTPUT_PASSIVE_LEVEL_LOW,
    .passive_level_out1 = XMC_CCU8_SLICE_OUTPUT_PASSIVE_LEVEL_LOW,
    .passive_level_out2 = XMC_CCU8_SLICE_OUTPUT_PASSIVE_LEVEL_LOW,
    .passive_level_out3 = XMC_CCU8_SLICE_OUTPUT_PASSIVE_LEVEL_LOW,
    .asymmetric_pwm = 1,
    .invert_out0 = true,
    .invert_out1 = false,
    .invert_out2 = false,
    .invert_out3 = false,
    .prescaler_initval = XMC_CCU8_SLICE_PRESCALER_1,
    .float_limit = XMC_CCU8_SLICE_PRESCALER_32768,
    .dither_limit = 0U,
    .timer_concatenation = 0,
};
const XMC_CCU8_SLICE_DEAD_TIME_CONFIG_t PWM_U_dead_time_config =
{
    .enable_dead_time_channel1 = true,
    .enable_dead_time_channel2 = true,
    .channel1_st_path = true,
    .channel2_st_path = true,
    .channel1_inv_st_path = true,
    .channel2_inv_st_path = true,
    .div = XMC_CCU8_SLICE_DTC_DIV_1,
    .channel1_st_rising_edge_counter = 4,
    .channel2_st_rising_edge_counter = 4,
    .channel1_st_falling_edge_counter = 4,
    .channel2_st_falling_edge_counter = 4,
};
const XMC_CCU8_SLICE_EVENT_CONFIG_t PWM_U_event0_config =
{
    .mapped_input = CCU80_IN2_SCU_GSC80,
    .edge = XMC_CCU8_SLICE_EVENT_EDGE_SENSITIVITY_RISING_EDGE,
    .level = XMC_CCU8_SLICE_EVENT_LEVEL_SENSITIVITY_ACTIVE_HIGH,
    .duration = XMC_CCU8_SLICE_EVENT_FILTER_DISABLED,
};
const XMC_CCU8_SLICE_EVENT_CONFIG_t PWM_U_event1_config =
{
    .mapped_input = CCU80_IN2_SCU_GSC80,
    .edge = XMC_CCU8_SLICE_EVENT_EDGE_SENSITIVITY_FALLING_EDGE,
    .level = XMC_CCU8_SLICE_EVENT_LEVEL_SENSITIVITY_ACTIVE_LOW,
    .duration = XMC_CCU8_SLICE_EVENT_FILTER_DISABLED,
};
const XMC_CCU8_SLICE_EVENT_CONFIG_t PWM_U_event2_config =
{
    .edge = XMC_CCU8_SLICE_EVENT_EDGE_SENSITIVITY_NONE,
    .level = XMC_CCU8_SLICE_EVENT_LEVEL_SENSITIVITY_ACTIVE_LOW,
    .duration = XMC_CCU8_SLICE_EVENT_FILTER_5_CYCLES,
};
const XMC_CCU8_SLICE_COMPARE_CONFIG_t ADC0_ISR0_compare_config =
{
    .timer_mode = XMC_CCU8_SLICE_TIMER_COUNT_MODE_EA,
    .monoshot = XMC_CCU8_SLICE_TIMER_REPEAT_MODE_REPEAT,
    .shadow_xfer_clear = 0,
    .dither_timer_period = 0,
    .dither_duty_cycle = 0,
    .prescaler_mode = XMC_CCU8_SLICE_PRESCALER_MODE_NORMAL,
    .mcm_ch1_enable = 0,
    .mcm_ch2_enable = 0,
    .slice_status = XMC_CCU8_SLICE_STATUS_CHANNEL_2,
    .passive_level_out0 = XMC_CCU8_SLICE_OUTPUT_PASSIVE_LEVEL_HIGH,
    .passive_level_out1 = XMC_CCU8_SLICE_OUTPUT_PASSIVE_LEVEL_HIGH,
    .passive_level_out2 = XMC_CCU8_SLICE_OUTPUT_PASSIVE_LEVEL_HIGH,
    .passive_level_out3 = XMC_CCU8_SLICE_OUTPUT_PASSIVE_LEVEL_HIGH,
    .asymmetric_pwm = 0,
    .invert_out0 = false,
    .invert_out1 = false,
    .invert_out2 = false,
    .invert_out3 = false,
    .prescaler_initval = XMC_CCU8_SLICE_PRESCALER_1,
    .float_limit = XMC_CCU8_SLICE_PRESCALER_32768,
    .dither_limit = 0U,
    .timer_concatenation = 0,
};
const XMC_CCU8_SLICE_EVENT_CONFIG_t ADC0_ISR0_event0_config =
{
    .mapped_input = CCU80_IN3_SCU_GSC80,
    .edge = XMC_CCU8_SLICE_EVENT_EDGE_SENSITIVITY_RISING_EDGE,
    .level = XMC_CCU8_SLICE_EVENT_LEVEL_SENSITIVITY_ACTIVE_HIGH,
    .duration = XMC_CCU8_SLICE_EVENT_FILTER_DISABLED,
};
const XMC_CCU8_SLICE_EVENT_CONFIG_t ADC0_ISR0_event1_config =
{
    .mapped_input = CCU80_IN3_SCU_GSC80,
    .edge = XMC_CCU8_SLICE_EVENT_EDGE_SENSITIVITY_FALLING_EDGE,
    .level = XMC_CCU8_SLICE_EVENT_LEVEL_SENSITIVITY_ACTIVE_LOW,
    .duration = XMC_CCU8_SLICE_EVENT_FILTER_DISABLED,
};
const XMC_CCU8_SLICE_EVENT_CONFIG_t ADC0_ISR0_event2_config =
{
    .edge = XMC_CCU8_SLICE_EVENT_EDGE_SENSITIVITY_NONE,
    .level = XMC_CCU8_SLICE_EVENT_LEVEL_SENSITIVITY_ACTIVE_HIGH,
    .duration = XMC_CCU8_SLICE_EVENT_FILTER_DISABLED,
};
const XMC_UART_CH_CONFIG_t CYBSP_DEBUG_UART_config =
{
    .baudrate = 115200UL,
    .normal_divider_mode = false,
    .data_bits = 8U,
    .frame_length = 8U,
    .stop_bits = 1U,
    .oversampling = 16U,
    .parity_mode = XMC_USIC_CH_PARITY_MODE_NONE,
};
XMC_VADC_GROUP_CONFIG_t vadc_0_group0_init_config =
{
    .emux_config.starting_external_channel = (uint32_t) 0,
    .emux_config.connected_channel = (uint32_t) 0,
    .emux_config.emux_mode = XMC_VADC_GROUP_EMUXMODE_SWCTRL,
    .emux_config.emux_coding = XMC_VADC_GROUP_EMUXCODE_BINARY,
    .emux_config.stce_usage = (uint32_t) 0,
    .boundary0 = (uint32_t) 0,
    .boundary1 = (uint32_t) 0,
    .arbitration_round_length = (uint32_t) 1,
    .arbiter_mode = (uint32_t) XMC_VADC_GROUP_ARBMODE_ALWAYS,
};
XMC_VADC_GROUP_CONFIG_t vadc_0_group1_init_config =
{
    .emux_config.starting_external_channel = (uint32_t) 0,
    .emux_config.connected_channel = (uint32_t) 0,
    .emux_config.emux_mode = XMC_VADC_GROUP_EMUXMODE_SWCTRL,
    .emux_config.emux_coding = XMC_VADC_GROUP_EMUXCODE_BINARY,
    .emux_config.stce_usage = (uint32_t) 0,
    .boundary0 = (uint32_t) 0,
    .boundary1 = (uint32_t) 0,
    .arbitration_round_length = (uint32_t) 1,
    .arbiter_mode = (uint32_t) XMC_VADC_GROUP_ARBMODE_ALWAYS,
};
const XMC_VADC_GLOBAL_CONFIG_t vadc_0_config =
{
    .boundary0 = 0U,
    .boundary1 = 0U,
    .clock_config.analog_clock_divider = 2U,
    .clock_config.msb_conversion_clock = 0U,
    .clock_config.arbiter_clock_divider = 0U,
    .class0.sample_time_std_conv = (uint32_t) 0,
    .class0.conversion_mode_standard = XMC_VADC_CONVMODE_12BIT,
    .class0.sampling_phase_emux_channel = (uint32_t) 0,
    .class0.conversion_mode_emux = XMC_VADC_CONVMODE_12BIT,
    .class1.sample_time_std_conv = (uint32_t) 0,
    .class1.conversion_mode_standard = XMC_VADC_CONVMODE_12BIT,
    .class1.sampling_phase_emux_channel = (uint32_t) 0,
    .class1.conversion_mode_emux = XMC_VADC_CONVMODE_12BIT,
    .data_reduction_control = 0U,
    .wait_for_read_mode = 0U,
    .event_gen_enable = 0U,
    .disable_sleep_mode_control = 0U,
};
const XMC_VADC_BACKGROUND_CONFIG_t vadc_0_background_scan_config =
{
    .conv_start_mode = (uint32_t) XMC_VADC_STARTMODE_WFS,
    .req_src_priority = (uint32_t) XMC_VADC_GROUP_RS_PRIORITY_0,
    .src_specific_result_reg = (uint32_t) 0,
    .trigger_signal = (uint32_t) XMC_VADC_REQ_TR_P,
    .trigger_edge = (uint32_t) XMC_VADC_TRIGGER_EDGE_NONE,
    .gate_signal = (uint32_t) XMC_VADC_REQ_GT_E,
    .timer_mode = (uint32_t) false,
    .external_trigger = (uint32_t) false,
    .req_src_interrupt = (uint32_t) false,
    .enable_auto_scan = (uint32_t) false,
    .load_mode = (uint32_t) XMC_VADC_SCAN_LOAD_OVERWRITE,
};
const XMC_VADC_QUEUE_ENTRY_t ADC_0_queue_entries_4 =
{
    .channel_num = (uint8_t)4,
    .refill_needed = (uint32_t)true,
    .generate_interrupt = (uint32_t)false,
    .external_trigger = (uint32_t)true,
};
const XMC_VADC_QUEUE_ENTRY_t ADC_0_queue_entries_5 =
{
    .channel_num = (uint8_t)5,
    .refill_needed = (uint32_t)true,
    .generate_interrupt = (uint32_t)false,
    .external_trigger = (uint32_t)false,
};
const XMC_VADC_QUEUE_ENTRY_t ADC_0_queue_entries_6 =
{
    .channel_num = (uint8_t)6,
    .refill_needed = (uint32_t)true,
    .generate_interrupt = (uint32_t)false,
    .external_trigger = (uint32_t)false,
};
const XMC_VADC_QUEUE_ENTRY_t ADC_0_queue_entries_7 =
{
    .channel_num = (uint8_t)7,
    .refill_needed = (uint32_t)true,
    .generate_interrupt = (uint32_t)true,
    .external_trigger = (uint32_t)false,
};
const XMC_VADC_RESULT_CONFIG_t ADC_0_result_0_config =
{
    .data_reduction_control = (uint32_t) 0,
    .post_processing_mode = (uint32_t) XMC_VADC_DMM_REDUCTION_MODE,
    .wait_for_read_mode = (uint32_t) false,
    .part_of_fifo = (uint32_t) false,
    .event_gen_enable = false,
};
const XMC_VADC_RESULT_CONFIG_t ADC_0_result_1_config =
{
    .data_reduction_control = (uint32_t) 0,
    .post_processing_mode = (uint32_t) XMC_VADC_DMM_REDUCTION_MODE,
    .wait_for_read_mode = (uint32_t) false,
    .part_of_fifo = (uint32_t) false,
    .event_gen_enable = false,
};
const XMC_VADC_RESULT_CONFIG_t ADC_0_result_2_config =
{
    .data_reduction_control = (uint32_t) 0,
    .post_processing_mode = (uint32_t) XMC_VADC_DMM_REDUCTION_MODE,
    .wait_for_read_mode = (uint32_t) false,
    .part_of_fifo = (uint32_t) false,
    .event_gen_enable = false,
};
const XMC_VADC_RESULT_CONFIG_t ADC_0_result_3_config =
{
    .data_reduction_control = (uint32_t) 0,
    .post_processing_mode = (uint32_t) XMC_VADC_DMM_REDUCTION_MODE,
    .wait_for_read_mode = (uint32_t) false,
    .part_of_fifo = (uint32_t) false,
    .event_gen_enable = false,
};
const XMC_VADC_QUEUE_CONFIG_t ADC_0_queue_config =
{
    .conv_start_mode = (uint32_t) XMC_VADC_STARTMODE_WFS,
    .req_src_priority = (uint32_t) XMC_VADC_GROUP_RS_PRIORITY_0,
    .src_specific_result_reg = (uint32_t) 0,
    .trigger_edge = (uint32_t) XMC_VADC_TRIGGER_EDGE_RISING,
    .timer_mode = (uint32_t) false,
    .external_trigger = (uint32_t) false,
};
const XMC_VADC_GROUP_CLASS_t vadc_0_0_iclass_0 =
{
    .sample_time_std_conv = (uint32_t) 0,
    .conversion_mode_standard = XMC_VADC_CONVMODE_12BIT,
    .sampling_phase_emux_channel = (uint32_t) 0,
    .conversion_mode_emux = XMC_VADC_CONVMODE_12BIT,
};
const XMC_VADC_GROUP_CLASS_t vadc_0_0_iclass_1 =
{
    .sample_time_std_conv = (uint32_t) 0,
    .conversion_mode_standard = XMC_VADC_CONVMODE_12BIT,
    .sampling_phase_emux_channel = (uint32_t) 0,
    .conversion_mode_emux = XMC_VADC_CONVMODE_12BIT,
};
const XMC_VADC_CHANNEL_CONFIG_t ADC0_CH_IV_config =
{
    .input_class = (uint32_t) XMC_VADC_CHANNEL_CONV_GROUP_CLASS0,
    .lower_boundary_select = (uint32_t) XMC_VADC_CHANNEL_BOUNDARY_GROUP_BOUND0,
    .upper_boundary_select = (uint32_t) XMC_VADC_CHANNEL_BOUNDARY_GROUP_BOUND0,
    .event_gen_criteria = (uint32_t) XMC_VADC_CHANNEL_EVGEN_NEVER,
    .sync_conversion = (uint32_t) false,
    .alternate_reference = (uint32_t) XMC_VADC_CHANNEL_REF_INTREF,
    .result_reg_number = (uint32_t) 0,
    .use_global_result = (uint32_t) 0,
    .result_alignment = (uint32_t) XMC_VADC_RESULT_ALIGN_LEFT,
    .broken_wire_detect_channel = (uint32_t) XMC_VADC_CHANNEL_BWDCH_VAGND,
    .broken_wire_detect = (uint32_t) false,
    .channel_priority = (bool) false,
    .alias_channel = (int8_t) XMC_VADC_CHANNEL_ALIAS_DISABLED,
};
const XMC_VADC_CHANNEL_CONFIG_t ADC0_CH_IW_config =
{
    .input_class = (uint32_t) XMC_VADC_CHANNEL_CONV_GROUP_CLASS0,
    .lower_boundary_select = (uint32_t) XMC_VADC_CHANNEL_BOUNDARY_GROUP_BOUND0,
    .upper_boundary_select = (uint32_t) XMC_VADC_CHANNEL_BOUNDARY_GROUP_BOUND0,
    .event_gen_criteria = (uint32_t) XMC_VADC_CHANNEL_EVGEN_NEVER,
    .sync_conversion = (uint32_t) false,
    .alternate_reference = (uint32_t) XMC_VADC_CHANNEL_REF_INTREF,
    .result_reg_number = (uint32_t) 1,
    .use_global_result = (uint32_t) 0,
    .result_alignment = (uint32_t) XMC_VADC_RESULT_ALIGN_LEFT,
    .broken_wire_detect_channel = (uint32_t) XMC_VADC_CHANNEL_BWDCH_VAGND,
    .broken_wire_detect = (uint32_t) false,
    .channel_priority = (bool) false,
    .alias_channel = (int8_t) XMC_VADC_CHANNEL_ALIAS_DISABLED,
};
const XMC_VADC_CHANNEL_CONFIG_t ADC0_CH_TEMP_config =
{
    .input_class = (uint32_t) XMC_VADC_CHANNEL_CONV_GROUP_CLASS0,
    .lower_boundary_select = (uint32_t) XMC_VADC_CHANNEL_BOUNDARY_GROUP_BOUND0,
    .upper_boundary_select = (uint32_t) XMC_VADC_CHANNEL_BOUNDARY_GROUP_BOUND0,
    .event_gen_criteria = (uint32_t) XMC_VADC_CHANNEL_EVGEN_NEVER,
    .sync_conversion = (uint32_t) false,
    .alternate_reference = (uint32_t) XMC_VADC_CHANNEL_REF_INTREF,
    .result_reg_number = (uint32_t) 2,
    .use_global_result = (uint32_t) 0,
    .result_alignment = (uint32_t) XMC_VADC_RESULT_ALIGN_LEFT,
    .broken_wire_detect_channel = (uint32_t) XMC_VADC_CHANNEL_BWDCH_VAGND,
    .broken_wire_detect = (uint32_t) false,
    .channel_priority = (bool) false,
    .alias_channel = (int8_t) XMC_VADC_CHANNEL_ALIAS_DISABLED,
};
const XMC_VADC_CHANNEL_CONFIG_t ADC0_CH_VBUS_config =
{
    .input_class = (uint32_t) XMC_VADC_CHANNEL_CONV_GROUP_CLASS0,
    .lower_boundary_select = (uint32_t) XMC_VADC_CHANNEL_BOUNDARY_GROUP_BOUND0,
    .upper_boundary_select = (uint32_t) XMC_VADC_CHANNEL_BOUNDARY_GROUP_BOUND0,
    .event_gen_criteria = (uint32_t) XMC_VADC_CHANNEL_EVGEN_NEVER,
    .sync_conversion = (uint32_t) false,
    .alternate_reference = (uint32_t) XMC_VADC_CHANNEL_REF_INTREF,
    .result_reg_number = (uint32_t) 3,
    .use_global_result = (uint32_t) 0,
    .result_alignment = (uint32_t) XMC_VADC_RESULT_ALIGN_LEFT,
    .broken_wire_detect_channel = (uint32_t) XMC_VADC_CHANNEL_BWDCH_VAGND,
    .broken_wire_detect = (uint32_t) false,
    .channel_priority = (bool) false,
    .alias_channel = (int8_t) XMC_VADC_CHANNEL_ALIAS_DISABLED,
};
const XMC_VADC_QUEUE_ENTRY_t ADC_1_queue_entries_7 =
{
    .channel_num = (uint8_t)3,
    .refill_needed = (uint32_t)true,
    .generate_interrupt = (uint32_t)true,
    .external_trigger = (uint32_t)true,
};
const XMC_VADC_RESULT_CONFIG_t ADC_1_result_0_config =
{
    .data_reduction_control = (uint32_t) 0,
    .post_processing_mode = (uint32_t) XMC_VADC_DMM_REDUCTION_MODE,
    .wait_for_read_mode = (uint32_t) false,
    .part_of_fifo = (uint32_t) false,
    .event_gen_enable = false,
};
const XMC_VADC_QUEUE_CONFIG_t ADC_1_queue_config =
{
    .conv_start_mode = (uint32_t) XMC_VADC_STARTMODE_WFS,
    .req_src_priority = (uint32_t) XMC_VADC_GROUP_RS_PRIORITY_0,
    .src_specific_result_reg = (uint32_t) 0,
    .trigger_edge = (uint32_t) XMC_VADC_TRIGGER_EDGE_RISING,
    .timer_mode = (uint32_t) false,
    .external_trigger = (uint32_t) false,
};
const XMC_VADC_GROUP_CLASS_t vadc_0_1_iclass_0 =
{
    .sample_time_std_conv = (uint32_t) 0,
    .conversion_mode_standard = XMC_VADC_CONVMODE_12BIT,
    .sampling_phase_emux_channel = (uint32_t) 0,
    .conversion_mode_emux = XMC_VADC_CONVMODE_12BIT,
};
const XMC_VADC_GROUP_CLASS_t vadc_0_1_iclass_1 =
{
    .sample_time_std_conv = (uint32_t) 0,
    .conversion_mode_standard = XMC_VADC_CONVMODE_12BIT,
    .sampling_phase_emux_channel = (uint32_t) 0,
    .conversion_mode_emux = XMC_VADC_CONVMODE_12BIT,
};
const XMC_VADC_CHANNEL_CONFIG_t ADC1_CH_IU_config =
{
    .input_class = (uint32_t) XMC_VADC_CHANNEL_CONV_GROUP_CLASS0,
    .lower_boundary_select = (uint32_t) XMC_VADC_CHANNEL_BOUNDARY_GROUP_BOUND0,
    .upper_boundary_select = (uint32_t) XMC_VADC_CHANNEL_BOUNDARY_GROUP_BOUND0,
    .event_gen_criteria = (uint32_t) XMC_VADC_CHANNEL_EVGEN_NEVER,
    .sync_conversion = (uint32_t) false,
    .alternate_reference = (uint32_t) XMC_VADC_CHANNEL_REF_INTREF,
    .result_reg_number = (uint32_t) 0,
    .use_global_result = (uint32_t) 0,
    .result_alignment = (uint32_t) XMC_VADC_RESULT_ALIGN_LEFT,
    .broken_wire_detect_channel = (uint32_t) XMC_VADC_CHANNEL_BWDCH_VAGND,
    .broken_wire_detect = (uint32_t) false,
    .channel_priority = (bool) false,
    .alias_channel = (int8_t) XMC_VADC_CHANNEL_ALIAS_DISABLED,
};

void init_cycfg_peripherals(void)
{
    XMC_CAN_InitEx(can_0_HW, CAN_CLOCK_SOURCE, CAN_FREQUENCY_VALUE);
    if(XMC_CAN_STATUS_SUCCESS == XMC_CAN_NODE_NominalBitTimeConfigureEx(can_0_node_0_HW, &can_0_node_0_bit_time_config))
    {
        XMC_CAN_NODE_EnableConfigurationChange(can_0_node_0_HW);
        XMC_CAN_NODE_SetInitBit(can_0_node_0_HW);
        XMC_CAN_NODE_ReSetAnalyzerMode(can_0_node_0_HW);
        XMC_CAN_NODE_SetReceiveInput(can_0_node_0_HW, CAN_NODE0_RECEIVE_INPUT);
        XMC_CAN_AllocateMOtoNodeList(can_0_HW, 0, 0);
        XMC_CAN_MO_Config(&can_0_node_0_LMO_0);
        /* Reset CCE and INIT bit NCR for node configuration. */
        XMC_CAN_NODE_DisableConfigurationChange(can_0_node_0_HW);
        XMC_CAN_NODE_ResetInitBit(can_0_node_0_HW);
    }
    XMC_CCU4_Init(SYNC_ISRn_HW, XMC_CCU4_SLICE_MCMS_ACTION_TRANSFER_PR_CR);
    XMC_CCU4_StartPrescaler(SYNC_ISRn_HW);
    XMC_CCU4_SLICE_CompareInit(SYNC_ISR1_HW, &SYNC_ISR1_compare_config);
    XMC_CCU4_SLICE_SetTimerCompareMatch(SYNC_ISR1_HW, 39999U);
    XMC_CCU4_SLICE_SetTimerPeriodMatch(SYNC_ISR1_HW, 39999U);
    XMC_CCU4_SetMultiChannelShadowTransferMode(SYNC_ISRn_HW, XMC_CCU4_MULTI_CHANNEL_SHADOW_TRANSFER_SW_SLICE0);
    XMC_CCU4_EnableShadowTransfer(SYNC_ISRn_HW,
        XMC_CCU4_SHADOW_TRANSFER_SLICE_0 |
        XMC_CCU4_SHADOW_TRANSFER_DITHER_SLICE_0 |
        XMC_CCU4_SHADOW_TRANSFER_PRESCALER_SLICE_0 );
    XMC_CCU4_SLICE_ConfigureEvent(SYNC_ISR1_HW, XMC_CCU4_SLICE_EVENT_0, &SYNC_ISR1_event0_config);
    XMC_CCU4_SLICE_ConfigureEvent(SYNC_ISR1_HW, XMC_CCU4_SLICE_EVENT_1, &SYNC_ISR1_event1_config);
    XMC_CCU4_SLICE_ConfigureEvent(SYNC_ISR1_HW, XMC_CCU4_SLICE_EVENT_2, &SYNC_ISR1_event2_config);
    XMC_CCU4_SLICE_StartConfig(SYNC_ISR1_HW, XMC_CCU4_SLICE_EVENT_0, XMC_CCU4_SLICE_START_MODE_TIMER_START_CLEAR);
    XMC_CCU4_SLICE_StopConfig(SYNC_ISR1_HW, XMC_CCU4_SLICE_EVENT_1, XMC_CCU4_SLICE_END_MODE_TIMER_STOP_CLEAR);
    XMC_CCU4_SLICE_SetInterruptNode(SYNC_ISR1_HW, XMC_CCU4_SLICE_IRQ_ID_COMPARE_MATCH_UP, XMC_CCU4_SLICE_SR_ID_3);
    XMC_CCU4_SLICE_EnableEvent(SYNC_ISR1_HW, XMC_CCU4_SLICE_IRQ_ID_COMPARE_MATCH_UP);
    XMC_CCU4_EnableClock(SYNC_ISRn_HW, SYNC_ISR1_NUM);
    XMC_CCU4_SLICE_SetTimerValue(SYNC_ISR1_HW, 0U);
    XMC_CCU4_Init(ccu4_1_HW, XMC_CCU4_SLICE_MCMS_ACTION_TRANSFER_PR_CR);
    XMC_CCU4_StartPrescaler(ccu4_1_HW);
    XMC_CCU4_SLICE_CompareInit(EXE_TIMER_L_HW, &EXE_TIMER_L_compare_config);
    XMC_CCU4_SLICE_SetTimerCompareMatch(EXE_TIMER_L_HW, 0U);
    XMC_CCU4_SLICE_SetTimerPeriodMatch(EXE_TIMER_L_HW, 65535U);
    XMC_CCU4_SetMultiChannelShadowTransferMode(ccu4_1_HW, XMC_CCU4_MULTI_CHANNEL_SHADOW_TRANSFER_SW_SLICE0);
    XMC_CCU4_EnableShadowTransfer(ccu4_1_HW,
        XMC_CCU4_SHADOW_TRANSFER_SLICE_0 |
        XMC_CCU4_SHADOW_TRANSFER_DITHER_SLICE_0 |
        XMC_CCU4_SHADOW_TRANSFER_PRESCALER_SLICE_0 );
    XMC_CCU4_SLICE_ConfigureEvent(EXE_TIMER_L_HW, XMC_CCU4_SLICE_EVENT_0, &EXE_TIMER_L_event0_config);
    XMC_CCU4_SLICE_ConfigureEvent(EXE_TIMER_L_HW, XMC_CCU4_SLICE_EVENT_1, &EXE_TIMER_L_event1_config);
    XMC_CCU4_SLICE_ConfigureEvent(EXE_TIMER_L_HW, XMC_CCU4_SLICE_EVENT_2, &EXE_TIMER_L_event2_config);
    XMC_CCU4_EnableClock(ccu4_1_HW, EXE_TIMER_L_NUM);
    XMC_CCU4_SLICE_SetTimerValue(EXE_TIMER_L_HW, 0U);
    XMC_CCU4_SLICE_CompareInit(EXE_TIMER_H_HW, &EXE_TIMER_H_compare_config);
    XMC_CCU4_SLICE_SetTimerCompareMatch(EXE_TIMER_H_HW, 0U);
    XMC_CCU4_SLICE_SetTimerPeriodMatch(EXE_TIMER_H_HW, 65535U);
    XMC_CCU4_SetMultiChannelShadowTransferMode(ccu4_1_HW, XMC_CCU4_MULTI_CHANNEL_SHADOW_TRANSFER_SW_SLICE1);
    XMC_CCU4_EnableShadowTransfer(ccu4_1_HW,
        XMC_CCU4_SHADOW_TRANSFER_SLICE_1 |
        XMC_CCU4_SHADOW_TRANSFER_DITHER_SLICE_1 |
        XMC_CCU4_SHADOW_TRANSFER_PRESCALER_SLICE_1 );
    XMC_CCU4_SLICE_ConfigureEvent(EXE_TIMER_H_HW, XMC_CCU4_SLICE_EVENT_0, &EXE_TIMER_H_event0_config);
    XMC_CCU4_SLICE_ConfigureEvent(EXE_TIMER_H_HW, XMC_CCU4_SLICE_EVENT_1, &EXE_TIMER_H_event1_config);
    XMC_CCU4_SLICE_ConfigureEvent(EXE_TIMER_H_HW, XMC_CCU4_SLICE_EVENT_2, &EXE_TIMER_H_event2_config);
    XMC_CCU4_EnableClock(ccu4_1_HW, EXE_TIMER_H_NUM);
    XMC_CCU4_SLICE_SetTimerValue(EXE_TIMER_H_HW, 0U);
    XMC_CCU4_SLICE_CompareInit(HALL_TIMER_L_HW, &HALL_TIMER_L_compare_config);
    XMC_CCU4_SLICE_SetTimerCompareMatch(HALL_TIMER_L_HW, 0U);
    XMC_CCU4_SLICE_SetTimerPeriodMatch(HALL_TIMER_L_HW, 65535U);
    XMC_CCU4_SetMultiChannelShadowTransferMode(ccu4_1_HW, XMC_CCU4_MULTI_CHANNEL_SHADOW_TRANSFER_SW_SLICE2);
    XMC_CCU4_EnableShadowTransfer(ccu4_1_HW,
        XMC_CCU4_SHADOW_TRANSFER_SLICE_2 |
        XMC_CCU4_SHADOW_TRANSFER_DITHER_SLICE_2 |
        XMC_CCU4_SHADOW_TRANSFER_PRESCALER_SLICE_2 );
    XMC_CCU4_SLICE_ConfigureEvent(HALL_TIMER_L_HW, XMC_CCU4_SLICE_EVENT_0, &HALL_TIMER_L_event0_config);
    XMC_CCU4_SLICE_ConfigureEvent(HALL_TIMER_L_HW, XMC_CCU4_SLICE_EVENT_1, &HALL_TIMER_L_event1_config);
    XMC_CCU4_SLICE_ConfigureEvent(HALL_TIMER_L_HW, XMC_CCU4_SLICE_EVENT_2, &HALL_TIMER_L_event2_config);
    XMC_CCU4_EnableClock(ccu4_1_HW, HALL_TIMER_L_NUM);
    XMC_CCU4_SLICE_SetTimerValue(HALL_TIMER_L_HW, 0U);
    XMC_CCU4_SLICE_CompareInit(HALL_TIMER_H_HW, &HALL_TIMER_H_compare_config);
    XMC_CCU4_SLICE_SetTimerCompareMatch(HALL_TIMER_H_HW, 0U);
    XMC_CCU4_SLICE_SetTimerPeriodMatch(HALL_TIMER_H_HW, 65535U);
    XMC_CCU4_SetMultiChannelShadowTransferMode(ccu4_1_HW, XMC_CCU4_MULTI_CHANNEL_SHADOW_TRANSFER_SW_SLICE3);
    XMC_CCU4_EnableShadowTransfer(ccu4_1_HW,
        XMC_CCU4_SHADOW_TRANSFER_SLICE_3 |
        XMC_CCU4_SHADOW_TRANSFER_DITHER_SLICE_3 |
        XMC_CCU4_SHADOW_TRANSFER_PRESCALER_SLICE_3 );
    XMC_CCU4_SLICE_ConfigureEvent(HALL_TIMER_H_HW, XMC_CCU4_SLICE_EVENT_0, &HALL_TIMER_H_event0_config);
    XMC_CCU4_SLICE_ConfigureEvent(HALL_TIMER_H_HW, XMC_CCU4_SLICE_EVENT_1, &HALL_TIMER_H_event1_config);
    XMC_CCU4_SLICE_ConfigureEvent(HALL_TIMER_H_HW, XMC_CCU4_SLICE_EVENT_2, &HALL_TIMER_H_event2_config);
    XMC_CCU4_EnableClock(ccu4_1_HW, HALL_TIMER_H_NUM);
    XMC_CCU4_SLICE_SetTimerValue(HALL_TIMER_H_HW, 0U);
    XMC_CCU8_Init(PWM_UVW_ADCn_ISRn_HW, XMC_CCU8_SLICE_MCMS_ACTION_TRANSFER_PR_CR);
    XMC_CCU8_StartPrescaler(PWM_UVW_ADCn_ISRn_HW);
    XMC_CCU8_SLICE_CompareInit(PWM_W_HW, &PWM_W_compare_config);
    XMC_CCU8_SLICE_SetTimerCompareMatchChannel1(PWM_W_HW, 0U);
    XMC_CCU8_SLICE_SetTimerCompareMatchChannel2(PWM_W_HW, 0U);
    XMC_CCU8_SLICE_SetTimerPeriodMatch(PWM_W_HW, 4000U);
    XMC_CCU8_SetMultiChannelShadowTransferMode(PWM_UVW_ADCn_ISRn_HW, XMC_CCU8_MULTI_CHANNEL_SHADOW_TRANSFER_SW_SLICE0);
    XMC_CCU8_EnableShadowTransfer(PWM_UVW_ADCn_ISRn_HW,XMC_CCU8_SHADOW_TRANSFER_SLICE_0 |XMC_CCU8_SHADOW_TRANSFER_DITHER_SLICE_0 |XMC_CCU8_SHADOW_TRANSFER_PRESCALER_SLICE_0 );
    XMC_CCU8_SLICE_ConfigureEvent(PWM_W_HW, XMC_CCU8_SLICE_EVENT_0, &PWM_W_event0_config);
    XMC_CCU8_SLICE_ConfigureEvent(PWM_W_HW, XMC_CCU8_SLICE_EVENT_1, &PWM_W_event1_config);
    XMC_CCU8_SLICE_ConfigureEvent(PWM_W_HW, XMC_CCU8_SLICE_EVENT_2, &PWM_W_event2_config);
    XMC_CCU8_SLICE_StartConfig(PWM_W_HW, XMC_CCU8_SLICE_EVENT_0, XMC_CCU8_SLICE_START_MODE_TIMER_START_CLEAR);
    XMC_CCU8_SLICE_StopConfig(PWM_W_HW, XMC_CCU8_SLICE_EVENT_1, XMC_CCU8_SLICE_END_MODE_TIMER_STOP_CLEAR);
    XMC_CCU8_EnableClock(PWM_UVW_ADCn_ISRn_HW, PWM_W_NUM);
    XMC_CCU8_SLICE_DeadTimeInit(PWM_W_HW, &PWM_W_dead_time_config);
    XMC_CCU8_SLICE_SetTimerValue(PWM_W_HW, 0U);
    XMC_CCU8_SLICE_CompareInit(PWM_V_HW, &PWM_V_compare_config);
    XMC_CCU8_SLICE_SetTimerCompareMatchChannel1(PWM_V_HW, 0U);
    XMC_CCU8_SLICE_SetTimerCompareMatchChannel2(PWM_V_HW, 0U);
    XMC_CCU8_SLICE_SetTimerPeriodMatch(PWM_V_HW, 4000U);
    XMC_CCU8_SetMultiChannelShadowTransferMode(PWM_UVW_ADCn_ISRn_HW, XMC_CCU8_MULTI_CHANNEL_SHADOW_TRANSFER_SW_SLICE1);
    XMC_CCU8_EnableShadowTransfer(PWM_UVW_ADCn_ISRn_HW,XMC_CCU8_SHADOW_TRANSFER_SLICE_1 |XMC_CCU8_SHADOW_TRANSFER_DITHER_SLICE_1 |XMC_CCU8_SHADOW_TRANSFER_PRESCALER_SLICE_1 );
    XMC_CCU8_SLICE_ConfigureEvent(PWM_V_HW, XMC_CCU8_SLICE_EVENT_0, &PWM_V_event0_config);
    XMC_CCU8_SLICE_ConfigureEvent(PWM_V_HW, XMC_CCU8_SLICE_EVENT_1, &PWM_V_event1_config);
    XMC_CCU8_SLICE_ConfigureEvent(PWM_V_HW, XMC_CCU8_SLICE_EVENT_2, &PWM_V_event2_config);
    XMC_CCU8_SLICE_StartConfig(PWM_V_HW, XMC_CCU8_SLICE_EVENT_0, XMC_CCU8_SLICE_START_MODE_TIMER_START_CLEAR);
    XMC_CCU8_SLICE_StopConfig(PWM_V_HW, XMC_CCU8_SLICE_EVENT_1, XMC_CCU8_SLICE_END_MODE_TIMER_STOP_CLEAR);
    XMC_CCU8_SLICE_TrapConfig(PWM_V_HW, XMC_CCU8_SLICE_TRAP_EXIT_MODE_SW, true);
    XMC_CCU8_EnableClock(PWM_UVW_ADCn_ISRn_HW, PWM_V_NUM);
    XMC_CCU8_SLICE_DeadTimeInit(PWM_V_HW, &PWM_V_dead_time_config);
    XMC_CCU8_SLICE_SetTimerValue(PWM_V_HW, 0U);
    XMC_CCU8_SLICE_CompareInit(PWM_U_HW, &PWM_U_compare_config);
    XMC_CCU8_SLICE_SetTimerCompareMatchChannel1(PWM_U_HW, 0U);
    XMC_CCU8_SLICE_SetTimerCompareMatchChannel2(PWM_U_HW, 0U);
    XMC_CCU8_SLICE_SetTimerPeriodMatch(PWM_U_HW, 4000U);
    XMC_CCU8_SetMultiChannelShadowTransferMode(PWM_UVW_ADCn_ISRn_HW, XMC_CCU8_MULTI_CHANNEL_SHADOW_TRANSFER_SW_SLICE2);
    XMC_CCU8_EnableShadowTransfer(PWM_UVW_ADCn_ISRn_HW,XMC_CCU8_SHADOW_TRANSFER_SLICE_2 |XMC_CCU8_SHADOW_TRANSFER_DITHER_SLICE_2 |XMC_CCU8_SHADOW_TRANSFER_PRESCALER_SLICE_2 );
    XMC_CCU8_SLICE_ConfigureEvent(PWM_U_HW, XMC_CCU8_SLICE_EVENT_0, &PWM_U_event0_config);
    XMC_CCU8_SLICE_ConfigureEvent(PWM_U_HW, XMC_CCU8_SLICE_EVENT_1, &PWM_U_event1_config);
    XMC_CCU8_SLICE_ConfigureEvent(PWM_U_HW, XMC_CCU8_SLICE_EVENT_2, &PWM_U_event2_config);
    XMC_CCU8_SLICE_StartConfig(PWM_U_HW, XMC_CCU8_SLICE_EVENT_0, XMC_CCU8_SLICE_START_MODE_TIMER_START_CLEAR);
    XMC_CCU8_SLICE_StopConfig(PWM_U_HW, XMC_CCU8_SLICE_EVENT_1, XMC_CCU8_SLICE_END_MODE_TIMER_STOP_CLEAR);
    XMC_CCU8_SLICE_TrapConfig(PWM_U_HW, XMC_CCU8_SLICE_TRAP_EXIT_MODE_SW, true);
    XMC_CCU8_EnableClock(PWM_UVW_ADCn_ISRn_HW, PWM_U_NUM);
    XMC_CCU8_SLICE_DeadTimeInit(PWM_U_HW, &PWM_U_dead_time_config);
    XMC_CCU8_SLICE_SetTimerValue(PWM_U_HW, 0U);
    XMC_CCU8_SLICE_CompareInit(ADC0_ISR0_HW, &ADC0_ISR0_compare_config);
    XMC_CCU8_SLICE_SetTimerCompareMatchChannel1(ADC0_ISR0_HW, 4000U);
    XMC_CCU8_SLICE_SetTimerCompareMatchChannel2(ADC0_ISR0_HW, 4000U);
    XMC_CCU8_SLICE_SetTimerPeriodMatch(ADC0_ISR0_HW, 7999U);
    XMC_CCU8_SetMultiChannelShadowTransferMode(PWM_UVW_ADCn_ISRn_HW, XMC_CCU8_MULTI_CHANNEL_SHADOW_TRANSFER_SW_SLICE3);
    XMC_CCU8_EnableShadowTransfer(PWM_UVW_ADCn_ISRn_HW,XMC_CCU8_SHADOW_TRANSFER_SLICE_3 |XMC_CCU8_SHADOW_TRANSFER_DITHER_SLICE_3 |XMC_CCU8_SHADOW_TRANSFER_PRESCALER_SLICE_3 );
    XMC_CCU8_SLICE_ConfigureEvent(ADC0_ISR0_HW, XMC_CCU8_SLICE_EVENT_0, &ADC0_ISR0_event0_config);
    XMC_CCU8_SLICE_ConfigureEvent(ADC0_ISR0_HW, XMC_CCU8_SLICE_EVENT_1, &ADC0_ISR0_event1_config);
    XMC_CCU8_SLICE_ConfigureEvent(ADC0_ISR0_HW, XMC_CCU8_SLICE_EVENT_2, &ADC0_ISR0_event2_config);
    XMC_CCU8_SLICE_StartConfig(ADC0_ISR0_HW, XMC_CCU8_SLICE_EVENT_0, XMC_CCU8_SLICE_START_MODE_TIMER_START_CLEAR);
    XMC_CCU8_SLICE_StopConfig(ADC0_ISR0_HW, XMC_CCU8_SLICE_EVENT_1, XMC_CCU8_SLICE_END_MODE_TIMER_STOP_CLEAR);
    XMC_CCU8_SLICE_SetInterruptNode(ADC0_ISR0_HW, XMC_CCU8_SLICE_IRQ_ID_COMPARE_MATCH_UP_CH_1, XMC_CCU8_SLICE_SR_ID_2);
    XMC_CCU8_SLICE_SetInterruptNode(ADC0_ISR0_HW, XMC_CCU8_SLICE_IRQ_ID_COMPARE_MATCH_UP_CH_2, XMC_CCU8_SLICE_SR_ID_3);
    XMC_CCU8_SLICE_EnableEvent(ADC0_ISR0_HW, XMC_CCU8_SLICE_IRQ_ID_COMPARE_MATCH_UP_CH_1);
    XMC_CCU8_SLICE_EnableEvent(ADC0_ISR0_HW, XMC_CCU8_SLICE_IRQ_ID_COMPARE_MATCH_UP_CH_2);
    XMC_CCU8_EnableClock(PWM_UVW_ADCn_ISRn_HW, ADC0_ISR0_NUM);
    XMC_CCU8_SLICE_SetTimerValue(ADC0_ISR0_HW, 0U);
    XMC_UART_CH_InitEx(CYBSP_DEBUG_UART_HW, &CYBSP_DEBUG_UART_config, false);
    XMC_UART_CH_SetInputSource(CYBSP_DEBUG_UART_HW, (XMC_UART_CH_INPUT_t)XMC_USIC_CH_INPUT_DX0, CYBSP_DEBUG_UART_DX0_INPUT);
    XMC_UART_CH_SetSamplePoint(CYBSP_DEBUG_UART_HW, 8U);
    XMC_USIC_CH_SetFractionalDivider(CYBSP_DEBUG_UART_HW, XMC_USIC_CH_BRG_CLOCK_DIVIDER_MODE_FRACTIONAL, 754U);
    XMC_USIC_CH_SetBaudrateDivider(CYBSP_DEBUG_UART_HW, XMC_USIC_CH_BRG_CLOCK_SOURCE_DIVIDER, false, 31U, XMC_USIC_CH_BRG_CTQSEL_PDIV, 0U, 15U);
    XMC_UART_CH_Start(CYBSP_DEBUG_UART_HW);
    /* Update group input classes configuration. */
    vadc_0_group0_init_config.class0 = vadc_0_0_iclass_0;
    vadc_0_group1_init_config.class0 = vadc_0_1_iclass_0;
    vadc_0_group0_init_config.class1 = vadc_0_0_iclass_1;
    vadc_0_group1_init_config.class1 = vadc_0_1_iclass_1;
    /* Global configuration. */
    XMC_VADC_GLOBAL_Init(vadc_0_HW, &vadc_0_config);
    XMC_VADC_GROUP_Init(ADC_0_HW, &vadc_0_group0_init_config);
    XMC_VADC_GROUP_Init(ADC_1_HW, &vadc_0_group1_init_config);
    XMC_VADC_GROUP_SetPowerMode(ADC_0_HW, (XMC_VADC_GROUP_POWERMODE_t) XMC_VADC_GROUP_POWERMODE_NORMAL);
    XMC_VADC_GROUP_SetPowerMode(ADC_1_HW, (XMC_VADC_GROUP_POWERMODE_t) XMC_VADC_GROUP_POWERMODE_NORMAL);
    XMC_VADC_GLOBAL_DisablePostCalibration(vadc_0_HW, 0U);
    XMC_VADC_GLOBAL_DisablePostCalibration(vadc_0_HW, 1U);
    XMC_VADC_GLOBAL_BindGroupToEMux(vadc_0_HW, 0U, 0U);
    XMC_VADC_GLOBAL_BindGroupToEMux(vadc_0_HW, 1U, 0U);
    /* Request source initializations. */
    XMC_VADC_GROUP_QueueSetGatingMode(ADC_0_HW, (XMC_VADC_GATEMODE_t) XMC_VADC_GATEMODE_IGNORE);
    XMC_VADC_GROUP_QueueSetReqSrcEventInterruptNode(ADC_0_HW, (XMC_VADC_SR_t) XMC_VADC_SR_GROUP_SR0);
    XMC_VADC_GROUP_QueueSetReqSrcEventInterruptNode(ADC_0_HW, (XMC_VADC_SR_t) XMC_VADC_SR_GROUP_SR1);
    XMC_VADC_GROUP_QueueSetReqSrcEventInterruptNode(ADC_0_HW, (XMC_VADC_SR_t) XMC_VADC_SR_GROUP_SR2);
    XMC_VADC_GROUP_QueueSetReqSrcEventInterruptNode(ADC_0_HW, (XMC_VADC_SR_t) XMC_VADC_SR_GROUP_SR3);
    XMC_VADC_GROUP_QueueDisableArbitrationSlot(ADC_0_HW);
    XMC_VADC_GROUP_QueueInit(ADC_0_HW, &ADC_0_queue_config);
    
    /* RESULT init. */
    XMC_VADC_GROUP_ResultInit(ADC_0_HW, (uint32_t)0, &ADC_0_result_0_config);
    XMC_VADC_GROUP_ResultInit(ADC_0_HW, (uint32_t)1, &ADC_0_result_1_config);
    XMC_VADC_GROUP_ResultInit(ADC_0_HW, (uint32_t)2, &ADC_0_result_2_config);
    XMC_VADC_GROUP_ResultInit(ADC_0_HW, (uint32_t)3, &ADC_0_result_3_config);
    
    /* Insert channels into the background request sources. */
    XMC_VADC_GROUP_QueueInsertChannel(ADC_0_HW, ADC_0_queue_entries_4);
    XMC_VADC_GROUP_QueueInsertChannel(ADC_0_HW, ADC_0_queue_entries_5);
    XMC_VADC_GROUP_QueueInsertChannel(ADC_0_HW, ADC_0_queue_entries_6);
    XMC_VADC_GROUP_QueueInsertChannel(ADC_0_HW, ADC_0_queue_entries_7);
    /* Channel init. */
    XMC_VADC_GROUP_ChannelInit(ADC_0_HW, (uint32_t)4, &ADC0_CH_IV_config);
    /* Channel init. */
    XMC_VADC_GROUP_ChannelInit(ADC_0_HW, (uint32_t)5, &ADC0_CH_IW_config);
    /* Channel init. */
    XMC_VADC_GROUP_ChannelInit(ADC_0_HW, (uint32_t)6, &ADC0_CH_TEMP_config);
    /* Channel init. */
    XMC_VADC_GROUP_ChannelInit(ADC_0_HW, (uint32_t)7, &ADC0_CH_VBUS_config);
    /* Request source initializations. */
    XMC_VADC_GROUP_QueueSetGatingMode(ADC_1_HW, (XMC_VADC_GATEMODE_t) XMC_VADC_GATEMODE_IGNORE);
    XMC_VADC_GROUP_QueueSetReqSrcEventInterruptNode(ADC_1_HW, (XMC_VADC_SR_t) XMC_VADC_SR_GROUP_SR0);
    XMC_VADC_GROUP_QueueSetReqSrcEventInterruptNode(ADC_1_HW, (XMC_VADC_SR_t) XMC_VADC_SR_GROUP_SR1);
    XMC_VADC_GROUP_QueueSetReqSrcEventInterruptNode(ADC_1_HW, (XMC_VADC_SR_t) XMC_VADC_SR_GROUP_SR2);
    XMC_VADC_GROUP_QueueSetReqSrcEventInterruptNode(ADC_1_HW, (XMC_VADC_SR_t) XMC_VADC_SR_GROUP_SR3);
    XMC_VADC_GROUP_QueueDisableArbitrationSlot(ADC_1_HW);
    XMC_VADC_GROUP_QueueInit(ADC_1_HW, &ADC_1_queue_config);
    
    /* RESULT init. */
    XMC_VADC_GROUP_ResultInit(ADC_1_HW, (uint32_t)0, &ADC_1_result_0_config);
    
    /* Insert channels into the background request sources. */
    XMC_VADC_GROUP_QueueInsertChannel(ADC_1_HW, ADC_1_queue_entries_7);
    /* Channel init. */
    XMC_VADC_GROUP_ChannelInit(ADC_1_HW, (uint32_t)3, &ADC1_CH_IU_config);
}
