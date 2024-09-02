/*******************************************************************************
* Copyright 2021-2024, Cypress Semiconductor Corporation (an Infineon company) or
* an affiliate of Cypress Semiconductor Corporation.  All rights reserved.
*
* This software, including source code, documentation and related
* materials ("Software") is owned by Cypress Semiconductor Corporation
* or one of its affiliates ("Cypress") and is protected by and subject to
* worldwide patent protection (United States and foreign),
* United States copyright laws and international treaty provisions.
* Therefore, you may use this Software only as provided in the license
* agreement accompanying the software package from which you
* obtained this Software ("EULA").
* If no EULA applies, Cypress hereby grants you a personal, non-exclusive,
* non-transferable license to copy, modify, and compile the Software
* source code solely for use in connection with Cypress's
* integrated circuit products.  Any reproduction, modification, translation,
* compilation, or representation of this Software except as specified
* above is prohibited without the express written permission of Cypress.
*
* Disclaimer: THIS SOFTWARE IS PROVIDED AS-IS, WITH NO WARRANTY OF ANY KIND,
* EXPRESS OR IMPLIED, INCLUDING, BUT NOT LIMITED TO, NONINFRINGEMENT, IMPLIED
* WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE. Cypress
* reserves the right to make changes to the Software without notice. Cypress
* does not assume any liability arising out of the application or use of the
* Software or any product or circuit described in the Software. Cypress does
* not authorize its products for use in any products where a malfunction or
* failure of the Cypress product may reasonably be expected to result in
* significant property damage, injury or death ("High Risk Product"). By
* including Cypress's product in a High Risk Product, the manufacturer
* of such system or application assumes all risk of such use and in doing
* so agrees to indemnify Cypress against all liability.
*******************************************************************************/

#if defined(CTRL_METHOD_RFO) || defined(CTRL_METHOD_SFO)

#pragma once

#include "General.h"
#include "Biquad.h"

typedef struct
{
    float r;			// [ohm], stator resistance
    float lq;			// [H], q-axis inductance
    float ld;			// [H], d-axis inductance
    float lam;			// [Wb], permanent magnet's flux linkage
} MOTOR_PROFILER_OUTPUT_t;

typedef struct
{
    MOTOR_PROFILER_OUTPUT_t out_saved;
    MOTOR_PROFILER_OUTPUT_t out_est;
    TIMER_t timer;
    
    float progress;

    PI_t pi_i_alpha;
    PI_t pi_i_beta;

    AB_t v_ab_mag_hf;
    AB_t v_ab_cmd_hf;
    AB_t v_ab_cmd_lf_filt[4];
    float v_s_cmd_lf_filt;

    AB_t i_ab_fb_hf;
    AB_t i_ab_fb_hf_sq;
    AB_t i_ab_fb_hf_sq_filt[4];

    ELEC_t w_cmd_list[PROF_SPEED_POINTS];
    ELEC_MECH_t w_cmd_final;

    uint8_t list_index;
    float w_h;
    float th_h;
    PARK_t park_h;

    float z_sq;
    float num;
    float den;

    float lam;
    float lam_filt[4];

    bool freq_sep_active;

} MOTOR_PROFILER_t;

extern MOTOR_PROFILER_t profiler;

void MOTOR_PROFILER_Init();
void MOTOR_PROFILER_Entry();
void MOTOR_PROFILER_Exit();
void MOTOR_PROFILER_RunISR0();

#endif
