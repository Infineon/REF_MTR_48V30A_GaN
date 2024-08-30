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


#pragma once

#ifdef __cplusplus
extern "C"
{
#endif
#include "../OperationalCode/Params.h"
#include "../OperationalCode/StateMachine.h"
#ifdef __cplusplus
}
#endif

typedef struct
{
    double i_uvw_fb[3];			// {u, v, w}

    uint32_t digital;
    double pot;					// %, potentiometer

    double w_fb_mech;			// mech
    double th_r_fb_mech;		// mech
    double delta_fb_elec;		// elec

    bool hall_signal[3];		// {u, v, w}
    uint32_t hall_time_cap;	    // ticks, 1/6th of period
} SIL_INPUT_CTRL_t;

typedef struct
{
    STATE_ID_t state;

    double w_cmd_int_elec;		// elec
    double w_hall_elec;			// elec
    double w_est_elec;			// elec
    double w_final_filt_elec;	// elec

    double th_r_cmd_elec;		// elec,
    double th_r_hall_elec;		// elec,
    double th_r_est_elec;		// elec,
    double th_r_error_elec;		// elec, degrees
    double delta_cmd_elec;		// elec, SFO
    double delta_est_elec;		// elec, SFO
    double delta_error_elec;	// elec, SFO, degrees

    double la_cmd_mtpa;			// SFO
    double la_cmd_final;		// SFO
    double la_qd_r_est[2];		//
    double la_qd_s_est[2];		// SFO

    double T_cmd_spd;			// SFO
    double T_cmd_int;			// SFO
    double T_cmd_mtpv;			// SFO
    double T_cmd_final;			// SFO
    double T_est;

    double i_cmd_int;			// RFO, TBC
    double i_qd_r_ref[2];		// {q, d}, RFO
    double i_qd_r_cmd[2];		// {q, d}, RFO
    double i_ab_fb_tot[2];		// {a, b}
    double i_ab_fb[2];			// {a, b}
    double i_s_fb;				//
    double i_qd_r_fb[2];		// {q, d}
    double i_qd_s_fb[2];		// {q, d}, SFO

    double v_qd_r_cmd[2];		// {q, d}
    double v_qd_s_cmd[2];		// {q, d}, SFO
    double v_ab_cmd[2];			// {a, b}
    double v_ab_cmd_tot[2];		// {a, b}
    double v_s_cmd;				//
    double d_uvw_cmd[3];		// {u, v, w}

    uint32_t faults_hw;
    uint32_t faults_sw;

    double test[128];

} SIL_OUTPUT_CTRL_t;

// SIL_GetDefaultParams() >> Overwrite params in MATLAB if necessary >> SIL_Start() + Simulink 
void SIL_GetDefaultParams(PARAMS_t* params_default);

void SIL_StartCtrl(PARAMS_t* params_final);
void SIL_TerminateCtrl();
void SIL_RunCtrl(SIL_INPUT_CTRL_t* input, SIL_OUTPUT_CTRL_t* output);