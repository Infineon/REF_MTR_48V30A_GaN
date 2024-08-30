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


#ifdef __cplusplus
extern "C"
{
#endif
#include "../OperationalCode/Controller.h"
#ifdef __cplusplus
}
#endif

#include "SILWrapCtrl.h"
#include "SILAuxFcns.h"

// Simulation constants .............................................................


// ...................................................................................

static float* d_u_cmd;
static float* d_v_cmd;
static float* d_w_cmd;
static UVW_t d_uvw_cmd_high_z = { NAN, NAN, NAN };

static void PhaseUEnterHighZ()
{
    d_u_cmd = &(d_uvw_cmd_high_z.u);
}
static void PhaseVEnterHighZ()
{
    d_v_cmd = &(d_uvw_cmd_high_z.v);
}
static void PhaseWEnterHighZ()
{
    d_w_cmd = &(d_uvw_cmd_high_z.w);
}

static void PhaseUExitHighZ()
{
    d_u_cmd = &(vars.d_uvw_cmd.u);
}
static void PhaseVExitHighZ()
{
    d_v_cmd = &(vars.d_uvw_cmd.v);
}
static void PhaseWExitHighZ()
{
    d_w_cmd = &(vars.d_uvw_cmd.w);
}


void SIL_StartCtrl(PARAMS_t* params_final)
{
    params = *params_final;
    vars = { 0 };

    hall.time_cap_freq = params.sys.samp.fsim;
    STATE_MACHINE_Init();

    PhaseUExitHighZ();
    PhaseVExitHighZ();
    PhaseWExitHighZ();
}

void SIL_TerminateCtrl()
{
    // Nothing to do here
}

void SIL_RunCtrl(SIL_INPUT_CTRL_t* input, SIL_OUTPUT_CTRL_t* output)
{
    // Read inputs .......................................................................................
#pragma warning(disable:4244)	// implicit conversion of double inputs from MATLAB to float
    sensor_iface.v_dc.raw = params.sys.vdc_nom;
    sensor_iface.i_samp_0.raw = input->i_uvw_fb[0];
    sensor_iface.i_samp_1.raw = input->i_uvw_fb[1];
    sensor_iface.i_samp_2.raw = input->i_uvw_fb[2];

    sensor_iface.digital.all = input->digital;
    sensor_iface.pot.raw = input->pot;

    vars.w_fb.mech = input->w_fb_mech;
    vars.w_fb.elec = MECH_TO_ELEC(input->w_fb_mech, params.motor.P);
    vars.th_r_fb.mech = input->th_r_fb_mech;
    vars.th_r_fb.elec = MECH_TO_ELEC(input->th_r_fb_mech, params.motor.P);

#if defined(CTRL_METHOD_SFO)
    vars.delta_fb.elec = input->delta_fb_elec;
#endif

    hall.signal.u = input->hall_signal[0];
    hall.signal.v = input->hall_signal[1];
    hall.signal.w = input->hall_signal[2];
    hall.period_cap = input->hall_time_cap;
#pragma warning(default:4244)	// implicit conversion of double inputs from MATLAB to float

    // Run control loop ..................................................................................
    static uint32_t isr1_ctr = 1U;
    if (isr1_ctr == params.sys.samp.fs0_fs1_ratio)
    {
        isr1_ctr = 1U;
        STATE_MACHINE_RunISR1();
    }
    else
    {
        isr1_ctr++;
    }
    STATE_MACHINE_RunISR0();

    // Write outputs .....................................................................................
    output->state = sm.current;

    output->w_cmd_int_elec = vars.w_cmd_int.elec;
    output->w_hall_elec = vars.w_hall.elec;
    output->w_est_elec = vars.w_est.elec;
    output->w_final_filt_elec = vars.w_final_filt.elec;

    output->th_r_cmd_elec = vars.th_r_cmd.elec;
    output->th_r_hall_elec = vars.th_r_hall.elec;
    output->th_r_est_elec = vars.th_r_est.elec;
    output->th_r_error_elec = RAD_TO_DEG(Wrap2Pi(vars.th_r_final.elec - vars.th_r_fb.elec));

    output->T_est = vars.T_est;

#if defined(CTRL_METHOD_RFO) || defined(CTRL_METHOD_TBC)
    output->i_cmd_int = vars.i_cmd_int;
#endif
    SIL_SetAB(Write, output->i_ab_fb_tot, &vars.i_ab_fb_tot);
    SIL_SetAB(Write, output->i_ab_fb, &vars.i_ab_fb);
    output->i_s_fb = vars.i_s_fb;
    SIL_SetQD(Write, output->i_qd_r_fb, &vars.i_qd_r_fb);

    SIL_SetQD(Write, output->v_qd_r_cmd, &vars.v_qd_r_cmd);
    SIL_SetAB(Write, output->v_ab_cmd, &vars.v_ab_cmd);
    SIL_SetAB(Write, output->v_ab_cmd_tot, &vars.v_ab_cmd_tot);
    output->v_s_cmd = vars.v_s_cmd.rad;

#if defined(CTRL_METHOD_TBC)
    if (ctrl.block_comm.enter_high_z_flag.u) { PhaseUEnterHighZ(); };
    if (ctrl.block_comm.enter_high_z_flag.v) { PhaseVEnterHighZ(); };
    if (ctrl.block_comm.enter_high_z_flag.w) { PhaseWEnterHighZ(); };
    if (ctrl.block_comm.exit_high_z_flag.u) { PhaseUExitHighZ(); };
    if (ctrl.block_comm.exit_high_z_flag.v) { PhaseVExitHighZ(); };
    if (ctrl.block_comm.exit_high_z_flag.w) { PhaseWExitHighZ(); };
#endif

    UVW_t d_uvw_cmd = { *d_u_cmd, *d_v_cmd, *d_w_cmd };
    SIL_SetUVW(Write, output->d_uvw_cmd, &d_uvw_cmd);

    for (uint8_t index = 0; index < MIN(sizeof(vars.test) / sizeof(float), sizeof(vars.test) / sizeof(float)); ++index)
    {
        output->test[index] = vars.test[index];
    }

    output->faults_hw = faults.flags_latched.hw.reg;
    output->faults_sw = faults.flags_latched.sw.reg;

#if defined(CTRL_METHOD_RFO)
    SIL_SetQD(Write, output->la_qd_r_est, &vars.la_qd_r_est);

    SIL_SetQD(Write, output->i_qd_r_ref, &vars.i_qd_r_ref);
    SIL_SetQD(Write, output->i_qd_r_cmd, &vars.i_qd_r_cmd);

#elif defined(CTRL_METHOD_SFO)
    output->delta_cmd_elec = vars.delta_cmd.elec;
    output->delta_est_elec = vars.delta_est.elec;
    output->delta_error_elec = RAD_TO_DEG(Wrap2Pi(vars.delta_est.elec - vars.delta_fb.elec));

    output->la_cmd_mtpa = vars.la_cmd_mtpa;
    output->la_cmd_final = vars.la_cmd_final;
    SIL_SetQD(Write, output->la_qd_r_est, &vars.la_qd_r_est);
    SIL_SetQD(Write, output->la_qd_s_est, &vars.la_qd_s_est);

    output->T_cmd_spd = vars.T_cmd_spd;
    output->T_cmd_int = vars.T_cmd_int;
    output->T_cmd_mtpv = vars.T_cmd_mtpv;
    output->T_cmd_final = vars.T_cmd_final;

    SIL_SetQD(Write, output->i_qd_s_fb, &vars.i_qd_s_fb);

    SIL_SetQD(Write, output->v_qd_s_cmd, &vars.v_qd_s_cmd);

#endif

}