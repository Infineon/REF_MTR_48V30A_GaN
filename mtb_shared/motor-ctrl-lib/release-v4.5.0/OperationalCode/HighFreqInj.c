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

#include "Controller.h"

HIGH_FREQ_INJ_t* high_freq_inj = &ctrl.high_freq_inj;

void HIGH_FREQ_INJ_Init()
{
    BIQUAD_ContinuousInit(&high_freq_inj->lpf_i_q_r, params.sys.samp.fs0, params.ctrl.high_freq_inj.lpf_biquad_a, params.ctrl.high_freq_inj.lpf_biquad_b);
    BIQUAD_ContinuousInit(&high_freq_inj->lpf_i_d_r, params.sys.samp.fs0, params.ctrl.high_freq_inj.lpf_biquad_a, params.ctrl.high_freq_inj.lpf_biquad_b);
    PI_UpdateParams(&high_freq_inj->pi_pll_r, params.ctrl.high_freq_inj.pll.kp, params.ctrl.high_freq_inj.pll.ki * params.sys.samp.ts0,
        -params.ctrl.high_freq_inj.pll.w_max.elec, params.ctrl.high_freq_inj.pll.w_max.elec);
    high_freq_inj->i_qd_r_fb_demod.d = 0.0f; // not needed
}

void HIGH_FREQ_INJ_Reset(const ELEC_t w0, const ELEC_t th0)
{
    BIQUAD_Reset(&high_freq_inj->lpf_i_q_r, 0.0f);
    BIQUAD_Reset(&high_freq_inj->lpf_i_d_r, 0.0f);
    PI_Reset(&high_freq_inj->pi_pll_r);
    high_freq_inj->pi_pll_r.integ = w0.elec;
    BILINEAR_INTEG_Reset(&high_freq_inj->integ_pll_r, th0.elec);
    high_freq_inj->th_h = Elec_Zero;
    vars.park_r = Park_Zero;
}

RAMFUNC_BEGIN
void HIGH_FREQ_INJ_RunFiltISR0()
{
    // Separating current feedback components (low-frequency and high-frequency components)
    ParkTransform(&vars.i_ab_fb_tot, &vars.park_r, &high_freq_inj->i_qd_r_fb_tot); // S061987
    vars.i_qd_r_fb.q = BIQUAD_Run(&high_freq_inj->lpf_i_q_r, high_freq_inj->i_qd_r_fb_tot.q);
    vars.i_qd_r_fb.d = BIQUAD_Run(&high_freq_inj->lpf_i_d_r, high_freq_inj->i_qd_r_fb_tot.d);
    high_freq_inj->i_qd_r_fb_hf.q = high_freq_inj->i_qd_r_fb_tot.q - vars.i_qd_r_fb.q;
    high_freq_inj->i_qd_r_fb_hf.d = high_freq_inj->i_qd_r_fb_tot.d - vars.i_qd_r_fb.d;
    ParkTransformInv(&vars.i_qd_r_fb, &vars.park_r, &vars.i_ab_fb);
}
RAMFUNC_END

RAMFUNC_BEGIN
void HIGH_FREQ_INJ_RunCtrlISR0()
{
    // Generating high-frequency angle and sin/cos terms
    high_freq_inj->th_h.elec = Wrap2Pi(params.ctrl.high_freq_inj.w_h * params.sys.samp.ts0 + high_freq_inj->th_h.elec);
    ParkInit(high_freq_inj->th_h.elec, &high_freq_inj->park_h);

    // Generating high-frequency voltage components to be added to the main voltage components
    high_freq_inj->v_qd_r_cmd.d = params.ctrl.high_freq_inj.v_qd_r_coeff.d * high_freq_inj->park_h.cosine;
    high_freq_inj->v_qd_r_cmd.q = params.ctrl.high_freq_inj.v_qd_r_coeff.q * vars.w_final_filt.elec * high_freq_inj->park_h.sine;
    ParkTransformInv(&high_freq_inj->v_qd_r_cmd, &vars.park_r, &high_freq_inj->v_ab_cmd);

    // Demodulation
    high_freq_inj->i_qd_r_fb_demod.q = high_freq_inj->park_h.sine * high_freq_inj->i_qd_r_fb_hf.q;

    // Pll's pi
    PI_Run(&high_freq_inj->pi_pll_r, 0.0f, -high_freq_inj->i_qd_r_fb_demod.q, 0.0f);
    vars.w_est.elec = high_freq_inj->pi_pll_r.output;

    // Pll's integrator
    high_freq_inj->integ_pll_r.integ =
        Wrap2Pi(BILINEAR_INTEG_Run(&high_freq_inj->integ_pll_r, vars.w_est.elec * params.sys.samp.ts0));
    vars.th_r_est.elec = high_freq_inj->integ_pll_r.integ;
    ParkInit(vars.th_r_est.elec, &vars.park_r);

#if defined(CTRL_METHOD_SFO)
    // Current model
#if defined(REGRESSION_TEST)
    vars.la_qd_r_est.q = params.motor.lq * high_freq_inj->i_qd_r_fb_tot.q;
    vars.la_qd_r_est.d = params.motor.ld * high_freq_inj->i_qd_r_fb_tot.d + params.motor.lam;
#else
    vars.la_qd_r_est.q = params.motor.lq * vars.i_qd_r_fb.q;
    vars.la_qd_r_est.d = params.motor.ld * vars.i_qd_r_fb.d + params.motor.lam;
#endif

    ToPolar(vars.la_qd_r_est.d, vars.la_qd_r_est.q, &high_freq_inj->la_polar_r_est);
    vars.la_qd_s_est.q = 0.0f;
    vars.la_qd_s_est.d = high_freq_inj->la_polar_r_est.rad;
    vars.delta_est.elec = high_freq_inj->la_polar_r_est.theta;
    vars.th_s_est.elec = Wrap2Pi(vars.th_r_est.elec + vars.delta_est.elec);

    ParkInit(vars.delta_est.elec, &high_freq_inj->park_delta);
    ParkTransformInv((QD_t*)(&vars.park_r), &high_freq_inj->park_delta, (AB_t*)(&vars.park_s)); // Less computations compared to ParkInit
    ParkTransform((AB_t*)(&vars.i_qd_r_fb), &high_freq_inj->park_delta, &vars.i_qd_s_fb);
#endif
}
RAMFUNC_END

RAMFUNC_BEGIN
void HIGH_FREQ_INJ_RunISR0()
{
    HIGH_FREQ_INJ_RunFiltISR0();
    HIGH_FREQ_INJ_RunCtrlISR0();
}
RAMFUNC_END

#endif