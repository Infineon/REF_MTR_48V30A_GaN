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


#include "Controller.h"

OBS_t obs;

void FLUX_FILT_Reset(FLUX_FILT_t* flux_filt, const float la_lead_0)
{
    BILINEAR_INTEG_Reset(&flux_filt->bilinear_1, 0.0f);
    BILINEAR_INTEG_Reset(&flux_filt->bilinear_2, 0.0f);
    BILINEAR_INTEG_Reset(&flux_filt->bilinear_3, 0.0f);
    flux_filt->bilinear_1.integ = la_lead_0 / obs.flux_filts_shared.gain;
}

float FLUX_FILT_Run(FLUX_FILT_t* flux_filt, const float input)
{
    (float)BILINEAR_INTEG_Run(&flux_filt->bilinear_3, (obs.flux_filts_shared.c3 * flux_filt->bilinear_1.integ) * params.sys.samp.ts0);
    (float)BILINEAR_INTEG_Run(&flux_filt->bilinear_2, (obs.flux_filts_shared.c2 * flux_filt->bilinear_1.integ + flux_filt->bilinear_3.integ) * params.sys.samp.ts0);
    (float)BILINEAR_INTEG_Run(&flux_filt->bilinear_1, (input - obs.flux_filts_shared.c1 * flux_filt->bilinear_1.integ - flux_filt->bilinear_2.integ) * params.sys.samp.ts0);
    return (flux_filt->bilinear_1.integ * obs.flux_filts_shared.gain);
}

void OBS_Init()
{
    vars.v_ab_obs = &vars.v_ab_cmd; // unless overwritten in some certain modes by feedback voltages
    obs.flux_filts_shared.gain = params.obs.flux_filt.gain;
    BIQUAD_ContinuousInit(&obs.biquad_w, params.sys.samp.fs0, params.obs.biquad_a, params.obs.biquad_b);
    obs.phase_comp.cosine = params.obs.flux_filt.phase_lead.cosine;
    PLL_Init(&obs.pll_r, &params.obs.pll, params.sys.samp.ts0);
#if defined(CTRL_METHOD_SFO)
    PLL_Init(&obs.pll_s, &params.obs.pll, params.sys.samp.ts0);
#endif
}

void OBS_Reset(AB_t* la_ab_lead, ELEC_t* w0, ELEC_t* th0)
{
    FLUX_FILT_Reset(&obs.flux_filt_alpha, la_ab_lead->alpha);
    FLUX_FILT_Reset(&obs.flux_filt_beta, la_ab_lead->beta);
    BIQUAD_Reset(&obs.biquad_w, w0->elec);
    PLL_Reset(&obs.pll_r, *w0, *th0);
    vars.w_est.elec = w0->elec;
    vars.th_r_est.elec = th0->elec;
    obs.w_filt_elec = w0->elec;
    obs.w_filt_elec_sign = SIGN(w0->elec);
#if defined(CTRL_METHOD_SFO)
    PLL_Reset(&obs.pll_s, *w0, *th0);
    vars.la_qd_s_est.q = 0.0f;
    vars.th_s_est.elec = th0->elec;
    vars.delta_est.elec = 0.0f;
#endif
}

RAMFUNC_BEGIN
void OBS_RunISR0()
{
    // Back emf calculations
    obs.d_la_ab.alpha = vars.v_ab_obs->alpha - params.motor.r * vars.i_ab_fb.alpha;
    obs.d_la_ab.beta = vars.v_ab_obs->beta - params.motor.r * vars.i_ab_fb.beta;

    // Adaptive flux filters
    if (obs.w_filt_elec > params.obs.w_hyst.elec)
    {
        obs.w_filt_elec_sign = 1.0f;
    }
    else if (obs.w_filt_elec < -params.obs.w_hyst.elec)
    {
        obs.w_filt_elec_sign = -1.0f;
    }
    float w_filt_elec_abs = ABS(obs.w_filt_elec);
    float w_filt_elec_abs_pow_two = POW_TWO(w_filt_elec_abs);
    float w_filt_elec_abs_pow_three = w_filt_elec_abs_pow_two * w_filt_elec_abs;
    obs.flux_filts_shared.c1 = params.obs.flux_filt.c1_coeff * w_filt_elec_abs;
    obs.flux_filts_shared.c2 = params.obs.flux_filt.c2_coeff * w_filt_elec_abs_pow_two;
    obs.flux_filts_shared.c3 = params.obs.flux_filt.c3_coeff * w_filt_elec_abs_pow_three;

    obs.la_ab_lead.alpha = FLUX_FILT_Run(&obs.flux_filt_alpha, obs.d_la_ab.alpha);
    obs.la_ab_lead.beta = FLUX_FILT_Run(&obs.flux_filt_beta, obs.d_la_ab.beta);

    // Current phase lead
    obs.phase_comp.sine = obs.w_filt_elec_sign * params.obs.flux_filt.phase_lead.sine;
    ParkTransform(&vars.i_ab_fb, &obs.phase_comp, (QD_t*)(&obs.i_ab_lead));

    // Adjusted flux calculations
    obs.la_ab_lead_adj.alpha = obs.la_ab_lead.alpha - params.motor.lq * obs.i_ab_lead.alpha;
    obs.la_ab_lead_adj.beta = obs.la_ab_lead.beta - params.motor.lq * obs.i_ab_lead.beta;

    float pll_th_offset_elec = params.obs.pll.th_offset.elec * obs.w_filt_elec_sign;
    // Phase lock loop: rotor angle
    obs.pll_r.th_offset.elec = pll_th_offset_elec;
    PLL_Run(&obs.pll_r, &obs.la_ab_lead_adj);
    vars.th_r_est.elec = obs.pll_r.th.elec;
    vars.w_est.elec = obs.pll_r.w.elec;

#if defined(PC_TEST)
    vars.test[0] = obs.la_ab_lead_adj.alpha;
    vars.test[1] = obs.la_ab_lead_adj.beta;
    vars.test[2] = ATan2(obs.la_ab_lead_adj.alpha, obs.la_ab_lead_adj.beta);
    vars.test[3] = Wrap2Pi(vars.test[2] - pll_th_offset_elec);
    vars.test[4] = Wrap2Pi(vars.test[2] - vars.th_r_fb.elec);
    vars.test[5] = Wrap2Pi(vars.test[3] - vars.th_r_fb.elec);

    vars.test[2] = RAD_TO_DEG(vars.test[2]);
    vars.test[3] = RAD_TO_DEG(vars.test[3]);
    vars.test[4] = RAD_TO_DEG(vars.test[4]);
    vars.test[5] = RAD_TO_DEG(vars.test[5]);
#endif

    // Phase lock loop: stator angle
#if defined(CTRL_METHOD_SFO)
    obs.pll_s.th_offset.elec = pll_th_offset_elec;
    PLL_Run(&obs.pll_s, &obs.la_ab_lead);
    vars.la_qd_s_est.d = sqrtf(POW_TWO(obs.la_ab_lead.alpha) + POW_TWO(obs.la_ab_lead.beta));
    vars.th_s_est.elec = obs.pll_s.th.elec;
    vars.delta_est.elec = Wrap2Pi(obs.pll_s.th.elec - obs.pll_r.th.elec);

#if defined(PC_TEST)
    vars.test[6] = obs.la_ab_lead.alpha;
    vars.test[7] = obs.la_ab_lead.beta;
    vars.test[8] = ATan2(obs.la_ab_lead.alpha, obs.la_ab_lead.beta);
    vars.test[9] = Wrap2Pi(vars.test[8] - pll_th_offset_elec);
    vars.test[10] = Wrap2Pi(vars.test[8] - vars.th_r_fb.elec - vars.delta_fb.elec);
    vars.test[11] = Wrap2Pi(vars.test[9] - vars.th_r_fb.elec - vars.delta_fb.elec);

    vars.test[8] = RAD_TO_DEG(vars.test[8]);
    vars.test[9] = RAD_TO_DEG(vars.test[9]);
    vars.test[10] = RAD_TO_DEG(vars.test[10]);
    vars.test[11] = RAD_TO_DEG(vars.test[11]);
#endif

#endif

    // Biquad filter
    obs.w_filt_elec = BIQUAD_Run(&obs.biquad_w, obs.pll_r.w.elec);
}
RAMFUNC_END