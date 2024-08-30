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

MOTOR_PROFILER_t profiler = { 0 };
static const float Progress_Step = 1.0f / 5.0f;

void MOTOR_PROFILER_Init()
{
    float progress_saved = profiler.progress;
    profiler = (MOTOR_PROFILER_t){ 0 };
    profiler.progress = progress_saved;

    float v_phase_max = LINE_TO_PHASE(params.sys.vdc_nom);
    PI_UpdateParams(&profiler.pi_i_alpha, params.profiler.kp_idc, params.profiler.ki_idc * params.sys.samp.ts0, -v_phase_max, v_phase_max);
    PI_UpdateParams(&profiler.pi_i_beta, params.profiler.kp_idc, params.profiler.ki_idc * params.sys.samp.ts0, -v_phase_max, v_phase_max);

    float w_cmd_coeff = (params.profiler.w_cmd_elec.max - params.profiler.w_cmd_elec.min) / ((float)(PROF_SPEED_POINTS - 1U));
    for (uint8_t index = 0U; index < PROF_SPEED_POINTS; ++index)
    {
        profiler.w_cmd_list[index].elec = (float)(index) * w_cmd_coeff + params.profiler.w_cmd_elec.min;
    }
}

static void CritDamp4thOrderLpfRunAB(AB_t input, AB_t outputs[4], float iir_coeff)
{
    // Critically-damped second-order low pass filter
    outputs[0].alpha += (input.alpha - outputs[0].alpha) * iir_coeff;
    outputs[1].alpha += (outputs[0].alpha - outputs[1].alpha) * iir_coeff;
    outputs[2].alpha += (outputs[1].alpha - outputs[2].alpha) * iir_coeff;
    outputs[3].alpha += (outputs[2].alpha - outputs[3].alpha) * iir_coeff;

    outputs[0].beta += (input.beta - outputs[0].beta) * iir_coeff;
    outputs[1].beta += (outputs[0].beta - outputs[1].beta) * iir_coeff;
    outputs[2].beta += (outputs[1].beta - outputs[2].beta) * iir_coeff;
    outputs[3].beta += (outputs[2].beta - outputs[3].beta) * iir_coeff;

}

static void CritDamp4thOrderLpfRunF(float input, float outputs[4], float iir_coeff)
{
    // Critically-damped second-order low pass filter
    outputs[0] += (input - outputs[0]) * iir_coeff;
    outputs[1] += (outputs[0] - outputs[1]) * iir_coeff;
    outputs[2] += (outputs[1] - outputs[2]) * iir_coeff;
    outputs[3] += (outputs[2] - outputs[3]) * iir_coeff;
}

static void LowFreqCurrentRegulatorISR0()
{
    // Low-frequency current separations:
    vars.i_ab_fb.alpha += (vars.i_ab_fb_tot.alpha - vars.i_ab_fb.alpha) * params.profiler.w_sep * params.sys.samp.ts0;
    vars.i_ab_fb.beta += (vars.i_ab_fb_tot.beta - vars.i_ab_fb.beta) * params.profiler.w_sep * params.sys.samp.ts0;

    // Current regulators:
    PI_Run(&profiler.pi_i_alpha, params.profiler.i_cmd_dc, vars.i_ab_fb.alpha, 0.0f);
    PI_Run(&profiler.pi_i_beta, 0.0f, vars.i_ab_fb.beta, 0.0f);
    vars.v_ab_cmd.alpha = profiler.pi_i_alpha.output;
    vars.v_ab_cmd.beta = profiler.pi_i_beta.output;

    // Adding high-frequency voltages
    vars.v_ab_cmd_tot.alpha = vars.v_ab_cmd.alpha + profiler.v_ab_cmd_hf.alpha;
    vars.v_ab_cmd_tot.beta = vars.v_ab_cmd.beta + profiler.v_ab_cmd_hf.beta;
}

static void HighFreqCurrentProcessISR0()
{
    // High-frequency current separations:
    profiler.i_ab_fb_hf.alpha = vars.i_ab_fb_tot.alpha - vars.i_ab_fb.alpha;
    profiler.i_ab_fb_hf.beta = vars.i_ab_fb_tot.beta - vars.i_ab_fb.beta;

    // Magnitude extraction:
    profiler.i_ab_fb_hf_sq.alpha = 2.0f * POW_TWO(profiler.i_ab_fb_hf.alpha);
    profiler.i_ab_fb_hf_sq.beta = 2.0f * POW_TWO(profiler.i_ab_fb_hf.beta);
    CritDamp4thOrderLpfRunAB(profiler.i_ab_fb_hf_sq, profiler.i_ab_fb_hf_sq_filt, params.profiler.w_sep * params.sys.samp.ts0);
}

static void LowFreqVoltProcessISR0()
{
    // Filtering:
    CritDamp4thOrderLpfRunAB(vars.v_ab_cmd, profiler.v_ab_cmd_lf_filt, params.profiler.w_sep * params.sys.samp.ts0);

    // Magnitude calcualtion:
    profiler.v_s_cmd_lf_filt = sqrtf(POW_TWO(profiler.v_ab_cmd_lf_filt[3].alpha) + POW_TWO(profiler.v_ab_cmd_lf_filt[3].beta));

}

static void HighFreqVoltGenerateISR0()
{
    // Generate excitation angle and sin/cos:
    profiler.th_h = Wrap2Pi(profiler.th_h + profiler.w_h * params.sys.samp.ts0);
    ParkInit(profiler.th_h, &profiler.park_h);

    // Generate high-frequency voltages:
    profiler.v_ab_cmd_hf.alpha = profiler.v_ab_mag_hf.alpha * profiler.park_h.cosine;
    profiler.v_ab_cmd_hf.beta = -profiler.v_ab_mag_hf.beta * profiler.park_h.sine;

}

static inline float HighFreqVoltMagCalc(float l)
{
    return (params.profiler.i_cmd_ac * sqrtf(POW_TWO(profiler.out_est.r) + POW_TWO(l * profiler.w_h)));
}

static inline float ReCalcInd(float z_sq)
{
    float w_h_sq = POW_TWO(profiler.w_h);
    profiler.num += MAX(z_sq - POW_TWO(profiler.out_est.r), 0.0f) * w_h_sq;
    profiler.den += POW_TWO(w_h_sq);
    return (sqrtf(profiler.num / profiler.den));
}

void MOTOR_PROFILER_Entry() // called before entering next state
{
    MOTOR_PROFILER_OUTPUT_t output;
    switch (sm.next)
    {
    case Prof_Rot_Lock:
        profiler.freq_sep_active = true;
        vars.i_ab_fb = (AB_t){ 0.0f, 0.0f };
        PI_Reset(&profiler.pi_i_alpha);
        PI_Reset(&profiler.pi_i_beta);
        profiler.v_ab_cmd_hf = (AB_t){ 0.0f,0.0f };
        StopWatchInit(&profiler.timer, params.profiler.time_rot_lock, params.sys.samp.ts0);
        profiler.progress = 0.0f;
        break;

    case Prof_R:
        profiler.freq_sep_active = true;
        profiler.v_ab_cmd_lf_filt[0] = (AB_t){ 0.0f,0.0f };
        profiler.v_ab_cmd_lf_filt[1] = (AB_t){ 0.0f,0.0f };
        profiler.v_ab_cmd_lf_filt[2] = (AB_t){ 0.0f,0.0f };
        profiler.v_ab_cmd_lf_filt[3] = (AB_t){ 0.0f,0.0f };
        StopWatchInit(&profiler.timer, params.profiler.time_res, params.sys.samp.ts0);
        break;

    case Prof_Ld:
        profiler.freq_sep_active = true;
        profiler.th_h = 0.0f;
        profiler.list_index = 0U;
        profiler.w_h = params.profiler.w_h[profiler.list_index];
        profiler.out_est.ld = 0.0f;
        profiler.num = 0.0f;
        profiler.den = 0.0f;
        profiler.v_ab_mag_hf.alpha = HighFreqVoltMagCalc(profiler.out_est.ld);
        profiler.v_ab_mag_hf.beta = 0.0f;
        profiler.i_ab_fb_hf_sq_filt[0] = (AB_t){ 0.0f,0.0f };
        profiler.i_ab_fb_hf_sq_filt[1] = (AB_t){ 0.0f,0.0f };
        profiler.i_ab_fb_hf_sq_filt[2] = (AB_t){ 0.0f,0.0f };
        profiler.i_ab_fb_hf_sq_filt[3] = (AB_t){ 0.0f,0.0f };
        StopWatchInit(&profiler.timer, params.profiler.time_ind, params.sys.samp.ts0);
        break;

    case Prof_Lq:
        profiler.freq_sep_active = true;
        profiler.list_index = 0U;
        profiler.w_h = params.profiler.w_h[profiler.list_index];
        profiler.out_est.lq = 0.0f;
        profiler.num = 0.0f;
        profiler.den = 0.0f;
        profiler.v_ab_mag_hf.alpha = 0.0f;
        profiler.v_ab_mag_hf.beta = HighFreqVoltMagCalc(profiler.out_est.lq);
        StopWatchInit(&profiler.timer, params.profiler.time_ind, params.sys.samp.ts0);
        break;

    default:
    case Prof_Finished:
        profiler.freq_sep_active = false;

        profiler.num += profiler.lam_filt[3];
        profiler.out_est.lam = profiler.num / profiler.den;
        if (params.profiler.overwrite)
        {
            params.motor.r = profiler.out_est.r;
            params.motor.lq = profiler.out_est.lq;
            params.motor.ld = profiler.out_est.ld;
            params.motor.lam = profiler.out_est.lam;
        }
        else
        {
            params.motor.r = profiler.out_saved.r;
            params.motor.lq = profiler.out_saved.lq;
            params.motor.ld = profiler.out_saved.ld;
            params.motor.lam = profiler.out_saved.lam;
        }

        vars.v_ab_cmd = (AB_t){ 0.0f,0.0f };
        vars.v_ab_cmd_tot = (AB_t){ 0.0f,0.0f };
        vars.d_uvw_cmd = (UVW_t){ 0.0f,0.0f,0.0f };
        hw_fcn.GateDriverEnterHighZ();
        output = profiler.out_est;
        MOTOR_PROFILER_Init();
        profiler.out_est = output;
        profiler.progress += Progress_Step;
        break;

    }
}

void MOTOR_PROFILER_Exit() // called before leaving current state
{
    switch (sm.current)
    {
    case Prof_Rot_Lock:
        profiler.progress += Progress_Step;
        break;

    case Prof_R:
        profiler.out_est.r = profiler.v_s_cmd_lf_filt / params.profiler.i_cmd_dc;
        profiler.progress += Progress_Step;
        break;

    case Prof_Ld:
        profiler.z_sq = POW_TWO(profiler.v_ab_mag_hf.alpha) / profiler.i_ab_fb_hf_sq_filt[3].alpha;
        profiler.out_est.ld = ReCalcInd(profiler.z_sq);
        profiler.progress += Progress_Step;
        break;

    case Prof_Lq:
        profiler.z_sq = POW_TWO(profiler.v_ab_mag_hf.beta) / profiler.i_ab_fb_hf_sq_filt[3].beta;
        profiler.out_est.lq = ReCalcInd(profiler.z_sq);
        
        profiler.freq_sep_active = false;
        profiler.list_index = 0U;
        profiler.w_cmd_final.elec = profiler.w_cmd_list[profiler.list_index].elec;
        profiler.w_cmd_final.mech = ELEC_TO_MECH(profiler.w_cmd_final.elec, params.motor.P);
        profiler.lam = 0.0f;
        profiler.lam_filt[0] = 0.0f;
        profiler.lam_filt[1] = 0.0f;
        profiler.lam_filt[2] = 0.0f;
        profiler.lam_filt[3] = 0.0f;
        profiler.num = 0.0f;
        profiler.den = (float)(PROF_SPEED_POINTS);
        profiler.out_saved.r = params.motor.r;
        profiler.out_saved.lq = params.motor.lq;
        profiler.out_saved.ld = params.motor.ld;
        profiler.out_saved.lam = params.motor.lam;
        params.motor.r = profiler.out_est.r;
        params.motor.lq = profiler.out_est.lq;
        params.motor.ld = profiler.out_est.ld;
        StopWatchInit(&profiler.timer, params.profiler.time_flux, params.sys.samp.ts0);
        profiler.progress += Progress_Step;
        break;

    default:
    case Prof_Finished:
        break;

    }
}

void MOTOR_PROFILER_RunISR0()
{
#if defined(CTRL_METHOD_SFO)
    PARK_t park_delta;
#endif

    switch (sm.current)
    {
    case Prof_Rot_Lock:
        LowFreqCurrentRegulatorISR0();
        StopWatchRun(&profiler.timer);
        break;

    case Prof_R:
        LowFreqCurrentRegulatorISR0();
        LowFreqVoltProcessISR0();
        StopWatchRun(&profiler.timer);
        break;

    case Prof_Ld:
        HighFreqVoltGenerateISR0();
        LowFreqCurrentRegulatorISR0();
        HighFreqCurrentProcessISR0();
        StopWatchRun(&profiler.timer);
        if (StopWatchIsDone(&profiler.timer) && ((++profiler.list_index) < PROF_FREQ_POINTS))
        {
            profiler.z_sq = POW_TWO(profiler.v_ab_mag_hf.alpha) / profiler.i_ab_fb_hf_sq_filt[3].alpha;
            profiler.out_est.ld = ReCalcInd(profiler.z_sq);
            profiler.w_h = params.profiler.w_h[profiler.list_index];
            profiler.v_ab_mag_hf.alpha = HighFreqVoltMagCalc(profiler.out_est.ld);
            StopWatchReset(&profiler.timer);
        }
        break;

    case Prof_Lq:
        HighFreqVoltGenerateISR0();
        LowFreqCurrentRegulatorISR0();
        HighFreqCurrentProcessISR0();
        StopWatchRun(&profiler.timer);
        if (StopWatchIsDone(&profiler.timer) && ((++profiler.list_index) < PROF_FREQ_POINTS))
        {
            profiler.z_sq = POW_TWO(profiler.v_ab_mag_hf.beta) / profiler.i_ab_fb_hf_sq_filt[3].beta;
            profiler.out_est.lq = ReCalcInd(profiler.z_sq);
            profiler.w_h = params.profiler.w_h[profiler.list_index];
            profiler.v_ab_mag_hf.beta = HighFreqVoltMagCalc(profiler.out_est.lq);
            StopWatchReset(&profiler.timer);
        }
        break;

    case Speed_CL:
#if defined(CTRL_METHOD_RFO)
        profiler.lam = obs.pll_r.mag - (params.motor.ld - params.motor.lq) * vars.i_qd_r_fb.d;
#elif defined(CTRL_METHOD_SFO)
        ParkInit(vars.delta_est.elec, &park_delta);
        profiler.lam = vars.la_qd_s_est.d * park_delta.cosine + params.motor.ld * (vars.i_qd_s_fb.q * park_delta.sine - vars.i_qd_s_fb.d * park_delta.cosine);
#endif
        CritDamp4thOrderLpfRunF(profiler.lam, profiler.lam_filt, params.profiler.w0_flux * params.sys.samp.ts0);

        if (ABS(vars.w_cmd_int.elec) == profiler.w_cmd_final.elec)
        {
            StopWatchRun(&profiler.timer);
            if (StopWatchIsDone(&profiler.timer) && ((++profiler.list_index) < PROF_SPEED_POINTS))
            {
                profiler.w_cmd_final.elec = profiler.w_cmd_list[profiler.list_index].elec;
                profiler.num += profiler.lam_filt[3];
                StopWatchReset(&profiler.timer);
            }
        }
        break;

    default:
    case Prof_Finished:
        break;
    }

#if defined(PC_TEST)
    vars.test[46] = profiler.v_ab_cmd_lf_filt[3].alpha;
    vars.test[47] = profiler.v_ab_cmd_lf_filt[3].beta;
    vars.test[48] = profiler.v_s_cmd_lf_filt;
    vars.test[49] = profiler.out_est.r;
    vars.test[50] = profiler.i_ab_fb_hf.alpha;
    vars.test[51] = profiler.i_ab_fb_hf.beta;
    vars.test[52] = profiler.i_ab_fb_hf_sq.alpha;
    vars.test[53] = profiler.i_ab_fb_hf_sq.beta;
    vars.test[54] = profiler.i_ab_fb_hf_sq_filt[3].alpha;
    vars.test[55] = profiler.i_ab_fb_hf_sq_filt[3].beta;
    vars.test[56] = profiler.v_ab_mag_hf.alpha;
    vars.test[57] = profiler.v_ab_mag_hf.beta;
    vars.test[58] = profiler.v_ab_cmd_hf.alpha;
    vars.test[59] = profiler.v_ab_cmd_hf.beta;
    vars.test[60] = profiler.th_h;
    vars.test[61] = profiler.w_h;
    vars.test[62] = profiler.z_sq;
    vars.test[63] = profiler.num;
    vars.test[64] = profiler.den;
    vars.test[65] = profiler.out_est.ld;
    vars.test[66] = profiler.out_est.lq;
    vars.test[67] = profiler.w_cmd_final.elec;
    vars.test[68] = profiler.lam;
    vars.test[69] = profiler.lam_filt[3];
    vars.test[70] = profiler.out_est.lam;
#endif
}

#endif