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


#ifdef _DEBUG	// Export debug data only in debug mode

#include "AuxMethods.h"
#include "AuxMethodTemplates.cpp" // Template definitions are in the source file
#include <iostream>
#include <iomanip>

#ifdef __cplusplus
extern "C"
{
#endif
#include "../OperationalCode/Controller.h"
#ifdef __cplusplus
}
#endif

// parameter string and its corresponding value, including configuration and conversion
#define EMPTY(config, param, conv)  (file << #config << ",,\n")
#define SV(config, param, conv)     (file << #config << "," << #param << "," << conv(param) << "\n")

#define SV_ALL(param,conv)      SV(ALL,param,conv)
#define SV_SEP(config)          EMPTY(config)   // separator lines in gui

#if defined(CTRL_METHOD_RFO)
#define SV_RFO(param,conv)      SV(RFO,param,conv)
#define SV_SFO(param,conv)      EMPTY(SFO,param,conv)
#define SV_TBC(param,conv)      EMPTY(TBC,param,conv)

#define SV_SFO_RFO(param,conv)  SV(SFO_RFO,param,conv)
#define SV_RFO_TBC(param,conv)  SV(RFO_TBC,param,conv)

#define OUTPUT_FILE_NAME        "RFO-ParamDefaultValuesForGUI.csv"

#elif defined(CTRL_METHOD_SFO)
#define SV_RFO(param,conv)      EMPTY(RFO,param,conv)
#define SV_SFO(param,conv)      SV(SFO,param,conv)
#define SV_TBC(param,conv)      EMPTY(TBC,param,conv)

#define SV_SFO_RFO(param,conv)  SV(SFO_RFO,param,conv)
#define SV_RFO_TBC(param,conv)  EMPTY(RFO_TBC,param,conv)

#define OUTPUT_FILE_NAME        "SFO-ParamDefaultValuesForGUI.csv"

#elif defined(CTRL_METHOD_TBC)
#define SV_RFO(param,conv)      EMPTY(RFO,param,conv)
#define SV_SFO(param,conv)      EMPTY(SFO,param,conv)
#define SV_TBC(param,conv)      SV(TBC,param,conv)

#define SV_SFO_RFO(param,conv)  EMPTY(SFO_RFO,param,conv)
#define SV_RFO_TBC(param,conv)  SV(RFO_TBC,param,conv)

#define OUTPUT_FILE_NAME        "TBC-ParamDefaultValuesForGUI.csv"

#endif

TEST_GROUP(GenParamDefaultValues)
{
    void setup()
    {
        PARAMS_Init();
    }

    void teardown()
    {
    }

};

TEST(GenParamDefaultValues, ForGUI)
{
    // Open file
    ofstream file(string("../RegressionTests/ExportedData/") + string(OUTPUT_FILE_NAME));
    file << setprecision(3);

#pragma warning(disable:4003)	// implicit conversion of double inputs from MATLAB to float

    SV_ALL(params.motor.P);
    SV_ALL(params.motor.lq);
    SV_ALL(params.motor.ld);
    SV_ALL(params.motor.lam);
    SV_ALL(params.motor.r);
    SV_ALL(params.motor.T_max);
    SV_ALL(params.motor.i_peak);
    SV_ALL(params.motor.i_cont);
    SV_ALL(params.motor.id_max);
    SV_ALL(params.motor.v_nom, PHASE_TO_LINE);
    SV_ALL(params.motor.w_nom.elec, RADSEC_TO_HZ);
    SV_ALL(params.motor.w_max.elec, RADSEC_TO_HZ);
    SV_SFO(params.motor.mtpv_margin, NORM_TO_PERC);
    SV_ALL(params.motor.zeta);
    SV_ALL(params.motor.i2t.therm_tau);
    SV_ALL(params.motor.i2t.on_level, NORM_TO_PERC);
    SV_ALL(params.motor.i2t.off_level, NORM_TO_PERC);
    SV_SFO(params.motor.mtpa_lut.x_min);
    SV_SFO(params.motor.mtpa_lut.x_max);
    SV_SFO(params.motor.mtpa_lut.x_step);
    SV_SFO(params.motor.mtpa_lut.x_step_inv);
    SV_SFO(params.motor.mtpa_lut.y[0]); // TBD array
    SV_SFO(params.motor.mtpv_lut.x_min);
    SV_SFO(params.motor.mtpv_lut.x_max);
    SV_SFO(params.motor.mtpv_lut.x_step);
    SV_SFO(params.motor.mtpv_lut.x_step_inv);
    SV_SFO(params.motor.mtpv_lut.y[0]); // TBD array
    SV_SFO_RFO(params.profiler.overwrite);
    SV_SFO_RFO(params.profiler.cmd_thresh, NORM_TO_PERC);
    SV_SFO_RFO(params.profiler.cmd_hyst, NORM_TO_PERC);
    SV_SFO_RFO(params.profiler.i_cmd_dc);
    SV_SFO_RFO(params.profiler.i_cmd_ac);
    SV_SFO_RFO(params.profiler.w_cmd_elec.min, RADSEC_TO_HZ);
    SV_SFO_RFO(params.profiler.w_cmd_elec.max, RADSEC_TO_HZ);
    SV_SFO_RFO(params.profiler.time_rot_lock);
    SV_SFO_RFO(params.profiler.time_flux);
    SV_SFO_RFO(params.profiler.w_h[0]); //TBD array
    SV_SFO_RFO(params.profiler.w_sep, RADSEC_TO_HZ);
    SV_SFO_RFO(params.profiler.w0_idc, RADSEC_TO_HZ);
    SV_SFO_RFO(params.profiler.kp_idc);
    SV_SFO_RFO(params.profiler.ki_idc);
    SV_SFO_RFO(params.profiler.time_res);
    SV_SFO_RFO(params.profiler.time_ind);
    SV_SFO_RFO(params.profiler.w0_flux, RADSEC_TO_HZ);
    SV_ALL(params.sys.vdc_nom);
    SV_ALL(params.sys.boot_time);
    SV_SFO_RFO(params.sys.dyno_lock_time);
    SV_SEP(ALL);
    SV_ALL(params.sys.samp.fpwm_fs0_ratio);
    SV_ALL(params.sys.samp.fs0);
    SV_ALL(params.sys.samp.fs0_fs1_ratio);
    SV_ALL(params.sys.samp.ts0);
    SV_ALL(params.sys.samp.fpwm);
    SV_ALL(params.sys.samp.tpwm);
    SV_ALL(params.sys.samp.fs1);
    SV_ALL(params.sys.samp.ts1);
    SV_SEP(ALL);
    SV_ALL(params.sys.analog.offset_null_time);
    SV_SEP(ALL);
    SV_ALL(params.sys.analog.calib.i_u.gain, NORM_TO_PERC);
    SV_ALL(params.sys.analog.calib.i_u.offset);
    SV_ALL(params.sys.analog.calib.i_v.gain, NORM_TO_PERC);
    SV_ALL(params.sys.analog.calib.i_v.offset);
    SV_ALL(params.sys.analog.calib.i_w.gain, NORM_TO_PERC);
    SV_ALL(params.sys.analog.calib.i_w.offset);
    SV_ALL(params.sys.analog.calib.v_uz.gain, NORM_TO_PERC);
    SV_ALL(params.sys.analog.calib.v_uz.offset);
    SV_ALL(params.sys.analog.calib.v_vz.gain, NORM_TO_PERC);
    SV_ALL(params.sys.analog.calib.v_vz.offset);
    SV_ALL(params.sys.analog.calib.v_wz.gain, NORM_TO_PERC);
    SV_ALL(params.sys.analog.calib.v_wz.offset);
    SV_ALL(params.sys.analog.calib.v_dc.gain, NORM_TO_PERC);
    SV_ALL(params.sys.analog.calib.v_dc.offset);
    SV_ALL(params.sys.analog.calib.temp_ps.gain, NORM_TO_PERC);
    SV_ALL(params.sys.analog.calib.temp_ps.offset);
    SV_ALL(params.sys.analog.calib.pot.gain, NORM_TO_PERC);
    SV_ALL(params.sys.analog.calib.pot.offset, NORM_TO_PERC);
    SV_SEP(ALL);
    SV_ALL(params.sys.analog.filt.w0_i_ph, RADSEC_TO_HZ);
    SV_ALL(params.sys.analog.filt.w0_v_ph, RADSEC_TO_HZ);
    SV_ALL(params.sys.analog.filt.w0_v_dc, RADSEC_TO_TAU);
    SV_ALL(params.sys.analog.filt.w0_temp_ps, RADSEC_TO_TAU);
    SV_ALL(params.sys.analog.filt.w0_pot, RADSEC_TO_TAU);
    SV_SEP(ALL);
    SV_ALL(params.sys.analog.shunt.type);
    SV_ALL(params.sys.analog.shunt.res);
    SV_ALL(params.sys.analog.shunt.opamp_gain);
    SV_SEP(ALL);
    SV_ALL(params.sys.analog.shunt.hyb_mod.adc_t_min);
    SV_ALL(params.sys.analog.shunt.hyb_mod.ki);
    SV_ALL(params.sys.analog.shunt.hyb_mod.adc_d_min, NORM_TO_PERC);
    SV_SEP(ALL);
    SV_ALL(params.sys.rate_lim.w_cmd.elec, RADSEC_TO_HZ);
    SV_RFO_TBC(params.sys.rate_lim.i_cmd);
    SV_SFO(params.sys.rate_lim.T_cmd);
    SV_SEP(ALL);
    SV_ALL(params.sys.faults.oc_thresh, NORM_TO_PERC);
    SV_ALL(params.sys.faults.vdc_time);
    SV_ALL(params.sys.faults.temp_ps_thresh);
    SV_ALL(params.sys.faults.short_method);
    SV_ALL(params.sys.faults.max_clr_tries);
    SV_ALL(params.sys.faults.watchdog_time);
    SV_ALL(params.sys.faults.vdc_thresh.min);
    SV_ALL(params.sys.faults.vdc_thresh.max);
    SV_ALL(params.sys.faults.w_thresh.elec, RADSEC_TO_HZ);
    SV_ALL(params.sys.faults.cmd_clr_thresh, NORM_TO_PERC);
    SV_SEP(ALL);
    SV_ALL(params.sys.cmd.source);
    SV_ALL(params.sys.cmd.w_max.mech, RADSEC_TO_HZ);
    SV_RFO_TBC(params.sys.cmd.i_max);
    SV_SFO(params.sys.cmd.T_max);
    SV_SEP(ALL);
    SV_SFO_RFO(params.sys.fb.mode);
    SV_TBC(params.sys.fb.mode);
    SV_SEP(ALL);
    SV_ALL(params.sys.fb.hall.w0_w, RADSEC_TO_HZ);
    SV_ALL(params.sys.fb.hall.w0_th.min, RADSEC_TO_HZ);
    SV_ALL(params.sys.fb.hall.w0_th.max, RADSEC_TO_HZ);
    SV_ALL(params.sys.fb.hall.tau_ratio);
    SV_ALL(params.sys.fb.hall.w_thresh.elec, RADSEC_TO_HZ);
    SV_ALL(params.sys.fb.hall.th_r_offset.elec, RAD_TO_DEG);
    SV_ALL(params.sys.fb.hall.deb_time);
    SV_ALL(params.sys.fb.hall.block_comm_offset_comp);
    SV_RFO(params.obs.w_thresh.elec, RADSEC_TO_HZ);
    SV_SFO(params.obs.w_thresh.elec, RADSEC_TO_HZ);
    SV_RFO(params.obs.w_hyst.elec, RADSEC_TO_HZ);
    SV_SFO(params.obs.w_hyst.elec, RADSEC_TO_HZ);
    SV_RFO(params.obs.lock_time);
    SV_SFO(params.obs.lock_time);
    SV_SEP(SFO_RFO);
    SV_RFO(params.obs.flux_filt.k1);
    SV_SFO(params.obs.flux_filt.k1);
    SV_RFO(params.obs.flux_filt.k2);
    SV_SFO(params.obs.flux_filt.k2);
    SV_RFO(params.obs.flux_filt.k3);
    SV_SFO(params.obs.flux_filt.k3);
    SV_RFO(params.obs.flux_filt.c1_coeff);
    SV_SFO(params.obs.flux_filt.c1_coeff);
    SV_RFO(params.obs.flux_filt.c2_coeff);
    SV_SFO(params.obs.flux_filt.c2_coeff);
    SV_RFO(params.obs.flux_filt.c3_coeff);
    SV_SFO(params.obs.flux_filt.c3_coeff);
    SV_RFO(params.obs.flux_filt.gain);
    SV_SFO(params.obs.flux_filt.gain);
    SV_RFO(params.obs.flux_filt.th_p.elec, RAD_TO_DEG);
    SV_SFO(params.obs.flux_filt.th_p.elec, RAD_TO_DEG);
    SV_RFO(params.obs.flux_filt.phase_lead.sine);
    SV_SFO(params.obs.flux_filt.phase_lead.sine);
    SV_RFO(params.obs.flux_filt.phase_lead.cosine);
    SV_SFO(params.obs.flux_filt.phase_lead.cosine);
    SV_SEP(SFO_RFO);
    SV_RFO(params.obs.biquad_a[0]);
    SV_SFO(params.obs.biquad_a[0]);
    SV_RFO(params.obs.biquad_a[1]);
    SV_SFO(params.obs.biquad_a[1]);
    SV_RFO(params.obs.biquad_a[2]);
    SV_SFO(params.obs.biquad_a[2]);
    SV_RFO(params.obs.biquad_b[0]);
    SV_SFO(params.obs.biquad_b[0]);
    SV_RFO(params.obs.biquad_b[1]);
    SV_SFO(params.obs.biquad_b[1]);
    SV_RFO(params.obs.biquad_b[2]);
    SV_SFO(params.obs.biquad_b[2]);
    SV_SEP(SFO_RFO);
    SV_RFO(params.obs.pll.w0, RADSEC_TO_HZ);
    SV_SFO(params.obs.pll.w0, RADSEC_TO_HZ);
    SV_RFO(params.obs.pll.w_max.elec, RADSEC_TO_HZ);
    SV_SFO(params.obs.pll.w_max.elec, RADSEC_TO_HZ);
    SV_RFO(params.obs.pll.kp);
    SV_SFO(params.obs.pll.kp);
    SV_RFO(params.obs.pll.ki);
    SV_SFO(params.obs.pll.ki);
    SV_RFO(params.obs.pll.th_offset.elec, RAD_TO_DEG);
    SV_SFO(params.obs.pll.th_offset.elec, RAD_TO_DEG);
    SV_ALL(params.mech.inertia);
    SV_ALL(params.mech.viscous);
    SV_ALL(params.mech.friction);
    SV_RFO_TBC(params.filt.acc_w0, RADSEC_TO_HZ);
    SV_SFO(params.filt.acc_w0, RADSEC_TO_HZ);
    SV_ALL(params.filt.trq_w0, RADSEC_TO_HZ);
    SV_SEP(ALL);
    SV_RFO_TBC(params.filt.spd_ar_en);
    SV_SFO(params.filt.spd_ar_en);
    SV_RFO_TBC(params.filt.spd_ar_wp[0], RADSEC_TO_HZ);
    SV_SFO(params.filt.spd_ar_wp[0], RADSEC_TO_HZ);
    SV_RFO_TBC(params.filt.spd_ar_wp[1], RADSEC_TO_HZ);
    SV_SFO(params.filt.spd_ar_wp[1], RADSEC_TO_HZ);
    SV_RFO_TBC(params.filt.spd_ar_wz[0], RADSEC_TO_HZ);
    SV_SFO(params.filt.spd_ar_wz[0], RADSEC_TO_HZ);
    SV_RFO_TBC(params.filt.spd_ar_wz[1], RADSEC_TO_HZ);
    SV_SFO(params.filt.spd_ar_wz[1], RADSEC_TO_HZ);
    SV_RFO(params.ctrl.mode);
    SV_SFO(params.ctrl.mode);
    SV_TBC(params.ctrl.mode);
    SV_SEP(ALL);
    SV_RFO(params.ctrl.speed.bw, RADSEC_TO_HZ);
    SV_SFO(params.ctrl.speed.bw, RADSEC_TO_HZ);
    SV_TBC(params.ctrl.speed.bw, RADSEC_TO_HZ);
    SV_RFO(params.ctrl.speed.ol_cl_tr_coeff, NORM_TO_PERC);
    SV_SFO(params.ctrl.speed.ol_cl_tr_coeff, NORM_TO_PERC);
    SV_TBC(params.ctrl.speed.ol_cl_tr_coeff, NORM_TO_PERC);
    SV_RFO(params.ctrl.speed.kp);
    SV_RFO(params.ctrl.speed.ki);
    SV_TBC(params.ctrl.speed.kp);
    SV_TBC(params.ctrl.speed.ki);
    SV_SFO(params.ctrl.speed.kp);
    SV_SFO(params.ctrl.speed.ki);
    SV_SEP(ALL);
    SV_RFO(params.ctrl.speed.ff_k_inertia);
    SV_RFO(params.ctrl.speed.ff_k_viscous);
    SV_RFO(params.ctrl.speed.ff_k_friction);
    SV_TBC(params.ctrl.speed.ff_k_inertia);
    SV_TBC(params.ctrl.speed.ff_k_viscous);
    SV_TBC(params.ctrl.speed.ff_k_friction);
    SV_SFO(params.ctrl.speed.ff_k_inertia);
    SV_SFO(params.ctrl.speed.ff_k_viscous);
    SV_SFO(params.ctrl.speed.ff_k_friction);
    SV_SEP(RFO_TBC);
    SV_TBC(params.ctrl.curr.bypass);
    SV_TBC(params.ctrl.curr.k_bypass);
    SV_RFO(params.ctrl.curr.bw, RADSEC_TO_HZ);
    SV_TBC(params.ctrl.curr.bw, RADSEC_TO_HZ);
    SV_RFO(params.ctrl.curr.ff_coef, NORM_TO_PERC);
    SV_TBC(params.ctrl.curr.ff_coef, NORM_TO_PERC);
    SV_RFO(params.ctrl.curr.i_cmd_thresh);
    SV_TBC(params.ctrl.curr.i_cmd_thresh);
    SV_RFO(params.ctrl.curr.i_cmd_hyst);
    SV_TBC(params.ctrl.curr.i_cmd_hyst);
    SV_RFO(params.ctrl.curr.kp.q);
    SV_RFO(params.ctrl.curr.kp.d);
    SV_RFO(params.ctrl.curr.ki.q);
    SV_RFO(params.ctrl.curr.ki.d);
    SV_RFO(params.ctrl.curr.v_max.q);
    SV_RFO(params.ctrl.curr.v_max.d);
    SV_TBC(params.ctrl.curr.kp);
    SV_TBC(params.ctrl.curr.ki);
    SV_TBC(params.ctrl.curr.v_max);
    SV_SEP(SFO);
    SV_SFO(params.ctrl.trq.w_z, RADSEC_TO_HZ);
    SV_SFO(params.ctrl.trq.w_ratio, NORM_TO_PERC);
    SV_SFO(params.ctrl.trq.delta_max.elec, RAD_TO_DEG);
    SV_SFO(params.ctrl.trq.T_cmd_thresh);
    SV_SFO(params.ctrl.trq.T_cmd_hyst);
    SV_SFO(params.ctrl.trq.curr_lmt_t_reach);
    SV_SFO(params.ctrl.trq.kp);
    SV_SFO(params.ctrl.trq.ki);
    SV_SFO(params.ctrl.trq.curr_lmt_ki);
    SV_SEP(SFO);
    SV_SFO(params.ctrl.flux.bw, RADSEC_TO_HZ);
    SV_SFO(params.ctrl.flux.pole_sep);
    SV_SFO(params.ctrl.flux.kp);
    SV_SFO(params.ctrl.flux.ki);
    SV_SFO(params.ctrl.flux.vd_max);
    SV_SEP(SFO);
    SV_SFO(params.ctrl.delta.bw, RADSEC_TO_HZ);
    SV_SFO(params.ctrl.delta.bw_mult);
    SV_SFO(params.ctrl.delta.bw_mult_wl.elec, RADSEC_TO_HZ);
    SV_SFO(params.ctrl.delta.bw_mult_wh.elec, RADSEC_TO_HZ);
    SV_SFO(params.ctrl.delta.pole_sep);
    SV_SFO(params.ctrl.delta.bw_mult_slope);
    SV_SFO(params.ctrl.delta.bw_mult_inter);
    SV_SFO(params.ctrl.delta.vq_max);
    SV_SEP(ALL);
    SV_ALL(params.ctrl.volt.w_thresh.elec, RADSEC_TO_HZ);
    SV_ALL(params.ctrl.volt.w_hyst.elec, RADSEC_TO_HZ);
    SV_ALL(params.ctrl.volt.v_min);
    SV_ALL(params.ctrl.volt.v_to_f_ratio);
    SV_ALL(params.ctrl.volt.mod_method);
    SV_SEP(ALL);
    SV_ALL(params.ctrl.volt.five_seg.en);
    SV_ALL(params.ctrl.volt.five_seg.active_mi, NORM_TO_PERC);
    SV_ALL(params.ctrl.volt.five_seg.inactive_mi, NORM_TO_PERC);
    SV_ALL(params.ctrl.volt.five_seg.w0_filt, RADSEC_TO_HZ);
    SV_SEP(SFO_RFO);
    SV_RFO(params.ctrl.flux_weaken.en);
    SV_SFO(params.ctrl.flux_weaken.en);
    SV_RFO(params.ctrl.flux_weaken.vdc_coeff, PHASE_TO_LINE);
    SV_SFO(params.ctrl.flux_weaken.vdc_coeff, PHASE_TO_LINE);
    SV_RFO(params.ctrl.flux_weaken.bw, RADSEC_TO_HZ);
    SV_RFO(params.ctrl.flux_weaken.ki);
    SV_SFO(params.ctrl.flux_weaken.w_min.elec, RADSEC_TO_HZ);
    SV_SEP(SFO_RFO);
    SV_SFO_RFO(params.ctrl.align.time);
    SV_SFO_RFO(params.ctrl.align.voltage);
    SV_SEP(SFO_RFO);
    SV_SFO_RFO(params.ctrl.six_pulse_inj.i_peak);
    SV_SFO_RFO(params.ctrl.six_pulse_inj.t_on);
    SV_SFO_RFO(params.ctrl.six_pulse_inj.t_off);
    SV_SFO_RFO(params.ctrl.six_pulse_inj.v_pulse);
    SV_SEP(SFO_RFO);
    SV_RFO(params.ctrl.high_freq_inj.w_h, RADSEC_TO_HZ);
    SV_SFO(params.ctrl.high_freq_inj.w_h, RADSEC_TO_HZ);
    SV_RFO(params.ctrl.high_freq_inj.w_sep, RADSEC_TO_HZ);
    SV_SFO(params.ctrl.high_freq_inj.w_sep, RADSEC_TO_HZ);
    SV_RFO(params.ctrl.high_freq_inj.i_qd_r_peak.d);
    SV_SFO(params.ctrl.high_freq_inj.i_qd_r_peak.d);
    SV_RFO(params.ctrl.high_freq_inj.v_qd_r_coeff.q);
    SV_SFO(params.ctrl.high_freq_inj.v_qd_r_coeff.q);
    SV_RFO(params.ctrl.high_freq_inj.v_qd_r_coeff.d);
    SV_SFO(params.ctrl.high_freq_inj.v_qd_r_coeff.d);
    SV_RFO(params.ctrl.high_freq_inj.i_qd_r_peak.q);
    SV_SFO(params.ctrl.high_freq_inj.i_qd_r_peak.q);
    SV_RFO(params.ctrl.high_freq_inj.bw_red_coeff, NORM_TO_PERC);
    SV_SFO(params.ctrl.high_freq_inj.bw_red_coeff, NORM_TO_PERC);
    SV_RFO(params.ctrl.high_freq_inj.lock_time);
    SV_SFO(params.ctrl.high_freq_inj.lock_time);
    SV_SEP(SFO_RFO);
    SV_RFO(params.ctrl.high_freq_inj.pll.w0, RADSEC_TO_HZ);
    SV_SFO(params.ctrl.high_freq_inj.pll.w0, RADSEC_TO_HZ);
    SV_RFO(params.ctrl.high_freq_inj.pll.w_max.elec, RADSEC_TO_HZ);
    SV_SFO(params.ctrl.high_freq_inj.pll.w_max.elec, RADSEC_TO_HZ);
    SV_RFO(params.ctrl.high_freq_inj.pll.kp);
    SV_SFO(params.ctrl.high_freq_inj.pll.kp);
    SV_RFO(params.ctrl.high_freq_inj.pll.ki);
    SV_SFO(params.ctrl.high_freq_inj.pll.ki);
    SV_RFO(params.ctrl.high_freq_inj.pll.th_offset.elec, RAD_TO_DEG);
    SV_SFO(params.ctrl.high_freq_inj.pll.th_offset.elec, RAD_TO_DEG);
    SV_SEP(RFO);
    SV_SEP(SFO);
    SV_RFO(params.ctrl.high_freq_inj.lpf_biquad_a[0]);
    SV_SFO(params.ctrl.high_freq_inj.lpf_biquad_a[0]);
    SV_RFO(params.ctrl.high_freq_inj.lpf_biquad_a[1]);
    SV_SFO(params.ctrl.high_freq_inj.lpf_biquad_a[1]);
    SV_RFO(params.ctrl.high_freq_inj.lpf_biquad_a[2]);
    SV_SFO(params.ctrl.high_freq_inj.lpf_biquad_a[2]);
    SV_RFO(params.ctrl.high_freq_inj.lpf_biquad_b[0]);
    SV_SFO(params.ctrl.high_freq_inj.lpf_biquad_b[0]);
    SV_RFO(params.ctrl.high_freq_inj.lpf_biquad_b[1]);
    SV_SFO(params.ctrl.high_freq_inj.lpf_biquad_b[1]);
    SV_RFO(params.ctrl.high_freq_inj.lpf_biquad_b[2]);
    SV_SFO(params.ctrl.high_freq_inj.lpf_biquad_b[2]);
    SV_SEP(TBC);
    SV_TBC(params.ctrl.tbc.mode);
    SV_TBC(params.ctrl.tbc.trap.ramp_cnt);
    SV_TBC(params.ctrl.tbc.trap.ramp_main_bw_ratio, NORM_TO_PERC);
    SV_TBC(params.ctrl.tbc.trap.ramp_ff_coef);
    SV_TBC(params.ctrl.tbc.trap.main_ff_coef);
    SV_TBC(params.ctrl.tbc.trap.ramp_kp);
    SV_TBC(params.ctrl.tbc.trap.ramp_ki);
    SV_TBC(params.ctrl.tbc.trap.ramp_cnt_inv);

#pragma warning(default:4003)	// implicit conversion of double inputs from MATLAB to float

    // Close file
    file.close();
}

#endif