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

#include "AuxMethods.h"
#include "AuxMethodTemplates.cpp" // Template definitions are in the source file
#include "CSysSim.h"

#ifdef __cplusplus
extern "C"
{
#endif
#include "../OperationalCode/Controller.h"
#ifdef __cplusplus
}
#endif

TEST_GROUP(FieldWeakeningTest)
{
    CSysSim* sys_sim;

    void setup()
    {
        sys_sim = new CSysSim();
        sys_sim->SetDiscreteIntegMethod(CDiscreteInteg::EMethod::Backward_Euler);

        params.sys.samp.fs0 = 50.0E3f;
        params.sys.samp.fsim_fs0_ratio = 2U;
        params.sys.cmd.w_max.mech = ELEC_TO_MECH(HZ_TO_RADSEC(1800.0f), params.motor.P); // 1.8kHz
        params.ctrl.mode = Speed_Mode_FOC_Sensorless_Volt_Startup;
        params.mech.viscous = 0.0002f * 1.0f;
        PARAMS_DEFAULT_InitAutoCalc();
        STATE_MACHINE_Init();
    }

    void teardown()
    {
        delete sys_sim;
    }

};

TEST(FieldWeakeningTest, NegativeSpeedCmdAboveBaseSpeed)
{
    sensor_iface.digital.dir = 0U;
    float dir = (sensor_iface.digital.dir == 1U) ? (+1.0f) : (-1.0f);

    // Turning potentiometer parameters
    sys_sim->m_pot.m_init = 0.0f;			// [%]
    sys_sim->m_pot.m_final = 1.0f;			// [%]
    sys_sim->m_pot.m_t_start = 0.5f;		// [sec]
    sys_sim->m_pot.m_t_stop = 8.5f;			// [sec]
    sys_sim->m_pot.m_t_slope = 7.5f;		// [sec]

    // Setting capture channels for capturing the variables before state transitions
    sm.vars.capture_channels[0] = &vars.w_cmd_int.elec;
    sm.vars.capture_channels[1] = &vars.w_est.elec;
    sm.vars.capture_channels[2] = &vars.w_final.elec;
    sm.vars.capture_channels[3] = &vars.w_final_filt.elec;
#if defined(CTRL_METHOD_RFO)
    sm.vars.capture_channels[4] = &vars.i_cmd_int;
#elif defined(CTRL_METHOD_SFO)
    sm.vars.capture_channels[4] = &vars.T_cmd_int;
#endif


    // Run simulation
    // Init state
    while (sm.current == Init)
    {
        sys_sim->RunOneTick();
    }
    auto init_time = sys_sim->GetDiffTime();
    CHECK_TRUE(sm.current == Brake_Boot);
    CHECK_EQUAL(sm.vars.init.offset_null_done, true);
    CHECK_EQUAL(sm.vars.init.param_init_done, true);
    CHECK_EQUAL(sm.vars.speed_reset_required, false);

    // Brake-boot state
    while (sm.current == Brake_Boot)
    {
        sys_sim->RunOneTick();
    }
    auto brake_boot_time = sys_sim->GetDiffTime();
    CHECK_TRUE(sm.current == Volt_Hz_OL);
    CHECK_EQUAL(StopWatchIsDone(&sm.vars.brake_boot.timer), true);
    CHECK_TRUE(brake_boot_time >= params.sys.boot_time);
    DOUBLES_EQUAL_PERC(params.ctrl.volt.w_thresh.elec * dir, vars.w_cmd_ext.elec, 0.01f);
    DOUBLES_EQUAL_PERC(params.ctrl.volt.w_thresh.elec * dir, vars.w_cmd_int.elec, 0.01f);
    DOUBLES_EQUAL(0.0f, sm.vars.capture_vals[0], 0.1f); // vars.w_cmd_int.elec
    DOUBLES_EQUAL(0.0f, sm.vars.capture_vals[2], 0.1f); // vars.w_final.elec
    DOUBLES_EQUAL(0.0f, sm.vars.capture_vals[3], 0.1f); // vars.w_final_filt.elec
#if defined(CTRL_METHOD_RFO)
    DOUBLES_EQUAL(0.0f, vars.i_cmd_int, 0.1f);
#elif defined(CTRL_METHOD_SFO)
    DOUBLES_EQUAL(0.0f, vars.T_cmd_int, 0.1f);
#endif	

    // Volt/Hz OL state
    while (sm.current == Volt_Hz_OL)
    {
        sys_sim->RunOneTick();
    }
    auto volt_hz_ol_time = sys_sim->GetDiffTime();
    CHECK_TRUE(sm.current == Speed_OL_To_CL);
    CHECK_EQUAL(ctrl.flux_weaken.activated, false);
    DOUBLES_EQUAL_PERC(params.obs.w_thresh.elec * dir, MECH_TO_ELEC(sys_sim->m_mech_load.m_w.mech, params.motor.P), 0.1f);
    DOUBLES_EQUAL_PERC(params.obs.w_thresh.elec * dir, sys_sim->m_pmsm.m_w_r.elec, 0.1f);
    DOUBLES_EQUAL_PERC(params.obs.w_thresh.elec * dir, vars.w_cmd_ext.elec, 0.01f);
    DOUBLES_EQUAL_PERC(params.obs.w_thresh.elec * dir, vars.w_cmd_int.elec, 0.01f);
    DOUBLES_EQUAL_PERC(params.obs.w_thresh.elec * dir, vars.w_final.elec, 0.1f);
    DOUBLES_EQUAL_PERC(params.obs.w_thresh.elec * dir, vars.w_final_filt.elec, 0.1f);


#if defined(CTRL_METHOD_RFO)
    DOUBLES_EQUAL(0.0f, vars.i_cmd_int, 0.1f);
#elif defined(CTRL_METHOD_SFO)
    DOUBLES_EQUAL(0.0f, vars.T_cmd_int, 0.1f);
#endif	

    // Speed_OL_To_CL state
    while (sm.current == Speed_OL_To_CL)
    {
        sys_sim->RunOneTick();
    }
    auto speed_ol_to_cl_time = sys_sim->GetDiffTime();
    CHECK_TRUE(sm.current == Speed_CL);
    CHECK_TRUE(StopWatchIsDone(&sm.vars.speed_ol_to_cl.timer));
    CHECK_TRUE(speed_ol_to_cl_time >= params.obs.lock_time);
    CHECK_EQUAL(ctrl.flux_weaken.activated, false);
    DOUBLES_EQUAL_PERC(vars.w_cmd_ext.mech, sys_sim->m_mech_load.m_w.mech, 0.1f);
    DOUBLES_EQUAL_PERC(vars.w_cmd_ext.elec, sys_sim->m_pmsm.m_w_r.elec, 0.1f);
    DOUBLES_EQUAL_PERC(vars.w_cmd_ext.elec, vars.w_cmd_int.elec, 0.01f);
    DOUBLES_EQUAL_PERC(vars.w_cmd_ext.elec, vars.w_est.elec, 0.1f);		// Verifying observer locked
    DOUBLES_EQUAL_PERC(vars.w_cmd_ext.elec, vars.w_final.elec, 0.1f);
    DOUBLES_EQUAL_PERC(vars.w_cmd_ext.elec, vars.w_final_filt.elec, 0.1f);
    ANGLE_EQUAL(sys_sim->m_pmsm.m_th_r.elec, vars.th_r_est.elec, DEG_TO_RAD(5.0f));		// Verifying observer locked
#if defined(CTRL_METHOD_SFO)
    ANGLE_EQUAL(sys_sim->m_pmsm.m_th_s.elec, vars.th_s_est.elec, DEG_TO_RAD(5.0f));		// Verifying observer locked
    ANGLE_EQUAL(sys_sim->m_pmsm.m_delta.elec, vars.delta_est.elec, DEG_TO_RAD(5.0f));		// Verifying observer locked
    DOUBLES_EQUAL(sys_sim->m_pmsm.m_la_qd_s.q, vars.la_qd_s_est.q, 0.05f * params.motor.lam);	// Verifying observer locked
    DOUBLES_EQUAL(sys_sim->m_pmsm.m_la_qd_s.d, vars.la_qd_s_est.d, 0.05f * params.motor.lam);	// Verifying observer locked
    DOUBLES_EQUAL(0.0f, sm.vars.capture_vals[4], 0.01f);	// vars.T_cmd_int
#elif defined(CTRL_METHOD_RFO)
    DOUBLES_EQUAL(0.0f, sm.vars.capture_vals[4], 0.01f);	// vars.i_cmd_int
#endif

    // Speed_CL state below base speed
    while ((sm.current == Speed_CL)
#if defined(CTRL_METHOD_RFO)
        && (ctrl.flux_weaken.id_fw == 0.0f))
#elif defined(CTRL_METHOD_SFO)
        && (ctrl.flux_weaken.la_cmd_lim >= vars.la_cmd_mtpa))
#endif
    {
        sys_sim->RunOneTick();
    }
    CHECK_TRUE(sm.current == Speed_CL);
    CHECK_EQUAL(ctrl.flux_weaken.activated, true);
    DOUBLES_EQUAL_PERC(vars.w_cmd_ext.mech, sys_sim->m_mech_load.m_w.mech, 0.05f);
    DOUBLES_EQUAL_PERC(vars.w_cmd_ext.elec, sys_sim->m_pmsm.m_w_r.elec, 0.05f);
    DOUBLES_EQUAL_PERC(vars.w_cmd_ext.elec, vars.w_cmd_int.elec, 0.01f);
    DOUBLES_EQUAL_PERC(vars.w_cmd_ext.elec, vars.w_est.elec, 0.05f);
    DOUBLES_EQUAL_PERC(vars.w_cmd_ext.elec, vars.w_final.elec, 0.05f);
    DOUBLES_EQUAL_PERC(vars.w_cmd_ext.elec, vars.w_final_filt.elec, 0.05f);
    ANGLE_EQUAL(sys_sim->m_pmsm.m_th_r.elec, vars.th_r_est.elec, DEG_TO_RAD(5.0f));
    ANGLE_EQUAL(sys_sim->m_pmsm.m_th_r.elec, vars.th_r_final.elec, DEG_TO_RAD(5.0f));
#if defined(CTRL_METHOD_SFO)
    ANGLE_EQUAL(sys_sim->m_pmsm.m_th_s.elec, vars.th_s_est.elec, DEG_TO_RAD(5.0f));
    ANGLE_EQUAL(sys_sim->m_pmsm.m_delta.elec, vars.delta_est.elec, DEG_TO_RAD(5.0f));
#endif
#if defined(CTRL_METHOD_RFO)
    DOUBLES_EQUAL(sys_sim->m_pmsm.m_i_qd_r.q, vars.i_qd_r_fb.q, 5.0f);
    DOUBLES_EQUAL(sys_sim->m_pmsm.m_i_qd_r.d, vars.i_qd_r_fb.d, 5.0f);
    DOUBLES_EQUAL(vars.i_qd_r_cmd.q, vars.i_qd_r_fb.q, 2.5f);
    DOUBLES_EQUAL(vars.i_qd_r_cmd.d, vars.i_qd_r_fb.d, 2.5f);
    DOUBLES_EQUAL(sys_sim->m_pmsm.m_la_qd_r.q, vars.la_qd_r_est.q, 0.05f * params.motor.lam);
    DOUBLES_EQUAL(sys_sim->m_pmsm.m_la_qd_r.d, vars.la_qd_r_est.d, 0.05f * params.motor.lam);
    POLAR_t i_polar_r;
    ToPolar(sys_sim->m_pmsm.m_i_qd_r.q, sys_sim->m_pmsm.m_i_qd_r.d, &i_polar_r);
    DOUBLES_EQUAL_PERC(i_polar_r.rad * dir, vars.i_cmd_int, 0.05f);
#elif defined(CTRL_METHOD_SFO)
    DOUBLES_EQUAL_PERC(vars.T_cmd_int, vars.T_est, 0.1f);
    DOUBLES_EQUAL(0.0f, vars.la_qd_s_est.q, 0.05f * params.motor.lam);
    DOUBLES_EQUAL(vars.la_cmd_final, vars.la_qd_s_est.d, 0.05f * params.motor.lam);
    DOUBLES_EQUAL(sys_sim->m_pmsm.m_la_qd_s.q, vars.la_qd_s_est.q, 0.05f * params.motor.lam);
    DOUBLES_EQUAL(sys_sim->m_pmsm.m_la_qd_s.d, vars.la_qd_s_est.d, 0.05f * params.motor.lam);
    ANGLE_EQUAL(vars.delta_cmd.elec, vars.delta_est.elec, DEG_TO_RAD(5.0f));
    ANGLE_EQUAL(sys_sim->m_pmsm.m_delta.elec, vars.delta_est.elec, DEG_TO_RAD(5.0f));
    DOUBLES_EQUAL(sys_sim->m_pmsm.m_i_qd_s.q, vars.i_qd_s_fb.q, 2.0f);
    DOUBLES_EQUAL(sys_sim->m_pmsm.m_i_qd_s.d, vars.i_qd_s_fb.d, 2.0f);
#endif
    DOUBLES_EQUAL_PERC(sys_sim->m_pmsm.m_T, vars.T_est, 0.1f);
#if defined(CTRL_METHOD_RFO)
    POLAR_t v_polar_r;
    ToPolar(vars.v_qd_r_cmd.q, vars.v_qd_r_cmd.d, &v_polar_r);
    DOUBLES_EQUAL_PERC(vars.v_dc * params.ctrl.flux_weaken.vdc_coeff, v_polar_r.rad, 0.05f);
#elif defined(CTRL_METHOD_SFO)
    POLAR_t v_polar_s;
    ToPolar(vars.v_qd_s_cmd.q, vars.v_qd_s_cmd.d, &v_polar_s);
    DOUBLES_EQUAL_PERC(vars.v_dc * params.ctrl.flux_weaken.vdc_coeff, v_polar_s.rad, 0.05f);
#endif

    // Speed_CL state above base speed: speed cmd ramp up
    sys_sim->RunUntil(sys_sim->m_pot.m_t_start + sys_sim->m_pot.m_t_slope);
    CHECK_TRUE(sm.current == Speed_CL);
    CHECK_EQUAL(ctrl.flux_weaken.activated, true);
    DOUBLES_EQUAL_PERC(vars.w_cmd_ext.mech, sys_sim->m_mech_load.m_w.mech, 0.05f);
    DOUBLES_EQUAL_PERC(vars.w_cmd_ext.elec, sys_sim->m_pmsm.m_w_r.elec, 0.05f);
    DOUBLES_EQUAL_PERC(vars.w_cmd_ext.elec, vars.w_cmd_int.elec, 0.01f);
    DOUBLES_EQUAL_PERC(vars.w_cmd_ext.elec, vars.w_est.elec, 0.05f);
    DOUBLES_EQUAL_PERC(vars.w_cmd_ext.elec, vars.w_final.elec, 0.05f);
    DOUBLES_EQUAL_PERC(vars.w_cmd_ext.elec, vars.w_final_filt.elec, 0.05f);
    ANGLE_EQUAL(sys_sim->m_pmsm.m_th_r.elec, vars.th_r_est.elec, DEG_TO_RAD(7.5f));
    ANGLE_EQUAL(sys_sim->m_pmsm.m_th_r.elec, vars.th_r_final.elec, DEG_TO_RAD(7.5f));
#if defined(CTRL_METHOD_SFO)
    ANGLE_EQUAL(sys_sim->m_pmsm.m_th_s.elec, vars.th_s_est.elec, DEG_TO_RAD(10.0f));
    ANGLE_EQUAL(sys_sim->m_pmsm.m_delta.elec, vars.delta_est.elec, DEG_TO_RAD(7.5f));
#endif
#if defined(CTRL_METHOD_RFO)
    DOUBLES_EQUAL(sys_sim->m_pmsm.m_i_qd_r.q, vars.i_qd_r_fb.q, 5.0f);
    DOUBLES_EQUAL(sys_sim->m_pmsm.m_i_qd_r.d, vars.i_qd_r_fb.d, 5.0f);
    DOUBLES_EQUAL(vars.i_qd_r_cmd.q, vars.i_qd_r_fb.q, 2.5f);
    DOUBLES_EQUAL(vars.i_qd_r_cmd.d, vars.i_qd_r_fb.d, 2.5f);
    DOUBLES_EQUAL(sys_sim->m_pmsm.m_la_qd_r.q, vars.la_qd_r_est.q, 0.05f * params.motor.lam);
    DOUBLES_EQUAL(sys_sim->m_pmsm.m_la_qd_r.d, vars.la_qd_r_est.d, 0.05f * params.motor.lam);
#elif defined(CTRL_METHOD_SFO)
    DOUBLES_EQUAL_PERC(vars.T_cmd_int, vars.T_est, 0.1f);
    DOUBLES_EQUAL(0.0f, vars.la_qd_s_est.q, 0.05f * params.motor.lam);
    DOUBLES_EQUAL(vars.la_cmd_final, vars.la_qd_s_est.d, 0.05f * params.motor.lam);
    DOUBLES_EQUAL(sys_sim->m_pmsm.m_la_qd_s.q, vars.la_qd_s_est.q, 0.05f * params.motor.lam);
    DOUBLES_EQUAL(sys_sim->m_pmsm.m_la_qd_s.d, vars.la_qd_s_est.d, 0.05f * params.motor.lam);
    ANGLE_EQUAL(vars.delta_cmd.elec, vars.delta_est.elec, DEG_TO_RAD(7.5f));
    ANGLE_EQUAL(sys_sim->m_pmsm.m_delta.elec, vars.delta_est.elec, DEG_TO_RAD(7.5f));
    DOUBLES_EQUAL(sys_sim->m_pmsm.m_i_qd_s.q, vars.i_qd_s_fb.q, 2.5f);
    DOUBLES_EQUAL(sys_sim->m_pmsm.m_i_qd_s.d, vars.i_qd_s_fb.d, 2.5f);
#endif
    DOUBLES_EQUAL_PERC(sys_sim->m_pmsm.m_T, vars.T_est, 0.1f);
#if defined(CTRL_METHOD_RFO)
    ToPolar(vars.v_qd_r_cmd.q, vars.v_qd_r_cmd.d, &v_polar_r);
    DOUBLES_EQUAL_PERC(vars.v_dc * params.ctrl.flux_weaken.vdc_coeff, v_polar_r.rad, 0.05f);
    CHECK_TRUE(vars.i_qd_r_cmd.q == vars.i_qd_r_ref.q);
    CHECK_TRUE(vars.i_qd_r_cmd.d < vars.i_qd_r_ref.d);
    CHECK_TRUE(ctrl.flux_weaken.id_fw < 0.0f);
#elif defined(CTRL_METHOD_SFO)
    ToPolar(vars.v_qd_s_cmd.q, vars.v_qd_s_cmd.d, &v_polar_s);
    DOUBLES_EQUAL_PERC(vars.v_dc * params.ctrl.flux_weaken.vdc_coeff, v_polar_s.rad, 0.075f);
    CHECK_TRUE(vars.la_cmd_final < vars.la_cmd_mtpa);
#endif

    // Speed_CL state above base speed: speed cmd steady state
    sys_sim->RunUntil(sys_sim->m_pot.m_t_stop);
    CHECK_TRUE(sm.current == Speed_CL);
    CHECK_EQUAL(ctrl.flux_weaken.activated, true);
    DOUBLES_EQUAL_PERC(vars.w_cmd_ext.mech, sys_sim->m_mech_load.m_w.mech, 0.01f);
    DOUBLES_EQUAL_PERC(vars.w_cmd_ext.elec, sys_sim->m_pmsm.m_w_r.elec, 0.01f);
    DOUBLES_EQUAL_PERC(vars.w_cmd_ext.elec, vars.w_cmd_int.elec, 0.01f);
    DOUBLES_EQUAL_PERC(vars.w_cmd_ext.elec, vars.w_est.elec, 0.01f);
    DOUBLES_EQUAL_PERC(vars.w_cmd_ext.elec, vars.w_final.elec, 0.01f);
    DOUBLES_EQUAL_PERC(vars.w_cmd_ext.elec, vars.w_final_filt.elec, 0.01f);
    ANGLE_EQUAL(sys_sim->m_pmsm.m_th_r.elec, vars.th_r_est.elec, DEG_TO_RAD(10.0f));
    ANGLE_EQUAL(sys_sim->m_pmsm.m_th_r.elec, vars.th_r_final.elec, DEG_TO_RAD(10.0f));
#if defined(CTRL_METHOD_SFO)
    ANGLE_EQUAL(sys_sim->m_pmsm.m_th_s.elec, vars.th_s_est.elec, DEG_TO_RAD(10.0f));
    ANGLE_EQUAL(sys_sim->m_pmsm.m_delta.elec, vars.delta_est.elec, DEG_TO_RAD(10.0f));
#endif
#if defined(CTRL_METHOD_RFO)
    DOUBLES_EQUAL(sys_sim->m_pmsm.m_i_qd_r.q, vars.i_qd_r_fb.q, 5.0f);
    DOUBLES_EQUAL(sys_sim->m_pmsm.m_i_qd_r.d, vars.i_qd_r_fb.d, 5.0f);
    DOUBLES_EQUAL(vars.i_qd_r_cmd.q, vars.i_qd_r_fb.q, 0.5f);
    DOUBLES_EQUAL(vars.i_qd_r_cmd.d, vars.i_qd_r_fb.d, 0.5f);
    DOUBLES_EQUAL(sys_sim->m_pmsm.m_la_qd_r.q, vars.la_qd_r_est.q, 0.05f * params.motor.lam);
    DOUBLES_EQUAL(sys_sim->m_pmsm.m_la_qd_r.d, vars.la_qd_r_est.d, 0.05f * params.motor.lam);
#elif defined(CTRL_METHOD_SFO)
    DOUBLES_EQUAL_PERC(vars.T_cmd_int, vars.T_est, 0.05f);
    DOUBLES_EQUAL(0.0f, vars.la_qd_s_est.q, 0.05f * params.motor.lam);
    DOUBLES_EQUAL(vars.la_cmd_final, vars.la_qd_s_est.d, 0.05f * params.motor.lam);
    DOUBLES_EQUAL(sys_sim->m_pmsm.m_la_qd_s.q, vars.la_qd_s_est.q, 0.05f * params.motor.lam);
    DOUBLES_EQUAL(sys_sim->m_pmsm.m_la_qd_s.d, vars.la_qd_s_est.d, 0.05f * params.motor.lam);
    ANGLE_EQUAL(vars.delta_cmd.elec, vars.delta_est.elec, DEG_TO_RAD(10.0f));
    ANGLE_EQUAL(sys_sim->m_pmsm.m_delta.elec, vars.delta_est.elec, DEG_TO_RAD(10.0f));
    DOUBLES_EQUAL(sys_sim->m_pmsm.m_i_qd_s.q, vars.i_qd_s_fb.q, 2.0f);
    DOUBLES_EQUAL(sys_sim->m_pmsm.m_i_qd_s.d, vars.i_qd_s_fb.d, 2.0f);
#endif
    DOUBLES_EQUAL_PERC(sys_sim->m_pmsm.m_T, vars.T_est, 0.1f);
#if defined(CTRL_METHOD_RFO)
    ToPolar(vars.v_qd_r_cmd.q, vars.v_qd_r_cmd.d, &v_polar_r);
    DOUBLES_EQUAL_PERC(vars.v_dc * params.ctrl.flux_weaken.vdc_coeff, v_polar_r.rad, 0.05f);
    CHECK_TRUE(vars.i_qd_r_cmd.q == vars.i_qd_r_ref.q);
    CHECK_TRUE(vars.i_qd_r_cmd.d < vars.i_qd_r_ref.d);
    CHECK_TRUE(ctrl.flux_weaken.id_fw < 0.0f);
#elif defined(CTRL_METHOD_SFO)
    ToPolar(vars.v_qd_s_cmd.q, vars.v_qd_s_cmd.d, &v_polar_s);
    DOUBLES_EQUAL_PERC(vars.v_dc * params.ctrl.flux_weaken.vdc_coeff, v_polar_s.rad, 0.075f);
    CHECK_TRUE(vars.la_cmd_final < vars.la_cmd_mtpa);
#endif

    // Speed_CL state above base speed: speed cmd ramp down
    while ((sm.current == Speed_CL)
#if defined(CTRL_METHOD_RFO)
        && (ctrl.flux_weaken.id_fw < 0.0f))
#elif defined(CTRL_METHOD_SFO)
        && (ctrl.flux_weaken.la_cmd_lim < vars.la_cmd_mtpa))
#endif
    {
        sys_sim->RunOneTick();
    }
    CHECK_TRUE(sm.current == Speed_CL);
    CHECK_EQUAL(ctrl.flux_weaken.activated, false);
    DOUBLES_EQUAL_PERC(vars.w_cmd_ext.mech, sys_sim->m_mech_load.m_w.mech, 0.05f);
    DOUBLES_EQUAL_PERC(vars.w_cmd_ext.elec, sys_sim->m_pmsm.m_w_r.elec, 0.05f);
    DOUBLES_EQUAL_PERC(vars.w_cmd_ext.elec, vars.w_cmd_int.elec, 0.01f);
    DOUBLES_EQUAL_PERC(vars.w_cmd_ext.elec, vars.w_est.elec, 0.05f);
    DOUBLES_EQUAL_PERC(vars.w_cmd_ext.elec, vars.w_final.elec, 0.05f);
    DOUBLES_EQUAL_PERC(vars.w_cmd_ext.elec, vars.w_final_filt.elec, 0.05f);
    ANGLE_EQUAL(sys_sim->m_pmsm.m_th_r.elec, vars.th_r_est.elec, DEG_TO_RAD(5.0f));
    ANGLE_EQUAL(sys_sim->m_pmsm.m_th_r.elec, vars.th_r_final.elec, DEG_TO_RAD(5.0f));
#if defined(CTRL_METHOD_SFO)
    ANGLE_EQUAL(sys_sim->m_pmsm.m_th_s.elec, vars.th_s_est.elec, DEG_TO_RAD(5.0f));
    ANGLE_EQUAL(sys_sim->m_pmsm.m_delta.elec, vars.delta_est.elec, DEG_TO_RAD(5.0f));
#endif
#if defined(CTRL_METHOD_RFO)
    DOUBLES_EQUAL(sys_sim->m_pmsm.m_i_qd_r.q, vars.i_qd_r_fb.q, 5.0f);
    DOUBLES_EQUAL(sys_sim->m_pmsm.m_i_qd_r.d, vars.i_qd_r_fb.d, 5.0f);
    DOUBLES_EQUAL(vars.i_qd_r_cmd.q, vars.i_qd_r_fb.q, 2.5f);
    DOUBLES_EQUAL(vars.i_qd_r_cmd.d, vars.i_qd_r_fb.d, 2.5f);
    DOUBLES_EQUAL(sys_sim->m_pmsm.m_la_qd_r.q, vars.la_qd_r_est.q, 0.05f * params.motor.lam);
    DOUBLES_EQUAL(sys_sim->m_pmsm.m_la_qd_r.d, vars.la_qd_r_est.d, 0.05f * params.motor.lam);
#elif defined(CTRL_METHOD_SFO)
    DOUBLES_EQUAL_PERC(vars.T_cmd_int, vars.T_est, 0.1f);
    DOUBLES_EQUAL(0.0f, vars.la_qd_s_est.q, 0.05f * params.motor.lam);
    DOUBLES_EQUAL(vars.la_cmd_final, vars.la_qd_s_est.d, 0.05f * params.motor.lam);
    DOUBLES_EQUAL(sys_sim->m_pmsm.m_la_qd_s.q, vars.la_qd_s_est.q, 0.05f * params.motor.lam);
    DOUBLES_EQUAL(sys_sim->m_pmsm.m_la_qd_s.d, vars.la_qd_s_est.d, 0.05f * params.motor.lam);
    ANGLE_EQUAL(vars.delta_cmd.elec, vars.delta_est.elec, DEG_TO_RAD(5.0f));
    ANGLE_EQUAL(sys_sim->m_pmsm.m_delta.elec, vars.delta_est.elec, DEG_TO_RAD(5.0f));
    DOUBLES_EQUAL(sys_sim->m_pmsm.m_i_qd_s.q, vars.i_qd_s_fb.q, 2.0f);
    DOUBLES_EQUAL(sys_sim->m_pmsm.m_i_qd_s.d, vars.i_qd_s_fb.d, 2.0f);
#endif
    DOUBLES_EQUAL_PERC(sys_sim->m_pmsm.m_T, vars.T_est, 0.1f);
#if defined(CTRL_METHOD_RFO)
    ToPolar(vars.v_qd_r_cmd.q, vars.v_qd_r_cmd.d, &v_polar_r);
    DOUBLES_EQUAL_PERC(vars.v_dc * params.ctrl.flux_weaken.vdc_coeff, v_polar_r.rad, 0.05f);
#elif defined(CTRL_METHOD_SFO)
    ToPolar(vars.v_qd_s_cmd.q, vars.v_qd_s_cmd.d, &v_polar_s);
    DOUBLES_EQUAL_PERC(vars.v_dc * params.ctrl.flux_weaken.vdc_coeff, v_polar_s.rad, 0.075f);
#endif

    // Speed_CL state below base speed: speed cmd ramp down
    while (sm.current == Speed_CL)
    {
        sys_sim->RunOneTick();
    }
    auto speed_cl_time = sys_sim->GetDiffTime();
    CHECK_TRUE(sm.current == Brake_Boot);
    CHECK_EQUAL(ctrl.flux_weaken.activated, false);
    float w_thresh_elec_expected = (params.obs.w_thresh.elec - params.obs.w_hyst.elec) * dir;
    DOUBLES_EQUAL_PERC(w_thresh_elec_expected, MECH_TO_ELEC(sys_sim->m_mech_load.m_w.mech, params.motor.P), 0.1f);
    DOUBLES_EQUAL_PERC(w_thresh_elec_expected, sys_sim->m_pmsm.m_w_r.elec, 0.1f);
    DOUBLES_EQUAL(0.0f, vars.w_cmd_int.elec, 0.001f);
    DOUBLES_EQUAL(0.0f, vars.w_final_filt.elec, 0.001f);
    DOUBLES_EQUAL_PERC(w_thresh_elec_expected, sm.vars.capture_vals[0], 0.1f);	// vars.w_cmd_int.elec
    DOUBLES_EQUAL_PERC(w_thresh_elec_expected, sm.vars.capture_vals[1], 0.1f);	// vars.w_est.elec
    DOUBLES_EQUAL_PERC(w_thresh_elec_expected, sm.vars.capture_vals[2], 0.1f);	// vars.w_final.elec
    DOUBLES_EQUAL_PERC(w_thresh_elec_expected, sm.vars.capture_vals[3], 0.1f);	// vars.w_final_filt.elec

    // Back to brake-boot state
    sys_sim->RunUntil(sys_sim->m_pot.m_t_stop + sys_sim->m_pot.m_t_slope);
    DOUBLES_EQUAL(0.0f, sys_sim->m_mech_load.m_w.mech, 0.1f);
    DOUBLES_EQUAL(0.0f, sys_sim->m_pmsm.m_w_r.elec, 0.1f);
    DOUBLES_EQUAL(0.0f, vars.w_cmd_int.elec, 0.1f);
    DOUBLES_EQUAL(0.0f, vars.w_final_filt.elec, 0.1f);
};


TEST(FieldWeakeningTest, PositiveSpeedCmdAboveBaseSpeed)
{
    sensor_iface.digital.dir = 1U;
    float dir = (sensor_iface.digital.dir == 1U) ? (+1.0f) : (-1.0f);

    // Turning potentiometer parameters
    sys_sim->m_pot.m_init = 0.0f;			// [%]
    sys_sim->m_pot.m_final = 1.0f;			// [%]
    sys_sim->m_pot.m_t_start = 0.5f;		// [sec]
    sys_sim->m_pot.m_t_stop = 8.5f;			// [sec]
    sys_sim->m_pot.m_t_slope = 7.5f;		// [sec]

    // Setting capture channels for capturing the variables before state transitions
    sm.vars.capture_channels[0] = &vars.w_cmd_int.elec;
    sm.vars.capture_channels[1] = &vars.w_est.elec;
    sm.vars.capture_channels[2] = &vars.w_final.elec;
    sm.vars.capture_channels[3] = &vars.w_final_filt.elec;
#if defined(CTRL_METHOD_RFO)
    sm.vars.capture_channels[4] = &vars.i_cmd_int;
#elif defined(CTRL_METHOD_SFO)
    sm.vars.capture_channels[4] = &vars.T_cmd_int;
#endif


    // Run simulation
    // Init state
    while (sm.current == Init)
    {
        sys_sim->RunOneTick();
    }
    auto init_time = sys_sim->GetDiffTime();
    CHECK_TRUE(sm.current == Brake_Boot);
    CHECK_EQUAL(sm.vars.init.offset_null_done, true);
    CHECK_EQUAL(sm.vars.init.param_init_done, true);
    CHECK_EQUAL(sm.vars.speed_reset_required, false);

    // Brake-boot state
    while (sm.current == Brake_Boot)
    {
        sys_sim->RunOneTick();
    }
    auto brake_boot_time = sys_sim->GetDiffTime();
    CHECK_TRUE(sm.current == Volt_Hz_OL);
    CHECK_EQUAL(StopWatchIsDone(&sm.vars.brake_boot.timer), true);
    CHECK_TRUE(brake_boot_time >= params.sys.boot_time);
    DOUBLES_EQUAL_PERC(params.ctrl.volt.w_thresh.elec * dir, vars.w_cmd_ext.elec, 0.01f);
    DOUBLES_EQUAL_PERC(params.ctrl.volt.w_thresh.elec * dir, vars.w_cmd_int.elec, 0.01f);
    DOUBLES_EQUAL(0.0f, sm.vars.capture_vals[0], 0.1f); // vars.w_cmd_int.elec
    DOUBLES_EQUAL(0.0f, sm.vars.capture_vals[2], 0.1f); // vars.w_final.elec
    DOUBLES_EQUAL(0.0f, sm.vars.capture_vals[3], 0.1f); // vars.w_final_filt.elec
#if defined(CTRL_METHOD_RFO)
    DOUBLES_EQUAL(0.0f, vars.i_cmd_int, 0.1f);
#elif defined(CTRL_METHOD_SFO)
    DOUBLES_EQUAL(0.0f, vars.T_cmd_int, 0.1f);
#endif	

    // Volt/Hz OL state
    while (sm.current == Volt_Hz_OL)
    {
        sys_sim->RunOneTick();
    }
    auto volt_hz_ol_time = sys_sim->GetDiffTime();
    CHECK_TRUE(sm.current == Speed_OL_To_CL);
    CHECK_EQUAL(ctrl.flux_weaken.activated, false);
    DOUBLES_EQUAL_PERC(params.obs.w_thresh.elec * dir, MECH_TO_ELEC(sys_sim->m_mech_load.m_w.mech, params.motor.P), 0.1f);
    DOUBLES_EQUAL_PERC(params.obs.w_thresh.elec * dir, sys_sim->m_pmsm.m_w_r.elec, 0.1f);
    DOUBLES_EQUAL_PERC(params.obs.w_thresh.elec * dir, vars.w_cmd_ext.elec, 0.01f);
    DOUBLES_EQUAL_PERC(params.obs.w_thresh.elec * dir, vars.w_cmd_int.elec, 0.01f);
    DOUBLES_EQUAL_PERC(params.obs.w_thresh.elec * dir, vars.w_final.elec, 0.1f);
    DOUBLES_EQUAL_PERC(params.obs.w_thresh.elec * dir, vars.w_final_filt.elec, 0.1f);
#if defined(CTRL_METHOD_RFO)
    DOUBLES_EQUAL(0.0f, vars.i_cmd_int, 0.1f);
#elif defined(CTRL_METHOD_SFO)
    DOUBLES_EQUAL(0.0f, vars.T_cmd_int, 0.1f);
#endif	

    // Speed_OL_To_CL state
    while (sm.current == Speed_OL_To_CL)
    {
        sys_sim->RunOneTick();
    }
    auto speed_ol_to_cl_time = sys_sim->GetDiffTime();
    CHECK_TRUE(sm.current == Speed_CL);
    CHECK_TRUE(StopWatchIsDone(&sm.vars.speed_ol_to_cl.timer));
    CHECK_TRUE(speed_ol_to_cl_time >= params.obs.lock_time);
    CHECK_EQUAL(ctrl.flux_weaken.activated, false);
    DOUBLES_EQUAL_PERC(vars.w_cmd_ext.mech, sys_sim->m_mech_load.m_w.mech, 0.1f);
    DOUBLES_EQUAL_PERC(vars.w_cmd_ext.elec, sys_sim->m_pmsm.m_w_r.elec, 0.1f);
    DOUBLES_EQUAL_PERC(vars.w_cmd_ext.elec, vars.w_cmd_int.elec, 0.01f);
    DOUBLES_EQUAL_PERC(vars.w_cmd_ext.elec, vars.w_est.elec, 0.1f);		// Verifying observer locked
    DOUBLES_EQUAL_PERC(vars.w_cmd_ext.elec, vars.w_final.elec, 0.1f);
    DOUBLES_EQUAL_PERC(vars.w_cmd_ext.elec, vars.w_final_filt.elec, 0.1f);
    ANGLE_EQUAL(sys_sim->m_pmsm.m_th_r.elec, vars.th_r_est.elec, DEG_TO_RAD(5.0f));		// Verifying observer locked
#if defined(CTRL_METHOD_SFO)
    ANGLE_EQUAL(sys_sim->m_pmsm.m_th_s.elec, vars.th_s_est.elec, DEG_TO_RAD(5.0f));		// Verifying observer locked
    ANGLE_EQUAL(sys_sim->m_pmsm.m_delta.elec, vars.delta_est.elec, DEG_TO_RAD(5.0f));		// Verifying observer locked
    DOUBLES_EQUAL(sys_sim->m_pmsm.m_la_qd_s.q, vars.la_qd_s_est.q, 0.05f * params.motor.lam);	// Verifying observer locked
    DOUBLES_EQUAL(sys_sim->m_pmsm.m_la_qd_s.d, vars.la_qd_s_est.d, 0.05f * params.motor.lam);	// Verifying observer locked
    DOUBLES_EQUAL(0.0f, sm.vars.capture_vals[4], 0.01f);	// vars.T_cmd_int
#elif defined(CTRL_METHOD_RFO)
    DOUBLES_EQUAL(0.0f, sm.vars.capture_vals[4], 0.01f);	// vars.i_cmd_int
#endif

    // Speed_CL state below base speed
    while ((sm.current == Speed_CL)
#if defined(CTRL_METHOD_RFO)
        && (ctrl.flux_weaken.id_fw == 0.0f))
#elif defined(CTRL_METHOD_SFO)
        && (ctrl.flux_weaken.la_cmd_lim >= vars.la_cmd_mtpa))
#endif
    {
        sys_sim->RunOneTick();
    }
    CHECK_TRUE(sm.current == Speed_CL);
    CHECK_EQUAL(ctrl.flux_weaken.activated, true);
    DOUBLES_EQUAL_PERC(vars.w_cmd_ext.mech, sys_sim->m_mech_load.m_w.mech, 0.05f);
    DOUBLES_EQUAL_PERC(vars.w_cmd_ext.elec, sys_sim->m_pmsm.m_w_r.elec, 0.05f);
    DOUBLES_EQUAL_PERC(vars.w_cmd_ext.elec, vars.w_cmd_int.elec, 0.01f);
    DOUBLES_EQUAL_PERC(vars.w_cmd_ext.elec, vars.w_est.elec, 0.05f);
    DOUBLES_EQUAL_PERC(vars.w_cmd_ext.elec, vars.w_final.elec, 0.05f);
    DOUBLES_EQUAL_PERC(vars.w_cmd_ext.elec, vars.w_final_filt.elec, 0.05f);
    ANGLE_EQUAL(sys_sim->m_pmsm.m_th_r.elec, vars.th_r_est.elec, DEG_TO_RAD(5.0f));
    ANGLE_EQUAL(sys_sim->m_pmsm.m_th_r.elec, vars.th_r_final.elec, DEG_TO_RAD(5.0f));
#if defined(CTRL_METHOD_SFO)
    ANGLE_EQUAL(sys_sim->m_pmsm.m_th_s.elec, vars.th_s_est.elec, DEG_TO_RAD(5.0f));
    ANGLE_EQUAL(sys_sim->m_pmsm.m_delta.elec, vars.delta_est.elec, DEG_TO_RAD(5.0f));
#endif
#if defined(CTRL_METHOD_RFO)
    DOUBLES_EQUAL(sys_sim->m_pmsm.m_i_qd_r.q, vars.i_qd_r_fb.q, 5.0f);
    DOUBLES_EQUAL(sys_sim->m_pmsm.m_i_qd_r.d, vars.i_qd_r_fb.d, 5.0f);
    DOUBLES_EQUAL(vars.i_qd_r_cmd.q, vars.i_qd_r_fb.q, 2.5f);
    DOUBLES_EQUAL(vars.i_qd_r_cmd.d, vars.i_qd_r_fb.d, 2.5f);
    DOUBLES_EQUAL(sys_sim->m_pmsm.m_la_qd_r.q, vars.la_qd_r_est.q, 0.05f * params.motor.lam);
    DOUBLES_EQUAL(sys_sim->m_pmsm.m_la_qd_r.d, vars.la_qd_r_est.d, 0.05f * params.motor.lam);
    POLAR_t i_polar_r;
    ToPolar(sys_sim->m_pmsm.m_i_qd_r.q, sys_sim->m_pmsm.m_i_qd_r.d, &i_polar_r);
    DOUBLES_EQUAL_PERC(i_polar_r.rad * dir, vars.i_cmd_int, 0.05f);
#elif defined(CTRL_METHOD_SFO)
    DOUBLES_EQUAL_PERC(vars.T_cmd_int, vars.T_est, 0.1f);
    DOUBLES_EQUAL(0.0f, vars.la_qd_s_est.q, 0.05f * params.motor.lam);
    DOUBLES_EQUAL(vars.la_cmd_final, vars.la_qd_s_est.d, 0.05f * params.motor.lam);
    DOUBLES_EQUAL(sys_sim->m_pmsm.m_la_qd_s.q, vars.la_qd_s_est.q, 0.05f * params.motor.lam);
    DOUBLES_EQUAL(sys_sim->m_pmsm.m_la_qd_s.d, vars.la_qd_s_est.d, 0.05f * params.motor.lam);
    ANGLE_EQUAL(vars.delta_cmd.elec, vars.delta_est.elec, DEG_TO_RAD(5.0f));
    ANGLE_EQUAL(sys_sim->m_pmsm.m_delta.elec, vars.delta_est.elec, DEG_TO_RAD(5.0f));
    DOUBLES_EQUAL(sys_sim->m_pmsm.m_i_qd_s.q, vars.i_qd_s_fb.q, 2.0f);
    DOUBLES_EQUAL(sys_sim->m_pmsm.m_i_qd_s.d, vars.i_qd_s_fb.d, 2.0f);
#endif
    DOUBLES_EQUAL_PERC(sys_sim->m_pmsm.m_T, vars.T_est, 0.1f);
#if defined(CTRL_METHOD_RFO)
    POLAR_t v_polar_r;
    ToPolar(vars.v_qd_r_cmd.q, vars.v_qd_r_cmd.d, &v_polar_r);
    DOUBLES_EQUAL_PERC(vars.v_dc * params.ctrl.flux_weaken.vdc_coeff, v_polar_r.rad, 0.05f);
#elif defined(CTRL_METHOD_SFO)
    POLAR_t v_polar_s;
    ToPolar(vars.v_qd_s_cmd.q, vars.v_qd_s_cmd.d, &v_polar_s);
    DOUBLES_EQUAL_PERC(vars.v_dc * params.ctrl.flux_weaken.vdc_coeff, v_polar_s.rad, 0.05f);
#endif

    // Speed_CL state above base speed: speed cmd ramp up
    sys_sim->RunUntil(sys_sim->m_pot.m_t_start + sys_sim->m_pot.m_t_slope);
    CHECK_TRUE(sm.current == Speed_CL);
    CHECK_EQUAL(ctrl.flux_weaken.activated, true);
    DOUBLES_EQUAL_PERC(vars.w_cmd_ext.mech, sys_sim->m_mech_load.m_w.mech, 0.05f);
    DOUBLES_EQUAL_PERC(vars.w_cmd_ext.elec, sys_sim->m_pmsm.m_w_r.elec, 0.05f);
    DOUBLES_EQUAL_PERC(vars.w_cmd_ext.elec, vars.w_cmd_int.elec, 0.01f);
    DOUBLES_EQUAL_PERC(vars.w_cmd_ext.elec, vars.w_est.elec, 0.05f);
    DOUBLES_EQUAL_PERC(vars.w_cmd_ext.elec, vars.w_final.elec, 0.05f);
    DOUBLES_EQUAL_PERC(vars.w_cmd_ext.elec, vars.w_final_filt.elec, 0.05f);
    ANGLE_EQUAL(sys_sim->m_pmsm.m_th_r.elec, vars.th_r_est.elec, DEG_TO_RAD(7.5f));
    ANGLE_EQUAL(sys_sim->m_pmsm.m_th_r.elec, vars.th_r_final.elec, DEG_TO_RAD(7.5f));
#if defined(CTRL_METHOD_SFO)
    ANGLE_EQUAL(sys_sim->m_pmsm.m_th_s.elec, vars.th_s_est.elec, DEG_TO_RAD(10.0f));
    ANGLE_EQUAL(sys_sim->m_pmsm.m_delta.elec, vars.delta_est.elec, DEG_TO_RAD(7.5f));
#endif
#if defined(CTRL_METHOD_RFO)
    DOUBLES_EQUAL(sys_sim->m_pmsm.m_i_qd_r.q, vars.i_qd_r_fb.q, 5.0f);
    DOUBLES_EQUAL(sys_sim->m_pmsm.m_i_qd_r.d, vars.i_qd_r_fb.d, 5.0f);
    DOUBLES_EQUAL(vars.i_qd_r_cmd.q, vars.i_qd_r_fb.q, 2.5f);
    DOUBLES_EQUAL(vars.i_qd_r_cmd.d, vars.i_qd_r_fb.d, 2.5f);
    DOUBLES_EQUAL(sys_sim->m_pmsm.m_la_qd_r.q, vars.la_qd_r_est.q, 0.05f * params.motor.lam);
    DOUBLES_EQUAL(sys_sim->m_pmsm.m_la_qd_r.d, vars.la_qd_r_est.d, 0.05f * params.motor.lam);
#elif defined(CTRL_METHOD_SFO)
    DOUBLES_EQUAL_PERC(vars.T_cmd_int, vars.T_est, 0.1f);
    DOUBLES_EQUAL(0.0f, vars.la_qd_s_est.q, 0.05f * params.motor.lam);
    DOUBLES_EQUAL(vars.la_cmd_final, vars.la_qd_s_est.d, 0.05f * params.motor.lam);
    DOUBLES_EQUAL(sys_sim->m_pmsm.m_la_qd_s.q, vars.la_qd_s_est.q, 0.05f * params.motor.lam);
    DOUBLES_EQUAL(sys_sim->m_pmsm.m_la_qd_s.d, vars.la_qd_s_est.d, 0.05f * params.motor.lam);
    ANGLE_EQUAL(vars.delta_cmd.elec, vars.delta_est.elec, DEG_TO_RAD(7.5f));
    ANGLE_EQUAL(sys_sim->m_pmsm.m_delta.elec, vars.delta_est.elec, DEG_TO_RAD(7.5f));
    DOUBLES_EQUAL(sys_sim->m_pmsm.m_i_qd_s.q, vars.i_qd_s_fb.q, 2.0f);
    DOUBLES_EQUAL(sys_sim->m_pmsm.m_i_qd_s.d, vars.i_qd_s_fb.d, 2.0f);
#endif
    DOUBLES_EQUAL_PERC(sys_sim->m_pmsm.m_T, vars.T_est, 0.1f);
#if defined(CTRL_METHOD_RFO)
    ToPolar(vars.v_qd_r_cmd.q, vars.v_qd_r_cmd.d, &v_polar_r);
    DOUBLES_EQUAL_PERC(vars.v_dc * params.ctrl.flux_weaken.vdc_coeff, v_polar_r.rad, 0.05f);
    CHECK_TRUE(vars.i_qd_r_cmd.q == vars.i_qd_r_ref.q);
    CHECK_TRUE(vars.i_qd_r_cmd.d < vars.i_qd_r_ref.d);
    CHECK_TRUE(ctrl.flux_weaken.id_fw < 0.0f);
#elif defined(CTRL_METHOD_SFO)
    ToPolar(vars.v_qd_s_cmd.q, vars.v_qd_s_cmd.d, &v_polar_s);
    DOUBLES_EQUAL_PERC(vars.v_dc * params.ctrl.flux_weaken.vdc_coeff, v_polar_s.rad, 0.075f);
    CHECK_TRUE(vars.la_cmd_final < vars.la_cmd_mtpa);
#endif

    // Speed_CL state above base speed: speed cmd steady state
    sys_sim->RunUntil(sys_sim->m_pot.m_t_stop);
    CHECK_TRUE(sm.current == Speed_CL);
    CHECK_EQUAL(ctrl.flux_weaken.activated, true);
    DOUBLES_EQUAL_PERC(vars.w_cmd_ext.mech, sys_sim->m_mech_load.m_w.mech, 0.01f);
    DOUBLES_EQUAL_PERC(vars.w_cmd_ext.elec, sys_sim->m_pmsm.m_w_r.elec, 0.01f);
    DOUBLES_EQUAL_PERC(vars.w_cmd_ext.elec, vars.w_cmd_int.elec, 0.01f);
    DOUBLES_EQUAL_PERC(vars.w_cmd_ext.elec, vars.w_est.elec, 0.01f);
    DOUBLES_EQUAL_PERC(vars.w_cmd_ext.elec, vars.w_final.elec, 0.01f);
    DOUBLES_EQUAL_PERC(vars.w_cmd_ext.elec, vars.w_final_filt.elec, 0.01f);
    ANGLE_EQUAL(sys_sim->m_pmsm.m_th_r.elec, vars.th_r_est.elec, DEG_TO_RAD(10.0f));
    ANGLE_EQUAL(sys_sim->m_pmsm.m_th_r.elec, vars.th_r_final.elec, DEG_TO_RAD(10.0f));
#if defined(CTRL_METHOD_SFO)
    ANGLE_EQUAL(sys_sim->m_pmsm.m_th_s.elec, vars.th_s_est.elec, DEG_TO_RAD(10.0f));
    ANGLE_EQUAL(sys_sim->m_pmsm.m_delta.elec, vars.delta_est.elec, DEG_TO_RAD(10.0f));
#endif
#if defined(CTRL_METHOD_RFO)
    DOUBLES_EQUAL(sys_sim->m_pmsm.m_i_qd_r.q, vars.i_qd_r_fb.q, 5.0f);
    DOUBLES_EQUAL(sys_sim->m_pmsm.m_i_qd_r.d, vars.i_qd_r_fb.d, 5.0f);
    DOUBLES_EQUAL(vars.i_qd_r_cmd.q, vars.i_qd_r_fb.q, 0.5f);
    DOUBLES_EQUAL(vars.i_qd_r_cmd.d, vars.i_qd_r_fb.d, 0.5f);
    DOUBLES_EQUAL(sys_sim->m_pmsm.m_la_qd_r.q, vars.la_qd_r_est.q, 0.05f * params.motor.lam);
    DOUBLES_EQUAL(sys_sim->m_pmsm.m_la_qd_r.d, vars.la_qd_r_est.d, 0.05f * params.motor.lam);
#elif defined(CTRL_METHOD_SFO)
    DOUBLES_EQUAL_PERC(vars.T_cmd_int, vars.T_est, 0.05f);
    DOUBLES_EQUAL(0.0f, vars.la_qd_s_est.q, 0.05f * params.motor.lam);
    DOUBLES_EQUAL(vars.la_cmd_final, vars.la_qd_s_est.d, 0.05f * params.motor.lam);
    DOUBLES_EQUAL(sys_sim->m_pmsm.m_la_qd_s.q, vars.la_qd_s_est.q, 0.05f * params.motor.lam);
    DOUBLES_EQUAL(sys_sim->m_pmsm.m_la_qd_s.d, vars.la_qd_s_est.d, 0.05f * params.motor.lam);
    ANGLE_EQUAL(vars.delta_cmd.elec, vars.delta_est.elec, DEG_TO_RAD(10.0f));
    ANGLE_EQUAL(sys_sim->m_pmsm.m_delta.elec, vars.delta_est.elec, DEG_TO_RAD(10.0f));
    DOUBLES_EQUAL(sys_sim->m_pmsm.m_i_qd_s.q, vars.i_qd_s_fb.q, 2.0f);
    DOUBLES_EQUAL(sys_sim->m_pmsm.m_i_qd_s.d, vars.i_qd_s_fb.d, 2.0f);
#endif
    DOUBLES_EQUAL_PERC(sys_sim->m_pmsm.m_T, vars.T_est, 0.1f);
#if defined(CTRL_METHOD_RFO)
    ToPolar(vars.v_qd_r_cmd.q, vars.v_qd_r_cmd.d, &v_polar_r);
    DOUBLES_EQUAL_PERC(vars.v_dc * params.ctrl.flux_weaken.vdc_coeff, v_polar_r.rad, 0.05f);
    CHECK_TRUE(vars.i_qd_r_cmd.q == vars.i_qd_r_ref.q);
    CHECK_TRUE(vars.i_qd_r_cmd.d < vars.i_qd_r_ref.d);
    CHECK_TRUE(ctrl.flux_weaken.id_fw < 0.0f);
#elif defined(CTRL_METHOD_SFO)
    ToPolar(vars.v_qd_s_cmd.q, vars.v_qd_s_cmd.d, &v_polar_s);
    DOUBLES_EQUAL_PERC(vars.v_dc * params.ctrl.flux_weaken.vdc_coeff, v_polar_s.rad, 0.075f);
    CHECK_TRUE(vars.la_cmd_final < vars.la_cmd_mtpa);
#endif

    // Speed_CL state above base speed: speed cmd ramp down
    while ((sm.current == Speed_CL)
#if defined(CTRL_METHOD_RFO)
        && (ctrl.flux_weaken.id_fw < 0.0f))
#elif defined(CTRL_METHOD_SFO)
        && (ctrl.flux_weaken.la_cmd_lim < vars.la_cmd_mtpa))
#endif
    {
        sys_sim->RunOneTick();
    }
    CHECK_TRUE(sm.current == Speed_CL);
    CHECK_EQUAL(ctrl.flux_weaken.activated, false);
    DOUBLES_EQUAL_PERC(vars.w_cmd_ext.mech, sys_sim->m_mech_load.m_w.mech, 0.05f);
    DOUBLES_EQUAL_PERC(vars.w_cmd_ext.elec, sys_sim->m_pmsm.m_w_r.elec, 0.05f);
    DOUBLES_EQUAL_PERC(vars.w_cmd_ext.elec, vars.w_cmd_int.elec, 0.01f);
    DOUBLES_EQUAL_PERC(vars.w_cmd_ext.elec, vars.w_est.elec, 0.05f);
    DOUBLES_EQUAL_PERC(vars.w_cmd_ext.elec, vars.w_final.elec, 0.05f);
    DOUBLES_EQUAL_PERC(vars.w_cmd_ext.elec, vars.w_final_filt.elec, 0.05f);
    ANGLE_EQUAL(sys_sim->m_pmsm.m_th_r.elec, vars.th_r_est.elec, DEG_TO_RAD(5.0f));
    ANGLE_EQUAL(sys_sim->m_pmsm.m_th_r.elec, vars.th_r_final.elec, DEG_TO_RAD(5.0f));
#if defined(CTRL_METHOD_SFO)
    ANGLE_EQUAL(sys_sim->m_pmsm.m_th_s.elec, vars.th_s_est.elec, DEG_TO_RAD(5.0f));
    ANGLE_EQUAL(sys_sim->m_pmsm.m_delta.elec, vars.delta_est.elec, DEG_TO_RAD(5.0f));
#endif
#if defined(CTRL_METHOD_RFO)
    DOUBLES_EQUAL(sys_sim->m_pmsm.m_i_qd_r.q, vars.i_qd_r_fb.q, 5.0f);
    DOUBLES_EQUAL(sys_sim->m_pmsm.m_i_qd_r.d, vars.i_qd_r_fb.d, 5.0f);
    DOUBLES_EQUAL(vars.i_qd_r_cmd.q, vars.i_qd_r_fb.q, 2.5f);
    DOUBLES_EQUAL(vars.i_qd_r_cmd.d, vars.i_qd_r_fb.d, 2.5f);
    DOUBLES_EQUAL(sys_sim->m_pmsm.m_la_qd_r.q, vars.la_qd_r_est.q, 0.05f * params.motor.lam);
    DOUBLES_EQUAL(sys_sim->m_pmsm.m_la_qd_r.d, vars.la_qd_r_est.d, 0.05f * params.motor.lam);
#elif defined(CTRL_METHOD_SFO)
    DOUBLES_EQUAL_PERC(vars.T_cmd_int, vars.T_est, 0.1f);
    DOUBLES_EQUAL(0.0f, vars.la_qd_s_est.q, 0.05f * params.motor.lam);
    DOUBLES_EQUAL(vars.la_cmd_final, vars.la_qd_s_est.d, 0.05f * params.motor.lam);
    DOUBLES_EQUAL(sys_sim->m_pmsm.m_la_qd_s.q, vars.la_qd_s_est.q, 0.05f * params.motor.lam);
    DOUBLES_EQUAL(sys_sim->m_pmsm.m_la_qd_s.d, vars.la_qd_s_est.d, 0.05f * params.motor.lam);
    ANGLE_EQUAL(vars.delta_cmd.elec, vars.delta_est.elec, DEG_TO_RAD(5.0f));
    ANGLE_EQUAL(sys_sim->m_pmsm.m_delta.elec, vars.delta_est.elec, DEG_TO_RAD(5.0f));
    DOUBLES_EQUAL(sys_sim->m_pmsm.m_i_qd_s.q, vars.i_qd_s_fb.q, 2.0f);
    DOUBLES_EQUAL(sys_sim->m_pmsm.m_i_qd_s.d, vars.i_qd_s_fb.d, 2.0f);
#endif
    DOUBLES_EQUAL_PERC(sys_sim->m_pmsm.m_T, vars.T_est, 0.1f);
#if defined(CTRL_METHOD_RFO)
    ToPolar(vars.v_qd_r_cmd.q, vars.v_qd_r_cmd.d, &v_polar_r);
    DOUBLES_EQUAL_PERC(vars.v_dc * params.ctrl.flux_weaken.vdc_coeff, v_polar_r.rad, 0.05f);
#elif defined(CTRL_METHOD_SFO)
    ToPolar(vars.v_qd_s_cmd.q, vars.v_qd_s_cmd.d, &v_polar_s);
    DOUBLES_EQUAL_PERC(vars.v_dc * params.ctrl.flux_weaken.vdc_coeff, v_polar_s.rad, 0.075f);
#endif

    // Speed_CL state below base speed: speed cmd ramp down
    while (sm.current == Speed_CL)
    {
        sys_sim->RunOneTick();
    }
    auto speed_cl_time = sys_sim->GetDiffTime();
    CHECK_TRUE(sm.current == Brake_Boot);
    CHECK_EQUAL(ctrl.flux_weaken.activated, false);
    float w_thresh_elec_expected = (params.obs.w_thresh.elec - params.obs.w_hyst.elec) * dir;
    DOUBLES_EQUAL_PERC(w_thresh_elec_expected, MECH_TO_ELEC(sys_sim->m_mech_load.m_w.mech, params.motor.P), 0.1f);
    DOUBLES_EQUAL_PERC(w_thresh_elec_expected, sys_sim->m_pmsm.m_w_r.elec, 0.1f);
    DOUBLES_EQUAL(0.0f, vars.w_cmd_int.elec, 0.001f);
    DOUBLES_EQUAL(0.0f, vars.w_final_filt.elec, 0.001f);
    DOUBLES_EQUAL_PERC(w_thresh_elec_expected, sm.vars.capture_vals[0], 0.1f);	// vars.w_cmd_int.elec
    DOUBLES_EQUAL_PERC(w_thresh_elec_expected, sm.vars.capture_vals[1], 0.1f);	// vars.w_est.elec
    DOUBLES_EQUAL_PERC(w_thresh_elec_expected, sm.vars.capture_vals[2], 0.1f);	// vars.w_final.elec
    DOUBLES_EQUAL_PERC(w_thresh_elec_expected, sm.vars.capture_vals[3], 0.1f);	// vars.w_final_filt.elec

    // Back to brake-boot state
    sys_sim->RunUntil(sys_sim->m_pot.m_t_stop + sys_sim->m_pot.m_t_slope);
    DOUBLES_EQUAL(0.0f, sys_sim->m_mech_load.m_w.mech, 0.1f);
    DOUBLES_EQUAL(0.0f, sys_sim->m_pmsm.m_w_r.elec, 0.1f);
    DOUBLES_EQUAL(0.0f, vars.w_cmd_int.elec, 0.1f);
    DOUBLES_EQUAL(0.0f, vars.w_final_filt.elec, 0.1f);
};

#endif