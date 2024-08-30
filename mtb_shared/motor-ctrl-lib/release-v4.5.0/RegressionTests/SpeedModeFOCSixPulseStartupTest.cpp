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

TEST_GROUP(SpeedModeFOCSixPulseStartupTest)
{
    CSysSim* sys_sim;

    const ELEC_t Initial_Angle = { DEG_TO_RAD(20.0f) }; // [Ra-elec]

    void setup()
    {
        sys_sim = new CSysSim();

        // Enable modeling of motor inducatance saturation (lq/ld)
        // for testing initial position (six pulse and high frequency injection):
        const QD_t I_QD_R_Sat = { params.motor.i_peak * 4.0f, params.motor.i_peak * 4.0f };
        sys_sim->m_pmsm.SetSaturationParams(En, I_QD_R_Sat);

        sys_sim->m_mech_load.Reset({ 0.0f }, { ELEC_TO_MECH(Initial_Angle.elec, params.motor.P) }); // to test rotor pre-alignment before spinning

        params.ctrl.mode = Speed_Mode_FOC_Sensorless_SixPulse_Startup; // [#]
        params.obs.w_thresh.elec = params.motor.w_nom.elec * 0.05f; // [Ra/sec-elec]
        PARAMS_DEFAULT_InitAutoCalc();
        STATE_MACHINE_Init();
    }

    void teardown()
    {
        delete sys_sim;
    }

};

TEST(SpeedModeFOCSixPulseStartupTest, ClosedLoopPositiveSpeedCmd)
{
    sensor_iface.digital.dir = 1U;
    float dir = (sensor_iface.digital.dir == 1U) ? (+1.0f) : (-1.0f);

    // Setting capture channels for capturing the variables before state transitions
    sm.vars.capture_channels[0] = &vars.w_cmd_int.elec;
    sm.vars.capture_channels[1] = &vars.w_est.elec;
    sm.vars.capture_channels[2] = &vars.w_final.elec;
    sm.vars.capture_channels[3] = &vars.w_final_filt.elec;

    // Turning potentiometer parameters
    sys_sim->m_pot.m_init = 0.0f;		// [%]
    sys_sim->m_pot.m_final = 1.0f;		// [%]
    sys_sim->m_pot.m_t_start = 0.5f;	// [sec]
    sys_sim->m_pot.m_t_stop = 3.5f;		// [sec]
    sys_sim->m_pot.m_t_slope = 2.5f;	// [sec]

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
    CHECK_TRUE(sm.current == Six_Pulse);
    CHECK_EQUAL(StopWatchIsDone(&sm.vars.brake_boot.timer), true);
    CHECK_TRUE(brake_boot_time >= params.sys.boot_time);
    DOUBLES_EQUAL_PERC(params.ctrl.volt.w_thresh.elec * dir, vars.w_cmd_ext.elec, 0.01f);
    DOUBLES_EQUAL(0.0f, vars.w_cmd_int.elec, 0.01f);

    // Six Pulse Injection state
    while (sm.current == Six_Pulse)
    {
        sys_sim->RunOneTick();
    }
    auto six_pulse_time = sys_sim->GetDiffTime();
    CHECK_TRUE(six_pulse_time > (6.0f * (params.ctrl.six_pulse_inj.t_on + params.ctrl.six_pulse_inj.t_off))); // speed cmd should go above the threshold
    CHECK_TRUE(sm.current == Speed_CL);
    CHECK_TRUE(ctrl.six_pulse_inj.state == Finished_Success);
    // Verifying six pulse injection didn't spin the motor:
    ANGLE_EQUAL(Initial_Angle.elec, sys_sim->m_pmsm.m_th_r.elec, DEG_TO_RAD(5.00f));
    // Actual motor angle should be withing +/-15 degrees of estimated angle:
    ANGLE_EQUAL(sys_sim->m_pmsm.m_th_r.elec, vars.th_r_est.elec, DEG_TO_RAD(15.01f));
    DOUBLES_EQUAL(0.0f, sys_sim->m_pmsm.m_w_r.elec, 10.0f); // transients settled
    DOUBLES_EQUAL(0.0f, sys_sim->m_pmsm.m_i_qd_r.q, 1.0f); // transients settled
    DOUBLES_EQUAL(0.0f, sys_sim->m_pmsm.m_i_qd_r.d, 1.0f); // transients settled
    DOUBLES_EQUAL(0.0f, sm.vars.capture_vals[0], 0.01f); // vars.w_cmd_int.elec
    DOUBLES_EQUAL_PERC(params.obs.w_thresh.elec * dir, vars.w_cmd_int.elec, 0.01f);

    // Speed-CL state: ramp up
    sys_sim->RunUntil(sys_sim->m_pot.m_t_start + sys_sim->m_pot.m_t_slope);
    DOUBLES_EQUAL_PERC(vars.w_cmd_ext.mech, sys_sim->m_mech_load.m_w.mech, 0.05f);
    DOUBLES_EQUAL_PERC(vars.w_cmd_ext.elec, sys_sim->m_pmsm.m_w_r.elec, 0.05f);
    DOUBLES_EQUAL_PERC(vars.w_cmd_ext.elec, vars.w_cmd_int.elec, 0.01f);
    DOUBLES_EQUAL_PERC(vars.w_cmd_ext.elec, vars.w_est.elec, 0.05f);
    DOUBLES_EQUAL_PERC(vars.w_cmd_ext.elec, vars.w_final.elec, 0.05f);
    DOUBLES_EQUAL_PERC(vars.w_cmd_ext.elec, vars.w_final_filt.elec, 0.05f);
    ANGLE_EQUAL(Wrap2Pi(MECH_TO_ELEC(sys_sim->m_mech_load.m_th.mech, params.motor.P)), sys_sim->m_pmsm.m_th_r.elec, DEG_TO_RAD(2.5f));
    ANGLE_EQUAL(sys_sim->m_pmsm.m_th_r.elec, vars.th_r_fb.elec, DEG_TO_RAD(0.5f));
    ANGLE_EQUAL(sys_sim->m_pmsm.m_th_r.elec, vars.th_r_est.elec, DEG_TO_RAD(5.0f));
    ANGLE_EQUAL(sys_sim->m_pmsm.m_th_r.elec, vars.th_r_final.elec, DEG_TO_RAD(5.0f));
#if defined(CTRL_METHOD_SFO)
    ANGLE_EQUAL(sys_sim->m_pmsm.m_th_s.elec, vars.th_s_est.elec, DEG_TO_RAD(5.0f));
    ANGLE_EQUAL(sys_sim->m_pmsm.m_delta.elec, vars.delta_est.elec, DEG_TO_RAD(5.0f));
#endif
#if defined(CTRL_METHOD_RFO)
    DOUBLES_EQUAL_PERC(vars.i_cmd_int, CReferenceFrame(0.0f, sys_sim->m_pmsm.m_i_qd_r).GetPolar().rad * dir, 0.05f);
    DOUBLES_EQUAL_PERC(vars.i_cmd_int, CReferenceFrame(0.0f, vars.i_qd_r_cmd).GetPolar().rad * dir, 0.05f);
    DOUBLES_EQUAL(sys_sim->m_pmsm.m_i_qd_r.q, vars.i_qd_r_fb.q, 5.0f);
    DOUBLES_EQUAL(sys_sim->m_pmsm.m_i_qd_r.d, vars.i_qd_r_fb.d, 5.0f);
    DOUBLES_EQUAL(vars.i_qd_r_cmd.q, vars.i_qd_r_fb.q, 2.5f);
    DOUBLES_EQUAL(vars.i_qd_r_cmd.d, vars.i_qd_r_fb.d, 2.5f);
    DOUBLES_EQUAL_PERC(sys_sim->m_pmsm.m_la_qd_r.q, vars.la_qd_r_est.q, 0.15f);
    DOUBLES_EQUAL_PERC(sys_sim->m_pmsm.m_la_qd_r.d, vars.la_qd_r_est.d, 0.15f);
#elif defined(CTRL_METHOD_SFO)
    DOUBLES_EQUAL_PERC(vars.T_cmd_int, vars.T_est, 0.1f);
    DOUBLES_EQUAL(0.0f, vars.la_qd_s_est.q, 0.01f);
    DOUBLES_EQUAL_PERC(vars.la_cmd_final, vars.la_qd_s_est.d, 0.1f);
    DOUBLES_EQUAL_PERC(sys_sim->m_pmsm.m_la_qd_s.q, vars.la_qd_s_est.q, 0.1f);
    DOUBLES_EQUAL_PERC(sys_sim->m_pmsm.m_la_qd_s.d, vars.la_qd_s_est.d, 0.1f);
    ANGLE_EQUAL(vars.delta_cmd.elec, vars.delta_est.elec, DEG_TO_RAD(5.0f));
    ANGLE_EQUAL(sys_sim->m_pmsm.m_delta.elec, vars.delta_est.elec, DEG_TO_RAD(5.0f));
    DOUBLES_EQUAL_PERC(sys_sim->m_pmsm.m_i_qd_s.q, vars.i_qd_s_fb.q, 0.1f);
    DOUBLES_EQUAL_PERC(sys_sim->m_pmsm.m_i_qd_s.d, vars.i_qd_s_fb.d, 0.1f);
#endif
    DOUBLES_EQUAL_PERC(sys_sim->m_pmsm.m_T, vars.T_est, 0.1f);

    // Speed-CL state: steady-state
    sys_sim->RunUntil(sys_sim->m_pot.m_t_stop);
    CHECK_TRUE(sm.current == Speed_CL);
    DOUBLES_EQUAL_PERC(vars.w_cmd_ext.mech, sys_sim->m_mech_load.m_w.mech, 0.01f);
    DOUBLES_EQUAL_PERC(vars.w_cmd_ext.elec, sys_sim->m_pmsm.m_w_r.elec, 0.01f);
    DOUBLES_EQUAL_PERC(vars.w_cmd_ext.elec, vars.w_cmd_int.elec, 0.01f);
    DOUBLES_EQUAL_PERC(vars.w_cmd_ext.elec, vars.w_est.elec, 0.01f);
    DOUBLES_EQUAL_PERC(vars.w_cmd_ext.elec, vars.w_final.elec, 0.01f);
    DOUBLES_EQUAL_PERC(vars.w_cmd_ext.elec, vars.w_final_filt.elec, 0.01f);
    ANGLE_EQUAL(Wrap2Pi(MECH_TO_ELEC(sys_sim->m_mech_load.m_th.mech, params.motor.P)), sys_sim->m_pmsm.m_th_r.elec, DEG_TO_RAD(2.5f));
    ANGLE_EQUAL(sys_sim->m_pmsm.m_th_r.elec, vars.th_r_fb.elec, DEG_TO_RAD(0.5f));
    ANGLE_EQUAL(sys_sim->m_pmsm.m_th_r.elec, vars.th_r_est.elec, DEG_TO_RAD(5.0f));
    ANGLE_EQUAL(sys_sim->m_pmsm.m_th_r.elec, vars.th_r_final.elec, DEG_TO_RAD(5.0f));
#if defined(CTRL_METHOD_SFO)
    ANGLE_EQUAL(sys_sim->m_pmsm.m_th_s.elec, vars.th_s_est.elec, DEG_TO_RAD(5.0f));
    ANGLE_EQUAL(sys_sim->m_pmsm.m_delta.elec, vars.delta_est.elec, DEG_TO_RAD(5.0f));
#endif
#if defined(CTRL_METHOD_RFO)
    DOUBLES_EQUAL_PERC(vars.i_cmd_int, CReferenceFrame(0.0f, sys_sim->m_pmsm.m_i_qd_r).GetPolar().rad * dir, 0.05f);
    DOUBLES_EQUAL_PERC(vars.i_cmd_int, CReferenceFrame(0.0f, vars.i_qd_r_cmd).GetPolar().rad * dir, 0.05f);
    DOUBLES_EQUAL(sys_sim->m_pmsm.m_i_qd_r.q, vars.i_qd_r_fb.q, 5.0f);
    DOUBLES_EQUAL(sys_sim->m_pmsm.m_i_qd_r.d, vars.i_qd_r_fb.d, 5.0f);
    DOUBLES_EQUAL(vars.i_qd_r_cmd.q, vars.i_qd_r_fb.q, 0.5f);
    DOUBLES_EQUAL(vars.i_qd_r_cmd.d, vars.i_qd_r_fb.d, 0.5f);
    DOUBLES_EQUAL_PERC(sys_sim->m_pmsm.m_la_qd_r.q, vars.la_qd_r_est.q, 0.15f);
    DOUBLES_EQUAL_PERC(sys_sim->m_pmsm.m_la_qd_r.d, vars.la_qd_r_est.d, 0.15f);
#elif defined(CTRL_METHOD_SFO)
    DOUBLES_EQUAL_PERC(vars.T_cmd_int, vars.T_est, 0.05f);
    DOUBLES_EQUAL(0.0f, vars.la_qd_s_est.q, 0.01f);
    DOUBLES_EQUAL_PERC(vars.la_cmd_final, vars.la_qd_s_est.d, 0.05f);
    DOUBLES_EQUAL_PERC(sys_sim->m_pmsm.m_la_qd_s.q, vars.la_qd_s_est.q, 0.05f);
    DOUBLES_EQUAL_PERC(sys_sim->m_pmsm.m_la_qd_s.d, vars.la_qd_s_est.d, 0.05f);
    ANGLE_EQUAL(vars.delta_cmd.elec, vars.delta_est.elec, DEG_TO_RAD(5.0f));
    ANGLE_EQUAL(sys_sim->m_pmsm.m_delta.elec, vars.delta_est.elec, DEG_TO_RAD(5.0f));
    DOUBLES_EQUAL_PERC(sys_sim->m_pmsm.m_i_qd_s.q, vars.i_qd_s_fb.q, 0.05f);
    DOUBLES_EQUAL_PERC(sys_sim->m_pmsm.m_i_qd_s.d, vars.i_qd_s_fb.d, 0.05f);
#endif
    DOUBLES_EQUAL_PERC(sys_sim->m_pmsm.m_T, vars.T_est, 0.1f);

    // Speed-CL state: steady-state: ramp down
    while (sm.current == Speed_CL)
    {
        sys_sim->RunOneTick();
    }
    auto speed_cl_time = sys_sim->GetDiffTime();
    CHECK_TRUE(sm.current == Brake_Boot);
    CHECK_TRUE(sm.vars.speed_reset_required == true);
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
    CHECK_TRUE(sm.vars.speed_reset_required == false);
    DOUBLES_EQUAL(0.0f, sys_sim->m_mech_load.m_w.mech, 0.1f);
    DOUBLES_EQUAL(0.0f, sys_sim->m_pmsm.m_w_r.elec, 0.1f);
    DOUBLES_EQUAL(0.0f, vars.w_cmd_int.elec, 0.1f);
    DOUBLES_EQUAL(0.0f, vars.w_final_filt.elec, 0.1f);
}


TEST(SpeedModeFOCSixPulseStartupTest, ClosedLoopNegativeSpeedCmd)
{
    sensor_iface.digital.dir = 0U;
    float dir = (sensor_iface.digital.dir == 1U) ? (+1.0f) : (-1.0f);

    // Setting capture channels for capturing the variables before state transitions
    sm.vars.capture_channels[0] = &vars.w_cmd_int.elec;
    sm.vars.capture_channels[1] = &vars.w_est.elec;
    sm.vars.capture_channels[2] = &vars.w_final.elec;
    sm.vars.capture_channels[3] = &vars.w_final_filt.elec;

    // Turning potentiometer parameters
    sys_sim->m_pot.m_init = 0.0f;		// [%]
    sys_sim->m_pot.m_final = 1.0f;		// [%]
    sys_sim->m_pot.m_t_start = 0.5f;	// [sec]
    sys_sim->m_pot.m_t_stop = 3.5f;		// [sec]
    sys_sim->m_pot.m_t_slope = 2.5f;	// [sec]

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
    CHECK_TRUE(sm.current == Six_Pulse);
    CHECK_EQUAL(StopWatchIsDone(&sm.vars.brake_boot.timer), true);
    CHECK_TRUE(brake_boot_time >= params.sys.boot_time);
    DOUBLES_EQUAL_PERC(params.ctrl.volt.w_thresh.elec * dir, vars.w_cmd_ext.elec, 0.01f);
    DOUBLES_EQUAL(0.0f, vars.w_cmd_int.elec, 0.01f);

    // Six Pulse Injection state
    while (sm.current == Six_Pulse)
    {
        sys_sim->RunOneTick();
    }
    auto six_pulse_time = sys_sim->GetDiffTime();
    CHECK_TRUE(six_pulse_time > (6.0f * (params.ctrl.six_pulse_inj.t_on + params.ctrl.six_pulse_inj.t_off))); // speed cmd should go above the threshold
    CHECK_TRUE(sm.current == Speed_CL);
    CHECK_TRUE(ctrl.six_pulse_inj.state == Finished_Success);
    // Verifying six pulse injection didn't spin the motor:
    ANGLE_EQUAL(Initial_Angle.elec, sys_sim->m_pmsm.m_th_r.elec, DEG_TO_RAD(5.00f));
    // Actual motor angle should be withing +/-15 degrees of estimated angle:
    ANGLE_EQUAL(sys_sim->m_pmsm.m_th_r.elec, vars.th_r_est.elec, DEG_TO_RAD(15.01f));
    DOUBLES_EQUAL(0.0f, sys_sim->m_pmsm.m_w_r.elec, 10.0f); // transients settled
    DOUBLES_EQUAL(0.0f, sys_sim->m_pmsm.m_i_qd_r.q, 1.0f); // transients settled
    DOUBLES_EQUAL(0.0f, sys_sim->m_pmsm.m_i_qd_r.d, 1.0f); // transients settled
    DOUBLES_EQUAL(0.0f, sm.vars.capture_vals[0], 0.01f); // vars.w_cmd_int.elec
    DOUBLES_EQUAL_PERC(params.obs.w_thresh.elec * dir, vars.w_cmd_int.elec, 0.01f);

    // Speed-CL state: ramp up
    sys_sim->RunUntil(sys_sim->m_pot.m_t_start + sys_sim->m_pot.m_t_slope);
    DOUBLES_EQUAL_PERC(vars.w_cmd_ext.mech, sys_sim->m_mech_load.m_w.mech, 0.05f);
    DOUBLES_EQUAL_PERC(vars.w_cmd_ext.elec, sys_sim->m_pmsm.m_w_r.elec, 0.05f);
    DOUBLES_EQUAL_PERC(vars.w_cmd_ext.elec, vars.w_cmd_int.elec, 0.01f);
    DOUBLES_EQUAL_PERC(vars.w_cmd_ext.elec, vars.w_est.elec, 0.05f);
    DOUBLES_EQUAL_PERC(vars.w_cmd_ext.elec, vars.w_final.elec, 0.05f);
    DOUBLES_EQUAL_PERC(vars.w_cmd_ext.elec, vars.w_final_filt.elec, 0.05f);
    ANGLE_EQUAL(Wrap2Pi(MECH_TO_ELEC(sys_sim->m_mech_load.m_th.mech, params.motor.P)), sys_sim->m_pmsm.m_th_r.elec, DEG_TO_RAD(2.5f));
    ANGLE_EQUAL(sys_sim->m_pmsm.m_th_r.elec, vars.th_r_fb.elec, DEG_TO_RAD(0.5f));
    ANGLE_EQUAL(sys_sim->m_pmsm.m_th_r.elec, vars.th_r_est.elec, DEG_TO_RAD(5.0f));
    ANGLE_EQUAL(sys_sim->m_pmsm.m_th_r.elec, vars.th_r_final.elec, DEG_TO_RAD(5.0f));
#if defined(CTRL_METHOD_SFO)
    ANGLE_EQUAL(sys_sim->m_pmsm.m_th_s.elec, vars.th_s_est.elec, DEG_TO_RAD(5.0f));
    ANGLE_EQUAL(sys_sim->m_pmsm.m_delta.elec, vars.delta_est.elec, DEG_TO_RAD(5.0f));
#endif
#if defined(CTRL_METHOD_RFO)
    DOUBLES_EQUAL_PERC(vars.i_cmd_int, CReferenceFrame(0.0f, sys_sim->m_pmsm.m_i_qd_r).GetPolar().rad * dir, 0.05f);
    DOUBLES_EQUAL_PERC(vars.i_cmd_int, CReferenceFrame(0.0f, vars.i_qd_r_cmd).GetPolar().rad * dir, 0.05f);
    DOUBLES_EQUAL(sys_sim->m_pmsm.m_i_qd_r.q, vars.i_qd_r_fb.q, 5.0f);
    DOUBLES_EQUAL(sys_sim->m_pmsm.m_i_qd_r.d, vars.i_qd_r_fb.d, 5.0f);
    DOUBLES_EQUAL(vars.i_qd_r_cmd.q, vars.i_qd_r_fb.q, 2.5f);
    DOUBLES_EQUAL(vars.i_qd_r_cmd.d, vars.i_qd_r_fb.d, 2.5f);
    DOUBLES_EQUAL_PERC(sys_sim->m_pmsm.m_la_qd_r.q, vars.la_qd_r_est.q, 0.15f);
    DOUBLES_EQUAL_PERC(sys_sim->m_pmsm.m_la_qd_r.d, vars.la_qd_r_est.d, 0.15f);
#elif defined(CTRL_METHOD_SFO)
    DOUBLES_EQUAL_PERC(vars.T_cmd_int, vars.T_est, 0.1f);
    DOUBLES_EQUAL(0.0f, vars.la_qd_s_est.q, 0.01f);
    DOUBLES_EQUAL_PERC(vars.la_cmd_final, vars.la_qd_s_est.d, 0.1f);
    DOUBLES_EQUAL_PERC(sys_sim->m_pmsm.m_la_qd_s.q, vars.la_qd_s_est.q, 0.1f);
    DOUBLES_EQUAL_PERC(sys_sim->m_pmsm.m_la_qd_s.d, vars.la_qd_s_est.d, 0.1f);
    ANGLE_EQUAL(vars.delta_cmd.elec, vars.delta_est.elec, DEG_TO_RAD(5.0f));
    ANGLE_EQUAL(sys_sim->m_pmsm.m_delta.elec, vars.delta_est.elec, DEG_TO_RAD(5.0f));
    DOUBLES_EQUAL_PERC(sys_sim->m_pmsm.m_i_qd_s.q, vars.i_qd_s_fb.q, 0.1f);
    DOUBLES_EQUAL_PERC(sys_sim->m_pmsm.m_i_qd_s.d, vars.i_qd_s_fb.d, 0.1f);
#endif
    DOUBLES_EQUAL_PERC(sys_sim->m_pmsm.m_T, vars.T_est, 0.1f);

    // Speed-CL state: steady-state
    sys_sim->RunUntil(sys_sim->m_pot.m_t_stop);
    CHECK_TRUE(sm.current == Speed_CL);
    DOUBLES_EQUAL_PERC(vars.w_cmd_ext.mech, sys_sim->m_mech_load.m_w.mech, 0.01f);
    DOUBLES_EQUAL_PERC(vars.w_cmd_ext.elec, sys_sim->m_pmsm.m_w_r.elec, 0.01f);
    DOUBLES_EQUAL_PERC(vars.w_cmd_ext.elec, vars.w_cmd_int.elec, 0.01f);
    DOUBLES_EQUAL_PERC(vars.w_cmd_ext.elec, vars.w_est.elec, 0.01f);
    DOUBLES_EQUAL_PERC(vars.w_cmd_ext.elec, vars.w_final.elec, 0.01f);
    DOUBLES_EQUAL_PERC(vars.w_cmd_ext.elec, vars.w_final_filt.elec, 0.01f);
    ANGLE_EQUAL(Wrap2Pi(MECH_TO_ELEC(sys_sim->m_mech_load.m_th.mech, params.motor.P)), sys_sim->m_pmsm.m_th_r.elec, DEG_TO_RAD(2.5f));
    ANGLE_EQUAL(sys_sim->m_pmsm.m_th_r.elec, vars.th_r_fb.elec, DEG_TO_RAD(0.5f));
    ANGLE_EQUAL(sys_sim->m_pmsm.m_th_r.elec, vars.th_r_est.elec, DEG_TO_RAD(5.0f));
    ANGLE_EQUAL(sys_sim->m_pmsm.m_th_r.elec, vars.th_r_final.elec, DEG_TO_RAD(5.0f));
#if defined(CTRL_METHOD_SFO)
    ANGLE_EQUAL(sys_sim->m_pmsm.m_th_s.elec, vars.th_s_est.elec, DEG_TO_RAD(5.0f));
    ANGLE_EQUAL(sys_sim->m_pmsm.m_delta.elec, vars.delta_est.elec, DEG_TO_RAD(5.0f));
#endif
#if defined(CTRL_METHOD_RFO)
    DOUBLES_EQUAL_PERC(vars.i_cmd_int, CReferenceFrame(0.0f, sys_sim->m_pmsm.m_i_qd_r).GetPolar().rad * dir, 0.05f);
    DOUBLES_EQUAL_PERC(vars.i_cmd_int, CReferenceFrame(0.0f, vars.i_qd_r_cmd).GetPolar().rad * dir, 0.05f);
    DOUBLES_EQUAL(sys_sim->m_pmsm.m_i_qd_r.q, vars.i_qd_r_fb.q, 5.0f);
    DOUBLES_EQUAL(sys_sim->m_pmsm.m_i_qd_r.d, vars.i_qd_r_fb.d, 5.0f);
    DOUBLES_EQUAL(vars.i_qd_r_cmd.q, vars.i_qd_r_fb.q, 0.5f);
    DOUBLES_EQUAL(vars.i_qd_r_cmd.d, vars.i_qd_r_fb.d, 0.5f);
    DOUBLES_EQUAL_PERC(sys_sim->m_pmsm.m_la_qd_r.q, vars.la_qd_r_est.q, 0.15f);
    DOUBLES_EQUAL_PERC(sys_sim->m_pmsm.m_la_qd_r.d, vars.la_qd_r_est.d, 0.15f);
#elif defined(CTRL_METHOD_SFO)
    DOUBLES_EQUAL_PERC(vars.T_cmd_int, vars.T_est, 0.05f);
    DOUBLES_EQUAL(0.0f, vars.la_qd_s_est.q, 0.01f);
    DOUBLES_EQUAL_PERC(vars.la_cmd_final, vars.la_qd_s_est.d, 0.05f);
    DOUBLES_EQUAL_PERC(sys_sim->m_pmsm.m_la_qd_s.q, vars.la_qd_s_est.q, 0.05f);
    DOUBLES_EQUAL_PERC(sys_sim->m_pmsm.m_la_qd_s.d, vars.la_qd_s_est.d, 0.05f);
    ANGLE_EQUAL(vars.delta_cmd.elec, vars.delta_est.elec, DEG_TO_RAD(5.0f));
    ANGLE_EQUAL(sys_sim->m_pmsm.m_delta.elec, vars.delta_est.elec, DEG_TO_RAD(5.0f));
    DOUBLES_EQUAL_PERC(sys_sim->m_pmsm.m_i_qd_s.q, vars.i_qd_s_fb.q, 0.05f);
    DOUBLES_EQUAL_PERC(sys_sim->m_pmsm.m_i_qd_s.d, vars.i_qd_s_fb.d, 0.05f);
#endif
    DOUBLES_EQUAL_PERC(sys_sim->m_pmsm.m_T, vars.T_est, 0.1f);

    // Speed-CL state: steady-state: ramp down
    while (sm.current == Speed_CL)
    {
        sys_sim->RunOneTick();
    }
    auto speed_cl_time = sys_sim->GetDiffTime();
    CHECK_TRUE(sm.current == Brake_Boot);
    CHECK_TRUE(sm.vars.speed_reset_required == true);
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
    CHECK_TRUE(sm.vars.speed_reset_required == false);
    DOUBLES_EQUAL(0.0f, sys_sim->m_mech_load.m_w.mech, 0.1f);
    DOUBLES_EQUAL(0.0f, sys_sim->m_pmsm.m_w_r.elec, 0.1f);
    DOUBLES_EQUAL(0.0f, vars.w_cmd_int.elec, 0.1f);
    DOUBLES_EQUAL(0.0f, vars.w_final_filt.elec, 0.1f);
}

#endif