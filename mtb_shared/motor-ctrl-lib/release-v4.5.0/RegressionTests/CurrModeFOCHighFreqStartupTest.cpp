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


#if defined(CTRL_METHOD_RFO)	// Current control only available in RFO

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

TEST_GROUP(CurrModeFOCHighFreqStartupTest)
{
    CSysSim* sys_sim;

    const ELEC_t Initial_Angle = { DEG_TO_RAD(-95.0f) }; // [Ra-elec]

    void setup()
    {
        sys_sim = new CSysSim();

        // Enable modeling of motor inducatance saturation (lq/ld)
        // for testing initial position (six pulse and high frequency injection):
        const QD_t I_QD_R_Sat = { params.motor.i_peak * 16.0f, params.motor.i_peak * 16.0f };
        sys_sim->m_pmsm.SetSaturationParams(En, I_QD_R_Sat);

        sys_sim->m_mech_load.Reset({ 0.0f }, { ELEC_TO_MECH(Initial_Angle.elec, params.motor.P) }); // to test rotor pre-alignment before spinning

        params.ctrl.mode = Curr_Mode_FOC_Sensorless_HighFreq_Startup; // [#]
        params.obs.w_thresh.elec = params.motor.w_nom.elec * 0.05f; // [Ra/sec-elec]
        PARAMS_DEFAULT_InitAutoCalc();
        STATE_MACHINE_Init();
    }

    void teardown()
    {
        delete sys_sim;
    }

    float OptimalCurrentAngle(float i_cmd)
    {
        float i_s = ABS(i_cmd);
        float angle_raw = asinf((params.motor.lam - sqrtf(POW_TWO(params.motor.lam) + 8.0f * POW_TWO((params.motor.ld - params.motor.lq) * i_s))) / (4.0f * (params.motor.lq - params.motor.ld) * i_s));
        float angle_final = (i_cmd > 0.0f) ? (-angle_raw) : (PI + angle_raw);
        return angle_final;
    }
};

TEST(CurrModeFOCHighFreqStartupTest, ClosedLoopPositiveCurrntCmd)
{
    sensor_iface.digital.dir = 1U;
    float dir = (sensor_iface.digital.dir == 1U) ? (+1.0f) : (-1.0f);

    // Setting capture channels for capturing the variables before state transitions
    sm.vars.capture_channels[0] = &vars.i_cmd_int;
    sm.vars.capture_channels[1] = &vars.i_qd_r_ref.q;
    sm.vars.capture_channels[2] = &vars.i_qd_r_ref.d;
    sm.vars.capture_channels[3] = &vars.i_qd_r_cmd.q;
    sm.vars.capture_channels[4] = &vars.i_qd_r_cmd.d;
    sm.vars.capture_channels[5] = &vars.th_r_est.elec;
    sm.vars.capture_channels[6] = &vars.w_est.elec;
    sm.vars.capture_channels[7] = &vars.la_qd_r_est.q;
    sm.vars.capture_channels[8] = &vars.la_qd_r_est.d;

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
    DOUBLES_EQUAL_PERC(params.ctrl.curr.i_cmd_thresh * dir, vars.i_cmd_ext, 0.01f);
    DOUBLES_EQUAL(0.0f, vars.i_cmd_int, 0.01f);

    // Six Pulse Injection state
    while (sm.current == Six_Pulse)
    {
        sys_sim->RunOneTick();
    }
    auto six_pulse_time = sys_sim->GetDiffTime();
    CHECK_TRUE(six_pulse_time > (6.0f * (params.ctrl.six_pulse_inj.t_on + params.ctrl.six_pulse_inj.t_off))); // speed cmd should go above the threshold
    CHECK_TRUE(sm.current == High_Freq);
    CHECK_TRUE(ctrl.six_pulse_inj.state == Finished_Success);
    // Verifying six pulse injection didn't spin the motor:
    ANGLE_EQUAL(Initial_Angle.elec, sys_sim->m_pmsm.m_th_r.elec, DEG_TO_RAD(5.00f));
    // Actual motor angle should be withing +/-15 degrees of estimated angle:
    ANGLE_EQUAL(sys_sim->m_pmsm.m_th_r.elec, vars.th_r_est.elec, DEG_TO_RAD(15.01f));
    DOUBLES_EQUAL(0.0f, sys_sim->m_pmsm.m_w_r.elec, 10.0f); // transients settled
    DOUBLES_EQUAL(0.0f, sys_sim->m_pmsm.m_i_qd_r.q, 1.0f); // transients settled
    DOUBLES_EQUAL(0.0f, sys_sim->m_pmsm.m_i_qd_r.d, 1.0f); // transients settled
    DOUBLES_EQUAL(0.0f, sm.vars.capture_vals[0], 0.01f); // vars.i_cmd_int
    DOUBLES_EQUAL_PERC(0.0f, vars.i_cmd_int, 0.01f);

    // High Frequency Injection state
    while (sm.current == High_Freq)
    {
        sys_sim->RunOneTick();
    }
    auto high_freq_time = sys_sim->GetDiffTime();
    CHECK_TRUE((high_freq_time / params.ctrl.high_freq_inj.lock_time) > 0.99f); // speed cmd should go above the threshold
    CHECK_TRUE(sm.current == Current_CL);
    // Verifying high frequency injection didn't spin the motor:
    ANGLE_EQUAL(Initial_Angle.elec, sys_sim->m_pmsm.m_th_r.elec, DEG_TO_RAD(5.0f));
    // Actual motor angle should be equal to estimated angle:
    ANGLE_EQUAL(sys_sim->m_pmsm.m_th_r.elec, vars.th_r_est.elec, DEG_TO_RAD(0.5f));
    DOUBLES_EQUAL(0.0f, sys_sim->m_pmsm.m_w_r.elec, 10.0f); // transients settled
    DOUBLES_EQUAL(0.0f, vars.i_qd_r_fb.q, 1.0f); // low-frequency current transients settled
    DOUBLES_EQUAL(0.0f, vars.i_qd_r_fb.d, 1.0f); // low-frequency current transients settled
    DOUBLES_EQUAL(0.0f, sm.vars.capture_vals[0], 0.01f); // vars.i_cmd_int
    DOUBLES_EQUAL_PERC(params.ctrl.curr.i_cmd_thresh * dir, vars.i_cmd_int, 0.01f);

    // Current-CL state: ramp up
    sys_sim->RunUntil(sys_sim->m_pot.m_t_start + sys_sim->m_pot.m_t_slope);
    DOUBLES_EQUAL_PERC(params.sys.cmd.i_max * dir, vars.i_cmd_int, 0.01f);
    DOUBLES_EQUAL_PERC(ABS(vars.i_cmd_int), CReferenceFrame(0.0f, vars.i_qd_r_ref).GetPolar().rad, 0.01f); // check phase advance (MTPA)
    ANGLE_EQUAL(OptimalCurrentAngle(vars.i_cmd_int), CReferenceFrame(0.0f, vars.i_qd_r_ref).GetPolar().theta, DEG_TO_RAD(5.0f)); // check phase advance (MTPA)
    DOUBLES_EQUAL_PERC(ABS(vars.i_cmd_int), CReferenceFrame(0.0f, vars.i_qd_r_cmd).GetPolar().rad, 0.01f); // check phase advance (MTPA)
    ANGLE_EQUAL(OptimalCurrentAngle(vars.i_cmd_int), CReferenceFrame(0.0f, vars.i_qd_r_cmd).GetPolar().theta, DEG_TO_RAD(5.0f)); // check phase advance (MTPA)
    DOUBLES_EQUAL(vars.i_qd_r_cmd.q, sys_sim->m_pmsm.m_i_qd_r.q, 5.0f); // check controller
    DOUBLES_EQUAL(vars.i_qd_r_cmd.d, sys_sim->m_pmsm.m_i_qd_r.d, 5.0f); // check controller
    ANGLE_EQUAL(sys_sim->m_pmsm.m_th_r.elec, vars.th_r_est.elec, DEG_TO_RAD(5.0f)); // verify observer is locked
    DOUBLES_EQUAL_PERC(sys_sim->m_pmsm.m_w_r.elec, vars.w_est.elec, 0.01f); // verify observer is locked
    DOUBLES_EQUAL_PERC(sys_sim->m_pmsm.m_T, vars.T_est, 0.05f);
    DOUBLES_EQUAL(sys_sim->m_pmsm.m_la_qd_r.q, vars.la_qd_r_est.q, 0.05f); // verify feedforward flux estimation
    DOUBLES_EQUAL(sys_sim->m_pmsm.m_la_qd_r.d, vars.la_qd_r_est.d, 0.05f); // verify feedforward flux estimation

    // Current-CL state: steady-state
    sys_sim->RunUntil(sys_sim->m_pot.m_t_stop);
    DOUBLES_EQUAL_PERC(params.sys.cmd.i_max * dir, vars.i_cmd_int, 0.01f);
    DOUBLES_EQUAL_PERC(ABS(vars.i_cmd_int), CReferenceFrame(0.0f, vars.i_qd_r_ref).GetPolar().rad, 0.01f); // check phase advance (MTPA)
    ANGLE_EQUAL(OptimalCurrentAngle(vars.i_cmd_int), CReferenceFrame(0.0f, vars.i_qd_r_ref).GetPolar().theta, DEG_TO_RAD(5.0f)); // check phase advance (MTPA)
    DOUBLES_EQUAL_PERC(ABS(vars.i_cmd_int), CReferenceFrame(0.0f, vars.i_qd_r_cmd).GetPolar().rad, 0.01f); // check phase advance (MTPA)
    ANGLE_EQUAL(OptimalCurrentAngle(vars.i_cmd_int), CReferenceFrame(0.0f, vars.i_qd_r_cmd).GetPolar().theta, DEG_TO_RAD(5.0f)); // check phase advance (MTPA)
    DOUBLES_EQUAL(vars.i_qd_r_cmd.q, sys_sim->m_pmsm.m_i_qd_r.q, 5.0f); // check controller
    DOUBLES_EQUAL(vars.i_qd_r_cmd.d, sys_sim->m_pmsm.m_i_qd_r.d, 5.0f); // check controller
    ANGLE_EQUAL(sys_sim->m_pmsm.m_th_r.elec, vars.th_r_est.elec, DEG_TO_RAD(5.0f)); // verify observer is locked
    DOUBLES_EQUAL_PERC(sys_sim->m_pmsm.m_w_r.elec, vars.w_est.elec, 0.01f); // verify observer is locked
    DOUBLES_EQUAL_PERC(sys_sim->m_pmsm.m_T, vars.T_est, 0.05f);
    DOUBLES_EQUAL(sys_sim->m_pmsm.m_la_qd_r.q, vars.la_qd_r_est.q, 0.05f); // verify feedforward flux estimation
    DOUBLES_EQUAL(sys_sim->m_pmsm.m_la_qd_r.d, vars.la_qd_r_est.d, 0.05f); // verify feedforward flux estimation

    // Current-CL state: steady-state: ramp down
    while (sm.current == Current_CL)
    {
        sys_sim->RunOneTick();
    }
    auto current_cl_time = sys_sim->GetDiffTime();
    float i_cmd_int_cap = sm.vars.capture_vals[0]; // vars.i_cmd_int
    QD_t i_qd_r_ref_cap = { sm.vars.capture_vals[1], sm.vars.capture_vals[2] }; // vars.i_qd_r_ref
    QD_t i_qd_r_cmd_cap = { sm.vars.capture_vals[3], sm.vars.capture_vals[4] }; // vars.i_qd_r_cmd
    DOUBLES_EQUAL(vars.i_cmd_ext, i_cmd_int_cap, 0.01f);
    DOUBLES_EQUAL_PERC(0.0f, vars.i_cmd_int, 0.01f);
    DOUBLES_EQUAL_PERC(ABS(i_cmd_int_cap), CReferenceFrame(0.0f, i_qd_r_ref_cap).GetPolar().rad, 0.01f); // check phase advance (MTPA)
    ANGLE_EQUAL(OptimalCurrentAngle(i_cmd_int_cap), CReferenceFrame(0.0f, i_qd_r_ref_cap).GetPolar().theta, DEG_TO_RAD(5.0f)); // check phase advance (MTPA)
    DOUBLES_EQUAL_PERC(ABS(i_cmd_int_cap), CReferenceFrame(0.0f, i_qd_r_cmd_cap).GetPolar().rad, 0.01f); // check phase advance (MTPA)
    ANGLE_EQUAL(OptimalCurrentAngle(i_cmd_int_cap), CReferenceFrame(0.0f, i_qd_r_cmd_cap).GetPolar().theta, DEG_TO_RAD(5.0f)); // check phase advance (MTPA)
    DOUBLES_EQUAL(i_qd_r_cmd_cap.q, sys_sim->m_pmsm.m_i_qd_r.q, 5.0f); // check controller
    DOUBLES_EQUAL(i_qd_r_cmd_cap.d, sys_sim->m_pmsm.m_i_qd_r.d, 5.0f); // check controller
    ANGLE_EQUAL(sys_sim->m_pmsm.m_th_r.elec, sm.vars.capture_vals[5], DEG_TO_RAD(5.0f)); // vars.th_r_est.elec
    DOUBLES_EQUAL_PERC(sys_sim->m_pmsm.m_w_r.elec, sm.vars.capture_vals[6], 0.01f); // vars.w_est.elec
    DOUBLES_EQUAL_PERC(sys_sim->m_pmsm.m_T, vars.T_est, 0.05f);
    DOUBLES_EQUAL(sys_sim->m_pmsm.m_la_qd_r.q, sm.vars.capture_vals[7], 0.05f); // vars.la_qd_r_est.q
    DOUBLES_EQUAL(sys_sim->m_pmsm.m_la_qd_r.d, sm.vars.capture_vals[8], 0.05f); // vars.la_qd_r_est.d

    // Back to brake-boot state
    sys_sim->RunUntil(sys_sim->m_pot.m_t_stop + sys_sim->m_pot.m_t_slope);
    DOUBLES_EQUAL_PERC(0.0f, vars.i_cmd_int, 0.01f);
    DOUBLES_EQUAL(0.0f, sys_sim->m_pmsm.m_i_qd_r.q, 0.01f);
    DOUBLES_EQUAL(0.0f, sys_sim->m_pmsm.m_i_qd_r.d, 0.01f);
    DOUBLES_EQUAL(0.0f, sys_sim->m_pmsm.m_w_r.elec, 0.1f);
    DOUBLES_EQUAL(0.0f, sys_sim->m_pmsm.m_T, 0.01f);
}


TEST(CurrModeFOCHighFreqStartupTest, ClosedLoopNegativeCurrntCmd)
{
    sensor_iface.digital.dir = 0U;
    float dir = (sensor_iface.digital.dir == 1U) ? (+1.0f) : (-1.0f);

    // Setting capture channels for capturing the variables before state transitions
    sm.vars.capture_channels[0] = &vars.i_cmd_int;
    sm.vars.capture_channels[1] = &vars.i_qd_r_ref.q;
    sm.vars.capture_channels[2] = &vars.i_qd_r_ref.d;
    sm.vars.capture_channels[3] = &vars.i_qd_r_cmd.q;
    sm.vars.capture_channels[4] = &vars.i_qd_r_cmd.d;
    sm.vars.capture_channels[5] = &vars.th_r_est.elec;
    sm.vars.capture_channels[6] = &vars.w_est.elec;
    sm.vars.capture_channels[7] = &vars.la_qd_r_est.q;
    sm.vars.capture_channels[8] = &vars.la_qd_r_est.d;

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
    DOUBLES_EQUAL_PERC(params.ctrl.curr.i_cmd_thresh * dir, vars.i_cmd_ext, 0.01f);
    DOUBLES_EQUAL(0.0f, vars.i_cmd_int, 0.01f);

    // Six Pulse Injection state
    while (sm.current == Six_Pulse)
    {
        sys_sim->RunOneTick();
    }
    auto six_pulse_time = sys_sim->GetDiffTime();
    CHECK_TRUE(six_pulse_time > (6.0f * (params.ctrl.six_pulse_inj.t_on + params.ctrl.six_pulse_inj.t_off))); // speed cmd should go above the threshold
    CHECK_TRUE(sm.current == High_Freq);
    CHECK_TRUE(ctrl.six_pulse_inj.state == Finished_Success);
    // Verifying six pulse injection didn't spin the motor:
    ANGLE_EQUAL(Initial_Angle.elec, sys_sim->m_pmsm.m_th_r.elec, DEG_TO_RAD(5.00f));
    // Actual motor angle should be withing +/-15 degrees of estimated angle:
    ANGLE_EQUAL(sys_sim->m_pmsm.m_th_r.elec, vars.th_r_est.elec, DEG_TO_RAD(15.01f));
    DOUBLES_EQUAL(0.0f, sys_sim->m_pmsm.m_w_r.elec, 10.0f); // transients settled
    DOUBLES_EQUAL(0.0f, sys_sim->m_pmsm.m_i_qd_r.q, 1.0f); // transients settled
    DOUBLES_EQUAL(0.0f, sys_sim->m_pmsm.m_i_qd_r.d, 1.0f); // transients settled
    DOUBLES_EQUAL(0.0f, sm.vars.capture_vals[0], 0.01f); // vars.i_cmd_int
    DOUBLES_EQUAL_PERC(0.0f, vars.i_cmd_int, 0.01f);

    // High Frequency Injection state
    while (sm.current == High_Freq)
    {
        sys_sim->RunOneTick();
    }
    auto high_freq_time = sys_sim->GetDiffTime();
    CHECK_TRUE((high_freq_time / params.ctrl.high_freq_inj.lock_time) > 0.99f); // speed cmd should go above the threshold
    CHECK_TRUE(sm.current == Current_CL);
    // Verifying high frequency injection didn't spin the motor:
    ANGLE_EQUAL(Initial_Angle.elec, sys_sim->m_pmsm.m_th_r.elec, DEG_TO_RAD(5.0f));
    // Actual motor angle should be equal to estimated angle:
    ANGLE_EQUAL(sys_sim->m_pmsm.m_th_r.elec, vars.th_r_est.elec, DEG_TO_RAD(0.5f));
    DOUBLES_EQUAL(0.0f, sys_sim->m_pmsm.m_w_r.elec, 10.0f); // transients settled
    DOUBLES_EQUAL(0.0f, vars.i_qd_r_fb.q, 1.0f); // low-frequency current transients settled
    DOUBLES_EQUAL(0.0f, vars.i_qd_r_fb.d, 1.0f); // low-frequency current transients settled
    DOUBLES_EQUAL(0.0f, sm.vars.capture_vals[0], 0.01f); // vars.i_cmd_int
    DOUBLES_EQUAL_PERC(params.ctrl.curr.i_cmd_thresh * dir, vars.i_cmd_int, 0.01f);

    // Current-CL state: ramp up
    sys_sim->RunUntil(sys_sim->m_pot.m_t_start + sys_sim->m_pot.m_t_slope);
    DOUBLES_EQUAL_PERC(params.sys.cmd.i_max * dir, vars.i_cmd_int, 0.01f);
    DOUBLES_EQUAL_PERC(ABS(vars.i_cmd_int), CReferenceFrame(0.0f, vars.i_qd_r_ref).GetPolar().rad, 0.01f); // check phase advance (MTPA)
    ANGLE_EQUAL(OptimalCurrentAngle(vars.i_cmd_int), CReferenceFrame(0.0f, vars.i_qd_r_ref).GetPolar().theta, DEG_TO_RAD(5.0f)); // check phase advance (MTPA)
    DOUBLES_EQUAL_PERC(ABS(vars.i_cmd_int), CReferenceFrame(0.0f, vars.i_qd_r_cmd).GetPolar().rad, 0.01f); // check phase advance (MTPA)
    ANGLE_EQUAL(OptimalCurrentAngle(vars.i_cmd_int), CReferenceFrame(0.0f, vars.i_qd_r_cmd).GetPolar().theta, DEG_TO_RAD(5.0f)); // check phase advance (MTPA)
    DOUBLES_EQUAL(vars.i_qd_r_cmd.q, sys_sim->m_pmsm.m_i_qd_r.q, 5.0f); // check controller
    DOUBLES_EQUAL(vars.i_qd_r_cmd.d, sys_sim->m_pmsm.m_i_qd_r.d, 5.0f); // check controller
    ANGLE_EQUAL(sys_sim->m_pmsm.m_th_r.elec, vars.th_r_est.elec, DEG_TO_RAD(5.0f)); // verify observer is locked
    DOUBLES_EQUAL_PERC(sys_sim->m_pmsm.m_w_r.elec, vars.w_est.elec, 0.01f); // verify observer is locked
    DOUBLES_EQUAL_PERC(sys_sim->m_pmsm.m_T, vars.T_est, 0.05f);
    DOUBLES_EQUAL(sys_sim->m_pmsm.m_la_qd_r.q, vars.la_qd_r_est.q, 0.05f); // verify feedforward flux estimation
    DOUBLES_EQUAL(sys_sim->m_pmsm.m_la_qd_r.d, vars.la_qd_r_est.d, 0.05f); // verify feedforward flux estimation

    // Current-CL state: steady-state
    sys_sim->RunUntil(sys_sim->m_pot.m_t_stop);
    DOUBLES_EQUAL_PERC(params.sys.cmd.i_max * dir, vars.i_cmd_int, 0.01f);
    DOUBLES_EQUAL_PERC(ABS(vars.i_cmd_int), CReferenceFrame(0.0f, vars.i_qd_r_ref).GetPolar().rad, 0.01f); // check phase advance (MTPA)
    ANGLE_EQUAL(OptimalCurrentAngle(vars.i_cmd_int), CReferenceFrame(0.0f, vars.i_qd_r_ref).GetPolar().theta, DEG_TO_RAD(5.0f)); // check phase advance (MTPA)
    DOUBLES_EQUAL_PERC(ABS(vars.i_cmd_int), CReferenceFrame(0.0f, vars.i_qd_r_cmd).GetPolar().rad, 0.01f); // check phase advance (MTPA)
    ANGLE_EQUAL(OptimalCurrentAngle(vars.i_cmd_int), CReferenceFrame(0.0f, vars.i_qd_r_cmd).GetPolar().theta, DEG_TO_RAD(5.0f)); // check phase advance (MTPA)
    DOUBLES_EQUAL(vars.i_qd_r_cmd.q, sys_sim->m_pmsm.m_i_qd_r.q, 5.0f); // check controller
    DOUBLES_EQUAL(vars.i_qd_r_cmd.d, sys_sim->m_pmsm.m_i_qd_r.d, 5.0f); // check controller
    ANGLE_EQUAL(sys_sim->m_pmsm.m_th_r.elec, vars.th_r_est.elec, DEG_TO_RAD(5.0f)); // verify observer is locked
    DOUBLES_EQUAL_PERC(sys_sim->m_pmsm.m_w_r.elec, vars.w_est.elec, 0.01f); // verify observer is locked
    DOUBLES_EQUAL_PERC(sys_sim->m_pmsm.m_T, vars.T_est, 0.05f);
    DOUBLES_EQUAL(sys_sim->m_pmsm.m_la_qd_r.q, vars.la_qd_r_est.q, 0.05f); // verify feedforward flux estimation
    DOUBLES_EQUAL(sys_sim->m_pmsm.m_la_qd_r.d, vars.la_qd_r_est.d, 0.05f); // verify feedforward flux estimation

    // Current-CL state: steady-state: ramp down
    while (sm.current == Current_CL)
    {
        sys_sim->RunOneTick();
    }
    auto current_cl_time = sys_sim->GetDiffTime();
    float i_cmd_int_cap = sm.vars.capture_vals[0]; // vars.i_cmd_int
    QD_t i_qd_r_ref_cap = { sm.vars.capture_vals[1], sm.vars.capture_vals[2] }; // vars.i_qd_r_ref
    QD_t i_qd_r_cmd_cap = { sm.vars.capture_vals[3], sm.vars.capture_vals[4] }; // vars.i_qd_r_cmd
    DOUBLES_EQUAL(vars.i_cmd_ext, i_cmd_int_cap, 0.01f);
    DOUBLES_EQUAL_PERC(0.0f, vars.i_cmd_int, 0.01f);
    DOUBLES_EQUAL_PERC(ABS(i_cmd_int_cap), CReferenceFrame(0.0f, i_qd_r_ref_cap).GetPolar().rad, 0.01f); // check phase advance (MTPA)
    ANGLE_EQUAL(OptimalCurrentAngle(i_cmd_int_cap), CReferenceFrame(0.0f, i_qd_r_ref_cap).GetPolar().theta, DEG_TO_RAD(5.0f)); // check phase advance (MTPA)
    DOUBLES_EQUAL_PERC(ABS(i_cmd_int_cap), CReferenceFrame(0.0f, i_qd_r_cmd_cap).GetPolar().rad, 0.01f); // check phase advance (MTPA)
    ANGLE_EQUAL(OptimalCurrentAngle(i_cmd_int_cap), CReferenceFrame(0.0f, i_qd_r_cmd_cap).GetPolar().theta, DEG_TO_RAD(5.0f)); // check phase advance (MTPA)
    DOUBLES_EQUAL(i_qd_r_cmd_cap.q, sys_sim->m_pmsm.m_i_qd_r.q, 5.0f); // check controller
    DOUBLES_EQUAL(i_qd_r_cmd_cap.d, sys_sim->m_pmsm.m_i_qd_r.d, 5.0f); // check controller
    ANGLE_EQUAL(sys_sim->m_pmsm.m_th_r.elec, sm.vars.capture_vals[5], DEG_TO_RAD(5.0f)); // vars.th_r_est.elec
    DOUBLES_EQUAL_PERC(sys_sim->m_pmsm.m_w_r.elec, sm.vars.capture_vals[6], 0.01f); // vars.w_est.elec
    DOUBLES_EQUAL_PERC(sys_sim->m_pmsm.m_T, vars.T_est, 0.05f);
    DOUBLES_EQUAL(sys_sim->m_pmsm.m_la_qd_r.q, sm.vars.capture_vals[7], 0.05f); // vars.la_qd_r_est.q
    DOUBLES_EQUAL(sys_sim->m_pmsm.m_la_qd_r.d, sm.vars.capture_vals[8], 0.05f); // vars.la_qd_r_est.d

    // Back to brake-boot state
    sys_sim->RunUntil(sys_sim->m_pot.m_t_stop + sys_sim->m_pot.m_t_slope);
    DOUBLES_EQUAL_PERC(0.0f, vars.i_cmd_int, 0.01f);
    DOUBLES_EQUAL(0.0f, sys_sim->m_pmsm.m_i_qd_r.q, 0.01f);
    DOUBLES_EQUAL(0.0f, sys_sim->m_pmsm.m_i_qd_r.d, 0.01f);
    DOUBLES_EQUAL(0.0f, sys_sim->m_pmsm.m_w_r.elec, 0.1f);
    DOUBLES_EQUAL(0.0f, sys_sim->m_pmsm.m_T, 0.01f);
}

#endif