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


#if defined(CTRL_METHOD_SFO)

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

TEST_GROUP(SlidingModeCurrLimiterTest)
{
    CSysSim* sys_sim;
    const float Epsilon = 1.0E-6f;

    void setup()
    {
        sys_sim = new CSysSim();
        sys_sim->SetDiscreteIntegMethod(CDiscreteInteg::EMethod::Backward_Euler);

        params.ctrl.mode = Trq_Mode_FOC_Sensorless_Align_Startup;
        params.sys.cmd.T_max = 1.4f; // higher than max torque
        params.motor.i2t.therm_tau = 1.0f; // shorter time constant to reduce simulation time
        PARAMS_DEFAULT_InitAutoCalc();
        STATE_MACHINE_Init();

    }

    void teardown()
    {
        delete sys_sim;
    }

};

TEST(SlidingModeCurrLimiterTest, PositiveTorqueCmd)
{
    sensor_iface.digital.dir = 1U;
    float dir = (sensor_iface.digital.dir == 1U) ? (+1.0f) : (-1.0f);

    // Turning potentiometer parameters
    sys_sim->m_pot.m_init = 0.0f;		// [%]
    sys_sim->m_pot.m_final = 1.0f;		// [%]
    sys_sim->m_pot.m_t_start = 0.5f;	// [sec]
    sys_sim->m_pot.m_t_stop = 5.0f;		// [sec]
    sys_sim->m_pot.m_t_slope = 2.5f;	// [sec]

    // Run simulation
    // Start up and run until motor i2t protection engages and reduces the current cap
    while (protect.motor.i2t.state == Dis)
    {
        sys_sim->RunOneTick();
    }
    CHECK_TRUE(sm.current == Torque_CL);
    float i2t_en_time = sys_sim->GetDiffTime();
    DOUBLES_EQUAL_PERC(protect.motor.i2t.i_on, protect.motor.i2t.i_filt, 0.01f); // i2t triggered and engaged
    DOUBLES_EQUAL_PERC(params.motor.i_cont, protect.motor.i2t.i_limit, 0.01f); // current cap reduced

    // Run until sliding mode controller engages and caps the torque command
    while (protect.motor.T_lmt >= ABS(vars.T_cmd_ext))
    {
        sys_sim->RunOneTick();
    }
    CHECK_TRUE(sm.current == Torque_CL);
    float sliding_mode_ctrl_engage_time_actual = sys_sim->GetDiffTime();
    float sliding_mode_ctrl_engage_time_expected = (params.motor.T_max - ABS(vars.T_cmd_ext)) / (params.ctrl.trq.curr_lmt_ki * params.sys.samp.fs0);
    DOUBLES_EQUAL_PERC(sliding_mode_ctrl_engage_time_expected, sliding_mode_ctrl_engage_time_actual, 0.01f);
    DOUBLES_EQUAL_PERC(vars.T_cmd_ext, vars.T_cmd_prot, 0.01f); // torque command about to change but still intact
    DOUBLES_EQUAL_PERC(vars.T_cmd_ext, vars.T_cmd_int, 0.01f); // torque command about to change but still intact
    CHECK_TRUE(protect.motor.i2t.state == En); // i2t still engaged
    DOUBLES_EQUAL_PERC(params.motor.i_cont, protect.motor.i2t.i_limit, 0.01f); // current cap still reduced

    // Check currents after sliding-mode controller reached the sliding surface
    sys_sim->RunFor(params.ctrl.trq.curr_lmt_t_reach);
    CHECK_TRUE(sm.current == Torque_CL);
    float sliding_mode_ctrl_reach_time = sys_sim->GetDiffTime();
    CHECK_TRUE(protect.motor.T_lmt < params.motor.T_max); // sliding mode controller still engaged
    CHECK_TRUE(ABS(vars.T_cmd_prot) < ABS(vars.T_cmd_ext)); // torque command capped
    CHECK_TRUE(ABS(vars.T_cmd_int) < ABS(vars.T_cmd_ext)); // torque command capped
    CHECK_TRUE(protect.motor.i2t.state == En); // i2t still engaged
    DOUBLES_EQUAL_PERC(params.motor.i_cont, protect.motor.i2t.i_limit, 0.01f); // current cap still reduced
    DOUBLES_EQUAL_PERC(POW_TWO(params.motor.i_cont), vars.i_s_fb_sq, 0.01f); // current regulated to motor.i_cont (current cap)

    // Run until the torque command is not capped anymore
    while (protect.motor.T_lmt <= ABS(vars.T_cmd_ext))
    {
        sys_sim->RunOneTick();
    }
    CHECK_TRUE(sm.current == Torque_CL);
    float remove_trq_cap_time = sys_sim->GetDiffTime();
    CHECK_TRUE(protect.motor.T_lmt < params.motor.T_max); // sliding mode controller still engaged
    DOUBLES_EQUAL_PERC(vars.T_cmd_ext, vars.T_cmd_prot, 0.01f); // torque cap about to be removed
    DOUBLES_EQUAL_PERC(vars.T_cmd_ext, vars.T_cmd_int, 0.01f); // torque cap about to be removed
    CHECK_TRUE(protect.motor.i2t.state == En); // i2t still engaged
    DOUBLES_EQUAL_PERC(params.motor.i_cont, protect.motor.i2t.i_limit, 0.01f); // current cap still reduced
    DOUBLES_EQUAL_PERC(POW_TWO(params.motor.i_cont), vars.i_s_fb_sq, 0.01f); // current still regulated to motor.i_cont (current cap)
    float T_cmd_ext_prev = vars.T_cmd_ext; // saved for calculations in the next step

    // Run until sliding mode controller completely disengages
    while (protect.motor.T_lmt < params.motor.T_max)
    {
        sys_sim->RunOneTick();
    }
    CHECK_TRUE(sm.current == Torque_CL);
    float sliding_mode_ctrl_disengage_time_actual = sys_sim->GetDiffTime();
    float sliding_mode_ctrl_disengage_time_expected = (params.motor.T_max - ABS(T_cmd_ext_prev)) / (params.ctrl.trq.curr_lmt_ki * params.sys.samp.fs0);
    DOUBLES_EQUAL(sliding_mode_ctrl_engage_time_expected, sliding_mode_ctrl_engage_time_actual, 0.005f);
    DOUBLES_EQUAL_PERC(params.motor.T_max, protect.motor.T_lmt, 0.01f); // sliding mode controller disengaged
    DOUBLES_EQUAL_PERC(vars.T_cmd_ext, vars.T_cmd_prot, 0.01f); // torque cap removed
    DOUBLES_EQUAL_PERC(vars.T_cmd_ext, vars.T_cmd_int, 0.01f); // torque cap removed
    CHECK_TRUE(protect.motor.i2t.state == En); // i2t still engaged

    // Run until motor i2t protection disengages
    while (protect.motor.i2t.state == En)
    {
        sys_sim->RunOneTick();
    }
    CHECK_TRUE(sm.current == Torque_CL);
    float i2t_disengage_time = sys_sim->GetDiffTime();
    DOUBLES_EQUAL_PERC(params.motor.T_max, protect.motor.T_lmt, 0.01f); // sliding mode controller still disengaged
    DOUBLES_EQUAL_PERC(vars.T_cmd_ext, vars.T_cmd_prot, 0.01f); // torque cap still removed
    DOUBLES_EQUAL_PERC(vars.T_cmd_ext, vars.T_cmd_int, 0.01f); // torque cap still removed
    DOUBLES_EQUAL_PERC(protect.motor.i2t.i_off, protect.motor.i2t.i_filt, 0.01f); // i2t triggered to disengage
    DOUBLES_EQUAL_PERC(params.motor.i_peak, protect.motor.i2t.i_limit, 0.01f); // current cap removed

    // Run until back to brake-boot-state
    sys_sim->RunUntil(sys_sim->m_pot.m_t_stop + sys_sim->m_pot.m_t_slope);
    DOUBLES_EQUAL(0.0f, sys_sim->m_pmsm.m_w_r.elec, 0.1f);
    DOUBLES_EQUAL_PERC(0.0f, vars.T_cmd_int, 0.01f);
    DOUBLES_EQUAL(0.0f, sys_sim->m_pmsm.m_T, 0.01f);
    DOUBLES_EQUAL(0.0f, sys_sim->m_pmsm.m_i_qd_s.q, 0.01f);
    DOUBLES_EQUAL(0.0f, sys_sim->m_pmsm.m_i_qd_s.d, 0.01f);

}


TEST(SlidingModeCurrLimiterTest, NegativeTorqueCmd)
{
    sensor_iface.digital.dir = 0U;
    float dir = (sensor_iface.digital.dir == 1U) ? (+1.0f) : (-1.0f);

    // Turning potentiometer parameters
    sys_sim->m_pot.m_init = 0.0f;		// [%]
    sys_sim->m_pot.m_final = 1.0f;		// [%]
    sys_sim->m_pot.m_t_start = 0.5f;	// [sec]
    sys_sim->m_pot.m_t_stop = 5.0f;		// [sec]
    sys_sim->m_pot.m_t_slope = 2.5f;	// [sec]

    // Run simulation
    // Start up and run until motor i2t protection engages and reduces the current cap
    while (protect.motor.i2t.state == Dis)
    {
        sys_sim->RunOneTick();
    }
    CHECK_TRUE(sm.current == Torque_CL);
    float i2t_en_time = sys_sim->GetDiffTime();
    DOUBLES_EQUAL_PERC(protect.motor.i2t.i_on, protect.motor.i2t.i_filt, 0.01f); // i2t triggered and engaged
    DOUBLES_EQUAL_PERC(params.motor.i_cont, protect.motor.i2t.i_limit, 0.01f); // current cap reduced

    // Run until sliding mode controller engages and caps the torque command
    while (protect.motor.T_lmt >= ABS(vars.T_cmd_ext))
    {
        sys_sim->RunOneTick();
    }
    CHECK_TRUE(sm.current == Torque_CL);
    float sliding_mode_ctrl_engage_time_actual = sys_sim->GetDiffTime();
    float sliding_mode_ctrl_engage_time_expected = (params.motor.T_max - ABS(vars.T_cmd_ext)) / (params.ctrl.trq.curr_lmt_ki * params.sys.samp.fs0);
    DOUBLES_EQUAL_PERC(sliding_mode_ctrl_engage_time_expected, sliding_mode_ctrl_engage_time_actual, 0.01f);
    DOUBLES_EQUAL_PERC(vars.T_cmd_ext, vars.T_cmd_prot, 0.01f); // torque command about to change but still intact
    DOUBLES_EQUAL_PERC(vars.T_cmd_ext, vars.T_cmd_int, 0.01f); // torque command about to change but still intact
    CHECK_TRUE(protect.motor.i2t.state == En); // i2t still engaged
    DOUBLES_EQUAL_PERC(params.motor.i_cont, protect.motor.i2t.i_limit, 0.01f); // current cap still reduced

    // Check currents after sliding-mode controller reached the sliding surface
    sys_sim->RunFor(params.ctrl.trq.curr_lmt_t_reach);
    CHECK_TRUE(sm.current == Torque_CL);
    float sliding_mode_ctrl_reach_time = sys_sim->GetDiffTime();
    CHECK_TRUE(protect.motor.T_lmt < params.motor.T_max); // sliding mode controller still engaged
    CHECK_TRUE(ABS(vars.T_cmd_prot) < ABS(vars.T_cmd_ext)); // torque command capped
    CHECK_TRUE(ABS(vars.T_cmd_int) < ABS(vars.T_cmd_ext)); // torque command capped
    CHECK_TRUE(protect.motor.i2t.state == En); // i2t still engaged
    DOUBLES_EQUAL_PERC(params.motor.i_cont, protect.motor.i2t.i_limit, 0.01f); // current cap still reduced
    DOUBLES_EQUAL_PERC(POW_TWO(params.motor.i_cont), vars.i_s_fb_sq, 0.01f); // current regulated to motor.i_cont (current cap)

    // Run until the torque command is not capped anymore
    while (protect.motor.T_lmt <= ABS(vars.T_cmd_ext))
    {
        sys_sim->RunOneTick();
    }
    CHECK_TRUE(sm.current == Torque_CL);
    float remove_trq_cap_time = sys_sim->GetDiffTime();
    CHECK_TRUE(protect.motor.T_lmt < params.motor.T_max); // sliding mode controller still engaged
    DOUBLES_EQUAL_PERC(vars.T_cmd_ext, vars.T_cmd_prot, 0.01f); // torque cap about to be removed
    DOUBLES_EQUAL_PERC(vars.T_cmd_ext, vars.T_cmd_int, 0.01f); // torque cap about to be removed
    CHECK_TRUE(protect.motor.i2t.state == En); // i2t still engaged
    DOUBLES_EQUAL_PERC(params.motor.i_cont, protect.motor.i2t.i_limit, 0.01f); // current cap still reduced
    DOUBLES_EQUAL_PERC(POW_TWO(params.motor.i_cont), vars.i_s_fb_sq, 0.01f); // current still regulated to motor.i_cont (current cap)
    float T_cmd_ext_prev = vars.T_cmd_ext; // saved for calculations in the next step

    // Run until sliding mode controller completely disengages
    while (protect.motor.T_lmt < params.motor.T_max)
    {
        sys_sim->RunOneTick();
    }
    CHECK_TRUE(sm.current == Torque_CL);
    float sliding_mode_ctrl_disengage_time_actual = sys_sim->GetDiffTime();
    float sliding_mode_ctrl_disengage_time_expected = (params.motor.T_max - ABS(T_cmd_ext_prev)) / (params.ctrl.trq.curr_lmt_ki * params.sys.samp.fs0);
    DOUBLES_EQUAL(sliding_mode_ctrl_engage_time_expected, sliding_mode_ctrl_engage_time_actual, 0.005f);
    DOUBLES_EQUAL_PERC(params.motor.T_max, protect.motor.T_lmt, 0.01f); // sliding mode controller disengaged
    DOUBLES_EQUAL_PERC(vars.T_cmd_ext, vars.T_cmd_prot, 0.01f); // torque cap removed
    DOUBLES_EQUAL_PERC(vars.T_cmd_ext, vars.T_cmd_int, 0.01f); // torque cap removed
    CHECK_TRUE(protect.motor.i2t.state == En); // i2t still engaged

    // Run until motor i2t protection disengages
    while (protect.motor.i2t.state == En)
    {
        sys_sim->RunOneTick();
    }
    CHECK_TRUE(sm.current == Torque_CL);
    float i2t_disengage_time = sys_sim->GetDiffTime();
    DOUBLES_EQUAL_PERC(params.motor.T_max, protect.motor.T_lmt, 0.01f); // sliding mode controller still disengaged
    DOUBLES_EQUAL_PERC(vars.T_cmd_ext, vars.T_cmd_prot, 0.01f); // torque cap still removed
    DOUBLES_EQUAL_PERC(vars.T_cmd_ext, vars.T_cmd_int, 0.01f); // torque cap still removed
    DOUBLES_EQUAL_PERC(protect.motor.i2t.i_off, protect.motor.i2t.i_filt, 0.01f); // i2t triggered to disengage
    DOUBLES_EQUAL_PERC(params.motor.i_peak, protect.motor.i2t.i_limit, 0.01f); // current cap removed

    // Run until back to brake-boot-state
    sys_sim->RunUntil(sys_sim->m_pot.m_t_stop + sys_sim->m_pot.m_t_slope);
    DOUBLES_EQUAL(0.0f, sys_sim->m_pmsm.m_w_r.elec, 0.1f);
    DOUBLES_EQUAL_PERC(0.0f, vars.T_cmd_int, 0.01f);
    DOUBLES_EQUAL(0.0f, sys_sim->m_pmsm.m_T, 0.01f);
    DOUBLES_EQUAL(0.0f, sys_sim->m_pmsm.m_i_qd_s.q, 0.01f);
    DOUBLES_EQUAL(0.0f, sys_sim->m_pmsm.m_i_qd_s.d, 0.01f);

}

#endif