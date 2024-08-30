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


#if defined(CTRL_METHOD_TBC)

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

TEST_GROUP(SpeedModeTrapCommHallTest)
{
    CSysSim* sys_sim;

    const ELEC_t Initial_Angle = { DEG_TO_RAD(45.0f) };

    void setup()
    {
        sys_sim = new CSysSim();
        sys_sim->SetMotorType(CMotor::EType::BLDC);
        sys_sim->SetLoadType(CLoad::EType::Passive_Mech_Load);
        sys_sim->SetFeedbackType(CFeedback::EType::Hall_Sensor);

        sys_sim->m_mech_load.Reset({ 0.0f }, { ELEC_TO_MECH(Initial_Angle.elec, params.motor.P) });

        params.ctrl.mode = Speed_Mode_Block_Comm_Hall;
        params.sys.fb.hall.block_comm_offset_comp = Dis;
        params.ctrl.tbc.mode = Trapezoidal_Commutation;
        params.sys.samp.fpwm_fs0_ratio = 1U;
        PARAMS_DEFAULT_InitAutoCalc();
        STATE_MACHINE_Init();
    }

    void teardown()
    {
        delete sys_sim;
    }
};

TEST(SpeedModeTrapCommHallTest, ClosedLoopPositiveSpeedCmd)
{
    sensor_iface.digital.dir = 1U;
    float dir = (sensor_iface.digital.dir == 1U) ? (+1.0f) : (-1.0f);

    // Setting capture channels for capturing the variables before state transitions
    sm.vars.capture_channels[0] = &vars.w_cmd_int.elec;
    sm.vars.capture_channels[1] = &vars.th_r_hall.elec;
    sm.vars.capture_channels[2] = &vars.w_hall.elec;

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
    CHECK_TRUE(sm.current == Speed_CL);
    CHECK_EQUAL(StopWatchIsDone(&sm.vars.brake_boot.timer), true);
    CHECK_TRUE(brake_boot_time >= params.sys.boot_time);
    DOUBLES_EQUAL_PERC(params.ctrl.volt.w_thresh.elec * dir, vars.w_cmd_ext.elec, 0.01f);
    DOUBLES_EQUAL_PERC(params.ctrl.volt.w_thresh.elec * dir, vars.w_cmd_int.elec, 0.01f);
    DOUBLES_EQUAL(0.0f, sm.vars.capture_vals[0], 0.01f); // vars.w_cmd_int
    ANGLE_EQUAL(Initial_Angle.elec, sys_sim->m_bldc.m_th_r.elec, DEG_TO_RAD(0.5f));
    ANGLE_EQUAL(sys_sim->m_bldc.m_th_r.elec, vars.th_r_fb.elec, DEG_TO_RAD(0.5f));
    ANGLE_EQUAL(sys_sim->m_bldc.m_th_r.elec, vars.th_r_hall.elec, DEG_TO_RAD(30.0f));
    ANGLE_EQUAL(sys_sim->m_bldc.m_th_r.elec, vars.th_r_final.elec, DEG_TO_RAD(30.0f));

    // Speed-CL state: ramp up
    sys_sim->RunUntil(sys_sim->m_pot.m_t_start + sys_sim->m_pot.m_t_slope);
    DOUBLES_EQUAL_PERC(MECH_TO_ELEC(params.sys.cmd.w_max.mech, params.motor.P) * dir, vars.w_cmd_int.elec, 0.01f);
    DOUBLES_EQUAL_PERC(sys_sim->m_bldc.m_w_r.elec, vars.w_hall.elec, 0.05f); // verify hall loop is locked
    DOUBLES_EQUAL_PERC(sys_sim->m_bldc.m_w_r.elec, vars.w_final.elec, 0.05f); // verify hall loop is locked
    DOUBLES_EQUAL_PERC(sys_sim->m_bldc.m_w_r.elec, vars.w_final_filt.elec, 0.05f); // verify hall loop is locked
    DOUBLES_EQUAL_PERC(vars.w_cmd_int.elec, vars.w_final_filt.elec, 0.01f); // check controller
    DOUBLES_EQUAL_PERC(vars.w_cmd_int.elec, sys_sim->m_bldc.m_w_r.elec, 0.01f);
    DOUBLES_EQUAL_PERC(sys_sim->m_bldc.m_T, vars.T_est, 0.10f);
    DOUBLES_EQUAL(ABS(vars.i_cmd_int) * ONE_OVER_SQRT_TWO, sys_sim->m_bldc.m_i_s, 15.0f); // check controller
    ANGLE_EQUAL(sys_sim->m_bldc.m_th_r.elec, vars.th_r_hall.elec, DEG_TO_RAD(7.5f)); // verify hall loop is locked

    // Speed-CL state: steady-state
    sys_sim->RunUntil(sys_sim->m_pot.m_t_stop);
    DOUBLES_EQUAL_PERC(MECH_TO_ELEC(params.sys.cmd.w_max.mech, params.motor.P) * dir, vars.w_cmd_int.elec, 0.01f);
    DOUBLES_EQUAL_PERC(sys_sim->m_bldc.m_w_r.elec, vars.w_hall.elec, 0.05f); // verify hall loop is locked
    DOUBLES_EQUAL_PERC(sys_sim->m_bldc.m_w_r.elec, vars.w_final.elec, 0.05f); // verify hall loop is locked
    DOUBLES_EQUAL_PERC(sys_sim->m_bldc.m_w_r.elec, vars.w_final_filt.elec, 0.05f); // verify hall loop is locked
    DOUBLES_EQUAL_PERC(vars.w_cmd_int.elec, vars.w_final_filt.elec, 0.01f); // check controller
    DOUBLES_EQUAL_PERC(vars.w_cmd_int.elec, sys_sim->m_bldc.m_w_r.elec, 0.01f);
    DOUBLES_EQUAL_PERC(sys_sim->m_bldc.m_T, vars.T_est, 0.10f);
    DOUBLES_EQUAL(ABS(vars.i_cmd_int) * ONE_OVER_SQRT_TWO, sys_sim->m_bldc.m_i_s, 15.0f); // check controller
    ANGLE_EQUAL(sys_sim->m_bldc.m_th_r.elec, vars.th_r_hall.elec, DEG_TO_RAD(7.5f)); // verify hall loop is locked

    // Speed-CL state: steady-state: ramp down
    while (sm.current == Speed_CL)
    {
        sys_sim->RunOneTick();
    }
    auto speed_cl_time = sys_sim->GetDiffTime();
    DOUBLES_EQUAL_PERC(vars.w_cmd_ext.elec, sm.vars.capture_vals[0], 0.01f);
    DOUBLES_EQUAL_PERC(0.0f, vars.w_cmd_int.elec, 0.01f);
    ANGLE_EQUAL(sys_sim->m_bldc.m_th_r.elec, sm.vars.capture_vals[1], DEG_TO_RAD(20.0f)); // vars.th_r_hall.elec
    DOUBLES_EQUAL_PERC(sys_sim->m_bldc.m_w_r.elec, sm.vars.capture_vals[2], 0.01f * params.motor.w_nom.elec); // vars.w_hall.elec

    // Back to brake-boot state
    sys_sim->RunUntil(sys_sim->m_pot.m_t_stop + sys_sim->m_pot.m_t_slope);
    DOUBLES_EQUAL_PERC(0.0f, vars.w_cmd_int.elec, 0.01f);
    DOUBLES_EQUAL(0.0f, sys_sim->m_bldc.m_i_s, 0.01f);
    DOUBLES_EQUAL(0.0f, sys_sim->m_bldc.m_w_r.elec, 0.1f);
    DOUBLES_EQUAL(0.0f, sys_sim->m_bldc.m_T, 0.01f);
};


TEST(SpeedModeTrapCommHallTest, ClosedLoopNegativeSpeedCmd)
{
    sensor_iface.digital.dir = 0U;
    float dir = (sensor_iface.digital.dir == 1U) ? (+1.0f) : (-1.0f);

    // Setting capture channels for capturing the variables before state transitions
    sm.vars.capture_channels[0] = &vars.w_cmd_int.elec;
    sm.vars.capture_channels[1] = &vars.th_r_hall.elec;
    sm.vars.capture_channels[2] = &vars.w_hall.elec;

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
    CHECK_TRUE(sm.current == Speed_CL);
    CHECK_EQUAL(StopWatchIsDone(&sm.vars.brake_boot.timer), true);
    CHECK_TRUE(brake_boot_time >= params.sys.boot_time);
    DOUBLES_EQUAL_PERC(params.ctrl.volt.w_thresh.elec * dir, vars.w_cmd_ext.elec, 0.01f);
    DOUBLES_EQUAL_PERC(params.ctrl.volt.w_thresh.elec * dir, vars.w_cmd_int.elec, 0.01f);
    DOUBLES_EQUAL(0.0f, sm.vars.capture_vals[0], 0.01f); // vars.w_cmd_int
    ANGLE_EQUAL(Initial_Angle.elec, sys_sim->m_bldc.m_th_r.elec, DEG_TO_RAD(0.5f));
    ANGLE_EQUAL(sys_sim->m_bldc.m_th_r.elec, vars.th_r_fb.elec, DEG_TO_RAD(0.5f));
    ANGLE_EQUAL(sys_sim->m_bldc.m_th_r.elec, vars.th_r_hall.elec, DEG_TO_RAD(30.0f));
    ANGLE_EQUAL(sys_sim->m_bldc.m_th_r.elec, vars.th_r_final.elec, DEG_TO_RAD(30.0f));

    // Speed-CL state: ramp up
    sys_sim->RunUntil(sys_sim->m_pot.m_t_start + sys_sim->m_pot.m_t_slope);
    DOUBLES_EQUAL_PERC(MECH_TO_ELEC(params.sys.cmd.w_max.mech, params.motor.P) * dir, vars.w_cmd_int.elec, 0.01f);
    DOUBLES_EQUAL_PERC(sys_sim->m_bldc.m_w_r.elec, vars.w_hall.elec, 0.05f); // verify hall loop is locked
    DOUBLES_EQUAL_PERC(sys_sim->m_bldc.m_w_r.elec, vars.w_final.elec, 0.05f); // verify hall loop is locked
    DOUBLES_EQUAL_PERC(sys_sim->m_bldc.m_w_r.elec, vars.w_final_filt.elec, 0.05f); // verify hall loop is locked
    DOUBLES_EQUAL_PERC(vars.w_cmd_int.elec, vars.w_final_filt.elec, 0.01f); // check controller
    DOUBLES_EQUAL_PERC(vars.w_cmd_int.elec, sys_sim->m_bldc.m_w_r.elec, 0.01f);
    DOUBLES_EQUAL_PERC(sys_sim->m_bldc.m_T, vars.T_est, 0.10f);
    DOUBLES_EQUAL(ABS(vars.i_cmd_int) * ONE_OVER_SQRT_TWO, sys_sim->m_bldc.m_i_s, 15.0f); // check controller
    ANGLE_EQUAL(sys_sim->m_bldc.m_th_r.elec, vars.th_r_hall.elec, DEG_TO_RAD(7.5f)); // verify hall loop is locked

    // Speed-CL state: steady-state
    sys_sim->RunUntil(sys_sim->m_pot.m_t_stop);
    DOUBLES_EQUAL_PERC(MECH_TO_ELEC(params.sys.cmd.w_max.mech, params.motor.P) * dir, vars.w_cmd_int.elec, 0.01f);
    DOUBLES_EQUAL_PERC(sys_sim->m_bldc.m_w_r.elec, vars.w_hall.elec, 0.05f); // verify hall loop is locked
    DOUBLES_EQUAL_PERC(sys_sim->m_bldc.m_w_r.elec, vars.w_final.elec, 0.05f); // verify hall loop is locked
    DOUBLES_EQUAL_PERC(sys_sim->m_bldc.m_w_r.elec, vars.w_final_filt.elec, 0.05f); // verify hall loop is locked
    DOUBLES_EQUAL_PERC(vars.w_cmd_int.elec, vars.w_final_filt.elec, 0.01f); // check controller
    DOUBLES_EQUAL_PERC(vars.w_cmd_int.elec, sys_sim->m_bldc.m_w_r.elec, 0.01f);
    DOUBLES_EQUAL_PERC(sys_sim->m_bldc.m_T, vars.T_est, 0.10f);
    DOUBLES_EQUAL(ABS(vars.i_cmd_int) * ONE_OVER_SQRT_TWO, sys_sim->m_bldc.m_i_s, 15.0f); // check controller
    ANGLE_EQUAL(sys_sim->m_bldc.m_th_r.elec, vars.th_r_hall.elec, DEG_TO_RAD(7.5f)); // verify hall loop is locked

    // Speed-CL state: steady-state: ramp down
    while (sm.current == Speed_CL)
    {
        sys_sim->RunOneTick();
    }
    auto speed_cl_time = sys_sim->GetDiffTime();
    DOUBLES_EQUAL_PERC(vars.w_cmd_ext.elec, sm.vars.capture_vals[0], 0.01f);
    DOUBLES_EQUAL_PERC(0.0f, vars.w_cmd_int.elec, 0.01f);
    ANGLE_EQUAL(sys_sim->m_bldc.m_th_r.elec, sm.vars.capture_vals[1], DEG_TO_RAD(20.0f)); // vars.th_r_hall.elec
    DOUBLES_EQUAL_PERC(sys_sim->m_bldc.m_w_r.elec, sm.vars.capture_vals[2], 0.01f * params.motor.w_nom.elec); // vars.w_hall.elec

    // Back to brake-boot state
    sys_sim->RunUntil(sys_sim->m_pot.m_t_stop + sys_sim->m_pot.m_t_slope);
    DOUBLES_EQUAL_PERC(0.0f, vars.w_cmd_int.elec, 0.01f);
    DOUBLES_EQUAL(0.0f, sys_sim->m_bldc.m_i_s, 0.01f);
    DOUBLES_EQUAL(0.0f, sys_sim->m_bldc.m_w_r.elec, 0.1f);
    DOUBLES_EQUAL(0.0f, sys_sim->m_bldc.m_T, 0.01f);
};

#endif