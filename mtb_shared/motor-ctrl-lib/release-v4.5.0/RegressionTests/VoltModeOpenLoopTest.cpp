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

TEST_GROUP(VoltModeOpenLoopTest)
{
    CSysSim* sys_sim;

    void setup()
    {
        sys_sim = new CSysSim();
#if defined(CTRL_METHOD_TBC)
        sys_sim->SetFeedbackType(CFeedback::EType::Hall_Sensor);
#endif
        params.ctrl.mode = Volt_Mode_Open_Loop;
        PARAMS_DEFAULT_InitAutoCalc();
        STATE_MACHINE_Init();
    }

    void teardown()
    {
        delete sys_sim;
    }

};

TEST(VoltModeOpenLoopTest, PositiveSpeedCmd)
{
    sensor_iface.digital.dir = 1U;
    float dir = (sensor_iface.digital.dir == 1U) ? (+1.0f) : (-1.0f);

    // Setting capture channels for capturing the variables before state transitions
    sm.vars.capture_channels[0] = &vars.w_cmd_int.elec;
    sm.vars.capture_channels[1] = &vars.w_final.elec;
    sm.vars.capture_channels[2] = &vars.w_final_filt.elec;

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
    DOUBLES_EQUAL(0.0f, sm.vars.capture_vals[1], 0.1f); // vars.w_final.elec
    DOUBLES_EQUAL(0.0f, sm.vars.capture_vals[2], 0.1f); // vars.w_final_filt.elec

    // Volt/Hz OL state
    sys_sim->RunUntil(sys_sim->m_pot.m_t_start + sys_sim->m_pot.m_t_slope);
    CHECK_TRUE(sm.current == Volt_Hz_OL);
    float acc_cmd_elec_expected = (sys_sim->m_pot.m_final - sys_sim->m_pot.m_init) / sys_sim->m_pot.m_t_slope
        * MECH_TO_ELEC(params.sys.cmd.w_max.mech * dir * sys_sim->m_pot.m_final, params.motor.P);
    DOUBLES_EQUAL_PERC(acc_cmd_elec_expected, vars.acc_cmd_int.elec, 0.1f);

    sys_sim->RunUntil(sys_sim->m_pot.m_t_stop);
    CHECK_TRUE(sm.current == Volt_Hz_OL);
    auto w_cmd_elec_expected = MECH_TO_ELEC(params.sys.cmd.w_max.mech * dir * sys_sim->m_pot.m_final, params.motor.P);
    DOUBLES_EQUAL_PERC(params.sys.cmd.w_max.mech * dir * sys_sim->m_pot.m_final, sys_sim->m_mech_load.m_w.mech, 0.01f);
    DOUBLES_EQUAL_PERC(w_cmd_elec_expected, sys_sim->m_pmsm.m_w_r.elec, 0.01f);
    DOUBLES_EQUAL_PERC(w_cmd_elec_expected, vars.w_cmd_int.elec, 0.01f);
    DOUBLES_EQUAL_PERC(w_cmd_elec_expected, vars.w_final.elec, 0.01f);
    DOUBLES_EQUAL_PERC(w_cmd_elec_expected, vars.w_final_filt.elec, 0.01f);

    while (sm.current == Volt_Hz_OL)
    {
        sys_sim->RunOneTick();
    }
    auto volt_hz_time = sys_sim->GetDiffTime();
    CHECK_TRUE(sm.current == Brake_Boot);
    float w_thresh_elec_expected = (params.ctrl.volt.w_thresh.elec - params.ctrl.volt.w_hyst.elec) * dir;
    DOUBLES_EQUAL(w_thresh_elec_expected, MECH_TO_ELEC(sys_sim->m_mech_load.m_w.mech, params.motor.P), 15.0f);
    DOUBLES_EQUAL(w_thresh_elec_expected, sys_sim->m_pmsm.m_w_r.elec, 15.0f);
    DOUBLES_EQUAL(0.0f, vars.w_cmd_int.elec, 0.001f);
    DOUBLES_EQUAL(0.0f, vars.w_final_filt.elec, 0.001f);
    DOUBLES_EQUAL(w_thresh_elec_expected, sm.vars.capture_vals[0], 15.0f);	// vars.w_cmd_int.elec
    DOUBLES_EQUAL(w_thresh_elec_expected, sm.vars.capture_vals[1], 15.0f);	// vars.w_final.elec
    DOUBLES_EQUAL(w_thresh_elec_expected, sm.vars.capture_vals[2], 25.0f);	// vars.w_final_filt.elec

    // Back to brake-boot state
    sys_sim->RunUntil(sys_sim->m_pot.m_t_stop + sys_sim->m_pot.m_t_slope);
    DOUBLES_EQUAL(0.0f, sys_sim->m_mech_load.m_w.mech, 0.1f);
    DOUBLES_EQUAL(0.0f, sys_sim->m_pmsm.m_w_r.elec, 0.1f);
    DOUBLES_EQUAL(0.0f, vars.w_cmd_int.elec, 0.1f);
    DOUBLES_EQUAL(0.0f, vars.w_final_filt.elec, 0.1f);

}

TEST(VoltModeOpenLoopTest, NegativeSpeedCmd)
{
    sensor_iface.digital.dir = 0U;
    float dir = (sensor_iface.digital.dir == 1U) ? (+1.0f) : (-1.0f);

    // Setting capture channels for capturing the variables before state transitions
    sm.vars.capture_channels[0] = &vars.w_cmd_int.elec;
    sm.vars.capture_channels[1] = &vars.w_final.elec;
    sm.vars.capture_channels[2] = &vars.w_final_filt.elec;

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
    DOUBLES_EQUAL(0.0f, sm.vars.capture_vals[1], 0.1f); // vars.w_final.elec
    DOUBLES_EQUAL(0.0f, sm.vars.capture_vals[2], 0.1f); // vars.w_final_filt.elec

    // Volt/Hz OL state
    sys_sim->RunUntil(sys_sim->m_pot.m_t_start + sys_sim->m_pot.m_t_slope);
    CHECK_TRUE(sm.current == Volt_Hz_OL);
    float acc_cmd_elec_expected = (sys_sim->m_pot.m_final - sys_sim->m_pot.m_init) / sys_sim->m_pot.m_t_slope
        * MECH_TO_ELEC(params.sys.cmd.w_max.mech * dir * sys_sim->m_pot.m_final, params.motor.P);
    DOUBLES_EQUAL_PERC(acc_cmd_elec_expected, vars.acc_cmd_int.elec, 0.1f);

    sys_sim->RunUntil(sys_sim->m_pot.m_t_stop);
    CHECK_TRUE(sm.current == Volt_Hz_OL);
    auto w_cmd_elec_expected = MECH_TO_ELEC(params.sys.cmd.w_max.mech * dir * sys_sim->m_pot.m_final, params.motor.P);
    DOUBLES_EQUAL_PERC(params.sys.cmd.w_max.mech * dir * sys_sim->m_pot.m_final, sys_sim->m_mech_load.m_w.mech, 0.01f);
    DOUBLES_EQUAL_PERC(w_cmd_elec_expected, sys_sim->m_pmsm.m_w_r.elec, 0.01f);
    DOUBLES_EQUAL_PERC(w_cmd_elec_expected, vars.w_cmd_int.elec, 0.01f);
    DOUBLES_EQUAL_PERC(w_cmd_elec_expected, vars.w_final.elec, 0.01f);
    DOUBLES_EQUAL_PERC(w_cmd_elec_expected, vars.w_final_filt.elec, 0.01f);

    while (sm.current == Volt_Hz_OL)
    {
        sys_sim->RunOneTick();
    }
    auto volt_hz_time = sys_sim->GetDiffTime();
    CHECK_TRUE(sm.current == Brake_Boot);
    float w_thresh_elec_expected = (params.ctrl.volt.w_thresh.elec - params.ctrl.volt.w_hyst.elec) * dir;
    DOUBLES_EQUAL(w_thresh_elec_expected, MECH_TO_ELEC(sys_sim->m_mech_load.m_w.mech, params.motor.P), 15.0f);
    DOUBLES_EQUAL(w_thresh_elec_expected, sys_sim->m_pmsm.m_w_r.elec, 15.0f);
    DOUBLES_EQUAL(0.0f, vars.w_cmd_int.elec, 0.001f);
    DOUBLES_EQUAL(0.0f, vars.w_final_filt.elec, 0.001f);
    DOUBLES_EQUAL(w_thresh_elec_expected, sm.vars.capture_vals[0], 15.0f);	// vars.w_cmd_int.elec
    DOUBLES_EQUAL(w_thresh_elec_expected, sm.vars.capture_vals[1], 15.0f);	// vars.w_final.elec
    DOUBLES_EQUAL(w_thresh_elec_expected, sm.vars.capture_vals[2], 25.0f);	// vars.w_final_filt.elec

    // Back to brake-boot state
    sys_sim->RunUntil(sys_sim->m_pot.m_t_stop + sys_sim->m_pot.m_t_slope);
    DOUBLES_EQUAL(0.0f, sys_sim->m_mech_load.m_w.mech, 0.1f);
    DOUBLES_EQUAL(0.0f, sys_sim->m_pmsm.m_w_r.elec, 0.1f);
    DOUBLES_EQUAL(0.0f, vars.w_cmd_int.elec, 0.1f);
    DOUBLES_EQUAL(0.0f, vars.w_final_filt.elec, 0.1f);

}
