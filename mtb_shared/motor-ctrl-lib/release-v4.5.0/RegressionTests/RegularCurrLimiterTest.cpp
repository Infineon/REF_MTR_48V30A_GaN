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


#if defined(CTRL_METHOD_RFO)

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

TEST_GROUP(RegularCurrLimiterTest)
{
    CSysSim* sys_sim;
    const float Epsilon = 1.0E-6f;

    void setup()
    {
        sys_sim = new CSysSim();
        sys_sim->SetDiscreteIntegMethod(CDiscreteInteg::EMethod::Backward_Euler);

        params.ctrl.mode = Curr_Mode_FOC_Sensorless_Align_Startup;
        params.sys.cmd.i_max = 70.0f; // between motor's continuous and peak current
        params.motor.i2t.therm_tau = 1.0f; // shorter time constant to reduce simulation time
        PARAMS_DEFAULT_InitAutoCalc();
        STATE_MACHINE_Init();

    }

    void teardown()
    {
        delete sys_sim;
    }

};

TEST(RegularCurrLimiterTest, PositiveCurrentCmd)
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
    sys_sim->RunOneTick();
    CHECK_TRUE(sm.current == Current_CL);
    float i2t_en_time = sys_sim->GetDiffTime();
    DOUBLES_EQUAL_PERC(protect.motor.i2t.i_on, protect.motor.i2t.i_filt, 0.01f); // i2t triggered and engaged
    DOUBLES_EQUAL_PERC(params.motor.i_cont, protect.motor.i2t.i_limit, 0.01f); // current cap reduced
    DOUBLES_EQUAL_PERC(params.motor.i_cont * dir, vars.i_cmd_prot, 0.01f); // internal command before rate limiter
    DOUBLES_EQUAL_PERC(vars.i_cmd_ext, vars.i_cmd_int, 0.01f); // internal command after rate limiter

    // Check currents after current controller reaches the new equilibrium point
    const float Rate_Limit_Time = (ABS(vars.i_cmd_ext) - params.motor.i_cont) / params.sys.rate_lim.i_cmd;
    const float Curr_Cntrl_Settle_Time = 6.0E-3f; // multiple of the controller's time constants
    sys_sim->RunFor(Rate_Limit_Time + Curr_Cntrl_Settle_Time);
    CHECK_TRUE(sm.current == Current_CL);
    float curr_ctrl_reach_time = sys_sim->GetDiffTime();
    CHECK_TRUE(protect.motor.i2t.state == En); // i2t still engaged
    DOUBLES_EQUAL_PERC(params.motor.i_cont, protect.motor.i2t.i_limit, 0.01f); // current cap still reduced
    DOUBLES_EQUAL_PERC(params.motor.i_cont * dir, vars.i_cmd_prot, 0.01f);
    DOUBLES_EQUAL_PERC(params.motor.i_cont * dir, vars.i_cmd_int, 0.01f); //  internal current command capped
    DOUBLES_EQUAL_PERC(POW_TWO(params.motor.i_cont), vars.i_s_fb_sq, 0.01f); // current regulated to motor.i_cont (current cap)

    // Run until the current command is not capped anymore
    while (ABS(vars.i_cmd_int) <= ABS(vars.i_cmd_ext))
    {
        sys_sim->RunOneTick();
    }
    CHECK_TRUE(sm.current == Current_CL);
    float remove_curr_cap_time = sys_sim->GetDiffTime();
    DOUBLES_EQUAL_PERC(vars.i_cmd_ext, vars.i_cmd_prot, 0.01f); // current limitation about to be removed
    DOUBLES_EQUAL_PERC(vars.i_cmd_ext, vars.i_cmd_int, 0.01f); // current limitation about to be removed
    CHECK_TRUE(protect.motor.i2t.state == En); // i2t still engaged
    DOUBLES_EQUAL_PERC(params.motor.i_cont, protect.motor.i2t.i_limit, 0.01f); // current cap still reduced
    DOUBLES_EQUAL_PERC(POW_TWO(params.motor.i_cont), vars.i_s_fb_sq, 0.01f); // current still regulated to motor.i_cont (current cap)

    // Run until motor i2t protection disengages
    while (protect.motor.i2t.state == En)
    {
        sys_sim->RunOneTick();
    }
    CHECK_TRUE(sm.current == Current_CL);
    float i2t_disengage_time = sys_sim->GetDiffTime();
    DOUBLES_EQUAL_PERC(vars.i_cmd_ext, vars.i_cmd_prot, 0.01f); // current limitation still removed
    DOUBLES_EQUAL_PERC(vars.i_cmd_ext, vars.i_cmd_int, 0.01f); // current limitation still removed
    DOUBLES_EQUAL_PERC(protect.motor.i2t.i_off, protect.motor.i2t.i_filt, 0.01f); // i2t triggered to disengage
    DOUBLES_EQUAL_PERC(params.motor.i_peak, protect.motor.i2t.i_limit, 0.01f); // current cap removed

    // Run until back to brake-boot-state
    sys_sim->RunUntil(sys_sim->m_pot.m_t_stop + sys_sim->m_pot.m_t_slope);
    DOUBLES_EQUAL(0.0f, sys_sim->m_pmsm.m_w_r.elec, 0.1f);
    DOUBLES_EQUAL_PERC(0.0f, vars.i_cmd_int, 0.01f);
    DOUBLES_EQUAL(0.0f, sys_sim->m_pmsm.m_T, 0.01f);
    DOUBLES_EQUAL(0.0f, sys_sim->m_pmsm.m_i_qd_s.q, 0.01f);
    DOUBLES_EQUAL(0.0f, sys_sim->m_pmsm.m_i_qd_s.d, 0.01f);

}


TEST(RegularCurrLimiterTest, NegativeCurrentCmd)
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
    sys_sim->RunOneTick();
    CHECK_TRUE(sm.current == Current_CL);
    float i2t_en_time = sys_sim->GetDiffTime();
    DOUBLES_EQUAL_PERC(protect.motor.i2t.i_on, protect.motor.i2t.i_filt, 0.01f); // i2t triggered and engaged
    DOUBLES_EQUAL_PERC(params.motor.i_cont, protect.motor.i2t.i_limit, 0.01f); // current cap reduced
    DOUBLES_EQUAL_PERC(params.motor.i_cont * dir, vars.i_cmd_prot, 0.01f); // internal command before rate limiter
    DOUBLES_EQUAL_PERC(vars.i_cmd_ext, vars.i_cmd_int, 0.01f); // internal command after rate limiter

    // Check currents after current controller reaches the new equilibrium point
    const float Rate_Limit_Time = (ABS(vars.i_cmd_ext) - params.motor.i_cont) / params.sys.rate_lim.i_cmd;
    const float Curr_Cntrl_Settle_Time = 6.0E-3f; // multiple of the controller's time constants
    sys_sim->RunFor(Rate_Limit_Time + Curr_Cntrl_Settle_Time);
    CHECK_TRUE(sm.current == Current_CL);
    float curr_ctrl_reach_time = sys_sim->GetDiffTime();
    CHECK_TRUE(protect.motor.i2t.state == En); // i2t still engaged
    DOUBLES_EQUAL_PERC(params.motor.i_cont, protect.motor.i2t.i_limit, 0.01f); // current cap still reduced
    DOUBLES_EQUAL_PERC(params.motor.i_cont * dir, vars.i_cmd_prot, 0.01f);
    DOUBLES_EQUAL_PERC(params.motor.i_cont * dir, vars.i_cmd_int, 0.01f); //  internal current command capped
    DOUBLES_EQUAL_PERC(POW_TWO(params.motor.i_cont), vars.i_s_fb_sq, 0.01f); // current regulated to motor.i_cont (current cap)

    // Run until the current command is not capped anymore
    while (ABS(vars.i_cmd_int) <= ABS(vars.i_cmd_ext))
    {
        sys_sim->RunOneTick();
    }
    CHECK_TRUE(sm.current == Current_CL);
    float remove_curr_cap_time = sys_sim->GetDiffTime();
    DOUBLES_EQUAL_PERC(vars.i_cmd_ext, vars.i_cmd_prot, 0.01f); // current limitation about to be removed
    DOUBLES_EQUAL_PERC(vars.i_cmd_ext, vars.i_cmd_int, 0.01f); // current limitation about to be removed
    CHECK_TRUE(protect.motor.i2t.state == En); // i2t still engaged
    DOUBLES_EQUAL_PERC(params.motor.i_cont, protect.motor.i2t.i_limit, 0.01f); // current cap still reduced
    DOUBLES_EQUAL_PERC(POW_TWO(params.motor.i_cont), vars.i_s_fb_sq, 0.01f); // current still regulated to motor.i_cont (current cap)

    // Run until motor i2t protection disengages
    while (protect.motor.i2t.state == En)
    {
        sys_sim->RunOneTick();
    }
    CHECK_TRUE(sm.current == Current_CL);
    float i2t_disengage_time = sys_sim->GetDiffTime();
    DOUBLES_EQUAL_PERC(vars.i_cmd_ext, vars.i_cmd_prot, 0.01f); // current limitation still removed
    DOUBLES_EQUAL_PERC(vars.i_cmd_ext, vars.i_cmd_int, 0.01f); // current limitation still removed
    DOUBLES_EQUAL_PERC(protect.motor.i2t.i_off, protect.motor.i2t.i_filt, 0.01f); // i2t triggered to disengage
    DOUBLES_EQUAL_PERC(params.motor.i_peak, protect.motor.i2t.i_limit, 0.01f); // current cap removed

    // Run until back to brake-boot-state
    sys_sim->RunUntil(sys_sim->m_pot.m_t_stop + sys_sim->m_pot.m_t_slope);
    DOUBLES_EQUAL(0.0f, sys_sim->m_pmsm.m_w_r.elec, 0.1f);
    DOUBLES_EQUAL_PERC(0.0f, vars.i_cmd_int, 0.01f);
    DOUBLES_EQUAL(0.0f, sys_sim->m_pmsm.m_T, 0.01f);
    DOUBLES_EQUAL(0.0f, sys_sim->m_pmsm.m_i_qd_s.q, 0.01f);
    DOUBLES_EQUAL(0.0f, sys_sim->m_pmsm.m_i_qd_s.d, 0.01f);

}


#endif