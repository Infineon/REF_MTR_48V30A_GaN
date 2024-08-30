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

static void CallbackISR0()
{
    SENSOR_IFACE_RunISR0();
    SIX_PULSE_INJ_RunISR0();
}

static void CallbackISR1()
{
    SENSOR_IFACE_RunISR1();
    SIX_PULSE_INJ_RunISR1();
}

TEST_GROUP(SixPulseInjTest)
{
    CSysSim* sys_sim;

    void setup()
    {
        sys_sim = new CSysSim();

        // PWM & ISR0 @ 10kHz, ISR1 @ 1kHz, Simulation @ 100kHz:
        params.sys.samp.fs0 = 10.0E3f;
        params.sys.samp.fs0_fs1_ratio = 10U;
        params.sys.samp.fsim_fs0_ratio = 10U;
        PARAMS_DEFAULT_InitAutoCalc();
        STATE_MACHINE_Init();

        // Only simulating SixPulseInj.h/.c:
        sys_sim->RegisterCallbackISR0(&CallbackISR0);
        sys_sim->RegisterCallbackISR1(&CallbackISR1);

        // Lock rotor
        sys_sim->m_mech_load.LockPosition();

    }

    void teardown()
    {
        delete sys_sim;
    }
};

TEST(SixPulseInjTest, InitialAngleIPM)
{
    // IPM: Lq!=Ld
    params.motor.lq = 28.0E-6f; // [H]
    params.motor.ld = 23.0E-6f; // [H]
    params.motor.r = 11.0E-3f; // [Ohm]
    PARAMS_DEFAULT_InitAutoCalc();
    const float Run_Time = 6.0f * (params.ctrl.six_pulse_inj.t_on + params.ctrl.six_pulse_inj.t_off) + 0.5f;

    // Init
    SENSOR_IFACE_Init();
    SENSOR_IFACE_Reset();
    SIX_PULSE_INJ_Init();
    SIX_PULSE_INJ_Reset();

    // Enable modeling of motor inducatance saturation (lq/ld)
    const QD_t I_QD_R_Sat = { params.motor.i_peak * 4.0f, params.motor.i_peak * 4.0f };
    sys_sim->m_pmsm.SetSaturationParams(En, I_QD_R_Sat);

    const int32_t N = 72; // Testing every initial angle in the range [0,2*PI) with step size = 2*PI/N
    ELEC_t th_r_expected = { 0.0f };

    for (int32_t index = 0; index < N; ++index)
    {
        // Take rotor to the initial angle to be tested
        th_r_expected.elec = index * (PI / N);
        sys_sim->ResetSim({ 0.0f }, { ELEC_TO_MECH(th_r_expected.elec, params.motor.P) });
        SIX_PULSE_INJ_Reset();

        // Run simulation
        sys_sim->RunFor(Run_Time);

        // Verify results
        ANGLE_EQUAL(th_r_expected.elec, ctrl.six_pulse_inj.th_r_est.elec, DEG_TO_RAD(15.01f)); // (+/-)15deg accuracy max
    }
}

TEST(SixPulseInjTest, InitialAngleSPM)
{
    // SPM: Lq==Ld
    params.motor.lq = 25.0E-6f; // [H]
    params.motor.ld = 25.0E-6f; // [H]
    params.motor.r = 11.0E-3f; // [Ohm]
    PARAMS_DEFAULT_InitAutoCalc();
    const float Run_Time = 6.0f * (params.ctrl.six_pulse_inj.t_on + params.ctrl.six_pulse_inj.t_off) + 0.5f;

    // Init
    SENSOR_IFACE_Init();
    SENSOR_IFACE_Reset();
    SIX_PULSE_INJ_Init();
    SIX_PULSE_INJ_Reset();

    // Enable modeling of motor inducatance saturation (lq/ld)
    const QD_t I_QD_R_Sat = { params.motor.i_peak * 4.0f, params.motor.i_peak * 4.0f };
    sys_sim->m_pmsm.SetSaturationParams(En, I_QD_R_Sat);

    const int32_t N = 72; // Testing every initial angle in the range [0,2*PI) with step size = 2*PI/N
    ELEC_t th_r_expected = { 0.0f };

    for (int32_t index = 0; index < N; ++index)
    {
        // Take rotor to the initial angle to be tested
        th_r_expected.elec = index * (PI / N);
        sys_sim->ResetSim({ 0.0f }, { ELEC_TO_MECH(th_r_expected.elec, params.motor.P) });
        SIX_PULSE_INJ_Reset();

        // Run simulation
        sys_sim->RunFor(Run_Time);

        // Verify results
        ANGLE_EQUAL(th_r_expected.elec, ctrl.six_pulse_inj.th_r_est.elec, DEG_TO_RAD(15.01f)); // (+/-)15deg accuracy max
    }
}

#endif