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
#include "CDyno.h"

using namespace ExportData;

TEST_GROUP(CDynoTest)
{
    float* T_motor;
    CDyno* dyno;

    void setup()
    {
        PARAMS_DEFAULT_InitManual();
        params.sys.samp.fs0 = 1.0E3;
        params.sys.samp.fs0_fs1_ratio = 10U;
        params.sys.samp.fsim_fs0_ratio = 1U;
        params.mech.inertia = 0.5f;
        PARAMS_DEFAULT_InitAutoCalc();

        T_motor = new float;
        *T_motor = 0.0f;
        dyno = new CDyno();
        dyno->SetInputPointers(&params, T_motor);
    }
    void teardown()
    {
        delete T_motor;
        delete dyno;
    }
};


TEST(CDynoTest, TrqCtrlSim)
{
    // Dyno's control knob:
    dyno->m_pot.m_init = 0.0f;
    dyno->m_pot.m_final = 1.0f;
    dyno->m_pot.m_t_start = 0.5f;
    dyno->m_pot.m_t_slope = 1.5f;
    dyno->m_pot.m_t_stop = 3.0f;
    dyno->m_T_cmd = 100.0f;
    const float Run_Time = 5.0f;

    // Dyno's control mode:
    dyno->SetCtrlMode(CDyno::ECtrlMode::Torque_Ctrl);

    // Run simulation:
    vector<float> time, T, w, th;
    for (uint32_t sim_cntr = 0U; sim_cntr < (Run_Time * params.sys.samp.fsim); ++sim_cntr)
    {
        dyno->RunSim();
        time.push_back(static_cast<float>(sim_cntr) * params.sys.samp.tsim);
        T.push_back(dyno->m_T);
        w.push_back(dyno->m_w.mech);
        th.push_back(dyno->m_th.mech);
    }

    // Export data for verification:
    DATA_SET_t<float> waveforms = {
        DATA_PAIR_t<float>("time", time),
        DATA_PAIR_t<float>("T", T),
        DATA_PAIR_t<float>("w", w),
        DATA_PAIR_t<float>("th", th)
    };
    ExportDataToCsvFile("CDynoTestTrqCtrl.csv", waveforms);
}

TEST(CDynoTest, SpdCtrlSim)
{
    // Dyno's control knob:
    dyno->m_pot.m_init = 0.0f;
    dyno->m_pot.m_final = 1.0f;
    dyno->m_pot.m_t_start = 0.5f;
    dyno->m_pot.m_t_slope = 1.5f;
    dyno->m_pot.m_t_stop = 3.0f;
    dyno->m_w_cmd.mech = 100.0f;
    const float Run_Time = 5.0f;

    // Dyno's control mode:
    dyno->SetCtrlMode(CDyno::ECtrlMode::Speed_Ctrl);

    // Run simulation:
    vector<float> time, T, w, th;
    for (uint32_t sim_cntr = 0U; sim_cntr < (Run_Time * params.sys.samp.fsim); ++sim_cntr)
    {
        dyno->RunSim();
        time.push_back(static_cast<float>(sim_cntr) * params.sys.samp.tsim);
        T.push_back(dyno->m_T);
        w.push_back(dyno->m_w.mech);
        th.push_back(dyno->m_th.mech);
    }

    // Export data for verification:
    DATA_SET_t<float> waveforms = {
        DATA_PAIR_t<float>("time", time),
        DATA_PAIR_t<float>("T", T),
        DATA_PAIR_t<float>("w", w),
        DATA_PAIR_t<float>("th", th)
    };
    ExportDataToCsvFile("CDynoTestSpdCtrl.csv", waveforms);
}

TEST(CDynoTest, PosCtrlSim)
{
    // Dyno's control knob:
    dyno->m_pot.m_init = 0.0f;
    dyno->m_pot.m_final = 1.0f;
    dyno->m_pot.m_t_start = 0.5f;
    dyno->m_pot.m_t_slope = 1.5f;
    dyno->m_pot.m_t_stop = 3.0f;
    dyno->m_th_cmd.mech = 100.0f;
    const float Run_Time = 5.0f;

    // Dyno's control mode:
    dyno->SetCtrlMode(CDyno::ECtrlMode::Position_Ctrl);

    // Run simulation:
    vector<float> time, T, w, th;
    for (uint32_t sim_cntr = 0U; sim_cntr < (Run_Time * params.sys.samp.fsim); ++sim_cntr)
    {
        dyno->RunSim();
        time.push_back(static_cast<float>(sim_cntr) * params.sys.samp.tsim);
        T.push_back(dyno->m_T);
        w.push_back(dyno->m_w.mech);
        th.push_back(dyno->m_th.mech);
    }

    // Export data for verification:
    DATA_SET_t<float> waveforms = {
        DATA_PAIR_t<float>("time", time),
        DATA_PAIR_t<float>("T", T),
        DATA_PAIR_t<float>("w", w),
        DATA_PAIR_t<float>("th", th)
    };
    ExportDataToCsvFile("CDynoTestPosCtrl.csv", waveforms);
}