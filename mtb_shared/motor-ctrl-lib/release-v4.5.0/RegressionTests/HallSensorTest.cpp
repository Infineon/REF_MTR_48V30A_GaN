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


#if defined(CTRL_METHOD_RFO) || defined(CTRL_METHOD_TBC)

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

using namespace ExportData;

static void CallbackISR0()
{
    HALL_SENSOR_RunISR0();
}

TEST_GROUP(HallSensorTest)
{
    CSysSim* sys_sim;

    void setup()
    {
        sys_sim = new CSysSim();
        sys_sim->SetFeedbackType(CFeedback::EType::Hall_Sensor);
        sys_sim->RegisterCallbackISR0(&CallbackISR0); // only simulating HallSensor.h/.c
        sys_sim->RegisterCallbackISR1(&EmptyFcn);

        params.sys.samp.fs0 = 20.0E3f;
        params.sys.samp.fs0_fs1_ratio = 10U;
        params.sys.samp.fsim_fs0_ratio = 5U;
        params.sys.fb.hall.w0_w = HZ_TO_RADSEC(100.0f); // [Ra/sec]
        params.sys.fb.hall.w0_th.min = HZ_TO_RADSEC(10.0f); // [Ra/sec]
        params.sys.fb.hall.w0_th.max = HZ_TO_RADSEC(100.0f); // [Ra/sec]
        params.sys.fb.hall.tau_ratio = 0.8f; // [sec/sec]=[]
        params.sys.fb.hall.w_thresh.elec = 200.0f; // [Ra/sec-elec]
        params.sys.fb.hall.deb_time = 1.0E-6f; // [sec]
        PARAMS_DEFAULT_InitAutoCalc();
        STATE_MACHINE_Init();

        hall.time_cap_freq = params.sys.samp.fsim;
        HALL_SENSOR_Init();
        HALL_SENSOR_Reset();
    }

    void teardown()
    {
        delete sys_sim;
    }
};

TEST(HallSensorTest, PositiveSpeed)
{
    const ELEC_t W0 = { HZ_TO_RADSEC(500.0f) };
    const uint32_t Check_Points = 50U;
    sys_sim->SetLoadType(CLoad::EType::Active_Dyno);
    sys_sim->m_dyno.SetCtrlMode(CDyno::ECtrlMode::Speed_Ctrl);
    sys_sim->m_dyno.Reset({ 0.0f }, { ELEC_TO_MECH(PI_OVER_FOUR,params.motor.P) });

    const float Run_Time = 100.0f * RADSEC_TO_PERIOD(ABS(W0.elec));
    sys_sim->m_dyno.m_w_cmd.mech = ELEC_TO_MECH(W0.elec, params.motor.P);
    sys_sim->m_dyno.m_pot.m_init = 0.0f;
    sys_sim->m_dyno.m_pot.m_final = 1.0f;
    sys_sim->m_dyno.m_pot.m_t_start = 0.0f;
    sys_sim->m_dyno.m_pot.m_t_slope = 0.3f * Run_Time;
    sys_sim->m_dyno.m_pot.m_t_stop = 0.4f * Run_Time;

    // Run simulation and record waveforms
    vector<float> time;
    vector<float> hall_signal_u, hall_signal_v, hall_signal_w, period;
    vector<float> w_fb, w_ff_filt, w_tot, th_r_fb, th_r_cap, th_r_est;

    while (sys_sim->GetAbsTime() < Run_Time)
    {
        // Run simulation
        sys_sim->RunOneTick();
        // Record waveforms
        time.push_back(sys_sim->GetAbsTime());
        hall_signal_u.push_back(static_cast<float>(hall.signal.u));
        hall_signal_v.push_back(static_cast<float>(hall.signal.v));
        hall_signal_w.push_back(static_cast<float>(hall.signal.w));
        period.push_back(static_cast<float>(hall.period_cap));
        w_fb.push_back(static_cast<float>(sys_sim->m_motor->m_w_r.elec));
        w_ff_filt.push_back(hall.w_ff_filt.elec);
        w_tot.push_back(hall.w_tot.elec);
        th_r_fb.push_back(vars.th_r_fb.elec);
        th_r_cap.push_back(hall.th_r_cap.elec);
        th_r_est.push_back(hall.th_r_est.elec);
    }

    // Export waveforms
    DATA_SET_t<float> waveforms = {
        DATA_PAIR_t<float>("time",					time),
        DATA_PAIR_t<float>("hall.signal.u",			hall_signal_u),
        DATA_PAIR_t<float>("hall.signal.v",			hall_signal_v),
        DATA_PAIR_t<float>("hall.signal.w",			hall_signal_w),
        DATA_PAIR_t<float>("period",				period),
        DATA_PAIR_t<float>("w_ff_filt",				w_ff_filt),
        DATA_PAIR_t<float>("w_tot",					w_tot),
        DATA_PAIR_t<float>("th_r_fb",				th_r_fb),
        DATA_PAIR_t<float>("th_r_cap",				th_r_cap),
        DATA_PAIR_t<float>("th_r_est",				th_r_est),
    };
    ExportDataToCsvFile("HallLowPositiveSpeed.csv", waveforms);

    const float Check_Start = sys_sim->m_dyno.m_pot.m_t_start + sys_sim->m_dyno.m_pot.m_t_slope;
    const float Check_Stop = sys_sim->m_dyno.m_pot.m_t_stop;
    const float Check_Duration = Check_Stop - Check_Start;
    for (float time = Check_Start; time < Check_Stop; time += (Check_Duration / Check_Points))
    {
        auto index = static_cast<uint32_t>(CSysSim::TimeToSysSimTick(time));
        ANGLE_EQUAL(th_r_fb.at(index), th_r_est.at(index), DEG_TO_RAD(15.0f));
        DOUBLES_EQUAL_PERC(w_fb.at(index), w_ff_filt.at(index), 0.05f);
    }
}

TEST(HallSensorTest, NegativeSpeed)
{
    const ELEC_t W0 = { HZ_TO_RADSEC(-500.0f) };
    const uint32_t Check_Points = 50U;
    sys_sim->SetLoadType(CLoad::EType::Active_Dyno);
    sys_sim->m_dyno.SetCtrlMode(CDyno::ECtrlMode::Speed_Ctrl);
    sys_sim->m_dyno.Reset({ 0.0f }, { ELEC_TO_MECH(PI_OVER_FOUR,params.motor.P) });

    const float Run_Time = 100.0f * RADSEC_TO_PERIOD(ABS(W0.elec));
    sys_sim->m_dyno.m_w_cmd.mech = ELEC_TO_MECH(W0.elec, params.motor.P);
    sys_sim->m_dyno.m_pot.m_init = 0.0f;
    sys_sim->m_dyno.m_pot.m_final = 1.0f;
    sys_sim->m_dyno.m_pot.m_t_start = 0.0f;
    sys_sim->m_dyno.m_pot.m_t_slope = 0.3f * Run_Time;
    sys_sim->m_dyno.m_pot.m_t_stop = 0.4f * Run_Time;

    // Run simulation and record waveforms
    vector<float> time;
    vector<float> hall_signal_u, hall_signal_v, hall_signal_w, period;
    vector<float> w_fb, w_ff_filt, w_tot, th_r_fb, th_r_cap, th_r_est;

    while (sys_sim->GetAbsTime() < Run_Time)
    {
        // Run simulation
        sys_sim->RunOneTick();
        // Record waveforms
        time.push_back(sys_sim->GetAbsTime());
        hall_signal_u.push_back(static_cast<float>(hall.signal.u));
        hall_signal_v.push_back(static_cast<float>(hall.signal.v));
        hall_signal_w.push_back(static_cast<float>(hall.signal.w));
        period.push_back(static_cast<float>(hall.period_cap));
        w_fb.push_back(static_cast<float>(sys_sim->m_motor->m_w_r.elec));
        w_ff_filt.push_back(hall.w_ff_filt.elec);
        w_tot.push_back(hall.w_tot.elec);
        th_r_fb.push_back(vars.th_r_fb.elec);
        th_r_cap.push_back(hall.th_r_cap.elec);
        th_r_est.push_back(hall.th_r_est.elec);
    }

    // Export waveforms
    DATA_SET_t<float> waveforms = {
        DATA_PAIR_t<float>("time",					time),
        DATA_PAIR_t<float>("hall.signal.u",			hall_signal_u),
        DATA_PAIR_t<float>("hall.signal.v",			hall_signal_v),
        DATA_PAIR_t<float>("hall.signal.w",			hall_signal_w),
        DATA_PAIR_t<float>("period",				period),
        DATA_PAIR_t<float>("w_ff_filt",				w_ff_filt),
        DATA_PAIR_t<float>("w_tot",					w_tot),
        DATA_PAIR_t<float>("th_r_fb",				th_r_fb),
        DATA_PAIR_t<float>("th_r_cap",				th_r_cap),
        DATA_PAIR_t<float>("th_r_est",				th_r_est),
    };
    ExportDataToCsvFile("HallLowNegativeSpeed.csv", waveforms);

    const float Check_Start = sys_sim->m_dyno.m_pot.m_t_start + sys_sim->m_dyno.m_pot.m_t_slope;
    const float Check_Stop = sys_sim->m_dyno.m_pot.m_t_stop;
    const float Check_Duration = Check_Stop - Check_Start;
    for (float time = Check_Start; time < Check_Stop; time += (Check_Duration / Check_Points))
    {
        auto index = static_cast<uint32_t>(CSysSim::TimeToSysSimTick(time));
        ANGLE_EQUAL(th_r_fb.at(index), th_r_est.at(index), DEG_TO_RAD(15.0f));
        DOUBLES_EQUAL_PERC(w_fb.at(index), w_ff_filt.at(index), 0.05f);
    }
}

TEST(HallSensorTest, AngleHold)
{
    const uint32_t Check_Points = 50U;
    sys_sim->SetLoadType(CLoad::EType::Passive_Mech_Load);
    sys_sim->m_mech_load.LockPosition();

    const float Run_Time = 10.0f / params.sys.fb.hall.w0_th.min;

    for (float th_r_cmd = -PI; th_r_cmd < PI; th_r_cmd += (TWO_PI / Check_Points))
    {
        sys_sim->m_mech_load.Reset({ 0.0f }, { ELEC_TO_MECH(th_r_cmd,params.motor.P) });
        sys_sim->RunFor(Run_Time);	// for hall estimation loop to settle
        ANGLE_EQUAL(th_r_cmd, hall.th_r_cap.elec, DEG_TO_RAD(30.001f));
        ANGLE_EQUAL(hall.th_r_cap.elec, hall.th_r_est.elec, DEG_TO_RAD(0.5f));
        DOUBLES_EQUAL(0.0f, hall.w_ff_filt.elec, 0.05f * HZ_TO_RADSEC(params.sys.fb.hall.w_thresh.elec));
    }
}


#endif