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

using namespace ExportData;

static void CallbackISR0()
{
    SENSOR_IFACE_RunISR0();

    vars.w_final.elec = vars.w_est.elec;
    CTRL_FILTS_RunAllISR0();

    HIGH_FREQ_INJ_RunISR0();

    vars.v_ab_cmd = ctrl.high_freq_inj.v_ab_cmd;
    vars.v_ab_cmd_tot = vars.v_ab_cmd;
    VOLT_MOD_RunISR0();
}

static void CallbackISR1()
{
    SENSOR_IFACE_RunISR1();
}

TEST_GROUP(HighFreqInjTest)
{
    CSysSim* sys_sim;

    void setup()
    {
        sys_sim = new CSysSim();

        params.sys.samp.fs0 = 20.0E3f;
        params.sys.samp.fs0_fs1_ratio = 10U;
        params.sys.samp.fsim_fs0_ratio = 2U;
        //params.ctrl.high_freq_inj.w_sep = 300.0f;
        //params.ctrl.high_freq_inj.pll.w0 = 30.0f;
        PARAMS_DEFAULT_InitAutoCalc();
        STATE_MACHINE_Init();

        // Only simulating SixPulseInj.h/.c:
        sys_sim->RegisterCallbackISR0(&CallbackISR0);
        sys_sim->RegisterCallbackISR1(&CallbackISR1);

    }

    void teardown()
    {
        delete sys_sim;
    }
};

TEST(HighFreqInjTest, StandingStillPositiveAngle)
{
    const ELEC_t Th_R_0 = { DEG_TO_RAD(50.0f) };

    // IPM: Lq!=Ld
    params.motor.lq = 28.0E-6f; // [H]
    params.motor.ld = 23.0E-6f; // [H]
    params.motor.r = 11.0E-3f; // [Ohm]
    PARAMS_DEFAULT_InitAutoCalc();

    const float Run_Time = 10.0f * (1.0f / params.ctrl.high_freq_inj.pll.w0);

    // Init
    SENSOR_IFACE_Init();
    SENSOR_IFACE_Reset();
    CTRL_FILTS_Init();
    CTRL_FILTS_Reset();
    HIGH_FREQ_INJ_Init();
    HIGH_FREQ_INJ_Reset({ 0.0f }, { 0.0f });
    vars.v_dc = sys_sim->m_vdc;
    VOLT_MOD_Init();

    // Enable modeling of motor inducatance saturation (lq/ld)
    //const QD_t I_QD_R_Sat = { params.motor.i_peak * 4.0f, params.motor.i_peak * 4.0f };
    //sys_sim->m_pmsm.SetSaturationParams(En, I_QD_R_Sat);

    // Take rotor to the initial angle to be tested
    sys_sim->m_mech_load.Reset({ 0.0f }, { ELEC_TO_MECH(Th_R_0.elec, params.motor.P) });

    // Lock rotor
    sys_sim->m_mech_load.LockPosition();

    // Run simulation and record waveforms
    vector<float> time;
    vector<float> w_est, w_final_filt;
    vector<float> th_r_est;
#if defined(CTRL_METHOD_SFO)
    vector<float> th_s_est, delta_est;
    vector<QD_t> la_qd_s_est;
#endif
    vector<QD_t> v_qd_r, i_qd_r_tot, i_qd_r_hf, i_qd_r;

    while (sys_sim->GetAbsTime() < Run_Time)
    {
        // Run simulation
        sys_sim->RunOneTick();
        // Record waveforms
        time.push_back(sys_sim->GetAbsTime());
        w_est.push_back(vars.w_est.elec);
        w_final_filt.push_back(vars.w_final_filt.elec);
        th_r_est.push_back(vars.th_r_est.elec);
#if defined(CTRL_METHOD_SFO)
        th_s_est.push_back(vars.th_s_est.elec);
        delta_est.push_back(vars.delta_est.elec);
        la_qd_s_est.push_back(vars.la_qd_s_est);
#endif
        v_qd_r.push_back(ctrl.high_freq_inj.v_qd_r_cmd);
        i_qd_r_tot.push_back(ctrl.high_freq_inj.i_qd_r_fb_tot);
        i_qd_r_hf.push_back(ctrl.high_freq_inj.i_qd_r_fb_hf);
        i_qd_r.push_back(vars.i_qd_r_fb);
    }

    // Export waveforms
    DATA_SET_t<float> waveforms = {
        DATA_PAIR_t<float>("time",				time),
        DATA_PAIR_t<float>("w_est",				w_est),
        DATA_PAIR_t<float>("w_final_filt",		w_final_filt),
        DATA_PAIR_t<float>("th_r_est",			th_r_est),
#if defined(CTRL_METHOD_SFO)
        DATA_PAIR_t<float>("th_s_est",			th_s_est),
        DATA_PAIR_t<float>("delta_est",			delta_est),
        DATA_PAIR_t<float>("la_qd_s_est.d",		CollectD(la_qd_s_est)),
#endif
        DATA_PAIR_t<float>("v_qd_r.q",			CollectQ(v_qd_r)),
        DATA_PAIR_t<float>("v_qd_r.d",			CollectD(v_qd_r)),
        DATA_PAIR_t<float>("i_qd_r_tot.q",		CollectQ(i_qd_r_tot)),
        DATA_PAIR_t<float>("i_qd_r_tot.d",		CollectD(i_qd_r_tot)),
        DATA_PAIR_t<float>("i_qd_r_hf.q",		CollectQ(i_qd_r_hf)),
        DATA_PAIR_t<float>("i_qd_r_hf.d",		CollectD(i_qd_r_hf)),
        DATA_PAIR_t<float>("i_qd_r.q",			CollectQ(i_qd_r)),
        DATA_PAIR_t<float>("i_qd_r.d",			CollectD(i_qd_r))
    };
#if defined(CTRL_METHOD_RFO)
    ExportDataToCsvFile("RFO-HighFreqInjPosAngle.csv", waveforms);
#elif defined(CTRL_METHOD_SFO)
    ExportDataToCsvFile("SFO-HighFreqInjPosAngle.csv", waveforms);
#endif
    // Verify steady-state has been reached (zero angle error)
    DOUBLES_EQUAL(0.0f, i_qd_r.back().q, 0.5f);

    // Verify correct angle and velocity estimation
    ANGLE_EQUAL(Th_R_0.elec, th_r_est.back(), DEG_TO_RAD(2.0f));
    DOUBLES_EQUAL(0.0f, w_est.back(), HZ_TO_RADSEC(2.0f));
    DOUBLES_EQUAL(0.0f, v_qd_r.back().q, 0.5f);
}

TEST(HighFreqInjTest, StandingStillNegativeAngle)
{
    const ELEC_t Th_R_0 = { DEG_TO_RAD(-20.0f) };

    // IPM: Lq!=Ld
    params.motor.lq = 28.0E-6f; // [H]
    params.motor.ld = 23.0E-6f; // [H]
    params.motor.r = 11.0E-3f; // [Ohm]
    PARAMS_DEFAULT_InitAutoCalc();

    const float Run_Time = 10.0f * (1.0f / params.ctrl.high_freq_inj.pll.w0);

    // Init
    SENSOR_IFACE_Init();
    SENSOR_IFACE_Reset();
    CTRL_FILTS_Init();
    CTRL_FILTS_Reset();
    HIGH_FREQ_INJ_Init();
    HIGH_FREQ_INJ_Reset({ 0.0f }, { 0.0f });
    vars.v_dc = sys_sim->m_vdc;
    VOLT_MOD_Init();

    // Enable modeling of motor inducatance saturation (lq/ld)
    //const QD_t I_QD_R_Sat = { params.motor.i_peak * 4.0f, params.motor.i_peak * 4.0f };
    //sys_sim->m_pmsm.SetSaturationParams(En, I_QD_R_Sat);

    // Take rotor to the initial angle to be tested
    sys_sim->m_mech_load.Reset({ 0.0f }, { ELEC_TO_MECH(Th_R_0.elec, params.motor.P) });

    // Lock rotor
    sys_sim->m_mech_load.LockPosition();

    // Run simulation and record waveforms
    vector<float> time;
    vector<float> w_est, w_final_filt;
    vector<float> th_r_est;
#if defined(CTRL_METHOD_SFO)
    vector<float> th_s_est, delta_est;
    vector<QD_t> la_qd_s_est;
#endif
    vector<QD_t> v_qd_r, i_qd_r_tot, i_qd_r_hf, i_qd_r;

    while (sys_sim->GetAbsTime() < Run_Time)
    {
        // Run simulation
        sys_sim->RunOneTick();
        // Record waveforms
        time.push_back(sys_sim->GetAbsTime());
        w_est.push_back(vars.w_est.elec);
        w_final_filt.push_back(vars.w_final_filt.elec);
        th_r_est.push_back(vars.th_r_est.elec);
#if defined(CTRL_METHOD_SFO)
        th_s_est.push_back(vars.th_s_est.elec);
        delta_est.push_back(vars.delta_est.elec);
        la_qd_s_est.push_back(vars.la_qd_s_est);
#endif
        v_qd_r.push_back(ctrl.high_freq_inj.v_qd_r_cmd);
        i_qd_r_tot.push_back(ctrl.high_freq_inj.i_qd_r_fb_tot);
        i_qd_r_hf.push_back(ctrl.high_freq_inj.i_qd_r_fb_hf);
        i_qd_r.push_back(vars.i_qd_r_fb);
    }

    // Export waveforms
    DATA_SET_t<float> waveforms = {
        DATA_PAIR_t<float>("time",				time),
        DATA_PAIR_t<float>("w_est",				w_est),
        DATA_PAIR_t<float>("w_final_filt",		w_final_filt),
        DATA_PAIR_t<float>("th_r_est",			th_r_est),
#if defined(CTRL_METHOD_SFO)
        DATA_PAIR_t<float>("th_s_est",			th_s_est),
        DATA_PAIR_t<float>("delta_est",			delta_est),
        DATA_PAIR_t<float>("la_qd_s_est.d",		CollectD(la_qd_s_est)),
#endif
        DATA_PAIR_t<float>("v_qd_r.q",			CollectQ(v_qd_r)),
        DATA_PAIR_t<float>("v_qd_r.d",			CollectD(v_qd_r)),
        DATA_PAIR_t<float>("i_qd_r_tot.q",		CollectQ(i_qd_r_tot)),
        DATA_PAIR_t<float>("i_qd_r_tot.d",		CollectD(i_qd_r_tot)),
        DATA_PAIR_t<float>("i_qd_r_hf.q",		CollectQ(i_qd_r_hf)),
        DATA_PAIR_t<float>("i_qd_r_hf.d",		CollectD(i_qd_r_hf)),
        DATA_PAIR_t<float>("i_qd_r.q",			CollectQ(i_qd_r)),
        DATA_PAIR_t<float>("i_qd_r.d",			CollectD(i_qd_r))
    };
#if defined(CTRL_METHOD_RFO)
    ExportDataToCsvFile("RFO-HighFreqInjNegAngle.csv", waveforms);
#elif defined(CTRL_METHOD_SFO)
    ExportDataToCsvFile("SFO-HighFreqInjNegAngle.csv", waveforms);
#endif
    // Verify steady-state has been reached (zero angle error)
    DOUBLES_EQUAL(0.0f, i_qd_r.back().q, 0.5f);

    // Verify correct angle and velocity estimation
    ANGLE_EQUAL(Th_R_0.elec, th_r_est.back(), DEG_TO_RAD(2.0f));
    DOUBLES_EQUAL(0.0f, w_est.back(), HZ_TO_RADSEC(2.0f));
    DOUBLES_EQUAL(0.0f, v_qd_r.back().q, 0.5f);
}


TEST(HighFreqInjTest, SpininnigPositiveDirection)
{

    // IPM: Lq!=Ld
    params.motor.lq = 28.0E-6f; // [H]
    params.motor.ld = 23.0E-6f; // [H]
    params.motor.r = 11.0E-3f; // [Ohm]
    PARAMS_DEFAULT_InitAutoCalc();

    // Init
    SENSOR_IFACE_Init();
    SENSOR_IFACE_Reset();
    CTRL_FILTS_Init();
    CTRL_FILTS_Reset();
    HIGH_FREQ_INJ_Init();
    HIGH_FREQ_INJ_Reset({ 0.0f }, { 0.0f });
    vars.v_dc = sys_sim->m_vdc;
    VOLT_MOD_Init();

    // Enable modeling of motor inducatance saturation (lq/ld)
    //const QD_t I_QD_R_Sat = { params.motor.i_peak * 4.0f, params.motor.i_peak * 4.0f };
    //sys_sim->m_pmsm.SetSaturationParams(En, I_QD_R_Sat);

    // Set dyno parameters
    sys_sim->SetLoadType(CLoad::EType::Active_Dyno);
    sys_sim->m_dyno.SetCtrlMode(CDyno::ECtrlMode::Speed_Ctrl);
    sys_sim->m_dyno.m_w_cmd.mech = HZ_TO_RADSEC(RPM_TO_HZ(800.0f));
    sys_sim->m_dyno.m_pot.m_init = 0.0f;
    sys_sim->m_dyno.m_pot.m_final = 1.0f;
    const float Tau_Multiple = (1.0f / params.ctrl.high_freq_inj.pll.w0) * 10.0f; // to let the loop settle
    sys_sim->m_dyno.m_pot.m_t_start = Tau_Multiple;
    sys_sim->m_dyno.m_pot.m_t_slope = Tau_Multiple * 10.0f;
    sys_sim->m_dyno.m_pot.m_t_stop = sys_sim->m_dyno.m_pot.m_t_start + sys_sim->m_dyno.m_pot.m_t_slope + Tau_Multiple * 5.0f;
    const float Run_Time = sys_sim->m_dyno.m_pot.m_t_stop + sys_sim->m_dyno.m_pot.m_t_slope + Tau_Multiple;

    // Run simulation and record waveforms
    vector<float> time;
    vector<float> w_est, w_final_filt, w_fb;
    vector<float> th_r_est, th_r_fb;
#if defined(CTRL_METHOD_SFO)
    vector<float> th_s_est, th_s_fb, delta_est, delta_fb;
    vector<QD_t> la_qd_s_est, la_qd_s_fb;
#endif
    vector<QD_t> v_qd_r, i_qd_r_tot, i_qd_r_hf, i_qd_r;

    bool capture = true;

    while (sys_sim->GetAbsTime() < Run_Time)
    {
        // Run simulation
        sys_sim->RunOneTick();
        // Record waveforms
        time.push_back(sys_sim->GetAbsTime());
        w_est.push_back(vars.w_est.elec);
        w_final_filt.push_back(vars.w_final_filt.elec);
        w_fb.push_back(vars.w_fb.elec);
        th_r_est.push_back(vars.th_r_est.elec);
        th_r_fb.push_back(sys_sim->m_pmsm.m_th_r.elec);
#if defined(CTRL_METHOD_SFO)
        th_s_est.push_back(vars.th_s_est.elec);
        th_s_fb.push_back(sys_sim->m_pmsm.m_th_s.elec);
        delta_est.push_back(vars.delta_est.elec);
        delta_fb.push_back(sys_sim->m_pmsm.m_delta.elec);
        la_qd_s_est.push_back(vars.la_qd_s_est);
        la_qd_s_fb.push_back(sys_sim->m_pmsm.m_la_qd_s);
#endif
        v_qd_r.push_back(ctrl.high_freq_inj.v_qd_r_cmd);
        i_qd_r_tot.push_back(ctrl.high_freq_inj.i_qd_r_fb_tot);
        i_qd_r_hf.push_back(ctrl.high_freq_inj.i_qd_r_fb_hf);
        i_qd_r.push_back(vars.i_qd_r_fb);

        if ((sys_sim->GetAbsTime() > sys_sim->m_pot.m_t_stop) && capture) // check in stady-state, there's always phase lag during acceleration
        {
            capture = false;
            DOUBLES_EQUAL_PERC(sys_sim->m_pmsm.m_w_r.elec, vars.w_final_filt.elec, 0.05f);
            ANGLE_EQUAL(sys_sim->m_pmsm.m_th_r.elec, vars.th_r_est.elec, DEG_TO_RAD(3.0f));
#if defined(CTRL_METHOD_SFO)
            ANGLE_EQUAL(sys_sim->m_pmsm.m_th_s.elec, vars.th_s_est.elec, DEG_TO_RAD(3.0f));
            ANGLE_EQUAL(sys_sim->m_pmsm.m_delta.elec, vars.delta_est.elec, DEG_TO_RAD(3.0f));
#endif
        }

    }

    // Export waveforms
    DATA_SET_t<float> waveforms = {
        DATA_PAIR_t<float>("time",				time),
        DATA_PAIR_t<float>("w_est",				w_est),
        DATA_PAIR_t<float>("w_final_filt",		w_final_filt),
        DATA_PAIR_t<float>("w_fb",				w_fb),
        DATA_PAIR_t<float>("th_r_est",			th_r_est),
        DATA_PAIR_t<float>("th_r_fb",			th_r_fb),
#if defined(CTRL_METHOD_SFO)
        DATA_PAIR_t<float>("th_s_est",			th_s_est),
        DATA_PAIR_t<float>("th_s_fb",			th_s_fb),
        DATA_PAIR_t<float>("delta_est",			delta_est),
        DATA_PAIR_t<float>("delta_fb",			delta_fb),
        DATA_PAIR_t<float>("la_qd_s_est.d",		CollectD(la_qd_s_est)),
        DATA_PAIR_t<float>("la_qd_s_fb.d",		CollectD(la_qd_s_fb)),
#endif
        DATA_PAIR_t<float>("v_qd_r.q",			CollectQ(v_qd_r)),
        DATA_PAIR_t<float>("v_qd_r.d",			CollectD(v_qd_r)),
        DATA_PAIR_t<float>("i_qd_r_tot.q",		CollectQ(i_qd_r_tot)),
        DATA_PAIR_t<float>("i_qd_r_tot.d",		CollectD(i_qd_r_tot)),
        DATA_PAIR_t<float>("i_qd_r_hf.q",		CollectQ(i_qd_r_hf)),
        DATA_PAIR_t<float>("i_qd_r_hf.d",		CollectD(i_qd_r_hf)),
        DATA_PAIR_t<float>("i_qd_r.q",			CollectQ(i_qd_r)),
        DATA_PAIR_t<float>("i_qd_r.d",			CollectD(i_qd_r))
    };

#if defined(CTRL_METHOD_RFO)
    ExportDataToCsvFile("RFO-HighFreqInjSpinPosDir.csv", waveforms);
#elif defined(CTRL_METHOD_SFO)
    ExportDataToCsvFile("SFO-HighFreqInjSpinPosDir.csv", waveforms);
#endif
}

TEST(HighFreqInjTest, SpininnigNegativeDirection)
{

    // IPM: Lq!=Ld
    params.motor.lq = 28.0E-6f; // [H]
    params.motor.ld = 23.0E-6f; // [H]
    params.motor.r = 11.0E-3f; // [Ohm]
    PARAMS_DEFAULT_InitAutoCalc();

    // Init
    SENSOR_IFACE_Init();
    SENSOR_IFACE_Reset();
    CTRL_FILTS_Init();
    CTRL_FILTS_Reset();
    HIGH_FREQ_INJ_Init();
    HIGH_FREQ_INJ_Reset({ 0.0f }, { 0.0f });
    vars.v_dc = sys_sim->m_vdc;
    VOLT_MOD_Init();

    // Enable modeling of motor inducatance saturation (lq/ld)
    //const QD_t I_QD_R_Sat = { params.motor.i_peak * 4.0f, params.motor.i_peak * 4.0f };
    //sys_sim->m_pmsm.SetSaturationParams(En, I_QD_R_Sat);

    // Set dyno parameters
    sys_sim->SetLoadType(CLoad::EType::Active_Dyno);
    sys_sim->m_dyno.SetCtrlMode(CDyno::ECtrlMode::Speed_Ctrl);
    sys_sim->m_dyno.m_w_cmd.mech = HZ_TO_RADSEC(RPM_TO_HZ(-800.0f));
    sys_sim->m_dyno.m_pot.m_init = 0.0f;
    sys_sim->m_dyno.m_pot.m_final = 1.0f;
    const float Tau_Multiple = (1.0f / params.ctrl.high_freq_inj.pll.w0) * 10.0f; // to let the loop settle
    sys_sim->m_dyno.m_pot.m_t_start = Tau_Multiple;
    sys_sim->m_dyno.m_pot.m_t_slope = Tau_Multiple * 10.0f;
    sys_sim->m_dyno.m_pot.m_t_stop = sys_sim->m_dyno.m_pot.m_t_start + sys_sim->m_dyno.m_pot.m_t_slope + Tau_Multiple * 5.0f;
    const float Run_Time = sys_sim->m_dyno.m_pot.m_t_stop + sys_sim->m_dyno.m_pot.m_t_slope + Tau_Multiple;

    // Run simulation and record waveforms
    vector<float> time;
    vector<float> w_est, w_final_filt, w_fb;
    vector<float> th_r_est, th_r_fb;
#if defined(CTRL_METHOD_SFO)
    vector<float> th_s_est, th_s_fb, delta_est, delta_fb;
    vector<QD_t> la_qd_s_est, la_qd_s_fb;
#endif
    vector<QD_t> v_qd_r, i_qd_r_tot, i_qd_r_hf, i_qd_r;

    bool capture = true;

    while (sys_sim->GetAbsTime() < Run_Time)
    {
        // Run simulation
        sys_sim->RunOneTick();
        // Record waveforms
        time.push_back(sys_sim->GetAbsTime());
        w_est.push_back(vars.w_est.elec);
        w_final_filt.push_back(vars.w_final_filt.elec);
        w_fb.push_back(vars.w_fb.elec);
        th_r_est.push_back(vars.th_r_est.elec);
        th_r_fb.push_back(sys_sim->m_pmsm.m_th_r.elec);
#if defined(CTRL_METHOD_SFO)
        th_s_est.push_back(vars.th_s_est.elec);
        th_s_fb.push_back(sys_sim->m_pmsm.m_th_s.elec);
        delta_est.push_back(vars.delta_est.elec);
        delta_fb.push_back(sys_sim->m_pmsm.m_delta.elec);
        la_qd_s_est.push_back(vars.la_qd_s_est);
        la_qd_s_fb.push_back(sys_sim->m_pmsm.m_la_qd_s);
#endif
        v_qd_r.push_back(ctrl.high_freq_inj.v_qd_r_cmd);
        i_qd_r_tot.push_back(ctrl.high_freq_inj.i_qd_r_fb_tot);
        i_qd_r_hf.push_back(ctrl.high_freq_inj.i_qd_r_fb_hf);
        i_qd_r.push_back(vars.i_qd_r_fb);

        if ((sys_sim->GetAbsTime() > sys_sim->m_pot.m_t_stop) && capture) // check in stady-state, there's always phase lag during acceleration
        {
            capture = false;
            DOUBLES_EQUAL_PERC(sys_sim->m_pmsm.m_w_r.elec, vars.w_final_filt.elec, 0.05f);
            ANGLE_EQUAL(sys_sim->m_pmsm.m_th_r.elec, vars.th_r_est.elec, DEG_TO_RAD(3.0f));
#if defined(CTRL_METHOD_SFO)
            ANGLE_EQUAL(sys_sim->m_pmsm.m_th_s.elec, vars.th_s_est.elec, DEG_TO_RAD(3.0f));
            ANGLE_EQUAL(sys_sim->m_pmsm.m_delta.elec, vars.delta_est.elec, DEG_TO_RAD(3.0f));
#endif
        }
    }

    // Export waveforms
    DATA_SET_t<float> waveforms = {
        DATA_PAIR_t<float>("time",				time),
        DATA_PAIR_t<float>("w_est",				w_est),
        DATA_PAIR_t<float>("w_final_filt",		w_final_filt),
        DATA_PAIR_t<float>("w_fb",				w_fb),
        DATA_PAIR_t<float>("th_r_est",			th_r_est),
        DATA_PAIR_t<float>("th_r_fb",			th_r_fb),
#if defined(CTRL_METHOD_SFO)
        DATA_PAIR_t<float>("th_s_est",			th_s_est),
        DATA_PAIR_t<float>("th_s_fb",			th_s_fb),
        DATA_PAIR_t<float>("delta_est",			delta_est),
        DATA_PAIR_t<float>("delta_fb",			delta_fb),
        DATA_PAIR_t<float>("la_qd_s_est.d",		CollectD(la_qd_s_est)),
        DATA_PAIR_t<float>("la_qd_s_fb.d",		CollectD(la_qd_s_fb)),
#endif
        DATA_PAIR_t<float>("v_qd_r.q",			CollectQ(v_qd_r)),
        DATA_PAIR_t<float>("v_qd_r.d",			CollectD(v_qd_r)),
        DATA_PAIR_t<float>("i_qd_r_tot.q",		CollectQ(i_qd_r_tot)),
        DATA_PAIR_t<float>("i_qd_r_tot.d",		CollectD(i_qd_r_tot)),
        DATA_PAIR_t<float>("i_qd_r_hf.q",		CollectQ(i_qd_r_hf)),
        DATA_PAIR_t<float>("i_qd_r_hf.d",		CollectD(i_qd_r_hf)),
        DATA_PAIR_t<float>("i_qd_r.q",			CollectQ(i_qd_r)),
        DATA_PAIR_t<float>("i_qd_r.d",			CollectD(i_qd_r))
    };

#if defined(CTRL_METHOD_RFO)
    ExportDataToCsvFile("RFO-HighFreqInjSpinNegDir.csv", waveforms);
#elif defined(CTRL_METHOD_SFO)
    ExportDataToCsvFile("SFO-HighFreqInjSpinNegDir.csv", waveforms);
#endif
}

#endif