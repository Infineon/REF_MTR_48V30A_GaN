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
#include "../OperationalCode/General.h"
#include "../OperationalCode/Params.h"
#include "../OperationalCode/Observer.h"
#include "../OperationalCode/CtrlVars.h"
#ifdef __cplusplus
}
#endif

TEST_GROUP(ObserverTest)
{
    void setup()
    {
        CSysSim::ResetOperationalCode();
        PARAMS_DEFAULT_Init();
    }

    void teardown()
    {
    }
};

TEST(ObserverTest, FluxObservers)
{
    // Parameters - only necessary ones
    // Motor parameters - Husqvarna motor
    params.motor.P = 14.0f; // [#]
    params.motor.lq = 28.0E-6f; // [H]
    params.motor.ld = 23.0E-6f; // [H]
    params.motor.lam = 1.9E-3f; // [Wb]
    params.motor.r = 11.0E-3f; // [Ohm]
    params.motor.i_cont = 50.0f; // [A]
    params.motor.w_nom.elec = MECH_TO_ELEC(HZ_TO_RADSEC(RPM_TO_HZ(12600.0f)), params.motor.P); // [Ra/sec-elec]
    params.motor.w_max.elec = MECH_TO_ELEC(HZ_TO_RADSEC(RPM_TO_HZ(12600.0f * 1.5f)), params.motor.P); // [Ra/sec-elec]

    // System parameters
    params.sys.samp.fs0 = 20.0E3f; // [Hz]
    params.sys.samp.ts0 = 1.0f / params.sys.samp.fs0; // [sec]

    // Observer parameters
    params.obs.flux_filt.k1 = 0.2f; // [#]
    params.obs.flux_filt.k2 = 0.02f; // [#]
    params.obs.flux_filt.k3 = 0.002f; // [#]
    params.obs.flux_filt.c1_coeff = params.obs.flux_filt.k1 + params.obs.flux_filt.k2 + params.obs.flux_filt.k3; // [#]
    params.obs.flux_filt.c2_coeff = params.obs.flux_filt.k1 * params.obs.flux_filt.k2 + params.obs.flux_filt.k2 * params.obs.flux_filt.k3 + params.obs.flux_filt.k1 * params.obs.flux_filt.k3; // [#]
    params.obs.flux_filt.c3_coeff = params.obs.flux_filt.k1 * params.obs.flux_filt.k2 * params.obs.flux_filt.k3; // [#]
    params.obs.flux_filt.gain = sqrtf(POW_TWO(1.0f - params.obs.flux_filt.c2_coeff) + POW_TWO(params.obs.flux_filt.c3_coeff - params.obs.flux_filt.c1_coeff)); // [#]
    params.obs.flux_filt.th_p.elec = ATan2(params.obs.flux_filt.c1_coeff - params.obs.flux_filt.c3_coeff, 1.0f - params.obs.flux_filt.c2_coeff); // [Ra-elec]
    ParkInit(-params.obs.flux_filt.th_p.elec, &params.obs.flux_filt.phase_lead);
    params.obs.biquad_a[0] = 1.0f; // [#]
    params.obs.biquad_a[1] = 0.0f; // [1/(Ra/sec)]
    params.obs.biquad_a[2] = 0.0f; // [1/(Ra/sec)^2]
    params.obs.biquad_b[0] = 1.0f; // [#]
    params.obs.biquad_b[1] = 1.0f / HZ_TO_RADSEC(200.0f) + 1.0f / HZ_TO_RADSEC(400.0f); // [1/(Ra/sec)]
    params.obs.biquad_b[2] = 1.0f / HZ_TO_RADSEC(200.0f) / HZ_TO_RADSEC(400.0f); // [1/(Ra/sec)^2]
    params.obs.pll.w0 = HZ_TO_RADSEC(1.5E3f); // [Ra/sec]
    params.obs.pll.kp = params.obs.pll.w0 / params.motor.lam; // [(Ra/sec-elec)/Wb]
    params.obs.pll.ki = POW_TWO(params.obs.pll.w0) / params.motor.lam * 0.5f; // [(Ra/sec).(Ra/sec-elec)/Wb]
    params.obs.pll.w_max.elec = params.motor.w_max.elec * 1.5f; // [Ra/sec-elec]
    params.obs.pll.th_offset.elec = params.obs.flux_filt.th_p.elec; // [Ra-elec]
    params.obs.w_thresh.elec = params.motor.w_nom.elec * 0.1f; // [Ra/sec-elec]
    params.obs.w_hyst.elec = params.obs.w_thresh.elec * 0.10f; // [Ra/sec-elec]
    //params.obs.lock_time = 0.250; // [sec]

    AB_t Zero_AB = { 0.0f, 0.0f };
    ELEC_t Zero_Elec = { 0.0f };
    OBS_Init();
    OBS_Reset(&Zero_AB, &Zero_Elec, &Zero_Elec);

    // Excitation
    const float F0 = 2.0E3f; // [Hz]
    const float T0 = 1 / F0; // [sec]
    const float W0 = TWO_PI * F0; // [Ra/sec]
    const float Run_Time = 20.0f * T0; // [sec]
    const QD_t I_QD_R_Expected = { 40.0f, -20.0f }; // [A]

    QD_t la_qd_r_expected = { params.motor.lq * I_QD_R_Expected.q, params.motor.ld * I_QD_R_Expected.d + params.motor.lam };
    QD_t la_qd_s_expected = { 0.0f, sqrtf(POW_TWO(la_qd_r_expected.q) + POW_TWO(la_qd_r_expected.d)) };
    ELEC_t delta_expected = { atan2f(la_qd_r_expected.q, la_qd_r_expected.d) };
    QD_t la_qd_r_adj_expected = { 0.0f, params.motor.lam + (params.motor.ld - params.motor.lq) * I_QD_R_Expected.d };
    QD_t v_qd_r_expected = { params.motor.r * I_QD_R_Expected.q + W0 * la_qd_r_expected.d, params.motor.r * I_QD_R_Expected.d - W0 * la_qd_r_expected.q };

    // Run observer
    vector<CReferenceFrame> la_lead_expected;
    vector<CReferenceFrame> la_lead_actual;
    vector<POLAR_t> la_lead_comparison; // rad = (rad_actual/rad_expected) [%], theta = (theta_actual-theta_expected) [Deg]
    vector<CReferenceFrame> la_lead_adj_expected;
    vector<CReferenceFrame> la_lead_adj_actual;
    vector<POLAR_t> la_lead_adj_comparison; // rad = (rad_actual/rad_expected) [%], theta = (theta_actual-theta_expected) [Deg]
    for (uint32_t index = 0; index < static_cast<uint32_t>(Run_Time * params.sys.samp.fs0); ++index)
    {
        // Generating inputs
        vars.th_r_fb.elec = Wrap2Pi(W0 * static_cast<float>(index) * params.sys.samp.ts0);
        ParkInit(vars.th_r_fb.elec, &vars.park_r);
        ParkTransformInv(&v_qd_r_expected, &vars.park_r, &vars.v_ab_cmd);
        ParkTransformInv(&I_QD_R_Expected, &vars.park_r, &vars.i_ab_fb);

        // Running observer
        OBS_RunISR0();

        // Expected and actual outputs
        la_lead_adj_expected.push_back(CReferenceFrame(vars.th_r_fb.elec + params.obs.flux_filt.th_p.elec, la_qd_r_adj_expected));
        la_lead_adj_actual.push_back(CReferenceFrame(vars.th_r_fb.elec + params.obs.flux_filt.th_p.elec, obs.la_ab_lead_adj));
        POLAR_t la_lead_adj_comp = { la_lead_adj_actual.back().GetPolar().rad / la_lead_adj_expected.back().GetPolar().rad, RAD_TO_DEG(la_lead_adj_actual.back().GetPolar().theta - la_lead_adj_expected.back().GetPolar().theta) };
        la_lead_adj_comparison.push_back(la_lead_adj_comp);

        la_lead_expected.push_back(CReferenceFrame(vars.th_r_fb.elec + delta_expected.elec + params.obs.flux_filt.th_p.elec, la_qd_s_expected));
        la_lead_actual.push_back(CReferenceFrame(vars.th_r_fb.elec + delta_expected.elec + params.obs.flux_filt.th_p.elec, obs.la_ab_lead));
        POLAR_t la_lead_comp = { la_lead_actual.back().GetPolar().rad / la_lead_expected.back().GetPolar().rad, RAD_TO_DEG(la_lead_actual.back().GetPolar().theta - la_lead_expected.back().GetPolar().theta) };
        la_lead_comparison.push_back(la_lead_comp);
    }

    // Get steady-state data
    vector<CReferenceFrame> la_lead_adj_expected_ss(la_lead_adj_expected.end() - static_cast<uint32_t>(T0 * params.sys.samp.fs0), la_lead_adj_expected.end());
    vector<CReferenceFrame> la_lead_adj_actual_ss(la_lead_adj_actual.end() - static_cast<uint32_t>(T0 * params.sys.samp.fs0), la_lead_adj_actual.end());
    vector<POLAR_t> la_lead_adj_comparison_ss(la_lead_adj_comparison.end() - static_cast<uint32_t>(T0 * params.sys.samp.fs0), la_lead_adj_comparison.end());
    vector<CReferenceFrame> la_lead_expected_ss(la_lead_expected.end() - static_cast<uint32_t>(T0 * params.sys.samp.fs0), la_lead_expected.end());
    vector<CReferenceFrame> la_lead_actual_ss(la_lead_actual.end() - static_cast<uint32_t>(T0 * params.sys.samp.fs0), la_lead_actual.end());
    vector<POLAR_t> la_lead_comparison_ss(la_lead_comparison.end() - static_cast<uint32_t>(T0 * params.sys.samp.fs0), la_lead_comparison.end());


    // Tests
    // First, making sure biquad filter has reached steady-state
    DOUBLES_EQUAL_PERC(W0, obs.w_filt_elec, 0.001f);

    vector<float> actual;
    vector<float> expected;
    // Verifying rotor flux magnitude estimation
    actual.clear();
    for (auto item : la_lead_adj_comparison_ss)
    {
        actual.push_back(item.rad);
    }
    CCompareData<float> compare(actual, 1.0f);
    compare.Execute();
    DOUBLES_EQUAL(compare.GetAbsDiffMax(), 0.00f, 0.15f); // [%]
    DOUBLES_EQUAL(compare.GetAbsDiffNorm(), 0.00f, 0.15f); // [%]

    // Verifying stator flux magnitude estimation
    actual.clear();
    for (auto item : la_lead_comparison_ss)
    {
        actual.push_back(item.rad);
    }
    compare = CCompareData<float>(actual, 1.0f);
    compare.Execute();
    DOUBLES_EQUAL(compare.GetAbsDiffMax(), 0.00f, 0.15f);
    DOUBLES_EQUAL(compare.GetAbsDiffNorm(), 0.00f, 0.15f);

    // Verifying rotor flux angle estimation
    actual.clear();
    for (auto item : la_lead_adj_comparison_ss)
    {
        actual.push_back(item.theta);
    }
    compare = CCompareData<float>(actual, 0.0f);
    compare.Execute();
    DOUBLES_EQUAL(compare.GetAbsDiffMax(), 0.0f, 3.0f); // [degrees]
    DOUBLES_EQUAL(compare.GetAbsDiffNorm(), 0.0f, 3.0f); // [degrees]

    // Verifying stator flux angle estimation
    actual.clear();
    for (auto item : la_lead_comparison_ss)
    {
        actual.push_back(item.theta);
    }
    compare = CCompareData<float>(actual, 0.0f);
    compare.Execute();
    DOUBLES_EQUAL(compare.GetAbsDiffMax(), 0.0f, 3.0f); // [degrees]
    DOUBLES_EQUAL(compare.GetAbsDiffNorm(), 0.0f, 3.0f); // [degrees]

    // Verifying rotor fluxes in qd reference frame
    actual.clear();
    expected.clear();
    for (uint32_t index = 0; index < la_lead_adj_expected_ss.size(); ++index)
    {
        expected.push_back(la_lead_adj_expected_ss[index].GetQD().q);
        actual.push_back(la_lead_adj_actual_ss[index].GetQD().q);
    }
    compare = CCompareData<float>(actual, expected);
    compare.Execute();
    DOUBLES_EQUAL(compare.GetAbsDiffMax(), 0.0f, 0.30E-3f);
    DOUBLES_EQUAL(compare.GetAbsDiffNorm(), 0.0f, 0.30E-3f);
    actual.clear();
    expected.clear();
    for (uint32_t index = 0; index < la_lead_adj_expected_ss.size(); ++index)
    {
        expected.push_back(la_lead_adj_expected_ss[index].GetQD().d);
        actual.push_back(la_lead_adj_actual_ss[index].GetQD().d);
    }
    compare = CCompareData<float>(actual, expected);
    compare.Execute();
    DOUBLES_EQUAL(compare.GetAbsDiffMax(), 0.0f, 0.30E-3f);
    DOUBLES_EQUAL(compare.GetAbsDiffNorm(), 0.0f, 0.30E-3f);

    // Verifying stator fluxes in qd reference frame
    actual.clear();
    expected.clear();
    for (uint32_t index = 0; index < la_lead_expected_ss.size(); ++index)
    {
        expected.push_back(la_lead_expected_ss[index].GetQD().q);
        actual.push_back(la_lead_actual_ss[index].GetQD().q);
    }
    compare = CCompareData<float>(actual, expected);
    compare.Execute();
    DOUBLES_EQUAL(compare.GetAbsDiffMax(), 0.0f, 0.30E-3f);
    DOUBLES_EQUAL(compare.GetAbsDiffNorm(), 0.0f, 0.30E-3f);
    actual.clear();
    expected.clear();
    for (uint32_t index = 0; index < la_lead_expected_ss.size(); ++index)
    {
        expected.push_back(la_lead_expected_ss[index].GetQD().d);
        actual.push_back(la_lead_actual_ss[index].GetQD().d);
    }
    compare = CCompareData<float>(actual, expected);
    compare.Execute();
    DOUBLES_EQUAL(compare.GetAbsDiffMax(), 0.0f, 0.30E-3f);
    DOUBLES_EQUAL(compare.GetAbsDiffNorm(), 0.0f, 0.30E-3f);

};


TEST(ObserverTest, AngleAndSpeedObservers)
{
    // Parameters - only necessary ones
    // Motor parameters - Husqvarna motor
    params.motor.P = 14.0f; // [#]
    params.motor.lq = 28.0E-6f; // [H]
    params.motor.ld = 23.0E-6f; // [H]
    params.motor.lam = 1.9E-3f; // [Wb]
    params.motor.r = 11.0E-3f; // [Ohm]
    params.motor.i_cont = 50.0f; // [A]
    params.motor.w_nom.elec = MECH_TO_ELEC(HZ_TO_RADSEC(RPM_TO_HZ(12600.0f)), params.motor.P); // [Ra/sec-elec]
    params.motor.w_max.elec = MECH_TO_ELEC(HZ_TO_RADSEC(RPM_TO_HZ(12600.0f * 1.5f)), params.motor.P); // [Ra/sec-elec]

    // System parameters
    params.sys.samp.fs0 = 20.0E3f; // [Hz]
    params.sys.samp.ts0 = 1.0f / params.sys.samp.fs0; // [sec]

    // Observer parameters
    params.obs.flux_filt.k1 = 0.2f; // [#]
    params.obs.flux_filt.k2 = 0.02f; // [#]
    params.obs.flux_filt.k3 = 0.002f; // [#]
    params.obs.flux_filt.c1_coeff = params.obs.flux_filt.k1 + params.obs.flux_filt.k2 + params.obs.flux_filt.k3; // [#]
    params.obs.flux_filt.c2_coeff = params.obs.flux_filt.k1 * params.obs.flux_filt.k2 + params.obs.flux_filt.k2 * params.obs.flux_filt.k3 + params.obs.flux_filt.k1 * params.obs.flux_filt.k3; // [#]
    params.obs.flux_filt.c3_coeff = params.obs.flux_filt.k1 * params.obs.flux_filt.k2 * params.obs.flux_filt.k3; // [#]
    params.obs.flux_filt.gain = sqrtf(POW_TWO(1.0f - params.obs.flux_filt.c2_coeff) + POW_TWO(params.obs.flux_filt.c3_coeff - params.obs.flux_filt.c1_coeff)); // [#]
    params.obs.flux_filt.th_p.elec = ATan2(params.obs.flux_filt.c1_coeff - params.obs.flux_filt.c3_coeff, 1.0f - params.obs.flux_filt.c2_coeff); // [Ra-elec]
    ParkInit(-params.obs.flux_filt.th_p.elec, &params.obs.flux_filt.phase_lead);
    params.obs.biquad_a[0] = 1.0f; // [#]
    params.obs.biquad_a[1] = 0.0f; // [1/(Ra/sec)]
    params.obs.biquad_a[2] = 0.0f; // [1/(Ra/sec)^2]
    params.obs.biquad_b[0] = 1.0f; // [#]
    params.obs.biquad_b[1] = 1.0f / HZ_TO_RADSEC(200.0f) + 1.0f / HZ_TO_RADSEC(400.0f); // [1/(Ra/sec)]
    params.obs.biquad_b[2] = 1.0f / HZ_TO_RADSEC(200.0f) / HZ_TO_RADSEC(400.0f); // [1/(Ra/sec)^2]
    params.obs.pll.w0 = HZ_TO_RADSEC(1.5E3f); // [Ra/sec]
    params.obs.pll.kp = params.obs.pll.w0 / params.motor.lam; // [(Ra/sec-elec)/Wb]
    params.obs.pll.ki = POW_TWO(params.obs.pll.w0) / params.motor.lam * 0.5f; // [(Ra/sec).(Ra/sec-elec)/Wb]
    params.obs.pll.w_max.elec = params.motor.w_max.elec * 1.5f; // [Ra/sec-elec]
    params.obs.pll.th_offset.elec = params.obs.flux_filt.th_p.elec; // [Ra-elec]
    params.obs.w_thresh.elec = params.motor.w_nom.elec * 0.1f; // [Ra/sec-elec]
    params.obs.w_hyst.elec = params.obs.w_thresh.elec * 0.10f; // [Ra/sec-elec]
    //params.obs.lock_time = 0.250; // [sec]

    AB_t Zero_AB = { 0.0f, 0.0f };
    ELEC_t Zero_Elec = { 0.0f };
    OBS_Init();
    OBS_Reset(&Zero_AB, &Zero_Elec, &Zero_Elec);

    // Excitation
    const float F0 = 2.0E3f; // [Hz]
    const float T0 = 1 / F0; // [sec]
    const float W0 = TWO_PI * F0; // [Ra/sec]
    const float Run_Time = 20.0f * T0; // [sec]
    const QD_t I_QD_R_Expected = { 40.0f, -20.0f }; // [A]

    QD_t la_qd_r_expected = { params.motor.lq * I_QD_R_Expected.q, params.motor.ld * I_QD_R_Expected.d + params.motor.lam };
    QD_t la_qd_s_expected = { 0.0f, sqrtf(POW_TWO(la_qd_r_expected.q) + POW_TWO(la_qd_r_expected.d)) };
    ELEC_t delta_expected = { atan2f(la_qd_r_expected.q, la_qd_r_expected.d) };
    QD_t v_qd_r_expected = { params.motor.r * I_QD_R_Expected.q + W0 * la_qd_r_expected.d, params.motor.r * I_QD_R_Expected.d - W0 * la_qd_r_expected.q };

    // Run observer
    vector<float> w_error;
    vector<float> th_r_error;
#ifdef CTRL_METHOD_SFO
    vector<float> th_s_error;
    vector<float> delta_error;
#endif
    for (uint32_t index = 0; index < static_cast<uint32_t>(Run_Time * params.sys.samp.fs0); ++index)
    {
        // Generating inputs
        vars.th_r_fb.elec = Wrap2Pi(W0 * static_cast<float>(index) * params.sys.samp.ts0);
        ParkInit(vars.th_r_fb.elec, &vars.park_r);
        ParkTransformInv(&v_qd_r_expected, &vars.park_r, &vars.v_ab_cmd);
        ParkTransformInv(&I_QD_R_Expected, &vars.park_r, &vars.i_ab_fb);

        // Running observer
        OBS_RunISR0();

        // Expected and actual outputs
        w_error.push_back(vars.w_est.elec - W0);
        th_r_error.push_back(Wrap2Pi(vars.th_r_est.elec - vars.th_r_fb.elec));
#ifdef CTRL_METHOD_SFO
        th_s_error.push_back(Wrap2Pi(vars.th_s_est.elec - vars.th_r_fb.elec - delta_expected.elec));
        delta_error.push_back(Wrap2Pi(vars.delta_est.elec - delta_expected.elec));
#endif
    }

    // Get steady-state data
    vector<float> w_error_ss(w_error.end() - static_cast<uint32_t>(T0 * params.sys.samp.fs0), w_error.end());
    vector<float> th_r_error_ss(th_r_error.end() - static_cast<uint32_t>(T0 * params.sys.samp.fs0), th_r_error.end());
#ifdef CTRL_METHOD_SFO
    vector<float> th_s_error_ss(th_s_error.end() - static_cast<uint32_t>(T0 * params.sys.samp.fs0), th_s_error.end());
    vector<float> delta_error_ss(delta_error.end() - static_cast<uint32_t>(T0 * params.sys.samp.fs0), delta_error.end());
#endif

    // Tests
    // First, making sure biquad filter has reached steady-state
    DOUBLES_EQUAL_PERC(W0, obs.w_filt_elec, 0.001f);

    // Verify estimated electrical speed
    CCompareData<float> compare(w_error_ss, 0.0f);
    compare.Execute();
    DOUBLES_EQUAL(0.0f, compare.GetAbsDiffMax(), 0.01 * W0);
    DOUBLES_EQUAL(0.0f, compare.GetAbsDiffNorm(), 0.01 * W0);

    // Verifying estimated rotor angle
    compare = CCompareData<float>(th_r_error_ss, 0.0f);
    compare.Execute();
    DOUBLES_EQUAL(0.0f, compare.GetAbsDiffMax(), 3.0f / 180.0f * PI);
    DOUBLES_EQUAL(0.0f, compare.GetAbsDiffNorm(), 3.0f / 180.f * PI);

#ifdef  CTRL_METHOD_SFO
    // Verifying estimated stator angle
    compare = CCompareData<float>(th_s_error_ss, 0.0f);
    compare.Execute();
    DOUBLES_EQUAL(0.0f, compare.GetAbsDiffMax(), 3.0f / 180.0f * PI);
    DOUBLES_EQUAL(0.0f, compare.GetAbsDiffNorm(), 3.0f / 180.f * PI);

    // Verifying estimated load angle
    compare = CCompareData<float>(delta_error_ss, 0.0f);
    compare.Execute();
    DOUBLES_EQUAL(0.0f, compare.GetAbsDiffMax(), 5.0f / 180.0f * PI);
    DOUBLES_EQUAL(0.0f, compare.GetAbsDiffNorm(), 5.0f / 180.f * PI);
#endif

};