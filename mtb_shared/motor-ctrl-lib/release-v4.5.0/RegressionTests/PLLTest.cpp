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

#ifdef __cplusplus
extern "C"
{
#endif
#include "../OperationalCode/General.h"
#include "../OperationalCode/Params.h"
#include "../OperationalCode/PLL.h"
#ifdef __cplusplus
}
#endif

TEST_GROUP(PhaseLockLoopTest)
{
    PLL_t* pll;

    void setup()
    {
        PARAMS_DEFAULT_Init();
        pll = new PLL_t;
    }

    void teardown()
    {
        delete pll;
    }
};

TEST(PhaseLockLoopTest, AngleTracking)
{
    // Parameters and initializations
    const float Fs = 20.0E3f; // [Hz]
    const float Ts = 1.0f / Fs; // [sec]
    const float Amplitude = 3.0f;
    const float W0 = 12.5E3f; // [Ra/sec]
    const float T0 = (TWO_PI / W0); // [sec]
    const float Run_Time = 10.0f * T0; // [sec]

    PLL_PARAMS_t p;
    p.w0 = 10.0E3f; // [Ra/sec]
    p.kp = p.w0 / Amplitude;
    p.ki = 0.5f * POW_TWO(p.w0) / Amplitude;
    p.w_max.elec = (Fs / 4.0f) * TWO_PI; // [Ra/sec]
    p.th_offset.elec = 0.0f;
    PLL_Init(pll, &p, Ts);
    const ELEC_t Zero = { 0.0f };
    PLL_Reset(pll, Zero, Zero);

    // Run phase lock loop
    vector<float> input_angle;
    vector<float> output_angle;
    vector<float> angle_diff_expected;
    vector<float> angle_diff_actual;
    for (uint32_t index = 0; index < (Run_Time * Fs); ++index)
    {
        input_angle.push_back(Wrap2Pi(W0 * static_cast<float>(index) * Ts));
        AB_t pll_input = { Amplitude * sinf(input_angle.back()), Amplitude * cosf(input_angle.back()) };
        PLL_Run(pll, &pll_input);
        output_angle.push_back(pll->th.elec);
        angle_diff_actual.push_back(Wrap2Pi(input_angle.back() - output_angle.back()));
        angle_diff_expected.push_back(0.0f);
    };

    // Get steady-state data
    vector<float> angle_diff_ss_actual(angle_diff_actual.end() - static_cast<uint32_t>(T0 * Fs), angle_diff_actual.end());
    vector<float> angle_diff_ss_expected(angle_diff_expected.end() - static_cast<uint32_t>(T0 * Fs), angle_diff_expected.end());

    // Compare expected vs actual pll output in steady-state
    CCompareData<float> compare_data(angle_diff_ss_actual, angle_diff_ss_expected);
    compare_data.Execute();
    auto abs_diff_max = compare_data.GetAbsDiffMax();
    auto abs_diff_norm = compare_data.GetAbsDiffNorm();
    DOUBLES_EQUAL(0.0f, abs_diff_max, 0.01f / 180.0f * PI);
    DOUBLES_EQUAL(0.0f, abs_diff_norm, 0.01f / 180.0f * PI);

};

TEST(PhaseLockLoopTest, MagnitudeTracking)
{
    // Parameters and initializations
    const float Fs = 50.0E3f; // [Hz]
    const float Ts = 1.0f / Fs; // [sec]
    const float Amplitude = 7.5f;
    const float W0 = 12.5E3f; // [Ra/sec]
    const float T0 = (TWO_PI / W0); // [sec]
    const float Run_Time = 10.0f * T0; // [sec]

    PLL_PARAMS_t p;
    p.w0 = 10.0E3f; // [Ra/sec]
    p.kp = p.w0 / Amplitude;
    p.ki = 0.5f * POW_TWO(p.w0) / Amplitude;
    p.w_max.elec = (Fs / 4.0f) * TWO_PI; // [Ra/sec]
    p.th_offset.elec = 0.0f;
    PLL_Init(pll, &p, Ts);
    const ELEC_t Zero = { 0.0f };
    PLL_Reset(pll, Zero, Zero);

    // Run phase lock loop
    vector<float> mag_actual;
    vector<float> mag_expected;
    for (uint32_t index = 0; index < (Run_Time * Fs); ++index)
    {
        float angle = Wrap2Pi(W0 * static_cast<float>(index) * Ts);
        AB_t pll_input = { Amplitude * sinf(angle), Amplitude * cosf(angle) };
        PLL_Run(pll, &pll_input);
        mag_actual.push_back(pll->mag);
        mag_expected.push_back(Amplitude);
    };

    // Get steady-state data
    vector<float> mag_ss_actual(mag_actual.end() - static_cast<uint32_t>(T0 * Fs), mag_actual.end());
    vector<float> mag_ss_expected(mag_expected.end() - static_cast<uint32_t>(T0 * Fs), mag_expected.end());

    // Compare expected vs actual pll output in steady-state
    CCompareData<float> compare_data(mag_ss_actual, mag_ss_expected);
    compare_data.Execute();
    auto abs_diff_max = compare_data.GetAbsDiffMax();
    auto abs_diff_norm = compare_data.GetAbsDiffNorm();
    DOUBLES_EQUAL(0.0f, abs_diff_max, Amplitude * 0.01f);
    DOUBLES_EQUAL(0.0f, abs_diff_norm, Amplitude * 0.01f);

};