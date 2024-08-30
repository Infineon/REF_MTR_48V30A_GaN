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
#include "../OperationalCode/Biquad.h"
#ifdef __cplusplus
}
#endif

TEST_GROUP(BiquadTest)
{
    BIQUAD_t* biquad;

    void setup()
    {
        biquad = new BIQUAD_t;
    }

    void teardown()
    {
        delete biquad;
    }
};

TEST(BiquadTest, PoleZeroInit)
{
    const float Fs = 1.0E6f;
    const float Wz[2] = { 2.0E3f, 50.0E3f };
    const float Wp[2] = { 1.0E3f, 15.0E3f };
    const float K = 5.0f;

    BIQUAD_PoleZeroInit(biquad, Fs, K, Wz, Wp);

    // Verifying biquad->a[] and biquad->b[]
    float num[2];
    float den[2];
    for (uint8_t index = 0; index < 2; ++index)
    {
        num[index] = biquad->a[0] + biquad->a[1] * (-Wz[index]) + biquad->a[2] * POW_TWO(-Wz[index]);
        den[index] = biquad->b[0] + biquad->b[1] * (-Wp[index]) + biquad->b[2] * POW_TWO(-Wp[index]);
        DOUBLES_EQUAL(0.0f, num[index], 1.0E-6f);
        DOUBLES_EQUAL(0.0f, den[index], 1.0E-6f);
    }

    // Verifying K
    DOUBLES_EQUAL_PERC(K, biquad->a[0] / biquad->b[0], 0.01f);
};

TEST(BiquadTest, ContinuousInit)
{
    const float Fs = 1.0E6f;
    const float Ts = 1.0f / Fs;
    const float A_Expected[3] = { 200.0E6f, 104.0E3f, 2.0f };
    const float B_Expected[3] = { 15.0E6f, 16.0E3f, 1.0f };

    BIQUAD_ContinuousInit(biquad, Fs, A_Expected, B_Expected);

    // Normalization
    float a_expected_norm[3];
    float b_expected_norm[3];
    for (uint8_t index = 0; index < 3; ++index)
    {
        a_expected_norm[index] = A_Expected[index] / B_Expected[0];
        b_expected_norm[index] = B_Expected[index] / B_Expected[0];
    }

    float a_actual[3];
    float b_actual[3];
    a_actual[0] = biquad->m[0] + biquad->m[1] + biquad->m[2];
    b_actual[0] = biquad->n[0] + biquad->n[1] + biquad->n[2];
    a_actual[1] = -(biquad->m[1] + 2.0f * biquad->m[2]) * Ts;
    b_actual[1] = -(biquad->n[1] + 2.0f * biquad->n[2]) * Ts;
    a_actual[2] = biquad->m[2] * POW_TWO(Ts);
    b_actual[2] = biquad->n[2] * POW_TWO(Ts);

    // Normalization
    float a_actual_norm[3];
    float b_actual_norm[3];
    for (uint8_t index = 0; index < 3; ++index)
    {
        a_actual_norm[index] = a_actual[index] / b_actual[0];
        b_actual_norm[index] = b_actual[index] / b_actual[0];
    }

    // Verifying biquad->m[] and biquad->n[]
    for (uint8_t index = 0; index < 3; ++index)
    {
        DOUBLES_EQUAL_PERC(a_expected_norm[index], a_actual_norm[index], 0.01f);
        DOUBLES_EQUAL_PERC(b_expected_norm[index], b_actual_norm[index], 0.01f);
    }

};

TEST(BiquadTest, FrequencyResponse)
{
    // Filter parameters
    const float Fs = 500.0E3f;
    const float Ts = 1.0f / Fs;
    const float Wz[2] = { 1.0E3f, 10.0E3f };
    const float Wp[2] = { 5.0E3f, 40.0E3f };
    const float K = 2.0f;

    // Excitation input
    const float Input_Magnitude = 1.0f;
    const complex<float> Input_Phasor(Input_Magnitude, 0.0f);
    const float W0 = 8.0E3f;

    // Expected output
    const complex<float> Num_Phasor = (1.0f + W0 / Wz[0] * 1.0if)* (1.0f + W0 / Wz[1] * 1.0if);
    const complex<float> Den_Phasor = (1.0f + W0 / Wp[0] * 1.0if)* (1.0f + W0 / Wp[1] * 1.0if);
    const complex<float> Output_Phasor = K * Num_Phasor / Den_Phasor;
    const float Amplitude_Expected = abs(Output_Phasor);
    const float Phase_Lag_Expected = arg(Output_Phasor);

    // Init and reset biquad
    BIQUAD_PoleZeroInit(biquad, Fs, K, Wz, Wp);
    BIQUAD_Reset(biquad, 0.0f);

    // Run biquad
    const float Period = (2.0f * static_cast<float>(PI) / W0);
    const float Run_Time = 10.0f * Period;
    vector<float> input;
    vector<float> output;
    for (uint32_t index = 0; index < static_cast<uint32_t>(Run_Time * Fs); ++index)
    {
        input.push_back(Input_Magnitude * cosf(W0 * index * Ts));
        output.push_back(BIQUAD_Run(biquad, input.back()));
    }

    // Get steady-state data
    vector<float>::const_iterator input_ss_first = input.end() - static_cast<uint32_t>(Period * Fs);
    vector<float>::const_iterator input_ss_last = input.end();
    vector<float> input_ss(input_ss_first, input_ss_last);
    vector<float>::const_iterator output_ss_first = output.end() - static_cast<uint32_t>(Period * Fs);
    vector<float>::const_iterator output_ss_last = output.end();
    vector<float> output_ss(output_ss_first, output_ss_last);

    // Analyze result
    CDiscreteFourierTransform<float> output_dft(output_ss);
    auto output_dft_result = output_dft.ExecuteAll();
    float amplitude_actual = 2.0f * abs(output_dft_result[1U]); // Because cos(x)=0.5*(exp(jx)+exp(-jx))
    float phase_lag_actual = arg(output_dft_result[1U]);

    DOUBLES_EQUAL_PERC(Amplitude_Expected, amplitude_actual, 0.05f);
    DOUBLES_EQUAL_PERC(Phase_Lag_Expected, phase_lag_actual, 0.05f);
};
