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
#include "../OperationalCode/ResonantFilt.h"
#ifdef __cplusplus
}
#endif

TEST_GROUP(ResonantFiltTest)
{
    RESONANT_t* resonant;

    void setup()
    {
        resonant = new RESONANT_t;
    }

    void teardown()
    {
        delete resonant;
    }

};

TEST(ResonantFiltTest, FrequencyResponseAtResonance)
{
    // Parameter initialization
    const float Fs = 500.0E3f;
    const float Ts = 1.0f / Fs;
    const float A1 = 15.0E3f;
    const float B1 = 15.0E3f;
    const float B0 = 64.0E6f;
    RESONANT_UpdateParams(resonant, A1, B1, B0, Ts);
    RESONANT_Reset(resonant);

    // Excitation input
    const float Input_Magnitude = 1.0f;
    const complex<float> Input_Phasor(Input_Magnitude, 0.0f);
    const float W0 = sqrtf(B0);

    // Expected output
    const complex<float> Num_Phasor = A1;
    const complex<float> Den_Phasor = B1;
    const complex<float> Output_Phasor = A1 / B1;
    const float Amplitude_Expected = abs(Output_Phasor);
    const float Phase_Lag_Expected = arg(Output_Phasor);

    // Run resonant
    const float Period = (2.0f * static_cast<float>(PI) / W0);
    const float Run_Time = 10.0f * Period;
    vector<float> input;
    vector<float> output;

    for (uint32_t index = 0; index < static_cast<uint32_t>(Run_Time * Fs); ++index)
    {
        input.push_back(Input_Magnitude * cosf(W0 * index * Ts));
        output.push_back(RESONANT_Run(resonant, input.back()));
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
    DOUBLES_EQUAL(Phase_Lag_Expected, phase_lag_actual, 0.05f);

};

TEST(ResonantFiltTest, FrequencyResponseBelowResonance)
{
    // Parameter initialization
    const float Fs = 500.0E3f;
    const float Ts = 1.0f / Fs;
    const float A1 = 8.0f * 100.0E3f;
    const float B1 = 200.0E3f;
    const float B0 = 100.0E3f * 100.0E3f;
    RESONANT_UpdateParams(resonant, A1, B1, B0, Ts);
    RESONANT_Reset(resonant);

    // Excitation input
    const float Input_Magnitude = 1.0f;
    const complex<float> Input_Phasor(Input_Magnitude, 0.0f);
    const float W0 = 5.0E3;

    // Expected output
    const complex<float> Num_Phasor = A1 * W0 * 1.0if;
    const complex<float> Den_Phasor = -POW_TWO(W0) + B1 * W0 * 1.0if + B0;
    const complex<float> Output_Phasor = Num_Phasor / Den_Phasor;
    const float Amplitude_Expected = abs(Output_Phasor);
    const float Phase_Lag_Expected = arg(Output_Phasor);

    // Run resonant
    const float Period = (2.0f * static_cast<float>(PI) / W0);
    const float Run_Time = 10.0f * Period;
    vector<float> input;
    vector<float> output;

    for (uint32_t index = 0; index < static_cast<uint32_t>(Run_Time * Fs); ++index)
    {
        input.push_back(Input_Magnitude * cosf(W0 * index * Ts));
        output.push_back(RESONANT_Run(resonant, input.back()));
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
    DOUBLES_EQUAL(Phase_Lag_Expected, phase_lag_actual, 0.05f);

};


TEST(ResonantFiltTest, FrequencyResponseAboveResonance)
{
    // Parameter initialization
    const float Fs = 500.0E3f;
    const float Ts = 1.0f / Fs;
    const float A1 = 3.0f * 1.0E3f;
    const float B1 = 0.5E3f;
    const float B0 = 1.0E3f * 1.0E3f;
    RESONANT_UpdateParams(resonant, A1, B1, B0, Ts);
    RESONANT_Reset(resonant);

    // Excitation input
    const float Input_Magnitude = 1.0f;
    const complex<float> Input_Phasor(Input_Magnitude, 0.0f);
    const float W0 = 60.0E3;

    // Expected output
    const complex<float> Num_Phasor = A1 * W0 * 1.0if;
    const complex<float> Den_Phasor = -POW_TWO(W0) + B1 * W0 * 1.0if + B0;
    const complex<float> Output_Phasor = Num_Phasor / Den_Phasor;
    const float Amplitude_Expected = abs(Output_Phasor);
    const float Phase_Lag_Expected = arg(Output_Phasor);

    // Run resonant
    const float Period = (2.0f * static_cast<float>(PI) / W0);
    const float Run_Time = 10.0f * Period;
    vector<float> input;
    vector<float> output;

    for (uint32_t index = 0; index < static_cast<uint32_t>(Run_Time * Fs); ++index)
    {
        input.push_back(Input_Magnitude * cosf(W0 * index * Ts));
        output.push_back(RESONANT_Run(resonant, input.back()));
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
    DOUBLES_EQUAL(Phase_Lag_Expected, phase_lag_actual, 0.05f);

};
