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

using namespace ExportData;

TEST_GROUP(GeneralFcnsTest)
{
    void setup()
    {
        PARAMS_DEFAULT_Init();
    }

    void teardown()
    {
    }

    UVW_SIGNAL_t ExpectedEquivHallSignal(ELEC_t th_r)
    {
        const ELEC_t th_flux = th_r;
        const ELEC_t th_bemf = { Wrap2Pi(th_flux.elec + PI_OVER_TWO) };
        
        UVW_SIGNAL_t hall = { 0U };
        if (((-PI_OVER_SIX) <= th_bemf.elec) && (th_bemf.elec < (PI_OVER_SIX)))
        {
            hall.uvw = 0b100;
        }
        else if (((PI_OVER_SIX) <= th_bemf.elec) && (th_bemf.elec < (PI_OVER_TWO)))
        {
            hall.uvw = 0b101;
        }
        else if (((-PI_OVER_TWO) <= th_bemf.elec) && (th_bemf.elec < (-PI_OVER_SIX)))
        {
            hall.uvw = 0b110;
        }
        else if (((PI_OVER_TWO) <= th_bemf.elec) && (th_bemf.elec < (FIVE_PI_OVER_SIX)))
        {
            hall.uvw = 0b001;
        }
        else if (((-FIVE_PI_OVER_SIX) <= th_bemf.elec) && (th_bemf.elec < (-PI_OVER_TWO)))
        {
            hall.uvw = 0b010;
        }
        else if (((FIVE_PI_OVER_SIX) <= th_bemf.elec) || (th_bemf.elec < (-FIVE_PI_OVER_SIX)))
        {
            hall.uvw = 0b011;
        }
        else
        {
            hall.uvw = 0b000; // fault, invalid
        }
        return hall;
    }

};

TEST(GeneralFcnsTest, ATan2)
{
    const int32_t N = 10;
    vector<float> th_ref, sin_ref, cos_ref, atan2_res;
    for (int32_t index = (-2 * N); index < (2 * N); ++index)
    {
        th_ref.push_back(index * PI / (float)(N));
        sin_ref.push_back(sinf(th_ref.back()));
        cos_ref.push_back(cosf(th_ref.back()));
        atan2_res.push_back(ATan2(sin_ref.back(), cos_ref.back()));
        ANGLE_EQUAL(th_ref.back(), atan2_res.back(), DEG_TO_RAD(0.05f));
    }

    // Export waveforms
    DATA_SET_t<float> waveforms = {
        DATA_PAIR_t<float>("time",			th_ref),
        DATA_PAIR_t<float>("sin_ref",		sin_ref),
        DATA_PAIR_t<float>("cos_ref",		cos_ref),
        DATA_PAIR_t<float>("atan2_res",		atan2_res)
    };
    ExportDataToCsvFile("ATan2.csv", waveforms);
}

TEST(GeneralFcnsTest, ASin)
{
    const int32_t N = 10;
    vector<float> x_ref, th_ref, th_res, th_error;
    for (int32_t index = -N; index <= N; ++index)
    {
        float x = (float)(index) / N;
        x_ref.push_back(x);
        th_ref.push_back(asinf(x)); // std lib
        th_res.push_back(ASin(x));
        th_error.push_back(th_res.back() - th_ref.back());
        ANGLE_EQUAL(th_ref.back(), th_res.back(), DEG_TO_RAD(0.05f));
    }

    // Export waveforms
    DATA_SET_t<float> waveforms = {
        DATA_PAIR_t<float>("time",			x_ref),
        DATA_PAIR_t<float>("th_ref",		th_ref),
        DATA_PAIR_t<float>("th_res",		th_res),
        DATA_PAIR_t<float>("th_error",		th_error)
    };
    ExportDataToCsvFile("ASin.csv", waveforms);
}

TEST(GeneralFcnsTest, ACos)
{
    const int32_t N = 10;
    vector<float> y_ref, th_ref, th_res, th_error;
    for (int32_t index = -N; index <= N; ++index)
    {
        float y = (float)(index) / N;
        y_ref.push_back(y);
        th_ref.push_back(acosf(y)); // std lib
        th_res.push_back(ACos(y));
        th_error.push_back(th_res.back() - th_ref.back());
        ANGLE_EQUAL(th_ref.back(), th_res.back(), DEG_TO_RAD(0.05f));
    }

    // Export waveforms
    DATA_SET_t<float> waveforms = {
        DATA_PAIR_t<float>("time",			y_ref),
        DATA_PAIR_t<float>("th_ref",		th_ref),
        DATA_PAIR_t<float>("th_res",		th_res),
        DATA_PAIR_t<float>("th_error",		th_error)
};
    ExportDataToCsvFile("ACos.csv", waveforms);
}

#if defined(CTRL_METHOD_TBC)
TEST(GeneralFcnsTest, EquivHallSignalFcn)
{
    const int32_t N = 100;
    vector<float> th_r;
    vector<float> hall_expected;
    vector<float> hall_actual;
    for (int32_t index = (-N); index < (+N); ++index)
    {
        th_r.push_back(index * PI / (float)(N));
        hall_expected.push_back(static_cast<float>(ExpectedEquivHallSignal({ th_r.back() }).uvw));
        hall_actual.push_back(static_cast<float>(BLOCK_COMM_EquivHallSignal({ th_r.back() }).uvw));
        CHECK_EQUAL(hall_expected.back(), hall_actual.back());
    }

    // Export waveforms
    DATA_SET_t<float> waveforms = {
        DATA_PAIR_t<float>("time",			th_r),
        DATA_PAIR_t<float>("hall_expected",	hall_expected),
        DATA_PAIR_t<float>("hall_actual",	hall_actual),
    };
    ExportDataToCsvFile("EquivHall.csv", waveforms);
}
#endif