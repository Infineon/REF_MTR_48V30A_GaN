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

TEST_GROUP(MotorProfilerTest)
{
    CSysSim* sys_sim;

    void setup()
    {
        sys_sim = new CSysSim();
        
        // Important parameter settings for testing motor profiler:

        const MECH_t Initial_Speed = { 0.0f };
        const MECH_t Initial_Angle = { ELEC_TO_MECH(PI_OVER_FOUR, params.motor.P) };
        sys_sim->m_mech_load.Reset(Initial_Speed, Initial_Angle); // to test rotor locking state

        const EN_DIS_t Sat_En = Dis;
        const QD_t Sat_I_QD_R_Ratio = { 16.0f, 16.0f };
        sys_sim->m_pmsm.SetSaturationParams(Sat_En, Sat_I_QD_R_Ratio);

        params.motor.r = 55.0E-3f; // [Ohm]
        params.sys.samp.fs0 = 40.0E3f;
        params.sys.samp.fsim_fs0_ratio = 2U;
        params.ctrl.mode = Motor_Profiler_Mode;
        params.mech.friction = 0.05f * 4.0f * 0.05f; // low friction to test rotor locking
        PARAMS_DEFAULT_InitAutoCalc();
        STATE_MACHINE_Init();

        // MINMAX_t Range_R_Over_L = { 200.0f, 450.0f }; // [Hz]
        // PROF_FREQ_POINTS = 5U
        // Simulation time = 14.0 sec
    }

    void teardown()
    {
        delete sys_sim;
    }

};

IGNORE_TEST(MotorProfilerTest, PlaceHolder)
{
}

#endif