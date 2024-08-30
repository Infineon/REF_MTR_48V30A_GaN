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


#include "SILWrapMotor.h"
#include "SILAuxFcns.h"
#include "AuxMethods.h"
#include "CMotor.h"
#include "CPMSM.h"
#include "CBLDC.h"

// Simulation constants .............................................................
// Select among supported motor types:
#if defined(CTRL_METHOD_RFO) || defined(CTRL_METHOD_SFO)
const auto Motor_Type = CMotor::EType::PMSM;
#elif defined(CTRL_METHOD_TBC)
const auto Motor_Type = CMotor::EType::BLDC;
#endif
// Bilinear has lower phase distortion, Backward_Euler has lower amplitude distortion
const auto Discrete_Integ_Method = CDiscreteInteg::EMethod::Bilinear;
// Enable saturation simulation for testing initial position detection (six pulse and high frequency injection):
const EN_DIS_t Sat_En = En;
const QD_t Sat_I_QD_R_Ratio = { 16.0f, 16.0f };

// ...................................................................................

static CMotor* motor = nullptr;
static CPMSM pmsm;
static CBLDC bldc;
static UVW_t v_uvw_n;
static MECH_t w_r;
static MECH_t th_r;

void SIL_StartMotor(PARAMS_t* params_final)
{
    params = *params_final;

    switch (Motor_Type)
    {
    case CMotor::EType::PMSM:
    default:
        motor = &pmsm;
        break;
    case CMotor::EType::BLDC:
        motor = &bldc;
        break;
    }

    motor->SetInputPointers(&params, &v_uvw_n, &w_r.mech, &th_r.mech);
    motor->SetDiscreteIntegMethod(Discrete_Integ_Method);
    const QD_t Sat_I_QD_R = { params.motor.i_peak * Sat_I_QD_R_Ratio.q, params.motor.i_peak * Sat_I_QD_R_Ratio.d };
    if (Motor_Type == CMotor::EType::PMSM)
    {
        pmsm.SetSaturationParams(Sat_En, Sat_I_QD_R);
    }
}

void SIL_TerminateMotor()
{
    // Nothing to do here
}

void SIL_RunMotor(SIL_INPUT_MOTOR_t* input, SIL_OUTPUT_MOTOR_t* output)
{
    // Read inputs .............................................................................................
#pragma warning(disable:4244)	// implicit conversion of double inputs from MATLAB to float
    SIL_SetUVW(Read, input->v_uvw_n, &v_uvw_n);
    w_r.mech = input->w_r_mech;
    th_r.mech = input->th_r_mech;
#pragma warning(default:4244)	// implicit conversion of double inputs from MATLAB to float

    // Run model ...............................................................................................
    motor->RunSim();

    // Write outputs ...........................................................................................
    if (Motor_Type == CMotor::EType::PMSM)
    {
        SIL_SetAB(Write, output->v_ab, &pmsm.m_v_ab);
        SIL_SetQD(Write, output->v_qd_r, &pmsm.m_v_qd_r);
        SIL_SetQD(Write, output->v_qd_s, &pmsm.m_v_qd_s);

        SIL_SetQD(Write, output->la_qd_r, &pmsm.m_la_qd_r);
        SIL_SetQD(Write, output->la_qd_s, &pmsm.m_la_qd_s);

        SIL_SetAB(Write, output->i_ab, &pmsm.m_i_ab);
        SIL_SetQD(Write, output->i_qd_r, &pmsm.m_i_qd_r);
        SIL_SetQD(Write, output->i_qd_s, &pmsm.m_i_qd_s);

        output->th_s_elec = pmsm.m_th_s.elec;
        output->delta_elec = pmsm.m_delta.elec;
    }
    else if (Motor_Type == CMotor::EType::BLDC)
    {
        SIL_SetUVW(Write, output->e_uvw_n, &bldc.m_e_uvw_n);
        output->i_s = bldc.m_i_s;
    }

    SIL_SetUVW(Write, output->i_uvw, &motor->m_i_uvw);
    output->th_r_elec = motor->m_th_r.elec;
    output->w_r_elec = motor->m_w_r.elec;

    output->T = motor->m_T;
}