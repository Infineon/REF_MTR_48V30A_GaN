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


#include "CPMSM.h"
#include <assert.h>

CPMSM::CPMSM() :
    CMotor(),
    RunDifferentialEquations(nullptr),
    m_integ_q(CDiscreteInteg::EMethod::Bilinear),
    m_integ_d(CDiscreteInteg::EMethod::Bilinear),
    m_i_qd_r_sat({ 0.0f, 0.0f }),
    m_i_qd_r_0({ 0.0f, 0.0f }),
    m_park_r({ 0.0f,1.0f }),
    m_park_s({ 0.0f,1.0f }),
    m_v_ab({ 0.0f,0.0f }),
    m_v_qd_r({ 0.0f,0.0f }),
    m_v_qd_s({ 0.0f,0.0f }),
    m_la_qd_r({ 0.0f,0.0f }),
    m_la_qd_s({ 0.0f,0.0f }),
    m_i_ab({ 0.0f,0.0f }),
    m_i_qd_r({ 0.0f,0.0f }),
    m_i_qd_s({ 0.0f,0.0f }),
    m_th_s({ 0.0f }),
    m_delta({ 0.0f })
{
    SetSaturationParams(Dis);
}

CPMSM::CPMSM(PARAMS_t* params, UVW_t* v_uvw_n, float* w_r_mech, float* th_r_mech) :
    CPMSM()
{
    SetInputPointers(params, v_uvw_n, w_r_mech, th_r_mech);
}

CPMSM::~CPMSM() {}

void CPMSM::RunSim()
{
    m_w_r.elec = 0.5f * m_params->motor.P * (*m_w_r_mech);
    m_th_r.elec = Wrap2Pi(0.5f * m_params->motor.P * (*m_th_r_mech));

    ClarkeTransform(m_v_uvw_n, &m_v_ab);
    ParkInit(m_th_r.elec, &m_park_r);
    ParkTransform(&m_v_ab, &m_park_r, &m_v_qd_r);

    (this->*RunDifferentialEquations)();

    ParkTransformInv(&m_i_qd_r, &m_park_r, &m_i_ab);
    ClarkeTransformInv(&m_i_ab, &m_i_uvw);

    m_la_qd_s.q = 0.0f;
    m_la_qd_s.d = sqrtf(POW_TWO(m_la_qd_r.q) + POW_TWO(m_la_qd_r.d));

    m_delta.elec = atan2f(m_la_qd_r.q, m_la_qd_r.d);
    m_th_s.elec = Wrap2Pi(m_th_r.elec + m_delta.elec);

    ParkInit(m_th_s.elec, &m_park_s);
    ParkTransform(&m_v_ab, &m_park_s, &m_v_qd_s);
    ParkTransform(&m_i_ab, &m_park_s, &m_i_qd_s);

    m_T = 0.75f * m_params->motor.P * (m_la_qd_r.d * m_i_qd_r.q - m_la_qd_r.q * m_i_qd_r.d);
}

void CPMSM::SetDiscreteIntegMethod(CDiscreteInteg::EMethod method)
{
    m_integ_q.SetMethod(method);
    m_integ_d.SetMethod(method);
}


void CPMSM::SetSaturationParams(EN_DIS_t en_dis, QD_t i_qd_r_sat)
{
    if (en_dis == Dis)
    {
        RunDifferentialEquations = &CPMSM::RunWithoutSaturation;
        m_integ_q.Reset(0.0f);
        m_integ_d.Reset(0.0f);
        m_i_qd_r = { 0.0f, 0.0f };
    }
    else // En
    {
        RunDifferentialEquations = &CPMSM::RunWithSaturation;
        m_integ_q.Reset(0.0f);
        m_integ_d.Reset(m_params->motor.lam);
        m_la_qd_r = { 0.0f, m_params->motor.lam };
        m_i_qd_r_sat = i_qd_r_sat;
        m_i_qd_r_0 = { 0.0f, m_params->motor.lam * i_qd_r_sat.d / (m_params->motor.ld * i_qd_r_sat.d - m_params->motor.lam) };
    }
}

void CPMSM::RunWithoutSaturation()
{
    float d_i_q_r = (m_v_qd_r.q - m_params->motor.r * m_i_qd_r.q - (m_w_r.elec) * (m_params->motor.ld * m_i_qd_r.d + m_params->motor.lam)) / (m_params->motor.lq * m_params->sys.samp.fsim);
    float d_i_d_r = (m_v_qd_r.d - m_params->motor.r * m_i_qd_r.d + (m_w_r.elec) * (m_params->motor.lq * m_i_qd_r.q)) / (m_params->motor.ld * m_params->sys.samp.fsim);

    m_i_qd_r.q = m_integ_q.Run(d_i_q_r);
    m_i_qd_r.d = m_integ_d.Run(d_i_d_r);

    m_la_qd_r.q = m_params->motor.lq * m_i_qd_r.q;
    m_la_qd_r.d = m_params->motor.ld * m_i_qd_r.d + m_params->motor.lam;
}

// Inductance saturation models are used for
// 1. Detecting initial d-axis polarity when using high-frequency-injection for IPMs
// 2. Detecting initial rotor angle when using six-pulse-injection for SPMs/IPMs
void CPMSM::RunWithSaturation()
{
    m_i_qd_r.q = (m_la_qd_r.q * m_i_qd_r_sat.q / ((m_params->motor.lq * m_i_qd_r_sat.q) - ABS(m_la_qd_r.q))) - m_i_qd_r_0.q;
    m_i_qd_r.d = (m_la_qd_r.d * m_i_qd_r_sat.d / ((m_params->motor.ld * m_i_qd_r_sat.d) - ABS(m_la_qd_r.d))) - m_i_qd_r_0.d;

    float d_la_q_r = (m_v_qd_r.q - m_params->motor.r * m_i_qd_r.q - m_w_r.elec * m_la_qd_r.d) / (m_params->sys.samp.fsim);
    float d_la_d_r = (m_v_qd_r.d - m_params->motor.r * m_i_qd_r.d + m_w_r.elec * m_la_qd_r.q) / (m_params->sys.samp.fsim);

    m_la_qd_r.q = m_integ_q.Run(d_la_q_r); // S061987
    m_la_qd_r.d = m_integ_d.Run(d_la_d_r);
}
