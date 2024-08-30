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
#include <assert.h>
#include <algorithm>    // std::transform
#include <functional>   // std::plus

CReferenceFrame::CReferenceFrame()
{
    m_theta = 0.0f;
    m_park.cosine = 1.0f; m_park.sine = 0.0f;
    m_ab.alpha = 0.0f; m_ab.beta = 0.0f;
    m_qd.q = 0.0f; m_qd.d = 0.0f;
    m_polar.rad = 0.0f; m_polar.theta = 0.0f;
}

CReferenceFrame::CReferenceFrame(float theta, AB_t ab)
{
    SetAB(theta, ab);
}

CReferenceFrame::CReferenceFrame(float theta, QD_t qd)
{
    SetQD(theta, qd);
}

CReferenceFrame::CReferenceFrame(float theta, POLAR_t polar)
{
    SetPolar(theta, polar);
}

CReferenceFrame::~CReferenceFrame() {};

void CReferenceFrame::SetAB(float theta, float ab_a, float ab_b)
{
    AB_t ab = { ab_a, ab_b };
    SetAB(theta, ab);
}

void CReferenceFrame::SetAB(float theta, AB_t ab)
{
    m_theta = Wrap2Pi(theta);
    m_ab = ab;
    ParkInit(m_theta, &m_park);
    ParkTransform(&m_ab, &m_park, &m_qd);
    ToPolar(m_ab.alpha, -m_ab.beta, &m_polar);
}

void CReferenceFrame::SetQD(float theta, float qd_q, float qd_d)
{
    QD_t qd = { qd_q, qd_d };
    SetQD(theta, qd);
}

void CReferenceFrame::SetQD(float theta, QD_t qd)
{
    m_theta = Wrap2Pi(theta);
    m_qd = qd;
    ParkInit(m_theta, &m_park);
    ParkTransformInv(&m_qd, &m_park, &m_ab);
    ToPolar(m_ab.alpha, -m_ab.beta, &m_polar);
}

void CReferenceFrame::SetPolar(float theta, float polar_rad, float polar_theta)
{
    POLAR_t polar = { polar_rad, polar_theta };
    SetPolar(theta, polar);
}

void CReferenceFrame::SetPolar(float theta, POLAR_t polar)
{
    m_theta = Wrap2Pi(theta);
    m_polar = polar;
    ParkInit(m_theta, &m_park);
    m_ab.alpha = m_polar.rad * m_park.cosine;
    m_ab.beta = -m_polar.rad * m_park.sine;
    ParkTransform(&m_ab, &m_park, &m_qd);
}

QD_t CReferenceFrame::GetQD()
{
    return m_qd;
}

AB_t CReferenceFrame::GetAB()
{
    return m_ab;
}

POLAR_t CReferenceFrame::GetPolar()
{
    return m_polar;
}

CDiscreteInteg::CDiscreteInteg(EMethod integ_method)
{
    SetMethod(integ_method);
    Reset(0.0f);
}

CDiscreteInteg::~CDiscreteInteg() {}

void CDiscreteInteg::SetMethod(EMethod method)
{
    m_method = method;
    if (m_method == EMethod::Forward_Euler)
    {
        RunPtr = &CDiscreteInteg::RunForwardEuler;
    }
    else if (m_method == EMethod::Backward_Euler)
    {
        RunPtr = &CDiscreteInteg::RunBackwardEuler;
    }
    else if (m_method == EMethod::Bilinear)
    {
        RunPtr = &CDiscreteInteg::RunBilinear;
    }
}

CDiscreteInteg::EMethod CDiscreteInteg::GetMethod()
{
    return m_method;
}

void CDiscreteInteg::Reset(float integ)
{
    m_integ = integ;
    m_prev_input = 0.0f;
}

float CDiscreteInteg::Run(float input)
{
    return (this->*RunPtr)(input);
}

float CDiscreteInteg::RunBackwardEuler(float input)
{
    m_integ += input;
    m_prev_input = input;
    return m_integ;
}

float CDiscreteInteg::RunForwardEuler(float input)
{
    m_integ += m_prev_input;
    m_prev_input = input;
    return m_integ;
}

float CDiscreteInteg::RunBilinear(float input)
{
    m_integ += (input + m_prev_input) * 0.5f;
    m_prev_input = input;
    return m_integ;
}

CBisectionRoots::CBisectionRoots(EquationFunc equation, MINMAX_t lim, float error_max) :
    m_equation(equation),
    m_lim(lim),
    m_error_max(error_max),
    m_try_count(0U)
{
    m_fault = (m_lim.min > m_lim.max);
}

CBisectionRoots::~CBisectionRoots() {}

float CBisectionRoots::Find()
{
    m_fault = m_fault || (++m_try_count > Max_Tries);

    float middle = 0.5f * (m_lim.min + m_lim.max);
    float f_min = m_equation(m_lim.min);
    float f_max = m_equation(m_lim.max);
    float f_middle = m_equation(middle);

    if (m_fault)
    {
        return NAN;
    }
    else if ((m_lim.max - m_lim.min) <= m_error_max)
    {
        return middle;
    }
    else if ((f_middle * f_min <= 0.0f) && (f_middle * f_max >= 0.0f))
    {
        m_lim.max = middle;
        return Find();
    }
    else if ((f_middle * f_min >= 0.0f) && (f_middle * f_max <= 0.0f))
    {
        m_lim.min = middle;
        return Find();
    }
    else
    {
        m_fault = true;
        return NAN;
    }
}


vector<float> ExportData::CollectQ(vector<QD_t>& input)
{
    vector<float> output;
    transform(input.begin(), input.end(), std::back_inserter(output), mem_fn(&QD_t::q));
    return output;
}

vector<float> ExportData::CollectD(vector<QD_t>& input)
{
    vector<float> output;
    transform(input.begin(), input.end(), std::back_inserter(output), mem_fn(&QD_t::d));
    return output;
}

vector<float> ExportData::CollectA(vector<AB_t>& input)
{
    vector<float> output;
    transform(input.begin(), input.end(), std::back_inserter(output), mem_fn(&AB_t::alpha));
    return output;
}

vector<float> ExportData::CollectB(vector<AB_t>& input)
{
    vector<float> output;
    transform(input.begin(), input.end(), std::back_inserter(output), mem_fn(&AB_t::beta));
    return output;
}