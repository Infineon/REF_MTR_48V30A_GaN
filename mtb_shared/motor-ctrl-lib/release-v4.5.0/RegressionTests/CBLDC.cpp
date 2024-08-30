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


#include "CBLDC.h"
#include <assert.h>

CBLDC::CBLDC() :
    CMotor(),
    m_high_z({ 0U }),
    m_i_s(0.0f),
    m_e_uvw_n({ 0.0f,0.0f,0.0f }),
    m_p_out(0.0f),
    m_high_z_prev({ 0U }),
    m_f_uvw({ 0.0f,0.0f,0.0f })
{}

CBLDC::CBLDC(PARAMS_t* params, UVW_t* v_uvw_n, float* w_r_mech, float* th_r_mech) : CBLDC()
{
    SetInputPointers(params, v_uvw_n, w_r_mech, th_r_mech);
}

CBLDC::~CBLDC() {}

float CBLDC::TrapezFcn(float th_flux)
{
    float result = 0.0f;
    float th_bemf = Wrap2Pi(th_flux + PI_OVER_TWO);
    if (((-PI_OVER_SIX) <= th_bemf) && (th_bemf < (PI_OVER_SIX)))
    {
        result = SIX_OVER_PI * th_bemf;
    }
    else if (((PI_OVER_SIX) <= th_bemf) && (th_bemf < (FIVE_PI_OVER_SIX)))
    {
        result = 1.0f;
    }
    else if (((-FIVE_PI_OVER_SIX) <= th_bemf) && (th_bemf < (-PI_OVER_SIX)))
    {
        result = -1.0f;
    }
    else if ((FIVE_PI_OVER_SIX) <= th_bemf)
    {
        result = 6.0f - (SIX_OVER_PI * th_bemf);
    }
    else if (th_bemf < (-FIVE_PI_OVER_SIX))
    {
        result = -6.0f - (SIX_OVER_PI * th_bemf);
    }
    return result;
}

void CBLDC::RunSim()
{
    // Rotor speed and angle
    m_w_r.elec = 0.5f * m_params->motor.P * (*m_w_r_mech);
    m_th_r.elec = Wrap2Pi(0.5f * m_params->motor.P * (*m_th_r_mech));

    // Back emfs
    m_f_uvw.u = TrapezFcn(m_th_r.elec);
    m_f_uvw.v = TrapezFcn(m_th_r.elec - TWO_PI_OVER_THREE);
    m_f_uvw.w = TrapezFcn(m_th_r.elec + TWO_PI_OVER_THREE);

    m_e_uvw_n.u = m_params->motor.lam * m_w_r.elec * m_f_uvw.u;
    m_e_uvw_n.v = m_params->motor.lam * m_w_r.elec * m_f_uvw.v;
    m_e_uvw_n.w = m_params->motor.lam * m_w_r.elec * m_f_uvw.w;

    // Currents, simulating block commutation
    float l_ave = 0.5f * (m_params->motor.lq + m_params->motor.ld);
    m_high_z_prev.state = m_high_z.state;
    m_high_z.u = isnan(m_v_uvw_n->u);
    m_high_z.v = isnan(m_v_uvw_n->v);
    m_high_z.w = isnan(m_v_uvw_n->w);
    // Instant commutation (ideal case):


#if defined(CTRL_METHOD_RFO) || defined(CTRL_METHOD_SFO)
    m_i_uvw.u = FALL_EDGE(m_high_z_prev.u, m_high_z.u) ? (RISE_EDGE(m_high_z_prev.v, m_high_z.v) ? m_i_uvw.v : m_i_uvw.w) : m_i_uvw.u;
    m_i_uvw.v = FALL_EDGE(m_high_z_prev.v, m_high_z.v) ? (RISE_EDGE(m_high_z_prev.w, m_high_z.w) ? m_i_uvw.w : m_i_uvw.u) : m_i_uvw.v;
    m_i_uvw.w = FALL_EDGE(m_high_z_prev.w, m_high_z.w) ? (RISE_EDGE(m_high_z_prev.u, m_high_z.u) ? m_i_uvw.u : m_i_uvw.v) : m_i_uvw.w;
#elif defined(CTRL_METHOD_TBC)
    if (params.ctrl.tbc.mode == Block_Commutation)
    {
        m_i_uvw.u = FALL_EDGE(m_high_z_prev.u, m_high_z.u) ? (RISE_EDGE(m_high_z_prev.v, m_high_z.v) ? m_i_uvw.v : m_i_uvw.w) : m_i_uvw.u;
        m_i_uvw.v = FALL_EDGE(m_high_z_prev.v, m_high_z.v) ? (RISE_EDGE(m_high_z_prev.w, m_high_z.w) ? m_i_uvw.w : m_i_uvw.u) : m_i_uvw.v;
        m_i_uvw.w = FALL_EDGE(m_high_z_prev.w, m_high_z.w) ? (RISE_EDGE(m_high_z_prev.u, m_high_z.u) ? m_i_uvw.u : m_i_uvw.v) : m_i_uvw.w;
    }
#endif

    switch (m_high_z.state)
    {
    case 0b000: // no block commutation
        m_i_uvw.u += (m_v_uvw_n->u - m_e_uvw_n.u - m_params->motor.r * m_i_uvw.u) * m_params->sys.samp.tsim / l_ave;
        m_i_uvw.v += (m_v_uvw_n->v - m_e_uvw_n.v - m_params->motor.r * m_i_uvw.v) * m_params->sys.samp.tsim / l_ave;
        m_i_uvw.w += (m_v_uvw_n->w - m_e_uvw_n.w - m_params->motor.r * m_i_uvw.w) * m_params->sys.samp.tsim / l_ave;
        m_i_n = (m_i_uvw.u + m_i_uvw.v + m_i_uvw.w) / 3.0f;
        break;

    case 0b001:
        m_i_uvw.u = 0.0f;
        m_i_uvw.v += (m_v_uvw_n->v - m_e_uvw_n.v - m_params->motor.r * m_i_uvw.v) * m_params->sys.samp.tsim / l_ave;
        m_i_uvw.w += (m_v_uvw_n->w - m_e_uvw_n.w - m_params->motor.r * m_i_uvw.w) * m_params->sys.samp.tsim / l_ave;
        m_i_n = (m_i_uvw.v + m_i_uvw.w) / 2.0f;
        break;

    case 0b010:
        m_i_uvw.u += (m_v_uvw_n->u - m_e_uvw_n.u - m_params->motor.r * m_i_uvw.u) * m_params->sys.samp.tsim / l_ave;
        m_i_uvw.v = 0.0f;
        m_i_uvw.w += (m_v_uvw_n->w - m_e_uvw_n.w - m_params->motor.r * m_i_uvw.w) * m_params->sys.samp.tsim / l_ave;
        m_i_n = (m_i_uvw.u + m_i_uvw.w) / 2.0f;
        break;

    case 0b100:
        m_i_uvw.u += (m_v_uvw_n->u - m_e_uvw_n.u - m_params->motor.r * m_i_uvw.u) * m_params->sys.samp.tsim / l_ave;
        m_i_uvw.v += (m_v_uvw_n->v - m_e_uvw_n.v - m_params->motor.r * m_i_uvw.v) * m_params->sys.samp.tsim / l_ave;
        m_i_uvw.w = 0.0f;
        m_i_n = (m_i_uvw.u + m_i_uvw.v) / 2.0f;
        break;

    default:
        // FAULT
        m_i_uvw.u = NAN;
        m_i_uvw.v = NAN;
        m_i_uvw.w = NAN;
        m_i_n = NAN;
        break;
    }

    m_i_uvw.u -= m_i_n;
    m_i_uvw.v -= m_i_n;
    m_i_uvw.w -= m_i_n;

    m_i_s = MAX(MAX(ABS(m_i_uvw.u), ABS(m_i_uvw.v)), ABS(m_i_uvw.w));

    // Power and torque
    m_p_out = m_e_uvw_n.u * m_i_uvw.u + m_e_uvw_n.v * m_i_uvw.v + m_e_uvw_n.w * m_i_uvw.w;
    m_T = 0.5f * m_params->motor.P * m_params->motor.lam * (m_f_uvw.u * m_i_uvw.u + m_f_uvw.v * m_i_uvw.v + m_f_uvw.w * m_i_uvw.w);

}