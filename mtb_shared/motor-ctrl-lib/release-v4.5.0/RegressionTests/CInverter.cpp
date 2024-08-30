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


#include "CInverter.h"

CInverter::CInverter() :
    m_d_uvw(nullptr),
    m_i_uvw(nullptr),
    m_vdc(nullptr),
    m_v_nz(0.0f)
{
    UVW_t UVW_0 = { 0.0f, 0.0f, 0.0f };
    m_v_uvw_z = UVW_0;
    m_v_uvw_n = UVW_0;
};

void CInverter::SetInputPointers(UVW_t* d_uvw, UVW_t* i_uvw, float* vdc)
{
    m_d_uvw = d_uvw;
    m_i_uvw = i_uvw;
    m_vdc = vdc;
}

CInverter::CInverter(UVW_t* d_uvw, UVW_t* i_uvw, float* vdc) :
    CInverter()
{
    SetInputPointers(d_uvw, i_uvw, vdc);
};

CInverter::~CInverter() {};

void CInverter::RunSim()
{
    m_v_uvw_z.u = (m_d_uvw->u - 0.5f) * (*m_vdc);
    m_v_uvw_z.v = (m_d_uvw->v - 0.5f) * (*m_vdc);
    m_v_uvw_z.w = (m_d_uvw->w - 0.5f) * (*m_vdc);

    float v_nz_num = (isnan(m_v_uvw_z.u) ? 0.0f : m_v_uvw_z.u) + (isnan(m_v_uvw_z.v) ? 0.0f : m_v_uvw_z.v) + (isnan(m_v_uvw_z.w) ? 0.0f : m_v_uvw_z.w);
    float v_nz_den = (isnan(m_v_uvw_z.u) ? 0.0f : 1.0f) + (isnan(m_v_uvw_z.v) ? 0.0f : 1.0f) + (isnan(m_v_uvw_z.w) ? 0.0f : 1.0f);
    m_v_nz = v_nz_num / v_nz_den;

    m_v_uvw_n.u = m_v_uvw_z.u - m_v_nz;
    m_v_uvw_n.v = m_v_uvw_z.v - m_v_nz;
    m_v_uvw_n.w = m_v_uvw_z.w - m_v_nz;
}