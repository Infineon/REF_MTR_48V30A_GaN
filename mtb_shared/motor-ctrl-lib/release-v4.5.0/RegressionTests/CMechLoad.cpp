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


#include "CMechLoad.h"

CMechLoad::CMechLoad() :
    CLoad(),
    RunSimPtr(nullptr),
    m_T_inertia(0.0f),
    m_T_viscous(0.0f),
    m_T_friction(0.0f)
{
    UnlockPosition();

    m_w.mech = 0.0f;
    m_w_prev.mech = 0.0f;
    m_th.mech = 0.0f;
}

CMechLoad::CMechLoad(PARAMS_t* params, float* T) :
    CMechLoad()
{
    SetInputPointers(params, T);
}

void CMechLoad::Reset(MECH_t w_0, MECH_t th_0)
{
    m_w.mech = w_0.mech;
    m_w_prev.mech = w_0.mech;
    m_th.mech = th_0.mech;

    m_w_mech.Reset(m_w.mech);
    m_th_mech.Reset(m_th.mech);
}

CMechLoad::~CMechLoad() {}

void CMechLoad::RunSimUnlocked()
{
    const float Fs = m_params->sys.samp.fsim;

    m_T_inertia = m_params->mech.inertia * (m_w.mech - m_w_prev.mech) * Fs;
    m_T_viscous = m_params->mech.viscous * m_w.mech;
    m_T_friction = m_params->mech.friction * SIGN(m_w.mech);
    m_T = m_T_inertia + m_T_viscous + m_T_friction;
    m_w_prev.mech = m_w.mech;

    float d_w_mech = ((*m_T_motor) - m_T_viscous - m_T_friction) / (Fs * m_params->mech.inertia);
    m_w.mech = m_w_mech.Run(d_w_mech);

    m_th_mech.m_integ = Wrap2Pi(m_th_mech.Run(m_w.mech / Fs));
    m_th.mech = m_th_mech.m_integ;
}

void CMechLoad::RunSimLocked() {} // Nothing to do

void CMechLoad::RunSim()
{
    (this->*RunSimPtr)();
}

void CMechLoad::UnlockPosition()
{
    RunSimPtr = &CMechLoad::RunSimUnlocked;
}

void CMechLoad::LockPosition()
{
    m_w.mech = 0.0f;
    m_w_prev.mech = 0.0f;
    m_w_mech.Reset(0.0f);

    RunSimPtr = &CMechLoad::RunSimLocked;
}
