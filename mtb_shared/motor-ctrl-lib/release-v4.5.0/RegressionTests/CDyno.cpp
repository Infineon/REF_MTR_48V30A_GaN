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


#include "CDyno.h"

CDyno::CDyno() :
    CLoad(),
    m_pot(),
    m_T_cmd(0.0f),
    m_w_cmd({ 0.0f }),
    m_th_cmd({ 0.0f }),
    m_w_prev({ 0.0f }),
    m_th_prev({ 0.0f })
{
    SetCtrlMode(CDyno::ECtrlMode::Speed_Ctrl);
}

CDyno::CDyno(PARAMS_t* params, float* T) : CDyno()
{
    SetInputPointers(params, T);
}

CDyno::~CDyno() {}

void CDyno::SetInputPointers(PARAMS_t* params, float* T)
{
    m_pot.SetInputPointers(params);
    m_params = params;
    m_T_motor = T;
}

void CDyno::SetCtrlMode(ECtrlMode ctrl_mode)
{
    m_ctrl_mode = ctrl_mode;
    switch (m_ctrl_mode)
    {
    default:
    case ECtrlMode::Speed_Ctrl:
        RunSimPtr = &CDyno::RunSimSpdCtrl;
        break;
    case ECtrlMode::Torque_Ctrl:
        RunSimPtr = &CDyno::RunSimTrqCtrl;
        break;
    case ECtrlMode::Position_Ctrl:
        RunSimPtr = &CDyno::RunSimPosCtrl;
        break;
    }
}

CDyno::ECtrlMode CDyno::GetCtrlMode()
{
    return m_ctrl_mode;
}

void CDyno::Reset(MECH_t w_0, MECH_t th_0)
{
    m_w.mech = w_0.mech;
    m_w_prev.mech = w_0.mech;
    m_th.mech = th_0.mech;
    m_th_prev.mech = th_0.mech;

    m_w_mech.Reset(m_w.mech);
    m_th_mech.Reset(m_th.mech);
}

void CDyno::RunSim()
{
    (this->*RunSimPtr)();
}

void CDyno::RunSimPosCtrl()
{
    // Position:
    m_pot.RunSim();
    m_th_prev.mech = m_th.mech;
    m_th.mech = m_pot.m_output * m_th_cmd.mech;

    // Speed:
    m_w_prev.mech = m_w.mech;
    m_w.mech = (m_th.mech - m_th_prev.mech) * (m_params->sys.samp.fsim);

    // Torque:
    m_T = (*m_T_motor) - (m_w.mech - m_w_prev.mech) * (m_params->sys.samp.fsim * m_params->mech.inertia);
}

void CDyno::RunSimSpdCtrl()
{
    // Speed:
    m_pot.RunSim();
    m_w_prev.mech = m_w.mech;
    m_w.mech = m_pot.m_output * m_w_cmd.mech;

    // Torque:
    m_T = (*m_T_motor) - (m_w.mech - m_w_prev.mech) * (m_params->sys.samp.fsim * m_params->mech.inertia);

    // Position:
    m_th_mech.m_integ = Wrap2Pi(m_th_mech.Run(m_w.mech * m_params->sys.samp.tsim));
    m_th_prev.mech = m_th.mech;
    m_th.mech = m_th_mech.m_integ;
}

void CDyno::RunSimTrqCtrl()
{
    // Torque:
    m_pot.RunSim();
    m_T = m_pot.m_output * m_T_cmd;

    // Speed:
    m_w_prev.mech = m_w.mech;
    m_w.mech = m_w_mech.Run(((*m_T_motor) - m_T) * (m_params->sys.samp.tsim) / (m_params->mech.inertia));

    // Position:
    m_th_mech.m_integ = Wrap2Pi(m_th_mech.Run(m_w.mech * m_params->sys.samp.tsim));
    m_th_prev.mech = m_th.mech;
    m_th.mech = m_th_mech.m_integ;
}
