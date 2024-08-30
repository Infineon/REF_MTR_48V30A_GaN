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


#include "CLoad.h"

CLoad::CLoad() :
    m_params(nullptr),
    m_T_motor(nullptr),
    m_w_mech(CDiscreteInteg::EMethod::Bilinear),
    m_th_mech(CDiscreteInteg::EMethod::Bilinear),
    m_T(0.0f),
    m_w({ 0.0f }),
    m_th({ 0.0f })
{}

CLoad::~CLoad() {}

void CLoad::SetInputPointers(PARAMS_t* params, float* T)
{
    m_params = params;
    m_T_motor = T;
}

void CLoad::SetDiscreteIntegMethod(CDiscreteInteg::EMethod method)
{
    m_w_mech.SetMethod(method);
    m_th_mech.SetMethod(method);
}

void CLoad::Reset(MECH_t w_0, MECH_t th_0)
{
    m_w.mech = w_0.mech;
    m_th.mech = th_0.mech;

    m_w_mech.Reset(m_w.mech);
    m_th_mech.Reset(m_th.mech);
}
