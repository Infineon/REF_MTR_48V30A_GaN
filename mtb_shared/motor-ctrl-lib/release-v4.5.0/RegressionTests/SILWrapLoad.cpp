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


#include "SILWrapLoad.h"
#include "SILAuxFcns.h"
#include "AuxMethods.h"
#include "CMechLoad.h"
#include "CDyno.h"

// Simulation constants .............................................................
// Select among supported load types:
const auto Load_Type = CLoad::EType::Passive_Mech_Load;
// Bilinear has lower phase distortion, Backward_Euler has lower amplitude distortion:
const auto Discrete_Integ_Method = CDiscreteInteg::EMethod::Bilinear;
// For testing initial position (six pulse and high frequency injection):
const ELEC_t Init_Angle = { DEG_TO_RAD(45.0f) };


// ...................................................................................

static CLoad* load = nullptr;
static CMechLoad mech_load;
static CDyno dyno;
static float T;

void SIL_StartLoad(PARAMS_t* params_final)
{
    params = *params_final;

    switch (Load_Type)
    {
    case CLoad::EType::Active_Dyno:
        load = &dyno;
        break;
    case CLoad::EType::Passive_Mech_Load:
        load = &mech_load;
    default:
        break;
    }

    load->SetInputPointers(params_final, &T);
    load->SetDiscreteIntegMethod(Discrete_Integ_Method);
    load->Reset({ 0.0f }, { ELEC_TO_MECH(Init_Angle.elec, params.motor.P) });
}

void SIL_TerminateLoad()
{
    // Nothing to do here
}

void SIL_RunLoad(SIL_INPUT_LOAD_t* input, SIL_OUTPUT_LOAD_t* output)
{
    // Read inputs .............................................................................................
#pragma warning(disable:4244)	// implicit conversion of double inputs from MATLAB to float
    T = input->T;
#pragma warning(default:4244)	// implicit conversion of double inputs from MATLAB to float

    // Run model ...............................................................................................
    load->RunSim();

    // Write outputs ...........................................................................................
    output->T_inertia = (Load_Type == CLoad::EType::Passive_Mech_Load) ? mech_load.m_T_inertia : 0.0f;
    output->T_viscous = (Load_Type == CLoad::EType::Passive_Mech_Load) ? mech_load.m_T_viscous : 0.0f;
    output->T_friction = (Load_Type == CLoad::EType::Passive_Mech_Load) ? mech_load.m_T_friction : 0.0f;
    output->T_load = load->m_T;

    output->w_r_mech = load->m_w.mech;
    output->th_r_mech = load->m_th.mech;
}