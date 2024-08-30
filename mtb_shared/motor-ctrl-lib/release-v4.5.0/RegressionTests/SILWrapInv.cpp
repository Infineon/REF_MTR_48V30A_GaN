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


#include "SILWrapInv.h"
#include "SILAuxFcns.h"
#include "CInverter.h"

// Simulation constants .............................................................


// ...................................................................................

static CInverter inverter;
static UVW_t d_uvw_cmd;
static UVW_t i_uvw;
static float vdc;

void SIL_StartInv(PARAMS_t* params_final)
{
    vdc = params_final->sys.vdc_nom;
    inverter.SetInputPointers(&d_uvw_cmd, &i_uvw, &vdc);
}

void SIL_TerminateInv()
{
    // Nothing to do here
}

void SIL_RunInv(SIL_INPUT_INV_t* input, SIL_OUTPUT_INV_t* output)
{
    // Read inputs .............................................................................................
    SIL_SetUVW(Read, input->d_uvw_cmd, &d_uvw_cmd);
    SIL_SetUVW(Read, input->i_uvw_cmd, &i_uvw);

    // Run model ...............................................................................................
    inverter.RunSim();

    // Write outputs ...........................................................................................
    output->v_nz = inverter.m_v_nz;
    SIL_SetUVW(Write, output->v_uvw_z, &inverter.m_v_uvw_z);
    SIL_SetUVW(Write, output->v_uvw_n, &inverter.m_v_uvw_n);
}