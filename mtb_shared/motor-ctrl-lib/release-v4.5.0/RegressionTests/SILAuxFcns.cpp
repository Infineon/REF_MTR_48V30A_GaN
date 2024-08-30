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


#include "SILAuxFcns.h"

void SIL_SetElecMech(DIR_t dir, double matlab_side[2], ELEC_MECH_t* c_side)
{
    if (dir == Read)
    {
        c_side->elec = (float)(matlab_side[0]);
        c_side->mech = (float)(matlab_side[1]);
    }
    else // Write
    {
        matlab_side[0] = (double)(c_side->elec);
        matlab_side[1] = (double)(c_side->mech);
    }
}

void SIL_SetUVW(DIR_t dir, double matlab_side[3], UVW_t* c_side)
{
    if (dir == Read)
    {
        c_side->u = (float)(matlab_side[0]);
        c_side->v = (float)(matlab_side[1]);
        c_side->w = (float)(matlab_side[2]);
    }
    else // Write
    {
        matlab_side[0] = (double)(c_side->u);
        matlab_side[1] = (double)(c_side->v);
        matlab_side[2] = (double)(c_side->w);
    }
}

void SIL_SetAB(DIR_t dir, double matlab_side[2], AB_t* c_side)
{
    if (dir == Read)
    {
        c_side->alpha = (float)(matlab_side[0]);
        c_side->beta = (float)(matlab_side[1]);
    }
    else // Write
    {
        matlab_side[0] = (double)(c_side->alpha);
        matlab_side[1] = (double)(c_side->beta);
    }
}

void SIL_SetQD(DIR_t dir, double matlab_side[2], QD_t* c_side)
{
    if (dir == Read)
    {
        c_side->q = (float)(matlab_side[0]);
        c_side->d = (float)(matlab_side[1]);
    }
    else // Write
    {
        matlab_side[0] = (double)(c_side->q);
        matlab_side[1] = (double)(c_side->d);
    }
}

void SIL_SetPolar(DIR_t dir, double matlab_side[2], POLAR_t* c_side)
{
    if (dir == Read)
    {
        c_side->rad = (float)(matlab_side[0]);
        c_side->theta = (float)(matlab_side[1]);
    }
    else // Write
    {
        matlab_side[0] = (double)(c_side->rad);
        matlab_side[1] = (double)(c_side->theta);
    }
}
