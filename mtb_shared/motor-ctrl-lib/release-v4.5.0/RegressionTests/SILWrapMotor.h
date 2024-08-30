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


#pragma once

#ifdef __cplusplus
extern "C"
{
#endif
#include "../OperationalCode/Params.h"
#ifdef __cplusplus
}
#endif

typedef struct
{
    double v_uvw_n[3];		// {u, v, w}
    double w_r_mech;		// mech
    double th_r_mech;		// mech
} SIL_INPUT_MOTOR_t;

typedef struct
{
    double e_uvw_n[3];		// {u, v, w}, TBC
    double v_ab[2];			// {a, b}
    double v_qd_r[2];		// {q, d}, RFO
    double v_qd_s[2];		// {q, d}, SFO

    double la_qd_r[2];		// {q, d}, RFO
    double la_qd_s[2];		// {q, d}, SFO

    double i_s;				// TBC
    double i_uvw[3];		// {u, v, w}
    double i_ab[2];			// {a, b}
    double i_qd_r[2];		// {q, d}, RFO
    double i_qd_s[2];		// {q, d}, SFO

    double th_r_elec;		// elec
    double th_s_elec;		// elec
    double delta_elec;		// elec
    double w_r_elec;		// elec

    double T;
} SIL_OUTPUT_MOTOR_t;

void SIL_StartMotor(PARAMS_t* params_final);
void SIL_TerminateMotor();
void SIL_RunMotor(SIL_INPUT_MOTOR_t* input, SIL_OUTPUT_MOTOR_t* output);