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
#include "AuxMethods.h"

#ifdef __cplusplus
extern "C"
{
#endif
#include "../OperationalCode/Controller.h"
#ifdef __cplusplus
}
#endif

// Note: NaN (0xFFFFFFFF) indicates high-z state in input voltages to the motor.

class CMotor // Base class for CPMSM and CBLDC
{
public:
    enum class EType
    {
        PMSM = 0,	// Sinusoidal back-emf, can be IPM or SPM
        BLDC = 1,	// Trapezoidal back-emf
    };

    CMotor();
    void SetInputPointers(PARAMS_t* params, UVW_t* v_uvw_n, float* w_r_mech, float* th_r_mech);
    CMotor(PARAMS_t* params, UVW_t* v_uvw_n, float* w_r_mech, float* th_r_mech);
    ~CMotor();

    virtual void SetDiscreteIntegMethod(CDiscreteInteg::EMethod method) {};
    virtual void RunSim() {};

    UVW_t m_i_uvw;
    float m_i_n;

    ELEC_t m_th_r;
    ELEC_t m_w_r;

    float m_T;

protected:
    PARAMS_t* m_params;
    UVW_t* m_v_uvw_n;
    float* m_w_r_mech;
    float* m_th_r_mech;
};