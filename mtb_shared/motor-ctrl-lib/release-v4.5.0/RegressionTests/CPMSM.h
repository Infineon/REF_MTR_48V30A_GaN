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
#include "CMotor.h"

#ifdef __cplusplus
extern "C"
{
#endif
#include "../OperationalCode/Controller.h"
#ifdef __cplusplus
}
#endif

class CPMSM : public CMotor
{
public:
    CPMSM();
    CPMSM(PARAMS_t* params, UVW_t* v_uvw_n, float* w_r_mech, float* th_r_mech);
    ~CPMSM();

    void SetDiscreteIntegMethod(CDiscreteInteg::EMethod method);
    void SetSaturationParams(EN_DIS_t en_dis, QD_t i_qd_r_sat = { 0.0f, 0.0f });

    void RunSim();

    AB_t m_v_ab;
    QD_t m_v_qd_r;
    QD_t m_v_qd_s;

    QD_t m_la_qd_r;
    QD_t m_la_qd_s;

    AB_t m_i_ab;
    QD_t m_i_qd_r;
    QD_t m_i_qd_s;

    ELEC_t m_th_s;
    ELEC_t m_delta;

private:
    void (CPMSM::* RunDifferentialEquations)();
    void RunWithoutSaturation();
    void RunWithSaturation();

    CDiscreteInteg m_integ_q;
    CDiscreteInteg m_integ_d;

    QD_t m_i_qd_r_sat;
    QD_t m_i_qd_r_0;

    PARK_t m_park_r;
    PARK_t m_park_s;
};
