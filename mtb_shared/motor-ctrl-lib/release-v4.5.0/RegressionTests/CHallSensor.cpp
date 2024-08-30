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


#include "CHallSensor.h"

CHallSensor::CHallSensor() : CFeedback(),
Hall_U_Angle({ DEG_TO_RAD(-60.0f) }),
Hall_V_Angle({ DEG_TO_RAD(+60.0f) }),
Hall_W_Angle({ DEG_TO_RAD(-180.0f) }),
m_hall_signal({ 0U }),
m_hall_signal_xor(0U),
m_period_cap(UINT32_MAX),
m_hall_signal_prev({ 0U }),
m_hall_signal_xor_prev(0U),
m_time(0U),
m_time_cap(0U),
m_th_r_elec(0.0f)
{
    m_th_r_elec = 0.0f;
    m_hall_signal.u = IsHallSignalActive(Hall_U_Angle);
    m_hall_signal.v = IsHallSignalActive(Hall_V_Angle);
    m_hall_signal.w = IsHallSignalActive(Hall_W_Angle);
    m_hall_signal_prev = m_hall_signal;

    m_hall_signal_xor = m_hall_signal.u ^ m_hall_signal.v ^ m_hall_signal.w;
    m_hall_signal_xor_prev = m_hall_signal_prev.u ^ m_hall_signal_prev.v ^ m_hall_signal_prev.w;
};

CHallSensor::CHallSensor(PARAMS_t* params, float* th_r_mech) : CFeedback(params, th_r_mech),
Hall_U_Angle({ DEG_TO_RAD(-60.0f) }),
Hall_V_Angle({ DEG_TO_RAD(+60.0f) }),
Hall_W_Angle({ DEG_TO_RAD(-180.0f) }),
m_hall_signal({ 0U }),
m_hall_signal_xor(0U),
m_period_cap(UINT32_MAX),
m_hall_signal_prev({ 0U }),
m_hall_signal_xor_prev(0U),
m_time(0U),
m_time_cap(0U),
m_th_r_elec(0.0f)
{
    m_th_r_elec = Wrap2Pi(MECH_TO_ELEC(*m_th_r_mech, m_params->motor.P));;
    m_hall_signal.u = IsHallSignalActive(Hall_U_Angle);
    m_hall_signal.v = IsHallSignalActive(Hall_V_Angle);
    m_hall_signal.w = IsHallSignalActive(Hall_W_Angle);
    m_hall_signal_prev = m_hall_signal;

    m_hall_signal_xor = m_hall_signal.u ^ m_hall_signal.v ^ m_hall_signal.w;
    m_hall_signal_xor_prev = m_hall_signal_prev.u ^ m_hall_signal_prev.v ^ m_hall_signal_prev.w;
};

CHallSensor::~CHallSensor() {};

inline bool CHallSensor::IsHallSignalActive(const ELEC_t hall_angle)
{
    ELEC_t rel_angle = { Wrap2Pi(m_th_r_elec - hall_angle.elec) };
    return ((0.0f <= rel_angle.elec) && (rel_angle.elec < DEG_TO_RAD(180.0f)));
}

void CHallSensor::RunSim()
{
    m_th_r_elec = Wrap2Pi(MECH_TO_ELEC(*m_th_r_mech, m_params->motor.P));

    m_hall_signal_prev = m_hall_signal;
    m_hall_signal.u = IsHallSignalActive(Hall_U_Angle);
    m_hall_signal.v = IsHallSignalActive(Hall_V_Angle);
    m_hall_signal.w = IsHallSignalActive(Hall_W_Angle);

    m_hall_signal_xor = m_hall_signal.u ^ m_hall_signal.v ^ m_hall_signal.w;
    m_hall_signal_xor_prev = m_hall_signal_prev.u ^ m_hall_signal_prev.v ^ m_hall_signal_prev.w;

    ++m_time;   // one free-running timer/counter
    if (TRANS_EDGE(m_hall_signal_xor_prev, m_hall_signal_xor))
    {
        m_period_cap = m_time - m_time_cap;
        m_time_cap = m_time;
    }

}