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
#include "CFeedback.h"

#ifdef __cplusplus
extern "C"
{
#endif
#include "../OperationalCode/Controller.h"
#ifdef __cplusplus
}
#endif


class CHallSensor : public CFeedback
{
public:

    CHallSensor();
    CHallSensor(PARAMS_t* params, float* th_r_mech);
    ~CHallSensor();

    void RunSim();

    UVW_SIGNAL_t m_hall_signal;
    bool m_hall_signal_xor;         // six steps per revolution
    uint32_t m_period_cap;          // [ticks], timer/counter capture value

private:
    inline bool IsHallSignalActive(const ELEC_t hall_angle);

    UVW_SIGNAL_t m_hall_signal_prev;
    bool m_hall_signal_xor_prev;    // six steps per revolution
    uint32_t m_time;                // [ticks], timer/counter value
    uint32_t m_time_cap;            // [ticks], timer/counter capture value

    float m_th_r_elec;

    // Rotor's electrical angle is 90 degrees ahead of permanent magnet's flux angle.
    // Hall sensors are placed 120 electrical degrees apart.
    const ELEC_t Hall_U_Angle;
    const ELEC_t Hall_V_Angle;
    const ELEC_t Hall_W_Angle;
};
