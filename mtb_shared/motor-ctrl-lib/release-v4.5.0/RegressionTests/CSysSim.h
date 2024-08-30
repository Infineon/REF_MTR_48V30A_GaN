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
#include "CPotentiometer.h"
#include "CInverter.h"
#include "CPMSM.h"
#include "CBLDC.h"
#include "CMechLoad.h"
#include "CDyno.h"
#include "CHallSensor.h"

#ifdef __cplusplus
extern "C"
{
#endif
#include "../OperationalCode/Controller.h"
#ifdef __cplusplus
}
#endif

class CSysSim
{
public:
    CSysSim();
    CSysSim(CMotor::EType motor_type, CLoad::EType load_type, CFeedback::EType feedback_type, MECH_t w_0, MECH_t th_0);
    ~CSysSim();

    void ResetSim(MECH_t w_0, MECH_t th_0);
    void SetMotorType(CMotor::EType motor_type);
    void SetLoadType(CLoad::EType load_type);
    void SetFeedbackType(CFeedback::EType feedback_type);

    void SetDiscreteIntegMethod(CDiscreteInteg::EMethod method);
    static void ResetOperationalCode();

    // For running other functions at ISR0/ISR1 instead of controller:
    void RegisterCallbackISR0(void(*FcnISR0)());
    void RegisterCallbackISR1(void(*FcnISR1)());

    static uint64_t TimeToSysSimTick(float time);
    static float SysSimTickToTime(uint64_t ticks);

    void RunOneTick();
    void RunFor(float time);	// [sec]
    void RunUntil(float time);	// [sec]
    float GetAbsTime();
    float GetDiffTime();

    float m_vdc;
    CPotentiometer m_pot;
    CInverter m_inverter;

    // Different motors
    CMotor::EType m_motor_type;
    CMotor* m_motor;
    CPMSM m_pmsm;
    CBLDC m_bldc;

    // Different loads
    CLoad::EType m_load_type;
    CLoad* m_load;
    CMechLoad m_mech_load;
    CDyno m_dyno;

    // Different feedbacks
    CFeedback::EType m_feedback_type;
    CFeedback* m_feedback;
    CHallSensor m_hall_sensor;

private:
    void (*CallbackISR0)();
    void (*CallbackISR1)();

    uint64_t m_sim_ctr;
    uint64_t m_sim_ctr_prev;
};