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


#include "CSysSim.h"

CSysSim::CSysSim() :
    m_pot(),
    m_inverter(),
    m_motor_type(CMotor::EType::PMSM),
    m_pmsm(),
    m_bldc(),
    m_load_type(CLoad::EType::Passive_Mech_Load),
    m_mech_load(),
    m_dyno(),
    m_feedback_type(CFeedback::EType::Hall_Sensor),
    m_hall_sensor(),
    m_sim_ctr(0U),
    m_sim_ctr_prev(0U)
{
    // Connecting simulation blocks
    m_pot.SetInputPointers(&params);

    m_motor = &m_pmsm;
    m_load = &m_mech_load;
    m_feedback = &m_hall_sensor;

    SetMotorType(m_motor_type);
    SetLoadType(m_load_type);
    SetFeedbackType(m_feedback_type);

    m_inverter.SetInputPointers(&vars.d_uvw_cmd, &m_motor->m_i_uvw, &m_vdc);

    // Set integration method
    SetDiscreteIntegMethod(CDiscreteInteg::EMethod::Bilinear);

    // Register ISR0/ISR1 callbacks
    RegisterCallbackISR0(&STATE_MACHINE_RunISR0);
    RegisterCallbackISR1(&STATE_MACHINE_RunISR1);

    // Init
    ResetOperationalCode();
    PARAMS_DEFAULT_Init();
    STATE_MACHINE_Init();
    m_vdc = params.sys.vdc_nom;
    hall.time_cap_freq = params.sys.samp.fsim;
    vars.d_uvw_cmd = { 0.0f, 0.0f, 0.0f };
}

void CSysSim::SetMotorType(CMotor::EType motor_type)
{
    m_motor_type = motor_type;
    switch (m_motor_type)
    {
    default:
    case CMotor::EType::PMSM:
        m_motor = &m_pmsm;
        break;
    case CMotor::EType::BLDC:
        m_motor = &m_bldc;
        break;
    }
    m_motor->SetInputPointers(&params, &m_inverter.m_v_uvw_n, &m_load->m_w.mech, &m_load->m_th.mech);
    m_load->SetInputPointers(&params, &m_motor->m_T);
}

void CSysSim::SetLoadType(CLoad::EType load_type)
{
    m_load_type = load_type;
    switch (m_load_type)
    {
    default:
    case CLoad::EType::Passive_Mech_Load:
        m_load = &m_mech_load;
        break;
    case CLoad::EType::Active_Dyno:
        m_load = &m_dyno;
        break;
    }
    m_load->SetInputPointers(&params, &m_motor->m_T);
    m_motor->SetInputPointers(&params, &m_inverter.m_v_uvw_n, &m_load->m_w.mech, &m_load->m_th.mech);
    m_feedback->SetInputPointers(&params, &m_load->m_th.mech);
}

void CSysSim::SetFeedbackType(CFeedback::EType feedback_type)
{
    m_feedback_type = feedback_type;
    switch (m_feedback_type)
    {
    default:
    case CFeedback::EType::Hall_Sensor:
        m_feedback = &m_hall_sensor;
        break;
    case CFeedback::EType::AqB_Encoder:
    case CFeedback::EType::Resolver:
        // TBD
        break;
    }
    m_feedback->SetInputPointers(&params, &m_load->m_th.mech);
}

void CSysSim::ResetSim(MECH_t w_0, MECH_t th_0)
{
    m_load->Reset(w_0, th_0);
}

CSysSim::CSysSim(CMotor::EType motor_type, CLoad::EType load_type, CFeedback::EType feedback_type, MECH_t w_0, MECH_t th_0) :
    CSysSim()
{
    SetMotorType(motor_type);
    SetLoadType(load_type);
    SetFeedbackType(feedback_type);
    ResetSim(w_0, th_0);
}

CSysSim::~CSysSim()
{
    ResetOperationalCode();
};

void CSysSim::RegisterCallbackISR0(void(*FcnISR0)())
{
    CallbackISR0 = FcnISR0;
}

void CSysSim::RegisterCallbackISR1(void(*FcnISR1)())
{
    CallbackISR1 = FcnISR1;
}

void CSysSim::RunOneTick()
{
    m_pot.RunSim();
    m_inverter.RunSim();
    m_motor->RunSim();
    m_load->RunSim();
    m_feedback->RunSim();

    if ((m_sim_ctr % params.sys.samp.fsim_fs0_ratio) == 0U)
    {
        sensor_iface.v_dc.raw = m_vdc;
        sensor_iface.i_samp_0.raw = m_motor->m_i_uvw.u;
        sensor_iface.i_samp_1.raw = m_motor->m_i_uvw.v;
        sensor_iface.i_samp_2.raw = m_motor->m_i_uvw.w;
        sensor_iface.pot.raw = m_pot.m_output;
        vars.w_fb.mech = m_load->m_w.mech;
        vars.th_r_fb.mech = m_load->m_th.mech;
        vars.w_fb.elec = m_motor->m_w_r.elec;
        vars.th_r_fb.elec = m_motor->m_th_r.elec;
#if defined(CTRL_METHOD_SFO)
        vars.delta_fb.elec = (m_motor_type == CMotor::EType::PMSM) ? (m_pmsm.m_delta.elec) : 0.0f;
#endif
#if defined(CTRL_METHOD_RFO) || defined(CTRL_METHOD_TBC)
        if (m_feedback_type == CFeedback::EType::Hall_Sensor)
        {
            hall.signal.uvw = m_hall_sensor.m_hall_signal.uvw;
            hall.period_cap = m_hall_sensor.m_period_cap;
        }
#endif
        CallbackISR0();

        if ((m_sim_ctr % (params.sys.samp.fsim_fs0_ratio * params.sys.samp.fs0_fs1_ratio)) == 0U)
        {
            CallbackISR1();
        }
    }

    ++m_sim_ctr;
}

void CSysSim::RunFor(float time)
{
    const uint32_t Time_Ticks = static_cast<uint32_t>(time * params.sys.samp.fsim);
    for (uint32_t index = 0U; index < Time_Ticks; ++index)
    {
        RunOneTick();
    }
}

void CSysSim::RunUntil(float time)
{
    const uint32_t Time_Ticks = static_cast<uint32_t>(time * params.sys.samp.fsim);
    while (m_sim_ctr < Time_Ticks)
    {
        RunOneTick();
    }
}

void CSysSim::ResetOperationalCode()
{
    params = { 0 };
    sm = { 0 };
    ctrl = { 0 };
    obs = { 0 };
    vars = { 0 };
    sensor_iface = { 0 };
    faults = { 0 };
    protect = { 0 };
}

float CSysSim::GetAbsTime()
{
    float result = static_cast<float>(m_sim_ctr) * params.sys.samp.tsim;
    return result;
}

float CSysSim::GetDiffTime()
{
    float result = static_cast<float>(m_sim_ctr - m_sim_ctr_prev) * params.sys.samp.tsim;
    m_sim_ctr_prev = m_sim_ctr;
    return result;
}

void CSysSim::SetDiscreteIntegMethod(CDiscreteInteg::EMethod method)
{
    m_motor->SetDiscreteIntegMethod(method);
    m_load->SetDiscreteIntegMethod(method);
}

uint64_t CSysSim::TimeToSysSimTick(float time)
{
    return static_cast<uint64_t>(time * params.sys.samp.fsim);
}

float CSysSim::SysSimTickToTime(uint64_t ticks)
{
    return (ticks * params.sys.samp.tsim);
}