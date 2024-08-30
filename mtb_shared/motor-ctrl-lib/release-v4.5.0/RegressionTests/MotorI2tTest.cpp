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


#include "AuxMethods.h"
#include "AuxMethodTemplates.cpp" // Template definitions are in the source file

#ifdef __cplusplus
extern "C"
{
#endif
#include "../OperationalCode/General.h"
#include "../OperationalCode/Params.h"
#include "../OperationalCode/CtrlVars.h"
#include "../OperationalCode/FaultProtect.h"
#ifdef __cplusplus
}
#endif

TEST_GROUP(MotorI2tTest)
{
    void setup()
    {
        PARAMS_DEFAULT_Init();

        params.motor.i_peak = 100.0f; // [A]
        params.motor.i_cont = 50.0f; // [A]
        params.motor.i2t.therm_tau = 0.5f; // [sec]
        params.motor.i2t.on_level = 1.00f; // [%]
        params.motor.i2t.off_level = 0.95f; // [%]
        params.sys.faults.oc_thresh = 1.20f; // [%]
        params.sys.samp.fs0 = 20.0E3; // [Hz]
        params.sys.samp.ts0 = 1.0f / params.sys.samp.fs0; // [sec]
        params.ctrl.mode = Volt_Mode_Open_Loop;

        FAULT_PROTECT_Init();
    }

    void teardown()
    {
    }
};

TEST(MotorI2tTest, RmsCurrentEstimation)
{
    const float I_Expected = params.motor.i_peak;
    const float Run_Time_Sec = params.motor.i2t.therm_tau * 10.0f;
    const uint32_t Run_Time_Ticks = static_cast<uint32_t>(Run_Time_Sec * params.sys.samp.fs0);

    FAULT_PROTECT_Reset();

    for (uint32_t index = 0U; index < Run_Time_Ticks; ++index)
    {
        vars.i_ab_fb_tot.alpha = I_Expected * cosf(static_cast<float>(index) * params.sys.samp.ts0);
        vars.i_ab_fb_tot.beta = -I_Expected * sinf(static_cast<float>(index) * params.sys.samp.ts0);
        FAULT_PROTECT_RunISR0();
        if ((index % params.sys.samp.fs0_fs1_ratio) == 0U)
        {
            FAULT_PROTECT_RunISR1();
        }
    }

    DOUBLES_EQUAL_PERC(I_Expected, protect.motor.i2t.i_filt, 0.01f);
};


TEST(MotorI2tTest, CurrentProtectionTrigger)
{
    const float I_Input = params.motor.i_peak * 0.7f + params.motor.i_cont * 0.3f;
    const float Run_Time_Sec = params.motor.i2t.therm_tau * 1.3f;
    const uint32_t Run_Time_Ticks = static_cast<uint32_t>(Run_Time_Sec * params.sys.samp.fs0);

    FAULT_PROTECT_Reset();

    float trigger_time_actual = 0.0f;
    EN_DIS_t protect_motor_i2t_state_prev = Dis;
    for (uint32_t index = 0U; index < Run_Time_Ticks; ++index)
    {
        vars.i_ab_fb_tot.alpha = I_Input * cosf(static_cast<float>(index) * params.sys.samp.ts0);
        vars.i_ab_fb_tot.beta = -I_Input * sinf(static_cast<float>(index) * params.sys.samp.ts0);
        FAULT_PROTECT_RunISR0();
        if ((index % params.sys.samp.fs0_fs1_ratio) == 0U)
        {
            FAULT_PROTECT_RunISR1();
            if ((protect.motor.i2t.state == En) && (protect_motor_i2t_state_prev == Dis)) // rising edge
            {
                trigger_time_actual = static_cast<float>(index) * params.sys.samp.ts0;
            }
            protect_motor_i2t_state_prev = protect.motor.i2t.state;
        }
    }

    const float I_Expected = I_Input * sqrtf(1.0f - expf(-Run_Time_Sec / params.motor.i2t.therm_tau));
    DOUBLES_EQUAL_PERC(I_Expected, protect.motor.i2t.i_filt, 0.01f);

    const float Trigger_Time_Expected = params.motor.i2t.therm_tau * logf(POW_TWO(I_Input) / (POW_TWO(I_Input) - POW_TWO(params.motor.i2t.on_level * params.motor.i_cont)));
    DOUBLES_EQUAL_PERC(Trigger_Time_Expected, trigger_time_actual, 0.01f);

    DOUBLES_EQUAL_PERC(params.motor.i_cont, protect.motor.i2t.i_limit, 1.0E-6f);
};

TEST(MotorI2tTest, CurrentProtectionRecover)
{
    const float I_Init = params.motor.i_peak * 0.6f + params.motor.i_cont * 0.4f;
    const float I_Input = params.motor.i_cont * 0.3f;
    const float Run_Time_Sec = params.motor.i2t.therm_tau * 1.8f;
    const uint32_t Run_Time_Ticks = static_cast<uint32_t>(Run_Time_Sec * params.sys.samp.fs0);

    FAULT_PROTECT_Reset();
    protect.motor.i2t.i_sq_filt = POW_TWO(I_Init);
    protect.motor.i2t.state = En;
    protect.motor.i2t.i_limit = params.motor.i_cont;

    float recover_time_actual = 0.0f;
    EN_DIS_t protect_motor_i2t_state_prev = En;
    for (uint32_t index = 0U; index < Run_Time_Ticks; ++index)
    {
        vars.i_ab_fb_tot.alpha = I_Input * cosf(static_cast<float>(index) * params.sys.samp.ts0);
        vars.i_ab_fb_tot.beta = -I_Input * sinf(static_cast<float>(index) * params.sys.samp.ts0);
        FAULT_PROTECT_RunISR0();
        if ((index % params.sys.samp.fs0_fs1_ratio) == 0U)
        {
            FAULT_PROTECT_RunISR1();
            if ((protect.motor.i2t.state == Dis) && (protect_motor_i2t_state_prev == En)) // falling edge
            {
                recover_time_actual = static_cast<float>(index) * params.sys.samp.ts0;
            }
            protect_motor_i2t_state_prev = protect.motor.i2t.state;
        }
    }

    const float I_Expected = sqrtf(POW_TWO(I_Input) + (POW_TWO(I_Init) - POW_TWO(I_Input)) * expf(-Run_Time_Sec / params.motor.i2t.therm_tau));
    DOUBLES_EQUAL_PERC(I_Expected, protect.motor.i2t.i_filt, 0.01f);

    const float Recover_Time_Expected = params.motor.i2t.therm_tau * logf((POW_TWO(I_Input) - POW_TWO(I_Init)) / (POW_TWO(I_Input) - POW_TWO(params.motor.i2t.off_level * params.motor.i_cont)));
    DOUBLES_EQUAL_PERC(Recover_Time_Expected, recover_time_actual, 0.01f);

    DOUBLES_EQUAL_PERC(params.motor.i_peak, protect.motor.i2t.i_limit, 1.0E-6f);
};

TEST(MotorI2tTest, CurrentProtectionFault)
{
    const float I_Init = params.motor.i_cont * 0.9f;
    const float I_Input = params.motor.i_peak * 0.8f + params.motor.i_cont * 0.2f;
    const float Run_Time_Sec = params.motor.i2t.therm_tau * 2.0f;
    const uint32_t Run_Time_Ticks = static_cast<uint32_t>(Run_Time_Sec * params.sys.samp.fs0);

    FAULT_PROTECT_Reset();
    protect.motor.i2t.i_sq_filt = POW_TWO(I_Init);
    protect.motor.i2t.state = Dis;
    protect.motor.i2t.i_limit = params.motor.i_peak;

    float fault_time_actual = 0.0f;
    bool fault_flags_sw_oc_prev = false;
    for (uint32_t index = 0U; index < Run_Time_Ticks; ++index)
    {
        vars.i_ab_fb_tot.alpha = I_Input * cosf(static_cast<float>(index) * params.sys.samp.ts0);
        vars.i_ab_fb_tot.beta = -I_Input * sinf(static_cast<float>(index) * params.sys.samp.ts0);
        FAULT_PROTECT_RunISR0();
        if ((index % params.sys.samp.fs0_fs1_ratio) == 0U)
        {
            FAULT_PROTECT_RunISR1();
            if ((faults.flags.sw.oc == true) && (fault_flags_sw_oc_prev == false)) // rising edge
            {
                fault_time_actual = static_cast<float>(index) * params.sys.samp.ts0;
            }
            fault_flags_sw_oc_prev = faults.flags.sw.oc;
        }
    }

    const float I_Expected = sqrtf(POW_TWO(I_Input) + (POW_TWO(I_Init) - POW_TWO(I_Input)) * expf(-Run_Time_Sec / params.motor.i2t.therm_tau));
    DOUBLES_EQUAL_PERC(I_Expected, protect.motor.i2t.i_filt, 0.01f);

    const float Fault_Time_Expected = params.motor.i2t.therm_tau * logf((POW_TWO(I_Input) - POW_TWO(I_Init)) / (POW_TWO(I_Input) - POW_TWO(params.sys.faults.oc_thresh * params.motor.i_cont)));
    DOUBLES_EQUAL_PERC(Fault_Time_Expected, fault_time_actual, 0.01f);

    DOUBLES_EQUAL_PERC(params.motor.i_cont, protect.motor.i2t.i_limit, 1.0E-6f);
};