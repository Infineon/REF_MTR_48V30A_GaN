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
#include "CSysSim.h"

#ifdef __cplusplus
extern "C"
{
#endif
#include "../OperationalCode/Controller.h"
#ifdef __cplusplus
}
#endif

PARAMS_t flash_params;
uint32_t flash_write_cnt;
bool SimFlashWrite(PARAMS_t* ram_data)
{
    flash_params = *ram_data;
    flash_write_cnt++;
    return true;
}
bool SimFlashRead(PARAMS_ID_t id, PARAMS_t* ram_data)
{
    if (flash_params.id.code != id.code || flash_params.id.build_config != id.build_config || flash_params.id.ver != id.ver)
    {
        return false;
    }
    else
    {
        *ram_data = flash_params;
        return true;
    }
}

bool hw_iface_reinit_done;
uint32_t hw_iface_reinit_cnt;
void SimHardwareIfaceInit()
{
    hw_iface_reinit_cnt++;
    hw_iface_reinit_done = true;
}

TEST_GROUP(FcnExeHandlerTest)
{
    CSysSim* sys_sim;

    void setup()
    {
        sys_sim = new CSysSim();
        sys_sim->m_pot.m_init = 0.0f;		// [%]
        sys_sim->m_pot.m_final = 1.0f;		// [%]
        sys_sim->m_pot.m_t_start = 0.5f;	// [sec]
        sys_sim->m_pot.m_t_stop = 3.5f;		// [sec]
        sys_sim->m_pot.m_t_slope = 2.5f;	// [sec]

        params.ctrl.mode = Volt_Mode_Open_Loop;
        PARAMS_DEFAULT_InitAutoCalc();
        STATE_MACHINE_Init();

        sensor_iface.digital.dir = 1U;

        hw_fcn.FlashWrite = SimFlashWrite;
        hw_fcn.FlashRead = SimFlashRead;
        flash_params = { 0 };
        flash_write_cnt = 0U;

        hw_fcn.HardwareIfaceInit = SimHardwareIfaceInit;
        hw_iface_reinit_done = false;
        hw_iface_reinit_cnt = 0U;
    }

    void teardown()
    {
        delete sys_sim;
    }

    void Handshake(FCN_EXE_LABEL_t fcn_label) // handshaking between gui and fw when requesting a fcn execution
    {
        FCN_EXE_REG_t fcn_mask = FCN_EXE_HANDLER_Mask(fcn_label);

        CHECK_FALSE(fcn_exe_handler.req & fcn_mask);

        // gui waits for previous execution to finish
        while (fcn_exe_handler.ack & fcn_mask)
        {
            sys_sim->RunOneTick();
        };

        // gui requests a new execution and waits for acknowledgement
        fcn_exe_handler.req |= fcn_mask;
        while (!(fcn_exe_handler.ack & fcn_mask))
        {
            sys_sim->RunOneTick();
        };

        // gui clears the request and waits for completion
        fcn_exe_handler.req &= ~fcn_mask;
        while (fcn_exe_handler.ack & fcn_mask)
        {
            sys_sim->RunOneTick();
        };

        CHECK_TRUE(fcn_exe_handler.done & fcn_mask);
    }
};

TEST(FcnExeHandlerTest, FlashParamsNotInInitState)
{
    // Pass init state
    while (sm.current == Init)
    {
        sys_sim->RunOneTick();
    }
    CHECK_TRUE(sm.current == Brake_Boot);

    // Initiate handshake
    Handshake(Flash_Params);

    // Verify that flash was not written to
    bool flash_check_ok = SimFlashRead({ PARAMS_CODE, BUILD_CONFIG_ID, PARAMS_VER }, &params);
    CHECK_FALSE(flash_check_ok);
    CHECK_TRUE(flash_write_cnt == 0U);
}

TEST(FcnExeHandlerTest, FlashParamsInInitState)
{
    // In init state
    sys_sim->RunOneTick();
    CHECK_TRUE(sm.current == Init);

    // Initiate handshake
    Handshake(Flash_Params);

    // Verify that flash was written to
    bool flash_check_ok = SimFlashRead({ PARAMS_CODE, BUILD_CONFIG_ID, PARAMS_VER }, &params);
    CHECK_TRUE(flash_check_ok);
    CHECK_TRUE(flash_write_cnt == 1U);
}

TEST(FcnExeHandlerTest, RestModulesNotInInitState)
{
    // Pass init state
    while (sm.current == Init)
    {
        sys_sim->RunOneTick();
    }
    CHECK_TRUE(sm.current == Brake_Boot);

    // Initiate handshake
    Handshake(Reset_Modules);

    // Verify that modules reset was not performed
    CHECK_FALSE(hw_iface_reinit_done);
    CHECK_TRUE(hw_iface_reinit_done == 0U);
}

TEST(FcnExeHandlerTest, RestModulesInInitState)
{
    // In init state
    sys_sim->RunOneTick();
    CHECK_TRUE(sm.current == Init);

    // Initiate handshake
    Handshake(Reset_Modules);

    // Verify that modules reset was performed
    CHECK_TRUE(hw_iface_reinit_done);
    CHECK_TRUE(hw_iface_reinit_done == 1U);
}

TEST(FcnExeHandlerTest, AutoCalcParamsNotInInitState)
{
    // Pass init state
    while (sm.current == Init)
    {
        sys_sim->RunOneTick();
    }
    CHECK_TRUE(sm.current == Brake_Boot);

    // Initiate handshake
    float prev_ctrl_speed_kp = params.ctrl.speed.kp;
    float prev_ctrl_speed_ki = params.ctrl.speed.ki;
    params.ctrl.speed.bw = 0.0f;
    Handshake(Auto_Calc_Params);

    // Verify parameters auto calculation was not performed
    DOUBLES_EQUAL_PERC(prev_ctrl_speed_kp, params.ctrl.speed.kp, 0.01f);
    DOUBLES_EQUAL_PERC(prev_ctrl_speed_ki, params.ctrl.speed.ki, 0.01f);
}

TEST(FcnExeHandlerTest, AutoCalcParamsInInitState)
{
    // In init state
    sys_sim->RunOneTick();
    CHECK_TRUE(sm.current == Init);

    // Initiate handshake
    float prev_ctrl_speed_kp = params.ctrl.speed.kp;
    float prev_ctrl_speed_ki = params.ctrl.speed.ki;
    params.ctrl.speed.bw = 0.0f;
    Handshake(Auto_Calc_Params);

    // Verify parameters auto calculation was not performed
    DOUBLES_EQUAL_PERC(0.0f, params.ctrl.speed.kp, 1.0E-6f);
    DOUBLES_EQUAL_PERC(0.0f, params.ctrl.speed.ki, 1.0E-6f);

}