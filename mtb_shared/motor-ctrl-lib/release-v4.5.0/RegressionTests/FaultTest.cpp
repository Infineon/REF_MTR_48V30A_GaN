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

// Simulate HW high-z state:
static bool hw_high_z = false;
static void SimEnterHighZ() { hw_high_z = true; };
static void SimExitHighZ() { hw_high_z = false; };

// Simulate HW fault clearing:
static FAULT_FLAGS_t hw_fault_latched = { 0 };
static FAULT_FLAGS_t hw_clr_fault_mask = { 0 };
static const uint32_t Hw_Clr_Fault_Ticks = 2U; // ISR1 ticks needed for HW to clear the fault (based on SPI scheme)
static void CallbackISR0() { STATE_MACHINE_RunISR0(); };
static void CallbackISR1()
{
    STATE_MACHINE_RunISR1();
    static uint32_t hw_clr_fault_cnt = 0U;
    if ((sm.vars.fault.clr_request) && (hw_clr_fault_cnt == 0U))
    {
        hw_clr_fault_cnt = 1U;
    }
    if (0U < hw_clr_fault_cnt)
    {
        if (hw_clr_fault_cnt == Hw_Clr_Fault_Ticks)
        {
            hw_fault_latched.hw.reg &= (~hw_clr_fault_mask.hw.reg); // Clear latched hw faults indicated by the mask
            hw_clr_fault_cnt = 0U;
        }
        else
        {
            ++hw_clr_fault_cnt;
        }
    }
    faults.flags.hw.reg = hw_fault_latched.hw.reg; // Latched hw fault
}


TEST_GROUP(FaultTest)
{
    CSysSim* sys_sim;

    void setup()
    {
        sys_sim = new CSysSim();

        // Hw simulation:
        hw_fcn.GateDriverEnterHighZ = SimEnterHighZ;
        hw_fcn.GateDriverExitHighZ = SimExitHighZ;
        hw_high_z = false;
        hw_clr_fault_mask.all = { 0 };
        hw_fault_latched.all = { 0 };
        sys_sim->RegisterCallbackISR0(CallbackISR0);
        sys_sim->RegisterCallbackISR1(CallbackISR1);

        // System parameters:
        params.ctrl.mode = Volt_Mode_Open_Loop;
        sys_sim->m_pot.m_init = 0.0f;		// [%]
        sys_sim->m_pot.m_final = 1.5f * params.ctrl.volt.w_thresh.elec / MECH_TO_ELEC(params.sys.cmd.w_max.mech, params.motor.P); // [%]
        sys_sim->m_pot.m_t_start = 0.25f;	// [sec]
        sys_sim->m_pot.m_t_stop = 0.75f;	// [sec]
        sys_sim->m_pot.m_t_slope = 0.25f;	// [sec]

        // Start up and go to volt ctrl state
        while (sm.current == Init) { sys_sim->RunOneTick(); }
        while (sm.current == Brake_Boot) { sys_sim->RunOneTick(); }
        sys_sim->RunFor(params.sys.samp.ts1);
    }

    void teardown()
    {
        delete sys_sim;
    }

    void TestFault(const bool from_hw, const FAULT_FLAGS_t fault, const FAULT_REACTION_t expected_reaction, const UVW_t expected_d_uvw_cmd)
    {
        // Triggering:
        if (from_hw)
        {
            faults.flags.hw.reg = fault.hw.reg;
        }
        else // from sw
        {
            faults.flags_latched.sw.reg = fault.sw.reg; // For simulation; SW updates faults.flags.sw reg.
        }

        // Detection:
        // Within one ISR1 period
        sys_sim->RunFor(params.sys.samp.ts1);
        CHECK_EQUAL(fault.all, faults.flags_latched.all);

        // Reaction:
        CHECK_EQUAL(expected_reaction, faults.reaction);
        CHECK_TRUE(sm.current == Fault);
        CHECK_EQUAL((expected_reaction == High_Z), hw_high_z);
        CHECK_EQUAL(expected_d_uvw_cmd.u, vars.d_uvw_cmd.u);
        CHECK_EQUAL(expected_d_uvw_cmd.v, vars.d_uvw_cmd.v);
        CHECK_EQUAL(expected_d_uvw_cmd.w, vars.d_uvw_cmd.w);

        // Both fault trigger and fault state persist:
        sys_sim->RunUntil(sys_sim->m_pot.m_t_start + sys_sim->m_pot.m_t_slope);
        CHECK_EQUAL(expected_reaction, faults.reaction);
        CHECK_TRUE(sm.current == Fault);
        CHECK_EQUAL((expected_reaction == High_Z), hw_high_z);
        CHECK_EQUAL(expected_d_uvw_cmd.u, vars.d_uvw_cmd.u);
        CHECK_EQUAL(expected_d_uvw_cmd.v, vars.d_uvw_cmd.v);
        CHECK_EQUAL(expected_d_uvw_cmd.w, vars.d_uvw_cmd.w);

        // Fault trigger removed but fault state persists:
        if (from_hw)
        {
            faults.flags.hw.reg = 0U;
        }
        else // from sw
        {
            faults.flags_latched.sw.reg = 0U;
        }
        sys_sim->RunUntil(sys_sim->m_pot.m_t_stop);
        CHECK_EQUAL(expected_reaction, faults.reaction);
        CHECK_TRUE(sm.current == Fault);
        CHECK_EQUAL((expected_reaction == High_Z), hw_high_z);
        CHECK_EQUAL(expected_d_uvw_cmd.u, vars.d_uvw_cmd.u);
        CHECK_EQUAL(expected_d_uvw_cmd.v, vars.d_uvw_cmd.v);
        CHECK_EQUAL(expected_d_uvw_cmd.w, vars.d_uvw_cmd.w);

        // Clearing:
        if (from_hw)
        {
            faults.flags.hw.reg = fault.hw.reg;
            hw_fault_latched.hw.reg = fault.hw.reg;
            // A. Simulating latched HW fault that doesn't clear, max tries reached
            hw_clr_fault_mask.hw.reg = 0;
            sys_sim->RunUntil(sys_sim->m_pot.m_t_stop + sys_sim->m_pot.m_t_slope);
            CHECK_EQUAL(expected_reaction, faults.reaction);
            CHECK_TRUE(sm.current == Fault);
            CHECK_EQUAL((expected_reaction == High_Z), hw_high_z);
            CHECK_EQUAL(false, sm.vars.fault.clr_success);
            CHECK_EQUAL(params.sys.faults.max_clr_tries, sm.vars.fault.clr_try_cnt);
            // B. Now latched HW fault is cleared but max tries reached
            hw_clr_fault_mask.hw.reg = fault.hw.reg;
            sys_sim->RunFor(params.sys.analog.offset_null_time * 2.0f); // To make sure we pass Init state
            CHECK_EQUAL(expected_reaction, faults.reaction);
            CHECK_TRUE(sm.current == Fault);
            CHECK_EQUAL((expected_reaction == High_Z), hw_high_z);
            CHECK_EQUAL(false, sm.vars.fault.clr_success);
            CHECK_EQUAL(params.sys.faults.max_clr_tries, sm.vars.fault.clr_try_cnt);
            // C. Try again with resetting try counter
            sm.vars.fault.clr_try_cnt = 0U;
            sys_sim->RunFor(params.sys.analog.offset_null_time * 2.0f); // To make sure we pass Init state
            CHECK_EQUAL(No_Reaction, faults.reaction);
            CHECK_EQUAL(false, hw_high_z);
            CHECK_EQUAL(true, sm.vars.fault.clr_success);
            CHECK_EQUAL(1U, sm.vars.fault.clr_try_cnt);
        }
        else // from sw
        {
            faults.flags.sw.reg = fault.sw.reg;
            sys_sim->RunUntil(sys_sim->m_pot.m_t_stop + sys_sim->m_pot.m_t_slope);
            CHECK_EQUAL(No_Reaction, faults.reaction);
            CHECK_EQUAL(false, hw_high_z);
            CHECK_EQUAL(true, sm.vars.fault.clr_success);
            CHECK_EQUAL(1U, sm.vars.fault.clr_try_cnt);
        }
    }
};

TEST(FaultTest, HardwareOverCurrentPhaseU)
{
    // Fault to be tested:
    FAULT_FLAGS_t test_fault = { 0 };
    test_fault.hw.cs_ocp = 0b001; // Phase U
    const FAULT_REACTION_t Expected_Reaction = High_Z;
    const UVW_t Expected_Duvw_Cmd = { 0.0f, 0.0f, 0.0f };

    TestFault(true, test_fault, Expected_Reaction, Expected_Duvw_Cmd);
}

TEST(FaultTest, HardwareOverCurrentPhaseV)
{
    // Fault to be tested:
    FAULT_FLAGS_t test_fault = { 0 };
    test_fault.hw.cs_ocp = 0b010; // Phase V
    const FAULT_REACTION_t Expected_Reaction = High_Z;
    const UVW_t Expected_Duvw_Cmd = { 0.0f, 0.0f, 0.0f };

    TestFault(true, test_fault, Expected_Reaction, Expected_Duvw_Cmd);
}

TEST(FaultTest, HardwareOverCurrentPhaseW)
{
    // Fault to be tested:
    FAULT_FLAGS_t test_fault = { 0 };
    test_fault.hw.cs_ocp = 0b100; // Phase W
    const FAULT_REACTION_t Expected_Reaction = High_Z;
    const UVW_t Expected_Duvw_Cmd = { 0.0f, 0.0f, 0.0f };

    TestFault(true, test_fault, Expected_Reaction, Expected_Duvw_Cmd);
}

TEST(FaultTest, HardwareChargePump)
{
    // Fault to be tested:
    FAULT_FLAGS_t test_fault = { 0 };
    test_fault.hw.cp = 0b1;
    const FAULT_REACTION_t Expected_Reaction = High_Z;
    const UVW_t Expected_Duvw_Cmd = { 0.0f, 0.0f, 0.0f };

    TestFault(true, test_fault, Expected_Reaction, Expected_Duvw_Cmd);
}

TEST(FaultTest, HardwareDvddOverCurrent)
{
    // Fault to be tested:
    FAULT_FLAGS_t test_fault = { 0 };
    test_fault.hw.dvdd_ocp = 0b1;
    const FAULT_REACTION_t Expected_Reaction = High_Z;
    const UVW_t Expected_Duvw_Cmd = { 0.0f, 0.0f, 0.0f };

    TestFault(true, test_fault, Expected_Reaction, Expected_Duvw_Cmd);
}

TEST(FaultTest, HardwareDvddUnderVoltage)
{
    // Fault to be tested:
    FAULT_FLAGS_t test_fault = { 0 };
    test_fault.hw.dvdd_uv = 0b1;
    const FAULT_REACTION_t Expected_Reaction = High_Z;
    const UVW_t Expected_Duvw_Cmd = { 0.0f, 0.0f, 0.0f };

    TestFault(true, test_fault, Expected_Reaction, Expected_Duvw_Cmd);
}

TEST(FaultTest, HardwareDvddOverVoltage)
{
    // Fault to be tested:
    FAULT_FLAGS_t test_fault = { 0 };
    test_fault.hw.dvdd_ov = 0b1;
    const FAULT_REACTION_t Expected_Reaction = High_Z;
    const UVW_t Expected_Duvw_Cmd = { 0.0f, 0.0f, 0.0f };

    TestFault(true, test_fault, Expected_Reaction, Expected_Duvw_Cmd);
}

TEST(FaultTest, HardwareBuckOverCurrent)
{
    // Fault to be tested:
    FAULT_FLAGS_t test_fault = { 0 };
    test_fault.hw.bk_ocp = 0b1;
    const FAULT_REACTION_t Expected_Reaction = High_Z;
    const UVW_t Expected_Duvw_Cmd = { 0.0f, 0.0f, 0.0f };

    TestFault(true, test_fault, Expected_Reaction, Expected_Duvw_Cmd);
}

TEST(FaultTest, HardwareOverTemperatureShutDown)
{
    // Fault to be tested:
    FAULT_FLAGS_t test_fault = { 0 };
    test_fault.hw.ots = 0b1;
    const FAULT_REACTION_t Expected_Reaction = High_Z;
    const UVW_t Expected_Duvw_Cmd = { 0.0f, 0.0f, 0.0f };

    TestFault(true, test_fault, Expected_Reaction, Expected_Duvw_Cmd);
}

TEST(FaultTest, HardwareLockedRotor)
{
    // Fault to be tested:
    FAULT_FLAGS_t test_fault = { 0 };
    test_fault.hw.rlock = 0b1;
    const FAULT_REACTION_t Expected_Reaction = High_Z;
    const UVW_t Expected_Duvw_Cmd = { 0.0f, 0.0f, 0.0f };

    TestFault(true, test_fault, Expected_Reaction, Expected_Duvw_Cmd);
}

TEST(FaultTest, HardwareWatchdog)
{
    // Fault to be tested:
    FAULT_FLAGS_t test_fault = { 0 };
    test_fault.hw.wd = 0b1;
    const FAULT_REACTION_t Expected_Reaction = High_Z;
    const UVW_t Expected_Duvw_Cmd = { 0.0f, 0.0f, 0.0f };

    TestFault(true, test_fault, Expected_Reaction, Expected_Duvw_Cmd);
}

TEST(FaultTest, HardwareOneTimeProgrammableMemory)
{
    // Fault to be tested:
    FAULT_FLAGS_t test_fault = { 0 };
    test_fault.hw.otp = 0b1;
    const FAULT_REACTION_t Expected_Reaction = High_Z;
    const UVW_t Expected_Duvw_Cmd = { 0.0f, 0.0f, 0.0f };

    TestFault(true, test_fault, Expected_Reaction, Expected_Duvw_Cmd);
}

TEST(FaultTest, SoftwareOverCurrent)
{
    // Fault to be tested:
    FAULT_FLAGS_t test_fault = { 0 };
    test_fault.sw.oc = 0b1;
    const FAULT_REACTION_t Expected_Reaction = High_Z;
    const UVW_t Expected_Duvw_Cmd = { 0.0f, 0.0f, 0.0f };

    TestFault(false, test_fault, Expected_Reaction, Expected_Duvw_Cmd);
}

TEST(FaultTest, SoftwarePowerStageOverTemperature)
{
    // Fault to be tested:
    FAULT_FLAGS_t test_fault = { 0 };
    test_fault.sw.ot_ps = 0b1;
    const FAULT_REACTION_t Expected_Reaction = High_Z;
    const UVW_t Expected_Duvw_Cmd = { 0.0f, 0.0f, 0.0f };

    TestFault(false, test_fault, Expected_Reaction, Expected_Duvw_Cmd);
}

TEST(FaultTest, SoftwareOverVoltageDC)
{
    // Fault to be tested:
    FAULT_FLAGS_t test_fault = { 0 };
    test_fault.sw.ov_vdc = 0b1;
    const FAULT_REACTION_t Expected_Reaction = Short_Motor;
    const UVW_t Expected_Duvw_Cmd = { 0.5f, 0.5f, 0.5f };

    TestFault(false, test_fault, Expected_Reaction, Expected_Duvw_Cmd);
}

TEST(FaultTest, SoftwareUnderVoltageDC)
{
    // Fault to be tested:
    FAULT_FLAGS_t test_fault = { 0 };
    test_fault.sw.uv_vdc = 0b1;
    const FAULT_REACTION_t Expected_Reaction = Short_Motor;
    const UVW_t Expected_Duvw_Cmd = { 0.5f, 0.5f, 0.5f };

    TestFault(false, test_fault, Expected_Reaction, Expected_Duvw_Cmd);
}

TEST(FaultTest, SoftwareOverSpeed)
{
    // Fault to be tested:
    FAULT_FLAGS_t test_fault = { 0 };
    test_fault.sw.os = 0b1;
    const FAULT_REACTION_t Expected_Reaction = Short_Motor;
    const UVW_t Expected_Duvw_Cmd = { 0.5f, 0.5f, 0.5f };

    TestFault(false, test_fault, Expected_Reaction, Expected_Duvw_Cmd);
}