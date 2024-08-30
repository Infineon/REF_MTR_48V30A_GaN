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


#if defined(CTRL_METHOD_TBC)

#include "Controller.h"

#if defined(REGRESSION_TEST)
static bool u_High_Z_Status;
static bool v_High_Z_Status;
static bool w_High_Z_Status;
#endif


// Hall Signal (WVU):	    0b110   ->  0b100   ->  0b101   ->  0b001   ->  0b011   ->  0b010  
// Rotor Angle:	            -150deg ->  -90deg  ->  -30deg  ->  +30deg  ->  +90deg  ->  +150deg
// Sector (positive):       0U      ->  1U      ->  2U      ->  3U      ->  4U      ->  5U 
// Sector (negative):       1U      ->  2U      ->  3U      ->  4U      ->  5U      ->  0U 
static inline void FindSector()
{
    ctrl.trap_comm.th_ramp.elec = (float)(params.ctrl.tbc.trap.ramp_cnt) * vars.w_final_filt.elec * params.sys.samp.ts0;
    ctrl.trap_comm.sect = (uint16_t)((Wrap2Pi(vars.th_r_hall.elec + 0.5f * (ctrl.trap_comm.th_ramp.elec - (float)(ctrl.trap_comm.dir - 1) * PI_OVER_THREE)) + PI) * THREE_OVER_PI);
    ctrl.trap_comm.sect %= 6U;
}

void TRAP_COMM_Init()
{
    PI_UpdateParams(&ctrl.trap_comm.pi_one, params.ctrl.tbc.trap.ramp_kp, params.ctrl.tbc.trap.ramp_ki * params.sys.samp.ts0, -params.ctrl.curr.v_max, params.ctrl.curr.v_max);
    PI_UpdateParams(&ctrl.trap_comm.pi_two, params.ctrl.tbc.trap.ramp_kp, params.ctrl.tbc.trap.ramp_ki * params.sys.samp.ts0, -params.ctrl.curr.v_max, params.ctrl.curr.v_max);
    PI_UpdateParams(&ctrl.trap_comm.pi_main, params.ctrl.curr.kp, params.ctrl.curr.ki * params.sys.samp.ts0, -params.ctrl.curr.v_max, params.ctrl.curr.v_max);

    FindSector();
    ctrl.trap_comm.sect_prev = (ctrl.trap_comm.sect - 1U) % 6U;
    ctrl.trap_comm.dir = 1;
}

void TRAP_COMM_Reset()
{
    PI_Reset(&ctrl.trap_comm.pi_one);
    PI_Reset(&ctrl.trap_comm.pi_two);
    PI_Reset(&ctrl.trap_comm.pi_main);
}

static inline void RefGenISR0()
{
    FindSector();
    ctrl.trap_comm.new_sect = (ctrl.trap_comm.sect != ctrl.trap_comm.sect_prev);
    if (ctrl.trap_comm.new_sect)
    {
        ctrl.trap_comm.ramp_cnt = 1U;
        ctrl.trap_comm.dir = (int16_t)(ctrl.trap_comm.sect - ctrl.trap_comm.sect_prev);
        if (ctrl.trap_comm.dir == -5)
        {
            ctrl.trap_comm.dir = 1;
        }
        else if (ctrl.trap_comm.dir == 5)
        {
            ctrl.trap_comm.dir = -1;
        }
        ctrl.trap_comm.sect_prev = ctrl.trap_comm.sect;
    }

    ctrl.trap_comm.i_xyz_ref.z = -1.0f;
    if (ctrl.trap_comm.ramp_cnt > params.ctrl.tbc.trap.ramp_cnt)
    {
        ctrl.trap_comm.i_xyz_ref.x = 1.0f;
        ctrl.trap_comm.i_xyz_ref.y = 0.0f;
        if (!ctrl.trap_comm.enter_high_z_flag || (ctrl.trap_comm.new_sect && params.ctrl.tbc.trap.ramp_cnt == 0U))
        {
            ctrl.trap_comm.ramp_done = true;
        }
        else
        {
            ctrl.trap_comm.ramp_done = false;
        }
        ctrl.trap_comm.enter_high_z_flag = true;
        ctrl.trap_comm.exit_high_z_flag = false;
    }
    else
    {
        ctrl.trap_comm.i_xyz_ref.x = params.ctrl.tbc.trap.ramp_cnt_inv * (float)(ctrl.trap_comm.ramp_cnt);
        ctrl.trap_comm.i_xyz_ref.y = 1.0f - ctrl.trap_comm.i_xyz_ref.x;
        ctrl.trap_comm.enter_high_z_flag = false;
        ctrl.trap_comm.exit_high_z_flag = true;
    }

    if (ctrl.trap_comm.dir == -1)
    {
        float current_swap = ctrl.trap_comm.i_xyz_ref.x;
        ctrl.trap_comm.i_xyz_ref.x = ctrl.trap_comm.i_xyz_ref.y;
        ctrl.trap_comm.i_xyz_ref.y = current_swap;
    }

    switch (ctrl.trap_comm.sect)
    {
    case 0U:
        ctrl.trap_comm.i_xyz_fb.z = vars.i_uvw_fb.u;
        ctrl.trap_comm.i_xyz_fb.y = vars.i_uvw_fb.v;
        ctrl.trap_comm.i_xyz_fb.x = vars.i_uvw_fb.w;
        break;
    case 1U:
        ctrl.trap_comm.i_xyz_fb.y = -vars.i_uvw_fb.u;
        ctrl.trap_comm.i_xyz_fb.x = -vars.i_uvw_fb.v;
        ctrl.trap_comm.i_xyz_fb.z = -vars.i_uvw_fb.w;
        break;
    case 2U:
        ctrl.trap_comm.i_xyz_fb.x = vars.i_uvw_fb.u;
        ctrl.trap_comm.i_xyz_fb.z = vars.i_uvw_fb.v;
        ctrl.trap_comm.i_xyz_fb.y = vars.i_uvw_fb.w;
        break;
    case 3U:
        ctrl.trap_comm.i_xyz_fb.z = -vars.i_uvw_fb.u;
        ctrl.trap_comm.i_xyz_fb.y = -vars.i_uvw_fb.v;
        ctrl.trap_comm.i_xyz_fb.x = -vars.i_uvw_fb.w;
        break;
    case 4U:
        ctrl.trap_comm.i_xyz_fb.y = vars.i_uvw_fb.u;
        ctrl.trap_comm.i_xyz_fb.x = vars.i_uvw_fb.v;
        ctrl.trap_comm.i_xyz_fb.z = vars.i_uvw_fb.w;
        break;
    case 5U:
        ctrl.trap_comm.i_xyz_fb.x = -vars.i_uvw_fb.u;
        ctrl.trap_comm.i_xyz_fb.z = -vars.i_uvw_fb.v;
        ctrl.trap_comm.i_xyz_fb.y = -vars.i_uvw_fb.w;
        break;
    }

}

void TRAP_COMM_RunISR0()
{
    RefGenISR0();
    ctrl.trap_comm.i_xyz_cmd.x = vars.i_cmd_int * ctrl.trap_comm.i_xyz_ref.x * ONE_OVER_SQRT_TWO;
    ctrl.trap_comm.i_xyz_cmd.y = vars.i_cmd_int * ctrl.trap_comm.i_xyz_ref.y * ONE_OVER_SQRT_TWO;
    ctrl.trap_comm.i_xyz_cmd.z = vars.i_cmd_int * ctrl.trap_comm.i_xyz_ref.z * ONE_OVER_SQRT_TWO;


    if (ctrl.trap_comm.ramp_cnt <= params.ctrl.tbc.trap.ramp_cnt)
    {
        ctrl.trap_comm.pi_one.ff = (float)(ctrl.trap_comm.dir) * vars.i_cmd_int * params.ctrl.tbc.trap.ramp_cnt_inv * params.ctrl.tbc.trap.ramp_ff_coef;
        ctrl.trap_comm.pi_two.ff = - ctrl.trap_comm.pi_one.ff;
        PI_Run(&ctrl.trap_comm.pi_one, ctrl.trap_comm.i_xyz_cmd.x, ctrl.trap_comm.i_xyz_fb.x, ctrl.trap_comm.pi_one.ff);
        PI_Run(&ctrl.trap_comm.pi_two, ctrl.trap_comm.i_xyz_cmd.y, ctrl.trap_comm.i_xyz_fb.y, ctrl.trap_comm.pi_two.ff);
        ctrl.trap_comm.ramp_cnt++;
    }
    else
    {
        PI_Reset(&ctrl.trap_comm.pi_one);
        PI_Reset(&ctrl.trap_comm.pi_two);
    }
    ctrl.trap_comm.pi_main.ff = params.motor.lam * vars.w_final.elec * params.ctrl.tbc.trap.main_ff_coef * ctrl.trap_comm.i_xyz_ref.z;
    PI_Run(&ctrl.trap_comm.pi_main, ctrl.trap_comm.i_xyz_cmd.z, ctrl.trap_comm.i_xyz_fb.z, ctrl.trap_comm.pi_main.ff);
    vars.i_s_fb = -SQRT_TWO * ctrl.trap_comm.i_xyz_fb.z;
    ctrl.trap_comm.v_xyz_cmd.x = ctrl.trap_comm.pi_one.output - ctrl.trap_comm.pi_main.output;
    ctrl.trap_comm.v_xyz_cmd.y = ctrl.trap_comm.pi_two.output - ctrl.trap_comm.pi_main.output;
    ctrl.trap_comm.v_xyz_cmd.z = ctrl.trap_comm.pi_main.output;


    ctrl.block_comm.exit_high_z_flag.uvw = 0b000;
    ctrl.block_comm.enter_high_z_flag.uvw = 0b000;
    if (ctrl.trap_comm.new_sect)
    {
        ctrl.block_comm.exit_high_z_flag.uvw = 0b111;
        ctrl.block_comm.enter_high_z_flag.uvw = 0b000;
        ctrl.trap_comm.new_sect = false;
    }

    switch (ctrl.trap_comm.sect)
    {
    case 0U:
        ctrl.trap_comm.v_uvw_cmd.u = ctrl.trap_comm.v_xyz_cmd.z;
        ctrl.trap_comm.v_uvw_cmd.v = ctrl.trap_comm.v_xyz_cmd.y;
        ctrl.trap_comm.v_uvw_cmd.w = ctrl.trap_comm.v_xyz_cmd.x;
        if (ctrl.trap_comm.ramp_done)
        {
            if (ctrl.trap_comm.dir == 1)
            {
                ctrl.block_comm.exit_high_z_flag.v = ctrl.trap_comm.exit_high_z_flag;
                ctrl.block_comm.enter_high_z_flag.v = ctrl.trap_comm.enter_high_z_flag;
            }
            else
            {
                ctrl.block_comm.exit_high_z_flag.w = ctrl.trap_comm.exit_high_z_flag;
                ctrl.block_comm.enter_high_z_flag.w = ctrl.trap_comm.enter_high_z_flag;
            }
        }
        break;
    case 1U:
        ctrl.trap_comm.v_uvw_cmd.u = -ctrl.trap_comm.v_xyz_cmd.y;
        ctrl.trap_comm.v_uvw_cmd.v = -ctrl.trap_comm.v_xyz_cmd.x;
        ctrl.trap_comm.v_uvw_cmd.w = -ctrl.trap_comm.v_xyz_cmd.z;
        if (ctrl.trap_comm.ramp_done)
        {
            if (ctrl.trap_comm.dir == 1)
            {
                ctrl.block_comm.exit_high_z_flag.u = ctrl.trap_comm.exit_high_z_flag;
                ctrl.block_comm.enter_high_z_flag.u = ctrl.trap_comm.enter_high_z_flag;
            }
            else
            {
                ctrl.block_comm.exit_high_z_flag.v = ctrl.trap_comm.exit_high_z_flag;
                ctrl.block_comm.enter_high_z_flag.v = ctrl.trap_comm.enter_high_z_flag;
            }
        }
        break;
    case 2U:
        ctrl.trap_comm.v_uvw_cmd.u = ctrl.trap_comm.v_xyz_cmd.x;
        ctrl.trap_comm.v_uvw_cmd.v = ctrl.trap_comm.v_xyz_cmd.z;
        ctrl.trap_comm.v_uvw_cmd.w = ctrl.trap_comm.v_xyz_cmd.y;
        if (ctrl.trap_comm.ramp_done)
        {
            if (ctrl.trap_comm.dir == 1)
            {
                ctrl.block_comm.exit_high_z_flag.w = ctrl.trap_comm.exit_high_z_flag;
                ctrl.block_comm.enter_high_z_flag.w = ctrl.trap_comm.enter_high_z_flag;
            }
            else
            {
                ctrl.block_comm.exit_high_z_flag.u = ctrl.trap_comm.exit_high_z_flag;
                ctrl.block_comm.enter_high_z_flag.u = ctrl.trap_comm.enter_high_z_flag;
            }
        }

        break;
    case 3U:
        ctrl.trap_comm.v_uvw_cmd.u = -ctrl.trap_comm.v_xyz_cmd.z;
        ctrl.trap_comm.v_uvw_cmd.v = -ctrl.trap_comm.v_xyz_cmd.y;
        ctrl.trap_comm.v_uvw_cmd.w = -ctrl.trap_comm.v_xyz_cmd.x;
        if (ctrl.trap_comm.ramp_done)
        {
            if (ctrl.trap_comm.dir == 1)
            {
                ctrl.block_comm.exit_high_z_flag.v = ctrl.trap_comm.exit_high_z_flag;
                ctrl.block_comm.enter_high_z_flag.v = ctrl.trap_comm.enter_high_z_flag;
            }
            else
            {
                ctrl.block_comm.exit_high_z_flag.w = ctrl.trap_comm.exit_high_z_flag;
                ctrl.block_comm.enter_high_z_flag.w = ctrl.trap_comm.enter_high_z_flag;
            }
        }
        break;
    case 4U:
        ctrl.trap_comm.v_uvw_cmd.u = ctrl.trap_comm.v_xyz_cmd.y;
        ctrl.trap_comm.v_uvw_cmd.v = ctrl.trap_comm.v_xyz_cmd.x;
        ctrl.trap_comm.v_uvw_cmd.w = ctrl.trap_comm.v_xyz_cmd.z;
        if (ctrl.trap_comm.ramp_done)
        {
            if (ctrl.trap_comm.dir == 1)
            {
                ctrl.block_comm.exit_high_z_flag.u = ctrl.trap_comm.exit_high_z_flag;
                ctrl.block_comm.enter_high_z_flag.u = ctrl.trap_comm.enter_high_z_flag;
            }
            else
            {
                ctrl.block_comm.exit_high_z_flag.v = ctrl.trap_comm.exit_high_z_flag;
                ctrl.block_comm.enter_high_z_flag.v = ctrl.trap_comm.enter_high_z_flag;
            }
        }
        break;
    case 5U:
        ctrl.trap_comm.v_uvw_cmd.u = -ctrl.trap_comm.v_xyz_cmd.x;
        ctrl.trap_comm.v_uvw_cmd.v = -ctrl.trap_comm.v_xyz_cmd.z;
        ctrl.trap_comm.v_uvw_cmd.w = -ctrl.trap_comm.v_xyz_cmd.y;
        if (ctrl.trap_comm.ramp_done)
        {
            if (ctrl.trap_comm.dir == 1)
            {
                ctrl.block_comm.exit_high_z_flag.w = ctrl.trap_comm.exit_high_z_flag;
                ctrl.block_comm.enter_high_z_flag.w = ctrl.trap_comm.enter_high_z_flag;
            }
            else
            {
                ctrl.block_comm.exit_high_z_flag.u = ctrl.trap_comm.exit_high_z_flag;
                ctrl.block_comm.enter_high_z_flag.u = ctrl.trap_comm.enter_high_z_flag;
            }
        }
        break;
    }

    float trap_v_cmd_min = MIN(MIN(ctrl.trap_comm.v_uvw_cmd.u, ctrl.trap_comm.v_uvw_cmd.v), ctrl.trap_comm.v_uvw_cmd.w);
    ctrl.trap_comm.v_uvw_cmd.u -= trap_v_cmd_min;
    ctrl.trap_comm.v_uvw_cmd.v -= trap_v_cmd_min;
    ctrl.trap_comm.v_uvw_cmd.w -= trap_v_cmd_min;

    float v_dc_inv = 1.0f / vars.v_dc;
    vars.d_uvw_cmd.u = SAT(0.0f, 1.0f, v_dc_inv * ctrl.trap_comm.v_uvw_cmd.u);
    vars.d_uvw_cmd.v = SAT(0.0f, 1.0f, v_dc_inv * ctrl.trap_comm.v_uvw_cmd.v);
    vars.d_uvw_cmd.w = SAT(0.0f, 1.0f, v_dc_inv * ctrl.trap_comm.v_uvw_cmd.w);


#if defined(REGRESSION_TEST)
    if (ctrl.trap_comm.new_sect)
    {
        u_High_Z_Status = false;
        v_High_Z_Status = false;
        w_High_Z_Status = false;
    }
    else if (ctrl.trap_comm.ramp_done)
    {
        u_High_Z_Status = ctrl.block_comm.enter_high_z_flag.u;
        v_High_Z_Status = ctrl.block_comm.enter_high_z_flag.v;
        w_High_Z_Status = ctrl.block_comm.enter_high_z_flag.w;
    }
    vars.d_uvw_cmd.u = (u_High_Z_Status) ? NAN : vars.d_uvw_cmd.u;
    vars.d_uvw_cmd.v = (v_High_Z_Status) ? NAN : vars.d_uvw_cmd.v;
    vars.d_uvw_cmd.w = (w_High_Z_Status) ? NAN : vars.d_uvw_cmd.w;
#endif

}

#endif