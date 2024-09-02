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


#include "Controller.h"

#if defined(CTRL_METHOD_RFO)

void CURRENT_CTRL_Init(const float bw_red_coeff)
{
    PI_UpdateParams(&ctrl.curr.pi_q, params.ctrl.curr.kp.q * bw_red_coeff, params.ctrl.curr.ki.q * bw_red_coeff * params.sys.samp.ts0, -params.ctrl.curr.v_max.q, params.ctrl.curr.v_max.q);
    PI_UpdateParams(&ctrl.curr.pi_d, params.ctrl.curr.kp.d * bw_red_coeff, params.ctrl.curr.ki.d * bw_red_coeff * params.sys.samp.ts0, -params.ctrl.curr.v_max.d, params.ctrl.curr.v_max.d);
};

void CURRENT_CTRL_Reset()
{
    PI_Reset(&ctrl.curr.pi_q);
    PI_Reset(&ctrl.curr.pi_d);
}

RAMFUNC_BEGIN
void CURRENT_CTRL_RunISR0()
{
    if (!sm.vars.high_freq.used) // handled by high freqeuncy injection module due to required frequency spectrum separation
    {
        // Current rotation
        ParkInit(vars.th_r_final.elec, &vars.park_r);
        ParkTransform(&vars.i_ab_fb, &vars.park_r, &vars.i_qd_r_fb);
    }

    // Flux calculations
    vars.la_qd_r_est.q = params.motor.lq * vars.i_qd_r_fb.q;
    vars.la_qd_r_est.d = params.motor.ld * vars.i_qd_r_fb.d + params.motor.lam;

    // Feed forwards
    ctrl.curr.ff.q = vars.la_qd_r_est.d * vars.w_final.elec * params.ctrl.curr.ff_coef;
    ctrl.curr.ff.d = -vars.la_qd_r_est.q * vars.w_final.elec * params.ctrl.curr.ff_coef;

    // PIs
    PI_Run(&ctrl.curr.pi_q, vars.i_qd_r_cmd.q, vars.i_qd_r_fb.q, ctrl.curr.ff.q);
    PI_Run(&ctrl.curr.pi_d, vars.i_qd_r_cmd.d, vars.i_qd_r_fb.d, ctrl.curr.ff.d);
    vars.v_qd_r_cmd.q = ctrl.curr.pi_q.output;
    vars.v_qd_r_cmd.d = ctrl.curr.pi_d.output;

    // Voltage derotation
    ParkTransformInv(&vars.v_qd_r_cmd, &vars.park_r, &vars.v_ab_cmd);
    vars.v_ab_cmd_tot = vars.v_ab_cmd;
    if (sm.vars.high_freq.used) // adding high frequency excitation components
    {
        vars.v_ab_cmd_tot.alpha += ctrl.high_freq_inj.v_ab_cmd.alpha;
        vars.v_ab_cmd_tot.beta += ctrl.high_freq_inj.v_ab_cmd.beta;
    }
}
RAMFUNC_END

#elif defined(CTRL_METHOD_TBC)

void CURRENT_CTRL_Init()
{
    PI_UpdateParams(&ctrl.curr.pi, params.ctrl.curr.kp, params.ctrl.curr.ki * params.sys.samp.ts0, -params.ctrl.curr.v_max, params.ctrl.curr.v_max);
};

void CURRENT_CTRL_Reset()
{
    PI_Reset(&ctrl.curr.pi);
}

RAMFUNC_BEGIN
void CURRENT_CTRL_RunISR0()
{
    // Feed forward
#if defined(PC_TEST)
    ctrl.curr.ff = params.motor.lam * vars.w_cmd_int.elec * params.ctrl.curr.ff_coef;
#else
    ctrl.curr.ff = 0.0f; // TBD: unstability observed when using feedforwad
#endif

    if (params.ctrl.curr.bypass)
    {
        vars.v_s_cmd.rad = params.ctrl.curr.k_bypass * vars.i_cmd_int + ctrl.curr.ff;
    }
    else
    {
        // PI
        PI_Run(&ctrl.curr.pi, vars.i_cmd_int, vars.i_s_fb, ctrl.curr.ff);
        vars.v_s_cmd.rad = ctrl.curr.pi.output;
    }
}
RAMFUNC_END

#endif
