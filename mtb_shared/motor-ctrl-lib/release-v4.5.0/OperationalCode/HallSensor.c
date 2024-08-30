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

HALL_SENS_t hall = { 0 };

float Angle_Capture_Table[HALL_SIGNAL_PERMUTATIONS] = \
// Hall Signal (WVU):	    0b101   ->  0b001   ->  0b011   ->  0b010   ->  0b110   ->  0b100
// Indices:                 5U      ->  1U      ->  3U      ->  2U      ->  6U      ->  4U
// Resulting Rotor Angle:	-30deg  ->  +30deg  ->  +90deg  ->  +150deg ->  -150deg ->  -90deg
{ NAN, DEG_TO_RAD(+30.0f), DEG_TO_RAD(+150.0f), DEG_TO_RAD(+90.0f), DEG_TO_RAD(-90.0f), DEG_TO_RAD(-30.0f), DEG_TO_RAD(-150.0f), NAN };

void HALL_SENSOR_Init()
{
    hall.w_conv_coeff = PI_OVER_THREE * hall.time_cap_freq;    // duration of one hall signal = period / 6
    hall.w_ff_coeff = params.sys.fb.hall.w0_w * params.sys.samp.ts0;
    hall.w_fb_coeff = THREE_OVER_PI / params.sys.fb.hall.tau_ratio;
    StopWatchInit(&hall.zero_spd_timer, PI / params.sys.fb.hall.w_thresh.elec, params.sys.samp.ts0);
}

void HALL_SENSOR_Reset()
{
    hall.signal.uvw = 0b010;
    hall.signal_prev.uvw = 0b010;
    hall.period_cap = UINT32_MAX;
    hall.w_sign = +1.0f;
    hall.w_ff.elec = 0.0f;
    hall.w_ff_filt.elec = 0.0f;
    hall.th_r_cap_prev.elec = DEG_TO_RAD(0.0f);
    hall.th_r_est.elec = DEG_TO_RAD(0.0f);
    StopWatchReset(&hall.zero_spd_timer);
}

RAMFUNC_BEGIN
void HALL_SENSOR_RunISR0()
{
    // Fault detection
    if ((hall.signal.uvw == 0b000) || (hall.signal.uvw == 0b111))
    {
        faults.flags.sw.hall = true;
        return;
    }

    // Speed feed forward
    if (StopWatchIsDone(&hall.zero_spd_timer))
    {
        hall.w_ff.elec = 0.0f;
    }
    else
    {
        StopWatchRun(&hall.zero_spd_timer);
        if (hall.period_cap != 0U)
        {
            hall.w_ff.elec = hall.w_sign * hall.w_conv_coeff / (float)(hall.period_cap);
        }
    }
    hall.w_ff_filt.elec += (hall.w_ff.elec - hall.w_ff_filt.elec) * hall.w_ff_coeff;

    // Angle estimation
    hall.th_r_cap.elec = Angle_Capture_Table[hall.signal.uvw] + params.sys.fb.hall.th_r_offset.elec;
    hall.trans = (hall.signal.uvw != hall.signal_prev.uvw);
    if (hall.trans)
    {
        hall.w_sign = SIGN(Wrap2Pi(hall.th_r_cap.elec - hall.th_r_cap_prev.elec));
        hall.th_r_cap_prev = hall.th_r_cap;
        StopWatchReset(&hall.zero_spd_timer);
    }
    hall.signal_prev.uvw = hall.signal.uvw;
    hall.w_tot.elec = hall.w_ff_filt.elec + SAT(params.sys.fb.hall.w0_th.min, params.sys.fb.hall.w0_th.max, hall.w_fb_coeff * hall.w_sign * hall.w_ff.elec)
        * Wrap2Pi(hall.th_r_cap.elec - hall.th_r_est.elec);
    hall.th_r_est.elec = Wrap2Pi(hall.th_r_est.elec + params.sys.samp.ts0 * hall.w_tot.elec);

    vars.w_hall.elec = hall.w_ff.elec;
    vars.th_r_hall.elec = hall.th_r_est.elec;

#if defined(PC_TEST)
    vars.test[39] = (float)(hall.signal.uvw);
    vars.test[40] = (float)(hall.signal_prev.uvw);
    vars.test[41] = hall.th_r_cap.elec;
    vars.test[42] = hall.w_tot.elec;
#endif
}
RAMFUNC_END
