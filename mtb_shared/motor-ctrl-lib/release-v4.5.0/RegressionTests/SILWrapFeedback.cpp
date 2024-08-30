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


#include "SILWrapFeedback.h"
#include "SILAuxFcns.h"
#include "AuxMethods.h"
#include "CFeedback.h"
#include "CHallSensor.h"

// Simulation constants .............................................................


// ...................................................................................

CFeedback::EType feedback_type;
static CFeedback* feedback = nullptr;
static CHallSensor hall_sensor;
static float th_r_mech;

void SIL_StartFeedback(PARAMS_t* params_final)
{
    params = *params_final;

    switch (params.sys.fb.mode)
    {   // TBD: to be expanded after supporting encoder
    case Sensorless:
#if defined(CTRL_METHOD_RFO) || defined(CTRL_METHOD_TBC)
    case Hall:
#endif
    default:
        feedback_type = CFeedback::EType::Hall_Sensor;
        feedback = &hall_sensor;
        break;
    }

    feedback->SetInputPointers(params_final, &th_r_mech);
}

void SIL_TerminateFeedback()
{
    // Nothing to do here
}

void SIL_RunFeedback(SIL_INPUT_FEEDBACK_t* input, SIL_OUTPUT_FEEDBACK_t* output)
{
    // Read inputs .............................................................................................
#pragma warning(disable:4244)	// implicit conversion of double inputs from MATLAB to float
    th_r_mech = input->th_r_mech;
#pragma warning(default:4244)	// implicit conversion of double inputs from MATLAB to float

    // Run model ...............................................................................................
    feedback->RunSim();

    // Write outputs ...........................................................................................
    output->signal[0] = (feedback_type == CFeedback::EType::Hall_Sensor) ? hall_sensor.m_hall_signal.u : false;
    output->signal[1] = (feedback_type == CFeedback::EType::Hall_Sensor) ? hall_sensor.m_hall_signal.v : false;
    output->signal[2] = (feedback_type == CFeedback::EType::Hall_Sensor) ? hall_sensor.m_hall_signal.w : false;
    output->time_cap = (feedback_type == CFeedback::EType::Hall_Sensor) ? hall_sensor.m_period_cap : 0U;
}