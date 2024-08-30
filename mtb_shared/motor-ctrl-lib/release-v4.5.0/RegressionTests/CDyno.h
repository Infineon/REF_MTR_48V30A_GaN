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
#include "CLoad.h"
#include "CPotentiometer.h"

#ifdef __cplusplus
extern "C"
{
#endif
#include "../OperationalCode/Controller.h"
#ifdef __cplusplus
}
#endif

// Simulating a dyno with torque, speed, and position control capability:
class CDyno : public CLoad
{
public:
    enum class ECtrlMode
    {
        Torque_Ctrl,
        Speed_Ctrl,
        Position_Ctrl
    };

    CDyno();
    CDyno(PARAMS_t* params, float* T);
    ~CDyno();

    void SetInputPointers(PARAMS_t* params, float* T);
    void SetCtrlMode(ECtrlMode ctrl_mode);
    ECtrlMode GetCtrlMode();

    void Reset(MECH_t w_0, MECH_t th_0);
    void RunSim();

    CPotentiometer m_pot;		// [%], generating the command value

    // Command values corresponding to potentimeter at 100 %:
    float m_T_cmd;				// [Nm], 
    MECH_t m_w_cmd;				// [Ra/sec-mech]
    MECH_t m_th_cmd;			// [Ra-mech]

private:
    void (CDyno::* RunSimPtr)();
    void RunSimPosCtrl();
    void RunSimSpdCtrl();
    void RunSimTrqCtrl();

    ECtrlMode m_ctrl_mode;		// [#], dyno control mode

    MECH_t m_w_prev;			// [Ra/sec-mech]
    MECH_t m_th_prev;			// [Ra/sec-mech]

};
