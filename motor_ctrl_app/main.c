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


#include "cybsp.h"
#include "stdio.h"
#if defined(CY_USING_HAL)
#include "cyhal.h"
#endif
#include "cy_retarget_io.h"
#include "../HwInterface/HardwareIface.h"
#include "Controller.h"

int main(void)
{
    cy_rslt_t result;

#if defined (CY_DEVICE_SECURE)
    cyhal_wdt_t wdt_obj;
    // Clear watchdog timer so that it doesn't trigger a reset
    result = cyhal_wdt_init(&wdt_obj, cyhal_wdt_get_max_timeout_ms());
    CY_ASSERT(result == CY_RSLT_SUCCESS);
    cyhal_wdt_free(&wdt_obj);
#endif

    // Initialize the device and board peripherals
    result = cybsp_init();

    /* Manually added start, clear the trap event2*/
    //XMC_SCU_SetCcuTriggerLow(SCU_GENERAL_CCUCON_GSC40_Msk|SCU_GENERAL_CCUCON_GSC80_Msk);
    XMC_SCU_SetCcuTriggerHigh(SCU_GENERAL_CCUCON_GSC40_Msk|SCU_GENERAL_CCUCON_GSC80_Msk);
    ADC0_ISR0_HW->SWR |= 0x00000C01U;
    PWM_W_HW->SWR |= 0x00000C01U;
    PWM_V_HW->SWR |= 0x00000C01U;
    PWM_U_HW->SWR |= 0x00000C01U;
    // Start the timers synchronously

    /* Manually added end*/

    CY_ASSERT(result == CY_RSLT_SUCCESS);

    // Initialize retarget-io to use the debug UART port
#if defined(COMPONENT_CAT1)
    result = cy_retarget_io_init(CYBSP_DEBUG_UART_TX, CYBSP_DEBUG_UART_RX, CY_RETARGET_IO_BAUDRATE);
#elif defined(COMPONENT_CAT3)
    result = cy_retarget_io_init(CYBSP_DEBUG_UART_HW);
#endif
    CY_ASSERT(result == CY_RSLT_SUCCESS);

    // Initialize controller
    HW_IFACE_ConnectFcnPointers(); // must be called before STATE_MACHINE_Init()
    STATE_MACHINE_Init();
	printf("Init Done\r\n");

    // Enable global interrupts
    __enable_irq();

    (void) (result);

    for (;;)
    {
#if defined(COMPONENT_CAT1)
    	Cy_SysLib_Delay(1000);
#elif defined(COMPONENT_CAT3)
    	XMC_Delay(1000);
#endif
        //printf("\r\nHEART BEAT!\r\nFaults {HW, SW} = {%08lx, %08lx}\r\n", faults.flags_latched.hw.reg, faults.flags_latched.sw.reg);
    }
}
