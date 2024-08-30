/* ===========================================================================
** Infineon Confidential - Copyright [2023].
** All rights reserved.
** Author: hamid.behjati@infineon.com
** ===========================================================================
*/
#include "HardwareIface.h"
#include "Controller.h"

void CAN_Comm_RunISR1()
{
    /* Update CAN message. Sample and send button state. */
    can_0_node_0_LMO_0.can_data[0] = hw.mcu.isr1.count;

    /* Configure data to be transmitted and data length code */
    XMC_CAN_MO_UpdateData(&can_0_node_0_LMO_0);

    /* Send data in CAN_NODE_LMO_0 */
    XMC_CAN_MO_Transmit(&can_0_node_0_LMO_0);
}



