/* ===========================================================================
 ** Infineon Confidential - Copyright [2024].
 ** All rights reserved.
 ** Author: hamid.behjati@infineon.com
 ** ===========================================================================
 */

#include "cybsp.h"
#include "stdio.h"
#include "General.h"

#ifndef MOTOR_CTRL_HW_CONFIG
#define MOTOR_CTRL_HW_CONFIG

/* Temperature sensor configurations */
#define ACTIVE_TEMP_SENSOR  true        // Active IC (e.g. MCP9700T-E/TT) vs Passive NTC (e.g. NCP18WF104J03RB)
extern  TEMP_SENS_LUT_t     Temp_Sens_LUT;
#define TEMP_SENSOR_OFFSET (0.5f)       // MCP9700AT-E/LT: 500mV is 0 ℃
#define TEMP_SENSOR_SCALE  (0.010f)     // MCP9700AT-E/LT: 10mV/℃
// #define TEMP_SENSOR_OFFSET (0.4f)       // MAX6612: 400mV is 0 ℃
// #define TEMP_SENSOR_SCALE  (0.01953f)     // MAX6612: 19.53mV/℃


/* ADC configurations*/
//#define ADC_VREF_GAIN		((5.0f)/(3.3f))         // [V/V], voltage-reference buffer gain (e.g. scaling 5.0V down to 3.3V)
#define ADC_VREF_GAIN		(1.0f)         // [V/V], No need for scaling for matchbox board, just 3.3V
#define ADC_CS_OPAMP_GAIN   (24.0f)                 // [V/V]
#define ADC_CS_SHUNT_RES	(1.0E-3f)				// [Ohm], cs shunt-resistor value, default
#define ADC_CS_SETTLE_RATIO	(0.5f)					// [], settling ratio used for single-shunt current sampling
#define ADC_SCALE_VUVW      ((5.6f)/(56.0f+5.6f))   // [V/V] = [Ohm/Ohm]
#define ADC_SCALE_VDC       ((10.0f)/(150.0f+10.0f))   // [V/V] = [Ohm/Ohm]


enum
{
    // ADC0 sequence
    ADC_ISAMPB = 0, ADC_ISAMPC = 1, ADC_TEMP = 2,   ADC_VBUS = 3,
    // ADC1 sequence
    ADC_ISAMPA = 4, ADC_ISAMPD = 5, ADC_ISAMPE = 6, ADC_VPOT = 7,
    // ADC2 sequence
    ADC_ISAMPF = 8, ADC_VU = 9,     ADC_VW = 10,    ADC_VV = 11,
    // Totals
    ADC_SEQ_MAX = 3, ADC_SAMP_PER_SEQ_MAX = 4, ADC_MAX = 12
};

// GPDMA pointers used by device configurator
extern volatile int32_t* DMA_ADC_0_Src_Addr;
extern volatile int32_t* DMA_ADC_0_Des_Addr;
extern volatile int32_t* DMA_ADC_1_Src_Addr;
extern volatile int32_t* DMA_ADC_1_Des_Addr;
#if defined(DMA_ADC_2_HW)
extern volatile int32_t* DMA_ADC_2_Src_Addr;
extern volatile int32_t* DMA_ADC_2_Des_Addr;
#endif

// 3 simultaneous sampling ADCs
void MCU_RoutingConfigMUXA();  // Routing ADCs, ADC0::[ISAMPA,ISAMPD,VU,VV] & ADC1::[ISAMPB,ISAMPE,VBUS,VPOT] & ADC2::[ISAMPC,ISAMPF,VW,TEMP], {ISAMPA,ISAMPB,ISAMPC}={IU,IV,IW}
void MCU_RoutingConfigMUXB();  // Routing ADCs, ADC0::[ISAMPA,ISAMPD,VU,VV] & ADC1::[ISAMPB,ISAMPE,VBUS,VPOT] & ADC2::[ISAMPC,ISAMPF,VW,TEMP], {ISAMPD,ISAMPE,ISAMPF}={-,I_DC_LINK,I_DC_LINK}

// Enable/disable timer reloads
void MCU_EnableTimerReload();
void MCU_DisableTimerReload();

#define CY_CFG_PWR_VDDA_MV	(3300.0f)

/* PWM configurations*/
#define PWM_INVERSION       false
#define PWM_TRIG_ADVANCE    0U		// [ticks]

/* Miscellaneous BSP definitions */
#define XMC_CCU8_SHADOW_TRANSFER_PWM_U_SLICE		XMC_CCU8_SHADOW_TRANSFER_SLICE_2
#define XMC_CCU8_SHADOW_TRANSFER_PWM_V_SLICE		XMC_CCU8_SHADOW_TRANSFER_SLICE_1
#define XMC_CCU8_SHADOW_TRANSFER_PWM_W_SLICE		XMC_CCU8_SHADOW_TRANSFER_SLICE_0
#define XMC_CCU8_SHADOW_TRANSFER_ADC0_ISR0_SLICE	XMC_CCU8_SHADOW_TRANSFER_SLICE_3
#define XMC_CCU8_SHADOW_TRANSFER_ADC1_ISR0_SLICE	XMC_CCU8_SHADOW_TRANSFER_SLICE_3
#define XMC_CCU8_SHADOW_TRANSFER_SYNC_ISR1_SLICE	XMC_CCU8_SHADOW_TRANSFER_SLICE_0

#if defined(COMPONENT_XMC42XX)
#define XMC_CCU4_SHADOW_TRANSFER_SYNC_ISR1_SLICE	XMC_CCU4_SHADOW_TRANSFER_SLICE_0
#endif

#define DMA_ADC_0_DMA_ADC_2_RunISR		GPDMA0_INTERRUPT_HANDLER
#define DMA_ADC_1_RunISR				ADC_1_SR3_INTERRUPT_HANDLER
#define	MCU_RunISR1						CCU40_3_IRQHandler  //CCU40_0_IRQHandler

#define	DMA_ADC_0_IRQ		GPDMA0_0_IRQn
#define	DMA_ADC_1_IRQ		VADC0_G1_3_IRQn
#define SYNC_ISR1_IRQ		CCU40_3_IRQn

#define ADC_0_TRIG			XMC_VADC_REQ_TR_I	 // CCU8.0.SR2
#define ADC_1_TRIG			XMC_VADC_REQ_TR_J	 // CCU8.0.SR3
#define ADC_2_TRIG			XMC_VADC_REQ_TR_I	 // CCU8.0.SR2

#define KIT_ID				(0x0007UL)	// For GUI's recognition of HW

#endif
