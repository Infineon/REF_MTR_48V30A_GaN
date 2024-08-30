/* ===========================================================================
 ** Infineon Confidential - Copyright [2024].
 ** All rights reserved.
 ** Author: hamid.behjati@infineon.com
 ** ===========================================================================
 */

#include "MotorCtrlHWConfig.h"
#include "HardwareIface.h"

TEMP_SENS_LUT_t   Temp_Sens_LUT   = 
{
    .step = 1.0f / (TEMP_SENS_LUT_WIDTH + 1.0f),    // [%], normalized voltage wrt Vcc
    .step_inv = (TEMP_SENS_LUT_WIDTH + 1.0f),       // [1/%], inverse normalized voltage
    .val = {109.5f, 85.4f, 71.7f, 62.0f, 54.3f, 47.7f, 41.9f, 36.5f, 31.4f, 26.3f, 21.2f, 16.0f, 10.2f, 3.7f, -4.3f, -16.1f} // [degree C]
};

#define ADC_RESULT_ADDR(instance, channel)    ((void*)&VADC_G##instance->RES[channel])

volatile int32_t* DMA_ADC_0_Src_Addr = ADC_RESULT_ADDR(0, 0);
volatile int32_t* DMA_ADC_0_Des_Addr = (&hw.mcu.dma_results[ADC_ISAMPB]);
volatile int32_t* DMA_ADC_1_Src_Addr = ADC_RESULT_ADDR(1, 0);
volatile int32_t* DMA_ADC_1_Des_Addr = (&hw.mcu.dma_results[ADC_ISAMPA]);

#if defined(DMA_ADC_2_HW)
volatile int32_t* DMA_ADC_2_Src_Addr = ADC_RESULT_ADDR(2, 0);
volatile int32_t* DMA_ADC_2_Des_Addr = (&hw.mcu.dma_results[ADC_ISAMPC]);
#endif

void MCU_RoutingConfigMUXA() {}
void MCU_RoutingConfigMUXB()
{
    hw.mcu.adc_mux.idx_isamp[0] = ADC_ISAMPF;
    hw.mcu.adc_mux.idx_isamp[1] = ADC_ISAMPE;
    hw.mcu.adc_mux.idx_isamp[2] = ADC_ISAMPD;
}

void MCU_DisableTimerReload()
{
    XMC_CCU8_SLICE_StartConfig(PWM_U_HW, XMC_CCU8_SLICE_EVENT_0, XMC_CCU8_SLICE_START_MODE_TIMER_START);  
    XMC_CCU8_SLICE_StartConfig(PWM_V_HW, XMC_CCU8_SLICE_EVENT_0, XMC_CCU8_SLICE_START_MODE_TIMER_START);
    XMC_CCU8_SLICE_StartConfig(PWM_W_HW, XMC_CCU8_SLICE_EVENT_0, XMC_CCU8_SLICE_START_MODE_TIMER_START);
    XMC_CCU8_SLICE_StartConfig(ADC0_ISR0_HW, XMC_CCU8_SLICE_EVENT_0, XMC_CCU8_SLICE_START_MODE_TIMER_START);
    XMC_CCU8_SLICE_StartConfig(ADC1_ISR0_HW, XMC_CCU8_SLICE_EVENT_0, XMC_CCU8_SLICE_START_MODE_TIMER_START);
}

void MCU_EnableTimerReload()
{
    XMC_CCU8_SLICE_StartConfig(PWM_U_HW, XMC_CCU8_SLICE_EVENT_0, XMC_CCU8_SLICE_START_MODE_TIMER_START_CLEAR);  
    XMC_CCU8_SLICE_StartConfig(PWM_V_HW, XMC_CCU8_SLICE_EVENT_0, XMC_CCU8_SLICE_START_MODE_TIMER_START_CLEAR);
    XMC_CCU8_SLICE_StartConfig(PWM_W_HW, XMC_CCU8_SLICE_EVENT_0, XMC_CCU8_SLICE_START_MODE_TIMER_START_CLEAR);
    XMC_CCU8_SLICE_StartConfig(ADC0_ISR0_HW, XMC_CCU8_SLICE_EVENT_0, XMC_CCU8_SLICE_START_MODE_TIMER_START_CLEAR);
    XMC_CCU8_SLICE_StartConfig(ADC1_ISR0_HW, XMC_CCU8_SLICE_EVENT_0, XMC_CCU8_SLICE_START_MODE_TIMER_START_CLEAR);
}
