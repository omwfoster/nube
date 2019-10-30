/*
 * omwof_irq.h
 *
 *  Created on: 22 Oct 2019
 *      Author: oli
 */

#ifndef OMWOF_OMWOF_IRQ_H_
#define OMWOF_OMWOF_IRQ_H_




void TIM4_IRQHandler(void);
void BSP_AUDIO_IN_Error_Callback(void);
void NMI_Handler(void);
void HardFault_Handler(void);
void MemManage_Handler(void);
void BusFault_Handler(void);
void UsageFault_Handler(void);
void SVC_Handler(void);
void DebugMon_Handler(void);
void PendSV_Handler(void);
void I2S2_IRQHandler(void);



#endif /* OMWOF_OMWOF_IRQ_H_ */
