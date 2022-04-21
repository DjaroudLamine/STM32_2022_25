/*
 * timer.h
 *
 *  Created on: Feb 13, 2021
 *      Author: 
 */
#include "stm32l4xx_hal.h"

#ifndef INC_TIMER_H_
#define INC_TIMER_H_

uint32_t DWT_Delay_Init(void);

// This Function Provides Delay In Microseconds Using DWT
__STATIC_INLINE void DWT_Delay_us(volatile uint32_t au32_microseconds)
{
  uint32_t au32_initial_ticks = DWT->CYCCNT;
  uint32_t au32_ticks = (HAL_RCC_GetHCLKFreq() / 1000000);
  au32_microseconds *= au32_ticks;
  while ((DWT->CYCCNT - au32_initial_ticks) < au32_microseconds-au32_ticks);
}

#endif /* INC_TIMER_H_ */
