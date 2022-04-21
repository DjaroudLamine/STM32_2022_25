/*
 * timer.h
 *
 *  Created on: Feb 13, 2021
 *      Author: JUANB
 */

#include "stm32l4xx_hal.h"

#ifndef INC_HT_H_
#define INC_HT_H_

void Data_Output (GPIO_TypeDef *PORT, uint16_t PIN);
void Data_Input (GPIO_TypeDef *PORT, uint16_t PIN);
void Read_data (uint8_t *data);

#endif /* INC_HT_H_ */
