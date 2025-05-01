/*
 * RNG_Driver.h
 *
 *  Created on: Nov 30, 2023
 *      Author: pflem
 */

#ifndef SRC_RNG_DRIVER_H_
#define SRC_RNG_DRIVER_H_

#include "../../Drivers/STM32F4xx_HAL_Driver/Inc/stm32f4xx_hal.h"

void RNG_Init();
uint32_t RNG_getNumber();

#endif /* SRC_RNG_DRIVER_H_ */
