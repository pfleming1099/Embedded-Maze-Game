/*
 * RNG_Driver.c
 *
 *  Created on: Nov 30, 2023
 *      Author: pflem
 */

#include "RNG_Driver.h"

static RNG_HandleTypeDef handleRNG = {0};
HAL_StatusTypeDef rng_status;

uint32_t random = 0;
void RNG_Init(){
	handleRNG.Instance = RNG;
	__HAL_RCC_RNG_CLK_ENABLE();
	HAL_RNG_Init(&handleRNG);
}

uint32_t RNG_getNumber(){
	RNG_HandleTypeDef handleRNG;
	uint32_t random;

	handleRNG.Instance = RNG;

	__HAL_RCC_RNG_CLK_ENABLE();
	HAL_RNG_Init(&handleRNG);

	rng_status = HAL_RNG_GenerateRandomNumber(&handleRNG, &random);

	HAL_RNG_DeInit(&handleRNG);

	return random;
}


