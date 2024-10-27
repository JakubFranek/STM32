/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    tim.c
  * @brief   This file provides code for the configuration
  *          of the TIM instances.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2024 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "tim.h"

/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/* TIM2 init function */
void MX_TIM2_Init(void)
{

  /* USER CODE BEGIN TIM2_Init 0 */

  /* USER CODE END TIM2_Init 0 */

  LL_TIM_InitTypeDef TIM_InitStruct = {0};

  /* Peripheral clock enable */
  LL_APB1_GRP1_EnableClock(LL_APB1_GRP1_PERIPH_TIM2);

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  TIM_InitStruct.Prescaler = 32;
  TIM_InitStruct.CounterMode = LL_TIM_COUNTERMODE_UP;
  TIM_InitStruct.Autoreload = 65535;
  TIM_InitStruct.ClockDivision = LL_TIM_CLOCKDIVISION_DIV1;
  LL_TIM_Init(TIM2, &TIM_InitStruct);
  LL_TIM_DisableARRPreload(TIM2);
  LL_TIM_SetClockSource(TIM2, LL_TIM_CLOCKSOURCE_INTERNAL);
  LL_TIM_SetTriggerOutput(TIM2, LL_TIM_TRGO_RESET);
  LL_TIM_DisableMasterSlaveMode(TIM2);
  /* USER CODE BEGIN TIM2_Init 2 */

  /* USER CODE END TIM2_Init 2 */

}

/* USER CODE BEGIN 1 */

void TIM2_delay_us(uint16_t us)
{
	uint16_t ticks = us;

	LL_TIM_DisableCounter(TIM2);
	LL_TIM_SetCounter(TIM2, 0);
	LL_TIM_EnableCounter(TIM2);
	while (LL_TIM_GetCounter(TIM2) < ticks);
	LL_TIM_DisableCounter(TIM2);
}

void TIM2_restart()
{
	LL_TIM_DisableCounter(TIM2);
	LL_TIM_SetCounter(TIM2, 0);
	LL_TIM_EnableCounter(TIM2);
}

void TIM2_stop()
{
	LL_TIM_DisableCounter(TIM2);
}

uint16_t TIM2_get_count()
{
	return LL_TIM_GetCounter(TIM2);
}

void schedule_interrupt(uint16_t us)
{
	LL_TIM_SetCounter(TIM21, 0);
	LL_TIM_SetAutoReload(TIM21, us);
	LL_TIM_EnableIT_UPDATE(TIM21);
	LL_TIM_EnableCounter(TIM21);
}

void disable_scheduled_interrupt(void)
{
	LL_TIM_DisableCounter(TIM21);
	LL_TIM_DisableIT_UPDATE(TIM21);
	LL_TIM_ClearFlag_UPDATE(TIM21);
}

/* USER CODE END 1 */
