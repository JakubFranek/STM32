/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    crc.c
  * @brief   This file provides code for the configuration
  *          of the CRC instances.
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
#include "crc.h"

/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/* CRC init function */
void MX_CRC_Init(void)
{

  /* USER CODE BEGIN CRC_Init 0 */

  /* USER CODE END CRC_Init 0 */

  /* Peripheral clock enable */
  LL_AHB1_GRP1_EnableClock(LL_AHB1_GRP1_PERIPH_CRC);

  /* USER CODE BEGIN CRC_Init 1 */

  /* USER CODE END CRC_Init 1 */
  LL_CRC_SetInputDataReverseMode(CRC, LL_CRC_INDATA_REVERSE_NONE);
  LL_CRC_SetOutputDataReverseMode(CRC, LL_CRC_OUTDATA_REVERSE_NONE);
  LL_CRC_SetInitialData(CRC, LL_CRC_DEFAULT_CRC_INITVALUE);
  LL_CRC_SetPolynomialCoef(CRC, 49);
  LL_CRC_SetPolynomialSize(CRC, LL_CRC_POLYLENGTH_8B);
  /* USER CODE BEGIN CRC_Init 2 */

  /* USER CODE END CRC_Init 2 */

}

/* USER CODE BEGIN 1 */
uint8_t calculate_CRC(uint8_t* data, uint8_t length)
{
	// Reset the CRC calculation unit
	LL_CRC_ResetCRCCalculationUnit(CRC);

	// Feed data to CRC calculation
	for (uint32_t i = 0; i < length; i++) {
		LL_CRC_FeedData8(CRC, data[i]);
	}

	// Return the CRC result (only the last 8 bits are relevant for CRC8)
	return (uint8_t)LL_CRC_ReadData8(CRC);
}
/* USER CODE END 1 */
