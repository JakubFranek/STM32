/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
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
#include "main.h"
#include "spi.h"
#include "tim.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "nrf24l01p.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define TRANSMITTER

//#define RECEIVER
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */
#define NRF24_DATA_LENGTH 8
/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */

// Following function prototypes needed here to init private variable nrf24_config
void nrf24l01p_set_cs(uint8_t state);
void nrf24l01p_set_ce(uint8_t state);

nrf24l01p_config_t nrf24_config = {
		.interface = {
				.set_cs = &nrf24l01p_set_cs,
				.set_ce = &nrf24l01p_set_ce,
				.spi_tx = &SPI1_Transmit,
				.spi_rx = &SPI1_Receive,
				.spi_tx_rx = &SPI1_TransmitReceive
		},
		.channel_MHz = 2500,
		.address_width = 5,
		.data_rate = NRF24L01P_1MBPS,
		.crc_length = NRF24L01P_CRC_1BYTE,
		.output_power = NRF24L01P_0DBM,
		.auto_ack_pipes = 0b00111111,
		.auto_retransmit_count = 3,
		.auto_retransmit_delay_250us = 1,
		.data_length = NRF24_DATA_LENGTH
};
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */
void EXTI1_Callback(void);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  LL_APB2_GRP1_EnableClock(LL_APB2_GRP1_PERIPH_SYSCFG);
  LL_APB1_GRP1_EnableClock(LL_APB1_GRP1_PERIPH_PWR);

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_SPI1_Init();
  MX_TIM2_Init();
  /* USER CODE BEGIN 2 */
  LL_SPI_Enable(SPI1);
  nrf24l01p_init(&nrf24_config);

#ifdef RECEIVER
  nrf24l01p_set_prx_mode();
  uint8_t rx_data[NRF24_DATA_LENGTH] = {0};
  nrf24l01p_set_tx_addr(0);
  nrf24l01p_set_rx_addr(0, 0);
#endif

#ifdef TRANSMITTER
  nrf24l01p_set_ptx_mode();
  uint8_t tx_data[NRF24_DATA_LENGTH] = {0, 1, 2, 3, 4, 5, 6, 7};
  //nrf24l01p_set_tx_addr(0xE7E7E7E7E7);
  //nrf24l01p_set_rx_addr(0, 0xE7E7E7E7E7);
#endif
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
#ifdef RECEIVER
	  // Nothing to do
#endif

#ifdef TRANSMITTER

	  nrf24l01p_power_up();

	  TIM2_delay_us(1500);
	  nrf24l01p_set_ce(1);

	  TIM2_delay_us(130);
	  nrf24l01p_write_tx_fifo(tx_data);
	  for(int i= 0; i < 8; i++)
	  {
		  tx_data[i]++;
	  }
#endif
	  //LL_GPIO_SetOutputPin(NRF24L01P_CE_PIN_PORT, NRF24L01P_CE_PIN_NUMBER);
	  LL_mDelay(250);
	  //LL_GPIO_ResetOutputPin(NRF24L01P_CE_PIN_PORT, NRF24L01P_CE_PIN_NUMBER);
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  LL_FLASH_SetLatency(LL_FLASH_LATENCY_1);
  while(LL_FLASH_GetLatency()!= LL_FLASH_LATENCY_1)
  {
  }
  LL_PWR_SetRegulVoltageScaling(LL_PWR_REGU_VOLTAGE_SCALE1);
  while (LL_PWR_IsActiveFlag_VOS() != 0)
  {
  }
  LL_RCC_HSI_Enable();

   /* Wait till HSI is ready */
  while(LL_RCC_HSI_IsReady() != 1)
  {

  }
  LL_RCC_HSI_SetCalibTrimming(16);
  LL_RCC_PLL_ConfigDomain_SYS(LL_RCC_PLLSOURCE_HSI, LL_RCC_PLL_MUL_4, LL_RCC_PLL_DIV_2);
  LL_RCC_PLL_Enable();

   /* Wait till PLL is ready */
  while(LL_RCC_PLL_IsReady() != 1)
  {

  }
  LL_RCC_SetAHBPrescaler(LL_RCC_SYSCLK_DIV_1);
  LL_RCC_SetAPB1Prescaler(LL_RCC_APB1_DIV_1);
  LL_RCC_SetAPB2Prescaler(LL_RCC_APB2_DIV_1);
  LL_RCC_SetSysClkSource(LL_RCC_SYS_CLKSOURCE_PLL);

   /* Wait till System clock is ready */
  while(LL_RCC_GetSysClkSource() != LL_RCC_SYS_CLKSOURCE_STATUS_PLL)
  {

  }

  LL_Init1msTick(32000000);

  LL_SetSystemCoreClock(32000000);
}

/* USER CODE BEGIN 4 */
void EXTI1_Callback(void)
{
	if(!LL_GPIO_IsInputPinSet(nRF24_IRQ_GPIO_Port, nRF24_IRQ_Pin))
	{
		nrf24l01p_irq_t irq_sources;
		if(nrf24l01p_irq(&irq_sources) != NRF24L01P_SUCCESS)
			return;

#ifdef RECEIVER
		if (irq_sources.rx_dr)
			nrf24l01p_rx_receive(rx_data);
#endif

#ifdef TRANSMITTER
		if (irq_sources.tx_ds)
			LL_GPIO_TogglePin(LD3_GPIO_Port, LD3_Pin);
		else if (irq_sources.max_rt)
			LL_GPIO_ResetOutputPin(LD3_GPIO_Port, LD3_Pin);
#endif
	}
}

void nrf24l01p_set_cs(uint8_t state)
{
	if (state)
		LL_GPIO_SetOutputPin(nRF24_CSN_GPIO_Port, nRF24_CSN_Pin);
	else
		LL_GPIO_ResetOutputPin(nRF24_CSN_GPIO_Port, nRF24_CSN_Pin);
}

void nrf24l01p_set_ce(uint8_t state)
{
	if (state)
		LL_GPIO_SetOutputPin(nRF24_CE_GPIO_Port, nRF24_CE_Pin);
	else
		LL_GPIO_ResetOutputPin(nRF24_CE_GPIO_Port, nRF24_CE_Pin);
}
/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
