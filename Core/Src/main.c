/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2026 STMicroelectronics.
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
#include "cmsis_os.h"
#include "adc.h"
#include "dma.h"
#include "fdcan.h"
#include "i2c.h"
#include "usart.h"
#include "opamp.h"
#include "spi.h"
#include "tim.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "COMMUNICATION_FDCAN.h"
#include "as5047.h"
#include "trans.h"
#include "FOC_CAL.h"
#include "app_uart_dma_debug.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
void MX_FREERTOS_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
/* 1=?? TIM+ADC ?????????????? PWM/?????? setPhaseVoltage??????? dbg ??? */
#define BRINGUP_ADC_TEST 1

uint16_t A,B;
volatile uint8_t cnt=0;
int16_t adc_read[6];
float uq =2;

volatile DbgMon_t dbg;

static void dbg_snapshot_opamp_ch(uint8_t idx, OPAMP_TypeDef *opamp)
{
  uint32_t csr = opamp->CSR;
  dbg.opamp[idx].csr = csr;
  dbg.opamp[idx].en = (csr & OPAMP_CSR_OPAMPxEN) ? 1U : 0U;
  dbg.opamp[idx].intout = (csr & OPAMP_CSR_OPAMPINTEN) ? 1U : 0U;
  dbg.opamp[idx].pggain = (uint16_t)((csr & OPAMP_CSR_PGGAIN_Msk) >> OPAMP_CSR_PGGAIN_Pos);
}

static void dbg_snapshot_all(void)
{
  dbg_snapshot_opamp_ch(0, OPAMP1);
  dbg_snapshot_opamp_ch(1, OPAMP3);
  dbg_snapshot_opamp_ch(2, OPAMP4);
  dbg.opamp[0].hal_state = hopamp1.State;
  dbg.opamp[1].hal_state = hopamp3.State;
  dbg.opamp[2].hal_state = hopamp4.State;
  dbg.adc_shunt[0] = adc_read[0];
  dbg.adc_shunt[1] = adc_read[1];
  dbg.adc_shunt[2] = adc_read[2];
  dbg.adc_reg[0] = adc_read[3];
  dbg.adc_reg[1] = adc_read[4];
  dbg.adc_reg[2] = adc_read[5];
}
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
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_SPI1_Init();
  MX_USART1_UART_Init();
  MX_FDCAN1_Init();
  MX_SPI3_Init();
  MX_TIM1_Init();
  MX_TIM8_Init();
  MX_ADC2_Init();
  MX_OPAMP1_Init();
  MX_OPAMP3_Init();
  MX_OPAMP4_Init();
  MX_ADC1_Init();
  MX_ADC3_Init();
  MX_ADC5_Init();
  MX_I2C1_Init();
  MX_LPUART1_UART_Init();
  MX_TIM15_Init();
  /* USER CODE BEGIN 2 */
  if (HAL_OPAMP_Start(&hopamp1) != HAL_OK) { Error_Handler(); }
  if (HAL_OPAMP_Start(&hopamp3) != HAL_OK) { Error_Handler(); }
  if (HAL_OPAMP_Start(&hopamp4) != HAL_OK) { Error_Handler(); }
  HAL_Delay(10);
HAL_ADCEx_Calibration_Start(&hadc1,	ADC_SINGLE_ENDED);
HAL_ADCEx_Calibration_Start(&hadc2,	ADC_SINGLE_ENDED);
HAL_ADCEx_Calibration_Start(&hadc3,	ADC_SINGLE_ENDED);
HAL_ADCEx_Calibration_Start(&hadc5,	ADC_SINGLE_ENDED);
	HAL_ADCEx_InjectedStart_IT(&hadc1);
  	__HAL_ADC_CLEAR_FLAG(&hadc1, ADC_FLAG_JEOC);
	HAL_ADCEx_InjectedStart_IT(&hadc2);
	__HAL_ADC_CLEAR_FLAG(&hadc2, ADC_FLAG_JEOC);
	HAL_ADCEx_InjectedStart_IT(&hadc3);
	__HAL_ADC_CLEAR_FLAG(&hadc3, ADC_FLAG_JEOC);
	HAL_ADCEx_InjectedStart_IT(&hadc5);
	__HAL_ADC_CLEAR_FLAG(&hadc5, ADC_FLAG_JEOC);
	dbg_snapshot_all();

 FDCAN1_Config();
  AS5047_Init(&AS5047_spi1_PORT, &hspi1, GPIOA, GPIO_PIN_4);
    AS5047_Init(&AS5047_spi3_PORT, &hspi3, GPIOA, GPIO_PIN_15);
	angle_init();
  telem_bringup_init();
  
  HAL_TIM_PWM_Start(&htim1,TIM_CHANNEL_1);
	HAL_TIM_PWM_Start(&htim1,TIM_CHANNEL_2);
	HAL_TIM_PWM_Start(&htim1,TIM_CHANNEL_3);
	HAL_TIMEx_PWMN_Start(&htim1,TIM_CHANNEL_1);
	HAL_TIMEx_PWMN_Start(&htim1,TIM_CHANNEL_2);
	HAL_TIMEx_PWMN_Start(&htim1,TIM_CHANNEL_3);
	HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_4);
#if !BRINGUP_ADC_TEST
	

	HAL_TIM_PWM_Start(&htim8,TIM_CHANNEL_1);
	HAL_TIM_PWM_Start(&htim8,TIM_CHANNEL_2);
	HAL_TIM_PWM_Start(&htim8,TIM_CHANNEL_3);
	HAL_TIMEx_PWMN_Start(&htim8,TIM_CHANNEL_1);
	HAL_TIMEx_PWMN_Start(&htim8,TIM_CHANNEL_2);
	HAL_TIMEx_PWMN_Start(&htim8,TIM_CHANNEL_3);
	HAL_TIM_PWM_Start(&htim8, TIM_CHANNEL_4);
#endif
	/* BRINGUP_ADC_TEST ??? TIM1 CH4 ???? ADC ??? */
	HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_4);
	HAL_TIM_Base_Start_IT(&htim1);
#if !BRINGUP_ADC_TEST
	HAL_TIM_Base_Start_IT(&htim8);
#endif

  /* USER CODE END 2 */

  /* Init scheduler */
  osKernelInitialize();

  /* Call init function for freertos objects (in cmsis_os2.c) */
  MX_FREERTOS_Init();

  /* Start scheduler */
  osKernelStart();

  /* We should never get here as control is now taken by the scheduler */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
//	  FDCAN_MOTER_START(&hfdcan1,100,200,300,400);
//	  HAL_Delay(1);
//	  HAL_UART_Transmit_DMA(&hlpuart1,tx_uart_data,sizeof(tx_uart_data));
//	  HAL_Delay(1);
//	  HAL_UART_Receive(&hlpuart1,rx_uart_data,sizeof (rx_uart_data),0xff);
//	  HAL_Delay(1);
//	  A= AS5047_read(&AS5047_spi1_PORT,ANGLEUNC);
//	  tx_uart_data[0]=A>>8;
//	  tx_uart_data[1]=A;
//	  HAL_Delay(1);
//	  B= AS5047_read(&AS5047_spi3_PORT,ANGLEUNC);
//	  tx_uart_data[2]=B>>8;
//	  tx_uart_data[3]=B;
//	  HAL_Delay(1);
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
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage
  */
  HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1_BOOST);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = RCC_PLLM_DIV1;
  RCC_OscInitStruct.PLL.PLLN = 20;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = RCC_PLLQ_DIV2;
  RCC_OscInitStruct.PLL.PLLR = RCC_PLLR_DIV2;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_4) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */
void HAL_ADCEx_InjectedConvCpltCallback(ADC_HandleTypeDef *hadc)
{
    UNUSED(hadc);
    
    if (hadc == &hadc1) {
        // ??????????????????????????
         adc_read[0] = (int16_t)hadc1.Instance->JDR1;
         dbg.opamp[0].adc_jdr1 = (uint16_t)hadc1.Instance->JDR1;
         dbg.opamp[0].adc_irq_cnt++;
         dbg_snapshot_all();
    }
	if (hadc == &hadc2) {
		adc_read[3] = (int16_t)hadc2.Instance->JDR1;
		adc_read[4] = (int16_t)hadc2.Instance->JDR2;
		adc_read[5] = (int16_t)hadc2.Instance->JDR3;
		dbg.adc_reg[0] = adc_read[3];
		dbg.adc_reg[1] = adc_read[4];
		dbg.adc_reg[2] = adc_read[5];
	}
	if (hadc == &hadc3) {
		adc_read[1] = (int16_t)hadc3.Instance->JDR1;
		dbg.opamp[1].adc_jdr1 = (uint16_t)hadc3.Instance->JDR1;
		dbg.opamp[1].adc_irq_cnt++;
		dbg_snapshot_all();
	}
	if (hadc == &hadc5) {
		adc_read[2] = (int16_t)hadc5.Instance->JDR1;
		dbg.opamp[2].adc_jdr1 = (uint16_t)hadc5.Instance->JDR1;
		dbg.opamp[2].adc_irq_cnt++;
		dbg_snapshot_all();
	}

}
/* USER CODE END 4 */

/**
  * @brief  Period elapsed callback in non blocking mode
  * @note   This function is called  when TIM7 interrupt took place, inside
  * HAL_TIM_IRQHandler(). It makes a direct call to HAL_IncTick() to increment
  * a global variable "uwTick" used as application time base.
  * @param  htim : TIM handle
  * @retval None
  */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  /* USER CODE BEGIN Callback 0 */

  /* USER CODE END Callback 0 */
  if (htim->Instance == TIM7) {
    HAL_IncTick();
  }
  /* USER CODE BEGIN Callback 1 */
  if (htim == &htim1) {
    volatile uint32_t isr_t0 = *(volatile uint32_t *)&DWT->CYCCNT;

    cnt++;
    if (cnt >= 200) {
    }
//    as5047_spi1.get = AS5047_GetAngle(&AS5047_spi1_PORT) * 7;
    setPhaseVoltage(&htim1, uq, 0, as5047_spi1.get+as5047_spi1.add);
#if !BRINGUP_ADC_TEST
//    setPhaseVoltage(&htim1, uq, 0, as5047_spi1.get);
//    as5047_spi1.get = AS5047_GetAngle(&AS5047_spi1_PORT) * 7;
//    setPhaseVoltage(&htim8, uq, 0, as5047_spi3.get);
#endif
//    adc_read[3] = hadc2.Instance->JDR1;
    telem_bringup_tick();
    __DSB();
    {
      volatile uint32_t isr_t1 = *(volatile uint32_t *)&DWT->CYCCNT;

      g_telem_dbg.isr_t0 = isr_t0;
      g_telem_dbg.isr_t1 = isr_t1;
      g_telem_dbg.cyccnt_end = isr_t1;
      g_telem_dbg.isr_delta = isr_t1 - isr_t0;
    }
  }
  /* USER CODE END Callback 1 */
}

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
