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
#include "cordic.h"
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
#include "board_encoder.h"
#include "encoder.h"
#include "app_uart_dma_debug.h"
#include "bsp_axes.h"
#include "encoder_cal.h"
#include "adc_foc_port.h"
#include "factory_nvm.h"
#include "motor_current.h"
#include "motor_params_m1.h"
#include "motor_phase_binding.h"
#include "phase_detect.h"
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
/* M1 20kHz 控制节拍�? ADC2 JEOC；TIM8 �? PWM+CH4 触发，无 Update IT */
#define BRINGUP_ADC_TEST 1

uint16_t A,B;
volatile uint8_t cnt=0;
int16_t adc_read[6];

static void dbg_snapshot_opamp_ch(uint8_t idx, OPAMP_TypeDef *opamp)
{
  uint32_t csr = opamp->CSR;
  dbg.opamp[idx].csr = csr;
  dbg.opamp[idx].en = (csr & OPAMP_CSR_OPAMPxEN) ? 1U : 0U;
  dbg.opamp[idx].intout = (csr & OPAMP_CSR_OPAMPINTEN) ? 1U : 0U;
  dbg.opamp[idx].pggain = (uint16_t)((csr & OPAMP_CSR_PGGAIN_Msk) >> OPAMP_CSR_PGGAIN_Pos);
}

static void dbg_snapshot_m1_adc(const bsp_axis_t *m1)
{
  uint8_t i;

  for (i = 0U; i < 3U; i++) {
    dbg.adc_offset[i] = m1->adc_cfg.ch[i].offset;
    dbg.adc_zeroed[i] = (int16_t)((int32_t)m1->adc.raw[i] - m1->adc_cfg.ch[i].offset);
    dbg.adc_reg[i] = m1->adc.raw[i];
  }
  dbg.adc_ia = m1->adc.ia;
  dbg.adc_ib = m1->adc.ib;
  dbg.adc_ic = m1->adc.ic;
}

static void dbg_snapshot_phase_binding(void)
{
  const motor_phase_binding_t *b = motor_phase_binding_get();
  uint8_t i;

  for (i = 0U; i < 3U; i++) {
    dbg.pwm_ch_to_phase_dbg[i] = b->pwm_ch_to_phase[i];
    dbg.adc_rank_to_phase_dbg[i] = b->adc_rank_to_phase[i];
    dbg.phase_sign_dbg[i] = b->phase_sign[i];
  }
}

/** 2325 Test3 sign；rank 排列随 M1_ADC_RANK_SWAP_IAIC 与 adc.c 注入顺序一致 */
static void m1_apply_binding_2325(void)
{
  motor_phase_binding_t b;

  motor_phase_binding_set_identity(&b);
#if M1_ADC_RANK_SWAP_IAIC
  /* JDR1=PC4(ia) JDR2=PA0(ib) JDR3=PC3(ic) */
  b.adc_rank_to_phase[0] = 0U;
  b.adc_rank_to_phase[1] = 1U;
  b.adc_rank_to_phase[2] = 2U;
#else
  /* JDR1=PC3(ic) JDR2=PA0(ib) JDR3=PC4(ia) */
  b.adc_rank_to_phase[0] = 2U;
  b.adc_rank_to_phase[1] = 1U;
  b.adc_rank_to_phase[2] = 0U;
#endif
  b.phase_sign[0] = -1;
  b.phase_sign[1] = -1;
  b.phase_sign[2] = -1;
  motor_phase_binding_set_active(&b, true);
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
  MX_CORDIC_Init();
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
	bsp_init();
	motor_current_init(bsp_axis(BSP_AXIS_M1));
	if (!bsp_axis_adc_calibrate_zero(BSP_AXIS_M1, 20, 100, 50)) {
		Error_Handler();
	}
	dbg_snapshot_m1_adc(bsp_axis(BSP_AXIS_M1));
	dbg_snapshot_all();

 FDCAN1_Config();
  AS5047_Init(&AS5047_spi3_PORT, &hspi3, GPIOA, GPIO_PIN_15);
  telem_bringup_init();
  telem_encoder_profile_bind(&enc_m1);

  /* M1: TIM8 PWM + CH4→ADC2；锁转子标定须在 Base+CH4+�? PWM 运行后进�? */
  HAL_TIM_PWM_Start(&htim8, TIM_CHANNEL_1);
  HAL_TIM_PWM_Start(&htim8, TIM_CHANNEL_2);
  HAL_TIM_PWM_Start(&htim8, TIM_CHANNEL_3);
  HAL_TIMEx_PWMN_Start(&htim8, TIM_CHANNEL_1);
  HAL_TIMEx_PWMN_Start(&htim8, TIM_CHANNEL_2);
  HAL_TIMEx_PWMN_Start(&htim8, TIM_CHANNEL_3);
  HAL_TIM_PWM_Start(&htim8, TIM_CHANNEL_4);
  HAL_TIM_Base_Start(&htim8);

  dbg.phase_cal_ok = 0U;
  dbg.binding_loaded = 0U;
  dbg.phase_cal_fail = 0U;
  dbg.phase_cal_fail_reason = PHASE_CAL_FAIL_NONE;
  dbg.phase_cal_channels_done = 0U;
  dbg.phase_cal_last_snr = 0.0f;

#if M1_APPLY_PHASE_BINDING && !M1_RUN_PHASE_CAL
#if M1_BINDING_OVERRIDE_2325
  m1_apply_binding_2325();
  dbg.binding_loaded = 1U;
  dbg_snapshot_phase_binding();
#else
  if (factory_nvm_apply_phase_binding()) {
    dbg.binding_loaded = 1U;
    dbg_snapshot_phase_binding();
  } else {
    motor_phase_binding_t id;

    motor_phase_binding_set_identity(&id);
    motor_phase_binding_set_active(&id, false);
  }
#endif
#endif

#if M1_RUN_PHASE_CAL
  {
    motor_phase_binding_t binding;
    bsp_axis_t *m1 = bsp_axis(BSP_AXIS_M1);

    if (phase_detect_run(&m1->adc, &htim8, &binding, false)) {
      dbg.phase_cal_ok = 1U;
      dbg.phase_cal_fail = 0U;
      dbg.phase_cal_fail_reason = PHASE_CAL_FAIL_NONE;
      dbg_snapshot_phase_binding();
    } else {
      motor_phase_binding_t id;

      motor_phase_binding_set_identity(&id);
      motor_phase_binding_set_active(&id, false);
      dbg.phase_cal_ok = 0U;
      dbg.phase_cal_fail = 1U;
    }
    g_cal_hold = 1U;
    for (;;) {
      __WFI();
    }
  }
#endif

#if M1_RUN_ENCODER_CAL
  {
    float add_raw;
    float add_final;

    if (!encoder_cal_run_lock_default(bsp_axis(BSP_AXIS_M1), &enc_m1, M1_POLE_PAIRS, &add_raw)) {
      Error_Handler();
    }
    dbg.enc_cal_add_raw = add_raw;
    add_final = encoder_cal_apply_pi_offset(add_raw);
    encoder_set_theta_el_offset(&enc_m1, add_final);
    dbg.enc_cal_add = add_final;
  }
#else
  encoder_set_theta_el_offset(&enc_m1, M1_ENCODER_OFFSET_RAD);
  dbg.enc_cal_add_raw = M1_ENCODER_OFFSET_RAD;
  dbg.enc_cal_add = M1_ENCODER_OFFSET_RAD;
  encoder_kick(&enc_m1);
#endif
#if !BRINGUP_ADC_TEST
  /* Second axis (TIM1) �� no ISR control loop here */
  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);
  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_2);
  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_3);
  HAL_TIMEx_PWMN_Start(&htim1, TIM_CHANNEL_1);
  HAL_TIMEx_PWMN_Start(&htim1, TIM_CHANNEL_2);
  HAL_TIMEx_PWMN_Start(&htim1, TIM_CHANNEL_3);
  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_4);
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
		bsp_axis_t *m1 = bsp_axis(BSP_AXIS_M1);

		if (m1->adc.cal_active) {
			adc_sample_on_injected(&m1->adc, hadc);
		} else if (g_phase_cal_active) {
			phase_detect_jeoc_tick(&m1->adc, hadc, &htim8);
		} else if (g_cal_hold) {
			phase_detect_hold_jeoc_tick(&m1->adc, hadc, &htim8);
		} else if (g_encoder_cal_active) {
			encoder_cal_jeoc_tick(m1);
		} else {
			adc_foc_port_on_jeoc(m1->adc_foc, &m1->adc, hadc);
			motor_current_tick(m1);
		}
		adc_read[3] = m1->adc.raw[0];
		adc_read[4] = m1->adc.raw[1];
		adc_read[5] = m1->adc.raw[2];
		dbg_snapshot_m1_adc(m1);
		if (g_phase_cal_active || g_cal_hold) {
			telem_bringup_tick();
			telem_bringup_try_send();
		}
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
  /* M1 控制已迁�? ADC2 JEOC；TIM8 �? Update IT */
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
