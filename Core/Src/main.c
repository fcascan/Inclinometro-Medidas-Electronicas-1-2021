/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2020 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under BSD 3-Clause license,
  * the "License"; You may not use this file except in compliance with the
  * License. You may obtain a copy of the License at:
  *                        opensource.org/licenses/BSD-3-Clause
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "../planificador/miniplanificador.h"
#include "mpu_6050.h"
#include "ssd1306.h"
#include "task.h"
#include "eeprom.h"
#include "eeprom_21.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
#define dwt_init() 			{DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk; DWT->CYCCNT=0;}
#define dwt_reset() 		{DWT->CYCCNT=0;}
#define dwt_read() 			(DWT->CYCCNT)

// Activar tarea_iwdg
#define __SET_IWDG
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
I2C_HandleTypeDef hi2c1;
#ifdef __SET_IWDG
	IWDG_HandleTypeDef hiwdg;
#endif
/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
#ifdef __SET_IWDG
	static void MX_IWDG_Init(void);
#endif
static void MX_I2C1_Init(void);


/* USER CODE BEGIN PFP */
void MPU6050_Init(void);
void start_timer(void);
uint32_t stop_timer(void);
uint8_t antirebote (uint8_t lectura_actual);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/*
 * Variables Para pasar datos entre tareas.
 */
uint32_t ticks;
/*
 * Lista de tareas del planificador.
 */
TaskStat lista_tareas[MAX_LEN_TASK_LIST];

/*
 * Prototipos de las funciones que hacen al sistema.
 */
void falla_sistema(void);
/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */
	uint32_t tics_despachador;
	uint32_t wcet_todo = 0;

	/*
	 * Pongo en diez la variable que cuenta la cantidad de ticks que tienen
	 * que pasar entre llamadas del despachador.
	 */
	ticks = TICK_SISTEMA;

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
  dwt_init();
  MX_GPIO_Init();
  MX_I2C1_Init();

  /* USER CODE BEGIN 2 */
  MPU6050_Init();
  SSD1306_Init();
  tareas_Init();
  EEPROM_Init();
	#ifdef __SET_IWDG
		MX_IWDG_Init();
	#endif
	HAL_GPIO_WritePin(GPIOC, Led_Blink_Pin, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(GPIOB, outputLed_Pin, GPIO_PIN_RESET);

/*
	 * Uso el timer 2 para el monitor del sistema.
	 */
	RCC->APB1ENR |= RCC_APB1ENR_TIM2EN;
	/*
	 * Esta línea configura el prescaler del timer
	 * que cuenta tiempo del procesador. Prestar atención.
	 */
	TIM2->PSC = (SystemCoreClock / 1000000) - 1;
	TIM2->CNT = -1;
	TIM2->CR1 |= TIM_CR1_CEN;
	TIM2->CR1 &= ~TIM_CR1_CEN;
  /*
	 * Inicializo el despachador de las tareas. Le tengo que pasar
	 * la lista de tareas, dos punteros a función: uno para inicializar
	 * un timer y otro para que me devuelva la cuenta y lo frene. El
	 * puntero restante es un puntero a función que se llama cuando
	 * hay falla en la medición de tiempos de las funciones.
	 *
	 */
	inicializar_despachador(lista_tareas,
	MAX_LEN_TASK_LIST, start_timer, stop_timer, falla_sistema);

	/*
	* Agrego las tareas que voy a utilizar.
	* Le tengo que pasar:
	* TaskStat *lista = la lista de tareas
	* void (*tarea)(void *p) = Puntero a Funcion de la implementacion de la tarea
	* void *param = El puntero a parametro pasado a la tarea
	* int offset = Cantidad de ticks que dejo pasar desde el inicio hasta que arranca a ejecutar
	* int periodo = Cada cuantos ticks del sistema esta tarea debe ejecutarse. Arranco en 1
	* int bcet = Best Case Execution Time (Mejor tiempo de ejecucion logrado por la tarea) inicializo con cero y despues lo mido bien,
	* int wcet = Worst Case Execution Time (Peor tiempo de ejecucion logrado por la tarea) inicializo con el tick de sistema y despues lo mido bien)
	*/
	agregar_tarea(lista_tareas, tarea_iwdg, NULL, 0, 1, 0, 5);			// tiempos obtenidos:
	agregar_tarea(lista_tareas, tarea_led_blinking, NULL, 0, 10, 0, 5); // et_wcet = 3
	agregar_tarea(lista_tareas, tarea_orienta, NULL, 0, 1, 0, 1100);   	// et_wcet = 987, tenia 1100
	agregar_tarea(lista_tareas, tarea_refresh, NULL, 0, 1, 0, 4100);  	// et_wcet = 3790, tenia 4100
	agregar_tarea(lista_tareas, tarea_pulsadores, NULL, 0, 10, 0, 10);  // et_wcet = 6
	agregar_tarea(lista_tareas, tarea_modos, NULL, 0, 10, 0, 250000);  	// et_wcet = 49486, tenia 55000
	agregar_tarea(lista_tareas, tarea_display, NULL, 0, 1, 0, 100000); 	// et_wcet = 8781, tenia 9500

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
	while (1)
	{
	    /* USER CODE END WHILE */

	    /* USER CODE BEGIN 3 */
		dwt_reset();
		if (!ticks)
		{
			ticks = TICK_SISTEMA;
			despachar_tareas();
		}
		tics_despachador = dwt_read();
		if (tics_despachador > wcet_todo)
			wcet_todo = tics_despachador;
	}
  /* USER CODE END 3 */
}

/* USER CODE BEGIN 4 */
////////////////////////////
//	  TAREA WATCHDOG	  //
////////////////////////////
void tarea_iwdg(void *p)
{
#ifdef __SET_IWDG
	HAL_IWDG_Refresh(&hiwdg);
#endif
}

void falla_sistema(void)
{
	__disable_irq();
	while (1)
	{
		for (uint32_t i = 0; i < 100000; i++);
		HAL_GPIO_TogglePin(GPIOC, Led_Blink_Pin);
	}
}

void start_timer(void)
{
	TIM2->CNT = 0;
	TIM2->CR1 |= TIM_CR1_CEN;
}

uint32_t stop_timer(void)
{
	uint32_t ret = TIM2->CNT;
	TIM2->CR1 &= ~TIM_CR1_CEN;
	return ret;
}

/* USER CODE END 4 */

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Initializes the CPU, AHB and APB busses clocks 
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_LSI|RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.LSIState = RCC_LSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL7;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the CPU, AHB and APB busses clocks 
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief I2C1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C1_Init(void)
{

  /* USER CODE BEGIN I2C1_Init 0 */

  /* USER CODE END I2C1_Init 0 */

  /* USER CODE BEGIN I2C1_Init 1 */

  /* USER CODE END I2C1_Init 1 */
  hi2c1.Instance = I2C1;
  hi2c1.Init.ClockSpeed = 400000;
  hi2c1.Init.DutyCycle = I2C_DUTYCYCLE_2;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C1_Init 2 */

  /* USER CODE END I2C1_Init 2 */

}
#ifdef __SET_IWDG
/**
  * @brief IWDG Initialization Function
  * @param None
  * @retval None
  */
static void MX_IWDG_Init(void)
{

  /* USER CODE BEGIN IWDG_Init 0 */

  /* USER CODE END IWDG_Init 0 */

  /* USER CODE BEGIN IWDG_Init 1 */

  /* USER CODE END IWDG_Init 1 */
  hiwdg.Instance = IWDG;
  hiwdg.Init.Prescaler = IWDG_PRESCALER_4;
  hiwdg.Init.Reload = 1300;
  if (HAL_IWDG_Init(&hiwdg) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN IWDG_Init 2 */

  /* USER CODE END IWDG_Init 2 */

}
#endif

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(Led_Blink_GPIO_Port, Led_Blink_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(outputLed_GPIO_Port, outputLed_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : Led_Blink_Pin */
  GPIO_InitStruct.Pin = Led_Blink_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(Led_Blink_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : inputHold_Pin inputZero_Pin inputMode_Pin */
  GPIO_InitStruct.Pin = inputHold_Pin|inputZero_Pin|inputMode_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLDOWN;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : outputLed_Pin */
  GPIO_InitStruct.Pin = outputLed_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(outputLed_GPIO_Port, &GPIO_InitStruct);

}

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */

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
     tex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
