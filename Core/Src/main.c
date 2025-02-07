/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2025 STMicroelectronics.
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
#include "adc.h"
#include "can.h"
#include "dma.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdio.h>
#include <string.h>
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
uint16_t adc_counter = 0;
uint8_t data_trigger = 0;
uint16_t lv_counter = 0;

uint16_t heartbeat_counter = 0;
uint8_t heartbeat_state = 0;


uint16_t current = 0; 
uint16_t raw_voltage = 0;
uint16_t rawValeus [2];


/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */


void heartbeat(void); // Function prototype for sending the heartbeat trigger
void StartADC1(void); // Function prototype for starting the ADC
void VoltageMessure(uint16_t adc_valeu); // Function prototype for sending data over CAN bus

void PWMDutyCycleSet(uint16_t ChannelName, uint32_t dutyCycle); // Function prototype for setting the duty cycle of the PWM signal
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
void TIM3_IRQHandler(void)
{
  /* USER CODE BEGIN TIM3_IRQn 0 */

  /* USER CODE END TIM3_IRQn 0 */
  HAL_TIM_IRQHandler(&htim3);
  /* USER CODE BEGIN TIM3_IRQn 1 */
    heartbeat_counter++; // Increment the heartbeat counter
    adc_counter++; // Increment the ADC counter
    data_trigger ++; // Set the data trigger
    lv_counter ++; // Increment the LV counter

  /* USER CODE END TIM3_IRQn 1 */
}



void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef* hadc)
{
  for (uint8_t i = 0; i < hadc1.Init.NbrOfConversion; i++){
    current = rawValeus[0]; // Current value
    raw_voltage = rawValeus[1]; // Voltage value
}


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
  MX_ADC1_Init();
  MX_CAN_Init();
  MX_USART1_UART_Init();
  MX_TIM4_Init();
  MX_TIM3_Init();
  /* USER CODE BEGIN 2 */
  HAL_ADC_Start_DMA(&hadc1, (uint32_t*)rawValeus, 2); // Start the ADC in DMA mode

     if (HAL_TIM_Base_Start_IT(&htim3) != HAL_OK)
  {
    /* Starting Error */
    Error_Handler();
  }

  HAL_GPIO_WritePin(GPIOC, Led_Debug_3_Pin, GPIO_PIN_SET); // Turn on the LED

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */


        StartADC1(); // Start the ADC
        VoltageMessure(raw_voltage); // Send the voltage value over CAN bus 
        heartbeat(); // Send the heartbeat trigger



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
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;
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
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_ADC;
  PeriphClkInit.AdcClockSelection = RCC_ADCPCLK2_DIV6;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */


void heartbeat(void){

  if (heartbeat_state<= 2 && heartbeat_counter >= 2){ // 200 ms

      HAL_GPIO_TogglePin(GPIOC, Led_Debug_1_Pin); // Toggle the LED
      heartbeat_state ++;
      heartbeat_counter = 0;
    
  } else{
    if (heartbeat_counter >= 9){ // 900 ms

      HAL_GPIO_TogglePin(GPIOC, Led_Debug_1_Pin); // Toggle the LED
      heartbeat_state = 0;
      heartbeat_counter = 0;
    
      }    
    }
}




void StartADC1(void)
{
  if (adc_counter >= 1){ // 100 ms
    adc_counter = 0;
    HAL_ADC_Start_IT(&hadc1); // Start the ADC in interrupt mode
  }
}

void VoltageMessure(uint16_t raw_voltage){

    static uint16_t voltage_int = 0; 
    static uint16_t voltage_frac = 0;
    static uint16_t voltage_mv = 0;
    static uint8_t lv_state = 0;
    char msg[100]; // Message buffer

    voltage_int = raw_voltage * 28.5 / 4095; // Voltage value
    voltage_frac = (raw_voltage * 28500 / 4095) % 1000; // Voltage value
    voltage_mv = raw_voltage * 28500 / 4095; // Voltage value

    if (voltage_mv >= 20000){ // 28.5 V
      lv_state = 2;
    } else if (voltage_mv >= 10000){ // 27 V
      lv_state = 1;
    } else {
      lv_state = 0;
    }

    switch (lv_state){
    case 2:
        lv_counter = 0;
        HAL_GPIO_WritePin(GPIOC, Led_Debug_3_Pin, GPIO_PIN_SET); // Turn on the LED
        break;
      
    case 1:
      if (lv_counter >= 7){ // 500 ms
        lv_counter = 0;
        HAL_GPIO_TogglePin(GPIOC, Led_Debug_3_Pin); // Turn on the LED
      }
      break;

    case 0:
      if (lv_counter >= 2){ // 100 ms
        lv_counter = 0;
        HAL_GPIO_TogglePin(GPIOC, Led_Debug_3_Pin); // Turn off the LED
      } 
      break;

    default:
      lv_counter = 0;
      HAL_GPIO_WritePin(GPIOC, Led_Debug_3_Pin, GPIO_PIN_RESET); // Turn on the LED
      break;
    }


    if (data_trigger >= 10){ // 1000 ms
      data_trigger = 0;
      sprintf(msg, "Voltage: %d.%03d\n ",voltage_int,voltage_frac); // Print the current and voltage values
      HAL_UART_Transmit(&huart1, (uint8_t*)msg, strlen(msg), HAL_MAX_DELAY); // Transmit the message to the UART
    }


}

// Function to set the duty cycle of the PWM signal 

void PWMDutyCycleSet(uint16_t ChannelName, uint32_t dutyCycle)
{
  switch (ChannelName)
  {
  case 1:     // ChannelName: 1 (radiator_fan_control_Pin)
      {
    TIM4->CCR1 = dutyCycle;
    HAL_TIM_PWM_Start(&htim4, radiator_fan_control_Pin);
    break;
      }
  default:
    break;
  }

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
