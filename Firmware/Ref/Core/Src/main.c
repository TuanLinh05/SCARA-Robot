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
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "scara.h"
//#include "Command_Line.h"
#include "Command_Line_Type.h"
#include "Ring_Buffer.h"
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
Scara_Theta Theta1;
Scara_Theta Theta2;
Scara_Theta Theta3;
uint8_t rxdata;
Packed_cmd_t Packed_cmd_theta1[MAX_LENGTH_RING_BUFFER];
Packed_cmd_t Packed_cmd_theta2[MAX_LENGTH_RING_BUFFER];
Packed_cmd_t Packed_cmd_theta3[MAX_LENGTH_RING_BUFFER];
RING_BUFFER_HandleTypeDef ring_buffer_theta1;
RING_BUFFER_HandleTypeDef ring_buffer_theta2;
RING_BUFFER_HandleTypeDef ring_buffer_theta3;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

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
  HAL_Init();

  /* USER CODE BEGIN Init */
  Init_Theta_Joint(&Theta1, &htim1, TIM_CHANNEL_1, GPIOB, GPIO_PIN_15, JOINT_REVOLUTE_3);
  Init_Theta_Joint(&Theta2, &htim2, TIM_CHANNEL_1, GPIOA, GPIO_PIN_1, JOINT_REVOLUTE_9);
  Init_Theta_Joint(&Theta3, &htim3, TIM_CHANNEL_1, GPIOA, GPIO_PIN_7, JOINT_PRISMATIC);
  RING_BUFFER_Init(&ring_buffer_theta1, Packed_cmd_theta1, MAX_LENGTH_RING_BUFFER);
  RING_BUFFER_Init(&ring_buffer_theta2, Packed_cmd_theta2, MAX_LENGTH_RING_BUFFER);
  RING_BUFFER_Init(&ring_buffer_theta3, Packed_cmd_theta3, MAX_LENGTH_RING_BUFFER);
  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_TIM1_Init();
  MX_TIM2_Init();
  MX_TIM3_Init();
  MX_USART2_UART_Init();
  /* USER CODE BEGIN 2 */
//  HAL_TIM_Base_Start_IT(&htim1);
//  Trapezoidal_Profile(0,540,&(Theta1.Vel),Theta1.Type);
//  HAL_TIM_PWM_Start_IT(Theta1.Joint.p_Motor_TIM, Theta1.Joint.Motor_Chanel);
  HAL_UART_Receive_IT(&huart2, &rxdata, 1);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
//	  HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_13);
//	  HAL_Delay(5000);
	  COMMAND_LINE_Handle();
	  Cmd_control_handle(&ring_buffer_theta1,&Theta1);
	  Cmd_control_handle(&ring_buffer_theta2,&Theta2);
	  Cmd_control_handle(&ring_buffer_theta3,&Theta3);
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

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL8;
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
}

/* USER CODE BEGIN 4 */
void HAL_TIM_PWM_PulseFinishedCallback(TIM_HandleTypeDef *htim){
	if(htim->Instance == Theta1.Joint.p_Motor_TIM->Instance){
//		if(Theta1.cnt_pulse < Theta1.Vel.Acc_step_index){
//			Set_Joint_Speed(&Theta1, Theta1.Vel.ARR_val);
//			Theta1.Vel.ARR_val--;
//		}else if(Theta1.cnt_pulse > Theta1.Vel.De_step_index){
//			Set_Joint_Speed(&Theta1, Theta1.Vel.ARR_val);
//			Theta1.Vel.ARR_val++;
//		}
//		Theta1.cnt_pulse++;
//		if(Theta1.cnt_pulse == Theta1.Vel.N_step) {
//			HAL_TIM_PWM_Stop_IT(Theta1.Joint.p_Motor_TIM, Theta1.Joint.Motor_Chanel);
//			return;//reach target
//		}
		Handle_Theta(&Theta1);
	}else if(htim->Instance == Theta2.Joint.p_Motor_TIM->Instance){
		Handle_Theta(&Theta2);
	}else if(htim->Instance == Theta3.Joint.p_Motor_TIM->Instance){
		Handle_Theta(&Theta3);
	}
}
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart){
	if(huart->Instance == huart2.Instance){
		COMMAND_LINE_Receive(rxdata);
		HAL_UART_Receive_IT(&huart2, &rxdata, 1);
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
