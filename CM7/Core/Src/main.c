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
#include "i2c.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "bmp2.h"
#include "pid.h"
#include <stdint.h>
#include <stdbool.h>
#include <stdio.h>
#include <math.h>
#include "lcd_i2c.h"

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

#ifndef HSEM_ID_0
#define HSEM_ID_0 (0U) /* HW semaphore 0*/
#endif

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
  BMP280_CalibrationData calib_data;


  char uart_msg[50];
  float temperature;
  float liczba_pom=0.0;
  char wejscie[3];
  char pomoc[2];
  float wartosc =28.0f;
  float wartosc_spr =0.0f;
  float kp=8000.0;
  float ki=1500.0;
  float kd=100.0;
  pid_str pid_controller;
  char uchyb[50];
  char sygnal[50];
  float pid_output=0.0;
  float scaled_output=0.0;
  int final_output=0;
  int licznik = 0;
  struct lcd_disp disp;
  int min_pid = 0;
  int max_pid = 1000;
  float min_zak = 20.0f;
  float max_zak = 40.0f;
  float wartosc_enk = 0.0f;

  //float sigmoid(float x) {return 2.0 * (1.0 / (1.0 + exp(-0.5*x)) - 0.5);}

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
/* USER CODE BEGIN Boot_Mode_Sequence_0 */
  int32_t timeout;
/* USER CODE END Boot_Mode_Sequence_0 */

/* USER CODE BEGIN Boot_Mode_Sequence_1 */
  /* Wait until CPU2 boots and enters in stop mode or timeout*/
  timeout = 0xFFFF;
  while((__HAL_RCC_GET_FLAG(RCC_FLAG_D2CKRDY) != RESET) && (timeout-- > 0));
  if ( timeout < 0 )
  {
  Error_Handler();
  }
/* USER CODE END Boot_Mode_Sequence_1 */
  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();
/* USER CODE BEGIN Boot_Mode_Sequence_2 */
/* When system initialization is finished, Cortex-M7 will release Cortex-M4 by means of
HSEM notification */
/*HW semaphore Clock enable*/
__HAL_RCC_HSEM_CLK_ENABLE();
/*Take HSEM */
HAL_HSEM_FastTake(HSEM_ID_0);
/*Release HSEM in order to notify the CPU2(CM4)*/
HAL_HSEM_Release(HSEM_ID_0,0);
/* wait until CPU2 wakes up from stop mode */
timeout = 0xFFFF;
while((__HAL_RCC_GET_FLAG(RCC_FLAG_D2CKRDY) == RESET) && (timeout-- > 0));
if ( timeout < 0 )
{
Error_Handler();
}
/* USER CODE END Boot_Mode_Sequence_2 */

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_I2C1_Init();
  MX_USART3_UART_Init();
  MX_TIM2_Init();
  MX_TIM3_Init();
  MX_I2C2_Init();
  MX_TIM8_Init();
  /* USER CODE BEGIN 2 */
  BMP280_Init(&hi2c1);


  BMP280_ReadCalibrationData(&hi2c1, &calib_data);
  HAL_TIM_PWM_Stop(&htim3, TIM_CHANNEL_3);
  __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_3, 0);
  	  htim3.Init.Period = 999;
  HAL_TIM_Base_Init(&htim3);
  HAL_TIM_Base_Start(&htim3);
  HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_3);

  pid_init(&pid_controller,  kp,  ki,  kd,5000);

  	  disp.addr = (0x27 << 1);
    disp.bl = true;
    lcd_init(&disp);


    HAL_TIM_Encoder_Start(&htim8 , TIM_CHANNEL_ALL) ;
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  HAL_TIM_Base_Start_IT(&htim2);
  HAL_UART_Receive_IT(&huart3, wejscie, 3);



  while (1)
  {
	  uint32_t enkoder = __HAL_TIM_GET_COUNTER(&htim8);

	  __HAL_TIM_SET_COUNTER(&htim8 , enkoder );
	  wartosc_enk =(float) (enkoder/50+20);
	  if(HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_0) == GPIO_PIN_SET)
	  {
	  char zmienna [20] ;
	  snprintf(zmienna, sizeof(zmienna), "%.2f",wartosc_enk);
	  HAL_UART_Transmit(&huart3 ,"Licznik:", strlen("Licznik:") ,1000 );
	  HAL_UART_Transmit(&huart3 ,( uint8_t*) zmienna, strlen(zmienna) ,1000 );
	  HAL_UART_Transmit(&huart3 ,( uint8_t*) "\r\n", strlen( "\r\n" ), 1000 );
	  wartosc = wartosc_enk;
	  }


 	 char sygnal[50];
 	 snprintf(sygnal, sizeof(sygnal), "Sygnal: %d \r\n", final_output);
 	 char tempe[50];
 	 snprintf(tempe, sizeof(tempe), "Temp: %.2f C\r\n", temperature);
	     sprintf((char *)disp.f_line, sygnal);
	     sprintf((char *)disp.s_line, tempe);
	     lcd_display(&disp);
	     HAL_Delay(500);



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

  /** Supply configuration update enable
  */
  HAL_PWREx_ConfigSupply(PWR_DIRECT_SMPS_SUPPLY);

  /** Configure the main internal regulator output voltage
  */
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE3);

  while(!__HAL_PWR_GET_FLAG(PWR_FLAG_VOSRDY)) {}

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_DIV1;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2
                              |RCC_CLOCKTYPE_D3PCLK1|RCC_CLOCKTYPE_D1PCLK1;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI;
  RCC_ClkInitStruct.SYSCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB3CLKDivider = RCC_APB3_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_APB1_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_APB2_DIV1;
  RCC_ClkInitStruct.APB4CLKDivider = RCC_APB4_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_1) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
	if (GPIO_Pin == GPIO_PIN_3)  // Sprawdzenie, czy to przycisk na PA3
	    {
	        if (HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_0) == GPIO_PIN_RESET)
	        {
	            HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0, GPIO_PIN_SET);  // Włącz diodę
	        }
	        else
	        {
	            HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0, GPIO_PIN_RESET); // Wyłącz diodę
	        }
	    }
}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  if (htim->Instance == TIM2)
  {
    // Timer interrupt triggered, perform UART transmission here
    temperature = BMP280_ReadTemperature(&hi2c1, &calib_data);
    pid_output = pid_calculate(&pid_controller, wartosc, temperature);
    licznik = licznik + 1;

    scaled_output = ((pid_output - min_pid) / (float)(max_pid - min_pid))*1000.0;


     if (scaled_output < 0.0) {
         final_output = 0;
     } else if (scaled_output > 1000.0) {
         final_output = 1000;
     } else {
         final_output = (int)scaled_output;  // rzutowanie na int
     }

           // Ustawienie wypełnienia PWM
     __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_3, final_output);
     if (licznik==50)
     {
    	 char msg[50];
    	 snprintf(msg, sizeof(msg), "Wypełnienie: %d \r\n", final_output);

    	 HAL_UART_Transmit(&huart3, (uint8_t *)msg, strlen(msg), HAL_MAX_DELAY);
    	 char uart_msg[50];
    	 snprintf(uart_msg, sizeof(uart_msg), "Temperature: %.2f C\r\n", temperature);

    	 HAL_UART_Transmit(&huart3, (uint8_t *)uart_msg, strlen(uart_msg), HAL_MAX_DELAY);
    	 licznik=0;
     }


  }
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
    if (huart->Instance == USART3) {
    	if(HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_0) == GPIO_PIN_RESET){

        if (strncmp("B", (char*)wejscie, 1) == 0) {
        	 	pomoc[0] = wejscie[1];
        	    pomoc[1] = wejscie[2];


        	wartosc_spr =(float) atoi(pomoc);

        	if (wartosc_spr>40 || wartosc_spr<20){
        		HAL_UART_Transmit(&huart3, "WARTOSC POZA ZAKRESEM BEZPIECZNYM 20-40 !!!", strlen("WARTOSC POZA ZAKRESEM BEZPIECZNYM 20-40 !!!"), 10);
        	}
        	else{
        			wartosc = wartosc_spr;
        			pid_reset(&pid_controller);
        		 // Wysyłamy wynik przez UART
        		 char transmit_msg[10];
        		 snprintf(transmit_msg, sizeof(transmit_msg), "%.1f", wartosc);

        		 HAL_UART_Transmit(&huart3 ,"ZADANE:", strlen("ZADANE:") ,1000 );
        		 HAL_UART_Transmit(&huart3, (uint8_t*)transmit_msg, strlen(transmit_msg), 10);
        		 HAL_UART_Transmit(&huart3 ,( uint8_t*) "\r\n", strlen( "\r\n" ), 1000 );
        		 memset(wejscie, ' ', 3);

        	}
        }
    	}
        HAL_UART_Receive_IT(&huart3, (uint8_t*)wejscie, 3);
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
