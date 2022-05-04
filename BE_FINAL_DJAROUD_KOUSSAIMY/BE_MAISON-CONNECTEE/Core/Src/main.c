/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2022 STMicroelectronics.
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
#include "i2c.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <string.h>
#include "lib_lcd.h"
#include "caracter.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
static rgb_lcd lcdData;
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
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */


uint8_t buf[12] ;
/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */
	 uint16_t readValue;
	  char msg[10];
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
  MX_USART2_UART_Init();
  MX_ADC_Init();
  MX_I2C1_Init();
  /* USER CODE BEGIN 2 */




    lcd_init(&hi2c1, &lcdData); // initialise le lcd
    lcd_position(&hi2c1,0,0);
    lcd_print(&hi2c1,"LUMINOSITE");
    reglagecouleur(50,50,9);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
	      HAL_ADC_Start(&hadc);
	  	  HAL_ADC_PollForConversion(&hadc, HAL_MAX_DELAY);
	  	  readValue = HAL_ADC_GetValue(&hadc);
	  	  sprintf(msg, "%hu\r\n", readValue);
	  	  HAL_UART_Transmit(&huart2, (uint8_t*)msg, strlen(msg), HAL_MAX_DELAY);



	  	 HAL_Delay(500);
	  	char res [10];

	  	if (readValue>3000)
	  	{
	  		 lcd_position(&hi2c1,2,1);
	  		 lcd_print(&hi2c1," IL FAIT JOUR");
	  	}else
	  	    {
	  		   lcd_position(&hi2c1,2,1);
	  	       lcd_print(&hi2c1,"IL FAIT NUIT\n");
	    	}

	  		/*  ftoat (readValue,res,1);
	  		  lcd_position(&hi2c1,8,1);
	  		  lcd_print(&hi2c1,res); */




    /* USER CODE END WHILE */
	  	strcpy((char*)buf,"NONE\r\n");

	  		    HAL_UART_Transmit(&huart2,buf,strlen((char*)buf), HAL_MAX_DELAY) ;


	  		  	    if (HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_5) && readValue<3000) // PIN HIGH

	  		  	    {     strcpy((char*)buf, "MOTION\r\n");

	  		  	    	HAL_UART_Transmit(&huart2,buf,strlen((char*)buf), HAL_MAX_DELAY) ;
	  		  	// HAL_GPIO_WritePin (GPIOB, GPIO_PIN_15, 1); // LED ON
	  		  	    	HAL_GPIO_WritePin(GPIOA, LD2_Pin|LED_ROUGE_Pin, 1);
	  		  	        HAL_Delay (1000);
	  		  	        HAL_GPIO_WritePin(GPIOA, LD2_Pin|LED_ROUGE_Pin, 0);

	  		  	    // HAL_GPIO_WritePin (GPIOB, GPIO_PIN_15, 0); // LED OFF
	  		  	    // while (HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_5)); // Wait for PB5 to go low  ' 0v '
	  		  	    }



    /* USER CODE BEGIN 3 */
	  	//HAL_GPIO_TogglePin (GPIOA, GPIO_PIN_1);
	    //HAL_Delay (100); /* Insérer un délai 100 ms */
	/*   HAL_GPIO_WritePin(GPIOA, LED_ROUGE_Pin|LD2_Pin, GPIO_PIN_RESET);
	  		     //HAL_Delay (100);
	  	  if (readValue < 2980)
	  	  {
	     HAL_GPIO_WritePin(GPIOA, LED_ROUGE_Pin|LD2_Pin, GPIO_PIN_SET);
	     HAL_Delay (1000);
	     //HAL_GPIO_WritePin(GPIOA, LED_ROUGE_Pin|LD2_Pin, GPIO_PIN_RESET);
	     //HAL_Delay (100);
	  	  }  */
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
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);
  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL6;
  RCC_OscInitStruct.PLL.PLLDIV = RCC_PLL_DIV3;
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

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_1) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */

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

