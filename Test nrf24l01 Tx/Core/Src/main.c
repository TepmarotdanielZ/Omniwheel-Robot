/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2023 STMicroelectronics.
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
#include "dma.h"
#include "spi.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
uint8_t Rxaddr[5] = { 'O', 'M', 'N', 'I', '0' };
uint8_t TxData[8];
uint32_t lastTime = 0;
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
int ADC_Read[2]={0};
int Joystick_y=0;
int Joystick_x=0;
int Switch=0;
int Omega=0;
int State_button=0;
int current_time=0;
int last_time=0;
int Button=0;
//int Vx=0;
//int Vy=0;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */
double map(double x,double in_min,double in_max,double out_min,double out_max)
{
  return (x-in_min)*(out_max-out_min)/(in_max-in_min)+out_min;
}

void nrf24l01_Master(void){
 TxData[0] = Joystick_x;//x1
 TxData[1] = Joystick_y;//y1
 TxData[2] = Omega;//x2
 TxData[3] = 4; //y2
 TxData[4] = 5; //y2
 TxData[5] = 6; //y2
 TxData[6] = 7; //y2
 TxData[7] = 8; //y2
     NRF24_Transmit(TxData,8);
      HAL_Delay(5);
}
void Joystick(){


      if (ADC_Read[0] >= 0 && ADC_Read[0] <= 120) {
           Joystick_x = map(ADC_Read[0], 0, 120, 0, 128);
        }
      if (ADC_Read[0] >= 130 && ADC_Read[0] <= 255) {
           Joystick_x = map(ADC_Read[0], 130, 255, 128, 255);
        }
      if (ADC_Read[0] > 120 && ADC_Read[0] < 130){
           Joystick_x = 128;
        }

      if (ADC_Read[1] >= 0 && ADC_Read[1] <= 120) {
           Joystick_y = map(ADC_Read[1], 0, 120, 0, 128);
        }
      if (ADC_Read[1] >= 130 && ADC_Read[1] <= 255) {
           Joystick_y = map(ADC_Read[1], 130, 255, 128, 255);
        }
      if (ADC_Read[1] > 120 && ADC_Read[1] < 130){
           Joystick_y = 128;
        }

      if (Switch == 0) {
    	  Joystick_y = Joystick_y;
               Omega = 128;
        }
      else {
               Omega = Joystick_y;
          Joystick_y = 128;
      }

    }


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

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_ADC1_Init();
  MX_SPI1_Init();
  /* USER CODE BEGIN 2 */
  HAL_ADC_Start_DMA(&hadc1, &ADC_Read,2);

  NRF24_Init();
  NRF24_TxMode(Rxaddr, 112, 8);//NRF24_TxMode(Address, channel, payload_size)

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
	  Button = HAL_GPIO_ReadPin(Button_GPIO_Port, Button_Pin);
	  if (Button == 0 && State_button == 0){
	  State_button=1;
	  last_time = HAL_GetTick();
	  }
	  if (Button == 1 && State_button == 1){
	  State_button = 0;
	  current_time = HAL_GetTick() ;
	  }
	  if (current_time - last_time > 20){
	  last_time = current_time;
	  Switch =! Switch;
	  }
	  Joystick();
	  nrf24l01_Master();
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
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE2);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 8;
  RCC_OscInitStruct.PLL.PLLN = 84;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 4;
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
