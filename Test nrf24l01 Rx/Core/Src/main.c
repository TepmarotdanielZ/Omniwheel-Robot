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
#include "dma.h"
#include "spi.h"
#include "tim.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "math.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

// adress nrf24l01
uint8_t Rxaddr[5] = { 'O', 'M', 'N', 'I', '0' };
uint8_t RxData[8];
uint32_t lastTime = 0;

// Ecoder_M1
#define ENCODER_M_L1_Pin GPIO_PIN_4
#define ENCODER_M_R1_Pin GPIO_PIN_5
#define ENCODER_M_L1_GPIO_Port GPIOA
#define ENCODER_M_R1_GPIO_Port GPIOA
//Ecoder_M2
#define ENCODER_M_L2_Pin GPIO_PIN_2
#define ENCODER_M_R2_Pin GPIO_PIN_10
#define ENCODER_M_L2_GPIO_Port GPIOB
#define ENCODER_M_R2_GPIO_Port GPIOB
//Ecoder_M3
#define ENCODER_M_L3_Pin GPIO_PIN_11
#define ENCODER_M_R3_Pin GPIO_PIN_12
#define ENCODER_M_L3_GPIO_Port GPIOA
#define ENCODER_M_R3_GPIO_Port GPIOA
//Ecoder_M4
#define ENCODER_M_L4_Pin GPIO_PIN_14
#define ENCODER_M_R4_Pin GPIO_PIN_15
#define ENCODER_M_L4_GPIO_Port GPIOB
#define ENCODER_M_R4_GPIO_Port GPIOB
//M1
#define MOTOR_L1 TIM_CHANNEL_1
#define MOTOR_R1 TIM_CHANNEL_2
//M2
#define MOTOR_L2 TIM_CHANNEL_3
#define MOTOR_R2 TIM_CHANNEL_4
//M3
#define MOTOR_L3 TIM_CHANNEL_1
#define MOTOR_R3 TIM_CHANNEL_2
//M4
#define MOTOR_L4 TIM_CHANNEL_3
#define MOTOR_R4 TIM_CHANNEL_4

//RGB
#define NEOPIXEL_ZERO 32 //ZERO=(ARR+1)0.32
#define NEOPIXEL_ONE 64 // ONE=(ARR+1)0.64
#define NUM_PIXELS 32  //Number of LEDs  model WS2812b
#define DMA_BUFF_SIZE 769//(NUM_PIXELS*24)+1
//Ultrasonic
#define TRIG_PIN GPIO_PIN_9
#define TRIG_PORT GPIOA
#define ECHO_PIN GPIO_PIN_8
#define ECHO_PORT GPIOA
uint32_t pMillis;
uint32_t Value1 = 0;
uint32_t Value2 = 0;
uint16_t Distance = 0;  // cm

//speed wheels int
int Vx = 0;
int Vy = 0;
int Omega = 0;
float r = 1;
float V0;
float V1;
float V2;
float V3;

//PID speed output
int encoder1[4];
int encoder2[4];
int last_encoder1[4] = { 0 };
int last_encoder2[4] = { 0 };
long last_count[4] = { 0 };
int counter[4] = { 0 };
int count[4] = { 0 };
double current_time[4];
long last_time[4] = { 0 };
int T[4] = { 0 };
float RPM[4] = { 0 };
float RPS[4] = { 0 };
float count_second[4] = { 0 };
float last_time_PID[4];
float current_time_PID[4];
float eslapedtime[4];
//float Error[4];
float last_Error[4] = { 0 };
float pro[4]= { 0 };
float in[4] = { 0 };
float dev[4] = { 0 };
float setpoint[4] = { 200, 200, 200, 200 };
//float setpoint[4] = { 0 };
double error[4];
float Kp =11.2;  //11.2
float Kd =7.5;
float Ki =0.2;
double pid[4] = { 0 };
float pre_error[4]= { 0 };
float pre_in[4]={0};
float pwm_M[4] = { 0 };
float pre_pid[4]={0};
float pre_dev[4]={0};
//int n;
//int Ts=0.0001;
//int kk;
///* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */
double map(double x, double in_min, double in_max, double out_min,
		double out_max) {
	return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}


void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin) {
	if (GPIO_Pin == ENCODER_M_L1_Pin || ENCODER_M_R1_Pin) {
		encoder1[0] = HAL_GPIO_ReadPin(ENCODER_M_L1_GPIO_Port,ENCODER_M_L1_Pin);
		encoder2[0] = HAL_GPIO_ReadPin(ENCODER_M_R1_GPIO_Port,ENCODER_M_R1_Pin);
	}
	if (GPIO_Pin == ENCODER_M_L2_Pin || ENCODER_M_R2_Pin) {
		encoder1[1] = HAL_GPIO_ReadPin(ENCODER_M_L2_GPIO_Port,ENCODER_M_L2_Pin);
		encoder2[1] = HAL_GPIO_ReadPin(ENCODER_M_R2_GPIO_Port,ENCODER_M_R2_Pin);
	}
	if (GPIO_Pin == ENCODER_M_L3_Pin || ENCODER_M_R3_Pin) {
		encoder1[2] = HAL_GPIO_ReadPin(ENCODER_M_L3_GPIO_Port,ENCODER_M_L3_Pin);
		encoder2[2] = HAL_GPIO_ReadPin(ENCODER_M_R3_GPIO_Port,ENCODER_M_R3_Pin);
	}
	if (GPIO_Pin == ENCODER_M_L4_Pin || ENCODER_M_R4_Pin) {
		encoder1[3] = HAL_GPIO_ReadPin(ENCODER_M_L4_GPIO_Port,ENCODER_M_L4_Pin);
		encoder2[3] = HAL_GPIO_ReadPin(ENCODER_M_R4_GPIO_Port,ENCODER_M_R4_Pin);
	}
}

void Kinematic_Omni_Wheels() {

    V0 = ((1/r)*(0.7071*Vx+0.7071*Vy)+Omega);
    V1 = ((1/r)*(0.7071*Vx-0.7071*Vy)+Omega);
    V2 = ((1/r)*(-0.7071*Vx-0.7071*Vy)+Omega);
    V3 = ((1/r)*(-0.7071*Vx+0.7071*Vy)+Omega);

}

void Encoder_M() {
	for(int i=0; i < 4; i++){

		if(encoder1[i] != last_encoder1[i]){
			last_encoder1[i] = encoder1[i];
			if(encoder1[i] == 0){
				if(encoder2[i] == 1){

					counter[i]++;
				}else{
					counter[i]--;
				}
			}else{
				if(encoder2[i] == 0){
					counter[i]++;
				}else{
					counter[i]--;
				}
			}
		}
		if(encoder2[i] != last_encoder2[i]){
			last_encoder2[i] = encoder2[i];
			if(encoder2[i] == 1){
				if(encoder1[i] == 1){
					counter[i]++;
				}else{
					counter[i]--;
				}
			}else{
				if(encoder1[i] == 0){
					counter[i]++;
				}else{
					counter[i]--;
				}
			}
		}
	}
	count[0] = counter[0];
	count[1] = counter[1];
	count[2] = counter[2];
	count[3] = counter[3];
}

void PID_Speed_Output() {

	//    sampling time
	for (int i = 0; i < 4; i++) {
		T[i] = HAL_GetTick() - last_time[i];
		if (T[i] >= 10) {

			setpoint[0] = V0;
			setpoint[1] = V1;
			setpoint[2] = V2;
			setpoint[3] = V3;

			count_second[i] = count[i] - last_count[i];
			RPS[i] = (count_second[i] * 100) / 1600;
			RPM[i] = (RPS[i] * 60);
			last_count[i] = count[i];
			last_time[i] = HAL_GetTick();
//
			current_time_PID[i] = HAL_GetTick();
			eslapedtime[i] = current_time_PID[i] - last_time_PID[i];
			error[i] = setpoint[i] - RPM[i];
			in[i] = in[i] + (error[i] * eslapedtime[i]);
			dev[i] = (error[i] - last_Error[i]) / eslapedtime[i];

			pid[i] = Kp * error[i] + Ki * in[i] + Kd * dev[i];

			// constrain value
			if (pid[i] < -850) {
				pwm_M[i] = -850;
			}
			if (pid[i] > 850) {
				pwm_M[i] = 850;
			}
			if (-850 <= pid[i] && pid[i] <= 850) {
				pwm_M[i] = pid[i];
			}
			//////////////////////
			last_Error[i] = error[i];
			last_time_PID[i] = current_time_PID[i];

//	             error[i]=setpoint[i]-RPM[i]; //error
//	             pro[i]=Kp*error[i]; //pro
//
//	             in[i]=((Ts*Ki)/2)*(error[i]+pre_error[i])+(in[i]);
//
//	             dev[i]=((2*Kd)/Ts)*(error[i]-pre_error[i])-dev[i];
//
//	             pid[i]=(pro[i]+in[i]+dev[i]);
//
//	             if (pid[i]<-700){
//	            	 pwm_M[i]=-700;
//	             }
//	               if(pid[i]>700){
//	            	   pwm_M[i]=700;
//	               }
//	             if(-700 <= pid[i] && pid[i] <= 700){
//	            	 pwm_M[i] = pid[i];
//	               }
//	             pre_error[i]=error[i];
//	             pre_in[i]=in[i];
////	             pre_dev[i]=dev[i];
		}
	}

}

void Motor_Speed() {

	//M1
	if (pwm_M[0] > 0) {
		__HAL_TIM_SET_COMPARE(&htim2, MOTOR_L1, pwm_M[0]);
		__HAL_TIM_SET_COMPARE(&htim2, MOTOR_R1, 0);
	}
	if (pwm_M[0] < 0) {
		__HAL_TIM_SET_COMPARE(&htim2, MOTOR_L1, 0);
		__HAL_TIM_SET_COMPARE(&htim2, MOTOR_R1, (-1) * pwm_M[0]);
	}
	if (pwm_M[0] == 0) {
		__HAL_TIM_SET_COMPARE(&htim2, MOTOR_L1, 0);
		__HAL_TIM_SET_COMPARE(&htim2, MOTOR_R1, 0);
	}
	  //M2
	if (pwm_M[1] > 0) {
		__HAL_TIM_SET_COMPARE(&htim2, MOTOR_L2, pwm_M[1]);
		__HAL_TIM_SET_COMPARE(&htim2, MOTOR_R2, 0);
	}
	if (pwm_M[1] < 0) {
		__HAL_TIM_SET_COMPARE(&htim2, MOTOR_L2, 0);
		__HAL_TIM_SET_COMPARE(&htim2, MOTOR_R2, (-1) * pwm_M[1]);
	}
	if (pwm_M[1] == 0) {
		__HAL_TIM_SET_COMPARE(&htim2, MOTOR_L2, 0);
		__HAL_TIM_SET_COMPARE(&htim2, MOTOR_R2, 0);
	}
//	M3
	if (pwm_M[2] > 0) {
		__HAL_TIM_SET_COMPARE(&htim3, MOTOR_L3, pwm_M[2]);
		__HAL_TIM_SET_COMPARE(&htim3, MOTOR_R3, 0);
	}
	if (pwm_M[2] < 0) {
		__HAL_TIM_SET_COMPARE(&htim3, MOTOR_L3, 0);
		__HAL_TIM_SET_COMPARE(&htim3, MOTOR_R3, (-1) * pwm_M[2]);
	}
	if (pwm_M[2] == 0) {
		__HAL_TIM_SET_COMPARE(&htim3, MOTOR_L3, 0);
		__HAL_TIM_SET_COMPARE(&htim3, MOTOR_R3, 0);
	}
//	//M4
	if (pwm_M[3] > 0) {
		__HAL_TIM_SET_COMPARE(&htim3, MOTOR_L4, pwm_M[3]);
		__HAL_TIM_SET_COMPARE(&htim3, MOTOR_R4, 0);
	}
	if (pwm_M[3] < 0) {
		__HAL_TIM_SET_COMPARE(&htim3, MOTOR_L4, 0);
		__HAL_TIM_SET_COMPARE(&htim3, MOTOR_R4, (-1) * pwm_M[3]);
	}
	if (pwm_M[3] == 0) {
		__HAL_TIM_SET_COMPARE(&htim3, MOTOR_L4, 0);
		__HAL_TIM_SET_COMPARE(&htim3, MOTOR_R4, 0);
	}

}


void reset_data(void) {
	RxData[0] = 128;    //x1
	RxData[1] = 128;    //y1
	RxData[2] = 128;    //x2
	RxData[3] = 128; //y2
	RxData[4] = 128; //y2
	RxData[5] = 128; //y2
	RxData[6] = 0; //y2
	RxData[7] = 0; //y2
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
  MX_SPI1_Init();
  MX_TIM2_Init();
  MX_TIM3_Init();
  MX_TIM4_Init();
  MX_TIM1_Init();
  /* USER CODE BEGIN 2 */
	//PWM Motor
	HAL_TIM_PWM_Start(&htim2, MOTOR_L1);
	HAL_TIM_PWM_Start(&htim2, MOTOR_R1);
	HAL_TIM_PWM_Start(&htim2, MOTOR_L2);
	HAL_TIM_PWM_Start(&htim2, MOTOR_R2);

	HAL_TIM_PWM_Start(&htim3, MOTOR_L3);
	HAL_TIM_PWM_Start(&htim3, MOTOR_R3);
	HAL_TIM_PWM_Start(&htim3, MOTOR_L4);
	HAL_TIM_PWM_Start(&htim3, MOTOR_R4);
	//PWM Ultrasonic
	HAL_TIM_Base_Start(&htim1);
	HAL_GPIO_WritePin(TRIG_PORT, TRIG_PIN, GPIO_PIN_RESET); // pull the TRIG pin low

	NRF24_Init();
	NRF24_RxMode(Rxaddr, 112, 8); //NRF24_TxMode(Address, channel, payload_size)
	RxData[0] = 128;  //x1
	RxData[1] = 128;  //y1
	RxData[2] = 128;  //x2
	RxData[3] = 128; //y2
	RxData[4] = 128; //y2
	RxData[5] = 128; //y2
	RxData[6] = 0; //y2
	RxData[7] = 0; //y2
//	RGB_Setup();

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
	while (1) {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */



		if (DataReady(1) > 0) {
			NRF24_Receive(RxData, 8);
			lastTime = HAL_GetTick();
		}
		if ((HAL_GetTick() - lastTime > 10)) {
			reset_data();
		}

		Vx = map(RxData[0], 0, 256, -300, 300);
		Vy = map(RxData[1], 0, 256, -300, 300);
		Omega = map(RxData[2], 0, 256, -300, 300);

//
		Kinematic_Omni_Wheels();
		Encoder_M();
		PID_Speed_Output();
		Motor_Speed();

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
  RCC_OscInitStruct.PLL.PLLM = 15;
  RCC_OscInitStruct.PLL.PLLN = 96;
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
	while (1) {
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
