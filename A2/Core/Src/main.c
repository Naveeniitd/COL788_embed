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

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
	#include <stdbool.h>
	#include "filter.h"
	#include <stdio.h>
	#include <string.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
	I2C_HandleTypeDef hi2c1;
	UART_HandleTypeDef huart2;

	/* Constants */
	const float rThreshold = 0.7;

	const float kLowPassCutoff = 5.0;   // 5 hz
	const float kHighPassCutoff = 0.5;  // 0.5 hz
	const float kSamplingFrequency = 400.0;
	const float kEdgeThreshold = -2000.0;

	const unsigned long kFingerThreshold = 10000;
	const unsigned int kFingerCooldownMs = 500;




	const float decayRate = 0.02;
	const float thrRate = 0.05;
	const int minDiff = 50;
	const uint32_t fingerThreshold = 10000;  // threshold for detecting a finger on the sensor

	float maxValue = 0;
	float minValue = 0;
	float threshold = 0;
	long lastHeartbeat = 0;
	float lastValue = 0;
	int fingerDetected = 0;

	float last_diff = NAN;
	bool crossed = false;
	long crossed_time = 0;
	/* variables */
	LowPassFilter low_pass_filter_red;
	LowPassFilter low_pass_filter_ir;
	HighPassFilter high_pass_filter;
	LowPassFilter low_pass_filter;
	Differentiator differentiator;
	MovingAverageFilter moving_avg_filter;
	MovingAverageFilter moving_avg_bpm;
	MovingAverageFilter moving_avg_spo2;
	/* statistic Variables */
	MinMaxAvgStatistic stat_red;
	MinMaxAvgStatistic stat_ir;


	/* finger Detection */
	long finger_timestamp = 0;
	bool finger_detected = false;


	void initialize_filters() {
		HighPassFilter_Init(&high_pass_filter, kHighPassCutoff, kSamplingFrequency);
		LowPassFilter_Init(&low_pass_filter_red, kLowPassCutoff, kSamplingFrequency);
		LowPassFilter_Init(&low_pass_filter_ir, kLowPassCutoff, kSamplingFrequency);
		Differentiator_Init(&differentiator, kSamplingFrequency);
		MovingAverageFilter_Init(&moving_avg_filter);
		MovingAverageFilter_Init(&moving_avg_bpm);
		MovingAverageFilter_Init(&moving_avg_spo2);
	}


	/* SpO2 Calibration Coefficients */
	float kSpO2_A = 1.5958422;
	float kSpO2_B = -34.6596622;
	float kSpO2_C = 112.6898759;

	/* Pulse Oximetry SpO2 Calculation */
	float calculate_SpO2(float rred, float rir) {
		float r = rred / rir;
		float spo2 = kSpO2_A * r * r + kSpO2_B * r + kSpO2_C;
		return spo2;
	}
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
I2C_HandleTypeDef hi2c1;

UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */
	#define MAX30102_ADDRESS  0x57 // I2C address ==== of MAX30102


	void Transmit_UART(const char *msg) {
			HAL_UART_Transmit(&huart2, (uint8_t*)msg, strlen(msg), HAL_MAX_DELAY);
		}

	void MAX30102_Init(void) {
		uint8_t configData[2];


//		0,1 == adress, value
		configData[0] = 0x09;  // select MODE_CONFIG register
		configData[1] = 0x03;  // set SpO2 mode (0x03== SpO2, 0x02 == heart rate only)
//		hi2cl handler for i2c
//		address shifterd by 1 only 7 bit sent (required)
//		2 indicate only 2 bytes to send

		HAL_I2C_Master_Transmit(&hi2c1, MAX30102_ADDRESS << 1, configData, 2, HAL_MAX_DELAY);

		// Set the LED pulse amplitude (this is an example, values need to be set properly)
		configData[0] = 0x0C;  // LED1_PULSE_AMPLITUDE ka register
		configData[1] = 0x1F;  // set the pulse amplitude for LED1 (Red LED we can see naked eyes)
		//		hi2cl handler for i2c
		//		adreess shifterd by 1 only 7 bit sent (required)
		//		2 indicate only 2 bytes to send

		HAL_I2C_Master_Transmit(&hi2c1, MAX30102_ADDRESS << 1, configData, 2, HAL_MAX_DELAY);

		// set LED2 (IR LED) pulse Amplitude
		configData[0] = 0x0D;  //register of ir led
		configData[1] = 0x1F;  // infra red LED
		//		hi2cl handler for i2c
		//		adreess shifterd by 1 only 7 bit sent (required)
		//		2 indicate only 2 bytes to send
		//		HAL_MAX_DELAY allows operation to block until successful trastmittion
		HAL_I2C_Master_Transmit(&hi2c1, MAX30102_ADDRESS << 1, configData, 2, HAL_MAX_DELAY);


	}


	void MAX30102_ReadRawData(uint32_t* redLED, uint32_t* irLED) {
		uint8_t rawData[6];  // 6 byte array as MAX30102 outputs 3 bytes for each LED data
		char uartBuf[50];
		// reading from FIFO register of MAX30102
		if (HAL_I2C_Mem_Read(&hi2c1, MAX30102_ADDRESS << 1, 0x07, I2C_MEMADD_SIZE_8BIT, rawData, 6, HAL_MAX_DELAY) != HAL_OK) {
			sprintf(uartBuf, "I2C read error\r\n");
			HAL_UART_Transmit(&huart2, (uint8_t*)uartBuf, strlen(uartBuf), HAL_MAX_DELAY);
			Error_Handler();  //  keep printing this message === so  avoid halting the program
		}

		// combine 3 bytes for each LED (Red and IR)
		*redLED = ((uint32_t)rawData[0] << 16) | ((uint32_t)rawData[1] << 8) | rawData[2];
		*irLED = ((uint32_t)rawData[3] << 16) | ((uint32_t)rawData[4] << 8) | rawData[5];
	}






	// register Addresses
//	#define MAX30105_INTSTAT1      0x00  // Interrupt Status 1
	#define MAX30105_INTSTAT2      0x01  // Interrupt Status 2 stat of interupt  temp 1
//	#define MAX30105_INTENABLE1    0x02  // Interrupt Enable 1
	#define MAX30105_INTENABLE2    0x03  // Interrupt Enable 2 INT_DIE_TEMP_RDY_ENABLE
	#define MAX30105_DIETEMPCONFIG 0x21  // Die Temperature Config
	#define MAX30105_TEMP_INTEGER  0x1F  // Temperature Integer Part
	#define MAX30105_TEMP_FRACTION 0x20  // Temperature Fractional Part


	#define MAX30105_INT_DIE_TEMP_RDY_ENABLE 0x02  // Bit 1



	void CheckAndClearInterruptStatus(void) {
	    uint8_t intStatus = 0;  // Variable to store the interrupt status
	    char uartBuf[50];

	    // Read the Interrupt Status 2 register (0x01)
	    HAL_StatusTypeDef status = HAL_I2C_Mem_Read(&hi2c1, MAX30102_ADDRESS << 1, MAX30105_INTSTAT2, I2C_MEMADD_SIZE_8BIT, &intStatus, 1, HAL_MAX_DELAY);

	    if (status == HAL_OK) {
	        // Successfully read the register, output the status
	        sprintf(uartBuf, "Interrupt Status 2: 0x%02X\r\n", intStatus);
	        HAL_UART_Transmit(&huart2, (uint8_t*)uartBuf, strlen(uartBuf), HAL_MAX_DELAY);

	        // Check if the DIE_TEMP_RDY bit (bit 1) is set
	        if (intStatus & MAX30105_INT_DIE_TEMP_RDY_ENABLE) {
	            sprintf(uartBuf, "Temperature Ready Interrupt was set, cleared now.\r\n");
	            HAL_UART_Transmit(&huart2, (uint8_t*)uartBuf, strlen(uartBuf), HAL_MAX_DELAY);
	        } else {
	            sprintf(uartBuf, "No Temperature Ready Interrupt pending.\r\n");
	            HAL_UART_Transmit(&huart2, (uint8_t*)uartBuf, strlen(uartBuf), HAL_MAX_DELAY);
	        }
	    } else {
	        // If the read fails, output an error message
	        sprintf(uartBuf, "Failed to read Interrupt Status 2 register.\r\n");
	        HAL_UART_Transmit(&huart2, (uint8_t*)uartBuf, strlen(uartBuf), HAL_MAX_DELAY);
	        Error_Handler();
	    }
	}



	void MAX30102_EnableTemperatureInterrupt(void) {
	    uint8_t data = MAX30105_INT_DIE_TEMP_RDY_ENABLE;
	    // write to INT_ENABLE_2 register (0x03)
	    HAL_I2C_Mem_Write(&hi2c1, MAX30102_ADDRESS << 1, MAX30105_INTENABLE2,
	                      I2C_MEMADD_SIZE_8BIT, &data, 1, HAL_MAX_DELAY);
	    char uartBuf[50];
	    sprintf(uartBuf, "Interrupt enabled...\r\n");
	    HAL_UART_Transmit(&huart2, (uint8_t*)uartBuf, strlen(uartBuf), HAL_MAX_DELAY);

	}


// function to start temperature measurement on MAX30102
	void MAX30102_StartTemperatureMeasurement(void) {
	    uint8_t data = 0x01;  // Start temperature measurement
	    // Write to TEMP_CONFIG register (0x21)
	    HAL_I2C_Mem_Write(&hi2c1, MAX30102_ADDRESS << 1, MAX30105_DIETEMPCONFIG,
	                      I2C_MEMADD_SIZE_8BIT, &data, 1, HAL_MAX_DELAY);
	}


// function reading temperature after interrupt signals it's ready
	float MAX30102_ReadTemperature(void) {
	    uint8_t tempInt = 0, tempFrac = 0;
	    // Read integer part of temperature
	    HAL_I2C_Mem_Read(&hi2c1, MAX30102_ADDRESS << 1, MAX30105_TEMP_INTEGER,
	                     I2C_MEMADD_SIZE_8BIT, &tempInt, 1, HAL_MAX_DELAY);
	    // Read fractional part of temperature
	    HAL_I2C_Mem_Read(&hi2c1, MAX30102_ADDRESS << 1, MAX30105_TEMP_FRACTION,
	                     I2C_MEMADD_SIZE_8BIT, &tempFrac, 1, HAL_MAX_DELAY);
	    // Calculate temperature in degrees Celsius
	    return (float)tempInt + ((float)tempFrac * 0.0625);
	}


// fnterrupt callback for PA1 external interrupt
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin) {
    if (GPIO_Pin == GPIO_PIN_1) {  // check if PA1 triggered the interrupt
    	// Debug message to check if interrupt is being triggered
//    	__HAL_GPIO_EXTI_CLEAR_FLAG(GPIO_PIN_1);

    	        char debugMsg[] = "Interrupt Triggered PA1!\r\n";
    	        HAL_UART_Transmit(&huart2, (uint8_t*)debugMsg, strlen(debugMsg), HAL_MAX_DELAY);

        uint8_t intStatus = 0;
        HAL_I2C_Mem_Read(&hi2c1, MAX30102_ADDRESS << 1, MAX30105_INTSTAT2, I2C_MEMADD_SIZE_8BIT, &intStatus, 1, HAL_MAX_DELAY);

        if (intStatus & MAX30105_INT_DIE_TEMP_RDY_ENABLE) {  // if temperature ready
            float temperature = MAX30102_ReadTemperature();  // read temperature
            char uartBuf[100];
            sprintf(uartBuf, "Temperature: %.2f°C\r\n", temperature);
            HAL_UART_Transmit(&huart2, (uint8_t*)uartBuf, strlen(uartBuf), HAL_MAX_DELAY);
            // clearing the interrupt by reading the status register again
            HAL_I2C_Mem_Read(&hi2c1, MAX30102_ADDRESS << 1, MAX30105_INTSTAT2, I2C_MEMADD_SIZE_8BIT, &intStatus, 1, HAL_MAX_DELAY);
        }

    }
    if (GPIO_Pin == GPIO_PIN_0) {  // check if PB0 triggered the interrupt
    	// Debug message to check if interrupt is being triggered
//    	__HAL_GPIO_EXTI_CLEAR_FLAG(GPIO_PIN_1);

    	        char debugMsg[] = "Interrupt Triggered PB0!\r\n";
    	        HAL_UART_Transmit(&huart2, (uint8_t*)debugMsg, strlen(debugMsg), HAL_MAX_DELAY);

        uint8_t intStatus = 0;
        HAL_I2C_Mem_Read(&hi2c1, MAX30102_ADDRESS << 1, MAX30105_INTSTAT2, I2C_MEMADD_SIZE_8BIT, &intStatus, 1, HAL_MAX_DELAY);

        if (intStatus & MAX30105_INT_DIE_TEMP_RDY_ENABLE) {  // if temperature ready
            float temperature = MAX30102_ReadTemperature();  // read temperature
            char uartBuf[100];
            sprintf(uartBuf, "Temperature: %.2f°C\r\n", temperature);
            HAL_UART_Transmit(&huart2, (uint8_t*)uartBuf, strlen(uartBuf), HAL_MAX_DELAY);
            // clearing the interrupt by reading the status register again
//            HAL_I2C_Mem_Read(&hi2c1, MAX30102_ADDRESS << 1, MAX30105_INTSTAT2, I2C_MEMADD_SIZE_8BIT, &intStatus, 1, HAL_MAX_DELAY);
        }

    }
}


	int Detect_Finger(uint32_t redLED) {
		return (redLED > fingerThreshold);
	}


	void Process_Heartbeat(uint32_t redLED) {
		char uartBuf[100];
		float currentValue = (float)redLED;

		// pdate max and min values
		maxValue = fmaxf(maxValue, currentValue);
		minValue = fminf(minValue, currentValue);

		// alculate dynamic threshold
		float nthreshold = (maxValue - minValue) * rThreshold + minValue;
		threshold = threshold * (1 - thrRate) + nthreshold * thrRate;
		threshold = fminf(maxValue, fmaxf(minValue, threshold));

		// check if the heartbeat is detected
		if (currentValue >= threshold && lastValue < threshold && (maxValue - minValue) > minDiff && HAL_GetTick() - lastHeartbeat > 300) {
			if (lastHeartbeat != 0) {
				int bpm = 60000 / (HAL_GetTick() - lastHeartbeat);
				if (bpm > 0 && bpm < 25000) {
					sprintf(uartBuf, "Heart Rate (BPM): %d\r\n", bpm);
					Transmit_UART(uartBuf);
				}
			}
			lastHeartbeat = HAL_GetTick();
		}

		// Apply decay to max/min
		maxValue -= (maxValue - currentValue) * decayRate;
		minValue += (currentValue - minValue) * decayRate;

		lastValue = currentValue;
	}
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_I2C1_Init(void);
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

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_USART2_UART_Init();
  MX_I2C1_Init();
  /* USER CODE BEGIN 2 */

	uint32_t redLED, irLED;


	char uartBuf[100];  // buffer for  messages
	sprintf(uartBuf, "UART initialezed. waiting for sensor data...\r\n");
	HAL_UART_Transmit(&huart2, (uint8_t*)uartBuf, strlen(uartBuf), HAL_MAX_DELAY);
	// MAX30102 sensor
	MAX30102_Init();
	MAX30102_EnableTemperatureInterrupt();
		  // filters
	initialize_filters();

	// enable the temperature interrupt on MAX30102

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
	  while (1)
	  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
		  sprintf(uartBuf, "starting temp measurement...\r\n");
		  HAL_UART_Transmit(&huart2, (uint8_t*)uartBuf, strlen(uartBuf), HAL_MAX_DELAY);

//		    CheckAndClearInterruptStatus();

		    MAX30102_StartTemperatureMeasurement();




//		 uint8_t intStatus = 0;
//		 HAL_I2C_Mem_Read(&hi2c1, MAX30102_ADDRESS << 1, MAX30105_INTSTAT2, I2C_MEMADD_SIZE_8BIT, &intStatus, 1, HAL_MAX_DELAY);
//		 if (intStatus & MAX30105_INT_DIE_TEMP_RDY_ENABLE) {
//		     sprintf(uartBuf, "temp ready\r\n");
//		     HAL_UART_Transmit(&huart2, (uint8_t*)uartBuf, strlen(uartBuf), HAL_MAX_DELAY);
//		 }


		    if (HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_1) == GPIO_PIN_RESET) {
		        sprintf(uartBuf, "INT pin is still LOW!\r\n");
		        HAL_UART_Transmit(&huart2, (uint8_t*)uartBuf, strlen(uartBuf), HAL_MAX_DELAY);
		    } else {
		        sprintf(uartBuf, "INT pin is HIGH!\r\n");
		        HAL_UART_Transmit(&huart2, (uint8_t*)uartBuf, strlen(uartBuf), HAL_MAX_DELAY);
		    }



		 HAL_Delay(1000);
		 continue;




		uint32_t start_time = HAL_GetTick();
		MAX30102_ReadRawData(&redLED, &irLED);
		float current_value_red = (float)redLED;
		float current_value_ir = (float)irLED;

	    char uartBuf[100];
	    sprintf(uartBuf, "Red LED: %lu, IR LED: %lu\r\n", redLED, irLED);


//	    Transmit_UART(uartBuf);



		  // Detect Finger using the Red LED value
				  if (current_value_red > kFingerThreshold) {
					  if (HAL_GetTick() - finger_timestamp > kFingerCooldownMs) {
						  finger_detected = true;
					  }
				  } else {
					  // Reset all values
					  if (finger_detected && (HAL_GetTick() - finger_timestamp > 1000)) {
					  Differentiator_Reset(&differentiator);
					  MovingAverageFilter_Reset(&moving_avg_bpm);
					  MovingAverageFilter_Reset(&moving_avg_spo2);
					  LowPassFilter_Reset(&low_pass_filter_red);
					  LowPassFilter_Reset(&low_pass_filter_ir);
					  HighPassFilter_Reset(&high_pass_filter);
					  MinMaxAvgStatistic_Reset(&stat_red);
					  MinMaxAvgStatistic_Reset(&stat_ir);

					  finger_detected = false;
					  finger_timestamp = HAL_GetTick();
					  }
				  }

				  if (finger_detected) {
					  // low Filters to Red and IR signals
					  current_value_red = LowPassFilter_Process(&low_pass_filter_red, current_value_red);
					  current_value_ir = LowPassFilter_Process(&low_pass_filter_ir, current_value_ir);

					  // statistics for both Red and IR signals
					  MinMaxAvgStatistic_Process(&stat_red, current_value_red);
					  MinMaxAvgStatistic_Process(&stat_ir, current_value_ir);

					  //  High Pass Filter for heartbeat detection
					  float current_value = HighPassFilter_Process(&high_pass_filter, current_value_red);
					  float current_diff = Differentiator_Process(&differentiator, current_value);

					  if (!isnan(current_diff) && !isnan(last_diff)) {
						  // heartbeat via zero-crossing
						  if (last_diff > 0 && current_diff < 0) {
							  crossed = true;
							  crossed_time = HAL_GetTick();
						  }

						  if (crossed && current_diff < kEdgeThreshold) {
							  if (lastHeartbeat != 0 && crossed_time - lastHeartbeat > 300) {
								  // Calculate heart rate (BPM)
								  int bpm = 60000 / (crossed_time - lastHeartbeat);

								  // Calculate R-values for SpO2 calculation
								  float rred = (MinMaxAvgStatistic_Maximum(&stat_red) - MinMaxAvgStatistic_Minimum(&stat_red)) / MinMaxAvgStatistic_Average(&stat_red);
								  float rir = (MinMaxAvgStatistic_Maximum(&stat_ir) - MinMaxAvgStatistic_Minimum(&stat_ir)) / MinMaxAvgStatistic_Average(&stat_ir);
								  float spo2 = calculate_SpO2(rred, rir);

								  if (bpm > 5 && bpm < 250) {
									  int avg_bpm = MovingAverageFilter_Process(&moving_avg_bpm, bpm);
									  int avg_spo2 = MovingAverageFilter_Process(&moving_avg_spo2, spo2);


									  sprintf(uartBuf, "heart rate: %d bpm, SpO2: %.2f%%\r\n", avg_bpm, (double)avg_spo2);


									  HAL_UART_Transmit(&huart2, (uint8_t*)uartBuf, strlen(uartBuf), HAL_MAX_DELAY);
								  }
							  }
							  crossed = false;
							  lastHeartbeat = crossed_time;
						  }
					  }
					  last_diff = current_diff;


					    // end time
					    uint32_t end_time = HAL_GetTick();

					    //  time taken
					    uint32_t time_taken = end_time - start_time;


					    sprintf(uartBuf, "Processing time: %lu ms\r\n", time_taken);
//					    Transmit_UART(uartBuf);

				  }else{
					  sprintf(uartBuf, "place finger\r\n");
					  Transmit_UART(uartBuf);
//					  HAL_Delay(500);
				  }




//				  HAL_Delay(2);  // sampling rate for 400 samples per second
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

  /** Configure the main internal regulator output voltage
  */
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_MSI;
  RCC_OscInitStruct.MSIState = RCC_MSI_ON;
  RCC_OscInitStruct.MSICalibrationValue = 0;
  RCC_OscInitStruct.MSIClockRange = RCC_MSIRANGE_5;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_MSI;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
  {
    Error_Handler();
  }
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_USART2|RCC_PERIPHCLK_I2C1;
  PeriphClkInit.Usart2ClockSelection = RCC_USART2CLKSOURCE_PCLK1;
  PeriphClkInit.I2c1ClockSelection = RCC_I2C1CLKSOURCE_PCLK1;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
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
  hi2c1.Init.Timing = 0x00000608;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.OwnAddress2Masks = I2C_OA2_NOMASK;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Analogue filter
  */
  if (HAL_I2CEx_ConfigAnalogFilter(&hi2c1, I2C_ANALOGFILTER_ENABLE) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Digital filter
  */
  if (HAL_I2CEx_ConfigDigitalFilter(&hi2c1, 0) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C1_Init 2 */

  /* USER CODE END I2C1_Init 2 */

}

/**
  * @brief USART2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART2_UART_Init(void)
{

  /* USER CODE BEGIN USART2_Init 0 */

  /* USER CODE END USART2_Init 0 */

  /* USER CODE BEGIN USART2_Init 1 */

  /* USER CODE END USART2_Init 1 */
  huart2.Instance = USART2;
  huart2.Init.BaudRate = 115200;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  huart2.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart2.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART2_Init 2 */

  /* USER CODE END USART2_Init 2 */

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
/* USER CODE BEGIN MX_GPIO_Init_1 */
/* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : PA1 */
  GPIO_InitStruct.Pin = GPIO_PIN_1;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : LD2_Pin */
  GPIO_InitStruct.Pin = LD2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(LD2_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : PB0 */
  GPIO_InitStruct.Pin = GPIO_PIN_0;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI0_1_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI0_1_IRQn);

/* USER CODE BEGIN MX_GPIO_Init_2 */

/* USER CODE END MX_GPIO_Init_2 */
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
