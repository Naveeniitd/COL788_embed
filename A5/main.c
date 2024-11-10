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
#include "cmsis_os.h"
#include "fatfs.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdbool.h>
#include <stdio.h>
#include "filter.h"
#include <string.h>
#include <stdarg.h> //for va_list var arg functions
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "semphr.h"

// Define data structures for raw and processed data
typedef struct {
    uint32_t redLED;
    uint32_t irLED;
} SensorData_t;

typedef struct {
    int bpm;
    float spo2;
} ProcessedData_t;

// Declare queues
QueueHandle_t xQueueSensorData;
QueueHandle_t xQueueProcessedData;

FATFS FatFs; 	//Fatfs handle
FIL fil; 		//File handle
FRESULT fres;
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
	I2C_HandleTypeDef hi2c1;
	UART_HandleTypeDef huart2;
	TaskHandle_t xTemperatureTaskHandle = NULL;
	SemaphoreHandle_t xI2CSemaphore;

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

SPI_HandleTypeDef hspi2;

UART_HandleTypeDef huart2;

osThreadId defaultTaskHandle;
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
	xSemaphoreTake(xI2CSemaphore, portMAX_DELAY);
	if (HAL_I2C_Mem_Read(&hi2c1, MAX30102_ADDRESS << 1, 0x07, I2C_MEMADD_SIZE_8BIT, rawData, 6, HAL_MAX_DELAY) != HAL_OK) {
		sprintf(uartBuf, "I2C read error\r\n");
		HAL_UART_Transmit(&huart2, (uint8_t*)uartBuf, strlen(uartBuf), HAL_MAX_DELAY);
        // Release the mutex before error handling
        xSemaphoreGive(xI2CSemaphore);
		Error_Handler();  //  keep printing this message === so  avoid halting the program
	}
    // Release the mutex after I2C operations
    xSemaphoreGive(xI2CSemaphore);
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
    // Take the I2C mutex
    xSemaphoreTake(xI2CSemaphore, portMAX_DELAY);
    // Write to TEMP_CONFIG register (0x21)
    HAL_I2C_Mem_Write(&hi2c1, MAX30102_ADDRESS << 1, MAX30105_DIETEMPCONFIG,
                      I2C_MEMADD_SIZE_8BIT, &data, 1, HAL_MAX_DELAY);

    // Release the mutex
    xSemaphoreGive(xI2CSemaphore);
}


// function reading temperature after interrupt signals it's ready
float MAX30102_ReadTemperature(void) {
    uint8_t tempInt = 0, tempFrac = 0;

    // Take the I2C mutex
        xSemaphoreTake(xI2CSemaphore, portMAX_DELAY);
    // Read integer part of temperature
    HAL_I2C_Mem_Read(&hi2c1, MAX30102_ADDRESS << 1, MAX30105_TEMP_INTEGER,
                     I2C_MEMADD_SIZE_8BIT, &tempInt, 1, HAL_MAX_DELAY);
    // Read fractional part of temperature
    HAL_I2C_Mem_Read(&hi2c1, MAX30102_ADDRESS << 1, MAX30105_TEMP_FRACTION,
                     I2C_MEMADD_SIZE_8BIT, &tempFrac, 1, HAL_MAX_DELAY);
    // Calculate temperature in degrees Celsius

    // Release the mutex
    xSemaphoreGive(xI2CSemaphore);

    return (float)tempInt + ((float)tempFrac * 0.0625);
}


// fnterrupt callback for PA1 external interrupt
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
    BaseType_t xHigherPriorityTaskWoken = pdFALSE;
    if (GPIO_Pin == GPIO_PIN_1) // Assuming PA1 is connected to MAX30102 interrupt
    {
        // Defer processing to the TemperatureTask
        vTaskNotifyGiveFromISR(xTemperatureTaskHandle, &xHigherPriorityTaskWoken);
        portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
    }
    if (GPIO_Pin == GPIO_PIN_0) // Assuming PB0 is connected to MAX30102 interrupt
    {
        // Defer processing to the TemperatureTask
        vTaskNotifyGiveFromISR(xTemperatureTaskHandle, &xHigherPriorityTaskWoken);
        portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
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
static void MX_SPI2_Init(void);
static void MX_I2C1_Init(void);
void StartDefaultTask(void const * argument);

/* USER CODE BEGIN PFP */
void myprintf(const char *fmt, ...);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
void myprintf(const char *fmt, ...) {
  static char buffer[256];
  va_list args;
  va_start(args, fmt);
  vsnprintf(buffer, sizeof(buffer), fmt, args);
  va_end(args);

  int len = strlen(buffer);
  HAL_UART_Transmit(&huart2, (uint8_t*)buffer, len, -1);

}




void SensingTask(void *pvParameters)
{
//	myprintf("sensing ke andar\r\n");
//	TickType_t xLastWakeTime;
//    const TickType_t xFrequency = pdMS_TO_TICKS(10); // Sampling rate of 100Hz

    // Initialize last wake time
//    xLastWakeTime = xTaskGetTickCount();
    SensorData_t sensorData;

    for (;;)
    {
        // Wait for the next cycle
//    	myprintf("SensingTask for loop \r\n");
//        vTaskDelayUntil(&xLastWakeTime, xFrequency);

        // Read data from the sensor
        uint32_t redLED, irLED;
        MAX30102_ReadRawData(&redLED, &irLED);

        // Create sensor data structure

        sensorData.redLED = redLED;
        sensorData.irLED = irLED;
        // Send data to the processing task
        xQueueSend(xQueueSensorData, &sensorData, portMAX_DELAY);

        // Send data to the processing task
//        if (xQueueSend(xQueueSensorData, &sensorData, portMAX_DELAY) != pdPASS) {
//            myprintf("Failed to send sensor data to queue\r\n");
//        }


    }
}


void ProcessingTask(void *pvParameters)
{
    myprintf("Processing Task Started.\r\n");

    SensorData_t sensorData;
    ProcessedData_t processedData;

    // Initialize filters and statistics
    initialize_filters();

    // Variables for processing
    float last_diff = NAN;
    bool crossed = false;
    uint32_t lastHeartbeat = 0;
    uint32_t crossed_time = 0; // Moved outside the if-block
    MinMaxAvgStatistic stat_red;
    MinMaxAvgStatistic stat_ir;
    MovingAverageFilter moving_avg_bpm;
    MovingAverageFilter moving_avg_spo2;

    // Initialize statistics and filters
    MinMaxAvgStatistic_Reset(&stat_red);
    MinMaxAvgStatistic_Reset(&stat_ir);
    MovingAverageFilter_Init(&moving_avg_bpm);
    MovingAverageFilter_Init(&moving_avg_spo2);

    for (;;)
    {
//        myprintf("ProcessingTask waiting for data.\r\n");

        // Wait for data from the sensing task
        if (xQueueReceive(xQueueSensorData, &sensorData, portMAX_DELAY) == pdPASS)
        {
//            myprintf("ProcessingTask received sensor data.\r\n");

            // Process the data
            uint32_t redLED = sensorData.redLED;
            uint32_t irLED = sensorData.irLED;

            float current_value_red = (float)redLED;
            float current_value_ir = (float)irLED;
            bool fingerIsDetected = Detect_Finger(current_value_red);

            if (fingerIsDetected)
            {
//                myprintf("Finger detected.\r\n");

                // Apply low-pass filters
                current_value_red = LowPassFilter_Process(&low_pass_filter_red, current_value_red);
                current_value_ir = LowPassFilter_Process(&low_pass_filter_ir, current_value_ir);

                // Update statistics
                MinMaxAvgStatistic_Process(&stat_red, current_value_red);
                MinMaxAvgStatistic_Process(&stat_ir, current_value_ir);

                // Apply high-pass filter and differentiator
                float high_passed = HighPassFilter_Process(&high_pass_filter, current_value_red);
                float current_diff = Differentiator_Process(&differentiator, high_passed);

                if (!isnan(current_diff) && !isnan(last_diff))
                {
                    // Detect heartbeat via zero-crossing
                    if (last_diff > 0 && current_diff < 0)
                    {
                        crossed = true;
                        crossed_time = xTaskGetTickCount();
//                        myprintf("Zero-crossing detected.\r\n");
                    }

                    if (crossed && current_diff < kEdgeThreshold)
                    {
//                    	myprintf("we are in final calculation\r\n");
                        // Calculate heart rate (BPM)
                        if (lastHeartbeat != 0)
                        {
                            uint32_t interval_ticks = crossed_time - lastHeartbeat;
                            // Assuming configTICK_RATE_HZ is 1000, interval_ticks = interval_ms
                            int bpm = 60000 / interval_ticks;

                            // Calculate R-values for SpO2 calculation
                            float rred = (MinMaxAvgStatistic_Maximum(&stat_red) - MinMaxAvgStatistic_Minimum(&stat_red)) / MinMaxAvgStatistic_Average(&stat_red);
                            float rir = (MinMaxAvgStatistic_Maximum(&stat_ir) - MinMaxAvgStatistic_Minimum(&stat_ir)) / MinMaxAvgStatistic_Average(&stat_ir);
                            float spo2 = calculate_SpO2(rred, rir);

                            if (bpm > 5 && bpm < 250)
                            {
                                // Apply moving average filters
                                processedData.bpm = MovingAverageFilter_Process(&moving_avg_bpm, bpm);
                                processedData.spo2 = MovingAverageFilter_Process(&moving_avg_spo2, spo2);
//                                myprintf("Heart Rate Process: %d BPM, SpO2: %.2f%%\r\n", processedData.bpm, processedData.spo2);
                                // Send processed data to SDCardWritingTask
                                xQueueSend(xQueueProcessedData, &processedData, portMAX_DELAY);
//                                if (xQueueSend(xQueueProcessedData, &processedData, portMAX_DELAY) != pdPASS) {
//                                    myprintf("Failed to send processed data to queue.\r\n");
//                                }

                                // UART Transmission

                            }
                        }
                        crossed = false;
                        lastHeartbeat = crossed_time;
                    }
                }
                last_diff = current_diff;
            }
            else
            {
//                myprintf("Place Finger.\r\n");

                // Reset all processing variables and filters
                Differentiator_Reset(&differentiator);
                MovingAverageFilter_Reset(&moving_avg_bpm);
                MovingAverageFilter_Reset(&moving_avg_spo2);
                LowPassFilter_Reset(&low_pass_filter_red);
                LowPassFilter_Reset(&low_pass_filter_ir);
                HighPassFilter_Reset(&high_pass_filter);
                MinMaxAvgStatistic_Reset(&stat_red);
                MinMaxAvgStatistic_Reset(&stat_ir);
                last_diff = NAN;
                crossed = false;
                lastHeartbeat = 0;
                crossed_time = 0;
            }
        }
    }
}


void SDCardWritingTask(void *pvParameters)
{
//	myprintf("sdcard ke andar \r\n");
    ProcessedData_t processedData;
    char dataBuf[100];


    for (;;)
    {
//    	myprintf("sdcard ke for loop \r\n");
        // Wait for processed data

        if (xQueueReceive(xQueueProcessedData, &processedData, portMAX_DELAY) == pdPASS)
        {
            // Write data to SD card
//        	myprintf("sdcard data mila\r\n");

//            myprintf("Heart Rate: %d BPM, SpO2: %.2f%%\r\n", processedData.bpm, (double)processedData.spo2);
            sprintf(dataBuf,"Heart Rate: %i BPM, SpO2: %.2f%%\r\n", processedData.bpm, processedData.spo2);
            myprintf(dataBuf);
//             Open the file for appending data
//            myprintf("just before");
            fres = f_open(&fil, "save.txt", FA_WRITE | FA_OPEN_APPEND);
            if (fres == FR_OK)
            {myprintf("open ok \r\n");}
            else
            {myprintf("error\r\n");}

            UINT bytesWrote;
             fres = f_write(&fil, dataBuf, strlen(dataBuf), &bytesWrote);
             if (fres == FR_OK)
             {myprintf("save ok\r\n");}
             else
             {myprintf("error\r\n");}
            f_close(&fil);

            // Optionally, send the data via UART
//            HAL_UART_Transmit(&huart2, (uint8_t *)dataBuf, strlen(dataBuf), HAL_MAX_DELAY);
        }
    }
}

void TemperatureTask(void *pvParameters)
{
//    FATFS FatFs;

    char dataBuf[100];


    for (;;)
    {
        // Wait for notification from ISR
        ulTaskNotifyTake(pdTRUE, portMAX_DELAY);

        // Read the temperature
        float temperature = MAX30102_ReadTemperature();

        // Process the data (if any processing is needed)
        // For example, apply a moving average filter or convert units

        // Write to SD card

        sprintf(dataBuf, "Temperature: %.2f°C\r\n", temperature);

        // Open the file for appending data
        fres = f_open(&fil, "save.txt", FA_WRITE | FA_OPEN_APPEND );
        if (fres == FR_OK)
        {myprintf("open ok \r\n");}
        else
        {myprintf("error\r\n");}

        UINT bytesWrote;
         fres = f_write(&fil, dataBuf, strlen(dataBuf), &bytesWrote);
         if (fres == FR_OK)
         {myprintf("save ok\r\n");}
         else
         {myprintf("error\r\n");}
        f_close(&fil);

        // Optionally, send the data via UART
        HAL_UART_Transmit(&huart2, (uint8_t *)dataBuf, strlen(dataBuf), HAL_MAX_DELAY);

        // Start a new temperature measurement
        MAX30102_StartTemperatureMeasurement();
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
  MX_USART2_UART_Init();
  MX_SPI2_Init();
  MX_FATFS_Init();
  MX_I2C1_Init();
  /* USER CODE BEGIN 2 */
  myprintf("\r\n~ SD card demo by kiwih ~\r\n\r\n");

    HAL_Delay(1000); //a short delay is important to let the SD card settle

    //some variables for FatFs
 //Result after operations

    //Open the file system
    fres = f_mount(&FatFs, "", 1); //1=mount now
    if (fres != FR_OK) {
  	myprintf("f_mount error (%i)\r\n", fres);
  	while(1);
    }

    //Let's get some statistics from the SD card
    DWORD free_clusters, free_sectors, total_sectors;

    FATFS* getFreeFs;

    fres = f_getfree("", &free_clusters, &getFreeFs);
    if (fres != FR_OK) {
  	myprintf("f_getfree error (%i)\r\n", fres);
  	while(1);
    }

    //Formula comes from ChaN's documentation
    total_sectors = (getFreeFs->n_fatent - 2) * getFreeFs->csize;
    free_sectors = free_clusters * getFreeFs->csize;

    myprintf("SD card stats:\r\n%10lu KiB total drive space.\r\n%10lu KiB available.\r\n", total_sectors / 2, free_sectors / 2);

    //Now let's try to open file "test.txt"
    fres = f_open(&fil, "test.txt", FA_READ);
    if (fres != FR_OK) {
  	myprintf("f_open error (%i)\r\n");
  	while(1);
    }
    myprintf("I was able to open 'test.txt' for reading!\r\n");

    //Read 30 bytes from "test.txt" on the SD card
    BYTE readBuf[30];

    //We can either use f_read OR f_gets to get data out of files
    //f_gets is a wrapper on f_read that does some string formatting for us
    TCHAR* rres = f_gets((TCHAR*)readBuf, 30, &fil);
    if(rres != 0) {
  	myprintf("Read string from 'test.txt' contents: %s\r\n", readBuf);
    } else {
  	myprintf("f_gets error (%i)\r\n", fres);
    }

    //Be a tidy kiwi - don't forget to close your file!
    f_close(&fil);

    //Now let's try and write a file "write.txt"
    fres = f_open(&fil, "save.txt", FA_WRITE | FA_OPEN_ALWAYS | FA_CREATE_ALWAYS);
    if(fres == FR_OK) {
  	myprintf("I was able to open 'save.txt' for writing\r\n");
    } else {
  	myprintf("f_open error (%i)\r\n", fres);
    }

    //Copy in a string
    strncpy((char*)readBuf, "a new file is made!\r\n", strlen("a new file is made!\r\n"));
    UINT bytesWrote;
    fres = f_write(&fil, readBuf, strlen("a new file is made!\r\n"), &bytesWrote);
    if(fres == FR_OK) {
  	myprintf("Wrote %i bytes to 'save.txt'!\r\n", bytesWrote);
    } else {
  	myprintf("f_write error (%i)\r\n");
    }

    //Be a tidy kiwi - don't forget to close your file!
    f_close(&fil);

    //We're done, so de-mount the drive
//    f_mount(NULL, "", 0);



    //a2...............................................................................

//	uint32_t redLED, irLED;
	char uartBuf[100];  // buffer for  messages
	sprintf(uartBuf, "UART initialezed. waiting for sensor data...\r\n");
	HAL_UART_Transmit(&huart2, (uint8_t*)uartBuf, strlen(uartBuf), HAL_MAX_DELAY);
	// MAX30102 sensor
	MAX30102_Init();
	MAX30102_EnableTemperatureInterrupt();
		  // filters
	initialize_filters();

	xI2CSemaphore = xSemaphoreCreateMutex();




	//    // Start the first temperature measurement
	    MAX30102_StartTemperatureMeasurement();






	xQueueSensorData = xQueueCreate(100, sizeof(SensorData_t));
	xQueueProcessedData = xQueueCreate(100, sizeof(ProcessedData_t));













	// Create Tasks
    BaseType_t a;
    // Create the tasks
    a= xTaskCreate(SensingTask, "SensingTask", 128, NULL, 2, NULL);

    if (a != pdPASS) {
        myprintf("Failed to create SensingTask\r\n");
        while(1);
    }
    a = xTaskCreate(ProcessingTask, "ProcessingTask", 256, NULL, 2, NULL);

    if (a != pdPASS) {
        myprintf("Failed to create ProcessingTask\r\n");
        while(1);
    }
    a = xTaskCreate(SDCardWritingTask, "SDCardWritingTask", 256, NULL, 2, NULL);
    if (a != pdPASS) {
        myprintf("Failed to create SDCardWritingTask\r\n");
        while(1);
    }


    // Create the TemperatureTask
    xTaskCreate(TemperatureTask, "TemperatureTask", 512, NULL, 2, &xTemperatureTaskHandle);









  /* USER CODE END 2 */

  /* USER CODE BEGIN RTOS_MUTEX */
  /* add mutexes, ... */
  /* USER CODE END RTOS_MUTEX */

  /* USER CODE BEGIN RTOS_SEMAPHORES */
  /* add semaphores, ... */
  /* USER CODE END RTOS_SEMAPHORES */

  /* USER CODE BEGIN RTOS_TIMERS */
  /* start timers, add new ones, ... */
  /* USER CODE END RTOS_TIMERS */

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* definition and creation of defaultTask */
  osThreadDef(defaultTask, StartDefaultTask, osPriorityNormal, 0, 128);
  defaultTaskHandle = osThreadCreate(osThread(defaultTask), NULL);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  /* USER CODE END RTOS_THREADS */

  /* Start scheduler */
  osKernelStart();

  /* We should never get here as control is now taken by the scheduler */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
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
  * @brief SPI2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI2_Init(void)
{

  /* USER CODE BEGIN SPI2_Init 0 */

  /* USER CODE END SPI2_Init 0 */

  /* USER CODE BEGIN SPI2_Init 1 */

  /* USER CODE END SPI2_Init 1 */
  /* SPI2 parameter configuration*/
  hspi2.Instance = SPI2;
  hspi2.Init.Mode = SPI_MODE_MASTER;
  hspi2.Init.Direction = SPI_DIRECTION_2LINES;
  hspi2.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi2.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi2.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi2.Init.NSS = SPI_NSS_SOFT;
  hspi2.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_128;
  hspi2.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi2.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi2.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi2.Init.CRCPolynomial = 7;
  if (HAL_SPI_Init(&hspi2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI2_Init 2 */

  /* USER CODE END SPI2_Init 2 */

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

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(SD_CS_GPIO_Port, SD_CS_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : B1_Pin */
  GPIO_InitStruct.Pin = B1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(B1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : PA0 PA1 */
  GPIO_InitStruct.Pin = GPIO_PIN_0|GPIO_PIN_1;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : LD2_Pin */
  GPIO_InitStruct.Pin = LD2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(LD2_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : SD_CS_Pin */
  GPIO_InitStruct.Pin = SD_CS_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(SD_CS_GPIO_Port, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI0_1_IRQn, 7, 0);
  HAL_NVIC_EnableIRQ(EXTI0_1_IRQn);

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/* USER CODE BEGIN Header_StartDefaultTask */
/**
  * @brief  Function implementing the defaultTask thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_StartDefaultTask */
void StartDefaultTask(void const * argument)
{
  /* USER CODE BEGIN 5 */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END 5 */
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
