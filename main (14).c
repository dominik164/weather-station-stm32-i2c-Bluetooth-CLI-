/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2021 STMicroelectronics.
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
/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <string.h>
#include <stdio.h>
#include "bmp280.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
typedef enum
{
    TEMPERATURE,
    PRESSURE,
    HUMIDITY,
    ALL
} BMP_SEND_MODE;
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
static const char HELP[] = "help";
static const char TEMPE[] = "temperature";
static const char PRESS[] = "pressure";
static const char HUMI[] = "humidity";
static const char TIMER_ON[] = "timeron";
static const char TIMER_OFF[] = "timeroff";
static const char MONITOR_ON[] = "monitoron";
static const char MONITOR_OFF[] = "monitoroff";
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
I2C_HandleTypeDef hi2c1;

TIM_HandleTypeDef htim10;

UART_HandleTypeDef huart1;

/* USER CODE BEGIN PV */
BMP280_HandleTypedef bmp280;

float pressure, temperature, humidity;

uint8_t msg_received[100];
uint8_t msg_send[200];

uint16_t size = 0;
uint8_t Received;
uint8_t msg_complete;

volatile uint8_t monitor_mode = 0;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_TIM10_Init(void);
static void MX_I2C1_Init(void);
/* USER CODE BEGIN PFP */
void got_msg();
void bmp_send(BMP_SEND_MODE mode);

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
    if (Received == '\n')
    {
        //enter
        got_msg();
        memset(msg_received, 0, 100);
        size = 0;
    }
    else if (Received == 127)
    {
        //backspace
        size--;
        msg_received[size] = '\0';
        //clear cmd row and print new string
        uint8_t msg_size = sprintf((char *)msg_send, "\r>>%s", msg_received);
        HAL_UART_Transmit(&huart1, "\r                              ", 31, 10);
        HAL_UART_Transmit(&huart1, msg_send, msg_size, 10);
    }
    else
    {
        msg_received[size] = Received;
        size++;
        //echo
        HAL_UART_Transmit_IT(&huart1, &Received, 1);
    }

    HAL_UART_Receive_IT(&huart1, &Received, 1);
}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *hitm)
{
    if (monitor_mode)
    {
        bmp_send(ALL);
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
    MX_USART1_UART_Init();
    MX_TIM10_Init();
    MX_I2C1_Init();
    /* USER CODE BEGIN 2 */
    HAL_UART_Receive_IT(&huart1, &Received, 1);
    HAL_TIM_Base_Start_IT(&htim10);
    uint8_t msg_size = sprintf((char *)msg_send, ">>");
    HAL_UART_Transmit_IT(&huart1, msg_send, msg_size);

    bmp280_init_default_params(&bmp280.params);
    bmp280.addr = BMP280_I2C_ADDRESS_0;
    bmp280.i2c = &hi2c1;
    while (!bmp280_init(&bmp280, &bmp280.params))
    {
        size = sprintf((char *)msg_send, "BMP280 initialization failed\n");
        HAL_UART_Transmit(&huart1, msg_send, size, 1000);
        HAL_Delay(2000);
    }
    bool bme280p = bmp280.id == BME280_CHIP_ID;
    /* USER CODE END 2 */

    /* Infinite loop */
    /* USER CODE BEGIN WHILE */
    while (1)
    {
    	/*Sleep and wait for interrupt*/
    	__WFI();
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
    __HAL_RCC_PWR_CLK_ENABLE();
    __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);
    /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
    RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
    RCC_OscInitStruct.HSIState = RCC_HSI_ON;
    RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
    RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
    if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
    {
        Error_Handler();
    }
    /** Initializes the CPU, AHB and APB buses clocks
  */
    RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK | RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
    RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI;
    RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
    RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
    RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

    if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
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
    hi2c1.Init.ClockSpeed = 100000;
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

/**
  * @brief TIM10 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM10_Init(void)
{

    /* USER CODE BEGIN TIM10_Init 0 */

    /* USER CODE END TIM10_Init 0 */

    /* USER CODE BEGIN TIM10_Init 1 */

    /* USER CODE END TIM10_Init 1 */
    htim10.Instance = TIM10;
    htim10.Init.Prescaler = 16000;
    htim10.Init.CounterMode = TIM_COUNTERMODE_UP;
    htim10.Init.Period = 9999;
    htim10.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
    htim10.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
    if (HAL_TIM_Base_Init(&htim10) != HAL_OK)
    {
        Error_Handler();
    }
    /* USER CODE BEGIN TIM10_Init 2 */

    /* USER CODE END TIM10_Init 2 */
}

/**
  * @brief USART1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART1_UART_Init(void)
{

    /* USER CODE BEGIN USART1_Init 0 */

    /* USER CODE END USART1_Init 0 */

    /* USER CODE BEGIN USART1_Init 1 */

    /* USER CODE END USART1_Init 1 */
    huart1.Instance = USART1;
    huart1.Init.BaudRate = 115200;
    huart1.Init.WordLength = UART_WORDLENGTH_8B;
    huart1.Init.StopBits = UART_STOPBITS_1;
    huart1.Init.Parity = UART_PARITY_NONE;
    huart1.Init.Mode = UART_MODE_TX_RX;
    huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
    huart1.Init.OverSampling = UART_OVERSAMPLING_16;
    if (HAL_UART_Init(&huart1) != HAL_OK)
    {
        Error_Handler();
    }
    /* USER CODE BEGIN USART1_Init 2 */

    /* USER CODE END USART1_Init 2 */
}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{

    /* GPIO Ports Clock Enable */
    __HAL_RCC_GPIOA_CLK_ENABLE();
    __HAL_RCC_GPIOB_CLK_ENABLE();
}

/* USER CODE BEGIN 4 */
void got_msg()
{
    uint8_t msg_size = 0;
    if (!strcmp(HELP, (char *)msg_received))
    {
        msg_size = sprintf((char *)msg_send, "\n\r --COMMANDS:--\n\r 1. temperature\n\r 2. pressure\n\r 3. humidity\n\r 4. timeron\n\r 5. timeroff\n\r 6. help\n\r 7. monitoron\n\r 8. monitoroff\n\r");
        HAL_UART_Transmit(&huart1, msg_send, msg_size, 10);
    }
    else if (!strcmp(TEMPE, (char *)msg_received))
    {
        bmp_send(TEMPERATURE);
    }
    else if (!strcmp(PRESS, (char *)msg_received))
    {
        bmp_send(PRESSURE);
    }
    else if (!strcmp(HUMI, (char *)msg_received))
    {
        bmp_send(HUMIDITY);
    }
    else if (!strcmp(TIMER_ON, (char *)msg_received))
    {
        //timer on
        msg_size = sprintf((char *)msg_send, "\n\r Timer1 on. PWM=60%. Freq=1kHz\n\r");
        HAL_UART_Transmit(&huart1, msg_send, msg_size, 10);
    }
    else if (!strcmp(TIMER_OFF, (char *)msg_received))
    {
        //TIMER OFF
        msg_size = sprintf((char *)msg_send, "\n\r Timer 1 off\n\r");
        HAL_UART_Transmit(&huart1, msg_send, msg_size, 10);
    }
    else if (!strcmp(MONITOR_ON, (char *)msg_received))
    {
        //MONITOR ON
        msg_size = sprintf((char *)msg_send, "\n\r Monitor 1 on\n\r");
        HAL_UART_Transmit(&huart1, msg_send, msg_size, 10);
        monitor_mode = 1;
    }
    else if (!strcmp(MONITOR_OFF, (char *)msg_received))
    {
        //MONITOR OFF
        msg_size = sprintf((char *)msg_send, "\n\r Monitor 1 off\n\r");
        HAL_UART_Transmit(&huart1, msg_send, msg_size, 10);
        monitor_mode = 0;
    }
    else
    {
        msg_size = sprintf((char *)msg_send, "\n\r bad command\n\r type help\n\r");
        HAL_UART_Transmit(&huart1, msg_send, msg_size, 10);
    }
    msg_size = sprintf((char *)msg_send, ">>");
    HAL_UART_Transmit_IT(&huart1, msg_send, msg_size);
}

void bmp_send(BMP_SEND_MODE mode)
{
    while (!bmp280_read_float(&bmp280, &temperature, &pressure, &humidity))
    {
        size = sprintf((char *)msg_send, "\n\rBMP280 reading failed\n");
        HAL_UART_Transmit(&huart1, msg_send, size, 10);
    }
    switch (mode)
    {
    case TEMPERATURE:
        size = sprintf((char *)msg_send, "\n\rTemperature: %.2f C\n\r", temperature);
        HAL_UART_Transmit(&huart1, msg_send, size, 10);
        break;
    case PRESSURE:
        size = sprintf((char *)msg_send, "\n\rPressure: %.2f Pa\n\r", pressure);
        HAL_UART_Transmit(&huart1, msg_send, size, 10);
        break;
    case HUMIDITY:
        size = sprintf((char *)msg_send, "\n\rHumidity: %.2f%\n\r", humidity);
        HAL_UART_Transmit(&huart1, msg_send, size, 10);
        break;
    case ALL:
        size = sprintf((char *)msg_send, "\n\rTemperature: %.2f C, Pressure: %.2f Pa, Humidity: %.2f%\n\r", temperature, pressure, humidity);
        HAL_UART_Transmit(&huart1, msg_send, size, 10);
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

#ifdef USE_FULL_ASSERT
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

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
