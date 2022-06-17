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
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "string.h"
#include "../../User/config.h"
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

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */
uint8_t RS485_ReadRReceiveData();
void RS485_BufSend(uint8_t *buf, uint8_t len);
void MOTOR_DirectionControl(uint8_t mode);
uint16_t SW_PinRead();
uint16_t CRC16(uint8_t *start_byte, uint16_t num_bytes);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
uint8_t aRxBuffer = 0;
struct RxData{
    uint8_t RxState;
    uint16_t RxCounter;
    uint8_t RxBuff[8];
}rx_data={0, 0, {0}};

uint16_t sw_pin = 0;
uint16_t ms = 0;
/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */
  uint8_t motor_mode=0;
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
  MX_TIM14_Init();
  /* USER CODE BEGIN 2 */
  HAL_UART_Receive_IT(&huart1, &aRxBuffer, 1);
  HAL_TIM_Base_Start_IT(&htim14);
    //HAL_GPIO_WritePin(GPIOA, GPIO_PIN_8, GPIO_PIN_SET);
  //HAL_GPIO_WritePin(GPIOB, GPIO_PIN_9, GPIO_PIN_SET);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    if(rx_data.RxState == 1)
    {
        motor_mode = RS485_ReadRReceiveData();
        rx_data.RxCounter = 0;
        rx_data.RxState = 0;
    }
    MOTOR_DirectionControl(motor_mode);
    motor_mode = 0;

    if(ms % 10000 == 0)
        HAL_GPIO_WritePin(GPIOA, GPIO_PIN_8, GPIO_PIN_SET);
      if(ms % 10100 == 0)
        HAL_GPIO_WritePin(GPIOA, GPIO_PIN_8, GPIO_PIN_RESET);
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
  HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSIDiv = RCC_HSI_DIV1;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = RCC_PLLM_DIV1;
  RCC_OscInitStruct.PLL.PLLN = 8;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLR = RCC_PLLR_DIV2;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
    if(TIM14 == htim->Instance)
    {
        ms++;
    }
}

void HAL_GPIO_EXTI_Falling_Callback(uint16_t GPIO_Pin)
{
    sw_pin = GPIO_Pin;
}

//串口中断回调
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
    rx_data.RxBuff[rx_data.RxCounter] = aRxBuffer;
    if(rx_data.RxCounter >0)
    {
        if(rx_data.RxCounter == 7)
        {
            rx_data.RxState = 1;
        }
        rx_data.RxCounter ++;
    }
    if(aRxBuffer == 0xa5 && rx_data.RxCounter == 0)
    {
        rx_data.RxCounter ++;
    }
    HAL_UART_Receive_IT(&huart1, &aRxBuffer, 1);
}

uint16_t SW_PinRead()
{
    static uint8_t sw_state = 0;
    static uint16_t old_ms;
    if(sw_pin == 0)
        return 0;
    if(sw_state == 0)
    {
        old_ms = ms;
        sw_state = 1;
    }
    if(sw_state == 1 && (ms - old_ms > 10))
    {

        if(HAL_GPIO_ReadPin(S1_GPIO_Port, sw_pin) == 0)
        {
            sw_state = 0;
            uint16_t sw_pin_old = sw_pin;
            sw_pin = 0;
            return sw_pin_old;
        }
    } else if(HAL_GPIO_ReadPin(S1_GPIO_Port, sw_pin) != 0)
    {
        sw_pin = 0;
        sw_state = 0;
    }
    return 0;
}

void RS485_BufSend(uint8_t *buf, uint8_t len)
{
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_9, GPIO_PIN_SET);
    uint8_t crc16 = CRC16(buf, 6);
    buf[6] = crc16;
    HAL_UART_Transmit(&huart1, buf, len, 0xffff);
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_9, GPIO_PIN_RESET);
}

//485协议处理
uint8_t RS485_ReadRReceiveData()
{
    if(rx_data.RxBuff[0] != 0xa5)
        return 0;
    if(rx_data.RxBuff[1] != 0xa5)
        return 0;
    if(rx_data.RxBuff[5] != 0x5a)
        return 0;
    if(rx_data.RxBuff[6] != 0x5a)
        return 0;
    if(rx_data.RxBuff[2] != DEVICE_ADDR) //读取设备地址
        return 0;
    uint8_t crc16 = CRC16(rx_data.RxBuff, 7);
    if(rx_data.RxBuff[7] != crc16)
        return 0;
    uint8_t TxData[7] = {0xa5, 0xa5, DEVICE_ADDR, 0x20, 0x5a, 0x5a, 0x00};
    //·RS485_BufSend(TxData, 7);
    return rx_data.RxBuff[4];
}

void MOTOR_DirectionControl(uint8_t mode)
{
    static  uint8_t motor1_state = 0;
    static  uint8_t motor2_state = 0;
    uint16_t sw_pin_num =  SW_PinRead();

    if(mode == MOTOR_FOREWORD)
    {
        motor1_state = 1;
        motor2_state = 1;
    }
    else if(mode == MOTOR_REVERSAL)
    {
        motor1_state = 2;
        motor2_state = 2;
    }
    if(motor1_state == 1)
    {
        if(HAL_GPIO_ReadPin(S1_GPIO_Port, S1_Pin) == 0)
        {
            motor1_state = 0;
            return;
        }
        HAL_GPIO_WritePin(M1A_GPIO_Port, M1A_Pin, GPIO_PIN_SET);
        HAL_GPIO_WritePin(M1B_GPIO_Port, M1B_Pin, GPIO_PIN_RESET);
        if(sw_pin_num == S1_Pin)
            motor1_state = 3;
    }
    if(motor1_state == 2)
    {
        if(HAL_GPIO_ReadPin(S2_GPIO_Port, S2_Pin) == 0)
        {
            motor1_state = 0;
            return;
        }
        HAL_GPIO_WritePin(M1A_GPIO_Port, M1A_Pin, GPIO_PIN_RESET);
        HAL_GPIO_WritePin(M1B_GPIO_Port, M1B_Pin, GPIO_PIN_SET);
        if(sw_pin_num == S2_Pin)
            motor1_state = 3;
    }
    if(motor1_state == 3)
    {
        HAL_GPIO_WritePin(M1A_GPIO_Port, M1A_Pin, GPIO_PIN_SET);
        HAL_GPIO_WritePin(M1B_GPIO_Port, M1B_Pin, GPIO_PIN_SET);
        motor1_state = 0;
    }

    if(motor2_state == 1)
    {
        if(HAL_GPIO_ReadPin(S3_GPIO_Port , S3_Pin) == 0)
        {
            motor2_state = 0;
            return;
        }
        HAL_GPIO_WritePin(M2A_GPIO_Port, M2A_Pin, GPIO_PIN_SET);
        HAL_GPIO_WritePin(M2B_GPIO_Port, M2B_Pin, GPIO_PIN_RESET);
        if(sw_pin_num == S3_Pin)
            motor2_state = 3;
    }
    if(motor2_state == 2)
    {
        HAL_GPIO_WritePin(M2A_GPIO_Port, M2A_Pin, GPIO_PIN_RESET);
        HAL_GPIO_WritePin(M2B_GPIO_Port, M2B_Pin, GPIO_PIN_SET);
    }
    if(motor2_state == 3)
    {
        HAL_GPIO_WritePin(M2A_GPIO_Port, M2A_Pin, GPIO_PIN_SET);
        HAL_GPIO_WritePin(M2B_GPIO_Port, M2B_Pin, GPIO_PIN_SET);
        motor2_state = 0;
    }
}

uint16_t CRC16(uint8_t *start_byte, uint16_t num_bytes)
{
    uint16_t check_sum = 0;
    while (num_bytes--)
    {
        check_sum += *start_byte++;
    }
    return check_sum;
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
