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
#include "can.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdio.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

#define MAX_CAN_MESSAGES 10

typedef struct {
  uint32_t id;          // 消息ID
  uint8_t data[8];     // 消息数据
} CAN_Message;

CAN_Message canMessageArray[MAX_CAN_MESSAGES];

uint8_t messageIndex = 0; // 当前存储的消息索引
/*
extern CAN_RxHeaderTypeDef rx_header;*/
extern CAN_TxHeaderTypeDef tx_header;
extern uint8_t rx_data[8];
extern uint8_t tx_data[8];


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

float ratio_;
float angle_; // deg 输出端累计转动角度
float delta_angle_; // deg 输出端新转动的角度
float ecd_angle_; // deg 当前电机编码器角度
float last_ecd_angle_; // deg 上次电机编码器角度
float delta_ecd_angle_; // deg 编码器端新转动的角度
float rotate_speed_;
float temp_;
float current_;
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */


void CAN1_Init(void) {

  // 配置CAN过滤器
  CAN_FilterTypeDef sFilterConfig;
  sFilterConfig.FilterBank = 0;
  sFilterConfig.FilterMode = CAN_FILTERMODE_IDMASK;  // 使用掩码模式
  sFilterConfig.FilterScale = CAN_FILTERSCALE_32BIT; // 32位过滤器
  sFilterConfig.FilterIdHigh = 0x0000;               // 过滤器ID
  sFilterConfig.FilterIdLow = 0x0000;
  sFilterConfig.FilterMaskIdHigh = 0x0000;           // 过滤器掩码
  sFilterConfig.FilterMaskIdLow = 0x0000;
  sFilterConfig.FilterFIFOAssignment = CAN_RX_FIFO0; // 分配到FIFO0
  sFilterConfig.FilterActivation = ENABLE;           // 启用过滤器
  sFilterConfig.SlaveStartFilterBank = 14;           // 备用过滤器

  // 配置CAN过滤器
  if (HAL_CAN_ConfigFilter(&hcan1, &sFilterConfig) != HAL_OK) {
    // 过滤器配置错误处理
    Error_Handler();
  }

  // 启动CAN外设
  if (HAL_CAN_Start(&hcan1) != HAL_OK) {
    // 启动错误处理
    Error_Handler();
  }

  // 启用接收FIFO0消息挂起中断
  if (HAL_CAN_ActivateNotification(&hcan1, CAN_IT_RX_FIFO0_MSG_PENDING) != HAL_OK) {
    // 中断激活错误处理
    Error_Handler();
  }
}

float linearMapping(int in,int in_min,int in_max,float out_min,float out_max) {
    	return (out_max-out_min)*(float)(in-in_min)/(float)(in_max-in_min)+out_min;
}

void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan) {
  CAN_RxHeaderTypeDef rx_header;
  uint8_t rx_data[8];
  HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, &rx_header,rx_data) ;
  uint16_t ecd_angle_mid = (rx_data[0] << 8) | rx_data[1];
  ecd_angle_ = linearMapping (ecd_angle_mid, 0, 8191,  0.0, 360) ;
  uint16_t rotate_speed_mid = (rx_data[2] << 8) | rx_data[3];
  rotate_speed_ = (float)(rotate_speed_mid * 6);
  int16_t current_mid = (rx_data[4] << 8) | rx_data[5];
  current_ = linearMapping (current_mid, -16384, 16384,  -20.0,20.0) ;
  uint8_t temp_mid = rx_data[6];
  temp_ = (float)(temp_mid);

  printf("Motor Speed: %d RPM, Current: %d mA\n", rotate_speed_, current_);

}

//UART
/*
UART_HandleTypeDef huart1;


void UART1_Init(void) {
  huart1.Instance = USART1;
  huart1.Init.BaudRate = 115200; // 设置波特率
  huart1.Init.WordLength = UART_WORDLENGTH_8B; // 字长8位
  huart1.Init.StopBits = UART_STOPBITS_1; // 停止位1
  huart1.Init.Parity = UART_PARITY_NONE; // 无奇偶校验
  huart1.Init.Mode = UART_MODE_TX_RX; // 发送和接收模式
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE; // 无流控制
  huart1.Init.OverSampling = UART_OVERSAMPLING_16; // 过采样16倍

  // 初始化UART
  if (HAL_UART_Init(&huart1) != HAL_OK) {
    // 初始化错误处理
    Error_Handler();
  }
}*/



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
  MX_CAN1_Init();
  MX_USART6_UART_Init();
  /* USER CODE BEGIN 2 */

  // HAL_CAN_ConfigFilter(&hcan1, CAN_RX_FIFO0);
  // HAL_CAN_Start(&hcan1);
  // HAL_CAN_ActivateNotification(&hcan1, CAN_IT_RX_FIFO0_MSG_PENDING);

  CAN1_Init();

  /* USER CODE END 2 */

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

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 6;
  RCC_OscInitStruct.PLL.PLLN = 180;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 4;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Activate the Over-Drive mode
  */
  if (HAL_PWREx_EnableOverDrive() != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */
int _write(int file,char * ch, int len) {
  HAL_UART_Transmit(&huart6, (uint8_t *)ch, 1,  10);
  return 1;

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
