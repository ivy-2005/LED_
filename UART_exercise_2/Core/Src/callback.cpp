//
// Created by 包辰宸 on 24-10-3.
//
#include "main.h"
#include "gpio.h"
#include "stm32f4xx_hal_tim.h"
#include "usart.h"


extern uint8_t rx_buff[8];
/*
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim) {
    if (htim->Instance == htim6.Instance) {
        HAL_GPIO_TogglePin(red_GPIO_Port, red_Pin);
    }
    else if (htim->Instance == htim1.Instance) {

    }
}*/
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart) {
    if (huart->Instance == huart6.Instance) {
        HAL_UART_Receive_IT(&huart6, rx_buff, 1);
    }
}