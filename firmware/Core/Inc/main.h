#ifndef GOFLY_MAIN_H
#define GOFLY_MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>

#ifndef GOFLY_TARGET
#define GOFLY_TARGET 0
#endif

#if GOFLY_TARGET
#include "stm32f4xx_hal.h"

extern SPI_HandleTypeDef hspi1;
extern SPI_HandleTypeDef hspi2;
extern SPI_HandleTypeDef hspi3;
extern I2C_HandleTypeDef hi2c1;
extern UART_HandleTypeDef huart1;
extern UART_HandleTypeDef huart2;
extern UART_HandleTypeDef huart3;
extern UART_HandleTypeDef huart4;
extern ADC_HandleTypeDef hadc1;
extern TIM_HandleTypeDef htim3;
extern TIM_HandleTypeDef htim6;
extern PCD_HandleTypeDef hpcd_USB_FS;
#endif

void SystemClock_Config(void);
void MX_GPIO_Init(void);

#ifdef __cplusplus
}
#endif

#endif
