/**
 ******************************************************************************
 * @file           : main.c
 * @author         : Neeraj
 * @brief          : GPIO Driver Test - LED Blinking on STM32F446RE Nucleo Board
 ******************************************************************************
 * @attention
 *
 * Copyright (c) 2025 STMicroelectronics.
 * All rights reserved.
 *
 * This software is licensed under terms that can be found in the LICENSE file
 * in the root directory of this software component.
 * If no LICENSE file comes with this software, it is provided AS-IS.
 *
 ******************************************************************************
 */

#include <stdint.h>
#include <stdio.h>
#include "stm32f446xx.h"
#include "stm32f446xx_gpio_driver.h"

#if !defined(__SOFT_FP__) && defined(__ARM_FP)
  #warning "FPU is not initialized, but the project is compiling for an FPU. Please initialize the FPU before use."
#endif

/**
 * @brief  Simple delay function
 * @param  count: Number of iterations for delay
 * @retval None
 */
void delay(void)
{
	// Simple delay loop (approximately 500ms at 16MHz)
	// This is a blocking delay - not accurate but sufficient for LED blinking
	for(uint32_t i = 0; i < 500000; i++);
}

int main(void)
{
	printf("Hello World - LED Blinking Program Started\n");
	
	GPIO_Handle_t GpioLed;
	
	// LED2 on Nucleo F446RE board is connected to PA5
	GpioLed.pGPIOx = GPIOA;
	GpioLed.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_5;
	GpioLed.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_OUT;
	GpioLed.GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPEED_FAST;
	GpioLed.GPIO_PinConfig.GPIO_PinOPType = GPIO_OP_TYPE_PP;
	GpioLed.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_NO_PUPD;
	
	// Initialize the GPIO pin
	GPIO_Init(&GpioLed);
	
	printf("GPIO Configuration Complete - Starting LED Blink\n");
	
    /* Loop forever and blink LED */
	while(1)
	{
		GPIO_ToggleOutputPin(GPIOA, GPIO_PIN_NO_5);
		delay();
	}
}
