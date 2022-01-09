#pragma once
#include "stm32f1xx_hal.h"
#include <stdint.h>
#include <stdio.h>
#include <string>
#include <vector>

class PRINTER
{
	public:
		PRINTER(UART_HandleTypeDef* huart);
		void printFloat(float value);
		void printString(std::string str);
		void println();

	private:
		UART_HandleTypeDef* _huart;
};
