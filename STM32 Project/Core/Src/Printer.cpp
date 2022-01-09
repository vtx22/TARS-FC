#include "Printer.hpp"

PRINTER::PRINTER(UART_HandleTypeDef* huart)
{
	_huart = huart;
}

void PRINTER::printFloat(float value)
{
	__disable_irq();
	char temp[25];
	uint8_t data[25];

	int cnt = sprintf(temp, "%f", value);

	for(int i = 0; i < cnt + 1; i++)
	{
		data[i] = temp[i];
	}

	HAL_UART_Transmit(_huart, data, cnt+1, 100);
	println();
	__enable_irq();
}

void PRINTER::printString(std::string str)
{
	__disable_irq();
	std::vector<uint8_t> vec(str.begin(), str.end());
	uint8_t *data = &vec[0];
	data[str.length()] = '\0';

	HAL_UART_Transmit(_huart, data, str.length() + 1, 100);
	println();
	__enable_irq();
}

void PRINTER::println()
{
	uint8_t nl[2];
	nl[0] = '\n';
	nl[1] = '\r';
	HAL_UART_Transmit(_huart, nl, 2, 100);
}
