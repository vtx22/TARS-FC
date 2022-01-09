#include "Motors.hpp"

MOTORS::MOTORS(TIM_HandleTypeDef* timer) : _timer(timer)
{
	startPWMChannels();
}
void MOTORS::calibrateMotors()
{


	setAllMotors(100, 100, 100, 100);
	HAL_Delay(4000);
	setAllMotorsOFF();


}

void MOTORS::setAllMotors(float pwm1, float pwm2, float pwm3, float pwm4)
{
	_pwm1 = pwm1;
	_pwm2 = pwm2;
	_pwm3 = pwm3;
	_pwm4 = pwm4;

	updateMotors();
}

void MOTORS::initMotors()
{
	setMotor(1, 20);
	setMotor(2, 20);
	setMotor(3, 20);
	setMotor(4, 20);
	updateMotors();
	HAL_Delay(2000);
	setAllMotorsOFF();
}

void MOTORS::startPWMChannels()
{
	HAL_TIM_Base_Start(_timer);
	HAL_TIM_PWM_Start(_timer, TIM_CHANNEL_1);
	HAL_TIM_PWM_Start(_timer, TIM_CHANNEL_2);
	HAL_TIM_PWM_Start(_timer, TIM_CHANNEL_3);
	HAL_TIM_PWM_Start(_timer, TIM_CHANNEL_4);
	setAllMotorsOFF();
}

uint32_t MOTORS::cycle2compare(float cycle)
{
	if(!_enabled) {return 0;}
	if(cycle > 100) {cycle = 100;}
	if(cycle < 0) {cycle = 0;}
	return (uint32_t)(MIN_CYCLE + (MAX_CYCLE - MIN_CYCLE) * cycle / 100.f);
}

void MOTORS::setMotor(uint8_t id, float pwm)
{
	switch(id)
	{
	case 1:
		_pwm1 = pwm;
		break;
	case 2:
		_pwm2 = pwm;
		break;
	case 3:
		_pwm3 = pwm;
		break;
	case 4:
		_pwm4 = pwm;
		break;
	}
}

void MOTORS::disableMotors()
{
	_enabled = false;
	setAllMotorsOFF();
}

void MOTORS::enableMotors()
{
	_enabled = true;
}

void MOTORS::setAllMotorsOFF()
{
	setMotor(1, 0);
	setMotor(2, 0);
	setMotor(3, 0);
	setMotor(4, 0);
	updateMotors();
}

void MOTORS::updateMotors()
{
	__HAL_TIM_SET_COMPARE(_timer, TIM_CHANNEL_1, cycle2compare(_pwm1));
	__HAL_TIM_SET_COMPARE(_timer, TIM_CHANNEL_2, cycle2compare(_pwm2));
	__HAL_TIM_SET_COMPARE(_timer, TIM_CHANNEL_3, cycle2compare(_pwm3));
	__HAL_TIM_SET_COMPARE(_timer, TIM_CHANNEL_4, cycle2compare(_pwm4));
}
