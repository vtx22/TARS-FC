#pragma once
#include <stdint.h>
#include "stm32f1xx_hal.h"

#define MIN_CYCLE 1000
#define MAX_CYCLE 2000

class MOTORS
{
public:
	MOTORS(TIM_HandleTypeDef* timer);

	void setAllMotorsOFF();
	void setMotor(uint8_t id, float pwm);
	void setAllMotors(float pwm1, float pwm2, float pwm3, float pwm4);
	void enableMotors();
	void disableMotors();
	void updateMotors();
	void initMotors();
	void calibrateMotors();
private:
	void startPWMChannels();
	uint32_t cycle2compare(float cycle);


	float _pwm1 = 0;
	float _pwm2 = 0;
	float _pwm3 = 0;
	float _pwm4 = 0;




	bool _enabled = false;

	TIM_HandleTypeDef* _timer;

};
