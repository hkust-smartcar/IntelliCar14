/*
 * linear_ccd_app.h
 * Linear CCD App
 *
 * Author: Louis Mo, Ming Tsang, Spartey Chan
 * Copyright (c) 2014 HKUST SmartCar Team
 */

#ifndef LINEAR_CCD_APP_H_
#define LINEAR_CCD_APP_H_

#include "linear_ccd/car.h"

#include "pid_controller.h"

namespace linear_ccd
{

class LinearCcdApp
{
public:
	LinearCcdApp();
	~LinearCcdApp();

	void Run();

	static int FwriteHandler(int, char *ptr, int len);

private:
	struct LedState
	{
		libutil::Clock::ClockInt prev_run;
		bool flag;

		LedState()
		{
			prev_run = 0;
			flag = false;
		}
	};

	struct SpeedControlState
	{
		libutil::Clock::ClockInt prev_run;
		uint32_t prev_speed;
		uint16_t prev_power;

		SpeedControlState()
		{
			prev_run = 0;
			prev_speed = 0;
			prev_power = 0;
		}
	};

	void LedPass();
	void SpeedControlPass();

	Car m_car;
	PidController m_pid_controller;

	LedState m_led_state;
	SpeedControlState m_speed_control_state;

	static LinearCcdApp *m_instance;
};

}

#endif /* LINEAR_CCD_APP_H_ */
