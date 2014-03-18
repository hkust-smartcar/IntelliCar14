/*
 * linear_ccd_app.h
 * Linear CCD App
 *
 * Author: Louis Mo, Ming Tsang, Spartey Chen
 * Copyright (c) 2014 HKUST SmartCar Team
 */

#ifndef LINEAR_CCD_APP_H_
#define LINEAR_CCD_APP_H_

#include "linear_ccd/car.h"

#include "libutil/clock.h"

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
				: prev_run(0), flag(false)
		{}
	};

	struct ServoState
	{
		libutil::Clock::ClockInt prev_run;

		ServoState()
				: prev_run(0)
		{}
	};

	struct SpeedState
	{
		libutil::Clock::ClockInt prev_run;
		libutil::PidController<uint32_t, uint16_t> pid;


		SpeedState();
	};

	void LedPass();
	void ServoPass();
	void SpeedControlPass();
	void Algorithm(const bool *ccd_data);
	void ccd_scan_all_white_or_all_black_sample(const bool *ccd_data);
	void ccd_print(const bool *ccd_data);

	Car m_car;

	LedState m_led_state;
	ServoState m_servo_state;
	SpeedState m_speed_state;

	static LinearCcdApp *m_instance;
};

}

#endif /* LINEAR_CCD_APP_H_ */
