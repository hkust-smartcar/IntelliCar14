/*
 * bt_controller.h
 *
 * Author: Louis Mo, Ming Tsang, Spartey Chen
 * Copyright (c) 2014 HKUST SmartCar Team
 */

#ifndef LINEAR_CCD_BT_CONTROLLER_H_
#define LINEAR_CCD_BT_CONTROLLER_H_

#include <libutil/clock.h>

#include "linear_ccd/speed_control_1.h"

namespace linear_ccd
{

class Car;

}

namespace linear_ccd
{

class BtController
{
public:
	BtController();

	bool Control(Car *car);

private:
	bool HandleInput(const char ch, Car *car);
	void SpeedControlPass(Car *car);

	bool m_is_activated;
	libutil::Clock::ClockInt m_prev;
	SpeedControl1 m_speed_control;
	libutil::Clock::ClockInt m_prev_speed_control_time;
};

}

#endif /* LINEAR_CCD_BT_CONTROLLER_H_ */
