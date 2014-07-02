/*
 * bt_controller.h
 *
 * Author: Louis Mo, Ming Tsang, Spartey Chen
 * Copyright (c) 2014 HKUST SmartCar Team
 */

#ifndef LINEAR_CCD_BT_CONTROLLER_H_
#define LINEAR_CCD_BT_CONTROLLER_H_

#include <libsc/k60/timer.h>

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
	explicit BtController(Car *car);

	bool Control();

private:
	bool HandleInput(const char ch);
	void SpeedControlPass();

	Car *m_car;
	bool m_is_activated;
	libsc::k60::Timer::TimerInt m_prev;
	SpeedControl1 m_speed_control;
	libsc::k60::Timer::TimerInt m_prev_speed_control_time;
};

}

#endif /* LINEAR_CCD_BT_CONTROLLER_H_ */
