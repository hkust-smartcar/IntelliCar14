/*
 * speed_control_1.h
 * Speed controller Gen1, using PID
 *
 * Author: Louis Mo, Ming Tsang, Spartey Chen
 * Copyright (c) 2014 HKUST SmartCar Team
 */

#ifndef LINEAR_CCD_SPEED_CONTROL_1_H_
#define LINEAR_CCD_SPEED_CONTROL_1_H_

#include <mini_common.h>
#include <cstdint>

#include "libutil/pid_controller.h"

#include "linear_ccd/speed_control_strategy.h"

namespace linear_ccd
{

class Car;

}

namespace linear_ccd
{

class SpeedControl1 : public SpeedControlStrategy
{
public:
	SpeedControl1();

	void OnFinishWarmUp(Car *car) override;
	void Control(Car *car) override;

	void SetMode(const Uint mode) override
	{
		m_mode = mode;
	}

private:
	void UpdatePid(const bool is_straight);

	libutil::PidController<int32_t, int> m_pid;
	uint8_t m_mode;

	bool m_is_startup;
};

}

#endif /* LINEAR_CCD_SPEED_CONTROL_1_H_ */
