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
#include "linear_ccd/turn_hint.h"

namespace linear_ccd
{

class Car;

}

namespace linear_ccd
{

class SpeedControl1 : public SpeedControlStrategy
{
public:
	explicit SpeedControl1(Car *car);

	void OnFinishWarmUp() override;
	void Control() override;

	void SetMode(const Uint mode) override
	{
		m_mode = mode;
	}

	void SetTurnHint(const TurnHint hint);

private:
	Car *m_car;
	libutil::PidController<int32_t, int32_t> m_pid;
	uint8_t m_mode;
	uint8_t m_straight_mode_delay;

	bool m_is_startup;
	int m_reverse_count;
};

}

#endif /* LINEAR_CCD_SPEED_CONTROL_1_H_ */
