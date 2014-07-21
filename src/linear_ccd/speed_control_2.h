/*
 * speed_control_2.h
 * Speed controller Gen2, using PID
 *
 * Author: Louis Mo, Ming Tsang, Spartey Chen
 * Copyright (c) 2014 HKUST SmartCar Team
 */

#ifndef LINEAR_CCD_SPEED_CONTROL_2_H_
#define LINEAR_CCD_SPEED_CONTROL_2_H_

#include <cstdint>

#include <libbase/k60/misc_utils.h>
#include <libsc/k60/timer.h>
#include <libutil/pid_controller.h>

#include "linear_ccd/speed_control_strategy.h"
#include "linear_ccd/turn_hint.h"

namespace linear_ccd
{

class Car;

}

namespace linear_ccd
{

class SpeedControl2 : public SpeedControlStrategy
{
public:
	struct Parameter
	{
		int sp = 0;
		float kp = 0;
		float ki = 0;
		float kd = 0;
		int turn_sp = 0;
	};

	explicit SpeedControl2(Car *const car);
	SpeedControl2(Car *const car, const Parameter &parameter);

	void SetParameter(const Parameter &parameter);
	const Parameter& GetParameter() const
	{
		return m_parameter;
	}

	void OnFinishWarmUp() override;
	int Control() override;

	void SetTurnHint(const TurnHint hint) override;

	float GetP() const
	{
		return m_pid.GetP();
	}

	float GetI() const
	{
		return m_pid.GetI();
	}

	float GetD() const
	{
		return m_pid.GetD();
	}

private:
	Car *const m_car;
	Parameter m_parameter;
	libutil::PidController<int32_t, int32_t> m_pid;
	uint8_t m_straight_mode_delay;

	libsc::k60::Timer::TimerInt m_start_time;
	bool m_is_startup;
	int m_reverse_count;

	bool m_is_permissive_reverse;
};

}

#endif /* LINEAR_CCD_SPEED_CONTROL_1_H_ */
