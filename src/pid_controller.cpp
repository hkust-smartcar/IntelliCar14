/*
 * pid_controller.cpp
 * Generic PID controller
 *
 * Author: Ming Tsang
 * Copyright (c) 2014 HKUST SmartCar Team
 */

#include <cstdint>

#include <libutil/clock.h>

#include "pid_controller.h"

using libutil::Clock;

PidController::PidController(const uint16_t setpoint, const float kp,
		const float ki, const float kd)
		: m_setpoint(setpoint), m_kp(kp), m_ki(ki), m_kd(kd),
		  m_accumulated_error(0), m_prev_error(0), m_prev_time(Clock::Time())
{}

uint16_t PidController::Calc(const Clock::ClockInt time,
		const uint16_t current_val)
{
	const Clock::ClockInt time_diff = Clock::TimeDiff(time, m_prev_time);
	const uint32_t error = m_setpoint - current_val;

	const float p = m_kp * error;
	m_accumulated_error += error;
	const float i = m_ki * m_accumulated_error;
	const float slope = (error - m_prev_error) / (float)time_diff;
	const float d = m_kd * slope;

	m_prev_error = error;
	m_prev_time = time;
	return p + i + d;
}
