/*
 * speed_control_2.cpp
 * Speed controller Gen2, using PID
 *
 * Author: Louis Mo, Ming Tsang, Spartey Chen
 * Copyright (c) 2014 HKUST SmartCar Team
 */

#include <climits>
#include <cstdint>

#include <libsc/k60/system.h>
#include <libsc/k60/timer.h>
#include <libutil/misc.h>
#include <libutil/pid_controller.h>
#include <libutil/pid_controller.tcc>

#include "linear_ccd/debug.h"
#include "linear_ccd/config.h"
#include "linear_ccd/beep_manager.h"
#include "linear_ccd/car.h"
#include "linear_ccd/linear_ccd_app.h"
#include "linear_ccd/speed_control_2.h"
#include "linear_ccd/turn_hint.h"

using namespace libsc::k60;
using namespace std;

#define MOTOR_MAX_PWM 7000
#define ACCELERATE_DELAY 15
#define I_LIMIT 3500

namespace linear_ccd
{

SpeedControl2::SpeedControl2(Car *const car)
		: SpeedControl2(car, Parameter())
{}

SpeedControl2::SpeedControl2(Car *const car, const Parameter &parameter)
		: m_car(car),
		  m_parameter(parameter),
		  m_pid(parameter.sp, parameter.kp, parameter.ki, parameter.kd),
		  m_straight_mode_delay(0),
		  m_start_time(0),
		  m_is_startup(true),
		  m_reverse_count(0),
		  m_is_permissive_reverse(false)
{
	m_pid.SetILimit(I_LIMIT);
}

void SpeedControl2::SetParameter(const Parameter &parameter)
{
	m_parameter = parameter;
	m_pid.SetSetpoint(m_parameter.sp);
	m_pid.SetKp(m_parameter.kp);
	m_pid.SetKi(m_parameter.ki);
	m_pid.SetKd(m_parameter.kd);
}

void SpeedControl2::OnFinishWarmUp()
{
	m_start_time = System::Time();
	m_pid.Restart();
}

int SpeedControl2::Control()
{
	m_car->UpdateEncoder();
	const int16_t count = m_car->GetEncoderCount();
	const Timer::TimerInt time = System::Time();
#ifdef DEBUG_PRINT_ENCODER
	iprintf("%d\n", count);
#endif

	int power = m_pid.Calc(time, count);

	// Prevent the output going crazy due to initially idle encoder
	if (m_is_startup && time < 250 + m_start_time)
	{
		const int clamp_power = m_car->GetMotorPower()
				+ libutil::Clamp<int>(-400, power - m_car->GetMotorPower(), 400);
		m_car->SetMotorPower(clamp_power / 10);
		return clamp_power;
	}
	else
	{
		if (m_is_startup)
		{
			m_is_startup = false;
			m_pid.Restart();
		}
/*
		if (abs(power - car->GetMotorPower()) > 3500)
		{
			// Max 3500 diff in one step
			power = libutil::Clamp<int>(car->GetMotorPower() - 3500, power,
					car->GetMotorPower() + 3500);
		}
*/
		if (power < 0)
		{
			if (power >= -6000 || m_is_permissive_reverse)
			{
				m_reverse_count = INT_MAX;
			}

			if (m_reverse_count < (m_pid.GetSetpoint() >> 2))
			{
				if (power >= -3500)
				{
					++m_reverse_count;
				}
				power = 0;
			}
		}
		else
		{
			m_reverse_count = 0;
		}
#ifdef DEBUG_BEEP_REVERSE_MOTOR
		if (power < 0)
		{
			m_car->GetBeepManager()->Beep(100);
		}
#endif
		m_car->SetMotorPower(libutil::Clamp<int>(-MOTOR_MAX_PWM, power,
				MOTOR_MAX_PWM) / 10);
		return power;
	}
}

void SpeedControl2::SetTurnHint(const TurnHint hint)
{
	switch (hint)
	{
	case TurnHint::STRAIGHT:
		if (m_straight_mode_delay == static_cast<uint8_t>(-1))
		{
			m_straight_mode_delay = ACCELERATE_DELAY;
		}
		else if (m_straight_mode_delay == 0)
		{
			m_pid.SetSetpoint(m_parameter.sp);
		}
		else
		{
			--m_straight_mode_delay;
		}
		m_is_permissive_reverse = false;
#ifdef DEBUG_BEEP_SPEED_STRAIGHT
		m_car->GetBeepManager()->Beep(100);
#endif
		break;

	case TurnHint::PRE_TURN:
		break;

	case TurnHint::TURN:
		m_pid.SetSetpoint(m_parameter.turn_sp);
		m_straight_mode_delay = static_cast<uint8_t>(-1);
		m_is_permissive_reverse = true;
		break;
	}
}

}
