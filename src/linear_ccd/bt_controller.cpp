/*
 * bt_controller.h
 *
 * Author: Louis Mo, Ming Tsang, Spartey Chen
 * Copyright (c) 2014 HKUST SmartCar Team
 */

#include <libutil/clock.h>

#include "linear_ccd/bt_controller.h"
#include "linear_ccd/car.h"

using libutil::Clock;

#define SPEED_CTRL_FREQ 21

namespace linear_ccd
{

BtController::BtController()
		: m_is_activated(false), m_prev(0)
{}

bool BtController::Control(Car *car)
{
	char ch;
	if (car->UartPeekChar(&ch) && HandleInput(ch, car))
	{
		if (!m_is_activated)
		{
			m_is_activated = true;
			m_speed_control.OnFinishWarmUp(car);
		}
		m_prev = Clock::Time();
		return true;
	}
	else if (m_is_activated)
	{
		SpeedControlPass(car);
		if (Clock::TimeDiff(Clock::Time(), m_prev) > 1000)
		{
			m_is_activated = false;
			car->StopMotor();
		}
	}
	return false;
}

bool BtController::HandleInput(const char ch, Car *car)
{
	switch (ch)
	{
	case 'w':
		car->SetTurning(0);
		m_speed_control.SetMode(5);
		return true;

	case 'a':
		car->SetTurning(-75);
		return true;

	case 'd':
		car->SetTurning(75);
		return true;

	case 's':
		car->SetTurning(0);
		m_speed_control.SetMode(6);
		return true;

	case 'W':
		car->SetTurning(0);
		m_speed_control.SetMode(7);
		return true;

	case 'A':
		car->SetTurning(-75);
		return true;

	case 'D':
		car->SetTurning(75);
		return true;

	case 'S':
		car->SetTurning(0);
		m_speed_control.SetMode(8);
		return true;

	case 'e':
	case 'E':
		car->SetTurning(0);
		return true;

	case 'q':
	case 'Q':
		m_speed_control.SetMode(0);
		car->StopMotor();
		return true;

	default:
		return false;
	}
}

void BtController::SpeedControlPass(Car *car)
{
	const Clock::ClockInt time = Clock::Time();
	if (Clock::TimeDiff(time, m_prev_speed_control_time) >= SPEED_CTRL_FREQ)
	{
		m_prev_speed_control_time = time;
		m_speed_control.Control(car);
	}
}

}
