/*
 * bt_controller.h
 *
 * Author: Louis Mo, Ming Tsang, Spartey Chen
 * Copyright (c) 2014 HKUST SmartCar Team
 */

#include <libsc/k60/system_timer.h>
#include <libsc/k60/timer.h>

#include "linear_ccd/bt_controller.h"
#include "linear_ccd/car.h"

using namespace libsc::k60;

#define SPEED_CTRL_FREQ 21

namespace linear_ccd
{

BtController::BtController(Car *car)
		: m_is_activated(false), m_prev(0), m_speed_control(car)
{}

bool BtController::Control()
{
	char ch;
	if (m_car->UartPeekChar(&ch) && HandleInput(ch))
	{
		if (!m_is_activated)
		{
			m_is_activated = true;
			m_speed_control.OnFinishWarmUp();
		}
		m_prev = SystemTimer::Time();
		return true;
	}
	else if (m_is_activated)
	{
		SpeedControlPass();
		if (Timer::TimeDiff(SystemTimer::Time(), m_prev) > 1000)
		{
			m_is_activated = false;
			m_car->StopMotor();
		}
	}
	return false;
}

bool BtController::HandleInput(const char ch)
{
	switch (ch)
	{
	case 'w':
		m_car->SetTurning(0);
		m_speed_control.SetMode(5);
		return true;

	case 'a':
		m_car->SetTurning(-75);
		return true;

	case 'd':
		m_car->SetTurning(75);
		return true;

	case 's':
		m_car->SetTurning(0);
		m_speed_control.SetMode(6);
		return true;

	case 'W':
		m_car->SetTurning(0);
		m_speed_control.SetMode(7);
		return true;

	case 'A':
		m_car->SetTurning(-75);
		return true;

	case 'D':
		m_car->SetTurning(75);
		return true;

	case 'S':
		m_car->SetTurning(0);
		m_speed_control.SetMode(8);
		return true;

	case 'e':
	case 'E':
		m_car->SetTurning(0);
		return true;

	case 'q':
	case 'Q':
		m_speed_control.SetMode(0);
		m_car->StopMotor();
		return true;

	default:
		return false;
	}
}

void BtController::SpeedControlPass()
{
	const Timer::TimerInt time = SystemTimer::Time();
	if (Timer::TimeDiff(time, m_prev_speed_control_time) >= SPEED_CTRL_FREQ)
	{
		m_prev_speed_control_time = time;
		m_speed_control.Control();
	}
}

}
