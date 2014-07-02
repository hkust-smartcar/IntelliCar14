/*
 * beep_manager.cpp
 *
 * Author: Louis Mo, Ming Tsang, Spartey Chen
 * Copyright (c) 2014 HKUST SmartCar Team
 */

#include <libsc/k60/system_timer.h>
#include <libsc/k60/timer.h>

#include "linear_ccd/beep_manager.h"
#include "linear_ccd/car.h"

using namespace libsc::k60;

namespace linear_ccd
{

BeepManager *BeepManager::m_instance = nullptr;

BeepManager::BeepManager(Car *const car)
		: m_car(car), m_start(0), m_duration(0), m_is_beep(false)
{}

BeepManager* BeepManager::GetInstance(Car *const car)
{
	if (!m_instance)
	{
		m_instance = new BeepManager(car);
	}
	return m_instance;
}

void BeepManager::Beep(const Timer::TimerInt duration)
{
	m_car->SetBuzzerBeep(true);
	m_duration = duration;
	m_start = SystemTimer::Time();
	m_is_beep = true;
}

void BeepManager::Process()
{
	if (!m_is_beep)
	{
		return;
	}

	if (Timer::TimeDiff(SystemTimer::Time(), m_start) >= m_duration)
	{
		m_car->SetBuzzerBeep(false);
		m_is_beep = false;
	}
}

}
