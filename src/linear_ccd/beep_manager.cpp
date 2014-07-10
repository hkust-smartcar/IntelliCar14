/*
 * beep_manager.cpp
 *
 * Author: Louis Mo, Ming Tsang, Spartey Chen
 * Copyright (c) 2014 HKUST SmartCar Team
 */

#include <libsc/k60/simple_buzzer.h>
#include <libsc/k60/system.h>
#include <libsc/k60/timer.h>

#include "linear_ccd/beep_manager.h"

using namespace libsc::k60;

namespace linear_ccd
{

BeepManager *BeepManager::m_instance = nullptr;

BeepManager::BeepManager(SimpleBuzzer *const buzzer)
		: m_buzzer(buzzer),
		  m_start(0),
		  m_duration(0),
		  m_is_beep(false)
{}

BeepManager* BeepManager::GetInstance(SimpleBuzzer *const buzzer)
{
	if (!m_instance)
	{
		m_instance = new BeepManager(buzzer);
	}
	return m_instance;
}

void BeepManager::Beep(const Timer::TimerInt duration)
{
	m_buzzer->SetBeep(true);
	m_duration = duration;
	m_start = System::Time();
	m_is_beep = true;
}

void BeepManager::Process()
{
	if (!m_is_beep)
	{
		return;
	}

	if (Timer::TimeDiff(System::Time(), m_start) >= m_duration)
	{
		m_buzzer->SetBeep(false);
		m_is_beep = false;
	}
}

}
