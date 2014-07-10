/*
 * beep_manager.h
 *
 * Author: Louis Mo, Ming Tsang, Spartey Chen
 * Copyright (c) 2014 HKUST SmartCar Team
 */

#ifndef LINEAR_CCD_BEEP_MANAGER_H_
#define LINEAR_CCD_BEEP_MANAGER_H_

#include <libsc/k60/timer.h>

namespace libsc
{
namespace k60
{

class SimpleBuzzer;

}
}

namespace linear_ccd
{

class BeepManager
{
public:
	static BeepManager* GetInstance(libsc::k60::SimpleBuzzer *const buzzer);
	static BeepManager* GetInstance()
	{
		return m_instance;
	}

	void Beep(const libsc::k60::Timer::TimerInt duration);
	void Process();

private:
	explicit BeepManager(libsc::k60::SimpleBuzzer *const buzzer);

	libsc::k60::SimpleBuzzer *const m_buzzer;
	libsc::k60::Timer::TimerInt m_start;
	libsc::k60::Timer::TimerInt m_duration;
	bool m_is_beep;

	static BeepManager *m_instance;
};

}

#endif /* LINEAR_CCD_BEEP_MANAGER_H_ */
