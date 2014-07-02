/*
 * beep_manager.h
 *
 * Author: Louis Mo, Ming Tsang, Spartey Chen
 * Copyright (c) 2014 HKUST SmartCar Team
 */

#ifndef LINEAR_CCD_BEEP_MANAGER_H_
#define LINEAR_CCD_BEEP_MANAGER_H_

#include <libsc/k60/timer.h>

#include "linear_ccd/car.h"

namespace linear_ccd
{

class BeepManager
{
public:
	static BeepManager* GetInstance(Car *const car);
	static BeepManager* GetInstance()
	{
		return m_instance;
	}

	void Beep(const libsc::k60::Timer::TimerInt duration);
	void Process();

private:
	explicit BeepManager(Car *const car);

	Car *const m_car;
	libsc::k60::Timer::TimerInt m_start;
	libsc::k60::Timer::TimerInt m_duration;
	bool m_is_beep;

	static BeepManager *m_instance;
};

}

#endif /* LINEAR_CCD_BEEP_MANAGER_H_ */
