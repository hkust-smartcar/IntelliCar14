/*
 * manual_program.h
 *
 * Author: Louis Mo, Ming Tsang, Spartey Chen
 * Copyright (c) 2014 HKUST SmartCar Team
 */

#ifndef LINEAR_CCD_MANUAL_PROGRAM_H_
#define LINEAR_CCD_MANUAL_PROGRAM_H_

namespace linear_ccd
{

class ManualProgram : public Program
{
public:
	ManualProgram();
	~ManualProgram();

	void Run() override;

private:
	struct ServoState
	{
		libsc::k60::Timer::TimerInt prev_run = 0;
	};

	struct JoystickState
	{
		libsc::k60::Timer::TimerInt prev_run = 0;
		uint8_t delay = 0;
	};

	Car m_car;
};

}

#endif /* LINEAR_CCD_MANUAL_PROGRAM_H_ */
