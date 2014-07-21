/*
 * manual_program_4.h
 *
 * Author: Louis Mo, Ming Tsang, Spartey Chen
 * Copyright (c) 2014 HKUST SmartCar Team
 */

#ifndef LINEAR_CCD_MANUAL_PROGRAM_4_H_
#define LINEAR_CCD_MANUAL_PROGRAM_4_H_

#include <libsc/k60/timer.h>

#include "linear_ccd/car.h"
#include "linear_ccd/dir_control_algorithm_4.h"
#include "linear_ccd/median_ccd_filter.h"
#include "linear_ccd/program.h"

namespace linear_ccd
{

class ManualProgram4 : public Program
{
public:
	ManualProgram4();
	~ManualProgram4();

	void Run() override;

private:
	struct SpeedState
	{
		libsc::k60::Timer::TimerInt prev_run = 0;
		bool is_triggered = false;
		bool is_stopping = false;
		libsc::k60::Timer::TimerInt trigger_time = 0;
	};

	struct TurnState
	{
		libsc::k60::Timer::TimerInt prev_run = 0;
	};

	void ServoPass();
	void SpeedControlPass();

	void TuningStage();

	void LcdRedraw();

	static int FwriteHandler(int, char *ptr, int len);
	static void HardFaultHandler(void);

	Car m_car;

	SpeedState m_speed_state;
	TurnState m_turn_state;

	MedianCcdFilter m_ccd_filter;

	DirControlAlgorithm4 m_dir_control;

	libsc::k60::Timer::TimerInt m_start;

	static ManualProgram4 *m_instance;
};

}

#endif /* LINEAR_CCD_MANUAL_PROGRAM_4_H_ */
