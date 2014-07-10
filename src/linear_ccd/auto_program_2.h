/*
 * auto_program_2.h
 *
 * Author: Louis Mo, Ming Tsang, Spartey Chen
 * Copyright (c) 2014 HKUST SmartCar Team
 */

#ifndef LINEAR_CCD_AUTO_PROGRAM_2_H_
#define LINEAR_CCD_AUTO_PROGRAM_2_H_

#include <libsc/k60/timer.h>

#include "linear_ccd/car.h"
#include "linear_ccd/dir_control_algorithm_2.h"
#include "linear_ccd/median_ccd_filter.h"
#include "linear_ccd/program.h"
#include "linear_ccd/speed_control_2.h"

namespace linear_ccd
{

class AutoProgram2 : public Program
{
public:
	AutoProgram2();
	~AutoProgram2();

	void Run() override;

private:
	struct EmergencyStopState
	{
		libsc::k60::Timer::TimerInt trigger_time = 0;
		bool is_triggered = false;
	};

	struct JoystickState
	{
		libsc::k60::Timer::TimerInt prev_run = 0;
		uint8_t delay = 0;
		uint8_t page = 0;
	};

	struct LedState
	{
		libsc::k60::Timer::TimerInt prev_run = 0;
		bool flag = false;
	};

	struct TurnState
	{
		libsc::k60::Timer::TimerInt prev_run = 0;
	};

	struct SpeedState
	{
		libsc::k60::Timer::TimerInt prev_run = 0;
	};

	void LedPass();
	void JoystickPass();
	void ServoPass();
	void SpeedControlPass();

	void DetectEmergencyStop();

	void TuningStage();
	void CountDownStage();

	void LcdRedraw();

	static int FwriteHandler(int, char *ptr, int len);
	static void HardFaultHandler(void);

	Car m_car;

	EmergencyStopState m_emergency_stop_state;
	JoystickState m_joystick_state;
	LedState m_led_state;
	SpeedState m_speed_state;
	TurnState m_turn_state;

	MedianCcdFilter m_ccd_filter;

	DirControlAlgorithm2 m_dir_control;
	SpeedControl2 m_speed_control;

	libsc::k60::Timer::TimerInt m_start;
	bool m_is_stop;

	static AutoProgram2 *m_instance;
};

}

#endif /* LINEAR_CCD_AUTO_PROGRAM_2_H_ */
