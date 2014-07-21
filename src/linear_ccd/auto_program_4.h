/*
 * auto_program_4.h
 *
 * Author: Louis Mo, Ming Tsang, Spartey Chen
 * Copyright (c) 2014 HKUST SmartCar Team
 */

#ifndef LINEAR_CCD_AUTO_PROGRAM_4_H_
#define LINEAR_CCD_AUTO_PROGRAM_4_H_

#include <libsc/k60/timer.h>

#include "linear_ccd/car.h"
#include "linear_ccd/dir_control_algorithm_4.h"
#include "linear_ccd/median_ccd_filter.h"
#include "linear_ccd/program.h"
#include "linear_ccd/speed_control_2.h"

namespace linear_ccd
{

class AutoProgram4 : public Program
{
public:
	AutoProgram4();
	~AutoProgram4();

	void Run() override;

private:
	struct BrakeState
	{
		bool is_brake = false;
		libsc::k60::Timer::TimerInt prev_run = 0;
	};

	struct EmergencyStopState
	{
		libsc::k60::Timer::TimerInt trigger_time = 0;
		bool is_triggered = false;
	};

	struct GoalState
	{
		libsc::k60::Timer::TimerInt trigger_time = 0;
		uint8_t trigger_id = (uint8_t)-1;
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

	void BrakePass();
	void LedPass();
	void JoystickPass();
	void ServoPass();
	void SpeedControlPass();

	void DetectEmergencyStop();

	void TuningStage();
	void CountDownStage();

	void LcdRedraw();

	void OnLightSensorDetectHandler(const uint8_t id);

	static int FwriteHandler(int, char *ptr, int len);
	static void HardFaultHandler(void);

	Car m_car;

	BrakeState m_brake_state;
	EmergencyStopState m_emergency_stop_state;
	GoalState m_goal_state;
	JoystickState m_joystick_state;
	LedState m_led_state;
	SpeedState m_speed_state;
	TurnState m_turn_state;

	MedianCcdFilter m_ccd_filter;

	DirControlAlgorithm4 m_dir_control;
	SpeedControl2 m_speed_control;

	libsc::k60::Timer::TimerInt m_start;
	bool m_is_stop;

	static AutoProgram4 *m_instance;
};

}

#endif /* LINEAR_CCD_AUTO_PROGRAM_4_H_ */
