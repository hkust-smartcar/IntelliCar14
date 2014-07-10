/*
 * auto_program.h
 *
 * Author: Louis Mo, Ming Tsang, Spartey Chen
 * Copyright (c) 2014 HKUST SmartCar Team
 */

#ifndef LINEAR_CCD_AUTO_PROGRAM_H_
#define LINEAR_CCD_AUTO_PROGRAM_H_

#include <cstdint>
#include <bitset>

#include <libsc/k60/linear_ccd.h>
#include <libsc/k60/timer.h>

#include "linear_ccd/car.h"
#include "linear_ccd/dir_control_algorithm.h"
#include "linear_ccd/median_ccd_filter.h"
#include "linear_ccd/program.h"
#include "linear_ccd/speed_control_strategy.h"
#include "linear_ccd/speed_control_1.h"

namespace linear_ccd
{

class AutoProgram : public Program
{
public:
	static constexpr int INITIAL_DELAY = 2000;

	AutoProgram();
	~AutoProgram();

	void Run() override;

private:
	struct LedState
	{
		libsc::k60::Timer::TimerInt prev_run = 0;
		bool flag = false;
	};

	struct ServoState
	{
		libsc::k60::Timer::TimerInt prev_run = 0;
	};

	struct SpeedState
	{
		libsc::k60::Timer::TimerInt prev_run = 0;
	};

	struct JoystickState
	{
		libsc::k60::Timer::TimerInt prev_run = 0;
		uint8_t delay = 0;
	};

	struct EmergencyStopState
	{
		libsc::k60::Timer::TimerInt trigger_time = 0;
		bool is_triggered = false;
	};

	void InitialStage();

	void LedPass();
	void ServoPass();
	void SpeedControlPass();
	void JoystickPass();
	void DetectStopLine();
	void DetectEmergencyStop();
	int16_t ConcludeTurning(const int16_t up_turn, const int16_t down_turn) const;

	void SetConstant(const bool is_straight);

	static int FwriteHandler(int, char *ptr, int len);
	static void HardFaultHandler(void);

	Car m_car;

	LedState m_led_state;
	ServoState m_servo_state;
	SpeedState m_speed_state;
	JoystickState m_joystick_state;
	EmergencyStopState m_emergency_stop_state;

	MedianCcdFilter m_ccd_filter;
	DirControlAlgorithm m_dir_control[2];
	SpeedControl1 m_speed_control;

	libsc::k60::Timer::TimerInt m_start;
	bool m_is_stop;

	bool m_is_turn;

	uint8_t m_mode;

	static AutoProgram *m_instance;
};

}

#endif /* LINEAR_CCD_AUTO_PROGRAM_H_ */
