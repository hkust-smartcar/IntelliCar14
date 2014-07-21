/*
 * manual_program_3.cpp
 *
 * Author: Louis Mo, Ming Tsang, Spartey Chen
 * Copyright (c) 2014 HKUST SmartCar Team
 */

#include <syscall.h>
#include <vectors.h>

#include <cstdint>

#include <algorithm>
#include <bitset>
#include <functional>
#include <string>

#include <libsc/k60/linear_ccd.h>
#include <libsc/k60/system.h>
#include <libsc/k60/timer.h>
#include <libutil/misc.h>
#include <libutil/string.h>

#include "linear_ccd/debug.h"
#include "linear_ccd/config.h"
#include "linear_ccd/beep_manager.h"
#include "linear_ccd/car.h"
#include "linear_ccd/dir_control_algorithm_3.h"
#include "linear_ccd/manual_program_3.h"
#include "linear_ccd/median_ccd_filter.h"
#include "linear_ccd/program.h"
#include "linear_ccd/tuning_menu_3.h"

using namespace libsc::k60;
using namespace libutil;
using namespace std;

namespace linear_ccd
{

ManualProgram3 *ManualProgram3::m_instance = nullptr;

ManualProgram3::ManualProgram3()
		: m_ccd_filter(2),
		  m_dir_control(&m_car),
		  m_start(0)
{
	m_instance = this;
	m_car.UartSendStr("|--- Running Manual 3 ---|\n");
	System::DelayMs(25);
}

ManualProgram3::~ManualProgram3()
{
	m_instance = nullptr;
}

void ManualProgram3::Run()
{
	__g_fwrite_handler = FwriteHandler;
	__g_hard_fault_handler = HardFaultHandler;

	TuningStage();

	m_start = System::Time();
	while (true)
	{
		ServoPass();
		SpeedControlPass();
		m_car.GetBeepManager()->Process();
	}
}

void ManualProgram3::ServoPass()
{
	const Timer::TimerInt time = System::Time();
	if (Timer::TimeDiff(time, m_turn_state.prev_run) >= Config::GetTurnInterval()
			&& m_car.IsCcdReady())
	{
#ifdef DEBUG_PRINT_INTERVAL
		iprintf("turn freq: %lu\n", Timer::TimeDiff(time, m_turn_state.prev_run));
#endif

		m_car.StartCcdSample();

		const bitset<LinearCcd::SENSOR_W> &raw_sample = m_car.GetCcdSample(0);
		const bitset<LinearCcd::SENSOR_W> &filtered_sample =
				m_ccd_filter.Filter(raw_sample);
		const int32_t turn = m_dir_control.Process(filtered_sample);
		m_car.SetTurning(Clamp<int>(-100, turn, 100));

#ifdef DEBUG_PRINT_ERROR_AND_PID
		static int print_error_pid_delay_ = 0;
		if (++print_error_pid_delay_ >= 5)
		{
			m_car.UartSendStr(String::Format("%d %.3f %.3f %d\n",
					64 - m_dir_control.GetMid(),
					m_dir_control.GetP(),
					m_dir_control.GetD(),
					-turn));
			print_error_pid_delay_ = 0;
		}
#endif

		m_turn_state.prev_run = time;
	}
	m_car.CcdSampleProcess();
}

void ManualProgram3::SpeedControlPass()
{
	const Timer::TimerInt time = System::Time();
	if (Timer::TimeDiff(time, m_speed_state.prev_run)
			>= Config::GetSpeedInterval())
	{
#ifdef DEBUG_PRINT_INTERVAL
		iprintf("spd freq: %lu\n", Timer::TimeDiff(time, m_speed_state.prev_run));
#endif

		m_car.UpdateEncoder();
		int encoder = m_car.GetEncoderCount();
		if (encoder >= 60 && !m_speed_state.is_triggered
				&& !m_speed_state.is_stopping)
		{
			encoder = std::min<int>(encoder, 200);
			m_car.SetMotorPower(encoder * 2);
			m_speed_state.is_triggered = true;
			m_speed_state.trigger_time = time;
		}
		else if (m_speed_state.is_triggered
				&& Timer::TimeDiff(time, m_speed_state.trigger_time) > 1000)
		{
			m_car.StopMotor();
			m_speed_state.is_triggered = false;
			m_speed_state.is_stopping = true;
			m_speed_state.trigger_time = time;
		}
		else if (m_speed_state.is_stopping
				&& Timer::TimeDiff(time, m_speed_state.trigger_time) > 3500)
		{
			m_speed_state.is_stopping = false;
		}

#ifdef DEBUG_PRINT_SPEED_ERROR_AND_PID
		static int print_error_pid_delay_ = 0;
		if (++print_error_pid_delay_ >= 3)
		{
			m_car.UartSendStr(String::Format("%d %.3f %.3f %.3f %d\n",
					m_speed_control.GetParameter().sp - m_car.GetEncoderCount(),
					m_speed_control.GetP(),
					m_speed_control.GetI(),
					m_speed_control.GetD(),
					power));
			print_error_pid_delay_ = 0;
		}
#endif

		m_speed_state.prev_run = time;
	}
}

void ManualProgram3::TuningStage()
{
	TuningMenu3 menu(&m_car);
	menu.Run();

	DirControlAlgorithm3::Parameter dp;
	dp.edge = menu.GetEdge();
	dp.kp = menu.GetTurnKp();
	dp.kd = menu.GetTurnKd();
	dp.turn_kp = menu.GetTurnTurnKp();
	dp.turn_kd = menu.GetTurnTurnKd();
	m_dir_control.SetParameter(dp);

	m_car.SetCcdDacThreshold(menu.GetCcdThreshold());

	m_car.UartSendStrLiteral("|--- Paramter ---|\n");
	m_car.UartSendStr(String::Format("%d\n", menu.GetCcdThreshold()));
	m_car.UartSendStr(String::Format("%d, %.3f, %.3f, %.3f, %.3f\n",
			dp.edge, dp.kp, dp.kd, dp.turn_kp, dp.turn_kd));
	m_car.UartSendStrLiteral("|--- Paramter End ---|\n");

	LcdRedraw();

	m_car.UpdateEncoder();
	m_dir_control.OnFinishWarmUp();
	for (int i = 0; i < LinearCcd::SENSOR_W; ++i)
	{
		m_car.CcdSampleProcess();
	}
	m_car.StartCcdSample();
}

void ManualProgram3::LcdRedraw()
{
	m_car.LcdClear(0);
	m_car.LcdSetRow(0);
	m_car.LcdPrintString(String::Format(
			"Turn\nEdge: %d\nKP: %.3f\nKD: %.3f\n",
			m_dir_control.GetParameter().edge,
			m_dir_control.GetParameter().kp,
			m_dir_control.GetParameter().kd).c_str(), 0xFFFF);
}

int ManualProgram3::FwriteHandler(int, char *ptr, int len)
{
	if (m_instance)
	{
		m_instance->m_car.UartSendBuffer((const Byte*)ptr, len);
	}
	return len;
}

void ManualProgram3::HardFaultHandler()
{
	if (m_instance)
	{
		m_instance->m_car.StopMotor();
		for (int i = 0; 0 < 4; ++i)
		{
			m_instance->m_car.SwitchLed(i, true);
		}
	}
	while(true)
	{}
}

}
