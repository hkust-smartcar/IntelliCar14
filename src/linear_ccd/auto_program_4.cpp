/*
 * auto_program_4.cpp
 *
 * Author: Louis Mo, Ming Tsang, Spartey Chen
 * Copyright (c) 2014 HKUST SmartCar Team
 */

#include <cstdint>

#include <bitset>
#include <functional>
#include <string>

#include <libbase/log.h>
#include <libbase/syscall.h>
#include <libbase/k60/vectors.h>

#include <libsc/k60/linear_ccd.h>
#include <libsc/k60/system.h>
#include <libsc/k60/timer.h>
#include <libutil/misc.h>
#include <libutil/string.h>

#include "linear_ccd/debug.h"
#include "linear_ccd/config.h"
#include "linear_ccd/auto_program_4.h"
#include "linear_ccd/beep_manager.h"
#include "linear_ccd/car.h"
#include "linear_ccd/dir_control_algorithm_3.h"
#include "linear_ccd/median_ccd_filter.h"
#include "linear_ccd/program.h"
#include "linear_ccd/speed_control_2.h"
#include "linear_ccd/tuning_menu_4.h"

using namespace libsc::k60;
using namespace libutil;
using namespace std;

namespace linear_ccd
{

AutoProgram4 *AutoProgram4::m_instance = nullptr;

AutoProgram4::AutoProgram4()
		: m_is_ignore_light_sensor(true),
		  m_car(std::bind(&AutoProgram4::OnLightSensorDetectHandler, this,
				  placeholders::_1)),
		  m_ccd_filter(2),
		  m_dir_control(&m_car),
		  m_speed_control(&m_car),
		  m_start(0),
		  m_is_stop(false)
{
	m_instance = this;
	m_car.UartSendStr("|--- Running Auto 4 ---|\n");
	System::DelayMs(25);
}

AutoProgram4::~AutoProgram4()
{
	m_instance = nullptr;
}

void AutoProgram4::Run()
{
	g_fwrite_handler = FwriteHandler;
	g_hard_fault_handler = HardFaultHandler;

	TuningStage();
	CountDownStage();

	m_start = System::Time();
	while (true)
	{
		ServoPass();
		if (!m_is_stop)
		{
			if (Config::GetAutoStopTime() != 0
					&& Timer::TimeDiff(System::Time(), m_start)
							>= Config::GetAutoStopTime())
			{
				m_is_stop = true;
			}
			else
			{
				if (m_brake_state.is_brake)
				{
					BrakePass();
				}
				else
				{
					SpeedControlPass();
				}
			}
			DetectEmergencyStop();
		}
		else
		{
			m_car.StopMotor();
			//BrakePass();
		}

		LedPass();
		JoystickPass();
		m_car.GetBeepManager()->Process();
	}
}

void AutoProgram4::BrakePass()
{
	const Timer::TimerInt time = System::Time();
	if (Timer::TimeDiff(time, m_brake_state.prev_run)
			>= Config::GetBrakeInterval())
	{
		m_car.UpdateEncoder();
		const int16_t count = m_car.GetEncoderCount();
		const uint16_t abs_count = abs(count);
		if (abs_count < 90)
		{
			m_car.StopMotor();
		}
		else if (abs_count < 160)
		{
			m_car.SetMotorPower(1900 * ((count > 0) ? -1 : 1));
		}
		else if (abs_count < 300)
		{
			m_car.SetMotorPower(2900 * ((count > 0) ? -1 : 1));
		}
		else
		{
			m_car.SetMotorPower(3600 * ((count > 0) ? -1 : 1));
		}

		m_brake_state.prev_run = time;
	}
}

void AutoProgram4::JoystickPass()
{
	const Timer::TimerInt time = System::Time();
	if (Timer::TimeDiff(time, m_joystick_state.prev_run)
			>= Config::GetJoystickInterval())
	{
		if (m_joystick_state.delay > 0)
		{
			--m_joystick_state.delay;
		}
		else
		{
			switch (m_car.GetJoystickState())
			{
			case Joystick::State::LEFT:
				if (m_joystick_state.page > 0)
				{
					--m_joystick_state.page;
					LcdRedraw();
				}
				break;

			case Joystick::State::RIGHT:
				if (m_joystick_state.page < 1)
				{
					++m_joystick_state.page;
					LcdRedraw();
				}
				break;

			case Joystick::State::SELECT:
				break;

			default:
				break;
			}
			m_joystick_state.delay = 10;
		}

		m_joystick_state.prev_run = time - (time % Config::GetJoystickInterval());
	}
}

void AutoProgram4::LedPass()
{
	const Timer::TimerInt time = System::Time();
	if (Timer::TimeDiff(time, m_led_state.prev_run) >= Config::GetLedInterval())
	{
		m_car.SwitchLed(0, m_led_state.flag);
		m_car.SwitchLed(1, !m_led_state.flag);
		m_led_state.flag ^= true;
		m_led_state.prev_run = time - (time % Config::GetLedInterval());
	}
}

void AutoProgram4::ServoPass()
{
	const Timer::TimerInt time = System::Time();
	if (Timer::TimeDiff(time, m_turn_state.prev_run) >= Config::GetTurnInterval()
			&& m_car.IsCcdReady())
	{
#ifdef DEBUG_PRINT_INTERVAL
		iprintf("turn freq: %lu\n", Timer::TimeDiff(time, m_turn_state.prev_run));
#endif

		m_car.StartCcdSample();

		const bitset<LinearCcd::kSensorW> &raw_sample = m_car.GetCcdSample(0);
		const bitset<LinearCcd::kSensorW> &filtered_sample =
				m_ccd_filter.Filter(raw_sample);
		const int32_t turn = m_dir_control.Process(filtered_sample);
		m_car.SetTurning(Clamp<int>(-100, turn, 100));

		if (abs(turn) >= Config::GetTurnThreshold())
		{
			m_speed_control.SetTurnHint(TurnHint::TURN);
		}
		else
		{
			m_speed_control.SetTurnHint(TurnHint::STRAIGHT);
		}

#ifdef DEBUG_PRINT_ERROR_AND_PID
		static int print_error_pid_delay_ = 0;
		if (++print_error_pid_delay_ >= 5)
		{
			m_car.UartSendStr(String::Format("%d %.3f %.3f %d\n",
					m_dir_control.GetParameter().mid - m_dir_control.GetMid(),
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

void AutoProgram4::SpeedControlPass()
{
	const Timer::TimerInt time = System::Time();
	if (Timer::TimeDiff(time, m_speed_state.prev_run)
			>= Config::GetSpeedInterval())
	{
#ifdef DEBUG_PRINT_INTERVAL
		iprintf("spd freq: %lu\n", Timer::TimeDiff(time, m_speed_state.prev_run));
#endif

		const int power = m_speed_control.Control();

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

void AutoProgram4::DetectEmergencyStop()
{
#ifdef DEBUG_USE_BT_EMERGENCY_STOP
	char ch;
	if (m_car.UartPeekChar(&ch))
	{
		m_brake_state.is_brake = true;
		m_car.GetBeepManager()->Beep(500);
	}
#endif

#ifdef DEBUG_USE_EMERGENCY_STOP
	static bool is_startup = true;
	const Timer::TimerInt time = System::Time();
	if (is_startup && Timer::TimeDiff(time, m_start) > 2000)
	{
		is_startup = false;
	}

	const int count = m_car.GetEncoderCount();
	if (!is_startup && abs(count) < 30)
	{
		if (m_emergency_stop_state.is_triggered)
		{
			if (Timer::TimeDiff(time, m_emergency_stop_state.trigger_time) > 150)
			{
				// Emergency stop
				m_is_stop = true;
				m_car.SetBuzzerBeep(true);
			}
		}
		else
		{
			m_emergency_stop_state.is_triggered = true;
			m_emergency_stop_state.trigger_time = time;
		}
	}
	else
	{
		m_emergency_stop_state.is_triggered = false;
	}

#else
	return;

#endif
}

void AutoProgram4::TuningStage()
{
	TuningMenu4 menu(&m_car);
	menu.Run();

	DirControlAlgorithm4::Parameter dp;
	dp.mid = menu.GetMid();
	dp.edge = menu.GetEdge();
	dp.kp = menu.GetTurnKp();
	dp.kp_fn = menu.GetTurnKpFn();
	dp.kd = menu.GetTurnKd();
	dp.kd_fn = menu.GetTurnKdFn();
	m_dir_control.SetParameter(dp);

	SpeedControl2::Parameter sp;
	sp.sp = menu.GetSpeedSp();
	sp.kp = menu.GetSpeedKp();
	sp.ki = menu.GetSpeedKi();
	sp.kd = menu.GetSpeedKd();
	sp.turn_sp = menu.GetSpeedTurnSp();
	m_speed_control.SetParameter(sp);

	m_car.SetCcdDacThreshold(menu.GetCcdThreshold());

	m_car.UartSendStrLiteral("|--- Paramter ---|\n");
	m_car.UartSendStr(String::Format("%d\n", menu.GetCcdThreshold()));
	m_car.UartSendStr(String::Format("%d, %d, %.3f, %d, %.3f, %d\n",
			dp.mid, dp.edge, dp.kp, dp.kp_fn, dp.kd, dp.kd_fn));
	m_car.UartSendStr(String::Format("%d, %.3f, %.3f, %.3f, %d\n",
			sp.sp, sp.kp, sp.ki, sp.kd, sp.turn_sp));
	m_car.UartSendStrLiteral("|--- Paramter End ---|\n");

	LcdRedraw();
}

void AutoProgram4::CountDownStage()
{
	m_car.SetBuzzerBeep(true);
	System::DelayMs(100);
	m_car.SetBuzzerBeep(false);
	System::DelayMs(400);
	m_car.SetBuzzerBeep(true);
	System::DelayMs(100);
	m_car.SetBuzzerBeep(false);
	System::DelayMs(400);
	m_car.SetBuzzerBeep(true);
	System::DelayMs(100);
	m_car.SetBuzzerBeep(false);
	System::DelayMs(400);
	m_car.SetBuzzerBeep(true);
	System::DelayMs(100);
	m_car.SetBuzzerBeep(false);
	System::DelayMs(400);
	m_car.SetBuzzerBeep(true);
	System::DelayMs(100);
	m_car.SetBuzzerBeep(false);

	m_car.UpdateEncoder();
	m_is_ignore_light_sensor = false;
	m_dir_control.OnFinishWarmUp();
	m_speed_control.OnFinishWarmUp();
	for (int i = 0; i < LinearCcd::kSensorW; ++i)
	{
		m_car.CcdSampleProcess();
	}
	m_car.StartCcdSample();
}

void AutoProgram4::LcdRedraw()
{
	m_car.LcdClear(0);
	m_car.LcdSetRow(0);
	switch (m_joystick_state.page)
	{
	default:
	case 0:
		m_car.LcdPrintString(String::Format(
				"Turn\nEdge: %d\nKP: %.3f\nKP fn: %d\nKD: %.3f\nKD fn: %d\n",
				m_dir_control.GetParameter().edge,
				m_dir_control.GetParameter().kp,
				m_dir_control.GetParameter().kp_fn,
				m_dir_control.GetParameter().kd,
				m_dir_control.GetParameter().kd_fn).c_str(), 0xFFFF);
		break;

	case 1:
		m_car.LcdPrintString(String::Format(
				"Speed\nSP: %d\nKP: %.3f\nKI: %.3f\nKD: %.3f\nTurn SP: %d\n",
				m_speed_control.GetParameter().sp,
				m_speed_control.GetParameter().kp,
				m_speed_control.GetParameter().ki,
				m_speed_control.GetParameter().kd,
				m_speed_control.GetParameter().turn_sp).c_str(), 0xFFFF);
		break;
	}
}

void AutoProgram4::OnLightSensorDetectHandler(const uint8_t id)
{
	if (m_is_ignore_light_sensor)
	{
		return;
	}

#ifdef DEBUG_DISABLE_GOAL
	return;
#else
	const Timer::TimerInt now = System::Time();
	if (Timer::TimeDiff(now, m_start) < 5000)
	{
		return;
	}

	if (Timer::TimeDiff(now, m_goal_state.trigger_time) > 250
			|| m_goal_state.trigger_id == (uint8_t)-1
			|| m_goal_state.trigger_id == id)
	{
		m_goal_state.trigger_time = now;
		m_goal_state.trigger_id = id;
#ifdef DEBUG_BEEP_LIGHT_SENSOR
		m_car.GetBeepManager()->Beep(100);
#endif
	}
	else
	{
		m_brake_state.is_brake = true;
		m_car.GetBeepManager()->Beep(500);
	}
#endif
}

int AutoProgram4::FwriteHandler(int, char *ptr, int len)
{
	if (m_instance)
	{
		m_instance->m_car.UartSendBuffer((const Byte*)ptr, len);
	}
	return len;
}

void AutoProgram4::HardFaultHandler()
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
