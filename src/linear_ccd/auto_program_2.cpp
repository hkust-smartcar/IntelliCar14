/*
 * auto_program_2.cpp
 *
 * Author: Louis Mo, Ming Tsang, Spartey Chen
 * Copyright (c) 2014 HKUST SmartCar Team
 */

#include <cstdint>
#include <cstring>

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
#include <libutil/remote_var_manager.h>
#include <libutil/string.h>

#include "linear_ccd/debug.h"
#include "linear_ccd/config.h"
#include "linear_ccd/auto_program_2.h"
#include "linear_ccd/beep_manager.h"
#include "linear_ccd/car.h"
#include "linear_ccd/dir_control_algorithm_2.h"
#include "linear_ccd/median_ccd_filter.h"
#include "linear_ccd/program.h"
#include "linear_ccd/speed_control_2.h"

using namespace libsc::k60;
using namespace libutil;
using namespace std;

namespace linear_ccd
{

namespace
{

class TuningMenu
{
public:
	explicit TuningMenu(Car *const car);

	void Run();

	uint32_t GetCcdThreshold() const
	{
		return m_ccd_threshold->GetInt();
	}

	uint32_t GetEdge() const
	{
		return m_edge->GetInt();
	}

	float GetTurnKp() const
	{
		return m_turn_kp->GetReal();
	}

	float GetTurnKd() const
	{
		return m_turn_kd->GetReal();
	}

	float GetTurnTurnKp() const
	{
		return m_turn_turn_kp->GetReal();
	}

	float GetTurnTurnKd() const
	{
		return m_turn_turn_kd->GetReal();
	}

	uint32_t GetSpeedSp() const
	{
		return m_speed_sp->GetInt();
	}

	float GetSpeedKp() const
	{
		return m_speed_kp->GetReal();
	}

	float GetSpeedKi() const
	{
		return m_speed_ki->GetReal();
	}

	float GetSpeedKd() const
	{
		return m_speed_kd->GetReal();
	}

	uint32_t GetSpeedTurnSp() const
	{
		return m_speed_turn_sp->GetInt();
	}

private:
	enum Page
	{
		TURN,
		SPEED,

		SIZE,
	};

	static constexpr int TUNABLE_INT_COUNT = 11;

	void Select(const int id);
	void SwitchPage(const int id);

	int GetMultiplier() const;

	void AdjustValueTurn(const bool is_positive);
	void AdjustValueSpeed(const bool is_positive);

	void Redraw(const bool is_clear_screen);

	Car *m_car;

	Page m_page;
	int m_select;

	RemoteVarManager::Var *m_ccd_threshold;
	RemoteVarManager::Var *m_edge;
	RemoteVarManager::Var *m_turn_kp;
	RemoteVarManager::Var *m_turn_kd;
	RemoteVarManager::Var *m_turn_turn_kp;
	RemoteVarManager::Var *m_turn_turn_kd;

	RemoteVarManager::Var *m_speed_sp;
	RemoteVarManager::Var *m_speed_kp;
	RemoteVarManager::Var *m_speed_ki;
	RemoteVarManager::Var *m_speed_kd;
	RemoteVarManager::Var *m_speed_turn_sp;
};

TuningMenu::TuningMenu(Car *const car)
		: m_car(car),
		  m_page(Page::TURN),
		  m_select(0)
{
	m_car->EnableRemoteVar(TUNABLE_INT_COUNT);
	auto manager = m_car->GetRemoteVarManager();

	m_ccd_threshold = manager->Register("", RemoteVarManager::Var::Type::INT);
	m_ccd_threshold->SetInt(Config::GetCcdThreshold(0));
	m_edge = manager->Register("", RemoteVarManager::Var::Type::INT);
	m_edge->SetInt(27);
	m_turn_kp = manager->Register("", RemoteVarManager::Var::Type::REAL);
	m_turn_kp->SetReal(1.00f);
	m_turn_kd = manager->Register("", RemoteVarManager::Var::Type::REAL);
	m_turn_kd->SetReal(1.00f);
	m_turn_turn_kp = manager->Register("", RemoteVarManager::Var::Type::REAL);
	m_turn_turn_kp->SetReal(13.24f);
	m_turn_turn_kd = manager->Register("", RemoteVarManager::Var::Type::REAL);
	m_turn_turn_kd->SetReal(0.42f);

	m_speed_sp = manager->Register("", RemoteVarManager::Var::Type::INT);
	m_speed_sp->SetInt(390);
	m_speed_kp = manager->Register("", RemoteVarManager::Var::Type::REAL);
	m_speed_kp->SetReal(104.5f);
	m_speed_ki = manager->Register("", RemoteVarManager::Var::Type::REAL);
	m_speed_ki->SetReal(100.0f);
	m_speed_kd = manager->Register("", RemoteVarManager::Var::Type::REAL);
	m_speed_kd->SetReal(0.05f);
	m_speed_turn_sp = manager->Register("", RemoteVarManager::Var::Type::INT);
	m_speed_turn_sp->SetInt(390);
}

void TuningMenu::Run()
{
	auto manager = m_car->GetRemoteVarManager();
	manager->Start(false);
	m_car->SetUartLoopMode(true);
	Redraw(true);

	int delay = 0;
	Timer::TimerInt time = System::Time();
	Timer::TimerInt prev_redraw = System::Time();
	bool is_break = false;
	while (!is_break)
	{
		const Timer::TimerInt now = System::Time();
		if (Timer::TimeDiff(now, time) >= 5)
		{
			if (delay > 0)
			{
				--delay;
			}
			else
			{
				switch (m_car->GetJoystickState())
				{
				case Joystick::State::UP:
					Select(m_select - 1);
					break;

				case Joystick::State::DOWN:
					Select(m_select + 1);
					break;

				case Joystick::State::LEFT:
					SwitchPage(m_page - 1);
					break;

				case Joystick::State::RIGHT:
					SwitchPage(m_page + 1);
					break;

				case Joystick::State::SELECT:
					is_break = true;
					break;

				default:
					break;
				}

				const bitset<2> &button = m_car->GetButtonState();
				if (button[0])
				{
					switch (m_page)
					{
					default:
					case Page::TURN:
						AdjustValueTurn(true);
						break;

					case Page::SPEED:
						AdjustValueSpeed(true);
						break;
					}
				}
				else if (button[1])
				{
					switch (m_page)
					{
					default:
					case Page::TURN:
						AdjustValueTurn(false);
						break;

					case Page::SPEED:
						AdjustValueSpeed(false);
						break;
					}
				}

				delay = 20;
			}

			time = now;
		}

		if (Timer::TimeDiff(now, prev_redraw) >= 50)
		{
			Redraw(false);
			prev_redraw = now;
		}
	}

	manager->Stop();
	m_car->SetUartLoopMode(false);
	System::DelayMs(1);
	m_car->UartEnableRx();
}

void TuningMenu::Select(const int id)
{
	switch (m_page)
	{
	case Page::TURN:
		m_select = Clamp<int>(0, id, 5);
		break;

	case Page::SPEED:
		m_select = Clamp<int>(0, id, 4);
		break;

	case Page::SIZE:
		break;
	}
}

void TuningMenu::SwitchPage(const int id)
{
	m_page = static_cast<Page>(Clamp<int>(0, id, Page::SIZE - 1));
	m_select = 0;
	Redraw(true);
}

int TuningMenu::GetMultiplier() const
{
	const bitset<5> &ss = m_car->GetSwitchState();
	for (int i = 0; i < 5; ++i)
	{
		if (ss[i])
		{
			return i + 1;
		}
	}
	return 1;
}

void TuningMenu::AdjustValueTurn(const bool is_positive)
{
	Byte data[5];
	switch (m_select)
	{
	case 0:
		{
			data[0] = m_ccd_threshold->GetId();
			uint32_t value = m_ccd_threshold->GetInt();
			value += (is_positive ? 1 : -1) * GetMultiplier();
			memcpy(data + 1, &value, 4);
		}
		break;

	case 1:
		{
			data[0] = m_edge->GetId();
			uint32_t value = m_edge->GetInt();
			value += (is_positive ? 1 : -1) * GetMultiplier();
			memcpy(data + 1, &value, 4);
		}
		break;

	case 2:
		{
			data[0] = m_turn_kp->GetId();
			float value = m_turn_kp->GetReal();
			value += (is_positive ? 0.05f : -0.05f) * GetMultiplier();
			memcpy(data + 1, &value, 4);
		}
		break;

	case 3:
		{
			data[0] = m_turn_kd->GetId();
			float value = m_turn_kd->GetReal();
			value += (is_positive ? 0.005f : -0.005f) * GetMultiplier();
			memcpy(data + 1, &value, 4);
		}
		break;

	case 4:
		{
			data[0] = m_turn_turn_kp->GetId();
			float value = m_turn_turn_kp->GetReal();
			value += (is_positive ? 0.15f : -0.15f) * GetMultiplier();
			memcpy(data + 1, &value, 4);
		}
		break;

	case 5:
		{
			data[0] = m_turn_turn_kd->GetId();
			float value = m_turn_turn_kd->GetReal();
			value += (is_positive ? 0.03f : -0.03f) * GetMultiplier();
			memcpy(data + 1, &value, 4);
		}
		break;

	default:
		return;
	}

	*reinterpret_cast<uint32_t*>(data + 1) =
			htobe32(*reinterpret_cast<uint32_t*>(data + 1));
	m_car->UartSendBuffer(data, 5);
}

void TuningMenu::AdjustValueSpeed(const bool is_positive)
{
	Byte data[5];
	switch (m_select)
	{
	case 0:
		{
			data[0] = m_speed_sp->GetId();
			uint32_t value = m_speed_sp->GetInt();
			value += (is_positive ? 5 : -5) * GetMultiplier();
			memcpy(data + 1, &value, 4);
		}
		break;

	case 1:
		{
			data[0] = m_speed_kp->GetId();
			float value = m_speed_kp->GetReal();
			value += (is_positive ? 0.5f : -0.5f) * GetMultiplier();
			memcpy(data + 1, &value, 4);
		}
		break;

	case 2:
		{
			data[0] = m_speed_ki->GetId();
			float value = m_speed_ki->GetReal();
			value += (is_positive ? 0.5f : -0.5f) * GetMultiplier();
			memcpy(data + 1, &value, 4);
		}
		break;

	case 3:
		{
			data[0] = m_speed_kd->GetId();
			float value = m_speed_kd->GetReal();
			value += (is_positive ? 0.02f : -0.02f) * GetMultiplier();
			memcpy(data + 1, &value, 4);
		}
		break;

	case 4:
		{
			data[0] = m_speed_turn_sp->GetId();
			uint32_t value = m_speed_turn_sp->GetInt();
			value += (is_positive ? 5 : -5) * GetMultiplier();
			memcpy(data + 1, &value, 4);
		}
		break;

	default:
		return;
	}

	*reinterpret_cast<uint32_t*>(data + 1) =
			htobe32(*reinterpret_cast<uint32_t*>(data + 1));
	m_car->UartSendBuffer(data, 5);
}

void TuningMenu::Redraw(const bool is_clear_screen)
{
	if (is_clear_screen)
	{
		m_car->LcdClear(0);
	}
	m_car->LcdSetRow(0);
	switch (m_page)
	{
	case Page::TURN:
		{
			m_car->LcdPrintString("Turn\n", 0xFFFF);
			m_car->LcdPrintString(String::Format(
					"CCD: %d\n", m_ccd_threshold->GetInt()).c_str(),
					0xFFFF, (m_select == 0) ? 0x35BC : 0);
			m_car->LcdPrintString(String::Format(
					"Edge: %d\n", m_edge->GetInt()).c_str(),
					0xFFFF, (m_select == 1) ? 0x35BC : 0);
			m_car->LcdPrintString(String::Format(
					"KP: %.3f\n", m_turn_kp->GetReal()).c_str(),
					0xFFFF, (m_select == 2) ? 0x35BC : 0);
			m_car->LcdPrintString(String::Format(
					"KD: %.3f\n", m_turn_kd->GetReal()).c_str(),
					0xFFFF, (m_select == 3) ? 0x35BC : 0);
			m_car->LcdPrintString(String::Format(
					"Turn KP: %.3f\n", m_turn_turn_kp->GetReal()).c_str(),
					0xFFFF, (m_select == 4) ? 0x35BC : 0);
			m_car->LcdPrintString(String::Format(
					"Turn KD: %.3f\n", m_turn_turn_kd->GetReal()).c_str(),
					0xFFFF, (m_select == 5) ? 0x35BC : 0);
		}
		break;

	case Page::SPEED:
		{
			m_car->LcdPrintString("Speed\n", 0xFFFF);
			m_car->LcdPrintString(String::Format(
					"SP: %d\n", m_speed_sp->GetInt()).c_str(),
					0xFFFF, (m_select == 0) ? 0x35BC : 0);
			m_car->LcdPrintString(String::Format(
					"KP: %.3f\n", m_speed_kp->GetReal()).c_str(),
					0xFFFF, (m_select == 1) ? 0x35BC : 0);
			m_car->LcdPrintString(String::Format(
					"KI: %.3f\n", m_speed_ki->GetReal()).c_str(),
					0xFFFF, (m_select == 2) ? 0x35BC : 0);
			m_car->LcdPrintString(String::Format(
					"KD: %.3f\n", m_speed_kd->GetReal()).c_str(),
					0xFFFF, (m_select == 3) ? 0x35BC : 0);
			m_car->LcdPrintString(String::Format(
					"Turn SP: %d\n", m_speed_turn_sp->GetInt()).c_str(),
					0xFFFF, (m_select == 4) ? 0x35BC : 0);
		}
		break;

	case Page::SIZE:
		break;
	}
}

}

AutoProgram2 *AutoProgram2::m_instance = nullptr;

AutoProgram2::AutoProgram2()
		: m_is_ignore_light_sensor(true),
		  m_car(std::bind(&AutoProgram2::OnLightSensorDetectHandler, this,
				  placeholders::_1)),
		  m_ccd_filter(2),
		  m_dir_control(&m_car),
		  m_speed_control(&m_car),
		  m_start(0),
		  m_is_stop(false)
{
	m_instance = this;
	m_car.UartSendStr("|--- Running Auto 2 ---|\n");
	System::DelayMs(25);
}

AutoProgram2::~AutoProgram2()
{
	m_instance = nullptr;
}

void AutoProgram2::Run()
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

void AutoProgram2::BrakePass()
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
			m_car.SetMotorPower(1800 * ((count > 0) ? -1 : 1));
		}
		else if (abs_count < 300)
		{
			m_car.SetMotorPower(2800 * ((count > 0) ? -1 : 1));
		}
		else
		{
			m_car.SetMotorPower(3500 * ((count > 0) ? -1 : 1));
		}

		m_brake_state.prev_run = time;
	}
}

void AutoProgram2::JoystickPass()
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

void AutoProgram2::LedPass()
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

void AutoProgram2::ServoPass()
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

void AutoProgram2::SpeedControlPass()
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

void AutoProgram2::DetectEmergencyStop()
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

void AutoProgram2::TuningStage()
{
	TuningMenu menu(&m_car);
	menu.Run();

	DirControlAlgorithm2::Parameter dp;
	dp.edge = menu.GetEdge();
	dp.kp = menu.GetTurnKp();
	dp.kd = menu.GetTurnKd();
	dp.turn_kp = menu.GetTurnTurnKp();
	dp.turn_kd = menu.GetTurnTurnKd();
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
	m_car.UartSendStr(String::Format("%d, %.3f, %.3f, %.3f, %.3f\n",
			dp.edge, dp.kp, dp.kd, dp.turn_kp, dp.turn_kd));
	m_car.UartSendStr(String::Format("%d, %.3f, %.3f, %.3f, %d\n",
			sp.sp, sp.kp, sp.ki, sp.kd, sp.turn_sp));
	m_car.UartSendStrLiteral("|--- Paramter End ---|\n");

	LcdRedraw();
}

void AutoProgram2::CountDownStage()
{
	m_car.SetBuzzerBeep(true);
	System::DelayMs(100);
	m_car.SetBuzzerBeep(false);
	System::DelayMs(500);
	m_car.SetBuzzerBeep(true);
	System::DelayMs(100);
	m_car.SetBuzzerBeep(false);
	System::DelayMs(500);
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

void AutoProgram2::LcdRedraw()
{
	m_car.LcdClear(0);
	m_car.LcdSetRow(0);
	switch (m_joystick_state.page)
	{
	default:
	case 0:
		m_car.LcdPrintString(String::Format(
				"Turn\nEdge: %d\nKP: %.3f\nKD: %.3f\n",
				m_dir_control.GetParameter().edge,
				m_dir_control.GetParameter().kp,
				m_dir_control.GetParameter().kd).c_str(), 0xFFFF);
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

void AutoProgram2::OnLightSensorDetectHandler(const uint8_t id)
{
	if (m_is_ignore_light_sensor)
	{
		return;
	}

	const Timer::TimerInt now = System::Time();
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
}

int AutoProgram2::FwriteHandler(int, char *ptr, int len)
{
	if (m_instance)
	{
		m_instance->m_car.UartSendBuffer((const Byte*)ptr, len);
	}
	return len;
}

void AutoProgram2::HardFaultHandler()
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
