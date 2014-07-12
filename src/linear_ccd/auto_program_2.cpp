/*
 * auto_program_2.cpp
 *
 * Author: Louis Mo, Ming Tsang, Spartey Chen
 * Copyright (c) 2014 HKUST SmartCar Team
 */

#include <mini_common.h>
#include <hw_common.h>
#include <syscall.h>
#include <vectors.h>

#include <cstdint>
#include <cstring>

#include <bitset>
#include <string>

#include <log.h>
#include <MK60_dac.h>

#include <libsc/k60/linear_ccd.h>
#include <libsc/k60/system.h>
#include <libsc/k60/timer.h>
#include <libutil/misc.h>
#include <libutil/string.h>
#include <libutil/tunable_int_manager.h>
#include <libutil/tunable_int_manager.tcc>

#include "linear_ccd/debug.h"
#include "linear_ccd/config.h"
#include "linear_ccd/auto_program_2.h"
#include "linear_ccd/beep_manager.h"
#include "linear_ccd/car.h"
#include "linear_ccd/median_ccd_filter.h"
#include "linear_ccd/program.h"
#include "linear_ccd/speed_control_1.h"

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
		return m_ccd_threshold->GetValue();
	}

	float GetEdge() const
	{
		return m_edge->GetValue();
	}

	float GetTurnKp() const
	{
		return TunableInt::AsFloat(m_turn_kp->GetValue());
	}

	float GetTurnKd() const
	{
		return TunableInt::AsFloat(m_turn_kd->GetValue());
	}

	float GetTurnTurnKp() const
	{
		return TunableInt::AsFloat(m_turn_turn_kp->GetValue());
	}

	float GetTurnTurnKd() const
	{
		return TunableInt::AsFloat(m_turn_turn_kd->GetValue());
	}

	int GetSpeedSp() const
	{
		return m_speed_sp->GetValue();
	}

	float GetSpeedKp() const
	{
		return TunableInt::AsFloat(m_speed_kp->GetValue());
	}

	float GetSpeedKi() const
	{
		return TunableInt::AsFloat(m_speed_ki->GetValue());
	}

	float GetSpeedKd() const
	{
		return TunableInt::AsFloat(m_speed_kd->GetValue());
	}

	int GetSpeedTurnSp() const
	{
		return m_speed_turn_sp->GetValue();
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

	const TunableInt *m_ccd_threshold;
	const TunableInt *m_edge;
	const TunableInt *m_turn_kp;
	const TunableInt *m_turn_kd;
	const TunableInt *m_turn_turn_kp;
	const TunableInt *m_turn_turn_kd;

	const TunableInt *m_speed_sp;
	const TunableInt *m_speed_kp;
	const TunableInt *m_speed_ki;
	const TunableInt *m_speed_kd;
	const TunableInt *m_speed_turn_sp;
};

TuningMenu::TuningMenu(Car *const car)
		: m_car(car),
		  m_page(Page::TURN),
		  m_select(0)
{
	auto manager = m_car->GetTunableIntManager<TUNABLE_INT_COUNT>();

	m_ccd_threshold = manager->Register("", TunableInt::Type::INTEGER,
			Config::GetCcdThreshold(0));
	m_edge = manager->Register("", TunableInt::Type::INTEGER,
			27);
	m_turn_kp = manager->Register("", TunableInt::Type::REAL,
			TunableInt::AsUnsigned(2.79f));
	m_turn_kd = manager->Register("", TunableInt::Type::REAL,
			TunableInt::AsUnsigned(0.059f));
	m_turn_turn_kp = manager->Register("", TunableInt::Type::REAL,
			TunableInt::AsUnsigned(13.24f));
	m_turn_turn_kd = manager->Register("", TunableInt::Type::REAL,
			TunableInt::AsUnsigned(0.42f));

	m_speed_sp = manager->Register("", TunableInt::Type::INTEGER,
			410);
	m_speed_kp = manager->Register("", TunableInt::Type::REAL,
			TunableInt::AsUnsigned(104.5f));
	m_speed_ki = manager->Register("", TunableInt::Type::REAL,
			TunableInt::AsUnsigned(100.0f));
	m_speed_kd = manager->Register("", TunableInt::Type::REAL,
			TunableInt::AsUnsigned(0.05f));
	m_speed_turn_sp = manager->Register("", TunableInt::Type::INTEGER,
			400);
}

void TuningMenu::Run()
{
	auto manager = m_car->GetTunableIntManager<TUNABLE_INT_COUNT>();
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
			uint32_t value = m_ccd_threshold->GetValue();
			value += (is_positive ? 1 : -1) * GetMultiplier();
			memcpy(data + 1, &value, 4);
		}
		break;

	case 1:
		{
			data[0] = m_edge->GetId();
			uint32_t value = m_edge->GetValue();
			value += (is_positive ? 1 : -1) * GetMultiplier();
			memcpy(data + 1, &value, 4);
		}
		break;

	case 2:
		{
			data[0] = m_turn_kp->GetId();
			float value = TunableInt::AsFloat(m_turn_kp->GetValue());
			value += (is_positive ? 0.05f : -0.05f) * GetMultiplier();
			memcpy(data + 1, &value, 4);
		}
		break;

	case 3:
		{
			data[0] = m_turn_kd->GetId();
			float value = TunableInt::AsFloat(m_turn_kd->GetValue());
			value += (is_positive ? 0.002f : -0.002f) * GetMultiplier();
			memcpy(data + 1, &value, 4);
		}
		break;

	case 4:
		{
			data[0] = m_turn_turn_kp->GetId();
			float value = TunableInt::AsFloat(m_turn_turn_kp->GetValue());
			value += (is_positive ? 0.15f : -0.15f) * GetMultiplier();
			memcpy(data + 1, &value, 4);
		}
		break;

	case 5:
		{
			data[0] = m_turn_turn_kd->GetId();
			float value = TunableInt::AsFloat(m_turn_turn_kd->GetValue());
			value += (is_positive ? 0.03f : -0.03f) * GetMultiplier();
			memcpy(data + 1, &value, 4);
		}
		break;

	default:
		return;
	}

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
			uint32_t value = m_speed_sp->GetValue();
			value += (is_positive ? 5 : -5) * GetMultiplier();
			memcpy(data + 1, &value, 4);
		}
		break;

	case 1:
		{
			data[0] = m_speed_kp->GetId();
			float value = TunableInt::AsFloat(m_speed_kp->GetValue());
			value += (is_positive ? 0.5f : -0.5f) * GetMultiplier();
			memcpy(data + 1, &value, 4);
		}
		break;

	case 2:
		{
			data[0] = m_speed_ki->GetId();
			float value = TunableInt::AsFloat(m_speed_ki->GetValue());
			value += (is_positive ? 0.5f : -0.5f) * GetMultiplier();
			memcpy(data + 1, &value, 4);
		}
		break;

	case 3:
		{
			data[0] = m_speed_kd->GetId();
			float value = TunableInt::AsFloat(m_speed_kd->GetValue());
			value += (is_positive ? 0.02f : -0.02f) * GetMultiplier();
			memcpy(data + 1, &value, 4);
		}
		break;

	case 4:
		{
			data[0] = m_speed_turn_sp->GetId();
			uint32_t value = m_speed_turn_sp->GetValue();
			value += (is_positive ? 5 : -5) * GetMultiplier();
			memcpy(data + 1, &value, 4);
		}
		break;

	default:
		return;
	}

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
					"CCD: %d\n", m_ccd_threshold->GetValue()).c_str(),
					0xFFFF, (m_select == 0) ? 0x35BC : 0);
			m_car->LcdPrintString(String::Format(
					"Edge: %d\n", m_edge->GetValue()).c_str(),
					0xFFFF, (m_select == 1) ? 0x35BC : 0);
			m_car->LcdPrintString(String::Format(
					"KP: %.3f\n", TunableInt::AsFloat(m_turn_kp->GetValue())).c_str(),
					0xFFFF, (m_select == 2) ? 0x35BC : 0);
			m_car->LcdPrintString(String::Format(
					"KD: %.3f\n", TunableInt::AsFloat(m_turn_kd->GetValue())).c_str(),
					0xFFFF, (m_select == 3) ? 0x35BC : 0);
			m_car->LcdPrintString(String::Format(
					"Turn KP: %.3f\n", TunableInt::AsFloat(m_turn_turn_kp->GetValue())).c_str(),
					0xFFFF, (m_select == 4) ? 0x35BC : 0);
			m_car->LcdPrintString(String::Format(
					"Turn KD: %.3f\n", TunableInt::AsFloat(m_turn_turn_kd->GetValue())).c_str(),
					0xFFFF, (m_select == 5) ? 0x35BC : 0);
		}
		break;

	case Page::SPEED:
		{
			m_car->LcdPrintString("Speed\n", 0xFFFF);
			m_car->LcdPrintString(String::Format(
					"SP: %d\n", m_speed_sp->GetValue()).c_str(),
					0xFFFF, (m_select == 0) ? 0x35BC : 0);
			m_car->LcdPrintString(String::Format(
					"KP: %.3f\n", TunableInt::AsFloat(m_speed_kp->GetValue())).c_str(),
					0xFFFF, (m_select == 1) ? 0x35BC : 0);
			m_car->LcdPrintString(String::Format(
					"KI: %.3f\n", TunableInt::AsFloat(m_speed_ki->GetValue())).c_str(),
					0xFFFF, (m_select == 2) ? 0x35BC : 0);
			m_car->LcdPrintString(String::Format(
					"KD: %.3f\n", TunableInt::AsFloat(m_speed_kd->GetValue())).c_str(),
					0xFFFF, (m_select == 3) ? 0x35BC : 0);
			m_car->LcdPrintString(String::Format(
					"Turn SP: %d\n", m_speed_turn_sp->GetValue()).c_str(),
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
		: m_ccd_filter(2),
		  m_dir_control(&m_car),
		  m_speed_control(&m_car),
		  m_start(0),
		  m_is_stop(false)
{
	m_instance = this;
}

AutoProgram2::~AutoProgram2()
{
	m_instance = nullptr;
}

void AutoProgram2::Run()
{
	__g_fwrite_handler = FwriteHandler;
	__g_hard_fault_handler = HardFaultHandler;

	TuningStage();
	CountDownStage();

	m_start = System::Time();
	while (true)
	{
		ServoPass();
		if (!m_is_stop)
		{
			if (Timer::TimeDiff(System::Time(), m_start)
					>= Config::GetAutoStopTime())
			{
				m_is_stop = true;
			}
			else
			{
				SpeedControlPass();
			}
			DetectEmergencyStop();
		}
		else
		{
			m_car.StopMotor();
		}

		LedPass();
		JoystickPass();
		m_car.GetBeepManager()->Process();
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

		const bitset<LinearCcd::SENSOR_W> &raw_sample = m_car.GetCcdSample(0);
		const bitset<LinearCcd::SENSOR_W> &filtered_sample =
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

		m_speed_control.Control();
		m_speed_state.prev_run = time;
	}
}

void AutoProgram2::DetectEmergencyStop()
{
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

	dac_init(DAC0);
	dac_out(DAC0, menu.GetCcdThreshold());

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
	m_dir_control.OnFinishWarmUp();
	m_speed_control.OnFinishWarmUp();
	for (int i = 0; i < LinearCcd::SENSOR_W; ++i)
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
