/*
 * tuning_menu_3.cpp
 *
 * Author: Louis Mo, Ming Tsang, Spartey Chen
 * Copyright (c) 2014 HKUST SmartCar Team
 */

#include <cstdint>
#include <cstring>

#include <bitset>

#include <libsc/k60/system.h>
#include <libsc/k60/timer.h>
#include <libutil/tunable_int_manager.h>
#include <libutil/tunable_int_manager.tcc>

#include "linear_ccd/debug.h"
#include "linear_ccd/config.h"
#include "linear_ccd/car.h"
#include "linear_ccd/tuning_menu_3.h"

using namespace libsc::k60;
using namespace libutil;
using namespace std;

namespace linear_ccd
{

TuningMenu3::TuningMenu3(Car *const car)
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
			TunableInt::AsUnsigned(1.00f));
	m_turn_kd = manager->Register("", TunableInt::Type::REAL,
			TunableInt::AsUnsigned(1.00f));
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

void TuningMenu3::Run()
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
	System::DelayMs(1);
	m_car->UartEnableRx();
}

void TuningMenu3::Select(const int id)
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

void TuningMenu3::SwitchPage(const int id)
{
	m_page = static_cast<Page>(Clamp<int>(0, id, Page::SIZE - 1));
	m_select = 0;
	Redraw(true);
}

int TuningMenu3::GetMultiplier() const
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

void TuningMenu3::AdjustValueTurn(const bool is_positive)
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
			value += (is_positive ? 0.02f : -0.02f) * GetMultiplier();
			memcpy(data + 1, &value, 4);
		}
		break;

	case 3:
		{
			data[0] = m_turn_kd->GetId();
			float value = TunableInt::AsFloat(m_turn_kd->GetValue());
			value += (is_positive ? 0.02f : -0.02f) * GetMultiplier();
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

void TuningMenu3::AdjustValueSpeed(const bool is_positive)
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

void TuningMenu3::Redraw(const bool is_clear_screen)
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
