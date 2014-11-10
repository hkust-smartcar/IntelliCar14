/*
 * calibrate_program.cpp
 *
 * Author: Louis Mo, Ming Tsang, Spartey Chen
 * Copyright (c) 2014 HKUST SmartCar Team
 */

#include <cstdint>
#include <cstring>

#include <bitset>

#include <libbase/k60/dac.h>
#include <libsc/k60/joystick.h>
#include <libsc/k60/linear_ccd.h>
#include <libsc/k60/simple_buzzer.h>
#include <libsc/k60/system.h>
#include <libsc/k60/timer.h>
#include <libutil/misc.h>
#include <libutil/string.h>
#include <libutil/remote_var_manager.h>

#include "linear_ccd/debug.h"
#include "linear_ccd/config.h"
#include "linear_ccd/beep_manager.h"
#include "linear_ccd/calibrate_program.h"
#include "linear_ccd/car.h"
#include "linear_ccd/dir_control_algorithm.h"
#include "linear_ccd/median_ccd_filter.h"

using namespace libbase::k60;
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

private:
	static constexpr int TUNABLE_INT_COUNT = 1;

	void Select(const int id);

	int GetMultiplier() const;

	void AdjustValue(const bool is_positive);

	void Redraw(const bool is_clear_screen);

	Car *m_car;

	int m_select;

	RemoteVarManager::Var *m_ccd_threshold;
};

TuningMenu::TuningMenu(Car *const car)
		: m_car(car),
		  m_select(0)
{
	m_car->EnableRemoteVar(TUNABLE_INT_COUNT);
	auto manager = m_car->GetRemoteVarManager();

	m_ccd_threshold = manager->Register("", RemoteVarManager::Var::Type::INT);
	m_ccd_threshold->SetInt(Config::GetCcdThreshold(0));
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

				case Joystick::State::SELECT:
					is_break = true;
					break;

				default:
					break;
				}

				const bitset<2> &button = m_car->GetButtonState();
				if (button[0])
				{
					AdjustValue(true);
				}
				else if (button[1])
				{
					AdjustValue(false);
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
	m_select = Clamp<int>(0, id, 0);
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

void TuningMenu::AdjustValue(const bool is_positive)
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
	m_car->LcdPrintString(String::Format(
			"CCD: %d\n", m_ccd_threshold->GetInt()).c_str(),
			0xFFFF, (m_select == 0) ? 0x35BC : 0);
}

}

CalibrateProgram::CalibrateProgram()
		: m_track_analyzer(Config::GetCcdMid(), 0),
		  m_ccd_filter(2)
{}

void CalibrateProgram::Run()
{
	TuningStage();

	bool is_print = true;

	Timer::TimerInt ccd_time = System::Time();
	Timer::TimerInt joystick_time = System::Time();
	int delay = 5;
	int row = 0;
	constexpr int MID_Y = St7735r::kH / 2;
	int y = 0;
	while (true)
	{
		const Timer::TimerInt now = System::Time();
		if (Timer::TimeDiff(now, ccd_time) >= Config::GetTurnInterval()
				&& m_car.IsCcdReady(0))
		{
			m_car.StartCcdSample(0);

			const bitset<LinearCcd::kSensorW> &raw_sample = m_car.GetCcdSample(0);
			const bitset<LinearCcd::kSensorW> &filtered_sample =
					m_ccd_filter.Filter(raw_sample);
			m_track_analyzer.Analyze(filtered_sample);

			if (delay > 0)
			{
				--delay;
			}
			else
			{
				if (is_print)
				{
#ifdef DEBUG_CALIBRATE_EDGE
					m_car.LcdSetRow(row);
					m_car.LcdPrintString(libutil::String::Format("%d %d %d\n",
							m_track_analyzer.GetCurrMid(),
							m_track_analyzer.GetLeftEdge(),
							m_track_analyzer.GetRightEdge()).c_str(), 0xFFFF);
					if (++row > 4)
					{
						row = 0;
					}
#endif

					const uint8_t buf_size = LinearCcd::kSensorW;
					uint8_t buf[buf_size] = {};
					for (int i = 0; i < LinearCcd::kSensorW; ++i)
					{
						buf[i] = filtered_sample[i] ? 0xFF : 0x00;
					}
					m_car.LcdDrawGrayscalePixelBuffer(0, y + MID_Y, buf_size,
							1, buf);
					if (m_track_analyzer.GetLeftEdge() != -1
							|| m_track_analyzer.GetRightEdge() != -1)
					{
						m_car.LcdDrawPixel(m_track_analyzer.GetCurrMid(),
								y + MID_Y, libutil::GetRgb565(0xE5, 0x33, 0xB5));
					}

#ifdef DEBUG_CALIBRATE_FILTER
					for (int i = 0; i < LinearCcd::kSensorW; ++i)
					{
						buf[i] = raw_sample[i] ? 0xFF : 0x00;
					}
					m_car.LcdDrawGrayscalePixelBuffer(0, y, buf_size, 1, buf);
#endif

					if (++y > MID_Y)
					{
						y = 0;
					}
				}
				delay = 10;
			}

			ccd_time = now;
		}
		m_car.CcdSampleProcess(0);
		m_car.GetBeepManager()->Process();

		if (Timer::TimeDiff(now, joystick_time) >= Config::GetJoystickInterval())
		{
			if (m_car.GetJoystickState() == Joystick::State::SELECT)
			{
				m_car.GetBeepManager()->Beep(100);
				is_print ^= true;
			}
		}
	}
}

void CalibrateProgram::TuningStage()
{
	TuningMenu menu(&m_car);
	menu.Run();

	m_car.SetCcdDacThreshold(menu.GetCcdThreshold());
	m_car.LcdClear(0);
	System::DelayMs(250);
}

}
