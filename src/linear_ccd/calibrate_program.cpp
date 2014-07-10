/*
 * calibrate_program.cpp
 *
 * Author: Louis Mo, Ming Tsang, Spartey Chen
 * Copyright (c) 2014 HKUST SmartCar Team
 */

#include <cstdint>

#include <bitset>

#include <MK60_dac.h>

#include <libsc/k60/joystick.h>
#include <libsc/k60/linear_ccd.h>
#include <libsc/k60/simple_buzzer.h>
#include <libsc/k60/system.h>
#include <libsc/k60/timer.h>
#include <libutil/string.h>

#include "linear_ccd/config.h"
#include "linear_ccd/beep_manager.h"
#include "linear_ccd/calibrate_program.h"
#include "linear_ccd/car.h"
#include "linear_ccd/dir_control_algorithm.h"

using namespace libsc::k60;
using namespace std;

namespace linear_ccd
{

CalibrateProgram::CalibrateProgram()
		: m_track_analyzer(0)
{}

void CalibrateProgram::Run()
{
	dac_init(DAC0);
	dac_out(DAC0, Config::GetCcdThreshold(0));

	SimpleBuzzer buzzer(0);
	Joystick joystick(0);
	bool is_print = true;

	Timer::TimerInt ccd_time = System::Time();
	Timer::TimerInt joystick_time = System::Time();
	int delay = 5;
	int row = 0;
	constexpr int MID_Y = libsc::Lcd::H / 2;
	int y = 0;
	while (true)
	{
		const Timer::TimerInt now = System::Time();
		if (Timer::TimeDiff(now, ccd_time) >= Config::GetTurnInterval()
				&& m_car.IsCcdReady(0))
		{
			m_car.StartCcdSample(0);

			const bitset<LinearCcd::SENSOR_W> &sample = m_car.GetCcdSample(0);
			m_track_analyzer.Analyze(sample);

			if (delay > 0)
			{
				--delay;
			}
			else
			{
				if (is_print)
				{
					m_car.LcdSetRow(row);
					m_car.LcdPrintString(libutil::String::Format("%d %d %d\n",
							m_track_analyzer.GetMid(),
							m_track_analyzer.GetLeftEdge(),
							m_track_analyzer.GetRightEdge()).c_str(), 0xFFFF);
					if (++row > 4)
					{
						row = 0;
					}

					const uint8_t buf_size = LinearCcd::SENSOR_W;
					uint8_t buf[buf_size] = {};
					for (int i = 0; i < LinearCcd::SENSOR_W; ++i)
					{
						buf[i] = sample[i] ? 0xFF : 0x00;
					}
					m_car.LcdDrawGrayscalePixelBuffer(0, y + MID_Y, buf_size,
							1, buf);
					if (m_track_analyzer.GetLeftEdge() != -1
							|| m_track_analyzer.GetRightEdge() != -1)
					{
						m_car.LcdDrawPixel(m_track_analyzer.GetMid(),
								y + MID_Y, libutil::GetRgb565(0xE5, 0x33, 0xB5));
					}
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

		if (Timer::TimeDiff(now, joystick_time) >= Config::GetJoystickInterval())
		{
			if (joystick.GetState() == Joystick::State::SELECT)
			{
				BeepManager::GetInstance(&buzzer)->Beep(100);
				is_print ^= true;
			}
		}
	}
}

}
