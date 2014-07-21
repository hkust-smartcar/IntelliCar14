/*
 * calibrate_program_2.cpp
 *
 * Author: Louis Mo, Ming Tsang, Spartey Chen
 * Copyright (c) 2014 HKUST SmartCar Team
 */

#include <mini_common.h>
#include <cstdint>
#include <cstring>

#include <bitset>

#include <MK60_adc.h>

#include <libbase/k60/gpio.h>

#include <libsc/k60/joystick.h>
#include <libsc/com/lcd.h>
#include <libsc/com/lcd_console.h>
#include <libsc/k60/linear_ccd.h>
#include <libsc/k60/simple_buzzer.h>
#include <libsc/k60/system.h>
#include <libsc/k60/timer.h>
#include <libutil/misc.h>
#include <libutil/string.h>

#include "linear_ccd/debug.h"
#include "linear_ccd/config.h"
#include "linear_ccd/beep_manager.h"
#include "linear_ccd/calibrate_program_2.h"

using namespace libbase::k60;
using namespace libsc::k60;
using namespace libsc;
using namespace libutil;
using namespace std;

namespace linear_ccd
{

namespace
{

constexpr int MID_ADC = 54;
constexpr int ADC_AMPLITUDE = 15;
constexpr int MIN_ADC = MID_ADC - ADC_AMPLITUDE;
constexpr int MAX_ADC = MID_ADC + ADC_AMPLITUDE;

Gpo::Config GetClkGpoConfig()
{
	Gpo::Config config;
	config.pin = PinConfig::Name::PTC2;
	return config;
}

Gpo::Config GetSiGpoConfig()
{
	Gpo::Config config;
	config.pin = PinConfig::Name::PTC3;
	return config;
}

void CcdDelay()
{
	// 50ns under 180MHz
	for (int i = 0; i < 9; ++i)
	{
		asm("nop");
	}
}

}

CalibrateProgram2::CalibrateProgram2()
{}

void CalibrateProgram2::Run()
{
	adc_init(ADC1_SE4a);
	Gpio clk_pin(GetClkGpoConfig());
	Gpio si_pin(GetSiGpoConfig());
	SimpleBuzzer buzzer(0);
	Joystick joystick(0);
	Lcd lcd(true);
	LcdConsole console(&lcd);
	bool is_print = true;

	Timer::TimerInt ccd_time = System::Time();
	Timer::TimerInt joystick_time = System::Time();
	int delay = 5;
	int row = 0;
	int joystick_delay = 0;
	constexpr int MID_Y = libsc::Lcd::H / 2;
	int y = 0;

	while (true)
	{
		const Timer::TimerInt now = System::Time();
		if (Timer::TimeDiff(now, ccd_time) >= Config::GetTurnInterval())
		{
			const uint8_t buf_size = LinearCcd::SENSOR_W;
			uint8_t buf[buf_size] = {};
			uint32_t total_adc = 0;

			clk_pin.Set(true);
			si_pin.Set(false);
			CcdDelay();

			si_pin.Set(true);
			clk_pin.Set(false);
			CcdDelay();

			clk_pin.Set(true);
			si_pin.Set(false);
			CcdDelay();

			for (int i = 0; i < LinearCcd::SENSOR_W; ++i)
			{
				clk_pin.Set(false);
				CcdDelay();
				uint8_t adc = adc_once(ADC1_SE4a, ADC_8bit);
				clk_pin.Set(true);
				CcdDelay();

				total_adc += adc;
				buf[i] = (Clamp<uint8_t>(MIN_ADC, adc, MAX_ADC) - MIN_ADC)
						/ (float)(MAX_ADC - MIN_ADC) * 0xFF;
			}

			if (delay > 0)
			{
				--delay;
			}
			else
			{
				if (is_print)
				{
					console.SetCursorRow(row);
					console.PrintString(libutil::String::Format("%lu %lu\n",
							total_adc, total_adc / 128).c_str(), 0xFFFF);
					if (++row > 4)
					{
						row = 0;
					}

					lcd.DrawGrayscalePixelBuffer(0, y + MID_Y, buf_size, 1, buf);

					if (++y > MID_Y)
					{
						y = 0;
					}
				}
				delay = 10;
			}

			ccd_time = now;
		}
		BeepManager::GetInstance(&buzzer)->Process();

		if (Timer::TimeDiff(now, joystick_time) >= 5)
		{
			if (joystick.GetState() == Joystick::State::SELECT
					&& joystick_delay == 0)
			{
				BeepManager::GetInstance(&buzzer)->Beep(100);
				is_print ^= true;
				joystick_delay = 200;
			}

			if (joystick_delay > 0)
			{
				--joystick_delay;
			}
		}
	}
}

}
