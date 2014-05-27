/*
 * linear_ccd_app.cpp
 * Linear CCD App
 *
 * Author: Louis Mo, Ming Tsang, Spartey Chen
 * Copyright (c) 2014 HKUST SmartCar Team
 */

#include <mini_common.h>
#include <hw_common.h>
#include <syscall.h>

#include <cstdint>
#include <cstdlib>
#include <bitset>

#include <log.h>
#include <MK60_gpio.h>

#include "linear_ccd/debug.h"

#include <libsc/com/linear_ccd.h>
#include <libutil/clock.h>
#include <libutil/misc.h>
#include <libutil/string.h>

#include "linear_ccd/bt_controller.h"
#include "linear_ccd/car.h"
#include "linear_ccd/linear_ccd_app.h"

using namespace std;
using libutil::Clock;

#define LED_FREQ 250
#define SERVO_FREQ 21
#define SPEED_CTRL_FREQ 19

namespace linear_ccd
{

LinearCcdApp *LinearCcdApp::m_instance = nullptr;

LinearCcdApp::LinearCcdApp()
		: m_dir_control(&m_car), m_is_stop(false), m_mode(0),
		  m_is_manual_interruptted(false)
{
	m_instance = this;
}

LinearCcdApp::~LinearCcdApp()
{
	m_instance = nullptr;
}

void LinearCcdApp::Run()
{
	__g_fwrite_handler = FwriteHandler;
	Clock::Init();
	InitialStage();

	int servo = 0;
	while (true)
	{
#ifdef DEBUG_MANUAL_CONTROL
		if (BtControlPass())
		{
			m_is_manual_interruptted = true;
		}
		if (m_is_manual_interruptted)
		{
			continue;
		}
#endif
		/*
		if (++servo > 100)
		{
			servo = 0;
		}
		m_car.SetRightPercentage(servo);
		DELAY_MS(25);
		*/
		ServoPass();
		//DetectStopLine();
		DetectEmergencyStop();
		if (!m_is_stop)
		{
			SpeedControlPass();
		}
		else
		{
			m_car.StopMotor();
		}
		LedPass();
	}
}

void LinearCcdApp::InitialStage()
{
	while (Clock::Time() < INITIAL_DELAY)
	{
		const uint8_t buttons = m_car.GetButtonState();
		for (int i = 0; i < 4; ++i)
		{
			if ((buttons >> i) & 0x1)
			{
				m_mode = i + 1;
				for (int j = 0; j < 4; ++j)
				{
					m_car.SwitchLed(j, (i == j));
				}
				break;
			}
		}
		m_car.UpdateGyro();
	}

	m_dir_control.SetMode(m_mode);
	m_speed_control.SetMode(m_mode);

	// Reset encoder count
	m_car.UpdateEncoder();

	m_dir_control.OnFinishWarmUp(&m_car);
	m_speed_control.OnFinishWarmUp(&m_car);

	// Dump first CCD sample
	for (int i = 0; i < libsc::LinearCcd::SENSOR_W; ++i)
	{
		m_car.CcdSampleProcess();
	}
	m_car.StartCcdSample();
	m_servo_state.prev_run = Clock::Time();
}

void LinearCcdApp::LedPass()
{
	const Clock::ClockInt time = Clock::Time();
	if (Clock::TimeDiff(time, m_led_state.prev_run) >= LED_FREQ)
	{
		if (m_mode > 0)
		{
			m_car.SwitchLed(m_mode - 1, m_led_state.flag);
		}
		else
		{
			for (int i = 0; i < 4; ++i)
			{
				m_car.SwitchLed(i, m_led_state.flag);
			}
		}
		m_led_state.flag ^= true;

		m_led_state.prev_run = time - (time % LED_FREQ);
	}
}

void LinearCcdApp::ServoPass()
{
	const Clock::ClockInt time = Clock::Time();
	if (Clock::TimeDiff(time, m_servo_state.prev_run) >= SERVO_FREQ
			&& m_car.IsCcdReady())
	{
#if defined(DEBUG_PRINT_INTERVAL) || defined(DEBUG_LCD_PRINT_INTERVAL)
		const Clock::ClockInt enter = Clock::Time();
#endif
#ifdef DEBUG_PRINT_INTERVAL
		iprintf("ccd freq: %d\n", Clock::TimeDiff(time, m_servo_state.prev_run));
#endif
#ifdef DEBUG_LCD_PRINT_INTERVAL
		m_car.LcdPrintString(libutil::String::Format("ccd freq: %d\n",
				Clock::TimeDiff(time, m_servo_state.prev_run)).c_str(),
				libutil::GetRgb565(0x44, 0xc5, 0xf5));
#endif

		m_servo_state.prev_run = time;
		m_car.StartCcdSample();

		const bitset<libsc::LinearCcd::SENSOR_W> &ccd_data = FilterCcdData(
				m_car.GetCcdSample());
		m_dir_control.Control(ccd_data);

#ifdef DEBUG_LCD_PRINT_CCD
		static int y = 0;
		const uint8_t buf_size = libsc::LinearCcd::SENSOR_W;
		uint8_t buf[buf_size] = {};
		for (int i = 0; i < libsc::LinearCcd::SENSOR_W; ++i)
		{
			buf[i] += ccd_data[i] ? 0xFF : 0x00;
		}
		m_car.LcdDrawGrayscalePixelBuffer(0, y, buf_size, 1, buf);
		if (y++ >= libsc::Lcd::H)
		{
			y = 0;
		}
#endif

#ifdef DEBUG_PRINT_CCD
		// Send CCD data through UART
		char str[libsc::LinearCcd::SENSOR_W];
		for (int i = 3; i < libsc::LinearCcd::SENSOR_W - 3; ++i)
		{
			str[i] = ccd_data[i] ? '#' : '.';
		}
		m_car.UartSendBuffer((uint8_t*)(str + 3), libsc::LinearCcd::SENSOR_W - 6);
		m_car.UartSendStr("\n");
#endif

#ifdef DEBUG_PRINT_INTERVAL
		iprintf("ccd time: %d\n", Clock::TimeDiff(Clock::Time(), enter));
#endif
#ifdef DEBUG_LCD_PRINT_INTERVAL
		m_car.LcdPrintString(libutil::String::Format("ccd time: %d\n",
				Clock::TimeDiff(Clock::Time(), enter)).c_str(),
				libutil::GetRgb565(0x44, 0xc5, 0xf5));
#endif
	}

	m_car.CcdSampleProcess();
}

void LinearCcdApp::SpeedControlPass()
{
	const Clock::ClockInt time = Clock::Time();
	if (Clock::TimeDiff(time, m_speed_state.prev_run) >= SPEED_CTRL_FREQ)
	{
#if defined(DEBUG_PRINT_INTERVAL) || defined(DEBUG_LCD_PRINT_INTERVAL)
		const Clock::ClockInt enter = Clock::Time();
#endif
#ifdef DEBUG_PRINT_INTERVAL
		iprintf("speed freq: %d\n", Clock::TimeDiff(time, m_speed_state.prev_run));
#endif
#ifdef DEBUG_LCD_PRINT_INTERVAL
		m_car.LcdPrintString(libutil::String::Format("spd freg: %d\n",
				Clock::TimeDiff(time, m_speed_state.prev_run)).c_str(),
				libutil::GetRgb565(0xf5, 0x44, 0xc5));
#endif

		m_speed_state.prev_run = time;

		m_speed_control.Control(&m_car);

#ifdef DEBUG_PRINT_INTERVAL
		iprintf("speed time: %d\n", Clock::TimeDiff(Clock::Time(), enter));
#endif
#ifdef DEBUG_LCD_PRINT_INTERVAL
		m_car.LcdPrintString(libutil::String::Format("spd time: %d\n",
				Clock::TimeDiff(Clock::Time(), enter)).c_str(),
				libutil::GetRgb565(0x44, 0xc5, 0xf5));
#endif
	}
}

bool LinearCcdApp::BtControlPass()
{
	return m_bt_control.Control(&m_car);
}

int LinearCcdApp::FwriteHandler(int, char *ptr, int len)
{
	if (m_instance)
	{
		m_instance->m_car.UartSendBuffer((const uint8_t*)ptr, len);
	}
	return len;
}

void LinearCcdApp::DetectStopLine()
{
	if (!m_car.IsLightSensorDetected(0) && !m_car.IsLightSensorDetected(1))
	{
		m_is_stop = true;
	}
}

void LinearCcdApp::DetectEmergencyStop()
{
	static bool is_startup = true;
	const Clock::ClockInt time = Clock::Time();
	if (is_startup && time > 2000 + INITIAL_DELAY)
	{
		is_startup = false;
	}

	const int16_t count = m_car.GetEncoderCount();
	if (!is_startup && abs(count) < 30)
	{
		// Emergency stop
		m_is_stop = true;
	}
}

bitset<libsc::LinearCcd::SENSOR_W> LinearCcdApp::FilterCcdData(
		const bitset<libsc::LinearCcd::SENSOR_W> &data) const
{
	bitset<libsc::LinearCcd::SENSOR_W> result = data;
	constexpr int block_size = 1;
	// Simple alg'm to filter out peculiar color in the middle
	for (int i = block_size; i < libsc::LinearCcd::SENSOR_W - block_size; ++i)
	{
		int value = 0;
		for (int j = 0; j < block_size; ++j)
		{
			value += data[i - j - 1];
		}
		for (int j = 0; j < block_size; ++j)
		{
			value += data[i + j + 1];
		}
		const int threshold = ceilf((block_size * 2) * 0.65f);
		if (value >= threshold)
		{
			result[i] = true;
		}
		else if (value <= block_size * 2 - threshold)
		{
			result[i] = false;
		}
	}
	return result;
}

}
