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

#include <log.h>
#include <MK60_gpio.h>

#include "linear_ccd/debug.h"
#include "linear_ccd/kalman.h"

#include <libsc/com/linear_ccd.h>
#include <libutil/clock.h>
#include <libutil/misc.h>
#include <libutil/string.h>

#include "linear_ccd/car.h"
#include "linear_ccd/linear_ccd_app.h"

using libutil::Clock;

#define LED_FREQ 250
#define SERVO_FREQ 17
#define SPEED_CTRL_FREQ 21

namespace linear_ccd
{

LinearCcdApp *LinearCcdApp::m_instance = nullptr;

LinearCcdApp::LinearCcdApp()
		: m_dir_control(&m_car), m_is_stop(false), m_mode(0)
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
	if (Clock::TimeDiff(time, m_servo_state.prev_run) >= SERVO_FREQ)
	{
#ifdef DEBUG_PRINT_INTERVAL
		iprintf("servo freq: %d\n", Clock::TimeDiff(time, m_servo_state.prev_run));
#endif

		m_servo_state.prev_run = time;

		const bool *ccd_data = m_car.SampleCcd();
		m_dir_control.Control(ccd_data);

#ifdef DEBUG_LCD_PRINT_CCD
		static int y = 0;
		const uint8_t buf_size = (libsc::LinearCcd::SENSOR_W - 12) / 2;
		uint8_t buf[buf_size] = {};
		for (int i = 0, j = buf_size; i < libsc::LinearCcd::SENSOR_W - 12;
				++i)
		{
			if (i % 2 == 0)
			{
				--j;
			}
			buf[j] += ccd_data[i + 6] ? 0x7F : 0x00;
		}
		for (int i = 0; i < buf_size; ++i)
		{
			buf[i] = 0xFF - buf[i];
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
		for (int i = 6; i < libsc::LinearCcd::SENSOR_W - 6; ++i)
		{
			str[i] = ccd_data[i] ? '#' : '.';
		}
		m_car.UartSendBuffer((uint8_t*)(str + 6), libsc::LinearCcd::SENSOR_W - 12);
		m_car.UartSendStr("\n");
#endif
	}
}

void LinearCcdApp::SpeedControlPass()
{
	const Clock::ClockInt time = Clock::Time();
	if (Clock::TimeDiff(time, m_speed_state.prev_run) >= SPEED_CTRL_FREQ)
	{
#ifdef DEBUG_PRINT_INTERVAL
		iprintf("speed freq: %d\n", Clock::TimeDiff(time, m_speed_state.prev_run));
#endif

		m_speed_state.prev_run = time;

		m_speed_control.Control(&m_car);
	}
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

}
