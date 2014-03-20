/*
 * linear_ccd_app.cpp
 * Linear CCD App
 *
 * Author: Louis Mo, Ming Tsang, Spartey Chen
 * Copyright (c) 2014 HKUST SmartCar Team
 */

#include <hw_common.h>
#include <syscall.h>

#include <cstdint>

#include <libsc/com/linear_ccd.h>
#include <libutil/clock.h>
#include <libutil/string.h>

#include "libutil/pid_controller.h"
#include "libutil/pid_controller.tcc"

#include "linear_ccd/car.h"

#include "linear_ccd/linear_ccd_app.h"

using libutil::Clock;

#define LED_FREQ 250
#define SERVO_FREQ 9
#define SPEED_CTRL_FREQ 100

#define SPEED_SP 300
#define SPEED_KP 2.666f
#define SPEED_KI 0.0f
#define SPEED_KD 8.0f

namespace linear_ccd
{

LinearCcdApp *LinearCcdApp::m_instance = nullptr;

LinearCcdApp::SpeedState::SpeedState()
		: prev_run(0),
		  pid(SPEED_SP, SPEED_KP, SPEED_KI, SPEED_KD)
{}

LinearCcdApp::LinearCcdApp()
		: m_dir_control(&m_car)
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
	libutil::Clock::Init();

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
		SpeedControlPass();
		LedPass();
	}
}

void LinearCcdApp::LedPass()
{
	const Clock::ClockInt time = Clock::Time();
	if (Clock::TimeDiff(time, m_led_state.prev_run) >= LED_FREQ)
	{
		m_car.SwitchLed(0, m_led_state.flag);
		m_car.SwitchLed(1, !m_led_state.flag);
		m_led_state.flag ^= true;

		m_led_state.prev_run = time - (time % LED_FREQ);
	}
}

void LinearCcdApp::ServoPass()
{
	const Clock::ClockInt time = Clock::Time();
	if (Clock::TimeDiff(time, m_servo_state.prev_run) >= SERVO_FREQ)
	{
		const bool *ccd_data = m_car.SampleCcd();
		m_dir_control.Control(ccd_data);


#ifdef DEBUG
		// Send CCD data through UART
		char str[libsc::LinearCcd::SENSOR_W];
		for (int i = 0; i < libsc::LinearCcd::SENSOR_W; ++i)
		{
			str[i] = ccd_data[i] ? '#' : '.';
		}
		m_car.UartSendBuffer((uint8_t*)str, libsc::LinearCcd::SENSOR_W);
		m_car.UartSendStr("\n");
#endif

		m_servo_state.prev_run = time;
	}
}

void LinearCcdApp::SpeedControlPass()
{
	const Clock::ClockInt time = Clock::Time();
	if (Clock::TimeDiff(time, m_speed_state.prev_run) >= SPEED_CTRL_FREQ)
	{
		uint16_t power = m_speed_state.pid.Calc(time,
				m_car.GetEncoderCount());
		power = 1500;
		m_car.SetMotorPower(power);

#ifdef DEBUG
		// Send speed PID through UART
		//m_car.UartSendStr(libutil::String::Format("%u\n", power).c_str());
#endif

		m_speed_state.prev_run = time;
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

}
