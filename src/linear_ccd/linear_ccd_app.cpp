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
#include <cstdlib>

#include <libsc/com/linear_ccd.h>
#include <libutil/clock.h>
#include <libutil/misc.h>
#include <libutil/pid_controller.h>
#include <libutil/pid_controller.tcc>
#include <libutil/string.h>

#include "linear_ccd/car.h"

#include "linear_ccd/linear_ccd_app.h"

using libutil::Clock;

#define LED_FREQ 250
#define SERVO_FREQ 9
#define SPEED_CTRL_FREQ 20

#define K_ID 0

namespace linear_ccd
{

namespace
{

struct SpeedConstant
{
	int encoder;
	int pwm;
	float kp;
	float ki;
	float kd;
};

constexpr SpeedConstant CONSTANTS[] =
{
		//{0, 0, 0.0f, 0.0f, 0.0f},
		//{120, 1280, 52.0f, 0.0f, 5.2f},
		{235, 1350, 55.0f, 0.0f, 5.5f},
		{400, 1630, 65.0f, 0.0f, 6.5f},
		{600, 2050, 95.0f, 0.0f, 9.5f}
};

}

LinearCcdApp *LinearCcdApp::m_instance = nullptr;

LinearCcdApp::SpeedState::SpeedState()
		: prev_run(0),
		  pid(CONSTANTS[K_ID].encoder, CONSTANTS[K_ID].kp, CONSTANTS[K_ID].ki,
				  CONSTANTS[K_ID].kd),
		  prev_count(0)
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
	m_car.SetMotorPower(CONSTANTS[K_ID].pwm);

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
		for (int i = 6; i < libsc::LinearCcd::SENSOR_W - 6; ++i)
		{
			str[i] = ccd_data[i] ? '#' : '.';
		}
		m_car.UartSendBuffer((uint8_t*)(str + 6), libsc::LinearCcd::SENSOR_W - 12);
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
		const uint32_t count = m_car.GetEncoderCount();
		const Uint count_diff = libsc::Encoder::CountDiff(count,
				m_speed_state.prev_count);
		m_speed_state.prev_count = count;
		//LOG_I("Encoder: %u", count_diff);
		int power = m_speed_state.pid.Calc(time, count_diff)
				+ CONSTANTS[K_ID].pwm;
		//LOG_I("Power: %d", power);
		//printf("%u\n", count_diff);
		m_car.SetMotorPower(libutil::Clamp<int>(-10000, power, 10000));

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
