/*
 * linear_ccd_app.cpp
 * Linear CCD App
 *
 * Author: Louis Mo, Ming Tsang, Spartey Chan
 * Copyright (c) 2014 HKUST SmartCar Team
 */

#include <syscall.h>

#include <cstdint>

#include <libutil/clock.h>

#include "pid_controller.h"

#include "linear_ccd/linear_ccd_app.h"

using libutil::Clock;

#define LED_FREQ 250
#define SPEED_CTRL_FREQ 100

#define SPEED_SP 300
#define SPEED_KP 2.666f
#define SPEED_KI 0.0f
#define SPEED_KD 8.0f

namespace linear_ccd
{

LinearCcdApp *LinearCcdApp::m_instance = nullptr;

LinearCcdApp::LinearCcdApp()
		: m_pid_controller(SPEED_SP, SPEED_KP, SPEED_KI, SPEED_KD)
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

	while (true)
	{
		LedPass();
		SpeedControlPass();
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

		m_led_state.prev_run %= time;
	}
}

void LinearCcdApp::SpeedControlPass()
{
	const Clock::ClockInt time = Clock::Time();
	if (Clock::TimeDiff(time, m_speed_control_state.prev_run) >= SPEED_CTRL_FREQ)
	{
		const uint16_t power = m_pid_controller.Calc(time, m_car.GetEncoderCount());
		m_car.SetMotorPower(power);

		m_speed_control_state.prev_run = time;
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
