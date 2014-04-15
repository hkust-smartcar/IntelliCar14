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
#define SERVO_FREQ 10
#define SPEED_CTRL_FREQ 20
#define INITIAL_DELAY 3000
#define MOTOR_MAX_PWM 7000

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
		{0, 0, 0.0f, 0.0f, 0.0f},
		//
		//{120, 1280, 52.0f, 0.0f, 5.2f},
		//{265, 1380, 55.0f, 2.8f, 5.5f},
		//{235, 1350, 55.0f, 3.0f, 5.5f},
		//{280, 1400, 58.0f, 3.0f, 5.8f},
		//{480, 2200, 0.0f, 0.0f, 0.0f},
		//{235, 0, 145.0f, 0.0f, 0.0f},
		//{235, 0, 143.0f, 0.0f, 0.0f},
		//{235, 0, 141.0f, 0.0f, 0.0f},
		//{235, 0, 141.0f, 0.0f, 0.0f},
		//{235, 0, 140.5f, 0.0f, 0.0f},
		//{235, 0, 140.333f, 0.0f, 0.0f},
		//{235, 0, 140.166f, 0.0f, 0.0f},
		//{235, 0, 140.0f, 0.0f, 0.0f},
		//{235, 0, 139.833f, 0.0f, 0.0f},
		//{235, 0, 139.666f, 0.0f, 0.0f},
		//{235, 0, 139.5f, 0.0f, 0.0f},
		//{235, 0, 139.0f, 0.0f, 0.0f},
		//{235, 0, 137.0f, 0.0f, 0.0f},
		//{235, 0, 135.0f, 0.0f, 0.0f},

		//{235, 0, 139.833f, 0.0f, 137.0f},
		//{235, 0, 139.833f, 0.0f, 136.75f},
		//{235, 0, 139.833f, 0.0f, 136.5f},
		//{235, 0, 139.833f, 0.0f, 136.25f},

		//{235, 0, 107.5f, 0.0f, 0.0f},
		//{235, 0, 106.5f, 0.0f, 0.0f},
		//{235, 0, 105.5f, 0.0f, 0.0f},
		//{235, 0, 104.5f, 0.0f, 0.0f},

		//{235, 0, 92.5f, 0.0f, 0.0f},
		//{235, 0, 91.875f, 0.0f, 0.0f},
		//{235, 0, 91.25f, 0.0f, 0.0f},
		//{235, 0, 90.625f, 0.0f, 0.0f},

		//{235, 0, 106.5f, 0.0f, 40.0f},
		//{235, 0, 106.5f, 0.0f, 35.0f},
		//{235, 0, 106.5f, 0.0f, 30.0f},
		//{235, 0, 106.5f, 0.0f, 25.0f},

		{180, 0, 106.5f, 0.0f, 27.5f},
		{235, 0, 106.5f, 0.0f, 27.5f},
		{290, 0, 106.5f, 0.0f, 27.5f},
		{330, 0, 106.5f, 0.0f, 27.5f},

		{235, 0, 139.833f, 0.0f, 136.75f},
		{235, 0, 139.833f, 0.0f, 136.75f},
		{235, 0, 139.833f, 0.0f, 136.75f},
		{235, 0, 139.833f, 0.0f, 136.75f},

		{480, 1720, 27.5f, 3.1f, 20.5f},
		{480, 1720, 27.5f, 3.1f, 13.5f},
		{480, 1720, 27.5f, 3.1f, 11.5f},
		{480, 1720, 27.5f, 3.1f, 9.5f},

		{480, 1720, 30.5f, 3.1f, 10.5f},
		{480, 1720, 27.5f, 3.1f, 15.5f},
		{480, 1720, 25.5f, 3.1f, 15.5f},
		{480, 1720, 22.5f, 3.1f, 15.5f},
		{400, 1630, 30.0f, 3.9f, 15.5f},
		{360, 1560, 30.0f, 3.9f, 15.5f},
		{320, 1480, 30.0f, 3.9f, 15.5f},
		{280, 1400, 28.0f, 3.9f, 8.5f},
		{235, 1350, 28.0f, 4.8f, 5.6f},
		{280, 1400, 58.0f, 3.0f, 5.8f},
		{235, 1350, 55.0f, 3.0f, 5.5f},
		{235, 1350, 55.0f, 3.0f, 5.5f},
		{235, 1350, 55.0f, 3.0f, 5.5f},
		{280, 1400, 58.0f, 3.0f, 5.8f},
		{235, 1350, 55.0f, 0.0f, 5.5f},
		{400, 1630, 65.0f, 0.0f, 6.5f},
		{600, 2050, 95.0f, 0.0f, 9.5f},
};

//#define TURN_CONSTANTS CONSTANTS

constexpr SpeedConstant TURN_CONSTANTS[] =
{
		{0, 0, 0.0f, 0.0f, 0.0f},
		//{120, 1280, 52.0f, 0.0f, 5.2f},
		//{265, 1380, 55.0f, 2.8f, 5.5f},
		//{235, 1350, 55.0f, 3.0f, 5.5f},
		//{280, 1400, 58.0f, 3.0f, 5.8f},
		//{360, 1560, 61.0f, 3.4f, 6.1f},

		{140, 0, 106.5f, 0.0f, 27.5f},
		{195, 0, 106.5f, 0.0f, 27.5f},
		{210, 0, 106.5f, 0.0f, 27.5f},
		{260, 0, 106.5f, 0.0f, 27.5f},

		{480, 1720, 30.5f, 3.1f, 15.5f},
		{480, 1720, 27.5f, 3.1f, 15.5f},
		{480, 1720, 25.5f, 3.1f, 15.5f},
		{480, 1720, 22.5f, 3.1f, 15.5f},
		{440, 1670, 30.0f, 3.9f, 15.5f},
		{320, 1480, 30.0f, 3.9f, 15.5f},
		{280, 1400, 30.0f, 3.9f, 15.5f},
		{235, 1350, 30.0f, 3.9f, 15.5f},
		{235, 1350, 28.0f, 3.9f, 8.5f},
		{235, 1350, 55.0f, 3.0f, 5.5f},
		{235, 1350, 55.0f, 3.0f, 5.5f},
		{280, 1400, 58.0f, 3.0f, 5.8f},
		{235, 1350, 55.0f, 0.0f, 5.5f},
		{400, 1630, 65.0f, 0.0f, 6.5f},
		{600, 2050, 95.0f, 0.0f, 9.5f},
};

}

LinearCcdApp *LinearCcdApp::m_instance = nullptr;

LinearCcdApp::SpeedState::SpeedState()
		: prev_run(0),
		  pid(CONSTANTS[0].encoder, CONSTANTS[0].kp, CONSTANTS[0].ki,
				  CONSTANTS[0].kd),
		  prev_count(0)
{
	pid.SetILimit(1500);
}

LinearCcdApp::LinearCcdApp()
		: m_dir_control(&m_car), m_is_stop(false), m_speed_choice(0)
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
				m_speed_choice = i + 1;
				for (int j = 0; j < 4; ++j)
				{
					m_car.SwitchLed(j, (i == j));
				}
				break;
			}
		}
		m_car.UpdateGyro();
	}

	m_car.SetMotorPower(CONSTANTS[m_speed_choice].pwm);
	m_speed_state.pid.SetSetpoint(CONSTANTS[m_speed_choice].encoder);
	m_speed_state.pid.SetKp(CONSTANTS[m_speed_choice].kp);
	m_speed_state.pid.SetKi(CONSTANTS[m_speed_choice].ki);
	m_speed_state.pid.SetKd(CONSTANTS[m_speed_choice].kd);
	m_speed_state.pid.Restart();
	m_dir_control.SetConstant(m_speed_choice);

	// Reset encoder count
	m_car.UpdateEncoder();
}

void LinearCcdApp::LedPass()
{
	const Clock::ClockInt time = Clock::Time();
	if (Clock::TimeDiff(time, m_led_state.prev_run) >= LED_FREQ)
	{
		if (m_speed_choice > 0)
		{
			m_car.SwitchLed(m_speed_choice - 1, m_led_state.flag);
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
		const bool *ccd_data = m_car.SampleCcd();
		m_dir_control.Control(ccd_data);

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

		m_servo_state.prev_run = time;
	}
}

void LinearCcdApp::SpeedControlPass()
{
	static bool is_startup = true;
	const Clock::ClockInt time = Clock::Time();

	if (Clock::TimeDiff(time, m_speed_state.prev_run) >= SPEED_CTRL_FREQ)
	{
		m_speed_state.prev_run = time;

		m_car.UpdateEncoder();
		const int16_t count = m_car.GetEncoderCount();
#ifdef DEBUG_PRINT_ENCODER
		iprintf("%d\n", count);
#endif

		if (!is_startup && abs(count) < 30)
		{
			// Emergency stop
			m_is_stop = true;
			return;
		}

		int power;
		if (abs(m_car.GetTurning()) <= 35)
		{
			SetConstant(true);
			power = m_speed_state.pid.Calc(time, count)
					+ CONSTANTS[m_speed_choice].pwm;
		}
		else
		{
			SetConstant(false);
			power = m_speed_state.pid.Calc(time, count)
					+ TURN_CONSTANTS[m_speed_choice].pwm;
		}
		//LOG_I("Power: %d", power);
		//printf("%u\n", count_diff);
		//iprintf("%d, %d\n", count, power);

		// Prevent the output going crazy due to initially idle encoder
		if (is_startup && time < 1000 + INITIAL_DELAY)
		{
			const int clamp_power = m_car.GetMotorPower()
					+ libutil::Clamp<int>(-280, power - m_car.GetMotorPower(),
							280);
			m_car.SetMotorPower(clamp_power);
		}
		else
		{
			if (is_startup)
			{
				is_startup = false;
				//m_speed_state.pid.Restart();
			}
			m_car.SetMotorPower(libutil::Clamp<int>(0, power, MOTOR_MAX_PWM));
		}

#ifdef DEBUG
		// Send speed PID through UART1
		//m_car.UartSendStr(libutil::String::Format("%u\n", power).c_str());
#endif
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

void LinearCcdApp::SetConstant(const bool is_straight)
{
	if (is_straight)
	{
		m_speed_state.pid.SetSetpoint(CONSTANTS[m_speed_choice].encoder);
		m_speed_state.pid.SetKp(CONSTANTS[m_speed_choice].kp);
		m_speed_state.pid.SetKi(CONSTANTS[m_speed_choice].ki);
		m_speed_state.pid.SetKd(CONSTANTS[m_speed_choice].kd);
	}
	else
	{
		m_speed_state.pid.SetSetpoint(TURN_CONSTANTS[m_speed_choice].encoder);
		m_speed_state.pid.SetKp(TURN_CONSTANTS[m_speed_choice].kp);
		m_speed_state.pid.SetKi(TURN_CONSTANTS[m_speed_choice].ki);
		m_speed_state.pid.SetKd(TURN_CONSTANTS[m_speed_choice].kd);
	}
}

}
