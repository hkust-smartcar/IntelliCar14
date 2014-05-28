/*
 * speed_control_1.cpp
 * Speed controller Gen1, using PID
 *
 * Author: Louis Mo, Ming Tsang, Spartey Chen
 * Copyright (c) 2014 HKUST SmartCar Team
 */

#include <cstdint>

#include <libutil/clock.h>
#include <libutil/misc.h>
#include <libutil/pid_controller.h>
#include <libutil/pid_controller.tcc>

#include "linear_ccd/car.h"
#include "linear_ccd/linear_ccd_app.h"
#include "linear_ccd/speed_control_1.h"

using namespace std;
using libutil::Clock;

#define MOTOR_MAX_PWM 6500

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

		//{180, 0, 106.5f, 0.0f, 27.5f},
		//{235, 0, 106.5f, 0.0f, 27.5f},
		//{290, 0, 106.5f, 0.0f, 27.5f},
		//{330, 0, 106.5f, 0.0f, 27.5f},

		/* ??? Values for new tires on air ???
		{120, 0, 26.8f, 0.0f, 10.0f},
		{180, 0, 26.8f, 0.0f, 10.0f},
		{235, 0, 26.8f, 0.0f, 10.0f},
		{290, 0, 26.8f, 0.0f, 10.0f},
		*/
		{200, 0, 578.0f, 0.0f, 57.8f},
		{220, 0, 578.0f, 0.0f, 57.8f},
		{240, 0, 578.0f, 0.0f, 57.8f},
		{280, 0, 578.0f, 0.0f, 57.8f},

		{120, 0, 578.0f, 0.0f, 57.8f},
		{180, 0, 578.0f, 0.0f, 57.8f},
		{200, 0, 578.0f, 0.0f, 57.8f},
		{220, 0, 578.0f, 0.0f, 57.8f},

		{350, 0, 555.0f, 0.0f, 27.5f},
		{350, 0, 555.0f, 0.0f, 27.5f},
		{350, 0, 555.0f, 0.0f, 27.5f},
		{350, 0, 555.0f, 0.0f, 27.5f},

		// Mode 5~8 are manual modes
		{155, 0, 26.8f, 0.0f, 10.0f},
		{-155, 0, 26.8f, 0.0f, 10.0f},
		{240, 0, 26.8f, 0.0f, 10.0f},
		{-240, 0, 26.8f, 0.0f, 10.0f},

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

		// Successful trial
		// Set 1
		//PWM[0, 7k]
		//turn threshold 35
		//{235, 0, 106.5f, 0.0f, 27.5f},

		// Set 2
		//PWM[0, 7k]
		//turn threshold 35
		//{290, 0, 106.5f, 0.0f, 27.5f},

		// Set 3
		//PWM[-4k, 7k]
		//turn threshold 35
		//{305, 0, 106.5f, 0.0f, 27.5f}, // V. Good

		// Set 4
		//PWM[-4k, 7k]
		//turn threshold 40
		//{340, 0, 106.5f, 0.0f, 27.5f}, // Acceptable
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

		//{140, 0, 106.5f, 0.0f, 27.5f},
		//{250, 0, 106.5f, 0.0f, 27.5f},

		//{160, 0, 578.0f, 0.0f, 57.8f},
		{140, 0, 578.0f, 0.0f, 57.8f},
		{140, 0, 578.0f, 0.0f, 57.8f},
		{140, 0, 578.0f, 0.0f, 57.8f},
		{140, 0, 578.0f, 0.0f, 57.8f},

		{120, 0, 578.0f, 0.0f, 57.8f},
		{140, 0, 578.0f, 0.0f, 57.8f},
		{140, 0, 578.0f, 0.0f, 57.8f},
		{140, 0, 578.0f, 0.0f, 57.8f},

		// Mode 5~8 are manual modes
		{155, 0, 106.5f, 0.0f, 27.5f},
		{-155, 0, 106.5f, 0.0f, 27.5f},
		{240, 0, 106.5f, 0.0f, 27.5f},
		{-240, 0, 106.5f, 0.0f, 27.5f},

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

		// Successful trial
		// Set 1
		//PWM[0, 7k]
		//turn threshold 35
		//{195, 0, 106.5f, 0.0f, 27.5f},

		// Set 2
		//PWM[0, 7k]
		//turn threshold 35
		//{205, 0, 106.5f, 0.0f, 27.5f},

		// Set 3
		//PWM[-4k, 7k]
		//turn threshold 35
		//{205, 0, 106.5f, 0.0f, 27.5f}, // V. Good

		// Set 4
		//PWM[-4k, 7k]
		//turn threshold 40
		//{225, 0, 106.5f, 0.0f, 27.5f}, // Acceptable
};

}

SpeedControl1::SpeedControl1()
		: m_pid(CONSTANTS[0].encoder, CONSTANTS[0].kp, CONSTANTS[0].ki,
					CONSTANTS[0].kd),
		  m_mode(0),
		  m_is_startup(true)
{
	m_pid.SetILimit(1500);
}

void SpeedControl1::OnFinishWarmUp(Car*)
{
	m_pid.Restart();
}

void SpeedControl1::Control(Car *car)
{
	car->UpdateEncoder();
	const int16_t count = car->GetEncoderCount();
	const Clock::ClockInt time = Clock::Time();
#ifdef DEBUG_PRINT_ENCODER
	iprintf("%d\n", count);
#endif

	int power;
	if (abs(car->GetTurning()) <= 38)
	{
		UpdatePid(true);
		power = m_pid.Calc(time, count) + CONSTANTS[m_mode].pwm;
	}
	else
	{
		UpdatePid(false);
		power = m_pid.Calc(time, count) + TURN_CONSTANTS[m_mode].pwm;
	}
	//LOG_I("Power: %d", power);
	//printf("%u\n", count_diff);
	//iprintf("%d, %d\n", count, power);

	// Prevent the output going crazy due to initially idle encoder
	if (m_is_startup && time < 1000 + LinearCcdApp::INITIAL_DELAY)
	{
		const int clamp_power = car->GetMotorPower()
				+ libutil::Clamp<int>(-350, power - car->GetMotorPower(),
						350);
		car->SetMotorPower(clamp_power);
	}
	else
	{
		if (m_is_startup)
		{
			m_is_startup = false;
			m_pid.Restart();
		}
		if (abs(power - car->GetMotorPower()) > 1800)
		{
			// Max 2500 diff in one step
			power = libutil::Clamp<int>(car->GetMotorPower() - 1800, power,
					car->GetMotorPower() + 1800);
		}
		car->SetMotorPower(libutil::Clamp<int>(-MOTOR_MAX_PWM, power, MOTOR_MAX_PWM));
	}

#ifdef DEBUG
	// Send speed PID through UART1
	//car->UartSendStr(libutil::String::Format("%u\n", power).c_str());
#endif
}

void SpeedControl1::UpdatePid(const bool is_straight)
{
	if (is_straight)
	{
		m_pid.SetSetpoint(CONSTANTS[m_mode].encoder);
		m_pid.SetKp(CONSTANTS[m_mode].kp);
		m_pid.SetKi(CONSTANTS[m_mode].ki);
		m_pid.SetKd(CONSTANTS[m_mode].kd);
	}
	else
	{
		m_pid.SetSetpoint(TURN_CONSTANTS[m_mode].encoder);
		m_pid.SetKp(TURN_CONSTANTS[m_mode].kp);
		m_pid.SetKi(TURN_CONSTANTS[m_mode].ki);
		m_pid.SetKd(TURN_CONSTANTS[m_mode].kd);
	}
}

}
