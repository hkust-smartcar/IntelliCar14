/*
 * speed_control_1.cpp
 * Speed controller Gen1, using PID
 *
 * Author: Louis Mo, Ming Tsang, Spartey Chen
 * Copyright (c) 2014 HKUST SmartCar Team
 */

#include <cstdint>

#include <libsc/k60/system_timer.h>
#include <libsc/k60/timer.h>
#include <libutil/misc.h>
#include <libutil/pid_controller.h>
#include <libutil/pid_controller.tcc>

#include "linear_ccd/config.h"
#include "linear_ccd/beep_manager.h"
#include "linear_ccd/car.h"
#include "linear_ccd/linear_ccd_app.h"
#include "linear_ccd/speed_control_1.h"
#include "linear_ccd/turn_hint.h"

using namespace libsc::k60;
using namespace std;

#define MOTOR_MAX_PWM 7000
#define ACCELERATE_DELAY 7
#define I_LIMIT 3500

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

		{320, 0, 104.5f, 100.0f, 0.002f},
		{320, 0, 104.5f, 100.0f, 0.002f},
		{320, 0, 104.5f, 100.0f, 0.002f},
		{320, 0, 104.5f, 100.0f, 0.002f},
		{320, 0, 104.5f, 100.0f, 0.002f},

		{250, 0, 209.0f, 0.0f, 7.5f},
		{250, 0, 209.0f, 0.0f, 7.5f},
		{250, 0, 209.0f, 0.0f, 7.5f},
		{250, 0, 209.0f, 0.0f, 7.5f},
		{250, 0, 209.0f, 0.0f, 7.5f},

		{250, 0, 104.5f, 80.0f, 0.0f},
		{250, 0, 104.5f, 120.0f, 0.0f},
		{250, 0, 104.5f, 160.0f, 0.0f},
		{250, 0, 104.5f, 200.0f, 0.0f},
		{250, 0, 104.5f, 240.0f, 0.0f},

		{350, 0, 209.0f, 0.0f, 0.0f},
		{350, 0, 259.0f, 0.0f, 0.0f},
		{350, 0, 309.0f, 0.0f, 0.0f},
		{350, 0, 359.0f, 0.0f, 0.0f},
		{350, 0, 409.0f, 0.0f, 0.0f},

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
		{235, 0, 106.5f, 0.0f, 27.5f},

		// Set 2
		//PWM[0, 7k]
		//turn threshold 35
		{290, 0, 106.5f, 0.0f, 27.5f},

		// Set 3
		//PWM[-4k, 7k]
		//turn threshold 35
		{305, 0, 106.5f, 0.0f, 27.5f}, // V. Good

		// Set 4
		//PWM[-4k, 7k]
		//turn threshold 40
		{340, 0, 106.5f, 0.0f, 27.5f}, // Acceptable

		// Set 5
		//Before fixing tire
		{240, 0, 578.0f, 0.0f, 57.8f},

		// Set 6
		// Before mech change
		{380, 0, 578.0f, 0.0f, 57.8f},

		// Set 7
		{340, 0, 578.0f, 0.0f, 57.8f},

		// Set 8
		// P only
		//{250, 0, 209.0f, 0.0f, 0.0f},

		// Set 9
		{310, 0, 104.5f, 100.0f, 0.002f},
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

		{310, 0, 104.5f, 100.0f, 0.002f},
		{310, 0, 104.5f, 100.0f, 0.002f},
		{310, 0, 104.5f, 100.0f, 0.002f},
		{310, 0, 104.5f, 100.0f, 0.002f},
		{310, 0, 104.5f, 100.0f, 0.002f},

		{200, 0, 240.0f, 0.0f, 2.5f},
		{200, 0, 240.0f, 0.0f, 5.0f},
		{200, 0, 240.0f, 0.0f, 7.5f},
		{200, 0, 240.0f, 0.0f, 10.0f},
		{200, 0, 240.0f, 0.0f, 12.5f},

		{250, 0, 104.5f, 80.0f, 0.0f},
		{250, 0, 104.5f, 120.0f, 0.0f},
		{250, 0, 104.5f, 160.0f, 0.0f},
		{250, 0, 104.5f, 200.0f, 0.0f},
		{250, 0, 104.5f, 240.0f, 0.0f},

		{240, 0, 209.0f, 0.0f, 0.0f},
		{240, 0, 259.0f, 0.0f, 0.0f},
		{240, 0, 309.0f, 0.0f, 0.0f},
		{240, 0, 359.0f, 0.0f, 0.0f},
		{240, 0, 409.0f, 0.0f, 0.0f},

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
		{195, 0, 106.5f, 0.0f, 27.5f},

		// Set 2
		//PWM[0, 7k]
		//turn threshold 35
		{205, 0, 106.5f, 0.0f, 27.5f},

		// Set 3
		//PWM[-4k, 7k]
		//turn threshold 35
		{205, 0, 106.5f, 0.0f, 27.5f}, // V. Good

		// Set 4
		//PWM[-4k, 7k]
		//turn threshold 40
		{225, 0, 106.5f, 0.0f, 27.5f}, // Acceptable

		// Set 5
		//Before fixing tire
		{140, 0, 578.0f, 0.0f, 47.8f},
		{150, 0, 578.0f, 0.0f, 47.8f},
		{160, 0, 578.0f, 0.0f, 47.8f}, // Good
		{170, 0, 578.0f, 0.0f, 47.8f},
		{180, 0, 578.0f, 0.0f, 47.8f},

		// Set 6
		{220, 0, 578.0f, 0.0f, 47.8f},
		{240, 0, 578.0f, 0.0f, 47.8f},
		{260, 0, 578.0f, 0.0f, 47.8f},
		{270, 0, 578.0f, 0.0f, 47.8f},
		{300, 0, 578.0f, 0.0f, 47.8f},

		// Set 7
		{270, 0, 578.0f, 0.0f, 47.8f},

		// Set 9
		{300, 0, 104.5f, 100.0f, 0.002f},
};

//#define PRE_TURN_CONSTANTS CONSTANTS
constexpr SpeedConstant PRE_TURN_CONSTANTS[] =
{
		{0, 0, 0.0f, 0.0f, 0.0f},

		{280, 0, 104.5f, 100.0f, 0.0f},
		{280, 0, 104.5f, 100.0f, 0.0f},
		{280, 0, 104.5f, 100.0f, 0.0f},
		{280, 0, 104.5f, 100.0f, 0.0f},
		{280, 0, 104.5f, 100.0f, 0.0f},

		{250, 0, 209.0f, 0.0f, 7.5f},
		{250, 0, 209.0f, 0.0f, 7.5f},
		{250, 0, 209.0f, 0.0f, 7.5f},
		{250, 0, 209.0f, 0.0f, 7.5f},
		{250, 0, 209.0f, 0.0f, 7.5f},

		{250, 0, 104.5f, 80.0f, 0.0f},
		{250, 0, 104.5f, 120.0f, 0.0f},
		{250, 0, 104.5f, 160.0f, 0.0f},
		{250, 0, 104.5f, 200.0f, 0.0f},
		{250, 0, 104.5f, 240.0f, 0.0f},

		{300, 0, 209.0f, 0.0f, 0.0f},
		{300, 0, 259.0f, 0.0f, 0.0f},
		{300, 0, 309.0f, 0.0f, 0.0f},
		{300, 0, 359.0f, 0.0f, 0.0f},
		{300, 0, 409.0f, 0.0f, 0.0f},

		// Set 6
		{290, 0, 578.0f, 0.0f, 42.8f},

		// Set 7
		{260, 0, 578.0f, 0.0f, 42.8f},
};

}

SpeedControl1::SpeedControl1(Car *car)
		: m_car(car), m_pid(CONSTANTS[0].encoder, CONSTANTS[0].kp,
				  CONSTANTS[0].ki, CONSTANTS[0].kd),
		  m_mode(0), m_straight_mode_delay(0),
		  m_is_startup(true)
{
	m_pid.SetILimit(I_LIMIT);
}

void SpeedControl1::OnFinishWarmUp()
{
	m_pid.Restart();
}

void SpeedControl1::Control()
{
	m_car->UpdateEncoder();
	const int16_t count = m_car->GetEncoderCount();
	const Timer::TimerInt time = SystemTimer::Time();
#ifdef DEBUG_PRINT_ENCODER
	iprintf("%d\n", count);
#endif

	int power = m_pid.Calc(time, count);
	/*
	if (abs(car->GetTurning()) <= Config::GetTurnThreshold())
	{
		SetTurnHint(TurnHint::STRAIGHT);
		power = m_pid.Calc(time, count) + CONSTANTS[m_mode].pwm;
	}
	else
	{
		SetTurnHint(TurnHint::TURN);
		power = m_pid.Calc(time, count) + TURN_CONSTANTS[m_mode].pwm;
	}
	*/
	//LOG_I("Power: %d", power);
	//printf("%u\n", count_diff);
	//iprintf("%d, %d\n", count, power);

	// Prevent the output going crazy due to initially idle encoder
	if (m_is_startup && time < 250 + LinearCcdApp::INITIAL_DELAY)
	{
		const int clamp_power = m_car->GetMotorPower()
				+ libutil::Clamp<int>(-400, power - m_car->GetMotorPower(), 400);
		m_car->SetMotorPower(clamp_power);
	}
	else
	{
		if (m_is_startup)
		{
			m_is_startup = false;
			m_pid.Restart();
		}
/*
		if (abs(power - car->GetMotorPower()) > 3500)
		{
			// Max 3500 diff in one step
			power = libutil::Clamp<int>(car->GetMotorPower() - 3500, power,
					car->GetMotorPower() + 3500);
		}
*/
		if (power < 0)
		{
			++m_reverse_count;
			if (power >= -2000
					&& m_reverse_count < (m_pid.GetSetpoint() >> 3))
			{
				if (TURN_CONSTANTS[m_mode].encoder == CONSTANTS[m_mode].encoder)
				{
					power = 0;
				}
				else if (m_car->GetTurning() <= Config::GetTurnThreshold())
				{
					power = 0;
				}
			}
		}
		else
		{
			m_reverse_count = 0;
		}
#ifdef DEBUG_BEEP_REVERSE_MOTOR
		if (power < 0)
		{
			BeepManager::GetInstance(m_car)->Beep(100);
		}
#endif
		m_car->SetMotorPower(libutil::Clamp<int>(-MOTOR_MAX_PWM, power,
				MOTOR_MAX_PWM));
	}

#ifdef DEBUG
	// Send speed PID through UART1
	//car->UartSendStr(libutil::String::Format("%u\n", power));
#endif
}

void SpeedControl1::SetTurnHint(const TurnHint hint)
{
	switch (hint)
	{
	case TurnHint::STRAIGHT:
		if (m_straight_mode_delay == static_cast<uint8_t>(-1))
		{
			m_straight_mode_delay = ACCELERATE_DELAY;
		}
		else if (m_straight_mode_delay == 0)
		{
			m_pid.SetSetpoint(CONSTANTS[m_mode].encoder);
			m_pid.SetKp(CONSTANTS[m_mode].kp);
			m_pid.SetKi(CONSTANTS[m_mode].ki);
			m_pid.SetKd(CONSTANTS[m_mode].kd);
		}
		else
		{
			--m_straight_mode_delay;
		}
		break;

	case TurnHint::PRE_TURN:
		m_pid.SetSetpoint(PRE_TURN_CONSTANTS[m_mode].encoder);
		m_pid.SetKp(PRE_TURN_CONSTANTS[m_mode].kp);
		m_pid.SetKi(PRE_TURN_CONSTANTS[m_mode].ki);
		m_pid.SetKd(PRE_TURN_CONSTANTS[m_mode].kd);
		break;

	case TurnHint::TURN:
		{
			/*
			const int min_sp = TURN_CONSTANTS[m_mode].encoder;
			const int max_sp = CONSTANTS[m_mode].encoder;
			const int diff_sp = max_sp - min_sp;
			const int sp = abs(m_car->GetTurning()) / 100.0f * diff_sp + min_sp;
			m_pid.SetSetpoint(sp);
			*/
			m_pid.SetSetpoint(TURN_CONSTANTS[m_mode].encoder);
			m_pid.SetKp(TURN_CONSTANTS[m_mode].kp);
			m_pid.SetKi(TURN_CONSTANTS[m_mode].ki);
			m_pid.SetKd(TURN_CONSTANTS[m_mode].kd);
			m_straight_mode_delay = static_cast<uint8_t>(-1);
		}
		break;
	}
}

}
