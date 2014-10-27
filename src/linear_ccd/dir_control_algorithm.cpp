/*
 * dir_control_algorithm.cpp
 * Algorithm for direction control
 *
 * Author: Louis Mo, Ming Tsang, Spartey Chen
 * Copyright (c) 2014 HKUST SmartCar Team
 */

#include <cstdint>
#include <cstdio>
#include <cmath>
#include <algorithm>
#include <bitset>

#include "linear_ccd/debug.h"

#include <libutil/kalman_filter.h>
#include <libutil/misc.h>
#include <libutil/positional_pid_controller.h>
#include <libutil/string.h>

#include "linear_ccd/beep_manager.h"
#include "linear_ccd/config.h"
#include "linear_ccd/car.h"
#include "linear_ccd/dir_control_algorithm.h"
#include "linear_ccd/track_analyzer.h"
#include "linear_ccd/turn_hint.h"

using namespace std;
using namespace libsc::k60;

//#define VALID_PIXEL (128 - 6)
//#define VALID_OFFSET 3
#define VALID_PIXEL Config::GetCcdValidPixel()
#define VALID_OFFSET Config::GetCcdValidPixelOffset()

#define CCD_MID_POS 64

#define KALMAN_FILTER_Q 0.0000005f
#define KALMAN_FILTER_R 0.00005f

namespace linear_ccd
{

namespace
{

struct ServoConstant
{
	float kp;
	float ki;
	float kd;
	uint8_t edge;
};

// The first one is being used when the motor is stopped (manual mode)
constexpr ServoConstant CONSTANTS[] =
{
		{0.0f, 0.0f, 0.0f, 0},
		//
		//{1.57f, 0.0f, 0.0f, 62},
		//{1.62f, 0.0f, 0.0f, 62},
		//{1.67f, 0.0f, 0.0f, 62},

		// Down
		{1.242f, 0.0f, 0.402f, 25},
		{1.242f, 0.0f, 0.602f, 25},
		{1.242f, 0.0f, 0.802f, 25},
		{1.242f, 0.0f, 1.002f, 25},
		{1.242f, 0.0f, 1.202f, 25},

		// Up
		{0.0f, 0.0f, 0.0f, 0},
		{0.0f, 0.0f, 0.0f, 0},
		{0.0f, 0.0f, 0.0f, 0},
		{0.0f, 0.0f, 0.0f, 0},
		{0.0f, 0.0f, 0.0f, 0},

		{3.27f, 0.0f, 0.45f, 36},
		{3.27f, 0.0f, 0.45f, 36},
		{3.27f, 0.0f, 0.45f, 36},
		{3.27f, 0.0f, 0.45f, 36},
		{3.27f, 0.0f, 0.45f, 36},

		{1.07f, 0.0f, 0.74f, 20},

		// Old
		//{1.275f, 0.0f, 0.88f, 45},
		{1.55f, 0.0f, 0.5f, 60},
		{3.05f, 0.0f, 1.0f, 60},
		{0.9f, 0.0f, 1.0f, 60},
		//{1.55f, 0.0f, 0.0f, 60},

		// Successful trial
		// Set 1
		//PWM[0, 7k]
		//turn threshold 35
		{1.08f, 0.0f, 0.98f, 65}, // Average
		{1.00f, 0.0f, 0.90f, 65}, // Good
		{0.98f, 0.0f, 0.88f, 65}, // Good
		{0.80f, 0.0f, 0.78f, 65}, // Worst

		// Set 2
		//PWM[0, 7k]
		//turn threshold 35
		{1.2f, 0.0f, 0.0f, 62}, // Good
		{1.44f, 0.0f, 0.0f, 62}, // Acceptable

		// Set 3
		//PWM[-4k, 7k]
		//turn threshold 35
		{1.57f, 0.0f, 1.02f, 62}, // V. Good

		// Set 4
		//PWM[-4k, 7k]
		//turn threshold 40
		{1.57f, 0.0f, 1.02f, 62},

		// Set 5
		//Before fixing tire
		{2.27f, 0.0f, 1.49f, 35},

		// Set 6
		// Down
		{1.77f, 0.0f, 1.32f, 49},
		// Up
		{1.47f, 0.0f, 1.19f, 23},

		// Set 7
		{2.47f, 0.0f, 1.32f, 49},

		// Set 8
		// Down
		{8.57f, 0.0f, 0.29f, 35},
		{8.57f, 0.0f, 0.32f, 35},
		{8.57f, 0.0f, 0.35f, 35},
		{8.57f, 0.0f, 0.38f, 35},
		{8.57f, 0.0f, 0.41f, 35},

		// Set 9
		{0.87f, 0.0f, 0.3f, 35},

		// Set 10
		{2.642f, 0.0f, 0.092f, 25},

		// Protection
		{0.0f, 0.0f, 0.0f, 0},
		{0.0f, 0.0f, 0.0f, 0},
		{0.0f, 0.0f, 0.0f, 0},
		{0.0f, 0.0f, 0.0f, 0},
		{0.0f, 0.0f, 0.0f, 0},
};

constexpr ServoConstant TURN_CONSTANTS[] =
{
		{0.0f, 0.0f, 0.0f, 0},

		{15.81f, 0.0f, 0.015f, 25},
		{15.81f, 0.0f, 0.015f, 25},
		{15.81f, 0.0f, 0.015f, 25},
		{15.81f, 0.0f, 0.015f, 25},
		{15.81f, 0.0f, 0.015f, 25},

		{0.0f, 0.0f, 0.0f, 0},
		{0.0f, 0.0f, 0.0f, 0},
		{0.0f, 0.0f, 0.0f, 0},
		{0.0f, 0.0f, 0.0f, 0},
		{0.0f, 0.0f, 0.0f, 0},

		// Set 5
		//Before fixing tire
		{3.25f, 0.0f, 1.34f, 35},

		// Set 9
		{13.81f, 0.0f, 1.15f, 35},

		// Set 10
		{21.81f, 0.0f, 0.015f, 25},

		// Protection
		{0.0f, 0.0f, 0.0f, 0},
		{0.0f, 0.0f, 0.0f, 0},
		{0.0f, 0.0f, 0.0f, 0},
		{0.0f, 0.0f, 0.0f, 0},
		{0.0f, 0.0f, 0.0f, 0},
};

constexpr ServoConstant PRE_TURN_FROM_STRAIGHT_CONSTANTS[] =
{
		{0.0f, 0.0f, 0.0f, 0},

		// Down
		{5.81f, 0.0f, 0.037f, 25},
		{5.81f, 0.0f, 0.037f, 25},
		{5.81f, 0.0f, 0.037f, 25},
		{5.81f, 0.0f, 0.037f, 25},
		{5.81f, 0.0f, 0.037f, 25},

		// Set 9
		{10.57f, 0.0f, 0.05f, 35},
		{13.57f, 0.0f, 0.07f, 35},
		{15.57f, 0.0f, 0.09f, 35},
		{17.57f, 0.0f, 0.11f, 35},
		{19.57f, 0.0f, 0.13f, 35},

		// Set 10
		{4.81f, 0.0f, 0.037f, 25},

		// Protection
		{0.0f, 0.0f, 0.0f, 0},
		{0.0f, 0.0f, 0.0f, 0},
		{0.0f, 0.0f, 0.0f, 0},
		{0.0f, 0.0f, 0.0f, 0},
		{0.0f, 0.0f, 0.0f, 0},
};

constexpr ServoConstant PRE_TURN_FROM_TURN_CONSTANTS[] =
{
		{0.0f, 0.0f, 0.0f, 0},

		// Down
		{2.91f, 0.0f, 0.029f, 25},
		{2.91f, 0.0f, 0.029f, 25},
		{2.91f, 0.0f, 0.029f, 25},
		{2.91f, 0.0f, 0.029f, 25},
		{2.91f, 0.0f, 0.029f, 25},

		// Up
		{0.0f, 0.0f, 0.0f, 0},
		{0.0f, 0.0f, 0.0f, 0},
		{0.0f, 0.0f, 0.0f, 0},
		{0.0f, 0.0f, 0.0f, 0},
		{0.0f, 0.0f, 0.0f, 0},

		// Protection
		{0.0f, 0.0f, 0.0f, 0},
		{0.0f, 0.0f, 0.0f, 0},
		{0.0f, 0.0f, 0.0f, 0},
		{0.0f, 0.0f, 0.0f, 0},
		{0.0f, 0.0f, 0.0f, 0},
};

}

DirControlAlgorithm::DirControlAlgorithm(Car *car)
		: m_car(car),
		  m_flat_gyro_angle(0),

		  m_track_analyzer(CCD_MID_POS, CONSTANTS[0].edge),
		  //m_mid_filter(0.0000005f, 5.0f, 64, 1),
		  m_mid_filter(KALMAN_FILTER_Q, KALMAN_FILTER_R, 64, 1),
		  m_filtered_mid(0),

		  m_servo_pid(CCD_MID_POS, CONSTANTS[0].kp, CONSTANTS[0].ki,
				  CONSTANTS[0].kd),

		  m_case(0),
		  m_turning(0),
		  m_prev_turning(0),
		  m_prev_turn_hint(TurnHint::STRAIGHT),

		  m_mode(0),
		  m_is_explicit_set_turn_hint(false)
{}

void DirControlAlgorithm::OnFinishWarmUp(Car *car)
{
	//m_gyro_filter = libutil::KalmanFilter(0.005f, 0.05f, car->GetGyroAngle(),
	//		1.0f);
	m_flat_gyro_angle = static_cast<int16_t>(car->GetGyroAngle());
	m_servo_pid.Reset();
}

int16_t DirControlAlgorithm::Process(const bitset<LinearCcd::kSensorW> &ccd_data)
{
/*
	if (DetectSlope())
	{
#ifdef DEBUG_PRINT_SLOPE
		LOG_D("Slope (Angle: %f, Flat: %d)", m_car->GetGyroAngle(),
				m_flat_gyro_angle);
#endif
		return 0;
	}
*/

	m_case = 0;
	m_turning = INT16_MIN;
	m_track_analyzer.Analyze(ccd_data);

	if (m_track_analyzer.IsAllBlack() || m_track_analyzer.IsAllWhite())
	{
		ProcessFill();
	}
	else
	{
		ProcessGeneral();
	}

	//LOG_D("current_mid_error_pos: %d", current_mid_error_pos);

#ifdef DEBUG_PRINT_SERVO_PID
	m_servo_pid.Print("servo");
#endif

	if (!m_is_explicit_set_turn_hint)
	{
		const int error = abs(CCD_MID_POS - m_track_analyzer.GetCurrMid());
		/*
		if (error > 16)
		{
			//SetTurnHint(TurnHint::TURN);
			m_turning = (CCD_MID_POS - m_raw_mid > 0) ? -100 : 100;
		}
		else if (error > 11)
		{
			SetTurnHint(TurnHint::PRE_TURN);
			//m_turning = (CCD_MID_POS - m_raw_mid > 0) ? -100 : 100;
		}
		else
		{
			SetTurnHint(TurnHint::STRAIGHT);
		}
		*/

		if (error > 18)
		{
			m_servo_pid.SetSetpoint(CCD_MID_POS
					+ ((CCD_MID_POS > m_track_analyzer.GetCurrMid()) ? 5 : -5));
			SetTurnHint(TurnHint::TURN);
		}
		else if (error > 11)
		{
			m_servo_pid.SetSetpoint(CCD_MID_POS
					+ ((CCD_MID_POS > m_track_analyzer.GetCurrMid()) ? 1 : -1));
			SetTurnHint(TurnHint::PRE_TURN);
		}
		else
		{
			if (m_servo_pid.GetSetpoint() != CCD_MID_POS)
			{
				m_mid_filter = libutil::KalmanFilter(KALMAN_FILTER_Q,
						KALMAN_FILTER_R, m_track_analyzer.GetPrevMid(), 1);
#ifdef DEBUG_BEEP_STRAIGHT_IN
				BeepManager::GetInstance(m_car)->Beep(100);
#endif
			}
			SetTurnHint(TurnHint::STRAIGHT);
		}
	}
	m_is_explicit_set_turn_hint = false;

	m_filtered_mid = m_track_analyzer.GetPrevMid();
	if (m_turning == INT16_MIN)
	{
		const int error = abs(CCD_MID_POS - m_track_analyzer.GetCurrMid());
		if (error <= 18)
		{
			m_filtered_mid = m_mid_filter.Filter(m_track_analyzer.GetCurrMid());
		}
		// Opposite direction
		m_turning = -m_servo_pid.Calc(m_filtered_mid);
	}

	/*
	if (current_mid_error_pos < 0)
	{
		__BREAKPOINT();
	}
	*/
	m_prev_turning = m_turning;
	return m_turning;
}

void DirControlAlgorithm::ProcessFill()
{
	/* ---------------------------------------- (no middle noise) Cross road*/
	if (m_track_analyzer.IsAllWhite())
	{
		m_case = 50;
	}

	/* |||||||||||||||||||||||||||||||||||||||| (all black) */
	else // m_track_analyzer.IsAllBlack()
	{
		m_case = 60;
		if (m_car->GetTurning() >= Config::GetTurnThreshold() + 5)
		{
			m_turning = 100;
		}
		else if (m_car->GetTurning() <= -Config::GetTurnThreshold() - 5)
		{
			m_turning = -100;
		}
	}
}

void DirControlAlgorithm::ProcessGeneral()
{
	/* ||||--------------------------------|||| */
	if (m_track_analyzer.GetLeftEdge() != -1
			&& m_track_analyzer.GetRightEdge() != -1)
	{
		m_case = 10;
	}

	/* ||||--------------------------------||||
	   |||||||||||||||------------------------
	   |||||||||||||||||||||||--------------- */
	else if (m_track_analyzer.GetRightEdge() == -1)
	{
		if (m_track_analyzer.GetLeftEdge()
				<= CCD_MID_POS - CONSTANTS[m_mode].edge)
		{
			// Possibly crossroad
			m_case = 20;
			m_turning = 0;
		}
		else
		{
			m_case = 21;
		}
	}

	/* ||||-------------------------------||||
	   --------------------------|||||||||||||
	   -----------------|||||||||||||||||||||| */
	else  // m_track_analyzer.GetLeftEdge() == -1
	{
		if (m_track_analyzer.GetRightEdge()
				>= CCD_MID_POS + CONSTANTS[m_mode].edge)
		{
			// Possibly crossroad
			m_case = 30;
			m_turning = 0;
		}
		else
		{
			m_case = 31;
		}
	}
}

bool DirControlAlgorithm::DetectSlope()
{
	// XXX
	return false;

	/*
	m_car->UpdateGyro();
	const float filter = m_gyro_filter.Filter(m_car->GetGyroAngle());
#ifdef DEBUG_PRINT_GYRO
	printf("%f\n", filter);
#endif
	return (abs(filter - m_flat_gyro_angle) >= 2500);
	*/
}

void DirControlAlgorithm::SetMode(const Uint mode)
{
	m_mode = mode;
	m_servo_pid.SetKp(CONSTANTS[m_mode].kp);
	m_servo_pid.SetKi(CONSTANTS[m_mode].ki);
	m_servo_pid.SetKd(CONSTANTS[m_mode].kd);
	m_servo_pid.Reset();
}

void DirControlAlgorithm::SetTurnHint(const TurnHint hint)
{
	switch (hint)
	{
	case TurnHint::STRAIGHT:
		m_servo_pid.SetSetpoint(CCD_MID_POS);
		m_servo_pid.SetKp(CONSTANTS[m_mode].kp);
		m_servo_pid.SetKi(CONSTANTS[m_mode].ki);
		m_servo_pid.SetKd(CONSTANTS[m_mode].kd);
		break;

	case TurnHint::PRE_TURN:
		/*
#ifdef DEBUG_BEEP_PRE_TURN
		BeepManager::GetInstance(m_car)->Beep(100);
#endif
		if (m_prev_turn_hint == TurnHint::STRAIGHT)
		{
			m_servo_pid.SetKp(PRE_TURN_FROM_STRAIGHT_CONSTANTS[m_mode].kp);
			m_servo_pid.SetKi(PRE_TURN_FROM_STRAIGHT_CONSTANTS[m_mode].ki);
			m_servo_pid.SetKd(PRE_TURN_FROM_STRAIGHT_CONSTANTS[m_mode].kd);
		}
		else if (m_prev_turn_hint == TurnHint::TURN)
		{
			m_servo_pid.SetKp(PRE_TURN_FROM_TURN_CONSTANTS[m_mode].kp);
			m_servo_pid.SetKi(PRE_TURN_FROM_TURN_CONSTANTS[m_mode].ki);
			m_servo_pid.SetKd(PRE_TURN_FROM_TURN_CONSTANTS[m_mode].kd);
		}
		*/
		// else not necessary
		m_servo_pid.SetKp(PRE_TURN_FROM_TURN_CONSTANTS[m_mode].kp);
		m_servo_pid.SetKi(PRE_TURN_FROM_TURN_CONSTANTS[m_mode].ki);
		m_servo_pid.SetKd(PRE_TURN_FROM_TURN_CONSTANTS[m_mode].kd);
		break;

	case TurnHint::TURN:
		m_servo_pid.SetKp(TURN_CONSTANTS[m_mode].kp);
		m_servo_pid.SetKi(TURN_CONSTANTS[m_mode].ki);
		m_servo_pid.SetKd(TURN_CONSTANTS[m_mode].kd);
		break;
	}
	m_prev_turn_hint = hint;
	m_is_explicit_set_turn_hint = true;
}

void DirControlAlgorithm::ResetMid()
{
	m_track_analyzer.Reset();
	m_filtered_mid = CCD_MID_POS;
}

}
