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
#include <libutil/pid_controller.h>
#include <libutil/pid_controller.tcc>
#include <libutil/string.h>

#include "linear_ccd/beep_manager.h"
#include "linear_ccd/config.h"
#include "linear_ccd/car.h"
#include "linear_ccd/dir_control_algorithm.h"

using namespace std;
using namespace libsc::k60;

//#define VALID_PIXEL (128 - 6)
//#define VALID_OFFSET 3
#define VALID_PIXEL Config::GetCcdValidPixel()
#define VALID_OFFSET Config::GetCcdValidPixelOffset()

#define CCD_MID_POS 64

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
		//{0.0f, 0.0f, 0.0f, 0},
		{0.87f, 0.0f, 0.8f, 35},
		//
		//{1.57f, 0.0f, 0.0f, 62},
		//{1.62f, 0.0f, 0.0f, 62},
		//{1.67f, 0.0f, 0.0f, 62},

		// Down
		{0.87f, 0.0f, 0.3f, 35},
		{0.87f, 0.0f, 0.3f, 35},
		{0.87f, 0.0f, 0.3f, 35},
		{0.87f, 0.0f, 0.3f, 35},
		{0.87f, 0.0f, 0.3f, 35},

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

		// Protection
		{0.0f, 0.0f, 0.0f, 0},
		{0.0f, 0.0f, 0.0f, 0},
		{0.0f, 0.0f, 0.0f, 0},
		{0.0f, 0.0f, 0.0f, 0},
		{0.0f, 0.0f, 0.0f, 0},
};

constexpr ServoConstant TURN_CONSTANTS[] =
{
		{17.81f, 0.0f, 1.15f, 35},

		{13.81f, 0.0f, 1.15f, 35},
		{15.81f, 0.0f, 1.15f, 35},
		{17.81f, 0.0f, 1.15f, 35},
		{19.81f, 0.0f, 1.15f, 35},
		{21.81f, 0.0f, 1.15f, 35},

		{0.0f, 0.0f, 0.0f, 0},
		{0.0f, 0.0f, 0.0f, 0},
		{0.0f, 0.0f, 0.0f, 0},
		{0.0f, 0.0f, 0.0f, 0},
		{0.0f, 0.0f, 0.0f, 0},

		// Set 5
		//Before fixing tire
		{3.25f, 0.0f, 1.34f, 35},

		// Protection
		{0.0f, 0.0f, 0.0f, 0},
		{0.0f, 0.0f, 0.0f, 0},
		{0.0f, 0.0f, 0.0f, 0},
		{0.0f, 0.0f, 0.0f, 0},
		{0.0f, 0.0f, 0.0f, 0},
};

constexpr ServoConstant PRE_TURN_CONSTANTS[] =
{
		{8.57f, 0.0f, 0.05f, 35},

		// Down
		{8.57f, 0.0f, 0.05f, 35},
		{8.57f, 0.0f, 0.07f, 35},
		{8.57f, 0.0f, 0.09f, 35},
		{8.57f, 0.0f, 0.11f, 35},
		{8.57f, 0.0f, 0.13f, 35},

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

		  m_is_all_black(false),
		  m_is_all_white(false),

		  m_curr_left_edge(Config::GetCcdValidPixelOffset()),
		  m_curr_right_edge(Config::GetCcdValidPixelOffset()
				  + Config::GetCcdValidPixel()),

		  m_prev_mid(CCD_MID_POS),
		  m_curr_mid(CCD_MID_POS),

		  m_servo_pid(CCD_MID_POS, CONSTANTS[0].kp, CONSTANTS[0].ki,
				  CONSTANTS[0].kd),
		  //m_mid_filter(0.0000005f, 0.000001f, 64, 1),
		  m_mid_filter(0.0000005f, 5.0f, 64, 1),
		  //m_gyro_filter(0, 0, 0, 0), // will be init later

		  m_case(0),
		  m_turning(0),
		  m_prev_turning(0),

		  m_mode(0),
		  m_is_explicit_set_turn_hint(false)
{}

void DirControlAlgorithm::OnFinishWarmUp(Car *car)
{
	//m_gyro_filter = libutil::KalmanFilter(0.005f, 0.05f, car->GetGyroAngle(),
	//		1.0f);
	m_flat_gyro_angle = static_cast<int16_t>(car->GetGyroAngle());
	m_servo_pid.Restart();
}

int16_t DirControlAlgorithm::Process(const bitset<LinearCcd::SENSOR_W> &ccd_data)
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
	m_curr_mid = 0;
	m_turning = INT16_MIN;

	ScanAllWhiteOrAllBlackSample(ccd_data);
	if (m_is_all_black || m_is_all_white)
	{
		ProcessFill();
	}
	else
	{
		ProcessGeneral(ccd_data);
	}

	//LOG_D("current_mid_error_pos: %d", current_mid_error_pos);

#ifdef DEBUG_PRINT_SERVO_PID
	m_servo_pid.Print("servo");
#endif

	if (!m_is_explicit_set_turn_hint)
	{
		const int error = abs(CCD_MID_POS - m_curr_mid);
		/*
		if (error > 16)
		{
			//SetTurnHint(TurnHint::TURN);
			m_turning = (CCD_MID_POS - m_curr_mid > 0) ? -100 : 100;
		}
		else if (error > 11)
		{
			SetTurnHint(TurnHint::PRE_TURN);
			//m_turning = (CCD_MID_POS - m_curr_mid > 0) ? -100 : 100;
		}
		else
		{
			SetTurnHint(TurnHint::STRAIGHT);
		}
		*/
		if (error > 14)
		{
			m_servo_pid.SetSetpoint(64 + ((CCD_MID_POS > m_curr_mid) ? -5 : 5));
			SetTurnHint(TurnHint::TURN);
		}
		else
		{
			if (m_servo_pid.GetSetpoint() != CCD_MID_POS)
			{
				m_mid_filter = libutil::KalmanFilter(0.0000005f, 5.0f,
						m_prev_mid, 1);
#ifdef DEBUG_BEEP_STRAIGHT_IN
				BeepManager::GetInstance(m_car)->Beep(100);
#endif
			}
			SetTurnHint(TurnHint::STRAIGHT);
		}
	}
	m_is_explicit_set_turn_hint = false;

	if (m_turning == INT16_MIN)
	{
		const int error = abs(CCD_MID_POS - m_curr_mid);
		if (error < 14)
		{
			m_curr_mid = m_mid_filter.Filter(m_curr_mid);
		}
		// Opposite direction
		m_turning = -m_servo_pid.Calc(m_curr_mid);
	}

	/*
	if (current_mid_error_pos < 0)
	{
		__BREAKPOINT();
	}
	*/
	m_prev_mid = m_curr_mid;
	//current_edge_middle_distance = current_1st_right_edge - current_1st_left_edge;
	m_prev_turning = m_turning;

	return m_turning;
}

void DirControlAlgorithm::ProcessFill()
{
	m_curr_left_edge = -1;
	m_curr_right_edge = -1;
	/* ---------------------------------------- (no middle noise) Cross road*/
	if (m_is_all_white)
	{
		m_case = 50;
		//m_curr_mid = CCD_MID_POS;
		m_curr_mid = m_prev_mid;
	}

	/* |||||||||||||||||||||||||||||||||||||||| (all black) */
	else // if (all_black_smaple_flag)
	{
		m_case = 60;
		//current_mid_error_pos = ccd_mid_pos + current_dir_error;
		m_curr_mid = m_prev_mid;
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

void DirControlAlgorithm::ProcessGeneral(
		const bitset<LinearCcd::SENSOR_W> &ccd_data)
{
	bool detect_left_flag = false;
	m_curr_left_edge = -1;
	for (int i = libutil::ClampVal<int>(VALID_OFFSET, m_prev_mid,
			VALID_PIXEL + VALID_OFFSET - 1); i >= VALID_OFFSET; --i)
	{
		// scan from last_sample_error_pos to left edge
		if (!ccd_data[i])
		{
			m_curr_left_edge = i;
			detect_left_flag = true;
			break;
		}
	}

	bool detect_right_flag = false;
	m_curr_right_edge = -1;
	for (int i = libutil::ClampVal<int>(VALID_OFFSET, m_prev_mid,
			VALID_PIXEL + VALID_OFFSET - 1); i < VALID_PIXEL + VALID_OFFSET; ++i)
	{
		// scan from last_sample_error_pos to right edge
		if (!ccd_data[i])
		{
			m_curr_right_edge = i;
			detect_right_flag = true;
			break;
		}
	}

	/* ||||--------------------------------|||| */
	if (detect_left_flag && detect_right_flag)
	{
		m_case = 10;
		m_curr_mid = (m_curr_left_edge + m_curr_right_edge) / 2;
	}

	/* ||||--------------------------------||||
	   |||||||||||||||------------------------
	   |||||||||||||||||||||||--------------- */
	else if (detect_left_flag && !detect_right_flag)
	{
		/*
		if (current_1st_left_edge == (VALID_PIXEL - 1))
		{
			if_case = 20;
			current_mid_error_pos = CCD_MID_POS;
		}
		*/
		const int track_w = CONSTANTS[m_mode].edge * 2;
		const int detect_w = VALID_PIXEL - m_curr_left_edge;
		const int imaginery_w = std::max<int>(track_w - detect_w, 0);
		const int imaginery_right_edge = VALID_PIXEL + imaginery_w;
		m_curr_mid = (m_curr_left_edge + imaginery_right_edge) / 2;

		const int side_space = CCD_MID_POS - CONSTANTS[m_mode].edge;
		if (m_curr_left_edge <= side_space)
		{
			// Possibly crossroad
			m_case = 20;
			//m_curr_mid = CCD_MID_POS;
			m_turning = 0;
		}
		else
		{
			m_case = 21;
			//m_curr_mid = current_1st_left_edge + CONSTANTS[m_mode].edge;
		}
	}

	/* ||||-------------------------------||||
	   --------------------------|||||||||||||
	   -----------------|||||||||||||||||||||| */
	else if (!detect_left_flag && detect_right_flag)
	{
		/*
		if (current_1st_right_edge == 0)
		{
			if_case = 30;
			current_mid_error_pos = CCD_MID_POS;
		}
		*/
		const int track_w = CONSTANTS[m_mode].edge * 2;
		const int detect_w = m_curr_right_edge;
		const int imaginery_w = std::max<int>(track_w - detect_w, 0);
		const int imaginery_left_edge = -imaginery_w;
		m_curr_mid = (imaginery_left_edge + m_curr_right_edge) / 2;

		const int side_space = CCD_MID_POS - CONSTANTS[m_mode].edge;
		if (m_curr_right_edge >= CCD_MID_POS + CONSTANTS[m_mode].edge)
		{
			// Possibly crossroad
			m_case = 30;
			//m_curr_mid = CCD_MID_POS;
			//m_curr_mid = current_1st_right_edge - CONSTANTS[m_mode].edge;
			m_turning = 0;
		}
		else
		{
			m_case = 31;
			//m_curr_mid = current_1st_right_edge - CONSTANTS[m_mode].edge;
			//LOG_D("current_1st_right_edge: %d", current_1st_right_edge);
		}
	}

	/*
	else if (!detect_left_flag && !detect_right_flag)
	{
		m_case = 40;
		m_curr_mid = CCD_MID_POS;
	}
	*/
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

void DirControlAlgorithm::ScanAllWhiteOrAllBlackSample(
		const bitset<LinearCcd::SENSOR_W> &ccd_data)
{
	m_is_all_black = true;
	m_is_all_white = true;

	for (int i = VALID_OFFSET; i < VALID_PIXEL - VALID_OFFSET
			&& (m_is_all_black || m_is_all_white); ++i)
	{
		if (ccd_data[i])
		{
			m_is_all_black = false;
		}
		else
		{
			m_is_all_white = false;
		}
	}
}

void DirControlAlgorithm::SetMode(const Uint mode)
{
	m_mode = mode;
	m_servo_pid.SetKp(CONSTANTS[m_mode].kp);
	m_servo_pid.SetKi(CONSTANTS[m_mode].ki);
	m_servo_pid.SetKd(CONSTANTS[m_mode].kd);
	m_servo_pid.Restart();
}

void DirControlAlgorithm::SetTurnHint(const TurnHint hint)
{
	switch (hint)
	{
	case TurnHint::STRAIGHT:
		m_servo_pid.SetSetpoint(64);
		m_servo_pid.SetKp(CONSTANTS[m_mode].kp);
		m_servo_pid.SetKi(CONSTANTS[m_mode].ki);
		m_servo_pid.SetKd(CONSTANTS[m_mode].kd);
		break;

	case TurnHint::PRE_TURN:
#ifdef DEBUG_BEEP_PRE_TURN
		BeepManager::GetInstance(m_car)->Beep(100);
#endif
		m_servo_pid.SetKp(PRE_TURN_CONSTANTS[m_mode].kp);
		m_servo_pid.SetKi(PRE_TURN_CONSTANTS[m_mode].ki);
		m_servo_pid.SetKd(PRE_TURN_CONSTANTS[m_mode].kd);
		break;

	case TurnHint::TURN:
		m_servo_pid.SetKp(TURN_CONSTANTS[m_mode].kp);
		m_servo_pid.SetKi(TURN_CONSTANTS[m_mode].ki);
		m_servo_pid.SetKd(TURN_CONSTANTS[m_mode].kd);
		break;
	}
	m_is_explicit_set_turn_hint = true;
}

void DirControlAlgorithm::ResetMid()
{
	m_curr_mid = m_prev_mid = CCD_MID_POS;
}

}
