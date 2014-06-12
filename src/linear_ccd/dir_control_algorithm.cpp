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

#include "linear_ccd/config.h"
#include "linear_ccd/car.h"
#include "linear_ccd/dir_control_algorithm.h"

using namespace std;

#define VALID_PIXEL (128 - 6)
#define VALID_OFFSET 3

#define CCD_MID_POS (VALID_PIXEL / 2)

namespace linear_ccd
{

namespace
{

struct ServoConstant
{
	float kp;
	float ki;
	float kd;
	int start_l;
	int start_r;
};

// The first one is being used when the motor is stopped (manual mode)
constexpr ServoConstant CONSTANTS[] =
{
		//{0.0f, 0.0f, 0.0f, 0, 0},
		{2.17f, 0.0f, 1.59f, 59, 59},
		//
		//{1.57f, 0.0f, 0.0f, 62, 62},
		//{1.62f, 0.0f, 0.0f, 62, 62},
		//{1.67f, 0.0f, 0.0f, 62, 62},

		{1.67f, 0.0f, 1.49f, 59, 59},
		{1.67f, 0.0f, 1.49f, 59, 59},
		{1.67f, 0.0f, 1.49f, 59, 59},
		{1.67f, 0.0f, 1.49f, 59, 59},
		{1.67f, 0.0f, 1.49f, 59, 59},

		{1.07f, 0.0f, 0.74f, 40, 40},
		{1.07f, 0.0f, 0.74f, 40, 40},
		{1.07f, 0.0f, 0.74f, 40, 40},
		{1.07f, 0.0f, 0.74f, 40, 40},
		{1.07f, 0.0f, 0.74f, 40, 40},

		//{1.275f, 0.0f, 0.88f, 45, 45},
		{1.55f, 0.0f, 0.5f, 60, 60},
		{3.05f, 0.0f, 1.0f, 60, 60},
		{0.9f, 0.0f, 1.0f, 60, 60},
		//{1.55f, 0.0f, 0.0f, 60, 60},

		// Successful trial
		// Set 1
		//PWM[0, 7k]
		//turn threshold 35
		//{1.08f, 0.0f, 0.98f, 65, 65}, // Average
		//{1.00f, 0.0f, 0.90f, 65, 65}, // Good
		//{0.98f, 0.0f, 0.88f, 65, 65}, // Good
		//{0.80f, 0.0f, 0.78f, 65, 65}, // Worst

		// Set 2
		//PWM[0, 7k]
		//turn threshold 35
		//{1.2f, 0.0f, 0.0f, 62, 62}, // Good
		//{1.44f, 0.0f, 0.0f, 62, 62}, // Acceptable

		// Set 3
		//PWM[-4k, 7k]
		//turn threshold 35
		//{1.57f, 0.0f, 1.02f, 62, 62}, // V. Good

		// Set 4
		//PWM[-4k, 7k]
		//turn threshold 40
		//{1.57f, 0.0f, 1.02f, 62, 62},

		// Protection
		{0.0f, 0.0f, 0.0f, 0, 0},
		{0.0f, 0.0f, 0.0f, 0, 0},
		{0.0f, 0.0f, 0.0f, 0, 0},
		{0.0f, 0.0f, 0.0f, 0, 0},
		{0.0f, 0.0f, 0.0f, 0, 0},
};

constexpr ServoConstant TURN_CONSTANTS[] =
{
		{2.8f, 0.0f, 1.3f, 58, 58},

		{3.25f, 0.0f, 1.34f, 59, 59},
		{3.25f, 0.0f, 1.34f, 59, 59},
		{3.25f, 0.0f, 1.34f, 59, 59},
		{3.25f, 0.0f, 1.34f, 59, 59},
		{3.25f, 0.0f, 1.34f, 59, 59},

		{0.0f, 0.0f, 0.0f, 0, 0},
		{0.0f, 0.0f, 0.0f, 0, 0},
		{0.0f, 0.0f, 0.0f, 0, 0},
		{0.0f, 0.0f, 0.0f, 0, 0},
		{0.0f, 0.0f, 0.0f, 0, 0},

		// Protection
		{0.0f, 0.0f, 0.0f, 0, 0},
		{0.0f, 0.0f, 0.0f, 0, 0},
		{0.0f, 0.0f, 0.0f, 0, 0},
		{0.0f, 0.0f, 0.0f, 0, 0},
		{0.0f, 0.0f, 0.0f, 0, 0},
};

}

DirControlAlgorithm::DirControlAlgorithm(Car *car)
		: m_car(car),
		  m_flat_gyro_angle(0),

		  all_white_smaple_flag(false),
		  all_black_smaple_flag(false),

		  m_prev_mid(CCD_MID_POS),

		  m_servo_pid(CCD_MID_POS, CONSTANTS[0].kp, CONSTANTS[0].ki,
				  CONSTANTS[0].kd),
		  m_gyro_filter(0, 0, 0, 0), // will be init later

		  m_mode(0)
{}

void DirControlAlgorithm::OnFinishWarmUp(Car *car)
{
	m_gyro_filter = libutil::KalmanFilter(0.005f, 0.05f, car->GetGyroAngle(),
			1.0f);
	m_flat_gyro_angle = static_cast<int16_t>(car->GetGyroAngle());
	m_servo_pid.Restart();
}

int16_t DirControlAlgorithm::Process(
		const bitset<libsc::LinearCcd::SENSOR_W> &ccd_data)
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
	if (all_black_smaple_flag || all_white_smaple_flag)
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

	if (m_turning == INT16_MIN)
	{
		if (m_turning <= Config::GetTurnThreshold())
		{
			UpdatePid(true);
		}
		else
		{
			UpdatePid(false);
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

	return m_turning;
}

void DirControlAlgorithm::ProcessFill()
{
	/* ---------------------------------------- (no middle noise) Cross road*/
	if (all_white_smaple_flag)
	{
		m_case = 50;
		m_curr_mid = CCD_MID_POS;
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
		const bitset<libsc::LinearCcd::SENSOR_W> &ccd_data)
{
	bool detect_left_flag = false;
	int current_1st_left_edge = VALID_PIXEL;
	for (int i = libutil::ClampVal<int>(0, m_prev_mid, libsc::LinearCcd::SENSOR_W);
			i >= VALID_OFFSET; --i)
	{ // scan from last_sample_error_pos to left edge
		if (!ccd_data[i + VALID_OFFSET])
		{
			current_1st_left_edge = i + VALID_OFFSET;
			detect_left_flag = true;
			break;
		}
	}

	bool detect_right_flag = false;
	int current_1st_right_edge = 0;
	for (int i = libutil::ClampVal<int>(0, m_prev_mid, libsc::LinearCcd::SENSOR_W);
			i < VALID_PIXEL; ++i)
	{  // scan from last_sample_error_pos to right edge
		if (!ccd_data[i + VALID_OFFSET])
		{
			current_1st_right_edge = i + VALID_OFFSET;
			detect_right_flag = true;
			break;
		}
	}

	/* ||||--------------------------------|||| */
	if (detect_left_flag && detect_right_flag)
	{
		m_case = 10;
		m_curr_mid = (current_1st_left_edge + current_1st_right_edge) / 2;

#ifdef DEBUG_PRINT_EDGE
		LOG_D("Edge: %d | %d", current_mid_error_pos - current_1st_left_edge,
				current_1st_right_edge - current_mid_error_pos);
#endif
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
		if (current_1st_left_edge < CONSTANTS[m_mode].start_l / 2)
		{
			// Possibly crossroad
			m_case = 20;
			//m_curr_mid = CCD_MID_POS;
			m_curr_mid = current_1st_left_edge + CONSTANTS[m_mode].start_r;
			m_turning = 0;
		}
		else
		{
			m_case = 21;
			m_curr_mid = current_1st_left_edge + CONSTANTS[m_mode].start_r;
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
		if (current_1st_right_edge < CONSTANTS[m_mode].start_r / 2)
		{
			// Possibly crossroad
			m_case = 30;
			//m_curr_mid = CCD_MID_POS;
			m_curr_mid = current_1st_right_edge - CONSTANTS[m_mode].start_l;
			m_turning = 0;
		}
		else
		{
			m_case = 31;
			m_curr_mid = current_1st_right_edge - CONSTANTS[m_mode].start_l;
			//LOG_D("current_1st_right_edge: %d", current_1st_right_edge);
		}
	}

	else if (!detect_left_flag && !detect_right_flag)
	{
		m_case = 40;
		m_curr_mid = CCD_MID_POS;
	}
}

bool DirControlAlgorithm::DetectSlope()
{
	// XXX
	return false;

	m_car->UpdateGyro();
	const float filter = m_gyro_filter.Filter(m_car->GetGyroAngle());
#ifdef DEBUG_PRINT_GYRO
	printf("%f\n", filter);
#endif
	return (abs(filter - m_flat_gyro_angle) >= 2500);
}

void DirControlAlgorithm::ScanAllWhiteOrAllBlackSample(
		const bitset<libsc::LinearCcd::SENSOR_W> &ccd_data)
{
	all_white_smaple_flag = true;
	all_black_smaple_flag = true;

	for (int i = 0; i < VALID_PIXEL
			&& (all_black_smaple_flag || all_white_smaple_flag); ++i)
	{
		if (ccd_data[i + VALID_OFFSET])
		{
			all_black_smaple_flag = false;
		}
		else
		{
			all_white_smaple_flag = false;
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

void DirControlAlgorithm::UpdatePid(const bool is_straight)
{
	if (is_straight)
	{
		m_servo_pid.SetKp(CONSTANTS[m_mode].kp);
		m_servo_pid.SetKi(CONSTANTS[m_mode].ki);
		m_servo_pid.SetKd(CONSTANTS[m_mode].kd);
	}
	else
	{
		m_servo_pid.SetKp(TURN_CONSTANTS[m_mode].kp);
		m_servo_pid.SetKi(TURN_CONSTANTS[m_mode].ki);
		m_servo_pid.SetKd(TURN_CONSTANTS[m_mode].kd);
	}
}

}
