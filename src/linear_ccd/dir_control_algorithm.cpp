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

#include "linear_ccd/debug.h"

#include <libutil/kalman_filter.h>
#include <libutil/pid_controller.h>
#include <libutil/pid_controller.tcc>
#include <libutil/misc.h>

#include "linear_ccd/car.h"
#include "linear_ccd/dir_control_algorithm.h"

using namespace std;

#define VALID_PIXEL 244
#define VALID_OFFSET 6

#define CCD_MID_POS 122

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
		{1.55f, 0.0f, 0.0f, 62, 62},
		//
		//{1.57f, 0.0f, 0.0f, 62, 62},
		//{1.62f, 0.0f, 0.0f, 62, 62},
		//{1.67f, 0.0f, 0.0f, 62, 62},

		{1.57f, 0.0f, 0.82f, 62, 62},
		{1.57f, 0.0f, 0.92f, 62, 62},
		{1.57f, 0.0f, 1.02f, 62, 62},
		{1.57f, 0.0f, 1.12f, 62, 62},

		//{1.275f, 0.0f, 0.88f, 45, 45},
		{1.55f, 0.0f, 0.5f, 60, 60},
		{3.05f, 0.0f, 1.0f, 60, 60},
		{0.9f, 0.0f, 1.0f, 60, 60},
		//{1.55f, 0.0f, 0.0f, 60, 60},
		{0.0f, 0.0f, 0.0f, 0, 0},
		{0.0f, 0.0f, 0.0f, 0, 0},
		{0.0f, 0.0f, 0.0f, 0, 0},
		{0.0f, 0.0f, 0.0f, 0, 0},

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
};

}

DirControlAlgorithm::DirControlAlgorithm(Car *car)
		: m_car(car),
		  m_flat_gyro_angle(0),

		  all_white_smaple_flag(false),
		  all_black_smaple_flag(false),

		  last_sample_error_pos(CCD_MID_POS),

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

void DirControlAlgorithm::Control(const bool *ccd_data)
{
	if (DetectSlope())
	{
#ifdef DEBUG_PRINT_SLOPE
		LOG_D("Slope (Angle: %f, Flat: %d)", m_car->GetGyroAngle(),
				m_flat_gyro_angle);
#endif
		m_car->SetTurning(0);
		return;
	}

	ccd_data += VALID_OFFSET;
	ScanAllWhiteOrAllBlackSample(ccd_data);

	bool detect_left_flag = false;
	int current_1st_left_edge = VALID_PIXEL;
	for (int i = libutil::Clamp<int>(0, last_sample_error_pos, VALID_PIXEL - 1);
			i >= 0; --i)
	{ // scan from last_sample_error_pos to left edge
		if (ccd_data[i])
		{
			current_1st_left_edge = i;
			detect_left_flag = true;
			break;
		}
	}

	bool detect_right_flag = false;
	int current_1st_right_edge = 0;
	for (int i = libutil::Clamp<int>(0, last_sample_error_pos, VALID_PIXEL - 1);
			i < VALID_PIXEL; ++i)
	{  // scan from last_sample_error_pos to right edge
		if (ccd_data[i])
		{
			current_1st_right_edge = i;
			detect_right_flag = true;
			break;
		}
	}

	int if_case = 0;
	int current_mid_error_pos = 0;
	/* ||||--------------------------------|||| */
	if (detect_left_flag && detect_right_flag)
	{
		if_case = 10;
		current_mid_error_pos =
				(current_1st_left_edge + current_1st_right_edge) / 2;

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
			if_case = 20;
			current_mid_error_pos = CCD_MID_POS;
		}
		else
		{
			if_case = 21;
			current_mid_error_pos = current_1st_left_edge
					+ CONSTANTS[m_mode].start_r;
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
			if_case = 30;
			current_mid_error_pos = CCD_MID_POS;
		}
		else
		{
			if_case = 31;
			current_mid_error_pos = current_1st_right_edge
					- CONSTANTS[m_mode].start_l;
			//LOG_D("current_1st_right_edge: %d", current_1st_right_edge);
		}
	}

	else if (!detect_left_flag && !detect_right_flag)
	{
		if_case = 40;
		current_mid_error_pos = CCD_MID_POS;
	}

	/* ---------------------------------------- (no middle noise) Cross road*/
	if (all_white_smaple_flag)
	{
		if_case = 50;
		current_mid_error_pos = CCD_MID_POS;
	}

	/* |||||||||||||||||||||||||||||||||||||||| (all black) */
	if (all_black_smaple_flag)
	{
		if_case = 60;
		//current_mid_error_pos = ccd_mid_pos + current_dir_error;
		current_mid_error_pos = last_sample_error_pos;
	}

#ifdef DEBUG_PRINT_CASE
	LOG_D("if_case :%d", if_case);
#endif
	//LOG_D("current_mid_error_pos: %d", current_mid_error_pos);

#ifdef DEBUG_PRINT_SERVO_PID
	m_servo_pid.Print("servo");
#endif

	const int turning = m_servo_pid.Calc(current_mid_error_pos);
#ifdef DEBUG_PRINT_TURNING
	LOG_D("turning :%d", turning);
#endif
	m_car->SetTurning(turning);

	/*
	if (current_mid_error_pos < 0)
	{
		__BREAKPOINT();
	}
	*/
	last_sample_error_pos = current_mid_error_pos;
	//current_edge_middle_distance = current_1st_right_edge - current_1st_left_edge;
}

bool DirControlAlgorithm::DetectSlope()
{
	m_car->UpdateGyro();
	const float filter = m_gyro_filter.Filter(m_car->GetGyroAngle());
#ifdef DEBUG_PRINT_GYRO
	printf("%f\n", filter);
#endif
	return (abs(filter - m_flat_gyro_angle) >= 2500);
}

void DirControlAlgorithm::ScanAllWhiteOrAllBlackSample(const bool *ccd_data)
{
	all_white_smaple_flag = true;
	all_black_smaple_flag = true;

	for (int i = 0; i < VALID_PIXEL
			&& (all_black_smaple_flag || all_white_smaple_flag); ++i)
	{
		if (!ccd_data[i])
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

}
