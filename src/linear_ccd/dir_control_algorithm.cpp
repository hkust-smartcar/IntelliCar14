/*
 * dir_control_algorithm.cpp
 * Algorithm for direction control
 *
 * Author: Louis Mo
 * Copyright (c) 2014 HKUST SmartCar Team
 */

#include <cstdint>
#include <cstdio>
#include <cmath>
#include <algorithm>

#include <libutil/pid_controller.h>
#include <libutil/pid_controller.tcc>

#include "linear_ccd/car.h"
#include "linear_ccd/dir_control_algorithm.h"

using namespace std;

#define VALID_PIXEL 244
#define SERVO_KP 1.55f
#define SERVO_KD 1.00f

#define CCD_MID_POS 122
#define START_LENGTH_L 60
#define START_LENGTH_R 60

namespace linear_ccd
{

DirControlAlgorithm::DirControlAlgorithm(Car *car)
		: m_car(car),

		  all_white_smaple_flag(false),
		  all_black_smaple_flag(false),

		  first_straight_line_flag(false),

		  current_mid_error_pos(CCD_MID_POS),
		  last_sample_error_pos(CCD_MID_POS),
		  previous_mid_error_pos(CCD_MID_POS),

		  current_1st_left_edge(VALID_PIXEL),
		  current_1st_right_edge(0),

		  detect_left_flag(false),
		  detect_right_flag(false),

		  m_servo_pid(CCD_MID_POS, SERVO_KP, 0.0f, SERVO_KD)
{}

void DirControlAlgorithm::Control(const bool *ccd_data)
{
	ccd_data += 6;

	CcdScanAllWhiteOrAllBlackSample(ccd_data);

	detect_left_flag = false;
	detect_right_flag = false;
	current_1st_left_edge = VALID_PIXEL;
	current_1st_right_edge = 0;

	for (int i = last_sample_error_pos; i > 0; --i)
	{ // scan from last_sample_error_pos to left edge
		if (ccd_data[i])
		{
			current_1st_left_edge = i;
			detect_left_flag = true;
			break;
		}
	}

	for (int i = last_sample_error_pos; i < VALID_PIXEL; ++i)
	{  // scan from last_sample_error_pos to right edge
		if (ccd_data[i])
		{
			current_1st_right_edge = i;
			detect_right_flag = true;
			break;
		}
	}

	int if_case = 0;
	/* ||||--------------------------------|||| */
	if (detect_left_flag && detect_right_flag)
	{
		if_case = 1;
		current_mid_error_pos =
				(current_1st_left_edge + current_1st_right_edge) / 2;

		/*if(!first_straight_line_flag){
			left_start_length = current_mid_error_pos - current_1st_left_edge;
			right_start_length =  current_1st_right_edge - current_mid_error_pos;
			first_straight_line_flag = true;
		}

		LOG_D("left_start_length: %d\n", left_start_length);
		LOG_D("right_start_length: %d\n", right_start_length);
		*/

	}

	/* ||||--------------------------------||||
	   |||||||||||||||------------------------
	   |||||||||||||||||||||||--------------- */
	else if (detect_left_flag && !detect_right_flag)
	{
		if_case = 2;
		current_mid_error_pos = current_1st_left_edge + START_LENGTH_R;

		if (current_1st_left_edge == (VALID_PIXEL - 1))
		{
			current_mid_error_pos = CCD_MID_POS;
		}
	}

	/* ||||-------------------------------||||
	   --------------------------|||||||||||||
	   -----------------|||||||||||||||||||||| */
	else if (!detect_left_flag && detect_right_flag)
	{
		if_case = 3;
		current_mid_error_pos = current_1st_right_edge - START_LENGTH_L;

		if (current_1st_right_edge == 0)
		{
			current_mid_error_pos = CCD_MID_POS;
		}
	}

	else if (!detect_left_flag && !detect_right_flag)
	{
		if_case = 4;
		current_mid_error_pos = CCD_MID_POS;
	}

	/* ---------------------------------------- (no middle noise) Cross road*/
	if (all_white_smaple_flag)
	{
		if_case = 5;
		current_mid_error_pos = CCD_MID_POS;
	}

	/* |||||||||||||||||||||||||||||||||||||||| (all black) */
	if(all_black_smaple_flag)
	{
		if_case = 6;
		//current_mid_error_pos = ccd_mid_pos + current_dir_error;
	}

	LOG_D("if_case :%d\n", if_case);

	m_servo_pid.Print("servo");
	m_car->SetTurning(m_servo_pid.Calc(current_mid_error_pos));

	/*
	if (current_mid_error_pos < 0)
	{
		__BREAKPOINT();
	}
	*/
	last_sample_error_pos = current_mid_error_pos;
	//current_edge_middle_distance = current_1st_right_edge - current_1st_left_edge;
}

void DirControlAlgorithm::CcdScanAllWhiteOrAllBlackSample(const bool *ccd_data)
{
	int white_counter = 0;
	int black_counter = 0;
	all_white_smaple_flag = false;
	all_black_smaple_flag = false;

	for (int i = 0; i < VALID_PIXEL; ++i)
	{
		if (!ccd_data[i])
		{
			++white_counter;
		}
		else
		{
			++black_counter;
		}
	}

	all_white_smaple_flag = (white_counter == VALID_PIXEL);
	all_black_smaple_flag = (black_counter == VALID_PIXEL);
}

}
