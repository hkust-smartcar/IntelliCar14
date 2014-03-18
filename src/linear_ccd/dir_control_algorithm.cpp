/*
 * dir_control_algorithm.cpp
 * Algorithm for direction control
 *
 * Author: Louis Mo
 * Copyright (c) 2014 HKUST SmartCar Team
 */

#include <cstdint>

#include "linear_ccd/dir_control_algorithm.h"

#define VALID_PIXEL 244

namespace linear_ccd
{

DirControlAlgorithm::DirControlAlgorithm(Car *car)
		: m_car(car),
		  left_start_length(25),
		  right_start_length(25),

		  ccd_mid_pos(121),

		  all_white_smaple_flag(0),
		  all_black_smaple_flag(0),

		  current_mid_error_pos(121),
		  last_sample_error_pos(121),
		  previous_mid_error_pos(121),

		  current_dir_error(0),
		  current_dir_arc_value_error(0),

		  current_1st_left_edge(243),
		  current_1st_right_edge(0),

		  current_edge_middle_distance(0),

		  detect_left_flag(0),
		  detect_right_flag(0)
{}

void DirControlAlgorithm::Control(const bool *ccd_data)
{
	detect_left_flag = 0;
	detect_right_flag = 0;
	current_1st_left_edge=243;
	current_1st_right_edge=0;

	for (int i = last_sample_error_pos; i > 0; i--)
	{ // scan from last_sample_error_pos to left edge
		if (ccd_data[i] == false)
		{
			current_1st_left_edge = i;
			detect_left_flag = 1;
			i = 1;
		}
	}

	for (int i = last_sample_error_pos; i < VALID_PIXEL; i++)
	{  // scan from last_sample_error_pos to right edge
		if (ccd_data[i] == false)
		{
			current_1st_right_edge = i;
			detect_right_flag = 1;
			i = 243;
		}
	}

	/* ||||--------------------------------|||| */
	if (detect_left_flag == 1 && detect_right_flag == 1)
	{
		current_mid_error_pos = (current_1st_left_edge + current_1st_right_edge) / 2;
		//left_start_length = current_mid_error_pos - current_1st_left_edge;
		//right_start_length = current_mid_error_pos + current_1st_right_edge;
	}

	/* ||||--------------------------------||||
	   |||||||||||||||------------------------
	   |||||||||||||||||||||||--------------- */
	else if (detect_left_flag == 1 && detect_right_flag == 0)
	{
		current_mid_error_pos = current_1st_left_edge + right_start_length;

		if (current_1st_left_edge == (VALID_PIXEL - 1))
		{
			current_mid_error_pos = ccd_mid_pos;
		}
	}

	/* ||||-------------------------------||||
	   --------------------------|||||||||||||
	   -----------------|||||||||||||||||||||| */
	else if (detect_left_flag == 0 && detect_right_flag == 1)
	{
		current_mid_error_pos = current_1st_right_edge - left_start_length;

		if (current_1st_right_edge == 0){
			current_mid_error_pos = ccd_mid_pos;
		}
	}

	/* ---------------------------------------- (no middle noise) Cross road*/
	if (all_white_smaple_flag == 1)
	{
		//current_mid_error_pos = ccd_mid_pos+(encoder_turn_error*35/100); // John added
		current_mid_error_pos = ccd_mid_pos;
	}

	/* |||||||||||||||||||||||||||||||||||||||| (all black) */
	if(all_black_smaple_flag == 1)
	{
		current_mid_error_pos = ccd_mid_pos;
	}

	current_dir_error = (current_mid_error_pos - ccd_mid_pos);
	//current_dir_arc_value_error = atan(current_dir_error*(atan_multiply_value))*1000;

	//previous_mid_error_pos = current_mid_error_pos;
	last_sample_error_pos = current_mid_error_pos;
	current_edge_middle_distance = current_1st_right_edge - current_1st_left_edge;
}

void DirControlAlgorithm::CcdScanAllWhiteOrAllBlackSample(const bool *ccd_data)
{
	int white_counter=0;
	int black_counter=0;
	all_white_smaple_flag= 0;
	all_black_smaple_flag= 0;

	for(int i = 0; i < VALID_PIXEL; i++)
	{
		if (ccd_data[i] == true)
		{
			white_counter++;
		}
		else if (ccd_data[i] == false)
		{
			black_counter++;
		}
	}

	if (white_counter == VALID_PIXEL)
	{
		all_white_smaple_flag = 1;
	}
	else if (black_counter == VALID_PIXEL)
	{
		all_black_smaple_flag = 1;
	}
}

}
