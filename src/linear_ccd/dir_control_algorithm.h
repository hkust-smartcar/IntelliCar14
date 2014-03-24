/*
 * dir_control_algorithm.h
 * Algorithm for direction control
 *
 * Author: Louis Mo
 * Copyright (c) 2014 HKUST SmartCar Team
 */

#ifndef LINEAR_CCD_DETECTION_ALGORITHM_H_
#define LINEAR_CCD_DETECTION_ALGORITHM_H_

#include <cstdint>

namespace linear_ccd
{

class Car;

}

namespace linear_ccd
{

class DirControlAlgorithm
{
public:
	DirControlAlgorithm(Car *car);

	void Control(const bool *ccd_data);

private:
	void CcdScanAllWhiteOrAllBlackSample(const bool *ccd_data);

	Car *m_car;

	int32_t left_start_length;
	int32_t right_start_length;

	const int32_t ccd_mid_pos;

	int all_white_smaple_flag;
	int all_black_smaple_flag;

	int current_mid_error_pos;
	int last_sample_error_pos;
	int previous_mid_error_pos;

	int current_dir_error;
	int previous_dir_error;
	int difference_dir_error;

	int current_1st_left_edge;
	int current_1st_right_edge;

	int32_t current_edge_middle_distance;

	int detect_left_flag;
	int detect_right_flag;
};

}

#endif /* DETECTION_ALGORITHM_H_ */
