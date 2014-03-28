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

#include <libutil/pid_controller.h>

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

	bool all_white_smaple_flag;
	bool all_black_smaple_flag;

	bool first_straight_line_flag;

	int current_mid_error_pos;
	int last_sample_error_pos;
	int previous_mid_error_pos;

	int current_1st_left_edge;
	int current_1st_right_edge;

	bool detect_left_flag;
	bool detect_right_flag;

	libutil::PidController<int, int> m_servo_pid;
};

}

#endif /* DETECTION_ALGORITHM_H_ */
