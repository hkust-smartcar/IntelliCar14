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
	explicit DirControlAlgorithm(Car *car);

	void Control(const bool *ccd_data);

	void SetConstant(const int id);

private:
	bool DetectSlope();
	void ScanAllWhiteOrAllBlackSample(const bool *ccd_data);

	Car *m_car;
	int16_t m_flat_gyro_angle;

	bool all_white_smaple_flag;
	bool all_black_smaple_flag;

	int last_sample_error_pos;

	libutil::PidController<int32_t, int32_t> m_servo_pid;

	uint8_t m_constant_choice;
};

}

#endif /* DETECTION_ALGORITHM_H_ */
