/*
 * dir_control_algorithm.h
 * Algorithm for direction control
 *
 * Author: Louis Mo, Ming Tsang, Spartey Chen
 * Copyright (c) 2014 HKUST SmartCar Team
 */

#ifndef LINEAR_CCD_DETECTION_ALGORITHM_H_
#define LINEAR_CCD_DETECTION_ALGORITHM_H_

#include <cstdint>

#include <libutil/kalman_filter.h>
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

	void OnFinishWarmUp(Car *car);
	void Control(const bool *ccd_data);
	void SetMode(const Uint mode);

private:
	bool DetectSlope();
	void ScanAllWhiteOrAllBlackSample(const bool *ccd_data);

	Car *m_car;
	int16_t m_flat_gyro_angle;

	bool all_white_smaple_flag;
	bool all_black_smaple_flag;

	int last_sample_error_pos;

	libutil::PidController<int32_t, int32_t> m_servo_pid;
	libutil::KalmanFilter m_gyro_filter;

	uint8_t m_mode;
};

}

#endif /* DETECTION_ALGORITHM_H_ */
