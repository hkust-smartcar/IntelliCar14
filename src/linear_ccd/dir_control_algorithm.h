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
#include <bitset>

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
	int16_t Process(const std::bitset<libsc::LinearCcd::SENSOR_W> &ccd_data);
	void SetMode(const Uint mode);

	uint8_t GetCase() const
	{
		return m_case;
	}

	int GetMid() const
	{
		return m_prev_mid;
	}

private:
	bool DetectSlope();
	void ScanAllWhiteOrAllBlackSample(
			const std::bitset<libsc::LinearCcd::SENSOR_W> &ccd_data);
	void UpdatePid(const bool is_straight);

	// All black or all white
	void ProcessFill();
	void ProcessGeneral(const std::bitset<libsc::LinearCcd::SENSOR_W> &ccd_data);

	Car *m_car;
	int16_t m_flat_gyro_angle;

	bool all_white_smaple_flag;
	bool all_black_smaple_flag;

	int m_prev_mid;
	int m_curr_mid;

	libutil::PidController<int32_t, int32_t> m_servo_pid;
	libutil::KalmanFilter m_gyro_filter;

	uint8_t m_case;
	int16_t m_turning;

	uint8_t m_mode;
};

}

#endif /* DETECTION_ALGORITHM_H_ */
