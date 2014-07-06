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

#include "linear_ccd/turn_hint.h"

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
	int16_t Process(const std::bitset<libsc::k60::LinearCcd::SENSOR_W> &ccd_data);
	void SetMode(const Uint mode);
	void SetTurnHint(const TurnHint hint);

	uint8_t GetCase() const
	{
		return m_case;
	}

	int GetMid() const
	{
		return m_prev_mid;
	}

	bool IsAllBlack() const
	{
		return m_is_all_black;
	}

	bool IsAllWhite() const
	{
		return m_is_all_white;
	}

	int GetCurrLeftEdge() const
	{
		return m_curr_left_edge;
	}

	int GetCurrRightEdge() const
	{
		return m_curr_right_edge;
	}

	int32_t GetSetpoint() const
	{
		return m_servo_pid.GetSetpoint();
	}

	float GetP() const
	{
		return m_servo_pid.GetP();
	}

	float GetD() const
	{
		return m_servo_pid.GetD();
	}

	void ResetMid();

private:
	bool DetectSlope();
	void ScanAllWhiteOrAllBlackSample(
			const std::bitset<libsc::k60::LinearCcd::SENSOR_W> &ccd_data);

	// All black or all white
	void ProcessFill();
	void ProcessGeneral(
			const std::bitset<libsc::k60::LinearCcd::SENSOR_W> &ccd_data);

	Car *m_car;
	int16_t m_flat_gyro_angle;

	bool m_is_all_black;
	bool m_is_all_white;

	int m_curr_left_edge;
	int m_curr_right_edge;

	int m_prev_mid;
	int m_curr_mid;

	libutil::PidController<int32_t, int32_t> m_servo_pid;
	libutil::KalmanFilter m_mid_filter;

	uint8_t m_case;
	int16_t m_turning;
	int16_t m_prev_turning;

	uint8_t m_mode;
	bool m_is_explicit_set_turn_hint;
};

}

#endif /* DETECTION_ALGORITHM_H_ */
