/*
 * dir_control_algorithm.h
 * Algorithm for direction control
 *
 * Author: Louis Mo, Ming Tsang, Spartey Chen
 * Copyright (c) 2014 HKUST SmartCar Team
 */

#ifndef LINEAR_CCD_DIR_CONTROL_ALGORITHM_H_
#define LINEAR_CCD_DIR_CONTROL_ALGORITHM_H_

#include <cstdint>
#include <bitset>

#include <libutil/kalman_filter.h>
#include <libutil/positional_pid_controller.h>

#include "linear_ccd/track_analyzer.h"
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
	int16_t Process(const std::bitset<libsc::k60::LinearCcd::kSensorW> &ccd_data);
	void SetMode(const Uint mode);
	void SetTurnHint(const TurnHint hint);

	uint8_t GetCase() const
	{
		return m_case;
	}

	int GetMid() const
	{
		return m_filtered_mid;
	}

	bool IsAllBlack() const
	{
		return m_track_analyzer.IsAllBlack();
	}

	bool IsAllWhite() const
	{
		return m_track_analyzer.IsAllWhite();
	}

	int GetCurrLeftEdge() const
	{
		return m_track_analyzer.GetLeftEdge();
	}

	int GetCurrRightEdge() const
	{
		return m_track_analyzer.GetRightEdge();
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

	// All black or all white
	void ProcessFill();
	void ProcessGeneral();

	Car *m_car;
	int16_t m_flat_gyro_angle;

	TrackAnalyzer m_track_analyzer;
	libutil::KalmanFilter m_mid_filter;
	int m_filtered_mid;

	libutil::PositionalPidController<int32_t, int32_t> m_servo_pid;

	uint8_t m_case;
	int16_t m_turning;
	int16_t m_prev_turning;
	TurnHint m_prev_turn_hint;

	uint8_t m_mode;
	bool m_is_explicit_set_turn_hint;
};

}

#endif /* LINEAR_CCD_DIR_CONTROL_ALGORITHM_H_ */
