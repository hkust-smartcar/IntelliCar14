/*
 * dir_control_algorithm_2.h
 *
 * Author: Louis Mo, Ming Tsang, Spartey Chen
 * Copyright (c) 2014 HKUST SmartCar Team
 */

#ifndef LINEAR_CCD_DIR_CONTROL_ALGORITHM_2_H_
#define LINEAR_CCD_DIR_CONTROL_ALGORITHM_2_H_

#include <cstdint>
#include <bitset>

#include <libbase/k60/misc_utils.h>
#include <libutil/kalman_filter.h>
#include <libutil/positional_pid_controller.h>

#include "linear_ccd/kd_function_2.h"
#include "linear_ccd/kp_function_2.h"
#include "linear_ccd/track_analyzer.h"
#include "linear_ccd/turn_hint.h"

namespace linear_ccd
{

class DirControlAlgorithm2
{
public:
	struct Parameter
	{
		int edge = 25;
		float kp = 0;
		float kd = 0;
		float turn_kp = 0;
		float turn_kd = 0;
	};

	explicit DirControlAlgorithm2(Car *const car);
	DirControlAlgorithm2(Car *const car, const Parameter &parameter);

	void SetParameter(const Parameter &parameter);
	const Parameter& GetParameter() const
	{
		return m_parameter;
	}

	void OnFinishWarmUp();
	int32_t Process(const std::bitset<libsc::k60::LinearCcd::kSensorW> &ccd_data);


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
		return m_pid.GetSetpoint();
	}

	float GetP() const
	{
		return m_pid.GetP();
	}

	float GetD() const
	{
		return m_pid.GetD();
	}

private:
	void ProcessFill();
	void ProcessGeneral();

	int ConcludeMid();
	float ConcludeKp();
	float ConcludeKd();

	void SetTurnHint(const TurnHint hint);

	Car *const m_car;
	Parameter m_parameter;

	TrackAnalyzer m_track_analyzer;
	libutil::KalmanFilter m_mid_filter;
	int m_filtered_mid;
	libutil::PositionalPidController<int32_t, int32_t> m_pid;
	KpFunction2 m_kp_func;
	KdFunction2 m_kd_func;

	uint8_t m_case;
	int32_t m_turning;
	int32_t m_prev_turning;

	Uint m_white_out_time;
};

}

#endif /* LINEAR_CCD_DIR_CONTROL_ALGORITHM_2_H_ */
