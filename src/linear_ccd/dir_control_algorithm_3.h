/*
 * dir_control_algorithm_3.h
 *
 * Author: Louis Mo, Ming Tsang, Spartey Chen
 * Copyright (c) 2014 HKUST SmartCar Team
 */

#ifndef LINEAR_CCD_DIR_CONTROL_ALGORITHM_3_H_
#define LINEAR_CCD_DIR_CONTROL_ALGORITHM_3_H_

#include <cstdint>
#include <bitset>

#include <libbase/k60/misc_utils.h>
#include <libutil/kalman_filter.h>
#include <libutil/pid_controller.h>

#include "linear_ccd/kd_function_3.h"
#include "linear_ccd/kd_function_4.h"
#include "linear_ccd/kp_function_3.h"
#include "linear_ccd/kp_function_4.h"
#include "linear_ccd/track_analyzer.h"
#include "linear_ccd/turn_hint.h"

namespace linear_ccd
{

class DirControlAlgorithm3
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

	explicit DirControlAlgorithm3(Car *const car);
	DirControlAlgorithm3(Car *const car, const Parameter &parameter);

	void SetParameter(const Parameter &parameter);
	const Parameter& GetParameter() const
	{
		return m_parameter;
	}

	void OnFinishWarmUp();
	int32_t Process(const std::bitset<libsc::k60::LinearCcd::SENSOR_W> &ccd_data);


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
	libutil::PidController<int32_t, int32_t> m_pid;
	KpFunction3 m_kp_func;
	KdFunction3 m_kd_func;

	uint8_t m_case;
	int32_t m_turning;
	int32_t m_prev_turning;

	Uint m_white_out_time;
};

}

#endif /* LINEAR_CCD_DIR_CONTROL_ALGORITHM_3_H_ */
