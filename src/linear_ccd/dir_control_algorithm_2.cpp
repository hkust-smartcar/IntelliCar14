/*
 * dir_control_algorithm_2.cpp
 *
 * Author: Louis Mo, Ming Tsang, Spartey Chen
 * Copyright (c) 2014 HKUST SmartCar Team
 */

#include <cstdint>

#include <bitset>

#include <libsc/k60/linear_ccd.h>
#include <libutil/kalman_filter.h>
#include <libutil/misc.h>
#include <libutil/misc.h>
#include <libutil/positional_pid_controller.h>
#include <libutil/string.h>

#include "linear_ccd/debug.h"
#include "linear_ccd/config.h"
#include "linear_ccd/beep_manager.h"
#include "linear_ccd/car.h"
#include "linear_ccd/dir_control_algorithm_2.h"
#include "linear_ccd/kd_function_3.h"
#include "linear_ccd/kp_function_3.h"
#include "linear_ccd/track_analyzer.h"
#include "linear_ccd/turn_hint.h"

using namespace libsc::k60;
using namespace std;

#define CCD_MID_POS Config::GetCcdMid()

#define KALMAN_FILTER_Q 0.0000005f
#define KALMAN_FILTER_R 0.0005f

namespace linear_ccd
{

DirControlAlgorithm2::DirControlAlgorithm2(Car *const car)
		: DirControlAlgorithm2(car, Parameter())
{}

DirControlAlgorithm2::DirControlAlgorithm2(Car *const car,
		const Parameter &parameter)
		: m_car(car),
		  m_parameter(parameter),

		  m_track_analyzer(CCD_MID_POS, parameter.edge),
		  m_mid_filter(KALMAN_FILTER_Q, KALMAN_FILTER_R, 64, 1),
		  m_filtered_mid(0),
		  m_pid(CCD_MID_POS, parameter.kp, 0.0f, parameter.kd),

		  m_case(0),
		  m_turning(0),
		  m_prev_turning(0)
{}

void DirControlAlgorithm2::SetParameter(const Parameter &parameter)
{
	m_parameter = parameter;
	m_kp_func.SetMultiplier(m_parameter.kp);
	m_kd_func.SetMultiplier(m_parameter.kd);
	m_track_analyzer.SetEdge(m_parameter.edge);
}

void DirControlAlgorithm2::OnFinishWarmUp()
{
	m_pid.Reset();
}

int32_t DirControlAlgorithm2::Process(
		const bitset<LinearCcd::kSensorW> &ccd_data)
{
	m_case = 0;
	m_turning = INT16_MIN;
	m_track_analyzer.Analyze(ccd_data);

	if (m_track_analyzer.IsAllBlack() || m_track_analyzer.IsAllWhite())
	{
		ProcessFill();
	}
	else
	{
		m_white_out_time = 0;
		ProcessGeneral();
	}

	m_filtered_mid = ConcludeMid();
	m_pid.SetKp(ConcludeKp());
	m_pid.SetKd(ConcludeKd());
	if (m_turning == INT16_MIN)
	{
		// Opposite direction
		m_turning = -m_pid.Calc(m_filtered_mid);
	}
	else
	{
		m_pid.ResetTime();
	}

	m_prev_turning = m_turning;
	return m_turning;
}

void DirControlAlgorithm2::ProcessFill()
{
	/* ---------------------------------------- (no middle noise) Cross road*/
	if (m_track_analyzer.IsAllWhite())
	{
		m_case = 50;
		if (m_white_out_time++ < 2)
		{
			m_turning = m_prev_turning;
		}
		else
		{
			m_turning = 0;
		}
	}

	/* |||||||||||||||||||||||||||||||||||||||| (all black) */
	else // m_track_analyzer.IsAllBlack()
	{
		m_case = 60;
		m_white_out_time = 0;
		if (m_car->GetTurning() >= Config::GetTurnThreshold() + 5)
		{
			m_turning = 100;
		}
		else if (m_car->GetTurning() <= -Config::GetTurnThreshold() - 5)
		{
			m_turning = -100;
		}
	}
}

void DirControlAlgorithm2::ProcessGeneral()
{
	/* ||||--------------------------------|||| */
	if (m_track_analyzer.GetLeftEdge() != -1
			&& m_track_analyzer.GetRightEdge() != -1)
	{
		m_case = 10;
	}

	/* ||||--------------------------------||||
	   |||||||||||||||------------------------
	   |||||||||||||||||||||||--------------- */
	else if (m_track_analyzer.GetRightEdge() == -1)
	{
		if (m_track_analyzer.GetLeftEdge()
				<= CCD_MID_POS - m_parameter.edge)
		{
			// Possibly crossroad
			m_case = 20;
			//m_turning = m_prev_turning;
#ifdef DEBUG_BEEP_GUESS_CROSSROAD
			m_car->GetBeepManager()->Beep(100);
#endif
		}
		else
		{
			m_case = 21;
		}
	}

	/* ||||-------------------------------||||
	   --------------------------|||||||||||||
	   -----------------|||||||||||||||||||||| */
	else  // m_track_analyzer.GetLeftEdge() == -1
	{
		if (m_track_analyzer.GetRightEdge()
				>= CCD_MID_POS + m_parameter.edge)
		{
			// Possibly crossroad
			m_case = 30;
			//m_turning = m_prev_turning;
#ifdef DEBUG_BEEP_GUESS_CROSSROAD
			m_car->GetBeepManager()->Beep(100);
#endif
		}
		else
		{
			m_case = 31;
		}
	}
}

int DirControlAlgorithm2::ConcludeMid()
{
	static bool is_turn_ = false;

	const int error = CCD_MID_POS - m_track_analyzer.GetCurrMid();
	int new_mid;
	if (abs(error) < 6)
	{
		if (is_turn_)
		{
			m_mid_filter = libutil::KalmanFilter(KALMAN_FILTER_Q,
					KALMAN_FILTER_R, 64, 1);
		}
		is_turn_ = false;
		new_mid = m_mid_filter.Filter(m_track_analyzer.GetCurrMid());
	}
	else
	{
		is_turn_ = true;
		new_mid = m_track_analyzer.GetCurrMid();
	}

	const int new_error = CCD_MID_POS - new_mid;
	return CCD_MID_POS - (int)(new_error * (1 + abs(new_error) / 85.0f));
	//return CCD_MID_POS - new_error;
}

float DirControlAlgorithm2::ConcludeKp()
{
	const int error = CCD_MID_POS - m_track_analyzer.GetCurrMid();
	return m_kp_func.Calc(error);
}

float DirControlAlgorithm2::ConcludeKd()
{
	const int error = CCD_MID_POS - m_track_analyzer.GetCurrMid();
	return m_kd_func.Calc(error);
}

}
