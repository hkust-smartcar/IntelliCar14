/*
 * dir_control_algorithm_4.cpp
 *
 * Author: Louis Mo, Ming Tsang, Spartey Chen
 * Copyright (c) 2014 HKUST SmartCar Team
 */

#include <cassert>
#include <cstdint>

#include <bitset>
#include <memory>

#include <libsc/k60/linear_ccd.h>
#include <libutil/incremental_pid_controller.h>
#include <libutil/incremental_pid_controller.tcc>
#include <libutil/kalman_filter.h>
#include <libutil/misc.h>
#include <libutil/misc.h>
#include <libutil/pid_controller.h>
#include <libutil/pid_controller.tcc>
#include <libutil/string.h>

#include "linear_ccd/debug.h"
#include "linear_ccd/config.h"
#include "linear_ccd/beep_manager.h"
#include "linear_ccd/car.h"
#include "linear_ccd/dir_control_algorithm_4.h"
#include "linear_ccd/kd_function_1.h"
#include "linear_ccd/kd_function_2.h"
#include "linear_ccd/kd_function_3.h"
#include "linear_ccd/kd_function_4.h"
#include "linear_ccd/kd_function_5.h"
#include "linear_ccd/kd_function_6.h"
#include "linear_ccd/kp_function_1.h"
#include "linear_ccd/kp_function_2.h"
#include "linear_ccd/kp_function_3.h"
#include "linear_ccd/kp_function_4.h"
#include "linear_ccd/kp_function_5.h"
#include "linear_ccd/kp_function_6.h"
#include "linear_ccd/kpid_function.h"
#include "linear_ccd/track_analyzer.h"
#include "linear_ccd/turn_hint.h"

using namespace libsc::k60;
using namespace std;

#define KALMAN_FILTER_Q 0.0000005f
#define KALMAN_FILTER_R 0.0005f

namespace linear_ccd
{

DirControlAlgorithm4::DirControlAlgorithm4(Car *const car)
		: DirControlAlgorithm4(car, Parameter())
{}

DirControlAlgorithm4::DirControlAlgorithm4(Car *const car,
		const Parameter &parameter)
		: m_car(car),
		  m_parameter(parameter),

		  m_track_analyzer(parameter.mid, parameter.edge),
		  m_mid_filter(KALMAN_FILTER_Q, KALMAN_FILTER_R, Config::GetCcdMid(), 1),
		  m_filtered_mid(0),
		  m_pid(parameter.mid, 0.0f, 0.0f, 0.0f),
		  m_kp_fn(nullptr),
		  m_kd_fn(nullptr),

		  m_case(0),
		  m_turning(0),
		  m_prev_turning(0)
{
	SetKpFunction(m_parameter.kp_fn);
	SetKdFunction(m_parameter.kd_fn);
}

void DirControlAlgorithm4::SetParameter(const Parameter &parameter)
{
	m_parameter = parameter;
	m_mid_filter = libutil::KalmanFilter(KALMAN_FILTER_Q, KALMAN_FILTER_R,
			m_parameter.mid, 1);
	m_pid.SetSetpoint(m_parameter.mid);
	SetKpFunction(m_parameter.kp_fn);
	SetKdFunction(m_parameter.kd_fn);
	m_track_analyzer.SetMid(m_parameter.mid);
	m_track_analyzer.SetEdge(m_parameter.edge);
}

void DirControlAlgorithm4::OnFinishWarmUp()
{
	m_pid.Restart();
}

int32_t DirControlAlgorithm4::Process(
		const bitset<LinearCcd::SENSOR_W> &ccd_data)
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

void DirControlAlgorithm4::ProcessFill()
{
	/* ---------------------------------------- (no middle noise) Cross road*/
	if (m_track_analyzer.IsAllWhite())
	{
		m_case = 50;
		if (m_white_out_time++ < 5)
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

void DirControlAlgorithm4::ProcessGeneral()
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
				<= m_parameter.mid - m_parameter.edge)
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
				>= m_parameter.mid + m_parameter.edge)
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

int DirControlAlgorithm4::ConcludeMid()
{
	//return m_track_analyzer.GetMid();
	static bool is_turn_ = false;

	const int error = m_parameter.mid - m_track_analyzer.GetCurrMid();
	int new_mid;
	if (abs(error) < 6)
	{
		if (is_turn_)
		{
			m_mid_filter = libutil::KalmanFilter(KALMAN_FILTER_Q,
					KALMAN_FILTER_R, m_parameter.mid, 1);
		}
		is_turn_ = false;
		new_mid = m_mid_filter.Filter(m_track_analyzer.GetCurrMid());
	}
	else
	{
		is_turn_ = true;
		new_mid = m_track_analyzer.GetCurrMid();
	}

	//const int new_error = m_parameter.mid - new_mid;
	//return m_parameter.mid - (int)(new_error * (1 + abs(new_error) / 85.0f));
	return new_mid;
}

float DirControlAlgorithm4::ConcludeKp()
{
	const int error = m_parameter.mid - m_track_analyzer.GetCurrMid();
	if (m_kp_fn)
	{
		return m_kp_fn->Calc(error);
	}
	else
	{
		return m_parameter.kp;
	}
}

float DirControlAlgorithm4::ConcludeKd()
{
	const int error = m_parameter.mid - m_track_analyzer.GetCurrMid();
	if (m_kd_fn)
	{
		return m_kd_fn->Calc(error);
	}
	else
	{
		return m_parameter.kd;
	}
}

void DirControlAlgorithm4::SetKpFunction(const int kp_fn)
{
	switch (kp_fn)
	{
	default:
		assert(false);

	case 0:
		m_kp_fn = nullptr;
		return;

	case 1:
		m_kp_fn.reset(new KpFunction1);
		break;

	case 2:
		m_kp_fn.reset(new KpFunction2);
		break;

	case 3:
		m_kp_fn.reset(new KpFunction3);
		break;

	case 4:
		m_kp_fn.reset(new KpFunction4);
		break;

	case 5:
		m_kp_fn.reset(new KpFunction5);
		break;

	case 6:
		m_kp_fn.reset(new KpFunction6);
		break;
	}

	m_kp_fn->SetMultiplier(m_parameter.kp);
}

void DirControlAlgorithm4::SetKdFunction(const int kd_fn)
{
	switch (kd_fn)
	{
	default:
		assert(false);

	case 0:
		m_kd_fn = nullptr;
		return;

	case 1:
		m_kd_fn.reset(new KdFunction1);
		break;

	case 2:
		m_kd_fn.reset(new KdFunction2);
		break;

	case 3:
		m_kd_fn.reset(new KdFunction3);
		break;

	case 4:
		m_kd_fn.reset(new KdFunction4);
		break;

	case 5:
		m_kd_fn.reset(new KdFunction5);
		break;

	case 6:
		m_kd_fn.reset(new KdFunction6);
		break;
	}

	m_kd_fn->SetMultiplier(m_parameter.kd);
}

}
