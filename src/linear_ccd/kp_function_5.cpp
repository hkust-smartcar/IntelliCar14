/*
 * kp_function_4.cpp
 *
 * Author: Louis Mo, Ming Tsang, Spartey Chen
 * Copyright (c) 2014 HKUST SmartCar Team
 */

#include <cstdlib>

#include "linear_ccd/kp_function_5.h"

namespace linear_ccd
{

float KpFunction5::OnCalc(const int error)
{
	const int abs_error = abs(error);
	if (abs_error <= 119)
	{
		const int adjusted_error = abs_error + 8;
		return (adjusted_error * adjusted_error) * 0.005f + 1.75f;
	}
	else
	{
		return 83;
	}
}

}
