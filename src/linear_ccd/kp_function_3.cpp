/*
 * kp_function_3.cpp
 *
 * Author: Louis Mo, Ming Tsang, Spartey Chen
 * Copyright (c) 2014 HKUST SmartCar Team
 */

#include <cstdlib>

#include "linear_ccd/kp_function_3.h"

namespace linear_ccd
{

float KpFunction3::OnCalc(const int error)
{
	const int abs_error = abs(error);
	if (abs_error <= 4)
	{
		return (abs_error * abs_error) / 14.0f;
	}
	else if (abs_error <= 24)
	{
		const int adjusted_err = abs_error - 1;
		return ((adjusted_err * adjusted_err * adjusted_err) / 150.0f + 1);
	}
	else
	{
		return 83;
	}
}

}
