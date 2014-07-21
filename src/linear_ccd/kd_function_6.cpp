/*
 * kd_function_6.cpp
 *
 * Author: Louis Mo, Ming Tsang, Spartey Chen
 * Copyright (c) 2014 HKUST SmartCar Team
 */

#include <cstdlib>

#include "linear_ccd/kd_function_6.h"

namespace linear_ccd
{

float KdFunction6::OnCalc(const int error)
{
	const int abs_error = abs(error);
	if (abs_error <= 7)
	{
		return 53 - (30.0f / (9 - abs_error));
	}
	else
	{
		return (float)(18 * 95) / (abs_error * abs_error) + (20.0f / abs_error);
	}
}

}
