/*
 * kd_function_3.cpp
 *
 * Author: Louis Mo, Ming Tsang, Spartey Chen
 * Copyright (c) 2014 HKUST SmartCar Team
 */

#include <cstdlib>

#include "linear_ccd/kd_function_3.h"

namespace linear_ccd
{

float KdFunction3::OnCalc(const int error)
{
	const int abs_error = abs(error);
	if (abs_error < 6)
	{
		return 50 - (1.0f / (7 - abs_error)) * 22;
	}
	else
	{
		return (float)(11 * 92) / (abs_error * abs_error);
	}
}

}
