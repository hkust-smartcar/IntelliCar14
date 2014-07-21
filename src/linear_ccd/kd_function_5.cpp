/*
 * kd_function_5.cpp
 *
 * Author: Louis Mo, Ming Tsang, Spartey Chen
 * Copyright (c) 2014 HKUST SmartCar Team
 */

#include <cstdlib>

#include "linear_ccd/kd_function_5.h"

namespace linear_ccd
{

float KdFunction5::OnCalc(const int error)
{
	const int abs_error = abs(error);
	if (abs_error < 6)
	{
		return 52 - (1.0f / (7.5f - abs_error)) * 25;
	}
	else
	{
		return (float)(13 * 99) / (abs_error * abs_error);
	}
}

}
