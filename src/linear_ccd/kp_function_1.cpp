/*
 * kp_function_1.cpp
 *
 * Author: Louis Mo, Ming Tsang, Spartey Chen
 * Copyright (c) 2014 HKUST SmartCar Team
 */

#include <cstdlib>

#include "linear_ccd/kp_function_1.h"

namespace linear_ccd
{

float KpFunction1::OnCalc(const int error)
{
	const int abs_error = abs(error);
	if (abs_error < 6)
	{
		return 0.5f;
	}
	else
	{
		return (1.0f + abs_error / 5.0f);
	}
}

}
