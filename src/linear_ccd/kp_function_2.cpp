/*
 * kp_function_2.cpp
 *
 * Author: Louis Mo, Ming Tsang, Spartey Chen
 * Copyright (c) 2014 HKUST SmartCar Team
 */

#include <cstdlib>

#include "linear_ccd/kp_function_2.h"

namespace linear_ccd
{

float KpFunction2::OnCalc(const int error)
{
	const int abs_error = abs(error);
	return (abs_error * abs_error) / 29.0f;
}

}
