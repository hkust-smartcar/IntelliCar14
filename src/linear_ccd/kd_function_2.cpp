/*
 * kd_function_2.cpp
 *
 * Author: Louis Mo, Ming Tsang, Spartey Chen
 * Copyright (c) 2014 HKUST SmartCar Team
 */

#include <cstdlib>

#include "linear_ccd/kd_function_2.h"

namespace linear_ccd
{

float KdFunction2::OnCalc(const int error)
{
	const int abs_error = abs(error);
	if (abs_error < 6)
	{
		return 50 - (1.0f / (7 - abs_error)) * 25;
	}
	else
	{
/*
		int factor = 1;
		if (abs_error >= 11)
		{
			return 0;
		}
		else
		{
			for (int i = 0; i < abs_error - 2; ++i)
			{
				factor *= abs_error;
			}
			return m_parameter.kd * 1.0f / factor * 19999;
		}
*/
		return (float)(10 * 91) / (abs_error * abs_error);
	}
}

}
