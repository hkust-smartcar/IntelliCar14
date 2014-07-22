/*
 * kp_function_5.h
 *
 * Author: Louis Mo, Ming Tsang, Spartey Chen
 * Copyright (c) 2014 HKUST SmartCar Team
 */

#ifndef LINEAR_CCD_KP_FUNCTION_5_H_
#define LINEAR_CCD_KP_FUNCTION_5_H_

#include "linear_ccd/kpid_function.h"

namespace linear_ccd
{

class KpFunction5 : public KpidFunction
{
protected:
	float OnCalc(const int error) override;
};

}

#endif /* LINEAR_CCD_KP_FUNCTION_5_H_ */
