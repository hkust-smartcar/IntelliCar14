/*
 * kd_function_3.h
 *
 * Author: Louis Mo, Ming Tsang, Spartey Chen
 * Copyright (c) 2014 HKUST SmartCar Team
 */

#ifndef LINEAR_CCD_KD_FUNCTION_3_H_
#define LINEAR_CCD_KD_FUNCTION_3_H_

#include "linear_ccd/kpid_function.h"

namespace linear_ccd
{

class KdFunction3 : public KpidFunction
{
protected:
	float OnCalc(const int error) override;
};

}

#endif /* LINEAR_CCD_KD_FUNCTION_3_H_ */
