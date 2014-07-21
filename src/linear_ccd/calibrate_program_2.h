/*
 * calibrate_program_2.h
 *
 * Author: Louis Mo, Ming Tsang, Spartey Chen
 * Copyright (c) 2014 HKUST SmartCar Team
 */

#ifndef LINEAR_CCD_CALIBRATE_PROGRAM_2_H_
#define LINEAR_CCD_CALIBRATE_PROGRAM_2_H_

#include <libbase/k60/dac.h>

#include "linear_ccd/program.h"

namespace linear_ccd
{

class CalibrateProgram2 : public Program
{
public:
	CalibrateProgram2();

	void Run() override;
};

}

#endif /* LINEAR_CCD_CALIBRATE_PROGRAM_2_H_ */
