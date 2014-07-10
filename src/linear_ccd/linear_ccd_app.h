/*
 * linear_ccd_app.h
 * Linear CCD App
 *
 * Author: Louis Mo, Ming Tsang, Spartey Chen
 * Copyright (c) 2014 HKUST SmartCar Team
 */

#ifndef LINEAR_CCD_APP_H_
#define LINEAR_CCD_APP_H_

#include "linear_ccd/program.h"

namespace linear_ccd
{

class LinearCcdApp
{
public:
	void Run();

private:
	void Beep();
	void ServoProtect();

	static Program::Token SelectProgram();
};

}

#endif /* LINEAR_CCD_APP_H_ */
