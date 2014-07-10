/*
 * program.h
 *
 * Author: Louis Mo, Ming Tsang, Spartey Chen
 * Copyright (c) 2014 HKUST SmartCar Team
 */

#ifndef LINEAR_CCD_PROGRAM_H_
#define LINEAR_CCD_PROGRAM_H_

namespace linear_ccd
{

class Program
{
public:
	enum struct Token
	{
		AUTO2,
		AUTO,
		CALIBRATE,

		SIZE,
	};

	virtual ~Program()
	{}

	virtual void Run() = 0;
};

}

#endif /* LINEAR_CCD_PROGRAM_H_ */
