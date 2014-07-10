/*
 * calibrate_program.h
 *
 * Author: Louis Mo, Ming Tsang, Spartey Chen
 * Copyright (c) 2014 HKUST SmartCar Team
 */

#ifndef LINEAR_CCD_CALIBRATE_PROGRAM_H_
#define LINEAR_CCD_CALIBRATE_PROGRAM_H_

#include "linear_ccd/car.h"
#include "linear_ccd/program.h"
#include "linear_ccd/track_analyzer.h"

namespace linear_ccd
{

class CalibrateProgram : public Program
{
public:
	CalibrateProgram();

	void Run() override;

private:
	Car m_car;
	TrackAnalyzer m_track_analyzer;
};

}

#endif /* LINEAR_CCD_CALIBRATE_PROGRAM_H_ */
