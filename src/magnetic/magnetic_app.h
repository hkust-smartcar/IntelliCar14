/*
 * magnetic_app.h
 * Magnetic App
 *
 * Author:
 * Copyright (c) 2014 HKUST SmartCar Team
 */

#ifndef MAGNETIC_APP_H_
#define MAGNETIC_APP_H_

#include "magnetic/car.h"
//#include <libutil/kalman_filter.h>

namespace magnetic
{
//class KalmanFilter;
class MagneticApp
{
public:
	MagneticApp();
	~MagneticApp();

	void Run();

	static int FwriteHandler(int, char *ptr, int len);

private:
	__ISR static void Pit0Handler();
	__ISR static void Pit1Handler();
	__ISR static void Pit2Handler();
	__ISR static void Pit3Handler();

	static void GetSensorValue();

	Car m_car;

	//libutil::KalmanFilter m_magneticfilter(1.0,2.0,1.0,0.1);

	static MagneticApp *m_instance;
};

}

#endif /* MAGNETIC_APP_H_ */
