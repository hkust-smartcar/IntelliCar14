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

#include <log.h>
#include <libutil/clock.h>
#include<stdlib.h>
#include<stdio.h>
#include <MK60_pit.h>
#include <vectors.h>
#include <libutil/clock.h>
#include <libutil/misc.h>
#include <libutil/pid_controller.h>
#include <libutil/pid_controller.tcc>
#include"libutil/clock.h"


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

	static MagneticApp *m_instance;
	void InitAll();
	void CaluServoPercentage();
	void DirectionControl();

};

}

#endif /* MAGNETIC_APP_H_ */
