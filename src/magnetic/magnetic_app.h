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
#include<libutil/kalman_filter.h>

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
	__ISR void ServoFTMHandler();
	static void GetSensorValue();
	Car m_car;

	static MagneticApp *m_instance;
	void InitAll();
	void CaluServoPercentage();
	void DirectionControl();
	void SpdCtrlpass();
	uint16 Sensor_Average(int);
	float sensor_offset();
	uint16 CalcOffset(uint16);
	libutil::KalmanFilter m_MagneticSenlx_filter;
	libutil::KalmanFilter m_MagneticSenly_filter;
	libutil::KalmanFilter m_MagneticSenrx_filter;
	libutil::KalmanFilter m_MagneticSenry_filter;

};

}

#endif /* MAGNETIC_APP_H_ */
