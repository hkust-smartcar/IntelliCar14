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

namespace magnetic
{

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

	Car m_car;

	static MagneticApp *m_instance;
};

}

#endif /* MAGNETIC_APP_H_ */
