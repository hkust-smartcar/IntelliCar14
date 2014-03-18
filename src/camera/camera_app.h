/*
 * camera_app.h
 * Camera App
 *
 * Author:
 * Copyright (c) 2014 HKUST SmartCar Team
 */

#ifndef CAMERA_APP_H_
#define CAMERA_APP_H_

#include "camera/car.h"

namespace camera
{

class CameraApp
{
public:
	CameraApp();
	~CameraApp();

	void Run();

	static int FwriteHandler(int, char *ptr, int len);

private:
	Car m_car;

	static CameraApp *m_instance;
};

}

#endif /* CAMERA_APP_H_ */
