/*
 * camera_app.cpp
 * Camera App
 *
 * Author:
 * Copyright (c) 2014 HKUST SmartCar Team
 */

#include <syscall.h>

#include "camera/car.h"

#include "camera/camera_app.h"

namespace camera
{

CameraApp *CameraApp::m_instance = nullptr;

CameraApp::CameraApp()
{
	m_instance = this;
}

CameraApp::~CameraApp()
{
	m_instance = nullptr;
}

void CameraApp::Run()
{
	__g_fwrite_handler = FwriteHandler;

	while (true)
	{

	}
}

int CameraApp::FwriteHandler(int, char *ptr, int len)
{
	if (m_instance)
	{
		m_instance->m_car.UartSendBuffer((const uint8_t*)ptr, len);
	}
	return len;
}

}
