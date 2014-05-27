/*
 * speed_control_strategy.h
 * Interface of speed controllers
 *
 * Author: Louis Mo, Ming Tsang, Spartey Chen
 * Copyright (c) 2014 HKUST SmartCar Team
 */

#ifndef LINEAR_CCD_SPEED_CONTROL_STRATEGY_H_
#define LINEAR_CCD_SPEED_CONTROL_STRATEGY_H_

#include <mini_common.h>

namespace linear_ccd
{

class Car;

}

namespace linear_ccd
{

class SpeedControlStrategy
{
public:
	virtual ~SpeedControlStrategy()
	{}

	virtual void OnFinishWarmUp(Car *car) = 0;
	virtual void Control(Car *car) = 0;
	virtual void SetMode(const Uint mode) = 0;
};

}

#endif /* LINEAR_CCD_SPEED_CONTROL_STRATEGY_H_ */