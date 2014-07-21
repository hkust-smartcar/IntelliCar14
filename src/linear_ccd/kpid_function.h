/*
 * kpid_function.h
 *
 * Author: Louis Mo, Ming Tsang, Spartey Chen
 * Copyright (c) 2014 HKUST SmartCar Team
 */

#ifndef LINEAR_CCD_KPID_FUNCTION_H_
#define LINEAR_CCD_KPID_FUNCTION_H_

namespace linear_ccd
{

class KpidFunction
{
public:
	virtual ~KpidFunction()
	{}

	float Calc(const int error)
	{
		return OnCalc(error) * m_multiplier;
	}

	void SetMultiplier(const float multiplier)
	{
		m_multiplier = multiplier;
	}

protected:
	virtual float OnCalc(const int error) = 0;

private:
	float m_multiplier = 1.0f;
};

}

#endif /* LINEAR_CCD_KPID_FUNCTION_H_ */
