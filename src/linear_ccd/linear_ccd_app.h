#ifndef LINEAR_CCD_APP_H_
#define LINEAR_CCD_APP_H_

#include <libsc/ccd_smart_car.h>

namespace linear_ccd
{

class LinearCcdApp
{
public:
	void Run();

private:
	libsc::CcdSmartCar mCar;
};

}

#endif /* LINEAR_CCD_APP_H_ */
