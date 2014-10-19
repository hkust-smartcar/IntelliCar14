#include <libbase/k60/mcg.h>

#include <libsc/lib_guard.h>

#if defined(K60_2014_CCD) || defined(K60_2013_CCD)
	#include "linear_ccd/linear_ccd_app.h"
#elif defined(K60_2014_CAMERA)
	#include "camera/camera_app.h"
#elif defined(K60_2014_MAGNETIC)
	#include "magnetic/magnetic_app.h"
#endif

namespace libbase
{
namespace k60
{

Mcg::Config Mcg::GetMcgConfig()
{
	Mcg::Config config;
	config.external_oscillator_khz = 50000;
	config.core_clock_khz = 100000;
	config.bus_clock_khz = 50000;
	config.flexbus_clock_khz = 50000;
	config.flash_clock_khz = 25000;
	return config;
}

}
}

int main()
{
	LIBSC_GUARD();
#if defined(LINEAR_CCD)
	linear_ccd::LinearCcdApp app;
#elif defined(CAMERA)
	camera::CameraApp app;
#elif defined(MAGNETIC)
	magnetic::MagneticApp app;
#endif
	app.Run();
	return 0;
}
