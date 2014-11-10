#include <libbase/k60/mcg.h>

#include <libsc/lib_guard.h>

#include "linear_ccd/linear_ccd_app.h"

namespace libbase
{
namespace k60
{

Mcg::Config Mcg::GetMcgConfig()
{
	Mcg::Config config;
	config.external_oscillator_khz = 50000;
	config.core_clock_khz = 180000;
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
	linear_ccd::LinearCcdApp app;
	app.Run();
	return 0;
}
