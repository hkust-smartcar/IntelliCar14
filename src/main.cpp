#if defined(LINEAR_CCD)
	#include "linear_ccd/linear_ccd_app.h"
#elif defined(CAMERA)
	#include "camera/camera_app.h"
#elif defined(MAGNETIC)
	#include "magnetic/magnetic_app.h"
#endif

int main()
{
#if defined(LINEAR_CCD)
	LinearCcdApp app;
#elif defined(CAMERA)
	CameraApp app;
#elif defined(MAGNETIC)
	MagneticApp app;
#endif
	app.Run();
	return 0;
}
