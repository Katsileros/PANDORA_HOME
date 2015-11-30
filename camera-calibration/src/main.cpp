#include "opencv/cv.h"
#include "opencv/highgui.h"
#include <iostream>
#include "opencv2/calib3d/calib3d.hpp"
#include "stereoCamera.h"

int main()
{
	//~ StereoCamera cam(1); // Calibration
	
    StereoCamera cam("Stereo"); //Disparity
    cam.generate3d();
    //~ cam.testRectification();

	return 0;
}
