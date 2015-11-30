#ifndef UTILITIES.H
#define UTILITIES.H

#include "opencv2/opencv.hpp"
#include <cstdlib>
#include <string>
#include <ctime>
#include <sstream>

#define MAX_CAMERA_NUMBER 10
#define DELAY 3
#define MAX_SCREENS 25


struct CameraParameters
{
	cv::Mat cameraMatrix;
	cv::Mat distortion;
};

struct CalibrationParameters
{
	// Type of Pattern
	int patternType;
	// Size of pattern.
	cv::Size2i patternSize;
	// Distance between corners on the checkboard Pattern or
	// distance between circle centers on the circle pattern.
	double dist;
};

struct StereoRectify
{
	cv::Mat R1;
	cv::Mat R2;
	cv::Mat P1;
	cv::Mat P2;
	cv::Mat Q;
};

struct StereoParameters
{
	cv::Mat Rotation;
	cv::Mat Translation;
	cv::Mat Essential;
	cv::Mat Fundamental;
};

class Utilities
{
	public:
	// Function used to determine which is the left one .
	static int getLeftCameraID();
	static CameraParameters readXML(std::string name);
	static void writeXML(std::string name,CameraParameters param);
};

#endif
