
#ifndef STEREO_CAMERA.H
#define STEREO_CAMERA.H

#include "Utilities.h"
#include "monocularCamera.h"
#include "opencv2/contrib/contrib.hpp"

class StereoCamera
{
	MonocularCamera right;
	MonocularCamera left;
	StereoParameters parameters;
	StereoRectify rectifiedParam;
	CalibrationParameters calibParameters;
	std::string fileName;
    cv::StereoSGBM sgbm;
    cv::StereoBM sbm;
    
	cv::Mat depth;
	
    
    
	public:
		static cv::Mat staticDisparity;
		StereoCamera();
		StereoCamera(std::string filename);
		StereoCamera(int num);
		void getCameraID(int *left,int* right);
		void inputParameters();
		void collectScreensManually();
		void readXML();
		void writeXML();
		void calibrate();
		void calibrate(CameraParameters leftParam,CameraParameters rightParam);
		void rectify();
		void testRectification();
		void generate3d();
		void TrackBarBM();
		void TrackBarBGM();
		void test();
		//~ void mouseCallback(int event, int x, int y, int flags);
		static void callBack(int event, int x, int y, int flags, void* param);
		static void minDisparitiesCallback(int values,void*param)
		{
			cv::StereoSGBM* temp = (cv::StereoSGBM*)param;
			temp->minDisparity = values - 100 ;
		}
		
		static void minDisparitiesCallbackBM(int values,void*param)
		{
			cv::StereoBM* temp = (cv::StereoBM*)param;
			temp->state->minDisparity = values - 100 ;
		}
		static void numDisparitiesCallback(int values,void *param)
		{
			cv::StereoSGBM* temp = (cv::StereoSGBM*)param;
			temp->numberOfDisparities = (int) ((int)values/16) * 16 ;
		}
		
		static void numDisparitiesCallbackBM(int values,void *param)
		{
			cv::StereoBM* temp = (cv::StereoBM*)param;
			temp->state->numberOfDisparities = (int) ((int)values/16) * 16 ;
		}
		
		void setFileName(std::string fileName)
		{
			this->fileName = fileName ;
		}
		
		static void SADcallback(int values,void*param)
		{
			cv::StereoBM* temp = (cv::StereoBM*)param;
			if((values%2) == 0)
			{
				values--;
			}
			if(values < 5)
			{
				values = 5;
			}
			
			temp->state->SADWindowSize = values;
		}
	    void mouseEvent(int event, int x,int y, int flags, void* param);
	    void averageValueCalculator(cv::Mat pointCloud);
	    void setAverages(cv::Mat frame,int pointxup, int pointyup, int pointxdown, int pointydown);
		void markTheCenter(cv::Mat frame, cv::Mat Q);
};

#endif

