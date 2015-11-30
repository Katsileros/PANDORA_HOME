#ifndef MONOCULAR_CAMERA.H
#define MONOCULAR_CAMERA.H

#include "Utilities.h"


class MonocularCamera
{
	int cameraID;
	std::string fileName;
	std::vector< std::vector<cv::Point2f> > imagePoints;
	std::vector< std::vector<cv::Point3f> > objectPoints;
	std::vector< cv::Mat > frames;
	CameraParameters parameters;
	CalibrationParameters calibParameters;
	
	
	public :
	
	MonocularCamera(){} ;
	MonocularCamera(int id);
	MonocularCamera(int id,std::string fileName);
	void calibrate();
	void calibrate(std::string fileName);
	void collectScreensManually();
	void autoCollectScreens();
	void generate3dObjectPoints();
	void readXML();
	void writeXML(); 
	void InputParameters();
	void pushPattern(std::vector<cv::Point2f> pattern);
	void pushScreen(cv::Mat img);
	void saveScreens();
	
	void clearPoints()
	{
		imagePoints.clear();
		objectPoints.clear();
	}
	
	void setFileName(std::string fileName)
	{
		this->fileName = fileName ;
	}
	std::string getFileName()
	{
		return this->fileName;
	}
	int getCameraID()
	{
		return this->cameraID;
	}
	void setCameraID(int id)
	{
		this->cameraID = id ;
	}
	
	
	CameraParameters getCameraParameters()
	{
		return this->parameters ; 
	}
	
	void setCalibrationParameters(CalibrationParameters param)
	{
		calibParameters = param;
	}
	
	
	std::vector< std::vector<cv::Point2f> > getImagePoints()
	{
		return imagePoints;
	}
	std::vector< std::vector<cv::Point3f> > getObjectPoints()
	{
		return objectPoints;
	}
	
	int getNumPoints()
	{
		return imagePoints.size();
	}
	
	
	};


#endif
