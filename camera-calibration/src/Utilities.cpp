#include "Utilities.h"

int Utilities::getLeftCameraID()
{
	cv::VideoCapture camera(0);
	if ( !camera.isOpened() )  
	{
	    std::cout << "Cannot open the video file" << std::endl;
	    return -1;
	}	
	cv::Mat frame;
	std::string windowName("Video Input");
	cv::namedWindow(windowName,true);
	char key;
	std::cout << "Is this the left camera ? [Y/N] " << std::endl;
	
	while (1)
	{
		camera.grab();
		camera.retrieve(frame);
		
		cv::imshow(windowName,frame);
		key = cv::waitKey(30) & 255 ;	
		switch(key)
		{
			case 'y':
			case 'Y':
				return 0;
			case 'N':
			case 'n':
				return 1;
			default:
				exit(-1);
		}
		
		
	}	

}

CameraParameters Utilities::readXML(std::string name)
{
	cv::FileStorage fs(name,cv::FileStorage::READ);
	cv::Mat parameters;
	cv::Mat distortCoeff;
	fs["Camera_Matrix"] >> parameters;
	fs["Distortion_Coefficients"] >> distortCoeff;
	
	CameraParameters cam;
	cam.cameraMatrix = parameters ;
	cam.distortion = distortCoeff ;
	fs.release();
	return cam;
}

void Utilities::writeXML(std::string name,CameraParameters param)
{
	cv::FileStorage fs(name,cv::FileStorage::WRITE);
	fs << "Camera_Matrix" << param.cameraMatrix ;
	fs << "Distortion_Coefficients" << param.distortion ;
	fs.release();
}

/*
int Utilities::countCameras()
{
	cv::VideoCapture cam;
	for ( int i = 0 ; i < MAX_CAMERA_NUMBER ; i++ )
	{
		cam.open(i);
		if (!cam.isOpened())
		{
			return i;
		}
	}
	return MAX_CAMERA_NUMBER;
} */
