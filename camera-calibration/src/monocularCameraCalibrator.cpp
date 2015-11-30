#include "monocularCameraCalibrator.h"

/**
 @brief Constructor
 @param camerIndicator [int] number of camera device 
 (default camera device 0)
 @return void
*/

monocularcameraCalibrator::monocularcameraCalibrator(int cameraIndicator)
{
    std::cout<<"MonocularcameraCalubrator instance created"<<std::endl;
    frameHeight=480;
    frameWidth=640;
    
    //!< indicate whether the complete board was foundor not (=0)
    foundFrameIndicator=0;

    CameraCapture=new cv::VideoCapture(cameraIndicator);
    if(!CameraCapture->isOpened())
    {
       std::cout<<"Selected camera is not initialized"<<std::endl;
    }
    
    frame=cv::Mat::zeros(frameHeight,frameWidth,CV_8UC1);
    calibratedframe=cv::Mat::zeros(frameHeight,frameWidth,CV_8UC1);
    greyframe=cv::Mat::zeros(frameHeight,frameWidth,CV_8UC1);
    
    CameraCapture->set(CV_CAP_PROP_FRAME_WIDTH, 640);
    CameraCapture->set(CV_CAP_PROP_FRAME_HEIGHT,480);
    
    cameraMatrix[0] = cv::Mat::eye(3, 3, CV_64F);
    distCoeffs[0] = cv::Mat::zeros(8, 1, CV_64F);
    calibrationflag=false;
 
}

/**
 @brief Function that takes frames from each camera
 @return void
*/

void monocularcameraCalibrator::createSnapshot()
{
   
  //!< Retrieve data(shnapshot) from camera
  CameraCapture->grab();
  CameraCapture->retrieve(frame);
  //!<Turn image into gray
  cv::cvtColor(frame, greyframe, CV_RGB2GRAY);
}

/**
 @brief Function that refreshes frames
 @return void
*/

void monocularcameraCalibrator::refreshWindow()
{
      cv::imshow("snapshot",frame);
}

/**
 @brief Function that detects and draws corners either in chessboard 
 or in circleboard
 @param patternchoice [int], type of pattern we want to use, 1 for 
 chessboard and 2 for circleboard
 @param patternsize [cv::Size] size of chessboard
 @param circlepatternsize [cv::Size] size of circleboard
 @return void
*/

void monocularcameraCalibrator::detectChessboard(int patternchoice,cv::Size patternsize,cv::Size circlepatternsize)
{
  int patternflag=2;
  
  if(patternchoice==1)
  {
    //!<Find and draw corners for the camera's snapshot
    patternflag=cv::findChessboardCorners(greyframe,patternsize,corners,
         cv::CALIB_CB_ADAPTIVE_THRESH+cv::CALIB_CB_NORMALIZE_IMAGE/*+CALIB_CB_FAST_CHECK*/);
         
    //!< Improve the found corners' coordinate accuracy for chessboard
    if(patternflag==0 && (corners.size()!=0))
    {
        cv::cornerSubPix(greyframe,corners, cv::Size(11, 11), cv::Size(-1, -1),
                     cv::TermCriteria(CV_TERMCRIT_EPS + CV_TERMCRIT_ITER, 30, 0.1));
    }
    //!< Draw the corners
    cv::drawChessboardCorners(frame,patternsize, cv::Mat(corners),patternflag);
  }
  else if(patternchoice==2)
  {
    //!<Find and draw corners for the camera's snapshot
    patternflag= cv::findCirclesGrid(greyframe,circlepatternsize,centers,
              cv::CALIB_CB_ASYMMETRIC_GRID/*+CALIB_CB_CLUSTERING*/);
    //!< Draw the corners
     cv::drawChessboardCorners(frame,circlepatternsize, cv::Mat(centers),patternflag);
  }
  else
  {
      std::cout<<"There is not such a pattern choice"<<std::endl;
  }
}

/**
 @brief In this function we set object kai image points. Object points 
 refer to the real 3d coordinates of pattern points in the calibration 
 pattern, whereas image points are 2d cordinates and refer to the 
 projections of calibration pattern points
 @param patternchoice [int], type of pattern we want to use, 1 for 
 chessboard and 2 for circleboard
 @param patternsize [cv::Size] size of chessboard
 @param circlepatternsize [cv::Size] size of circleboard
 @param squareSize [int] size of each squre of chessborad
 @param circleSize [int] raduis of each circle in circleboard
 @return void
*/

void monocularcameraCalibrator::setPoints(int patternchoice,cv::Size patternsize,cv::Size circlepatternsize,int squareSize,int circleSize)
{
    //!< Set object points
    std::vector<cv::Point3f> obj;
    if(patternchoice==1)
    {
        for( int j = 0; j < patternsize.height; j++ )
        {
          for( int k = 0; k < patternsize.width; k++ )
          {
           obj.push_back(cv::Point3f(float(j*squareSize), float(k*squareSize), 0));
          }
        }
    }
    else
    {
        for( int i = 0; i < circlepatternsize.height; i++ )
        {
            for( int j = 0; j < circlepatternsize.width; j++ )
            {
            obj.push_back(cv::Point3f(float((2*j + i % 2)*circleSize), float(i*circleSize), 0));
            }
        }
    }
    objectPoints.push_back(obj);
   
   //!< Set imagePoints
   if(patternchoice==1)
    {
        imagePoints.push_back(corners);
    }
    else
    {
       imagePoints.push_back(centers);
    }
    std::cout<<"Object and Image Points are stored"<<std::endl;
}

/**
 @brief Function that calculates camerMatrix, distortion coeefficients
 and RMS error
 @return void
*/

void monocularcameraCalibrator::calibratefunction()
{
  cv::Size imageSize=greyframe.size();
	std::vector<cv::Mat> rvecs;
	std::vector<cv::Mat> tvecs;
  double cameraReprojectionError;
  std::cout<<"Camera Calibration begins!"<<std::endl;

  cameraReprojectionError = cv::calibrateCamera(objectPoints, imagePoints, imageSize, cameraMatrix[0],
          distCoeffs[0], rvecs, tvecs,CV_CALIB_FIX_K4| CV_CALIB_FIX_K5);
  std::cout<<"Calibration for left camera ended! RMS error:" <<cameraReprojectionError<<std::endl;
  
  calibrationflag=true;
  std::cout<<"Camera is calibrated"<<std::endl;
  
  this->savedata();
  this->undistort();
}

/**
 @brief Function that computes the undistortion and rectification 
 transformation map and applies a generic geometrical transformation 
 to an image.
 @return void
*/

void monocularcameraCalibrator::undistort()
{
    cv::Size imageSize=greyframe.size();
    cv::Mat map1, map2;
    cv::initUndistortRectifyMap(cameraMatrix[0], distCoeffs[0], cv::Mat(),
        cv::getOptimalNewCameraMatrix(cameraMatrix[0], distCoeffs[0], imageSize, 1, imageSize, 0),imageSize, CV_16SC2, map1, map2);
    cv::remap(greyframe, calibratedframe, map1, map2, cv::INTER_LINEAR);
}

/**
 @brief Function that saves calibration data in a file named
 intrinsics_eye.yml
 @return void
*/
void monocularcameraCalibrator::savedata()
{
	cv::FileStorage fs("intrinsics_eye.yml", CV_STORAGE_WRITE);
	if( fs.isOpened() )
	{
		fs << "cameraMatrix" << cameraMatrix[0] << "distortionCoefficientMatrix" << distCoeffs[0];
		fs.release();
	}
	else
		std::cout << "Error: can not save the intrinsic parameters"<<std::endl;
}

/**
 @brief Function that calibrates one camera. First function 
 setpoints() is called several times (approximately 20-30 times)
 and afterwards calibratefunction() to calculate and save all camera's
 intrinsics parameters.
 @param patternchoice [int], type of pattern we want to use, 1 for 
 chessboard and 2 for circleboard
 @param patternsize [cv::Size] size of chessboard
 @param circlepatternsize [cv::Size] size of circleboard
 @param squareSize [int] size of each squre of chessborad
 @param circleSize [int] raduis of each circle in circleboard
 @return void
*/
void monocularcameraCalibrator:: monocularCalibrate(int patternchoice,cv::Size patternsize,cv::Size circlepatternsize,int squareSize,int circleSize)
{
	int KeyPressed=255; 
	std::cout<<"Press esc if you want to stop the process"<<std::endl;
	std::cout<<"Press p to take another snapshot"<<std::endl;
	std::cout<<"Press s to calibrate camera"<<std::endl;
	while(1)
    {	
      createSnapshot();
      detectChessboard(patternchoice,patternsize,circlepatternsize);
      refreshWindow();
              
      KeyPressed=cvWaitKey(10) & 255;
      //!< different choices
      if(KeyPressed==27) //!< KeyPressed==esc
      {
         break;
      }
      if (KeyPressed==112) //!< KeyPressed==p
      {
        setPoints(patternchoice,patternsize,circlepatternsize,squareSize,circleSize);
      }
      if(KeyPressed==115) //!< KeyPressed==s
      {
         calibratefunction();
         cv::imshow("calibrated snapshot",greyframe);
      }
       
     }
}

//!< Destructor
monocularcameraCalibrator::~monocularcameraCalibrator()
{
	std::cout<<"monocularcameraCalibrator instance deleted"<<std::endl;
}


