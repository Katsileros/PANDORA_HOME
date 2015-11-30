#include "stereoCameraCalibrator.h"

/**
 @brief Constructor, initializes left and right camera device id
 @return void
*/
stereoCameraCalibrator::stereoCameraCalibrator(int device1, int device2)
{
	leftCamera = new  monocularcameraCalibrator(device1);
	rightCamera = new  monocularcameraCalibrator(device2);

  stereocalibrationflag=false;
}

//!<Destructor
stereoCameraCalibrator::~stereoCameraCalibrator()
{
  std::cout<<"stereo camera calibrator instance deleted"<<std::endl;
}

/**
 @brief Function that takes frames from each camera and saves it
 to be processed
 @return void
*/
  
void stereoCameraCalibrator::createSnapshot()
{
  //!< Retrieve data(shnapshot) from camera
  leftCamera->CameraCapture->grab();
  rightCamera->CameraCapture->grab();
  leftCamera->CameraCapture->retrieve(leftCamera->frame);
  rightCamera->CameraCapture->retrieve(rightCamera->frame);
  
  //!< Turn image into gray
  cvtColor(leftCamera->frame, leftCamera->greyframe, CV_RGB2GRAY);
  cvtColor(rightCamera->frame, rightCamera->greyframe, CV_RGB2GRAY);
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
void stereoCameraCalibrator::stereodetectChessboard(int patternchoice,cv::Size patternsize,cv::Size circlepatternsize)
{
  int patternflagleft=2;
  int patternflagright=2;

  if(patternchoice==1)
  {
    //!< Find and draw corners for the camera's snapshot
    patternflagleft=cv::findChessboardCorners(leftCamera->greyframe,patternsize,leftCamera->corners,
           cv::CALIB_CB_ADAPTIVE_THRESH+cv::CALIB_CB_NORMALIZE_IMAGE/*+CALIB_CB_FAST_CHECK*/);
    patternflagleft=findChessboardCorners(rightCamera->greyframe,patternsize,rightCamera->corners,
           cv::CALIB_CB_ADAPTIVE_THRESH+cv::CALIB_CB_NORMALIZE_IMAGE/*+CALIB_CB_FAST_CHECK*/);
    //!< Improve the found corners' coordinate accuracy for chessboard
    if( patternflagleft==0 && patternflagright==0 &&(leftCamera->corners.size()!=0)&&(rightCamera->corners.size()!=0))
    {
        cornerSubPix(leftCamera->greyframe,leftCamera->corners, cv::Size(11, 11), cv::Size(-1, -1),
                     cv::TermCriteria(CV_TERMCRIT_EPS + CV_TERMCRIT_ITER, 30, 0.1));
        cornerSubPix(rightCamera->greyframe,rightCamera->corners, cv::Size(11, 11), cv::Size(-1, -1),
                     cv::TermCriteria(CV_TERMCRIT_EPS + CV_TERMCRIT_ITER, 30, 0.1));
    }
    //!< Draw the corners
    cv::drawChessboardCorners(leftCamera->frame,patternsize, cv::Mat(leftCamera->corners), patternflagleft);
       cv::drawChessboardCorners(rightCamera->frame,patternsize, cv::Mat(rightCamera->corners), patternflagright);
  }
  else if(patternchoice==2)
  {
    //!< Find and draw corners for the left camera's snapshot
    patternflagleft= cv::findCirclesGrid(leftCamera->greyframe,circlepatternsize,leftCamera->centers,
                cv::CALIB_CB_ASYMMETRIC_GRID/*+CALIB_CB_CLUSTERING*/);
    patternflagright= cv::findCirclesGrid(rightCamera->greyframe,circlepatternsize,rightCamera->centers,
                cv::CALIB_CB_ASYMMETRIC_GRID/*+CALIB_CB_CLUSTERING*/);
    //!< Draw the corners
    cv::drawChessboardCorners(leftCamera->frame,circlepatternsize,cv::Mat(leftCamera->centers),patternflagleft);
    cv::drawChessboardCorners(rightCamera->frame,circlepatternsize,cv::Mat(rightCamera->centers),patternflagright);
  }
  else
  {
      std::cout<<"There is not such a pattern choise"<<std::endl;
  }
}

/**
 @brief Function that calculates camerMatrix, distortion coeefficients
 for left and right camera, fundamental matrix and all the other
 extrinsics parameters
 and RMS error
 @return void
*/
void stereoCameraCalibrator::stereoCalibratefunction()
{
  std::vector<cv::Mat> rvecs;
	std::vector<cv::Mat> tvecs;
	cv::Size imageSize=leftCamera->greyframe.size();
  double cameraReprojectionError;
  std::cout<<"Calibration for stereo begins!"<<std::endl;
  cameraReprojectionError = cv::stereoCalibrate(leftCamera->objectPoints, stereoimagePoints[0], stereoimagePoints[1],
    leftCamera->cameraMatrix[0],leftCamera->distCoeffs[0],rightCamera->cameraMatrix[0],rightCamera->distCoeffs[0],
    imageSize, R, T, E, F,cv::TermCriteria(CV_TERMCRIT_ITER+CV_TERMCRIT_EPS, 100, 1e-5),CV_CALIB_FIX_INTRINSIC);
  
  cv::FileStorage filestream("fundamental_essential.yml", CV_STORAGE_WRITE);
	if(filestream.isOpened() )
	{
		filestream << "R" << R << "T" << T << "E"<< E <<"F" << F;
		filestream.release();
	}
	else
	{
		std::cout << "Error: can not save the fundamental and essential matrix"<<std::endl;
  }
	stereocalibrationflag=true;
  std::cout<<"Calibration for stereo ended! RMS error:" <<cameraReprojectionError<<std::endl;
  this->savedata();
  this->undistort();
}

/**
@brief Function that saves calibration data in a file named
extrinsics_eye.yml
@return void
*/

void stereoCameraCalibrator::savedata()
{
  cv::Size imageSize=leftCamera->greyframe.size();
	cv::FileStorage fs("extrinsics_eye.yml", CV_STORAGE_WRITE);
	cv::Mat R1, R2, P1, P2, Q;
	cv::Rect validRoi[2];
	cv::stereoRectify(leftCamera->cameraMatrix[0],leftCamera->distCoeffs[0],
					rightCamera->cameraMatrix[0],rightCamera->distCoeffs[0],
			imageSize, R, T, R1, R2, P1, P2, Q,
			cv::CALIB_ZERO_DISPARITY, 1, imageSize, &validRoi[0], &validRoi[1]);
	if( fs.isOpened() )
	{
		fs << "R" << R << "T" << T << "R1" << R1 << "R2" << R2 << "P1" << P1 << "P2" << P2 << "Q" << Q;
		fs.release();
	}
	else
	{
		std::cout << "Error: can not save the extrinsic parameters"<<std::endl;
    }
	std::cout << "Saved parameters!"<<std::endl;
}

/**
 @brief Function that refreshes frames
 @return void
*/
void stereoCameraCalibrator::refreshWindow()
{
  cv::imshow( "left snapshot", leftCamera->frame );
	cv::imshow( "right snapshot", rightCamera->frame);
}

/**
@brief Function that computes the undistortion and rectification 
transformation map and applies a generic geometrical transformation 
to an image.
@return void
*/
void stereoCameraCalibrator::undistort()
{
  cv::Mat R1, R2, P1, P2, Q;
  cv::Size imageSize=leftCamera->greyframe.size();
	cv::Rect roi1, roi2;

	cv::stereoRectify( leftCamera->cameraMatrix[0],leftCamera-> distCoeffs[0],
               rightCamera-> cameraMatrix[0],rightCamera-> distCoeffs[0], imageSize, R, T, R1, R2, P1, P2, Q,
	cv::CALIB_ZERO_DISPARITY, -1, imageSize, &roi1, &roi2 );

	
  //!<The initUndistortRectifyMap computes the undistortion and rectification transformation map
  //!< returns rmap to be used in remap function
	cv::initUndistortRectifyMap(leftCamera->cameraMatrix[0],leftCamera-> distCoeffs[0], R1, P1, imageSize, CV_16SC2, rmap[0][0], rmap[0][1]);
	cv::initUndistortRectifyMap(rightCamera-> cameraMatrix[0],rightCamera-> distCoeffs[0], R2, P2, imageSize, CV_16SC2, rmap[1][0], rmap[1][1]);
  cv::remap(leftCamera->frame, leftCamera->calibratedframe, rmap[0][0], rmap[0][1], cv::INTER_LINEAR);
  cv::remap(rightCamera->frame,rightCamera->calibratedframe, rmap[1][0], rmap[1][1], cv::INTER_LINEAR);
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
void stereoCameraCalibrator::stereosetPoints(int patternchoice,cv::Size patternsize,cv::Size circlepatternsize,int squareSize,int circleSize)
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
  //!< ObjectPoints is the same for both cameras
  stereoobjectPoints.push_back(obj);
  //!< Set imagePoints
  if(patternchoice==1)
  {
      stereoimagePoints[0].push_back(leftCamera->corners);
      stereoimagePoints[1].push_back(rightCamera->corners);
  }
  else
  {
     stereoimagePoints[0].push_back(leftCamera->centers);
     stereoimagePoints[1].push_back(rightCamera->centers);
  }
  std::cout<<"Object and Image Points are stored"<<std::endl;
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
void stereoCameraCalibrator:: stereoCalibrate(int patternchoice,cv::Size patternsize,cv::Size circlepatternsize,int squareSize,int circleSize)
{
	int KeyPressed=255; 
	std::cout<<"Press esc if you want to stop the process"<<std::endl;
	std::cout<<"Press p to take another snapshot"<<std::endl;
	std::cout<<"Press s to calibrate camera"<<std::endl;
	while(1)
    {	
      createSnapshot();
      stereodetectChessboard(patternchoice,patternsize,circlepatternsize);
      refreshWindow();
              
      KeyPressed=cvWaitKey(10) & 255;
      //!< different choices
      if(KeyPressed==27) //!< KeyPressed==esc
      {
         break;
      }
      if (KeyPressed==112) //!< KeyPressed==p
      {
        stereosetPoints(patternchoice,patternsize,circlepatternsize,squareSize,circleSize);
      }
      if(KeyPressed==115) //!< KeyPressed==s
      {
         stereoCalibratefunction();
         cv::imshow("left calibrated snapshot",leftCamera->greyframe);
         cv::imshow("right calibrated snapshot",rightCamera->greyframe);
      }
       
     }
}
