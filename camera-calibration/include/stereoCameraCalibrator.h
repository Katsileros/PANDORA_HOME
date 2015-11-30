#ifndef STEREOCAMERACALIBRATOR_H
#define  STEREOCAMERACALIBRATOR_H
#include "opencv/cv.h"
#include "opencv/highgui.h"
#include <iostream>
#include "opencv2/calib3d/calib3d.hpp"
#include "monocularCameraCalibrator.h"

class stereoCameraCalibrator
{
  private:
  
  monocularcameraCalibrator *leftCamera;
  monocularcameraCalibrator *rightCamera;
  bool stereocalibrationflag;
  cv::Mat R, T, E, F;
	cv::Mat rmap[2][2];
	std::vector<std::vector<cv::Point2f> > stereoimagePoints[2];
	std::vector<std::vector<cv::Point3f> > stereoobjectPoints;
  
  public:
  
  //!< Constructor
  stereoCameraCalibrator(int device1, int device2);
  
  //!< Destructor
  virtual ~stereoCameraCalibrator();
  
  /**
    @brief Function that detects and draws corners either in chessboard 
    or in circleboard
    @param patternchoice [int], type of pattern we want to use, 1 for 
    chessboard and 2 for circleboard
    @param patternsize [cv::Size] size of chessboard
    @param circlepatternsize [cv::Size] size of circleboard
    @return void
  */
  void stereodetectChessboard(int patternchoise,cv::Size patternsize,cv::Size circlepatternsize);
  
  void stereosetPoints(int patternchoice,cv::Size patternsize,cv::Size circlepatternsize,int squareSize,int circleSize);
  
  /**
    @brief Function that calculates camerMatrix, distortion coeefficients
    for left and right camera, fundamental matrix and all the other
    extrinsics parameters
    and RMS error
    @return void
  */
  void stereoCalibratefunction();
  
  /**
    @brief Function that takes frames from each camera and saves it
    to be processed
    @return void
  */
  void createSnapshot();
  
  /**
    @brief Function that saves calibration data in a file named
    extrinsics_eye.yml
    @return void
  */
  void savedata();
  
  /**
    @brief Function that refreshes frames
    @return void
  */
  void refreshWindow();
  
  /**
    @brief Function that computes the undistortion and rectification 
    transformation map and applies a generic geometrical transformation 
    to an image.
    @return void
  */
  void undistort();
  
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
  void stereoCalibrate(int patternchoice,cv::Size patternsize,cv::Size circlepatternsize,int squareSize,int circleSize);

};

#endif
