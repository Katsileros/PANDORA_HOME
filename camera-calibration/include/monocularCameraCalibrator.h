#ifndef MONOCULARCAMERACALIBRATOR_H
#define MONOCULARCAMERACALIBRATOR_H

#include "opencv/cv.h"
#include "opencv/highgui.h"
#include <iostream>
#include <vector>

class monocularcameraCalibrator
{
  public:
  
   int frameHeight;
   int frameWidth;
   
   cv::VideoCapture* CameraCapture;
   cv::Mat frame;
   cv::Mat greyframe;
   cv::Mat calibratedframe;
   
  //!< This vector will be filled by detected corners
  std::vector<cv::Point2f> corners;
  //!< This vector will be filled by detected centers
  std::vector<cv::Point2f> centers;
  //!< Indicate whether the complete board was foundor not (=0)
  int foundFrameIndicator;
  //!< Projection of calibration pattern points
  std::vector<std::vector<cv::Point2f> > imagePoints;
  //!< Real 3d coordinates
  std::vector<std::vector<cv::Point3f> > objectPoints;
  //!< Used to store cameraMatrix and distCoeffs
  cv::Mat cameraMatrix[1], distCoeffs[1];
  //!< Flag tha indicates whether a camera is calibrated or not
  bool calibrationflag;
  
  public:
  
  /**
   @brief Constructor
   @param camerIndicator [int] number of camera device 
   (default camera device 0)
   @return void
  */
  monocularcameraCalibrator(int cameraIndicator);
  
  //!<Destructor
  virtual ~monocularcameraCalibrator();
  
  /**
   @brief Function that takes frames from each camera
   @return void
  */
  void createSnapshot();
  
  /**
   @brief Function that refreshes frames
   @return void
  */
  void refreshWindow();
  
  /**
   @brief Function that detects and draws corners either in chessboard 
   or in circleboard
   @param patternchoice [int], type of pattern we want to use, 1 for 
   chessboard and 2 for circleboard
   @param patternsize [cv::Size] size of chessboard
   @param circlepatternsize [cv::Size] size of circleboard
   @return void
  */
  void detectChessboard(int patternchoise,cv::Size patternsize,cv::Size circlepatternsize);
  
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
  void setPoints(int patternchoise,cv::Size patternsize,cv::Size circlepatternsize,int squareSize,int circleSize);
  
  /**
   @brief Function that calculates camerMatrix, distortion coeefficients
   and RMS error
   @return void
  */
  void calibratefunction();
  
  /**
   @brief Function that computes the undistortion and rectification 
   transformation map and applies a generic geometrical transformation 
   to an image.
   @return void
  */
  void undistort();
  
  /**
   @brief Function that saves calibration data in a file named
   intrinsics_eye.yml
   @return void
  */
  void savedata();
  
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
  void monocularCalibrate(int patternchoise,cv::Size patternsize,cv::Size circlepatternsize,int squareSize,int circleSize);
};

#endif
