#include <iostream>
#include "opencv/cv.h"
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/core/core.hpp"

#include <boost/lexical_cast.hpp>
#include <vector>


/**
@class poseEstimation
@brief Calculates the pose of an object next to a chessboard, using opencv::solvePnP function. 
**/
class poseEstimation{

	private:
		//!< This vector will be filled by detected corners
		std::vector<cv::Point2f> corners_;
		//!< Real 3d coordinates
		std::vector<cv::Point3f> boardPoints_;
		
		//!< Rotation vector[3x1], output of solvePnP function
		cv::Mat rVec_;
		//!< Translation vector[3x1], output of solvePnP function
		cv::Mat tVec_;
		//!< Rodrigues [3x3] matrix
		cv::Mat R_;
		
		//!< Mat[4x4] of the final pose of the object, pose = [ R t; 0 0 0 1];
		cv::Mat pose_;
		
		//!< Mat[2x1] contains the coordinates (x,y) of the object, located in the image plane.
		cv::Mat objectLoc_;
		
		//!< Size of the chessboard pattern
		cv::Size2i patternSize_;
		//!< Image height
		int frameHeight_;
		//!< Image width
		int frameWidth_;
		//!<  value of chessboard square size
		float squareSize_;
		
		//!<  Input image of an object next to the chessboard pattern
		cv::Mat input_;
		
		//!<  Mat[8x1] of the distortion coefficients of the camera
		cv::Mat distortionCoefficients_;
		//!<  Mat[3x3]: THE camera matrix
		cv::Mat cameraMatrix_;


	public:
		/**
		@brief Constructor
		@param inputFile contains the appropriate information about calibration parameters, pattern size etc...
		**/
		poseEstimation(std::string inputFile);
		
		/**
		@brief Destructor
		**/
		~poseEstimation();
		
		/**
		@brief Calculates the [4x4] pose matrix of a 
		* given image with a chessboard pattern inside
		@param k: The number of the given image
		**/
		void calcPose(int k);
		
		/**
		@brief creates the 3D points of your chessboard in its own coordinate system 
		**/
		void Create3DChessboardCorners();
		
		/**
		@brief Takes an input image, and detect the chessboard pattern. 
		**/
		void readImgaes(cv::Mat img1);
		
		/**
		@brief Clears vectors and mats used in the current itteration 
		**/
		void clearVecMat();
		
		/**
		@brief Write the calculated pose to the output xml file 
		**/
		void writePose2XML(cv::FileStorage fs, int k);
		
		/**
		@brief Returns the coordinates (x,y) of the object in the image plane; 
		**/
		cv::Mat getObjectLocation(){ return objectLoc_;};
};
