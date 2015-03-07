#include "poseEstimation.h"


/**
@brief Constructor. Reads the calibration and chessboard pattern parameters. 
@return void
@param none
**/
poseEstimation::poseEstimation(std::string inputFile)
{	
    cv::FileStorage fs(inputFile,cv::FileStorage::READ);
    if (!fs.isOpened())
    {
		std::cout << "Failed to open Parameters.xml." << std::endl;
    }
    
    fs["frameWidth"]			 >> frameWidth_;
    fs["frameHeight"]			 >> frameHeight_;
	fs["Camera_Matrix"] 		 >> cameraMatrix_;
	fs["squareSize"]    		 >> squareSize_;
	fs["patternSize_width"] 	 >> patternSize_.width;
	fs["patternSize_height"] 	 >> patternSize_.height;
	fs["distortionCoefficients"] >> distortionCoefficients_;
	fs.release(); 
	
    input_ = cv::Mat::zeros(frameHeight_,frameWidth_,CV_8UC1);
    objectLoc_ = cv::Mat::zeros(2,1,CV_64FC1);
    //~ pose_ = cv::Mat::zeros(4,4,CV_64FC1);
}

/**
@brief Reads an input image. Inside this image, detects the chessboard pattern
	 * Also draws the detected corners onto the pattern
@return void
@param cv::Mat The input images. Must include a chessboard pattern inside.
**/
void poseEstimation::readImgaes(cv::Mat img)
{
	input_ = img;
	
	cv::Mat grayframe = cv::Mat::zeros(frameHeight_,frameWidth_,CV_8UC1);
	int found;
	int KeyPressed;
	
	// Convert image to gray scale.
	cv::cvtColor( input_ , grayframe , CV_BGR2GRAY );
  
	//!<Find and draw corners for the camera's snapshot
    found = cv::findChessboardCorners(grayframe,patternSize_,corners_,
		 CV_CALIB_CB_ADAPTIVE_THRESH | CV_CALIB_CB_FAST_CHECK | CV_CALIB_CB_NORMALIZE_IMAGE);
         //~ cv::CALIB_CB_ADAPTIVE_THRESH+cv::CALIB_CB_NORMALIZE_IMAGE/*+CALIB_CB_FAST_CHECK*/);
         
    //!< Improve the found corners' coordinate accuracy for chessboard
    if(found==0 && (corners_.size()!=0))
    {
        cv::cornerSubPix(grayframe,corners_, cv::Size(11, 11), cv::Size(-1, -1),
                     cv::TermCriteria(CV_TERMCRIT_EPS + CV_TERMCRIT_ITER, 30, 0.1));
    }
    //!< Draw the corners
    cv::drawChessboardCorners(input_,patternSize_, cv::Mat(corners_),found);
    
    //~ cv::imshow("input image",input_);
    //~ std::cout << "Press any key." << std::endl;
    //~ cv::waitKey();
}

/**
@brief Find the pose of the image(input of readImages function)
	 * Creates a 3d coordinate system according to the pattern axes.
	 * Solves the PnP algorithm, and the result is the rVec and tVec.
@return void
@param none
**/
void poseEstimation::calcPose(int k)
{
  this->Create3DChessboardCorners();
  
  cv::solvePnPRansac(cv::Mat(boardPoints_).reshape(1), cv::Mat(corners_).reshape(1), cameraMatrix_, distortionCoefficients_, rVec_, tVec_); 
  //~ cv::solvePnP(cv::Mat(boardPoints_).reshape(1), cv::Mat(corners_).reshape(1), cameraMatrix_, distortionCoefficients_, rVec_, tVec_); 
  
  //~ std::cout << "tVec_: \n  " << tVec_ << std::endl;
  //~ std::cout << "rVec_: \n  " << rVec_ << std::endl;
  
  cv::Rodrigues(rVec_, R_);
  //~ std::cout << "R: \n" << R_ << std::endl;
  
  pose_ = cv::Mat::zeros(4,4,CV_64FC1);
  
  pose_(cv::Range(0, 3), cv::Range(0, 3)) = R_ * 1; // copies R_ into pose_
  pose_(cv::Range(0, 3), cv::Range(3, 4)) = tVec_ * 1; // copies tVec_ into pose_
  pose_.at<double>(3,0) = 0; pose_.at<double>(3,1) = 0; pose_.at<double>(3,2) = 0; pose_.at<double>(3,3) = 1;
  
  float X,Y,Z,col,row;
  float focalLengthX = 575;
  float focalLengthY = 575;
  double centerX = 320.5;
  double centerY = 240.5;
  
  //~ cv::circle(input_, cv::Point(centerX, centerY), 10, cv::Scalar(0,0,255), -1, 8);
  
  X = tVec_.at<double>(0,0);
  Y = tVec_.at<double>(1,0);
  Z = tVec_.at<double>(2,0);

  //~ std::cout << "(X,Y,Z) = " << "(" << X << "," << Y << "," << Z << ")" << std::endl;
  
  cv::Mat objLoc = cv::Mat(4, 1, CV_64F);
  objLoc.at<double>(0, 0) = 8*squareSize_;
  objLoc.at<double>(1, 0) = -2*squareSize_;
  objLoc.at<double>(2, 0) = 0;
  objLoc.at<double>(3, 0) = 1;

  objLoc = pose_ * objLoc;
  int objcol = (objLoc.at<double>(0, 0) * focalLengthX) / objLoc.at<double>(2,0) + centerX;
  int objrow = (objLoc.at<double>(1, 0) * focalLengthY) / objLoc.at<double>(2,0) + centerY;
  objectLoc_.at<double>(1,0) = objcol;
  objectLoc_.at<double>(0,0) = objrow;
  cv::circle(input_, cv::Point(objcol, objrow), 4, cv::Scalar(255,255,255), -1, 8);
  //~ std::cout << "objectLoc_: \n" << objectLoc_ << std::endl;
  
  col = ( ((X * focalLengthX) / Z ) +  centerX );
  row = ( ((Y * focalLengthY) / Z ) +  centerY );

  //~ std::cout << "(col,row) = " << "(" << col << "," << row << ")" << std::endl;
  cv::circle(input_, cv::Point(col, row), 4, cv::Scalar(255,0,0), -1, 8);
   
  cv::imshow("input image",input_);
  //~ std::cout << "pose: \n" << pose_ << std::endl;
  
  std::cout << "Press any key." << std::endl;
  cv::waitKey();
}

/**
@brief creates the 3D points of your chessboard in its own coordinate system
@return void
@param none
**/
void poseEstimation::Create3DChessboardCorners()
{
  int i=0, j=0;
 
  for( i = 0; i < patternSize_.height; i++ )
  {
    for( j = 0; j < patternSize_.width; j++ )
    {
      boardPoints_.push_back(cv::Point3f(float(i*squareSize_), float(j*squareSize_), 0));
    }
  }
 
}

void poseEstimation::writePose2XML(cv::FileStorage fs,int k)
{
	std::string ss = "pose" + boost::lexical_cast<std::string>(k);
	fs << ss << pose_ ;
}

void poseEstimation::clearVecMat()
{
  corners_.clear();
  boardPoints_.clear();
  rVec_.release();
  tVec_.release();
  R_.release();
  pose_.release();
  input_.release();
}
