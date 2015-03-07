#ifndef BACKGROUND_SUBTRACTION_H
#define  BACKGROUND_SUBTRACTION_H

#include <iostream>

#include "opencv2/highgui/highgui.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/core/core.hpp"

#include <pcl/common/common_headers.h>
#include <pcl/ModelCoefficients.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/visualization/cloud_viewer.h>


/**
@class BackgroundSubtraction
@brief The main background removal class. Contains the main functionalities of removing the background of an image.
**/

class BackgroundSubtraction{
	
	public:
		//!< Variable used to threshold the image
		int thresh;
		//!< Variable used to threshold the image
		int maxThresh;
		//!< OpenCV item used to create the bounding box
		cv::RNG rng;
		
		//!< Erosion OpenCV function, parameter
		int erosion_elem;
		//!< Erosion OpenCV function, parameter
		int erosion_size;
		//!< Erosion OpenCV function, parameter
		int max_elem;
		//!< Erosion OpenCV function, parameter
		int max_kernel_size;
	
	public:
		//!< Constructor
		BackgroundSubtraction();
		//!<Destructor
		~BackgroundSubtraction(){};
		
		/**
		@brief Returns the private variable rgb_
		@return void
		**/
		cv::Mat getRgb(){ return rgb_; }
		
		/**
		@brief Returns the private variable depth_
		@return void
		**/
		cv::Mat getDepth(){ return depth_; }
		
		/**
		@brief Returns the private variable pclMat_
		@return void
		**/
		cv::Mat getPclMat(){ return pclMat_; }
		
		/**
		@brief Returns the private variable pcd_
		@return void
		**/
		pcl::PointCloud<pcl::PointXYZ>::Ptr getPclPcd(){ return pcd_; }
		
		/**
		@brief Returns the private variable imgThresholded_
		@return void
		**/
		cv::Mat getImgThresholded(){ return imgThresholded_; }
		
		/**
		@brief Returns the private variable srcGray_
		@return void
		**/
		cv::Mat getSrcGray(){ return srcGray_; }
		
		/**
		@brief Set the private variable rgb_
		@return void
		**/
		void setRgb( cv::Mat y ){ rgb_ = y; }
		
		/**
		@brief Sets the private variable depth_
		@return void
		**/
		void setDepth( cv::Mat y ){ depth_ = y; }
		
		/**
		@brief Sets the private variable depth_
		@return void
		**/
		void setPclMat( cv::Mat y ){ pclMat_ = y; }
		
		/**
		@brief Sets the private variable depth_
		@return void
		**/
		void setPclPcd( pcl::PointCloud<pcl::PointXYZ>::Ptr pcd ){ pcd_ = pcd; }
		
		/**
		@brief Sets the private variable imgThresholded_
		@return void
		**/
		void setImgThresholded( cv::Mat y ){ imgThresholded_ = y; }
		
		/**
		@brief Sets the private variable srcGray_
		@return void
		**/
		void setSrcGray( cv::Mat y ){ srcGray_ = y; }
		
		/**
		@brief Removes the background of an image
			 * First reduces the region of interest using depth map
			 * After that, isolates the item by use the rgb values
		@return void
		**/
		void removeBackGround();
		
		/**
		@brief Creates the trackbar with threshold value
		@param img [cv::Mat] : The input gray scale image
		@return void
		**/
		void thresholdImg( cv::Mat img );
		
		/**
		@brief This function calculate the bounding boxes in the given image
		@return void
		**/
		void threshCallback( int, void* );
		
		/**
		@brief Function: Erosion opencv function 
		@return void
		**/
		void erosion( int, void* );
		
		
		
	private:
		//!< Private variables of rgb and depth input images
		cv::Mat rgb_, depth_;
		//!< Private variable of thresholded image
		cv::Mat imgThresholded_;
		//!< Private variable of the gray image used in callback function
		cv::Mat srcGray_;
		//!< Private variable of the pclMat cloud
		cv::Mat pclMat_;
		//!< Private variable pcd
		pcl::PointCloud<pcl::PointXYZ>::Ptr pcd_;
		
};

#endif
