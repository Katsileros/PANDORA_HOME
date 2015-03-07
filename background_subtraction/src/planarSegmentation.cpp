#include "planarSegmentation.h"

planarSegmentation::planarSegmentation()
{
  //~ focalLengthX = 575;
  //~ focalLengthY = 575;
  //~ centerX = 320.5;
  //~ centerY = 240.5;
	
  cloud.reset(new pcl::PointCloud<pcl::PointXYZ>());
  cloud->width = 640;
  cloud->height =480;
  cloud->points.resize (cloud->width * cloud->height);
  
  inliers.reset(new pcl::PointIndices());

}

void planarSegmentation::fillPcd(cv::Mat depth_mat)
{ 
  unsigned short depth_value;
  float X,Y,Z;
  float focalLengthX = 575;
  float focalLengthY = 575;
  double centerX = 320.5;
  double centerY = 240.5;
  
  depth_ = depth_mat;
  
  for(int i=0;i<depth_mat.rows;i++)
  {  
     for(int j=0;j<depth_mat.cols;j++)
     {
		depth_value = depth_mat.at<unsigned short>(i,j);
		if(depth_value != 0){			//Alliws bgainei synexia to 0,0,0
			///Z = depth(row, col) / 1000;
			Z = float(depth_value)/1000.0f; 
			///X = (col - centerX) * Z / focalLengthX;
			X = ((j - centerX) * Z) / focalLengthX;
			///Y = (row - centerY) * Z / focalLengthY;
			Y = ((i - centerY) * Z) / focalLengthY;
		
			cloud->points[i*depth_mat.cols+j].x = X;
			cloud->points[i*depth_mat.cols+j].y = Y;
			cloud->points[i*depth_mat.cols+j].z = Z;
			//Enallaktika:
			//~ cloud->at(j,i) = pcl::PointXYZ(X,Y,Z);		
			//~ pntCld->push_back(pcl::PointXYZ (X,Y,Z));
			depth_.at<unsigned short>(i,j) = Z;
		}
     } 	
  }
}

void planarSegmentation::segment()
{
  ///Ax+By+Cz+D=0
  pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients);///A,B,C,D
  ///create the segmentation object
  pcl::SACSegmentation<pcl::PointXYZ> seg;
  ///optional
  seg.setOptimizeCoefficients(true);
  ///mandatory
  seg.setModelType(pcl::SACMODEL_PLANE);
  seg.setMethodType(pcl::SAC_RANSAC);
  seg.setDistanceThreshold (threshold_);
  
  seg.setInputCloud(cloud);
  seg.segment(*inliers, *coefficients);    
  
  std::cout<<"Model Coefficients: "<< coefficients->values[0] << " " 
                                   << coefficients->values[1] << " "
                                   << coefficients->values[2] << " " 
                                   << coefficients->values[3] << std::endl;
                                 
  std::cerr << "Model inliers: " << inliers->indices.size () << std::endl;
}

void planarSegmentation::visualize(cv::Mat rgb_image)
{
	int x,y,id;
	int focalLengthX = 575;
	int focalLengthY = 575;
	double centerX = 320.5;
	double centerY = 240.5;
	
	rgbSeg_ = rgb_image;
	
	for(int i=0;i<inliers->indices.size();i++)
	{	
		id = inliers->indices[i];
		if (cloud->points[id].z != 0)
		{
			x = int( centerX + focalLengthX*cloud->points[id].x/cloud->points[id].z + 0.5f);///x = cx + fx*X/Z
			y = int( centerY + focalLengthY*cloud->points[id].y/cloud->points[id].z + 0.5f);///y = cy + fx*Y/Z
			rgb_image.at<cv::Vec3b>(y,x)[0] = 0;
			rgb_image.at<cv::Vec3b>(y,x)[1] = 0;
			rgb_image.at<cv::Vec3b>(y,x)[2] = 0;
		}
	}
	
	rgbSeg_ = rgb_image;
	
	cv::imshow("Segmented",rgb_image);
	cv::waitKey();
	
}
