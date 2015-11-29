#include <pcl/visualization/cloud_viewer.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_cloud.h>
#include <pcl/io/ply_io.h>
#include <pcl/common/transforms.h>

#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
 
void depthToPointCloud(cv::Mat rgb, cv::Mat depth)
{
    int focalLengthX = 525;
    int focalLengthY = 525;
    double centerX = 320;
    double centerY = 240;

    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>);

    cloud->width = rgb.cols;
    cloud->height = rgb.rows;
    cloud->points.resize (cloud->width * cloud->height);

    unsigned short depth_value;
    float x,y,z;
    for(int i=0;i<depth.rows;i++)
    {
        for(int j=0;j<depth.cols;j++)
        {
            depth_value = depth.at<unsigned short>(i,j);
            if(depth_value != 0)
            {
                //!< Z = depth(row, col) / 1000;
                //!< X = (col - centerX) * Z / focalLengthX;
                //!< Y = (row - centerY) * Z / focalLengthY;
                z = float(depth_value) / 1000.0f;
                x = (j - centerX) * z / focalLengthX;
                y = (i - centerY) * z / focalLengthY;

                cloud->points[i*rgb.rows+j].x = x;
                cloud->points[i*rgb.rows+j].y = y;
                cloud->points[i*rgb.rows+j].z = z;

                //~ //!< save rgb data
                //~ uint8_t b = rgb.at<cv::Vec3b>(i,j)[0];
                //~ uint8_t g = rgb.at<cv::Vec3b>(i,j)[1];
                //~ uint8_t r = rgb.at<cv::Vec3b>(i,j)[2];
                //~ cloud->points[i*rgb.rows+j].rgb = (r << 16 | g << 8 | b);
            }
        }
    }
    Eigen::Affine3f transform = Eigen::Affine3f::Identity();
	transform.rotate (Eigen::AngleAxisf (-180, Eigen::Vector3f::UnitX()));
	
	pcl::transformPointCloud (*cloud, *cloud, transform);
	
    pcl::io::savePCDFileASCII ("../driller/data/pcd/testingPointCloud0.pcd", *cloud);
}  
 
int main(int argc, char** argv)
{
    cv::Mat imageRGB = cv::imread("../driller/data/color0.jpg",CV_LOAD_IMAGE_COLOR);
    if(! imageRGB.data )                              // Check for invalid input
    {
        cout <<  "Could not open or find the image-rgb" << std::endl ;
        return -1;
    }
    //~ cv::imshow("imageRGB",imageRGB);
    //~ cv::waitKey();
    
    cv::Mat imageDepth = cv::imread("../driller/data/depth0.png",CV_LOAD_IMAGE_ANYDEPTH);
    if(! imageDepth.data )                              // Check for invalid input
    {
        cout <<  "Could not open or find the image-depth" << std::endl ;
        return -1;
    }
    //~ cv::imshow("imageDepth",imageDepth);
    //~ cv::waitKey();
    
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>);
   
    depthToPointCloud(imageRGB, imageDepth);
   
   	///////std::string ss1 = "clouds/NP1_" + boost::lexical_cast<std::string>(3) + ".pcd";
   	std::string ss1 = "../driller/data/pcd/testingPointCloud0.pcd";
	
	
	if (pcl::io::loadPCDFile(ss1, *cloud) == -1) //* load the file
	{
		std::cout << ("Couldn't read pcd file \n") << std::endl;
	}
	
	//~ Eigen::Affine3f transform = Eigen::Affine3f::Identity();
	//~ transform.rotate (Eigen::AngleAxisf (-180, Eigen::Vector3f::UnitX()));
	//~ 
	//~ pcl::transformPointCloud (*cloud, *cloud, transform);
	
	pcl::visualization::CloudViewer viewer ("Simple Cloud Viewer");
    viewer.showCloud (cloud);
    while (!viewer.wasStopped ())
    {
    }
		
		
}
