#include <pcl/visualization/cloud_viewer.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_cloud.h>
#include <pcl/io/ply_io.h>
 
int main(int argc, char** argv)
{
  
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>);
   
    if(argc != 2)
    {
    std::cout << "Usage: " << argv[0] << "  Filename(.ply)" << std::endl;
    return EXIT_FAILURE;
    }
   
   	//~ std::string ss1 = "screenshot" + boost::lexical_cast<std::string>(0) + ".pcd";
   	 std::string ss1 = argv[1];
/*
	//~ if (pcl::io::loadPCDFile(ss1, *cloud) == -1) //* load the file
	//~ {
		//~ std::cout << ("Couldn't read pcd file \n") << std::endl;
	//~ }
	 if(pcl::io::loadPLYFile<pcl::PointXYZ> (ss1, *cloud) == -1)
     {
        std::cout << "ERROR : couldn't find file" << std::endl;
        exit(0);
     } 
	else
	{
		std::cout << "Loaded "
            << cloud->width * cloud->height
            << " data points from test_pcd.pcd with the following fields: "
            << std::endl;
		
		//~ pcl::visualization::CloudViewer viewer ("Simple Cloud Viewer");
		//~ viewer.showCloud (cloud);
		//~ while (!viewer.wasStopped ())
		//~ {
		//~ }
	}
	*/
	
	
		boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer (new pcl::visualization::PCLVisualizer ("3D Viewer"));
		viewer->addModelFromPLYFile(ss1.c_str());
		//~ viewer.showCloud (cloud);
		while(1){
		viewer->spinOnce( 100 );}
	
}
