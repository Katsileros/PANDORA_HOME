#include <pcl/point_types.h>
#include <pcl/features/vfh.h>
#include <pcl/features/normal_3d.h>
#include <pcl/common/centroid.h>
#include <pcl/point_cloud.h>
#include <pcl/features/crh.h>
#include <pcl/io/io.h>
#include <pcl/io/pcd_io.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/visualization/histogram_visualizer.h>
#include <pcl/console/parse.h>
#include <pcl/console/print.h>
#include <pcl/io/ply_io.h>

#include <flann/flann.h>
#include <flann/io/hdf5.h>

#include <boost/filesystem.hpp>
#include <iostream>
#include <fstream>

/*! 
 *  \brief     This class creates VFH features
 *  \author    Katsileros Petros
 *  \date      21/3/2015
 *  \bug       Must define the input-output read & write folders and files
 *  \copyright GNU Public License.
 */

/**
@class vfh_features
@brief Calculates the VFH descriptor for an object 
**/

class vfh_features{
	
	private:
		//!< The vfh descriptor variable
		pcl::PointCloud<pcl::VFHSignature308>::Ptr vfhs_;
		//!< The input point cloud
		pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_ ;
  		//!< The normals of the given input
  		pcl::PointCloud<pcl::Normal>::Ptr normals_;
  		//!< Number of training data
  		int num_;
	
	public:
		/**
		@brief Constructor
		@param inputFolder: Contains the pcd training data files
		@param num: Number of pcd files, goind to be read
		**/
		vfh_features(int num);
		
		/**
		@brief Destructor
		**/
		~vfh_features();
		
		/**
		@brief Calculate the vfh descriptor of the given object point cloud
		**/
		void vfh_compute();
		
		/**
		@brief Returns the private variable vfhs_
		**/
		pcl::PointCloud<pcl::VFHSignature308>::Ptr getVFH(){return vfhs_;};
		
		/**
		@ Read the point clouds from the input folder. Caclulate the normals and
		* the VFH descriptors using vfh_compute() func and saves the models.
		* Then saves the training data to files. 
		* Finally creates the kdtree index for the models.
		@param folder: The output save folder for the pcd VFH models
		@param filenam: The path of the ply files.
		**/
		void buildTree(std::string folder, std::string filename);
};
