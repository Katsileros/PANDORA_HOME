#include <pcl/point_types.h>
#include <pcl/features/fpfh_omp.h>
#include <pcl/features/normal_3d_omp.h>
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
#include <pcl/filters/approximate_voxel_grid.h>
#include <pcl/keypoints/uniform_sampling.h>
#include <pcl/common/transforms.h>

#include <flann/flann.h>
#include <flann/io/hdf5.h>

#include <boost/filesystem.hpp>
#include <iostream>
#include <fstream>

/*! 
 *  \brief     This class creates PFH features
 *  \author    Katsileros Petros
 *  \date      5/5/2015
 *  \copyright GNU Public License.
 */

/**
@class pfh_features
@brief Calculates the PFH descriptor for an object 
**/

class pfh_features{
	
	private:
		//!< The pfh descriptor variable
		pcl::PointCloud<pcl::FPFHSignature33>::Ptr pfhs_;
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
		pfh_features(int num);
		
		/**
		@brief Destructor
		**/
		~pfh_features();
		
		/**
		@brief Calculate the vfh descriptor of the given object point cloud
		**/
		void pfh_compute();
		
		/**
		@brief Returns the private variable pfhs_
		**/
		pcl::PointCloud<pcl::FPFHSignature33>::Ptr getPFH(){return pfhs_;};
		
		/**
		@ Read the point clouds from the input folder. Caclulate the normals and
		* the PFH descriptors using vfh_compute() func and saves the models.
		* Then saves the training data to files. 
		* Finally creates the kdtree index for the models.
		@param folder: The output save folder for the pcd PFH models
		@param filenam: The path of the ply files.
		**/
		void buildTree(std::string folder, std::string filename);
		
		/**
		 * Mean value of the PFH features
		 **/ 
		void meanFPFHValue(pcl::PointCloud<pcl::FPFHSignature33>::Ptr meanFPFH);
		
		/**
		 * Take the input scene(input frame) and split it to several windows.
		 * For every window calculate the mean PFH-feature and save it
		 * in order to make the comparison with the training features
		@param Input: The overall point cloud of the input scene
		@param Input: The folder name
		@param Output: (40x40)-points PointClouds of the input scene 
		**/
		void inputForTesting(pcl::PointCloud<pcl::PointXYZ>::Ptr inputCloud,std::string folder);
};
