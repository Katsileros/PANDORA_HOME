#include "create_features.h"

/*! 
 *  \brief     This class creates PFH features
 *  \author    Katsileros Petros
 *  \date      5/5/2015
 *  \bug       FLANN::Matrix is fixed sized to the upper bound
 *  \copyright GNU Public License.
 */

/**
@brief Constructor
@param inputFolder: Contains the pcd training data files
@param num: Number of pcd files, goind to be read
**/
pfh_features::pfh_features(int num)
{
  cloud_.reset(new pcl::PointCloud<pcl::PointXYZ>());
  normals_.reset(new pcl::PointCloud<pcl::Normal> ());
  pfhs_.reset(new pcl::PointCloud<pcl::FPFHSignature33> ());
  
  cloud_->points.resize (cloud_->width * cloud_->height); 

  num_ = num;
}

/**
@brief Calculates the normals of the given input pcd file
* 	   Then finds the pfh descriptor 
@return void
**/
void pfh_features::pfh_compute()
{
  // Estimate the normals.
  pcl::NormalEstimationOMP<pcl::PointXYZ, pcl::Normal> normalEstimation;
  normalEstimation.setInputCloud(cloud_);
  normalEstimation.setRadiusSearch(0.03);
  pcl::search::KdTree<pcl::PointXYZ>::Ptr kdtree(new pcl::search::KdTree<pcl::PointXYZ>);
  normalEstimation.setSearchMethod(kdtree);
  normalEstimation.compute(*normals_);
 
  // PFH estimation object.
  pcl::FPFHEstimationOMP<pcl::PointXYZ, pcl::Normal, pcl::FPFHSignature33> pfh;
  pfh.setInputCloud(cloud_);
  pfh.setInputNormals(normals_);
  pfh.setSearchMethod(kdtree);

  pfh.setRadiusSearch (0.05);
  
  pfh.compute(*pfhs_);
}

/**
@ Read the point clouds from the input folder. Caclulate the normals and
* the VFH descriptors using pfh_compute() func and saves the models.
* Then saves the training data to files. 
* Finally creates the kdtree index for the models.
@param folder: The output save folder for the pcd VFH models
@param filenam: The path of the ply files.
* ex. For file bowl_1_0:8 filename is /folder_bowl/bowl_1_
**/
void pfh_features::buildTree(std::string folder,std::string filename)
{
  /// A folder with name "folder" MUST NOT exists.
  mkdir(folder.c_str(), 0777);
  
  for(int i=0;i<num_;i++)
  {
		std::string ss1 = filename + boost::lexical_cast<std::string>(i+1) + ".pcd";
		
		//~ if (pcl::io::loadPCDFile(ss1, *this->cloud_) == -1) //* load the file
		if(pcl::io::loadPCDFile<pcl::PointXYZ> (ss1, *this->cloud_) == -1)
		{
			std::cout << ("Couldn't read pcd file \n") << std::endl;
		}
		
		this->pfh_compute();
		
		std::string ss2 = folder.c_str() + boost::lexical_cast<std::string>(i) + "_PFH.pcd";	
		pcl::io::savePCDFileASCII (ss2.c_str(), *this->getPFH());	
		//~ std::cout << "Saved " << ss2 << std::endl;
  }
  
  
  pcl::console::print_highlight ("Saved %d PFH models in models folder.\n", (int)num_);
  
  // Convert data into FLANN format
  flann::Matrix<float> data (new float[num_ * (33*308)], num_, (33*308));
  pcl::PointCloud<pcl::FPFHSignature33>::Ptr tmpPFH (new pcl::PointCloud<pcl::FPFHSignature33> ());
	
  for(int i=0;i<num_;i++)
  {
	std::string ss1 = folder + boost::lexical_cast<std::string>(i) + "_PFH.pcd";
	if (pcl::io::loadPCDFile(ss1, *tmpPFH) == -1) //* load the file
	{
		std::cout << ("Couldn't read pcd file \n") << std::endl;
	}
	
	//~ pcl::console::print_highlight("PFH Descriptor  %s: \n", ss1.c_str ());
    // Make the data one big vector of size [num,(33*308)]. (33*308) is themax fpfh feature size
    for(int j=0;j<tmpPFH->points.size();j++)
    {
		for(int k=0;k<33;k++){
			data[i][j*33] = tmpPFH->points[j].histogram[k];
		}
    } 
  }
  
  std::string kdtree_idx_file_name = folder + "kdtree.idx";
  std::string training_data_h5_file_name = folder + "training_data.h5";
  std::string training_data_list_file_name = folder + "training_data.list";
  
  pcl::console::print_highlight ("Loaded %d PFH models. Creating training data %s/%s.\n", 
      (int)num_, training_data_h5_file_name.c_str (), training_data_list_file_name.c_str ());
  
  // Save data to disk (list of models)
  flann::save_to_file (data, training_data_h5_file_name, "training_data");
  std::ofstream fs;
  fs.open (training_data_list_file_name.c_str ());
  for (size_t i = 0; i < num_; ++i)
    fs << "model-" + boost::lexical_cast<std::string>(i) << "\n";
  fs.close ();
 
  // Build the tree index and save it to disk
  pcl::console::print_error ("Building the kdtree index (%s) for %d elements...\n", kdtree_idx_file_name.c_str (), (int)num_);
  //~ flann::Index<flann::ChiSquareDistance<float> > index (data, flann::LinearIndexParams ());
  flann::Index<flann::ChiSquareDistance<float> > index (data, flann::KDTreeIndexParams (4));
  index.buildIndex ();
  index.save (kdtree_idx_file_name);
  delete[] data.ptr ();
}
