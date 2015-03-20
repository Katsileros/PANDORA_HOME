#include "vfh_features.h"

/**
@brief Constructor
@param inputFolder: Contains the pcd training data files
@param num: Number of pcd files, goind to be read
**/
vfh_features::vfh_features(std::string inputFolder,int num)
{
  cloud_.reset(new pcl::PointCloud<pcl::PointXYZ>());
  normals_.reset(new pcl::PointCloud<pcl::Normal> ());
  vfhs_.reset(new pcl::PointCloud<pcl::VFHSignature308> ());
  
  cloud_->points.resize (cloud_->width * cloud_->height); 
  
  folder_ = inputFolder;
  num_ = num;
}

/**
@brief Calculates the normals of the given input pcd file
* 	   Then finds the vfh descriptor 
@return void
**/
void vfh_features::vfh_compute()
{		
  // Estimate the normals.
  pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> normalEstimation;
  normalEstimation.setInputCloud(cloud_);
  normalEstimation.setRadiusSearch(0.03);
  pcl::search::KdTree<pcl::PointXYZ>::Ptr kdtree(new pcl::search::KdTree<pcl::PointXYZ>);
  normalEstimation.setSearchMethod(kdtree);
  normalEstimation.compute(*normals_);
 
  // VFH estimation object.
  pcl::VFHEstimation<pcl::PointXYZ, pcl::Normal, pcl::VFHSignature308> vfh;
  vfh.setInputCloud(cloud_);
  vfh.setInputNormals(normals_);
  vfh.setSearchMethod(kdtree);
  // Optionally, we can normalize the bins of the resulting histogram,
  // using the total number of points.
  vfh.setNormalizeBins(true);
  // Also, we can normalize the SDC with the maximum size found between
  // the centroid and any of the cluster's points.
  vfh.setNormalizeDistance(false);
  
  vfh.compute(*vfhs_);
}

/**
@ Read the point clouds from the input folder. Caclulate the normals and
* the VFH descriptors using vfh_compute() func and saves the models.
* Then saves the training data to files. 
* Finally creates the kdtree index for the models.
**/
void vfh_features::buildTree()
{
	/// A folder with name models MUST exists.
  mkdir("models", 0777);
  
  for(int i=0;i<num_;i++)
  {
		std::string ss1 = folder_ + "/apple/apple_1/apple_1_1_" + boost::lexical_cast<std::string>(i+1) + ".pcd";
		
		  if (pcl::io::loadPCDFile(ss1, *this->cloud_) == -1) //* load the file
		  {
				std::cout << ("Couldn't read pcd file \n") << std::endl;
		  }
		
		this->vfh_compute();
		
		std::string ss2 = "models/" + boost::lexical_cast<std::string>(i+1) + "_VFH.pcd";
		pcl::io::savePCDFileASCII (ss2, *this->getVFH());
		//~ std::cout << "Saved " << ss2 << std::endl;
  }
  
  pcl::console::print_highlight ("Saved %d VFH models in models folder.\n", (int)num_);
  
  // Convert data into FLANN format
  flann::Matrix<float> data (new float[num_ * 308], num_, 308);

  pcl::PointCloud<pcl::VFHSignature308>::Ptr tmpVFH (new pcl::PointCloud<pcl::VFHSignature308> ());
	
  for(int i=0;i<num_;i++)
  {
	std::string ss1 = "models/" + boost::lexical_cast<std::string>(i+1) + "_VFH.pcd";
	if (pcl::io::loadPCDFile(ss1, *tmpVFH) == -1) //* load the file
	{
		std::cout << ("Couldn't read pcd file \n") << std::endl;
	}
	
	//~ std::cout << std::endl;
	//~ pcl::console::print_highlight("VFH Descriptor  %s: \n", ss1.c_str ());
  
    for(int j=0;j<308;j++)
    {
		//~ std::cerr << tmpVFH->points[0].histogram[j] << " ";
		data[i][j] = tmpVFH->points[0].histogram[j];
    } 
    //~ std::cout << std::endl;
  }
  
  std::string kdtree_idx_file_name = "kdtree.idx";
  std::string training_data_h5_file_name = "training_data.h5";
  std::string training_data_list_file_name = "training_data.list";
  
  pcl::console::print_highlight ("Loaded %d VFH models. Creating training data %s/%s.\n", 
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
