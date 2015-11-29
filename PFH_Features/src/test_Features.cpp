#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/common/common.h>
#include <pcl/common/transforms.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/console/parse.h>
#include <pcl/console/print.h>
#include <pcl/io/pcd_io.h>
#include <pcl/io/ply_io.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/search/kdtree.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/keypoints/uniform_sampling.h>


#include "opencv/cv.h"
#include "opencv2/opencv.hpp"
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/core/core.hpp"

#include <flann/flann.h>
#include <flann/io/hdf5.h>

#include <iostream>
#include <boost/filesystem.hpp>

#include "create_features.h"

/*! 
 *  \brief     This function tests PFH-matching with KNN-algorithm 
 *  \author    Katsileros Petros
 *  \date      19/5/2015
 *  \copyright GNU Public License.
 */

typedef std::pair<std::string, std::vector<float> > pfh_model;

/** \brief Loads an n-D histogram file as a PFH signature
  * \param path the input file name
  * \param vfh the resultant VFH model
  */
bool
loadHist (const boost::filesystem::path &path, pfh_model &pfh)
{
  int pfh_idx;
  // Load the file as a PCD
  try
  {
    pcl::PCLPointCloud2 cloud;
    int version;
    Eigen::Vector4f origin;
    Eigen::Quaternionf orientation;
    pcl::PCDReader r;
    int type; unsigned int idx;
    r.readHeader (path.string (), cloud, origin, orientation, version, type, idx);

    pfh_idx = pcl::getFieldIndex (cloud, "fpfh");
    if (pfh_idx == -1)
      return (false);
  }
  catch (const pcl::InvalidConversionException&)
  {
    return (false);
  }
  
  // Treat the PFH signature as a single Point Cloud
  pcl::PointCloud <pcl::FPFHSignature33> point;
  pcl::io::loadPCDFile (path.string (), point);
  

  std::vector <pcl::PCLPointField> fields;
  getFieldIndex (point, "fpfh", fields);
  // This is the max size of a pfh feature
  pfh.second.resize (33);
  
  //~ std::cout << fields[pfh_idx].count << std::endl;
  //~ std::cout << point.points.size() << std::endl;

  for (size_t i = 0; i < fields[pfh_idx].count; ++i)
  {
	pfh.second[i] = point.points[0].histogram[i];
  }

  //~ std::cout << pfh.second.size() << std::endl;
  pfh.first = path.string ();
  return (true);
}


/** \brief Search for the closest k neighbors
  * \param index the tree
  * \param model the query model
  * \param k the number of neighbors to search for
  * \param indices the resultant neighbor indices
  * \param distances the resultant neighbor distances
  */
inline void
nearestKSearch (flann::Index<flann::ChiSquareDistance<float> > &index, const pfh_model &model, 
                int k, flann::Matrix<int> &indices, flann::Matrix<float> &distances)
{
  // Query point
  flann::Matrix<float> p = flann::Matrix<float>(new float[model.second.size ()], 1, model.second.size ());
  memcpy (&p.ptr ()[0], &model.second[0], p.cols * p.rows * sizeof (float));

  indices = flann::Matrix<int>(new int[k], 1, k);
  distances = flann::Matrix<float>(new float[k], 1, k);
  index.knnSearch (p, indices, distances, k, flann::SearchParams (128));
  delete[] p.ptr ();
}

/** \brief Load the list of file model names from an ASCII file
  * \param models the resultant list of model name
  * \param filename the input file name
  */
bool
loadFileList (std::vector<pfh_model> &models, const std::string &filename)
{
  ifstream fs;
  fs.open (filename.c_str ());
  if (!fs.is_open () || fs.fail ())
    return (false);

  std::string line;
  while (!fs.eof ())
  {
    getline (fs, line);
    if (line.empty ())
      continue;
    pfh_model m;
    m.first = line;
    models.push_back (m);
  }
  fs.close ();
  return (true);
}

/** \brief Load the depth map
  * \param filename
  */
//~ cv::Mat loadDepth( std::string file , std::string filename)
cv::Mat loadDepth( std::string file)
{
	std::ifstream l_file(file.c_str());
	
	if( l_file.fail() == true ) 
	{
		printf("cv_load_depth: could not open file for reading!\n");
	}
	int l_row;
	int l_col;

	l_file.read((char*)&l_row,sizeof(l_row));
	l_file.read((char*)&l_col,sizeof(l_col));

	cv::Mat image = cv::Mat::zeros(l_row,l_col,CV_16U);	
	
	//~ cv::Mat pose = cv::Mat::zeros(l_row,l_col,CV_8UC3);
	//~ pose = cv::imread(filename);
	//~ std::cout << "channels: " << pose.channels() << std::endl;
	//~ cv::imshow("pose", pose);
	//~ std::cout << pose.size() << std::endl;
	//~ cv::waitKey();
	
	//~ pose = cv::Mat::zeros(l_row,l_col,CV_8UC3);
	//~ cv::Mat poseItemSpace = cv::Mat(l_row,l_col,CV_8UC3,cv::Scalar(255,255,255));
	//~ cv::Mat poseItemSpace = cv::Mat(l_row,l_col,CV_8UC3);
	
	for(int l_r=0;l_r<l_row;l_r++)
	{
		for(int l_c=0;l_c<l_col;l_c++)
		{	
			//~ if( (pose.at<cv::Vec3b>(l_c,l_r)[0] != 255) && (pose.at<cv::Vec3b>(l_c,l_r)[1] != 255) && (pose.at<cv::Vec3b>(l_c,l_r)[2] != 255) )
			//~ {
				//~ std::cout << "R: " << pose.at<cv::Vec3b>(l_c,l_r)[0] << "  G: " <<  pose.at<cv::Vec3b>(l_c,l_r)[1] << "  B: " << pose.at<cv::Vec3b>(l_c,l_r)[2] << std::endl;
				//~ std::cout << "(row,col): (" << l_r << "," << l_c << ")" << std::endl;
				l_file.read((char*)&image.at<unsigned short>(l_r,l_c),sizeof(unsigned short));
			//~ }
				
				//~ poseItemSpace.at<cv::Vec3i>(l_c,l_r)[0] = 255;
				//~ poseItemSpace.at<cv::Vec3i>(l_c,l_r)[1] = 255;
				//~ poseItemSpace.at<cv::Vec3i>(l_c,l_r)[2] = 255;
		}
	}	
	l_file.close();
	
	//~ std::cout << poseItemSpace << std::endl;

	//~ cv::imshow("poseBlack", poseItemSpace);
	//~ cv::waitKey();
	
	//~ cv::imshow("image", image);
	//~ cv::waitKey();	

	return image;
}

void viewerCloud(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud)
{
    pcl::visualization::PCLVisualizer* viewer(new pcl::visualization::PCLVisualizer);
    viewer->setBackgroundColor(0,0,0);
    viewer->addPointCloud<pcl::PointXYZ> (cloud, "cloud");
    //~ viewer->addCoordinateSystem(1.0);
    //viewer->initCameraParameters();
    while(!viewer->wasStopped())
    {
        viewer->spinOnce(100);
        boost::this_thread::sleep (boost::posix_time::microseconds (100000));
    }
}

//!< RANSAC
void findPlanes(const pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, std::vector<Eigen::Vector4d> &planes)
{
    pcl::PointCloud<pcl::PointXYZ>::Ptr tempCloud(new pcl::PointCloud<pcl::PointXYZ>);
    *tempCloud = *cloud;

    pcl::SACSegmentation<pcl::PointXYZ> seg;
    seg.setOptimizeCoefficients(true);
    seg.setProbability(0.99);
    seg.setModelType(pcl::SACMODEL_PLANE);
    seg.setMethodType(pcl::SAC_RANSAC);
    seg.setDistanceThreshold(0.01);
    seg.setMaxIterations(500);

    pcl::ExtractIndices<pcl::PointXYZ> extract;

    while(1)
    {
        pcl::ModelCoefficients::Ptr coeffs(new pcl::ModelCoefficients);
        pcl::PointIndices::Ptr inliersPlane (new pcl::PointIndices);

        seg.setInputCloud(tempCloud);
        seg.segment(*inliersPlane, *coeffs);

        if(inliersPlane->indices.size() < 50000) break;

        Eigen::Vector4d cand(coeffs->values[0], coeffs->values[1], coeffs->values[2], coeffs->values[3]);

        pcl::PointCloud<pcl::PointXYZ>::Ptr cloudFiltered(new pcl::PointCloud<pcl::PointXYZ>);
        extract.setInputCloud(tempCloud);
        extract.setIndices(inliersPlane);
        extract.setNegative(true);
        extract.filter(*cloudFiltered);
        *tempCloud = *cloudFiltered;

        planes.push_back(cand);
    }
}

void pointsAbovePlane(const pcl::PointCloud<pcl::PointXYZ>::Ptr cloud,
                                    const Eigen::Vector4d &plane,
                                    pcl::IndicesPtr &indices)
{
	float tmin = 0.01;
    float tmax = 0.5;
    for(int i = 0; i < cloud->height; i++)
    {
        for(int j = 0; j < cloud->width; j++)
        {
            pcl::PointXYZ p = cloud->at(j, i);

            Eigen::Vector4d p_(p.x, p.y, p.z, 1.0);

            double dist = p_.dot(plane);

            if(dist > tmin  &&  dist < tmax)
            {
                indices->push_back(i * cloud->width + j);
            }
        }
    }
}

void findClusters(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, pcl::IndicesPtr &indices)
{

     //!< Creating the KdTree object for the search method of the extraction
    pcl::search::KdTree<pcl::PointXYZ>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZ>);
    tree->setInputCloud(cloud);

    std::vector<pcl::PointIndices> cluster_indices;
    pcl::EuclideanClusterExtraction<pcl::PointXYZ> ec;
    ec.setClusterTolerance (0.01); //!< 1cm
    ec.setMinClusterSize (1000);
    ec.setMaxClusterSize (cloud->size());
    ec.setSearchMethod (tree);
    ec.setInputCloud(cloud);
    ec.setIndices(indices);
    ec.extract (cluster_indices);
    
    //~ std::cout << "Cluster_indicies: " << cluster_indices.size() << std::endl;

    cv::Mat mask = cv::Mat::zeros(cloud->height, cloud->width, CV_8UC1);
    int clusterId = 1;
    for(int i = 0; i < cluster_indices.size(); i++)
    {
        pcl::PointIndices cluster = cluster_indices[i];
        //~ std::cout << "Cluster_" << i << ": " << std::endl;
        //~ std::cout << cluster << std::endl;
        
        pcl::PointCloud<pcl::PointXYZ>::Ptr newCloud (new pcl::PointCloud<pcl::PointXYZ>);
        newCloud->points.resize (cloud->width * cloud->height); 
        std::cout << "Cluster[" << i << "]_indices.size: " << cluster.indices.size() << std::endl;
        
        for(int j = 0; j < cluster.indices.size(); j++)
        {
			//~ std::cout << cluster.indices[j] << std::endl;
			//~ std::cout << "x: " << cloud->points[cluster.indices[j]].x << "  y: " << cloud->points[cluster.indices[j]].y << "  z: " << cloud->points[cluster.indices[j]].z << std::endl;
			newCloud->points[cluster.indices[j]].x = cloud->points[cluster.indices[j]].x;
			newCloud->points[cluster.indices[j]].y = cloud->points[cluster.indices[j]].y;
			newCloud->points[cluster.indices[j]].z = cloud->points[cluster.indices[j]].z;
        }
                
	    // Uniform sampling object.
	    pcl::UniformSampling<pcl::PointXYZ> filter;
	    filter.setInputCloud(newCloud);
	    // We set the size of every voxel to be 1x1x1cm
	    // (only one point per every cubic centimeter will survive).
	    filter.setRadiusSearch(0.01f);
	    // We need an additional object to store the indices of surviving points.
	    pcl::PointCloud<int> keypointIndices;
	
	    filter.compute(keypointIndices);
	    pcl::copyPointCloud(*newCloud, keypointIndices.points, *newCloud);
        
        std::cout << "mask-size after sub-sampling: " << newCloud->size() << std::endl;
        
        std::string outputFilename =  "../test_driller/testingNewPoses/newPose_" + boost::lexical_cast<std::string>(i) + std::string(".pcd");
        pcl::io::savePCDFileASCII (outputFilename, *newCloud);
        
        //~ viewerCloud(newCloud);
        
        clusterId++;
    }

    //~ int noOfRegions = cluster_indices.size();

}

void inputForTesting(pcl::PointCloud<pcl::PointXYZ>::Ptr inputCloud,std::string folder)
{
	pcl::PointCloud<pcl::PointXYZ>::Ptr newWindowCloud (new pcl::PointCloud<pcl::PointXYZ>);
	
	//remove NAN points from the cloud
    std::vector<int> indices;
    pcl::removeNaNFromPointCloud(*inputCloud,*inputCloud, indices);
    
    Eigen::Affine3f transform = Eigen::Affine3f::Identity();
	transform.rotate (Eigen::AngleAxisf (-180, Eigen::Vector3f::UnitX()));
	
	pcl::transformPointCloud (*inputCloud, *inputCloud, transform);
    
    //~ pcl::visualization::CloudViewer viewer ("Simple Cloud Viewer");
    //~ viewer.showCloud (inputCloud);
    //~ while (!viewer.wasStopped ())
    //~ {
    //~ }
	
	mkdir(folder.c_str(), 0777);
	
	//~ std::cout << "input-width=" << inputCloud->width <<	 " , input-height=" << inputCloud->height << std::endl;
	//~ inputCloud->points.resize (inputCloud->width * inputCloud->height); 
	
	int i,j;
	
	//!< find planes
    std::vector<Eigen::Vector4d> planes;
    findPlanes(inputCloud, planes);
    
    for(int i = 0; i < planes.size(); i++)
    {
        //!< find what part of the point cloud lie above each candidate plane
        pcl::IndicesPtr indices(new std::vector <int>);
        pointsAbovePlane(inputCloud, planes[i], indices);

        //~ //!< find clusters on the remaining point cloud to obtain individual object point clouds
        findClusters(inputCloud, indices);
	}
	
	std::cout << "Over" << std::endl;
}

void prepareTestingData() {
  pcl::PointCloud<pcl::PointXYZ>::Ptr testingCloud (new pcl::PointCloud<pcl::PointXYZ>);

  if (pcl::io::loadPCDFile<pcl::PointXYZ> ("../test_driller/testingPointCloud0.pcd", *testingCloud) == -1) //* load the file
  {
    PCL_ERROR ("Couldn't read file test_pcd.pcd \n");
    exit(0);
  }
  	
  //~ viewerCloud(testingCloud);
  inputForTesting(testingCloud,"../test_driller");
  
  int num_test_data = 11;
  pfh_features *pfh_test = new pfh_features(num_test_data);
  pfh_test->buildTree("../test_driller/","../test_driller/testingNewPoses/newPose_");
  
  exit(0);
	
}


int main (int argc, char** argv)
{  
	
  int k = 6;

  double thresh = DBL_MAX;     // No threshold, disabled by default

  //~ if (argc < 2)
  //~ {
    //~ pcl::console::print_error 
      //~ ("Need at least three parameters! Syntax is: %s <query_pfh_model.pcd> [options] {kdtree.idx} {training_data.h5} {training_data.list}\n", argv[0]);
    //~ pcl::console::print_info ("    where [options] are:  -k      = number of nearest neighbors to search for in the tree (default: "); 
    //~ pcl::console::print_value ("%d", k); pcl::console::print_info (")\n");
    //~ pcl::console::print_info ("                          -thresh = maximum distance threshold for a model to be considered VALID (default: "); 
    //~ pcl::console::print_value ("%f", thresh); pcl::console::print_info (")\n\n");
    //~ return (-1);
  //~ }
  
  prepareTestingData();
  
  std::string extension (".pcd");
  transform (extension.begin (), extension.end (), extension.begin (), (int(*)(int))tolower);

  // Load the test histogram
  std::vector<int> pcd_indices = pcl::console::parse_file_extension_argument (argc, argv, ".pcd");
  std::cout << pcd_indices.at(0) << std::endl;
  pfh_model histogram;
  if (!loadHist (argv[pcd_indices.at (0)], histogram))
  {
    pcl::console::print_error ("Cannot load test file %s\n", argv[pcd_indices.at (0)]);
    return (-1);
  }

  pcl::console::parse_argument (argc, argv, "-thresh", thresh);
  // Search for the k closest matches
  pcl::console::parse_argument (argc, argv, "-k", k);
  pcl::console::print_highlight ("Using "); pcl::console::print_value ("%d", k); pcl::console::print_info (" nearest neighbors.\n");

  std::string kdtree_idx_file_name = "train/kdtree.idx";
  std::string training_data_h5_file_name = "train/training_data.h5";
  std::string training_data_list_file_name = "train/training_data.list";

  std::vector<pfh_model> models;
  flann::Matrix<int> k_indices;
  flann::Matrix<float> k_distances;
  flann::Matrix<float> data;
  // Check if the data has already been saved to disk
  if (!boost::filesystem::exists ("train/training_data.h5") || !boost::filesystem::exists ("train/training_data.list"))
  {
    pcl::console::print_error ("Could not find training data models files %s and %s!\n", 
        training_data_h5_file_name.c_str (), training_data_list_file_name.c_str ());
    return (-1);
  }
  else
  {
    loadFileList (models, training_data_list_file_name);
    flann::load_from_file (data, training_data_h5_file_name, "training_data");
    pcl::console::print_highlight ("Training data found. Loaded %d PFH models from %s/%s.\n", 
        (int)data.rows, training_data_h5_file_name.c_str (), training_data_list_file_name.c_str ());
  }

  // Check if the tree index has already been saved to disk
  if (!boost::filesystem::exists (kdtree_idx_file_name))
  {
    pcl::console::print_error ("Could not find kd-tree index in file %s!", kdtree_idx_file_name.c_str ());
    return (-1);
  }
  else
  {
    flann::Index<flann::ChiSquareDistance<float> > index (data, flann::SavedIndexParams ("train/kdtree.idx"));
    index.buildIndex ();
    nearestKSearch (index, histogram, k, k_indices, k_distances);
  }

  // Output the results on screen
  pcl::console::print_highlight ("The closest %d neighbors for %s are:\n", k, argv[pcd_indices[0]]);
  for (int i = 0; i < k; ++i)
  {
    pcl::console::print_info ("    %d - %s (%d) with a distance of: %f\n", 
        i, models.at (k_indices[0][i]).first.c_str (), k_indices[0][i], k_distances[0][i]);
  }

  /*
  pcl::visualization::PCLVisualizer viewer;
  int y_s = (int)floor (sqrt ((double)k));
  int x_s = y_s + (int)ceil ((k / (double)y_s) - y_s);
  double x_step = (double)(1 / (double)x_s);
  double y_step = (double)(1 / (double)y_s);
  int viewport = 0, l = 0, m = 0;
  
  for (int i = 0; i < k; ++i)
  {
    viewer.createViewPort (l * x_step, m * y_step, (l + 1) * x_step, (m + 1) * y_step, viewport);
    l++;
    if (l >= x_s)
    {
      l = 0;
      m++;
    }
	  
    std::string cloud_name = "food_can/food_can_1/food_can_1_1_" + boost::lexical_cast<std::string>(k_indices[0][i]+1) + ".pcd";

    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>);
    pcl::console::print_highlight (stderr, "Loading "); 
    pcl::console::print_value (stderr, "%s ", cloud_name.c_str ());
    if (pcl::io::loadPCDFile (cloud_name, *cloud) == -1)
      break; 
    //~ if (pcl::io::loadPLYFile (cloud_name, *cloud) == -1)
      //~ break;
    

    if (cloud->points.size () == 0)
      break;

    //~ cloud->points.resize (cloud->height * cloud->width);

    pcl::console::print_info ("[done, "); 
    pcl::console::print_value ("%d", (int)cloud->points.size ()); 
    pcl::console::print_info (" points]\n");
    pcl::console::print_info ("Available dimensions: "); 
    pcl::console::print_value ("%s\n", pcl::getFieldsList (*cloud).c_str ());

    // Demean the cloud
    Eigen::Vector4f centroid;
    pcl::compute3DCentroid (*cloud, centroid);
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_xyz_demean (new pcl::PointCloud<pcl::PointXYZ>);
    pcl::demeanPointCloud<pcl::PointXYZ> (*cloud, centroid, *cloud_xyz_demean);
    // Add to renderer*
    viewer.addPointCloud (cloud_xyz_demean, cloud_name, viewport);
    
    // Check if the model found is within our inlier tolerance
    std::stringstream ss;
    ss << k_distances[0][i];
    if (k_distances[0][i] > thresh)
    {
      viewer.addText (ss.str (), 20, 30, 1, 0, 0, ss.str (), viewport);  // display the text with red

      // Create a red line
      pcl::PointXYZ min_p, max_p;
      pcl::getMinMax3D (*cloud_xyz_demean, min_p, max_p);
      std::stringstream line_name;
      line_name << "line_" << i;
      viewer.addLine (min_p, max_p, 1, 0, 0, line_name.str (), viewport);
      viewer.setShapeRenderingProperties (pcl::visualization::PCL_VISUALIZER_LINE_WIDTH, 5, line_name.str (), viewport);
    }
    else
      viewer.addText (ss.str (), 20, 30, 0, 1, 0, ss.str (), viewport);

    // Increase the font size for the score*
    viewer.setShapeRenderingProperties (pcl::visualization::PCL_VISUALIZER_FONT_SIZE, 18, ss.str (), viewport);

    // Add the cluster name
    viewer.addText (cloud_name, 20, 10, cloud_name, viewport);  
  }
  
  // Add coordianate systems to all viewports
  viewer.addCoordinateSystem (0.1, 0);
  viewer.spin();
  
 */
 
  return (0);
}

