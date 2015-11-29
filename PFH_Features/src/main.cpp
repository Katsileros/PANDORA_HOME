#include "create_features.h"

int main(int argc, char **argv)
{
  /// NUmber of data to read	
  int num_train_data = 10;
  //~ int num_test_data = 11;
  /// The data set folder
  //~ std::string ss1 = "bowl";
  
  pfh_features *pfh_train = new pfh_features(num_train_data);
  pfh_train->buildTree("../train_driller/","../driller/RenderedPoses/renderedPose");
  
  return 0;
}
