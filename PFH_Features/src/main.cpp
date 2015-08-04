#include "create_features.h"

int main(int argc, char **argv)
{
  /// NUmber of data to read	
  int num_train_data = 3;
  int num_test_data = 1;
  /// The data set folder
  std::string ss1 = "bowl";
  
  pfh_features *pfh_train = new pfh_features(num_train_data);
  //~ pfh_train->buildTree("train/","food_can/food_can_1/food_can_1_1_");
  pfh_train->buildTree("train_driller/","driller/RenderedPoses/renderedPose");
  
  //~ pfh_features *pfh_test = new pfh_features(num_test_data);
  //~ pfh_test->buildTree("test/","food_can/food_can_14/food_can_14_1_");
  
  return 0;
}
