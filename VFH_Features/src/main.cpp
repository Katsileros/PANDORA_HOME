#include "create_features.h"

int main(int argc, char **argv)
{
  /// NUmber of data to read	
  int num_train_data = 10;
  int num_test_data = 1;
  /// The data set folder
  std::string ss1 = "bowl";
  
  vfh_features *vfh_train = new vfh_features(num_train_data);
  vfh_train->buildTree("train/","food_can/food_can_1/food_can_1_1_");
  
  vfh_features *vfh_test = new vfh_features(num_test_data);
  vfh_test->buildTree("test/","food_can/food_can_14/food_can_14_1_");
  
  return 0;
}
