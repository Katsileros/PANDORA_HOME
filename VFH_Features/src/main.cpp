#include "vfh_features.h"

int main(int argc, char **argv)
{	
  int num_train_data = 10;
  
  std::string ss1 = "rgbd-dataset";
  vfh_features *vfh = new vfh_features(ss1, num_train_data);
  vfh->buildTree();
  
  return 0;
}
