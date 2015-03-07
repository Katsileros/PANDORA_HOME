#include "poseEstimation.h"

int main(int argc, char **argv)
{
	int i = 0;
	
	poseEstimation *pEst = new  poseEstimation("Parameters.xml");
	
	cv::FileStorage fs("Output.xml",cv::FileStorage::WRITE);
	
	for (i=0; i<4; i++)
	{
		std::string ss = "rgb" + boost::lexical_cast<std::string>(i+1) + ".png";
		std::cout << "\nimage:  " << ss << std::endl;
		
		pEst->readImgaes(cv::imread(ss));
		pEst->calcPose(i+1);
		pEst->writePose2XML(fs, i+1);
		pEst->clearVecMat();
	}
	fs.release();
	
	return 0;
	
}
