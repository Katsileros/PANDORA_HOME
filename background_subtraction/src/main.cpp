#include "BackgroundSubtraction.h"
#include "planarSegmentation.h"

int main(int argc, char **argv)
{
	BackgroundSubtraction *bg_sub = new  BackgroundSubtraction();
	
	bg_sub->setRgb(cv::imread("rgb.png"));
	bg_sub->setDepth(cv::imread("kafes_depth.png"));
	
	planarSegmentation seg;
	seg.fillPcd(bg_sub->getDepth());
	//~ seg.setThreshold(0.00092);
	seg.setThreshold(0.00094);
	seg.segment();
	seg.visualize(bg_sub->getRgb());

	bg_sub->setRgb(seg.getRgbSeg());
	bg_sub->setDepth(seg.getDepth());

	seg.fillPcd(bg_sub->getDepth());
	seg.setThreshold(0.0332);
	seg.segment();
	seg.visualize(bg_sub->getRgb());
		
	//~ bg_sub->setRgb(seg.getRgbSeg());
	//~ bg_sub->setDepth(seg.getDepth());
	
	//~ bg_sub->removeBackGround();
	//~ std::cout<< "\nFinished removeBackground." << std::endl;
	//~ cv::waitKey();
	//~ bg_sub->thresholdImg(bg_sub->getRgb());
	//~ std::cout<< "\nFinished boundingBox." << std::endl;
	//~ cv::waitKey();
	
	return 0;
}
