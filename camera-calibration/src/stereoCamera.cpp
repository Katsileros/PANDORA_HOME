#include "stereoCamera.h"
#include <limits>

int height=480;
int width=640;
int pointxup=0;
int pointyup=0;
int pointxdown=0;
int pointydown=0;
float averagex=0;
float averagey=0;
float averagez=0;
cv::Mat StereoCamera::staticDisparity;

StereoCamera::StereoCamera(std::string filename)
{
	int leftID;
	// Find the left and right cameras.
	//~ leftID = getLeftCameraID();
	left = MonocularCamera(leftID , "Left" );
	right = MonocularCamera(!leftID, "Right");
	this->left.setCameraID(1);
	this->right.setCameraID(2);
	setFileName(filename);
	this->readXML();
	
	//~ this->depth = cv::Mat(cv::Size(640,480),CV_32FC1);
	//~ cv::namedWindow("Left Frame");
	//~ cv::setMouseCallback("Left Frame",&callBack,&this->depth);
	this->TrackBarBM();
	//~ this->TrackBarBGM();
	
	}
	
StereoCamera::StereoCamera(int num)
{
	this->left.setCameraID(2);
	this->right.setCameraID(1);
	//~ Create the left camera object and read the calibration 
	//~ parameters.
	//~ Pass the parameters to the right camera.
	this->inputParameters();
	left.setFileName("Left");
	right.setFileName("Right");
	this->setFileName("Stereo");
	//~ Calibrate the Monocular Cameras.
	left.calibrate("Left");
	right.calibrate("Right");
	
	left.clearPoints();
	right.clearPoints();
	//~ Rename the monoculars to match Stereo pair.
	left.setFileName("StereoLeft");
	right.setFileName("StereoRight");
	//~ Calibrate the stereo Camera.
	this->calibrate(left.getCameraParameters(),right.getCameraParameters());
	this->rectify();
	std::cout << "Finished Calibrating Stereo Camera " << std::endl;
	//~ Save the Extrinsics parameters to a file.
	this->writeXML();
	
	std::cout << "Wrote parameters to file " << std::endl;
	
	//~ Generate the Disparity Map
	//~ this->TrackBarBM();
	//~ this->TrackBarBGM();
	//~ this->generate3d();
	}

void StereoCamera::TrackBarBM()
{
	sbm = cv::StereoBM(cv::StereoBM::BASIC_PRESET);
	std::string windowName("Parameters BM");
	cv::namedWindow(windowName,true);
	int values = 192;
	
	// Set initial properties.
	sbm.state->SADWindowSize = 5;
	sbm.state->numberOfDisparities = 64;
	sbm.state->preFilterSize = 5;
	sbm.state->preFilterCap = 61;
	sbm.state->minDisparity = 8;
	sbm.state->textureThreshold = 507;
	sbm.state->uniquenessRatio = 1;
	sbm.state->speckleWindowSize = 8;
	sbm.state->speckleRange = 4;
	sbm.state->disp12MaxDiff = 1;
	
/*	
    bm.state->preFilterCap = 31;
// bm.state->SADWindowSize  = SADWindowSize > 0 ? SADWindowSize : 9;
    bm.state->minDisparity = 0;
    bm.state->numberOfDisparities = 32;
    bm.state->textureThreshold = 10;
    bm.state->uniquenessRatio = 15;
    bm.state->speckleWindowSize = 100;
    bm.state->speckleRange = 32;
    bm.state->disp12MaxDiff = 1;
	*/

	
	cv::createTrackbar("SADWindowSize", windowName, &sbm.state->SADWindowSize,  180, &SADcallback, &this->sbm);
	
	cv::createTrackbar("numberOfDisparities", windowName, &sbm.state->numberOfDisparities, 200 ,
	 &numDisparitiesCallbackBM,&this->sbm);
	
	cv::createTrackbar("preFilterCap", windowName, &sbm.state->preFilterCap,  63);
	
	
	cv::createTrackbar("minDisparity", windowName, &sbm.state->minDisparity,  300 ,
	&minDisparitiesCallbackBM,&this->sbm);
	
	cv::createTrackbar("uniquenessRatio", windowName, &sbm.state->uniquenessRatio,  5);
	
	cv::createTrackbar("Speckle Window Size", windowName, 
		&sbm.state->speckleWindowSize , 200);
	
	cv::createTrackbar("Speckle Range", windowName, 
		&sbm.state->speckleRange , 5);
	
	cv::createTrackbar("Disp12MaxDiff",windowName,&sbm.state->disp12MaxDiff,100);
	
	
	}
	
void StereoCamera::TrackBarBGM()
{
	std::string windowName("Parameters SGBM");
	cv::namedWindow(windowName,true);
	int values = 192;
	
	// Set initial properties.
	sgbm.SADWindowSize = 5;
	sgbm.numberOfDisparities = 192;
	sgbm.preFilterCap = 4;
	sgbm.minDisparity = -64;
	sgbm.uniquenessRatio = 1;
	sgbm.speckleWindowSize = 150;
	sgbm.speckleRange = 2;
	sgbm.disp12MaxDiff = 10;
	sgbm.fullDP = false;
	sgbm.P1 = 600;
	sgbm.P2 = 2400;	
	
	cv::createTrackbar("SADWindowSize", windowName, &sgbm.SADWindowSize,  12);
	
	cv::createTrackbar("numberOfDisparities", windowName, &values, 200 ,
	 &numDisparitiesCallback,&this->sgbm);
	
	cv::createTrackbar("preFilterCap", windowName, &sgbm.preFilterCap,  60);
	
	cv::createTrackbar("minDisparity", windowName, &sgbm.minDisparity,  300 ,
	&minDisparitiesCallback,&this->sgbm);
	
	cv::createTrackbar("uniquenessRatio", windowName, &sgbm.uniquenessRatio,  5);
	
	cv::createTrackbar("P1", windowName, &sgbm.P1,  2000);
	
	cv::createTrackbar("P2", windowName, &sgbm.P2,  3000);
	
	cv::createTrackbar("Speckle Window Size", windowName, 
		&sgbm.speckleWindowSize , 200);
	
	cv::createTrackbar("Speckle Range", windowName, 
		&sgbm.speckleRange , 5);
	
	cv::createTrackbar("Disp12MaxDiff",windowName,&sgbm.disp12MaxDiff,100);
	
}
	


	
void StereoCamera::testRectification()
{
	cv::destroyAllWindows();
	cv::VideoCapture leftCamera(left.getCameraID());
	cv::VideoCapture rightCamera(right.getCameraID());
	
	cv::Mat rightFrame;
	cv::Mat leftFrame;
	cv::Mat rightRectified;
	cv::Mat leftRectified;
	
	cv::Mat leftImg;
	cv::Mat rightImg;
	
	bool end = false;
	
	std::vector<cv::Point2f> leftPattern;
	std::vector<cv::Point2f> rightPattern;
	
	std::string leftWindow("Left Camera");
	std::string rightWindow("Right Camera");
	
	if ( !leftCamera.isOpened() || !rightCamera.isOpened() )
	{
		std::cout << "Error opening the cameras! The program is exiting!" << std::endl ;
		exit(1);
	}
	
	leftCamera.set(CV_CAP_PROP_FRAME_WIDTH, 640);
    leftCamera.set(CV_CAP_PROP_FRAME_HEIGHT,480);
    
    rightCamera.set(CV_CAP_PROP_FRAME_WIDTH, 640);
    rightCamera.set(CV_CAP_PROP_FRAME_HEIGHT,480);
    CameraParameters leftParam = left.getCameraParameters();
    CameraParameters rightParam = right.getCameraParameters();
    cv::Mat leftRmap[2];
    cv::Mat rightRmap[2];
    
    while( true )
	{
		leftCamera.grab();
		leftCamera.retrieve(leftFrame);
		rightCamera.grab();
		rightCamera.retrieve(rightFrame);
		
		//~ cv::imshow("Left", leftFrame);
		//~ cv::imshow("Right", rightFrame);

		
		cv::initUndistortRectifyMap(leftParam.cameraMatrix, leftParam.distortion, 
									rectifiedParam.R1, rectifiedParam.P1, cv::Size(640, 480), CV_16SC2,
									leftRmap[0], leftRmap[1]);
									
		cv::initUndistortRectifyMap(rightParam.cameraMatrix, rightParam.distortion, 
									rectifiedParam.R2, rectifiedParam.P2, cv::Size(640, 480), CV_16SC2,
									rightRmap[0], rightRmap[1]);
									
		cv::remap( leftFrame, leftRectified,  leftRmap[0], leftRmap[1], CV_INTER_LINEAR );
		cv::remap( rightFrame, rightRectified,  rightRmap[0], rightRmap[1], CV_INTER_LINEAR );
		cv::imshow("LeftRectified", leftRectified);
		cv::imshow("RightRectified", rightRectified);
		
		if (cv::waitKey(30)>=0) break;
	}
	
}

void StereoCamera::callBack(int event, int x, int y, int flags, void* param)
{
	cv::Mat* depth2 = (cv::Mat*)param;
	
	std::vector<cv::Mat> points;
	cv::split(depth2[0],points);

	 if  ( event == cv::EVENT_LBUTTONDOWN )
     {
		 std::cout <<"(x,y)"<<x<<","<<y<<"    disparity:"<< staticDisparity.at<float>(y,x) << std::endl;
		 //~ std::cout<<"Point in callback: ("<< points[0].at<float>(240,320) <<","<< points[1].at<float>(240,320) <<","<< points[2].at<float>(240,320) <<")"<<std::endl; // Depth is float
		 std::cout<<"Point in callback: ("<< points[0].at<float>(y,x) <<","<< points[1].at<float>(y,x) <<","<< points[2].at<float>(y,x) <<")"<<std::endl; // Depth is float
		 //~ std::cout<<"Point in callback: ("<< points[0].at<float>(240,320) <<std::endl; // Depth is float
     }
     
}
	

void StereoCamera::generate3d()
{	
	cv::VideoCapture leftCamera(left.getCameraID());
	cv::VideoCapture rightCamera(right.getCameraID());
	

	cv::Mat rightFrame;
	cv::Mat leftFrame;
	cv::Mat rightRectified;
	cv::Mat leftRectified;
	cv::Mat disparity(480,640,CV_16S);
	cv::Mat temp;
	cv::Mat depthTest;
	
	cv::Mat leftImg;
	cv::Mat rightImg;
	
	bool end = false;
	
	std::vector<cv::Point2f> leftPattern;
	std::vector<cv::Point2f> rightPattern;
	
	
	if ( !leftCamera.isOpened() || !rightCamera.isOpened() )
	{
		std::cout << "Error opening the cameras! The program is exiting!" << std::endl ;
		exit(1);
	}
	
	
	leftCamera.set(CV_CAP_PROP_FRAME_WIDTH, 640);
    leftCamera.set(CV_CAP_PROP_FRAME_HEIGHT,480);
    
    rightCamera.set(CV_CAP_PROP_FRAME_WIDTH, 640);
    rightCamera.set(CV_CAP_PROP_FRAME_HEIGHT,480);
    CameraParameters leftParam = left.getCameraParameters();
    CameraParameters rightParam = right.getCameraParameters();
    cv::Mat leftRmap[2];
    cv::Mat rightRmap[2];
    cv::Mat eqDepth;
	
	std::vector<cv::Mat> channels;
	
	double min = std::numeric_limits<double>::max();
	double max = std::numeric_limits<double>::min();
    
    while( true )
	{
		leftCamera.grab();
		leftCamera.retrieve(leftFrame);
		rightCamera.grab();
		rightCamera.retrieve(rightFrame);
		
		cv::initUndistortRectifyMap(leftParam.cameraMatrix, leftParam.distortion, 
									rectifiedParam.R1, rectifiedParam.P1, cv::Size(640, 480), CV_16SC2,
									leftRmap[0], leftRmap[1]);
									
		cv::initUndistortRectifyMap(rightParam.cameraMatrix, rightParam.distortion, 
									rectifiedParam.R2, rectifiedParam.P2, cv::Size(640, 480), CV_16SC2,
									rightRmap[0], rightRmap[1]);
						
		cv::Mat gray_left_rect,gray_right_rect;
					
		cv::remap( leftFrame, leftRectified,  leftRmap[0], leftRmap[1], CV_INTER_LINEAR );
		cv::remap( rightFrame, rightRectified,  rightRmap[0], rightRmap[1], CV_INTER_LINEAR );
		cv::cvtColor( leftRectified, gray_left_rect, CV_BGR2GRAY );
		cv::cvtColor( rightRectified, gray_right_rect, CV_BGR2GRAY );
	
	
		cv::namedWindow("Disparity Map");
		//~ cv::namedWindow("Depth");
		cv::namedWindow("Left Frame");
		//~ cv::namedWindow("Right Frame");
		
	
		// SGBM
		
		//~ cv::Mat show_disparity(480,640,CV_8UC1);
		//~ sgbm(leftRectified,rightRectified,disparity);
		//~ cv::normalize(disparity,show_disparity,0,255,CV_MINMAX,CV_8U);
		//~ disparity.convertTo(disparity, CV_32FC1,1./16); // OR CV_8U ?
		//~ cv::imshow("Disparity Map",show_disparity);
	
	
		// SBM
		
		cv::Mat show_disparity;
		sbm(gray_left_rect,gray_right_rect,disparity);
		//~ // disparity.convertTo(show_disparity,CV_8UC1,255./(16*sbm.state->numberOfDisparities));
		cv::normalize(disparity,show_disparity,0,255,CV_MINMAX,CV_8U);
		cv::imshow("Disparity Map",show_disparity);
		
		
		//~ double min;
		//~ double max;
		//~ cv::minMaxLoc(disparity,&min,&max);
		//~ double norm = 255 / max ; 
		//~ disp = disparity * norm / 255 ;
		
		disparity.convertTo(staticDisparity, CV_32FC1,1./16);
		cv::reprojectImageTo3D(staticDisparity , this->depth, rectifiedParam.Q , true ,CV_32F);
		
		//~ sleep(5); 
		
		//~ cv::split(this->depth,channels);
		//~ std::cout<<"Point: ("<< channels[0].at<float>(240,320) <<","<< channels[1].at<float>(240,320) <<","<< channels[2].at<float>(240,320) <<")"<<std::endl; // Depth is float
		
		//~ std::cout <<"    disparity:"<< disparity.at<float>(53,240) << std::endl;
		//~ std::cout<<"Point: ("<< this->depth.channels() <<std::endl; // Depth is float

		
		//~ cv::split(this->depth,channels);
		//~ temp = channels[2] ;
		//~ temp.convertTo(eqDepth,CV_8UC1);
		//~ cv::normalize(eqDepth,eqDepth,0,255,CV_MINMAX,CV_8U);
		
			
		cv::setMouseCallback("Disparity Map", &callBack, &this->depth);
		//~ cv::setMouseCallback("Disparity Map", &callBack, &channels);
		
		cv::imshow("Left Frame", leftRectified);
		//~ cv::imshow("Right Frame", rightRectified);
		
		//~ cv::imshow("Depth",eqDepth);

		if (cv::waitKey(30)>=0) break;
	
	}
	
}

void StereoCamera::test()
{
	cv::Mat left = cv::imread("left_frame.jpg",CV_LOAD_IMAGE_GRAYSCALE);
	cv::Mat right = cv::imread("right_frame.jpg",CV_LOAD_IMAGE_GRAYSCALE);
	
	
	
	CameraParameters leftParam = this->left.getCameraParameters();
    CameraParameters rightParam = this->right.getCameraParameters();
    cv::Mat leftRmap[2];
    cv::Mat rightRmap[2];
    cv::Mat disp;
    cv::Mat eqDepth;
    //~ cv::Mat dispFloat;
	
	std::vector<cv::Mat> channels;
	double min;
	double max;
	
	cv::Mat rightFrame;
	cv::Mat leftFrame;
	cv::Mat rightRectified;
	cv::Mat leftRectified;
	cv::Mat disparity;
	cv::Mat temp;
    

	
	cv::initUndistortRectifyMap(leftParam.cameraMatrix, leftParam.distortion, 
								rectifiedParam.R1, rectifiedParam.P1, cv::Size(640, 480), CV_16SC2,
								leftRmap[0], leftRmap[1]);
								
	cv::initUndistortRectifyMap(rightParam.cameraMatrix, rightParam.distortion, 
								rectifiedParam.R2, rectifiedParam.P2, cv::Size(640, 480), CV_16SC2,
								rightRmap[0], rightRmap[1]);
								
	cv::remap( left, leftRectified,  leftRmap[0], leftRmap[1], CV_INTER_LINEAR );
	cv::remap( right, rightRectified,  rightRmap[0], rightRmap[1], CV_INTER_LINEAR );
	
	
	//~ sgbm(leftRectified,rightRectified,disparity);
	sbm(leftRectified,rightRectified,disparity);
	//~ disparity.convertTo(disp,CV_8UC1);
	//~ cv::equalizeHist(disp,disp);
	cv::normalize(disparity,disp,0,255,CV_MINMAX,CV_8U);
	//~ disparity.convertTo(dispFloat,CV_8U,1/16);
	//~ cv::minMaxLoc(disparity,&min,&max);
	//~ double norm = 255 / max ; 
	//~ disp = disparity * norm / 255 ;
	//~ cv::reprojectImageTo3D(disparity , this->depth, rectifiedParam.Q , true ,CV_32F);
	
	//~ cv::Mat_<float> vec(4,1);
	//~ for(int y=0; y<dispFloat.rows; ++y) {
	    //~ for(int x=0; x<dispFloat.cols; ++x) {
	        //~ vec(0)=x; vec(1)=y; vec(2)=dispFloat.at<float>(y,x); vec(3)=1;
	        //~ vec = Q*vec;
	        //~ vec /= vec(3);
	        //~ cv::Vec3f &point = XYZ.at<cv::Vec3f>(y,x);
	        //~ point[0] = pt(0);
	        //~ point[1] = pt(1);
	        //~ point[2] = pt(2);
	    //~ }
	//~ }
	//~ cv::split(this->depth,channels);
	//~ temp = channels[2] ;
	//~ temp.convertTo(eqDepth,CV_8UC1);
	//~ cv::equalizeHist(eqDepth,eqDepth);
	//~ cv::imshow("Left Frame",left);
	//~ cv::imshow("Right Frame",right);
	cv::imshow("Disparity Map",disp);
	//~ cv::imshow("Depth",eqDepth);
	
	cv::waitKey();
	}



void StereoCamera::calibrate()
{
	std::cout << "Starting to calibrate the Stereo Camera ! " << std::endl;
	// Read the parameters for the calibration.
	inputParameters();
	
	collectScreensManually();
	
	left.calibrate();
	std::cout << "Calibrated Left" << std::endl;
	right.calibrate();
	std::cout << "Calibrated Right" << std::endl;

	
	CameraParameters leftParam = left.getCameraParameters();
	CameraParameters rightParam = right.getCameraParameters();

	
	double error = stereoCalibrate(left.getObjectPoints(),
	 left.getImagePoints() , right.getImagePoints() , 
	 leftParam.cameraMatrix, leftParam.distortion , 
	 rightParam.cameraMatrix, rightParam.distortion, 
	 cv::Size(640,480), 
	 parameters.Rotation , parameters.Translation , 
	 parameters.Essential , 
	 parameters.Fundamental,
	 cv::TermCriteria(CV_TERMCRIT_ITER+CV_TERMCRIT_EPS, 100, 1e-5),
		CV_CALIB_USE_INTRINSIC_GUESS);
                    //~ CV_CALIB_FIX_ASPECT_RATIO +
                    //~ CV_CALIB_ZERO_TANGENT_DIST +
                    //~ CV_CALIB_SAME_FOCAL_LENGTH +
                    //~ CV_CALIB_RATIONAL_MODEL +
                 //~ 
   //~ CV_CALIB_FIX_K3 + CV_CALIB_FIX_K4 + CV_CALIB_FIX_K5) ;
	
	
	std::cout << "Stereo Calibration Error : " <<  error << std::endl;
	 
}

void StereoCamera::rectify()
{
	CameraParameters paramLeft = left.getCameraParameters();
	CameraParameters paramRight = right.getCameraParameters();
	
	cv::Mat R1 , R2 , P1 , P2  , Q ; 
	cv::Rect validRoi[2];
	
	cv::stereoRectify(paramLeft.cameraMatrix , paramLeft.distortion , 
	paramRight.cameraMatrix , paramRight.distortion , cv::Size(640,480) , parameters.Rotation ,
	parameters.Translation , R1 , R2 , P1 , P2 , Q , cv::CALIB_ZERO_DISPARITY , 1 , 
	cv::Size(640,480) , &validRoi[0] , &validRoi[1] ) ;
	
	rectifiedParam.R1 = R1 ;
	rectifiedParam.R2 = R2 ;
	rectifiedParam.P1 = P1 ;
	rectifiedParam.P2 = P2 ;
	rectifiedParam.Q = Q;
	
	
	}
	

void StereoCamera::inputParameters()
{
	// Pattern parameters input .
	//~ std::cout << "Please input the parameters of the camera ." <<std::endl;
	//~ std::cout << "Please enter the type of the pattern you wish "<<
		//~ "to use for the Calibration" << std::endl ;
	//~ std::cout << "Press 1 for a chessboard pattern " << std::endl;
	//~ std::cout << "Press 2 for a symmetrical circle pattern " << std::endl;
	//~ std::cout << "Press 3 for an asymmetrical circle pattern " << std::endl;
	//~ std::cin >> calibParameters.patternType ;
	calibParameters.patternType = 1;
	
	//~ std::cout << "Enter the width of the pattern " << std::endl;
	//~ std::cin >> calibParameters.patternSize.width ;
	
	calibParameters.patternSize.width = 8;
	//~ calibParameters.patternSize.width = 4;
	
	//~ std::cout << "Enter the height of the pattern " << std::endl;
	//~ std::cin >> calibParameters.patternSize.height ;
	
	calibParameters.patternSize.height = 7;
	//~ calibParameters.patternSize.height = 11; // 11
	
	
	// We remove 1 tile from every dimension for the checkboard
	// pattern to ensure we take into account only the internal
	// corners.
	if ( calibParameters.patternType == 1 )
	{
		calibParameters.patternSize.width--;
		calibParameters.patternSize.height--;
	}
	
	switch( calibParameters.patternType)
	{
		case 1:
			//~ std::cout << "Give the distance between the corners of the chessboard. " << std::endl;
			//~ std::cin >> calibParameters.dist ;
			calibParameters.dist = 2.5;
			break;
		case 2:
		case 3:
			//~ std::cout << "Give the radius of the circles. " << std::endl;
			//~ std::cin >> calibParameters.dist ;
			calibParameters.dist = 0.5;
			break;
	}
	left.setCalibrationParameters(calibParameters);
	right.setCalibrationParameters(calibParameters);
	//~ std::cout << "Calibration ... " << std::endl;
}


void StereoCamera::calibrate(CameraParameters leftParam,CameraParameters rightParam)
{
	std::cout << "Starting to calibrate the Stereo Camera ! " << std::endl;
	// Read the parameters for the calibration.
	inputParameters();
	
	collectScreensManually();
	left.generate3dObjectPoints();
	right.generate3dObjectPoints();
	
	std::cout << "Left Screens " << left.getNumPoints() << std::endl;
	std::cout << "Right Screens " << right.getNumPoints() << std::endl;
	
	double error = stereoCalibrate(left.getObjectPoints(),
	 left.getImagePoints() , right.getImagePoints() , 
	 leftParam.cameraMatrix, leftParam.distortion , 
	 rightParam.cameraMatrix, rightParam.distortion, 
	 cv::Size(640,480), 
	 parameters.Rotation , parameters.Translation , 
	 parameters.Essential , 
	 parameters.Fundamental,
	 cv::TermCriteria(CV_TERMCRIT_ITER+CV_TERMCRIT_EPS, 500, 1e-5),
                    //~ CV_CALIB_FIX_ASPECT_RATIO +
                    //~ CV_CALIB_ZERO_TANGENT_DIST +
                    //~ CV_CALIB_SAME_FOCAL_LENGTH +
                    //~ CV_CALIB_RATIONAL_MODEL +
                 
   CV_CALIB_FIX_K3 + CV_CALIB_FIX_K4 + CV_CALIB_FIX_K5) ;
	
	
	std::cout << "Stereo Calibration Error : " <<  error << std::endl;
}

void StereoCamera::collectScreensManually()
{
	cv::VideoCapture leftCamera(left.getCameraID());
	cv::VideoCapture rightCamera(right.getCameraID());
	
	cv::Mat rightFrame;
	cv::Mat leftFrame;
	cv::Mat rightGray;
	cv::Mat leftGray;
	
	cv::Mat leftImg;
	cv::Mat rightImg;
	
	bool rightFound = false;

	bool leftFound = false;
	bool end = false;
	
	std::vector<cv::Point2f> leftPattern;
	std::vector<cv::Point2f> rightPattern;
	
	std::string leftWindow("Left Camera");
	std::string rightWindow("Right Camera");
	
	cv::namedWindow(leftWindow);
	cv::namedWindow(rightWindow);

	if ( !leftCamera.isOpened() || !rightCamera.isOpened() )
	{
		std::cout << "Error opening the cameras! The program is exiting!" << std::endl ;
		exit(1);
	}
	
	leftCamera.set(CV_CAP_PROP_FRAME_WIDTH, 640);
    leftCamera.set(CV_CAP_PROP_FRAME_HEIGHT,480);
    
    rightCamera.set(CV_CAP_PROP_FRAME_WIDTH, 640);
    rightCamera.set(CV_CAP_PROP_FRAME_HEIGHT,480);
    
    
	
	while( true )
	{
		leftCamera.grab();
		leftCamera.retrieve(leftFrame);
		rightCamera.grab();
		rightCamera.retrieve(rightFrame);
		
		// Detect Pattern in frame.
		// Convert image to gray scale.
		cv::cvtColor( rightFrame , rightGray , CV_BGR2GRAY );
		cv::cvtColor( leftFrame , leftGray , CV_BGR2GRAY );
		
		switch( calibParameters.patternType )
		{
			// Chessboard pattern case.
			case 1:
			// Find corners the corners of the chessboard .
				leftFound = cv::findChessboardCorners(leftGray, calibParameters.patternSize , leftPattern , 
				cv::CALIB_CB_ADAPTIVE_THRESH + cv::CALIB_CB_NORMALIZE_IMAGE + cv::CALIB_CB_FAST_CHECK );
				
				rightFound = cv::findChessboardCorners(rightGray, calibParameters.patternSize , rightPattern , 
				cv::CALIB_CB_ADAPTIVE_THRESH + cv::CALIB_CB_NORMALIZE_IMAGE + cv::CALIB_CB_FAST_CHECK );
				
				if ( leftFound && rightFound )
				{
					// Improve the accuracy of the corners found on the previous step.
					cv::cornerSubPix(leftGray, leftPattern, calibParameters.patternSize , cv::Size(-1, -1) ,
					  cv::TermCriteria(CV_TERMCRIT_EPS + CV_TERMCRIT_ITER, 30, 0.1)) ;
					  
					cv::cornerSubPix(rightGray, rightPattern, calibParameters.patternSize , cv::Size(-1, -1) ,
					  cv::TermCriteria(CV_TERMCRIT_EPS + CV_TERMCRIT_ITER, 30, 0.1)) ;
					    
					break;
				}
				
				
			// Symmetrical circle grid pattern case.
			case 2:
			case 3:
				// Detect the centers of the circles of an asymmetrical pattern.
				leftFound = cv::findCirclesGrid(leftGray, calibParameters.patternSize , leftPattern , cv::CALIB_CB_ASYMMETRIC_GRID );
				
				rightFound = cv::findCirclesGrid(rightGray, calibParameters.patternSize , rightPattern , cv::CALIB_CB_ASYMMETRIC_GRID );
				
				break;
		}
		
		leftImg = leftFrame.clone();
		rightImg = rightFrame.clone();
		cv::drawChessboardCorners(leftFrame, calibParameters.patternSize , leftPattern , leftFound );
		cv::drawChessboardCorners(rightFrame, calibParameters.patternSize , rightPattern , leftFound );

		cv::imshow(leftWindow,leftFrame) ;
		cv::imshow(rightWindow,rightFrame) ;

		
		int key = cv::waitKey(10) & 255 ;
		
		switch(key)
		{
		// Escape key has been pressed
		// The program will be stopped.
			case 27:
				std::cout << "Program is exiting! " << std::endl;
				exit(1);
			
		// The key "S" has been pressed
			case 's':
				if (!leftFound || !rightFound)
				{
					std::cout << "Invalid Screenshot !" << std::endl;
					break;
				}
				// If the pattern has been located store the result.
				left.pushPattern(leftPattern);
				right.pushPattern(rightPattern);
				left.pushScreen(leftImg);
				right.pushScreen(rightImg);
				std::cout << "Saved Screenshot!" << std::endl ;
				break;						
		// The screenshot collection routine is terminated.
		// The camera will be calibrated with the screenshots 
		// that were collected.
			case 'c':
				std::cout << "Screenshot collection finished!" << std::endl;
				std::cout << "Proceeding to calibration!" << std::endl;
				cv::destroyWindow(leftWindow);
				cv::destroyWindow(rightWindow);
				end = true ;
				break;
			case 'n':
				std::cout << "Number of screenshots : " << left.getNumPoints() << std::endl;
				break;
		}
		if (end) break;
	}
	

	
}

void StereoCamera::getCameraID(int* left,int* right)
{
	cv::VideoCapture camera(0);
	if ( !camera.isOpened() )  
	{
	    std::cout << "Cannot open the video file" << std::endl;
	    //~ return -1;
	}	
	cv::Mat frame;
	std::string windowName("Video Input");
	cv::namedWindow(windowName,true);
	int key;
	std::cout << "Is this the left camera ? [Y/N] " << std::endl;
	int code ;
	while (1)
	{
		camera.grab();
		camera.retrieve(frame);
		
		cv::imshow(windowName,frame);
		key = cv::waitKey(30) & 255 ;	
		switch(key)
		{
			case 'y':
			case 'Y':
				cv::destroyAllWindows();
				//~ return 0;
			case 'N':
			case 'n':
				cv::destroyAllWindows();
				//~ return 1;
			default:
				continue;
		}
		
		
	}
}

void StereoCamera::readXML()
{
	cv::FileStorage fs(this->fileName+".xml",cv::FileStorage::READ);
	if (!fs.isOpened())
	{
		std::cerr << "Failed to open " << this->fileName << std::endl;
		exit(1);
	}
	fs["Fundamental"] >> parameters.Fundamental ;
	fs["R1"] >> rectifiedParam.R1 ;
	fs["P1"] >> rectifiedParam.P1 ;
	fs["R2"] >> rectifiedParam.R2 ;
	fs["P2"] >> rectifiedParam.P2 ;
	fs["Q"] >>  rectifiedParam.Q ;
	fs.release();
	}
	
void StereoCamera::writeXML()
{
	cv::FileStorage fs(fileName+".xml",cv::FileStorage::WRITE);

	fs << "R1" << rectifiedParam.R1 ;
	fs << "P1" << rectifiedParam.P1 ;
	fs << "R2" << rectifiedParam.R2 ;
	fs << "P2" << rectifiedParam.P2 ;
	fs << "Q" << rectifiedParam.Q ;
	fs << "Fundamental" << parameters.Fundamental ;
	fs.release();
	}
	//~ 
//~ //mouse event handler to indicate pixels(2D) values
//~ void  StereoCamera::mouseEvent(int event, int x,int y, int flags, void* param)
//~ {
    //~ cv::Mat* rgb = (cv::Mat*) param;
    //~ if (event == CV_EVENT_LBUTTONDOWN)
    //~ {
        //~ pointyup=y;
        //~ pointxup=x;
       //~ // cout<<y<<endl;
       //~ // cout<<x<<endl;
    //~ }
    //~ if (event == CV_EVENT_RBUTTONDOWN)
    //~ {
        //~ pointydown=y;
        //~ pointxdown=x;
        //~ //cout<<y<<endl;
       //~ // cout<<x<<endl;
    //~ }
//~ }
//~ void StereoCamera::averageValueCalculator(cv::Mat pointCloud)
//~ {
    //~ double average=0;
    //~ float sumx=0;
    //~ float sumy=0;
    //~ float sumz=0;
    //~ int i,j;
    //~ int counter=0;
    //~ for(i=pointxup;i<pointxdown;i++)
    //~ {
        //~ for(j=pointyup;j<pointydown;j++)
        //~ {
           //~ if((!isinf(pointCloud.at<cv::Vec3f>(i,j)[0])) || (!isinf(pointCloud.at<cv::Vec3f>(i,j)[1])) || (!isinf(pointCloud.at<cv::Vec3f>(i,j)[2])))
        //~ {
         //~ sumx+=pointCloud.at<cv::Vec3f>(i,j)[0];
         //~ sumy+=pointCloud.at<cv::Vec3f>(i,j)[1];
         //~ sumz+=pointCloud.at<cv::Vec3f>(i,j)[2];
         //~ counter++;
        //~ }
      //~ }
    //~ }
   //~ averagex=sumx/counter;
   //~ averagey=sumy/counter;
   //~ averagez=sumz/counter;
//~ }
//~ void StereoCamera::setAverages(cv::Mat frame,int pointxup, int pointyup, int pointxdown, int pointydown)
//~ {
    //~ int i,j;
//~ 
    //~ cv::Point pt1(pointxup,pointyup);
    //~ cv::Point pt2(pointxdown,pointydown);
    //~ rectangle(frame,pt1,pt2,cv::Scalar(0, 255, 0), 4 );
//~ 
     //~ averageValueCalculator(depthmap);
     //~ std::cout<<"Average Vector Values"<<std::endl;
     //~ std::cout<<averagex<<std::endl;
     //~ std::cout<<averagey<<std::endl;
     //~ std::cout<<averagez<<std::endl;
//~ }
 //~ void StereoCamera::markTheCenter(cv::Mat frame, cv::Mat Q)
//~ {
    //~ int centerx,centery,cx,cy;
    //~ double f,a,b;
    //~ int radius=3;
    //~ int lineType=8;
    //~ int shift=0;
    //~ int thickness=10;
     //~ cx=Q.at<double>(0,3);
     //~ cy=Q.at<double>(1,3);
     //~ f=Q.at<double>(2,3);
     //~ a=Q.at<double>(3,2);
     //~ b=Q.at<double>(3,3);
      //~ //find center (0,0)
     //~ centerx=-cx;
     //~ centery=-cy;
     //~ cv::Point center(centery,centerx);
     //~ cv::Point xaxis1(centery,0);
     //~ cv::Point xaxis2(centery,width);
     //~ cv::Point yaxis1(0,centerx);
     //~ cv::Point yaxis2(width,centerx);
     //~ //blue
     //~ cv::line(frame,xaxis1,center,cv::Scalar(115, 10, 0), 2 );
     //~ cv::line(frame,center,xaxis2,cv::Scalar(115, 10, 0), 2 );
     //~ //light blue
     //~ cv::line(frame,yaxis1,center,cv::Scalar(200, 99, 50), 2 );
     //~ cv::line(frame,center,yaxis2,cv::Scalar(200, 99, 50), 2 );
     //~ cv::circle(frame,center,radius,cv::Scalar(0, 0, 100), 4,lineType,shift);
     //~ cv::Point param1((width/2)-50,15);
     //~ cv::Point param2((width/2)-50,460);
     //~ cv::Point param3(0,height/2);
     //~ cv::Point param4(width-150,height/2);
      //~ cv::putText(frame,"y axis- cam",param1, cv::FONT_HERSHEY_COMPLEX_SMALL, 0.8, cv::Scalar(200,200,250));
      //~ cv::putText(frame,"Y axis+ cam",param2, cv::FONT_HERSHEY_COMPLEX_SMALL, 0.8, cv::Scalar(200,200,250));
      //~ cv::putText(frame,"x axis- cam",param3, cv::FONT_HERSHEY_COMPLEX_SMALL, 0.8, cv::Scalar(200,200,250));
      //~ cv::putText(frame,"x axis+ cam",param4, cv::FONT_HERSHEY_COMPLEX_SMALL, 0.8, cv::Scalar(200,200,250));
//~ }
