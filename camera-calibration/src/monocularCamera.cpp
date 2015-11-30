#include "monocularCamera.h"

MonocularCamera::MonocularCamera(int id)
{
	// Save camera ID.
	this->cameraID = id ;
	std::cout << "Starting camera Calibration!" << std::endl;
	InputParameters();
}

void MonocularCamera::InputParameters()
{
	// Pattern parameters input .
	std::cout << "Please input the parameters of the camera ." <<std::endl;
	std::cout << "Please enter the type of the pattern you wish "<<
		"to use for the Calibration" << std::endl ;
	std::cout << "Press 1 for a chessboard pattern " << std::endl;
	std::cout << "Press 2 for a symmetrical circle pattern " << std::endl;
	std::cout << "Press 3 for an asymmetrical circle pattern " << std::endl;
	std::cin >> calibParameters.patternType ;
	
	std::cout << "Enter the width of the pattern " << std::endl;
	std::cin >> calibParameters.patternSize.width ;
	
	std::cout << "Enter the height of the pattern " << std::endl;
	std::cin >> calibParameters.patternSize.height ;
	
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
			std::cout << "Give the distance between the corners of the chessboard. " << std::endl;
			std::cin >> calibParameters.dist ;
			break;
		case 2:
		case 3:
			std::cout << "Give the radius of the circles. " << std::endl;
			std::cin >> calibParameters.dist ;
			break;
	}
}

//  TO DO : Read parameters from file.
MonocularCamera::MonocularCamera(int id,std::string fileName)
{
	this->fileName = fileName ;
	this->cameraID = id ;
	this->readXML();
}

void MonocularCamera::calibrate()
{
	// Camera Calibration
	std::vector<cv::Mat> tvecs;
	std::vector<cv::Mat> rvecs;
	
	parameters.cameraMatrix = cv::Mat();
	parameters.distortion = cv::Mat();
	
	this->generate3dObjectPoints();

	double error = calibrateCamera(objectPoints , imagePoints, cv::Size(640,480) , parameters.cameraMatrix 
	 , parameters.distortion , tvecs, rvecs );
	 std::cout << "Calibration error : " << error << std::endl;
	
	// Save camera parameters to xml file .
	writeXML();
	std::cout << "Would you like to save the image frames ? [Y/N] " << std::endl;
	int key = cv::waitKey() & 255 ;
	
	switch(key)
	{
		case 'y':
		case 'Y':
			saveScreens();
			break;
		case 'N':
		case 'n':
		default:
			break;
	}

	
	std::cout << std::endl << "Finished calibrating " << fileName << " camera." << std::endl;	
	std::cout << "Files have been saved to " << this->fileName << std::endl;
}

void MonocularCamera::calibrate(std::string fileName)
{
	
	// Screenshot Collection 
	
	int answer;
	std::cout << "Do you want the screenshots to be collected manually ? " 
	  << std::endl ;
	std::cout << "Press 1 if yes , 0 if no ." << std::endl;
	std::cin >>  answer ;
	
	switch(answer)
	{
		case 1: 
			this->collectScreensManually();
			break;
		case 0:
			this->autoCollectScreens();
			break;
		default:
			std::cout << "Wrong input. The program has to exit. " <<std::endl;
			exit(1);
	}
	
	// Object point generation. (3d points)
	// Must generate the same number of objects as the number of image
	// points.
	
	this->generate3dObjectPoints();
	
	// Camera Calibration
	std::vector<cv::Mat> tvecs;
	std::vector<cv::Mat> rvecs;

	double error = calibrateCamera(objectPoints,imagePoints, cv::Size(640,480) , parameters.cameraMatrix 
	 , parameters.distortion , tvecs, rvecs );
	 std::cout << this->fileName << " Calibration error : " << error << std::endl;
	
	// Save camera parameters to xml file .
	writeXML();
	std::cout << "Would you like to save the image frames ? [Y/N] " << std::endl;
	char key = cv::waitKey() & 255 ;
	
	switch(key)
	{
		case 'y':
		case 'Y':
			saveScreens();
			break;
		case 'N':
		case 'n':
		default:
			break;
	}

	
	std::cout << std::endl << "Finished calibrating camera." << std::endl;
	
	std::cout << "Files have been saved to " << this->fileName << std::endl;
	
}

void MonocularCamera::saveScreens()
{
	std::string str;
	std::stringstream s;	
	for ( int i = 0 ; i < frames.size() ; i++ )
	{
			s<<i+1;
			str += s.str();
			str +=".jpg" ;
			std::cout << "Saving image " << i << std::endl;				
			cv::imwrite(fileName+str,frames[i]);
			str = std::string("");
			s.str("");
	}
	std::cout << "Saved images! " << std::endl;

}


void MonocularCamera::pushPattern(std::vector<cv::Point2f> pattern)
{
	imagePoints.push_back(pattern);
}

void MonocularCamera::pushScreen(cv::Mat img)
{
	frames.push_back(img);
}



// Function used to collect screenshots from camera.

void MonocularCamera::collectScreensManually()
{
	cv::VideoCapture camera( this->cameraID );
	cv::Mat frame;
	cv::Mat gray ; 
	std::string windowName("Video Input");
	cv::namedWindow(windowName,true);
	std::vector<cv::Point2f> pattern;
	bool found = false ;
	bool end = false;
	int key;
	int count = 0 ;
	cv::Mat img;
	
	// Open the camera.
	if (!camera.isOpened() )
	{
		std::cout << "Error opening the camera! The program is exiting!" << std::endl ;
		exit(1);
	}
	
	camera.set(CV_CAP_PROP_FRAME_WIDTH, 640);
    camera.set(CV_CAP_PROP_FRAME_HEIGHT,480);
	
	// Print instructions.
	std::cout << "Press S to capture frame " << std::endl;
	std::cout << "Press C to stop capturing frames and to start the calibration ." << std::endl;
	std::cout << "Press escape if you want to exit the program. " << std::endl;
	
	while ( true )
	{
		// Get grame.
		camera.grab();
		camera.retrieve(frame) ; 
		
		// Detect Pattern in frame.
		// Convert image to gray scale.
		cv::cvtColor( frame , gray , CV_BGR2GRAY );
		
		
		switch( calibParameters.patternType )
		{
			// Chessboard pattern case.
			case 1:
			// Find corners the corners of the chessboard .
				found = cv::findChessboardCorners(gray, calibParameters.patternSize , pattern , 
				cv::CALIB_CB_ADAPTIVE_THRESH + cv::CALIB_CB_NORMALIZE_IMAGE + cv::CALIB_CB_FAST_CHECK );
				if (found)
				{
					// Improve the accuracy of the corners found on the previous step.
					cv::cornerSubPix(gray,pattern, calibParameters.patternSize , cv::Size(-1, -1) ,
					  cv::TermCriteria(CV_TERMCRIT_EPS + CV_TERMCRIT_ITER, 30, 0.1)) ;
					break;
				}
				
				
			// Symmetrical circle grid pattern case.
			case 2:
			case 3:
				// Detect the centers of the circles of an asymmetrical pattern.
				found = cv::findCirclesGrid(gray, calibParameters.patternSize , pattern , cv::CALIB_CB_ASYMMETRIC_GRID );
				break;
		}	
		img = frame.clone();
		cv::drawChessboardCorners(frame, calibParameters.patternSize , pattern , found );
		cv::imshow(windowName,frame) ;
		
		key = cv::waitKey(10) & 255 ;
		
		switch(key)
		{
		// Escape key has been pressed
		// The program will be stopped.
			case 27:
				std::cout << "Program is exiting! " << std::endl;
				exit(1);
			
		// The key "S" has been pressed
			case 's':
				if (!found)
				{
					std::cout << "Invalid Screenshot !" << std::endl;
					break;
				}
				// If the pattern has been located store the result.
				//~ imagePoints.push_back(pattern);	
				pushPattern(pattern);
				frames.push_back(img);			
				std::cout << "Saved Screenshot!" << std::endl ;
				break;
				
				
				
		// The screenshot collection routine is terminated.
		// The camera will be calibrated with the screenshots 
		// that were collected.
			case 'c':
				std::cout << "Screenshot collection finished!" << std::endl;
				std::cout << "Proceeding to calibration!" << std::endl;
				end = true ;
				break;
			case 'n':
				std::cout << "Number of screenshots : " << imagePoints.size() << std::endl;
				break;
		}
		if (end) break;
	}
	
}


// Function used to automatically gather screenshots every DELAY seconds.
void MonocularCamera::autoCollectScreens()
{
	cv::VideoCapture camera( this->cameraID );
	cv::Mat frame;
	cv::Mat gray ; 
	std::string windowName("Video Input");
	cv::namedWindow(windowName,true);
	std::vector<cv::Point2f> pattern;
	bool found = false ;
	bool end = false;
	int key ;
	
	time_t current ;
	time_t previous ;
	
	// Open the camera.
	if (!camera.isOpened() )
	{
		std::cout << "Error opening the camera! The program is exiting!" << std::endl ;
		exit(1);
	}
	
	std::cout << "Press N to print the number of screens collected. " << std::endl;
	std::cout << "Press C to stop capturing frames and to start the calibration ." << std::endl;
	std::cout << "Press escape if you want to exit the program. " << std::endl;
	previous = time(NULL);
	
	while ( true )
	{
		// Get frame.
		camera.grab();
		camera.retrieve(frame) ; 
		
		// Detect Pattern in frame.
		// Convert image to gray scale.
		cv::cvtColor( frame , gray , CV_BGR2GRAY );
		
		
		switch( calibParameters.patternType )
		{
			// Chessboard pattern case.
			case 1:
			// Find corners the corners of the chessboard .
				found = cv::findChessboardCorners(gray, calibParameters.patternSize , pattern , 
				cv::CALIB_CB_ADAPTIVE_THRESH + cv::CALIB_CB_NORMALIZE_IMAGE + cv::CALIB_CB_FAST_CHECK );
			if (found)
			{
				// Improve the accuracy of the corners found on the previous step.
				cv::cornerSubPix(gray,pattern, calibParameters.patternSize , cv::Size(-1, -1) ,
				  cv::TermCriteria(CV_TERMCRIT_EPS + CV_TERMCRIT_ITER, 30, 0.1)) ;
				break;
			}
				
				
			// Symmetrical circle grid pattern case.
			case 2:
				// Detect the centers of the circles of a symmetrical pattern.
				found = cv::findCirclesGrid(gray, calibParameters.patternSize , pattern , cv::CALIB_CB_SYMMETRIC_GRID );
				break;
			
			// Asymmetrical circle grid pattern case.
			case 3:
				// Detect the centers of the circles of an asymmetrical pattern.
				found = cv::findCirclesGrid(gray, calibParameters.patternSize , pattern , cv::CALIB_CB_ASYMMETRIC_GRID );
				break;
			
		}
		// Draw the detected corners on the image.
		
		drawChessboardCorners(frame, calibParameters.patternSize , pattern , found );
		cv::imshow(windowName,frame) ;
		
		key = cv::waitKey(10) & 255 ;
		
		switch(key)
		{
		// Escape key has been pressed
		// The program will be stopped.
			case 27:
				std::cout << "Program is exiting! " << std::endl;
				exit(1);
			
		// The key "N" has been pressed and we print the number of
		// screenshots gathered.
			case 'n':
				std::cout << "Number of frames " << this->imagePoints.size() << std::endl ;		
				break;
		// Calibrate the camera.
			case 'c':
				std::cout << "Screenshot collection terminated by user." << std::endl;
				std::cout << "Proceeding to calibration!" << std::endl;
				end = true ;
				break;
			default:
				break;
		}
		current = time(NULL);
		// Check for valid frame every 3 seconds,so as to give
		// the user time to move the calibration pattern.
		if (current - previous >= DELAY && found )
		{
			previous = current ; 
			imagePoints.push_back(pattern);
			std::cout << "Saved Screenshot!" << std::endl ;
		}
		
		if ( this->imagePoints.size() > MAX_SCREENS || end) break;
	}
}


// Function used to generate the coordinates of the pattern in 3D space.

void MonocularCamera::generate3dObjectPoints()
{
	std::vector<cv::Point3f> obj;
   
    
    switch ( calibParameters.patternType )
    {
		// Chessboard and symmetrical circle grid case .
		case 1:
		case 2:
			for( int j = 0; j < calibParameters.patternSize.height; j++ )
	        {
	          for( int k = 0; k < calibParameters.patternSize.width ; k++ )
	          {
				obj.push_back(cv::Point3f(float(j*calibParameters.dist), float(k*calibParameters.dist), 0));
	          }
	        }
			break;
			
		// Asymmetrical circle grid case .
		case 3:
			for( int i = 0; i < calibParameters.patternSize.height ; i++ )
	        {
	            for( int j = 0; j < calibParameters.patternSize.width ; j++ )
	            {
					obj.push_back(cv::Point3f(float((2*j + i % 2)*calibParameters.dist), float(i*calibParameters.dist), 0));
	            }
	        }
	        break;
	}
	
	for ( int i = 0 ; i < imagePoints.size() ; i++ )
	{
		this->objectPoints.push_back(obj);
	}
}


void MonocularCamera::readXML()
{
	cv::FileStorage fs(fileName+".xml",cv::FileStorage::READ);
	fs["Camera_Matrix"] >> parameters.cameraMatrix;
	fs["Distortion_Coefficients"] >> parameters.distortion;
	fs.release();
}

void MonocularCamera::writeXML()
{
	cv::FileStorage fs(fileName+".xml",cv::FileStorage::WRITE);
	fs << "Camera_Matrix" << parameters.cameraMatrix ;
	fs << "Distortion_Coefficients" << parameters.distortion ;
	fs.release();
}
