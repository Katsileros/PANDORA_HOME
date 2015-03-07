#include "BackgroundSubtraction.h"

BackgroundSubtraction::BackgroundSubtraction()
{	
	thresh = 100;
	maxThresh = 255;
	rng(12345);
}

void BackgroundSubtraction::removeBackGround()
{
	int i=0, j=0;
	
	cv::Mat rgb,depth;
	rgb = getRgb();
	depth = getDepth();
	
	cv::namedWindow("rgb");
	//~ cv::namedWindow("depth");
	
	cv::imshow("rgb",rgb);
	cv::waitKey();
 	
 	unsigned short min = (short(INT_MAX)), max = 0;
	for(i=0;i<480;i++)
	{
		for(j=0;j<640;j++)
		{
			unsigned short depth_value = depth.at<unsigned short>(i, j);
			
			if(depth_value > max)
				max = depth_value;
			if(depth_value < min)
				min = depth_value;
		}
	}
 
	//~ std::cout << "min: " << min << std::endl;
	//~ std::cout << "max: " << max << std::endl;
	
	for(i=0;i<480;i++)
	{
		for(j=0;j<640;j++)
		{
			cv::Vec3b depth_vector = rgb.at<cv::Vec3b>(i, j);
			unsigned short depth_value = depth.at<unsigned short>(i, j);
			
			if((depth_value < 460) || (depth_value > 980))
			{	
				depth_vector.val[0] = 255;
				depth_vector.val[1] = 255;
				depth_vector.val[2] = 255;
				rgb.at<cv::Vec3b>(i, j) = depth_vector;
			}
		}
	}
	
	cv::imshow("rgb",rgb);
	cv::waitKey();
	
	
	for(i=0;i<480;i++)
	{
		for(j=0;j<640;j++)
		{
			cv::Vec3b rgb_color_vector = rgb.at<cv::Vec3b>(i, j);
			
			if( (rgb_color_vector.val[2] >= 180) || (rgb_color_vector.val[2] <= 30) ) // red
			 //~ || (rgb_color_vector.val[1] >= 80 )  // green
			 //~ || (rgb_color_vector.val[0] >= 70) )// blue
			{
				rgb_color_vector.val[0] = 255; // blue
				rgb_color_vector.val[1] = 255; // green
				rgb_color_vector.val[2] = 255; // red
				//~ //rgb.at<cv::Vec3b>(i, j) = rgb_color_vector;
			}
			
			if( (rgb_color_vector.val[2] >= 90)   // red
			 && (rgb_color_vector.val[1] >= 90)   // green
			 && (rgb_color_vector.val[0] >= 90) ) // blue
			{
				rgb_color_vector.val[0] = 255; // blue
				rgb_color_vector.val[1] = 255; // green
				rgb_color_vector.val[2] = 255; // red
				//~ //rgb.at<cv::Vec3b>(i, j) = rgb_color_vector;
			}
			
			rgb.at<cv::Vec3b>(i, j) = rgb_color_vector;
		
		}
	}
	
	cv::imshow("rgb",rgb);
	
	erosion_elem = 2;
	erosion_size = 4;
	max_elem = 2;
	max_kernel_size = 21;
	
	//~ /// Create windows
	//~ cv::namedWindow( "Erosion Demo", CV_WINDOW_AUTOSIZE );
//~ 
	//~ /// Create Erosion Trackbar
	//~ cvCreateTrackbar( "Element:\n 0: Rect \n 1: Cross \n 2: Ellipse", "Erosion Demo",
                  //~ &erosion_elem, max_elem);
                  //~ //erosion );
 //~ 
	//~ cvCreateTrackbar( "Kernel size:\n 2n +1", "Erosion Demo",
                  //~ &erosion_size, max_kernel_size);
                  //~ //erosion );
           
    erosion(0,0);      
            
    //~ for(;;)
	//~ {
		//~ erosion(0,0);
		//~ if (cv::waitKey() == 27) break; ///press Esc to terminate the procedure
	//~ }
	
	
	cv::waitKey();
	
	//~ return 0;
	
}


void BackgroundSubtraction::thresholdImg(cv::Mat img)
{	
	setImgThresholded( img );
	cv::imshow("imgThresholded", imgThresholded_);
	cv::waitKey();
	
	//~ /// Create Window
	std::string source_window = "Source";
	cv::namedWindow( source_window, CV_WINDOW_AUTOSIZE );
	//~ cv::imshow( source_window, cv::imread("rgb.png") );
	cv::imshow( source_window, imgThresholded_ );
	
	//~ cv::imwrite("imgThresholded.png", imgThresholded);

	cvCreateTrackbar( " Threshold:", "Source", &thresh, maxThresh);
	
	/// Convert image to gray and blur it
	//~ cv::cvtColor( imgThresholded, src_gray, CV_BGR2GRAY );
	//~ cv::blur( src_gray, src_gray, cv::Size(3,3) );
	setSrcGray( imgThresholded_ );
	cv::cvtColor( srcGray_, srcGray_, CV_BGR2GRAY );
	cv::blur( srcGray_, srcGray_, cv::Size(3,3) );
	
	for(;;)
	{
		threshCallback( 0, 0 );
		if (cv::waitKey() == 27) break; ///press Esc to terminate the procedure
		}
}


void BackgroundSubtraction::threshCallback(int, void* )
{	
	cv::Mat threshold_output;
	std::vector<std::vector<cv::Point> > contours;
	std::vector<cv::Vec4i> hierarchy;
  
	int KeyPressed=255;

	/// Detect edges using Threshold
	cv::threshold( getSrcGray(), threshold_output, thresh, 255, cv::THRESH_BINARY );
	/// Find contours
	cv::findContours( threshold_output, contours, hierarchy, CV_RETR_TREE, CV_CHAIN_APPROX_SIMPLE, cv::Point(0, 0) );
	
	/// Approximate contours to polygons + get bounding rects and circles
	std::vector<std::vector<cv::Point> > contours_poly( contours.size() );
	std::vector<cv::Rect> boundRect( contours.size() );
	std::vector<cv::Point2f>center( contours.size() );
	std::vector<float>radius( contours.size() );
	
	for( int i = 0; i < contours.size(); i++ )
	{  
		cv::approxPolyDP( cv::Mat(contours[i]), contours_poly[i], 3, true );
		boundRect[i] = boundingRect( cv::Mat(contours_poly[i]) );
		minEnclosingCircle( (cv::Mat)contours_poly[i], center[i], radius[i] );
    }


	/// Draw polygonal contour + bonding rects + circles
	//~ cv::Mat drawing = cv::Mat::zeros( threshold_output.size(), CV_8UC3 );
	cv::Mat drawing = getImgThresholded().clone();
	for( int i = 0; i< contours.size(); i++ )
	{
       cv::Scalar color = cv::Scalar( rng.uniform(0, 255), rng.uniform(0,255), rng.uniform(0,255) );
       cv::drawContours( drawing, contours_poly, i, color, 1, 8, std::vector<cv::Vec4i>(), 0, cv::Point() );
       cv::rectangle( drawing, boundRect[i].tl(), boundRect[i].br(), color, 2, 8, 0 );
       cv::circle( drawing, center[i], (int)radius[i], color, 2, 8, 0 );
     }

	/// Show in a window
	cv::namedWindow( "Contours", CV_WINDOW_AUTOSIZE );
	for(;;)
	{
		cv::imshow( "Contours", drawing );
		cv::imwrite( "./contours.png", drawing);
	
        if (cv::waitKey() == 27) break; ///press Esc to terminate the procedure
	}
}


void BackgroundSubtraction::erosion( int, void* )
{
  cv::Mat erosion_dst,src;

  int erosion_type;
  if( erosion_elem == 0 ){ erosion_type = cv::MORPH_RECT; }
  else if( erosion_elem == 1 ){ erosion_type = cv::MORPH_CROSS; }
  else if( erosion_elem == 2) { erosion_type = cv::MORPH_ELLIPSE; }

  cv::Mat element = getStructuringElement( erosion_type,
                                       cv::Size( 2*erosion_size + 1, 2*erosion_size+1 ),
                                       cv::Point( erosion_size, erosion_size ) );

  /// Apply the erosion operation
  src = getRgb();
  cv::erode( src, erosion_dst, element );
  imshow( "Erosion Demo", erosion_dst );
  cv::waitKey();
  setRgb(erosion_dst);
}
