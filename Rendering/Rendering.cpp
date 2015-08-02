#include <vtkPolyDataMapper.h>
#include <vtkActor.h>
#include "vtkMatrix4x4.h"
#include "vtkMatrix3x3.h"
#include <vtkSmartPointer.h>
#include <vtkRenderWindow.h>
#include <vtkRenderer.h>
#include <vtkRenderWindowInteractor.h>
#include <vtkPolyData.h>
#include <vtkCamera.h>
#include <vtkWindowToImageFilter.h>
#include <vtkPNGWriter.h>
#include <vtkXMLPolyDataWriter.h>
#include <vtkTransform.h>
#include <vtkTransformFilter.h>
#include <vtkTransformPolyDataFilter.h>

#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/surface/vtk_smoothing/vtk_utils.h>
#include <pcl/io/ply_io.h> 
#include <pcl/io/vtk_io.h>
#include <pcl/io/vtk_lib_io.h> 
#include <pcl/visualization/cloud_viewer.h>

#include "opencv/cv.h"
#include "opencv2/opencv.hpp"
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/core/core.hpp"

#include <boost/filesystem.hpp>
#include <vector>
#include <iostream>
#include <string>
#include <fstream>

vtkSmartPointer<vtkMatrix4x4> readPose(std::string foldername,int k);
vtkSmartPointer<vtkMatrix4x4> readTransofrm(std::string folder);
cv::Mat loadDepth( std::string a_name , std::string filename);

int main(int, char *[])
{
	//!< set virtual camera parameters
    const double zNear = 0.1;//!< near clipping plane
    const double zFar = 10000.0;//!< far clipping plane
    const double height = 640;
    const double width = 480;
    const double fy = 573.57;//!< focal length at Y axis
    double fovy = 2 * atan( (height/2) / fy );//!< the camera view angle

    vtkSmartPointer<vtkCamera> camera = vtkSmartPointer<vtkCamera>::New();
    //!< approximate the transform of xtion to virtual camera
    camera->SetClippingRange(zNear, zFar);
    camera->SetViewAngle( fovy * 180/M_PI );

    mkdir("poses", 0777);
    
	//!< load Data
	vtkSmartPointer<vtkPLYReader> reader =
    vtkSmartPointer<vtkPLYReader>::New();
	reader->SetFileName("driller/mesh.ply");
	reader->Update();

    for(int k=0;k<3;k++)
    {	
     
    vtkSmartPointer<vtkMatrix4x4> m = vtkSmartPointer<vtkMatrix4x4>::New();
    std::string foldername = "driller/data/";
	m = readPose(foldername,k);
    
    vtkSmartPointer<vtkTransformPolyDataFilter> transformFilter = vtkSmartPointer<vtkTransformPolyDataFilter>::New();
    transformFilter->SetInputConnection(reader->GetOutputPort());
    
    vtkSmartPointer<vtkTransform> transform = vtkSmartPointer<vtkTransform>::New();
	transform -> Identity();
	transform->SetMatrix(m);
	transform -> Inverse();
	
    transformFilter->SetTransform(transform);
    transformFilter->Update();
    
    vtkSmartPointer<vtkTransformPolyDataFilter> KinectTransformFilter = vtkSmartPointer<vtkTransformPolyDataFilter>::New();
    KinectTransformFilter->SetInputConnection(transformFilter->GetOutputPort());
    
    vtkSmartPointer<vtkTransform> KinectTransform = vtkSmartPointer<vtkTransform>::New();	
    KinectTransform -> Identity();
	KinectTransform->RotateX(180.0);
	
	KinectTransformFilter->SetTransform(KinectTransform);
	KinectTransformFilter->Update();
	
	vtkSmartPointer<vtkPolyDataMapper> mapper = vtkSmartPointer<vtkPolyDataMapper>::New();
	mapper->SetInputConnection(KinectTransformFilter->GetOutputPort());
	//~ mapper->SetInputConnection(reader->GetOutputPort());
	mapper->Update();
	
	vtkSmartPointer<vtkActor> actor = vtkSmartPointer<vtkActor>::New();
	actor->SetMapper(mapper);

	vtkSmartPointer<vtkRenderWindow> renderWindow = vtkSmartPointer<vtkRenderWindow>::New();
	//~ vtkSmartPointer<vtkRenderWindowInteractor> renderWindowInteractor = vtkSmartPointer<vtkRenderWindowInteractor>::New();
	vtkSmartPointer<vtkRenderer> renderer = vtkSmartPointer<vtkRenderer>::New();

	renderWindow->SetSize(height, width);
	renderWindow->AddRenderer(renderer);
	//~ renderWindowInteractor->SetRenderWindow(renderWindow);
	
	renderer->AddActor(actor);
	renderer->SetActiveCamera(camera);
	renderer->SetBackground(1,1,1); // Background color white
	
	renderWindow->Render();

	vtkSmartPointer<vtkWindowToImageFilter> windowToImageFilter = vtkSmartPointer<vtkWindowToImageFilter>::New();
	// Screenshot  
	windowToImageFilter->SetInput(renderWindow);
	windowToImageFilter->SetMagnification(1); //set the resolution of the output image ('x1' times the current resolution of vtk render window)
	windowToImageFilter->Update();
	
	vtkSmartPointer<vtkPLYWriter> writer = vtkSmartPointer<vtkPLYWriter>::New();
	
	std::string filename = "poses/screenshot" + boost::lexical_cast<std::string>(k) + ".ply";
	writer->SetFileName(filename.c_str());
	writer->SetFileTypeToASCII();
	writer->SetInputConnection(KinectTransformFilter->GetOutputPort());
	writer->Write(); 
	
	//~ renderWindowInteractor->Start();
	
	}
	
	return EXIT_SUCCESS;
	
}

vtkSmartPointer<vtkMatrix4x4> readPose(std::string foldername,int k)
{	
	vtkSmartPointer<vtkMatrix4x4> m = vtkSmartPointer<vtkMatrix4x4>::New();
	
	std::string file = foldername + "rot" + boost::lexical_cast<std::string>(k) + ".rot";
	std::ifstream fileR(file.c_str());

	vtkSmartPointer<vtkMatrix3x3> R = vtkSmartPointer<vtkMatrix3x3>::New();
	int row = 0, col = 0;
	
	std::string line;
	while (std::getline(fileR, line)) { 
		std::istringstream stream(line);
	
		if(row > 0){
			double x;
			col = 0;
			while (stream >> x) {  
				R -> SetElement(row-1,col,x);
				col++;
			}
		}
		row++;
	}
	fileR.close();
	
	R -> Transpose();
	
	file = foldername + "tra" + boost::lexical_cast<std::string>(k) + ".tra";
	std::ifstream fileT(file.c_str());
	double T[3][1];
	row = 0; 
	col = 0;
	
	while (std::getline(fileT, line)) { 
		std::istringstream stream(line);
	
		if(row > 0){
			double x;
			col = 0;
			while (stream >> x) {  
				T[row-1][col] = x*10;
				col++;
			}
		}
		row++;
	}
	fileT.close();
	
	// m must be homogeneous 4x4
	m->SetElement(3, 0, 0);
	m->SetElement(3, 1, 0);
	m->SetElement(3, 2, 0);
	m->SetElement(3, 3, 1);
	
	
	//!< |        |        ]
    //!< |  R^T   |-R^T * t]
    //!< |        |        ]
    //!< |0  0  0 |     1  ]
	double lastCol[3][1];
	lastCol[0][0] = (R->GetElement(0,0)*T[0][0]) + (R->GetElement(0,1)*T[1][0]) + (R->GetElement(0,2)*T[2][0]);
	lastCol[1][0] = (R->GetElement(1,0)*T[0][0]) + (R->GetElement(1,1)*T[1][0]) + (R->GetElement(1,2)*T[2][0]);
	lastCol[2][0] = (R->GetElement(2,0)*T[0][0]) + (R->GetElement(2,1)*T[1][0]) + (R->GetElement(2,2)*T[2][0]);
	
	m->SetElement(0,0,R->GetElement(0,0));  m->SetElement(0,1,R->GetElement(0,1)); m->SetElement(0,2,R->GetElement(0,2));  m->SetElement(0,3,-lastCol[0][0]);
	m->SetElement(1,0,R->GetElement(1,0));  m->SetElement(1,1,R->GetElement(1,1)); m->SetElement(1,2,R->GetElement(1,2));  m->SetElement(1,3,-lastCol[1][0]);
	m->SetElement(2,0,R->GetElement(2,0));  m->SetElement(2,1,R->GetElement(2,1)); m->SetElement(2,2,R->GetElement(2,2));  m->SetElement(2,3,-lastCol[2][0]);
	
	return m;
	
}

