cmake_minimum_required(VERSION 2.8.3)
find_package( OpenCV REQUIRED )

PROJECT(camera_calibrator) 

include_directories(include)
set(SRC
    src/main.cpp
    src/Utilities.cpp
    src/monocularCamera.cpp
    src/stereoCamera.cpp)    
add_executable( ${PROJECT_NAME} ${SRC} )
target_link_libraries( ${PROJECT_NAME}  ${OpenCV_LIBS} )
