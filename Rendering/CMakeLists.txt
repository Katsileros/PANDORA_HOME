cmake_minimum_required(VERSION 2.8)

PROJECT(Rendering)

find_package(VTK REQUIRED)

find_package( OpenCV REQUIRED )
find_package( PCL 1.7 REQUIRED )

include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})
include(${VTK_USE_FILE})

add_executable(Rendering Rendering.cpp)

target_link_libraries( Rendering  ${OpenCV_LIBS} ${PCL_LIBRARIES} ${Boost_LIBRARIES} ${FLANN_LIBRARIES} ${VTK_LIBRARIES})
