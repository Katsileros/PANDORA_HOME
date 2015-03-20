cmake_minimum_required(VERSION 2.8 FATAL_ERROR)

list(APPEND CMAKE_MODULE_PATH ${CMAKE_CURRENT_SOURCE_DIR})
PROJECT(VFH_Features)

find_package( OpenCV REQUIRED )
find_package( PCL 1.7 REQUIRED )

include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})


find_package(HDF5)
if(HDF5_FOUND)

  find_package(FLANN)
  include_directories(include)
  include_directories(${FLANN_INCLUDE_DIRS})
  include_directories(${HDF5_INCLUDE_DIR})

  set( SRC src/vfh_features.cpp
		   src/main.cpp
		 )    
  add_executable( ${PROJECT_NAME} ${SRC} )
  target_link_libraries( ${PROJECT_NAME}  ${OpenCV_LIBS} ${PCL_LIBRARIES} ${Boost_LIBRARIES} ${FLANN_LIBRARIES} ${HDF5_hdf5_LIBRARY} )

endif(HDF5_FOUND)
