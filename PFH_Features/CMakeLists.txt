cmake_minimum_required(VERSION 2.8 FATAL_ERROR)

list(APPEND CMAKE_MODULE_PATH ${CMAKE_CURRENT_SOURCE_DIR})
PROJECT(create_Features)

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

  set( SRC src/create_features.cpp
		   src/main.cpp
		 )    
  add_executable( ${PROJECT_NAME} ${SRC} )
  target_link_libraries( ${PROJECT_NAME}  ${OpenCV_LIBS} ${PCL_LIBRARIES} ${Boost_LIBRARIES} ${FLANN_LIBRARIES} ${HDF5_hdf5_LIBRARY} )
  
  add_executable(test_Features src/test_Features.cpp src/create_features.cpp)
  target_link_libraries(test_Features ${PCL_LIBRARIES} ${Boost_LIBRARIES} ${OpenCV_LIBS} ${FLANN_LIBRARIES} ${HDF5_hdf5_LIBRARY} )

  add_executable(transform2pcd src/transform2pcd.cpp)
  target_link_libraries(transform2pcd ${PCL_LIBRARIES} ${OpenCV_LIBS} ${Boost_LIBRARIES})
endif(HDF5_FOUND)
