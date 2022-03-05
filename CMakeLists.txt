cmake_minimum_required(VERSION 2.8.3)
project(lidar_stereo_ros)

#get_filename_component(EXTERNAL_LIBRARIES ../../ ABSOLUTE)
find_package(PCL 1.9 REQUIRED PATHS /media/ashu/25EA6BA0225EEBFD/ashu_ws/CoSM_ICP_v2/external_libraries/pcl_1_9/install_dir/share/pcl-1.9 NO_DEFAULT_PATH )
find_package(OpenCV REQUIRED PATHS /media/ashu/25EA6BA0225EEBFD/ashu_ws/libraries/opencv_4_5_0/install_dir/lib/cmake/opencv4 NO_DEFAULT_PATH)
find_package(Boost) 

include_directories(${PCL_INCLUDE_DIRS})
include_directories(${Boost_INCLUDE_DIRS}) 
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})



add_executable (LidarCamCalib LidarCamCalib.cpp)
target_link_libraries (LidarCamCalib ${PCL_LIBRARIES} ${OpenCV_LIBS})

