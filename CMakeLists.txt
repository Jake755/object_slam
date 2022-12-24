cmake_minimum_required(VERSION 2.8.3)
project(object_slam)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall -std=c++14 -pthread")

IF(NOT CMAKE_BUILD_TYPE)
  SET(CMAKE_BUILD_TYPE Release)
ENDIF()

MESSAGE("Build type: " ${CMAKE_BUILD_TYPE})
set(CMAKE_CXX_STANDARD 14)

add_definitions(-w)

find_package(catkin REQUIRED COMPONENTS 
  cmake_modules
  roscpp 
  sensor_msgs 
  nav_msgs 
  message_filters 
  image_transport 
  cv_bridge
  image_geometry
  pcl_ros
  pcl_conversions
)

find_package(OpenCV REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(Pangolin REQUIRED)
find_package(PythonLibs 2.7 REQUIRED) 
find_package(PCL REQUIRED COMPONENTS common)
find_package(OpenMP REQUIRED)
if(OPENMP_FOUND)
    message("OPENMP FOUND")
    set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} ${OpenMP_C_FLAGS}")
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${OpenMP_CXX_FLAGS}")
    set(CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} ${OpenMP_EXE_LINKER_FLAGS}")
endif()

catkin_package(
  INCLUDE_DIRS include 
  LIBRARIES ${PROJECT_NAME}
  CATKIN_DEPENDS cmake_modules roscpp nav_msgs sensor_msgs message_filters image_transport cv_bridge pcl_ros  pcl_conversions
)

include_directories(
${PROJECT_SOURCE_DIR}
${PROJECT_SOURCE_DIR}/include
${PROJECT_SOURCE_DIR}/lib
${catkin_INCLUDE_DIRS}
${OpenCV_INCLUDE_DIRS}
${EIGEN_INCLUDE_DIR}
${Pangolin_INCLUDE_DIRS}
${PYTHON_INCLUDE_DIRS}
${PCL_INCLUDE_DIRS}
)

link_directories(
${PROJECT_SOURCE_DIR}/Thirdparty/ncnn
${PROJECT_SOURCE_DIR}/lib
${catkin_LIB_DIRS}
)

add_library(${PROJECT_NAME} SHARED
src/System.cc
src/Tracking.cc
src/Tracking_util.cc
src/LocalMapping.cc
src/LocalMappingLater.cpp
src/LoopClosing.cc
src/ORBextractor.cc
src/ORBmatcher.cc
src/FrameDrawer.cc
src/Converter.cc
src/MapPoint.cc
src/MapObject.cc
src/KeyFrame.cc
src/Map.cc
src/MapDrawer.cc
src/Optimizer.cc
src/PnPsolver.cc
src/Frame.cc
src/KeyFrameDatabase.cc
src/Sim3Solver.cc
src/Initializer.cc
src/Viewer.cc
src/Parameters.cpp
src/g2o_Object.cpp
) 

target_link_libraries(${PROJECT_NAME}
${OpenCV_LIBS}
${Pangolin_LIBRARIES}
${catkin_LIBRARIES}  # need detect_3d_cuboid
${PROJECT_SOURCE_DIR}/Thirdparty/DBoW2/lib/libDBoW2.so
${PROJECT_SOURCE_DIR}/Thirdparty/g2o/lib/libg2o.so
${PROJECT_SOURCE_DIR}/Thirdparty/ncnn/libncnn.a
${PROJECT_SOURCE_DIR}/lib/libdetect_3d_cuboid.so
${PROJECT_SOURCE_DIR}/lib/libline_lbd_lib.so
${PROJECT_SOURCE_DIR}/lib/libtictoc_profiler.so
${PROJECT_SOURCE_DIR}/lib/libyolact_tool.so
)

add_executable(ros_mono src/ros_mono.cc)
target_link_libraries(ros_mono  
${PROJECT_NAME} ${catkin_LIBRARIES} ${OpenCV_LIBRARIES} ${PCL_LIBRARIES}
${PROJECT_SOURCE_DIR}/Thirdparty/ncnn/libncnn.a
${PROJECT_SOURCE_DIR}/lib/libdetect_3d_cuboid.so
${PROJECT_SOURCE_DIR}/lib/libline_lbd_lib.so
${PROJECT_SOURCE_DIR}/lib/libtictoc_profiler.so
${PROJECT_SOURCE_DIR}/lib/libyolact_tool.so
)

set(CMAKE_INSTALL_PREFIX ${CMAKE_SOURCE_DIR}/../devel/lib)

FILE(COPY ${PROJECT_SOURCE_DIR}/lib/libline_lbd_lib.so DESTINATION ${CMAKE_SOURCE_DIR}/../devel/lib)

message(STATUS ${OpenCV_INCLUDE_DIRS})
