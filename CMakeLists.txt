cmake_minimum_required(VERSION 2.8.3)
project(ar_demo)

set(CMAKE_BUILD_TYPE "Release")
set(CMAKE_CXX_FLAGS "-std=c++11 -DEIGEN_DONT_PARALLELIZE")
#-DEIGEN_USE_MKL_ALL")
set(CMAKE_CXX_FLAGS_RELEASE "-O3 -Wall -g")

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  image_transport
  sensor_msgs
  cv_bridge
  message_filters
  camera_model
  interactive_markers
)
find_package(OpenCV REQUIRED)

catkin_package(

)


include_directories(
  ${catkin_INCLUDE_DIRS}
)

set(CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/cmake)
find_package(Eigen3)
include_directories(
  ${catkin_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIR}
)

add_executable(ar_demo_node src/ar_demo_node.cpp)

# Interactive Markers
add_executable(interactive_marker_server src/interactive_marker_server.cpp)
add_executable(ar_demo_node2
                  src/ar_demo_node2.cpp
                  src/PinholeCamera.cpp
                  src/MeshObject.cpp
                  src/SceneRenderer.cpp
              )
add_executable(ar_demo_node3
                src/ar_demo_node3.cpp
                src/PinholeCamera.cpp #TODO: Remove, this, instead add camodocal
                src/MeshObject.cpp
                src/SceneRenderer.cpp
                src/ARDataManager.cpp
                src/ARDataNode.cpp
                src/utils/PoseManipUtils.cpp
                src/utils/RosMarkerUtils.cpp
            )


 target_link_libraries(ar_demo_node
   ${catkin_LIBRARIES} ${OpenCV_LIBS}
 )

 # Interactive Markers
 target_link_libraries(interactive_marker_server
   ${catkin_LIBRARIES} ${OpenCV_LIBS}
 )

 target_link_libraries(ar_demo_node2
   ${catkin_LIBRARIES} ${OpenCV_LIBS}
 )

 target_link_libraries(ar_demo_node3
   ${catkin_LIBRARIES} ${OpenCV_LIBS}
 )
