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
find_package(PCL REQUIRED)


catkin_package(

)


include_directories(
  ${catkin_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
)

set(CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/cmake)
find_package(Eigen3)
include_directories(
  ${catkin_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIR}
)


include_directories(src/utils/camodocal/include)
FILE(GLOB CamodocalCameraModelSources
        src/utils/camodocal/src/chessboard/Chessboard.cc
        src/utils/camodocal/src/calib/CameraCalibration.cc
        src/utils/camodocal/src/camera_models/Camera.cc
        src/utils/camodocal/src/camera_models/CameraFactory.cc
        src/utils/camodocal/src/camera_models/CostFunctionFactory.cc
        src/utils/camodocal/src/camera_models/PinholeCamera.cc
        src/utils/camodocal/src/camera_models/CataCamera.cc
        src/utils/camodocal/src/camera_models/EquidistantCamera.cc
        src/utils/camodocal/src/camera_models/ScaramuzzaCamera.cc
        src/utils/camodocal/src/sparse_graph/Transform.cc
        src/utils/camodocal/src/gpl/gpl.cc
        src/utils/camodocal/src/gpl/EigenQuaternionParameterization.cc
    )

FILE( GLOB KuseUtils
    src/utils/PoseManipUtils.cpp
    src/utils/RosMarkerUtils.cpp
    src/utils/EstimationFromPointClouds.cpp
)

# Naive AR node from Qin Tong
add_executable(ar_demo_node src/ar_demo_node.cpp)

# Interactive Markers
add_executable(interactive_marker_server src/interactive_marker_server.cpp)

# AR Node by mpkuse
add_executable(ar_demo_node3
                src/ar_demo_node3.cpp
                #src/PinholeCamera.cpp #TODO: Remove, this, instead add camodocal
                src/MeshObject.cpp
                src/SceneRenderer.cpp
                src/ARDataManager.cpp
                src/ARDataNode.cpp
                ${KuseUtils}
                ${CamodocalCameraModelSources}
            )

# Unit Tests Start
add_executable( unit_test_estimate_ground_plane
                    src/unit_tests/estimate_ground_plane.cpp
                    ${KuseUtils}
              )
# Unit Tests ENDS


 target_link_libraries(ar_demo_node
   ${catkin_LIBRARIES} ${OpenCV_LIBS}
 )

 # Interactive Markers
 target_link_libraries(interactive_marker_server
   ${catkin_LIBRARIES} ${OpenCV_LIBS}
 )

 target_link_libraries(ar_demo_node3
   ${catkin_LIBRARIES} ${OpenCV_LIBS}  ${PCL_LIBRARIES}
 )


# Unit Tests Start
 target_link_libraries(unit_test_estimate_ground_plane
   ${catkin_LIBRARIES} ${OpenCV_LIBS}  ${PCL_LIBRARIES}
 )
# Unit Tests ENDS
