#pragma once
// STD headers
#include <vector>
#include <queue>
#include <map>
#include <mutex>
#include <atomic>

// OpenCV
#include <opencv2/opencv.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <eigen3/Eigen/Dense>
#include <eigen3/Eigen/Geometry>

// Ros
#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>
#include <geometry_msgs/PoseStamped.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/image_encodings.h>
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>

// Eigen
#include <eigen3/Eigen/Dense>
#include <eigen3/Eigen/Geometry>


// My Utility Headers
#include "utils/TermColor.h"
#include "utils/ElapsedTime.h"

#include "ARDataNode.h"

using namespace std;
using namespace Eigen;


class ARDataManager
{
public:
    ARDataManager()
    {
        run_thread_flag = false;
    }

    ~ARDataManager()
    {
        for( auto it=data_map.begin() ; it!=data_map.end() ; it++ ) {
            it->second->~ARDataNode();
            delete it->second;
        }
    }

    // callbacks
    void raw_image_callback( const sensor_msgs::ImageConstPtr msg );
    void odom_pose_callback( const nav_msgs::Odometry::ConstPtr msg ); ///< w_T_c. pose of camera in the world-cordinate system. All the cameras. only a subset of this will be keyframes
    void imuodom_pose_callback( const nav_msgs::Odometry::ConstPtr msg ); //< w_T_imu. pose of the imu in world-cordinate. @200hz

    // thread -
    void run_thread_enable() { run_thread_flag=true; }
    void run_thread_disable() { run_thread_flag=false; }
    void run_thread( int hz);


    // returns the latest node where camera pose (and image_raw) is available (ie. green)
    const ARDataNode* latestNodeWherePoseisAvailable() const;

    // returns the latest node where atleast imu pose is available
    const ARDataNode* latestNodeWhereIMUPoseisAvailable() const;

private:
    mutable std::mutex data_map_mutex;
    std::map< ros::Time, ARDataNode* > data_map;

    std::map<ros::Time,ARDataNode*>::iterator findClosestKey( const ros::Time& key);

    // run thread
    atomic<bool> run_thread_flag;

};
