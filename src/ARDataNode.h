#pragma once
/** Holds the data of pose and image for AR purpose
        Author  : Manohar Kuse <mpkuse@connect.ust.hk>
        Created : 8th Jan, 2019
*/

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
#include "utils/PoseManipUtils.h"
#include "utils/MiscUtils.h"

using namespace Eigen;

class ARDataNode
{
public:
    ARDataNode();
    ~ARDataNode();

    bool isPoseAvailable() const {return is_pose_available; }
    const ros::Time getPoseTimestamp() const { return timestamp_pose; }
    const Matrix4d& getPose() const;

    bool isImageAvailable() const {return is_image_available; }
    const ros::Time getImageTimestamp() const { return timestamp_image; }
    const cv::Mat& getImage() const;


    void setImageFromMsg( const sensor_msgs::ImageConstPtr msg );
    void setPoseFromMsg(  const nav_msgs::Odometry::ConstPtr msg ); // TODO remove/rename. dont pass msg



    void setOptCamPose( ros::Time _t, const Matrix4d ws_T_cam, int worldID, int setID_of_worldID  );
    bool isOptPoseAvailable() const {return is_opt_pose_available; }
    const ros::Time getOptPoseTimestamp() const { return timestamp_opt_pose; }
    const Matrix4d& getOptCamPose() const;
    const int getOptPose_worldID() const { return worldID; }
    const int getOptPose_setID_of_worldID() const { return setID_of_worldID; }

private:
    mutable std::mutex vars_mutex;

    ros::Time timestamp_pose;
    Matrix4d w_T_c;
    std::atomic<bool> is_pose_available;

    ros::Time timestamp_image;
    cv::Mat image;
    std::atomic<bool> is_image_available;


    ros::Time timestamp_opt_pose;
    Matrix4d ws_T_cam; // pose of camera (strictly) in ref frame of setID_of_worldID
    int worldID;
    int setID_of_worldID;
    std::atomic<bool> is_opt_pose_available;
};
