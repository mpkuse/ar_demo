#include "ARDataNode.h"



///////////////// ARDataNode ////////////////////
ARDataNode::ARDataNode()
{
    is_pose_available = false;
    is_image_available = false;
    is_imupose_available = false;
}

ARDataNode::~ARDataNode()
{
    // cout << "ARDataNode::~ARDataNode descructor called\n";
    std::lock_guard<std::mutex> lk(vars_mutex);
    if( isImageAvailable() )
        image.release();

    is_pose_available = false;
    is_image_available = false;
    is_imupose_available = false;
}

const Matrix4d& ARDataNode::getPose() const
{
    std::lock_guard<std::mutex> lk(vars_mutex);
    assert( is_pose_available && "[ARDataNode::getPose] pose doesn't appear to be set and you are requesting a pose");
    return w_T_c;
}

const cv::Mat& ARDataNode::getImage() const
{
    std::lock_guard<std::mutex> lk(vars_mutex);
    assert( is_image_available && "[ARDataNode::getImage] image doesn't appear to be set and you are requesting an image");
    return image;
}

const Matrix4d& ARDataNode::getIMUPose() const
{
    std::lock_guard<std::mutex> lk(vars_mutex);
    assert( is_imupose_available && "[ARDataNode::getIMUPose] imupose doesn't appear to be set and you are requesting an imupose");
    return w_T_imu;
}

void ARDataNode::setImageFromMsg( const sensor_msgs::ImageConstPtr msg )
{
    std::lock_guard<std::mutex> lk(vars_mutex);
    // image = cv_bridge::toCvShare(msg)->image;
    image = cv_bridge::toCvCopy(msg)->image;

    timestamp_image = msg->header.stamp;
    is_image_available = true;
}

void ARDataNode::setPoseFromMsg(  const nav_msgs::Odometry::ConstPtr msg )
{
    std::lock_guard<std::mutex> lk(vars_mutex);
    PoseManipUtils::geometry_msgs_Pose_to_eigenmat( msg->pose.pose, w_T_c );

    timestamp_pose = msg->header.stamp;
    is_pose_available = true;
}

void ARDataNode::setIMUPoseFromMsg(  const nav_msgs::Odometry::ConstPtr msg )
{
    std::lock_guard<std::mutex> lk(vars_mutex);
    PoseManipUtils::geometry_msgs_Pose_to_eigenmat( msg->pose.pose, w_T_imu );

    timestamp_imupose = msg->header.stamp;
    is_imupose_available = true;
}
