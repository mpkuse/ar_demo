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

#include "utils/RosMarkerUtils.h"
#include "utils/PoseManipUtils.h"
#include "utils/EstimationFromPointClouds.h"

#include "ARDataNode.h"
#include "SceneRenderer.h"

// PCL Library
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl/filters/statistical_outlier_removal.h>

using namespace std;
using namespace Eigen;


class ARDataManager
{
public:
    ARDataManager(  )
    {
        run_thread_flag = false;
        monitor_thread_flag=false;

        m_ground_plane_estimated = false;
    }

    ~ARDataManager()
    {
        for( auto it=data_map.begin() ; it!=data_map.end() ; it++ ) {
            it->second->~ARDataNode();
            delete it->second;
        }
    }

    void setRenderer( SceneRenderer* _renderer ) {
        renderer = _renderer;
    }
    void setARImagePublisher( image_transport::Publisher _pub ) {
        pub_ARimage = _pub;
        isPubARImageset = true;
    }

    void setMarkerPublisher( ros::Publisher& pub_marker )
    {
        this->pub_marker = pub_marker;
        isPubMarkerset = true;
    }


    // callbacks
    void raw_image_callback( const sensor_msgs::ImageConstPtr msg );
    void odom_pose_callback( const nav_msgs::Odometry::ConstPtr msg ); ///< w_T_c. pose of camera in the world-cordinate system. All the cameras. only a subset of this will be keyframes


    void mesh_pose_callback( const geometry_msgs::PoseStamped::ConstPtr msg ); //< updates the pose of mesh(i) which are inside renderer.
    void surfelmap_callback(const sensor_msgs::PointCloud2::ConstPtr pointcloud_map);

    void detailed_path_callback( const nav_msgs::Path::ConstPtr msg );
    void hz200_imu_callback( const geometry_msgs::PoseStamped::ConstPtr msg );
private:
    // in the `hz200_imu_callback` we simply push msgs on the queue.
    // in the rendering thread, we pop from the queue
    mutable std::mutex imuprop_msgs_mutex;
    std::queue<geometry_msgs::PoseStamped> imuprop_msgs;


public:

    // thread -
    void run_thread_enable() { run_thread_flag=true; }
    void run_thread_disable() { run_thread_flag=false; }
    // void run_thread( int hz);
    void run_thread_duo( int hz); //newer

    // thread - monitor
    void monitor_thread_enable() { monitor_thread_flag=true; }
    void monitor_thread_disable() { monitor_thread_flag=false; }
    void monitor_thread( int hz, bool printing);


    // returns the latest node where camera pose (and image_raw) is available (ie. green)
    const ARDataNode* latestNodeWherePoseisAvailable() const;

    // returns the latest node where optimized pose is available (in addition to the image)
    const ARDataNode* latestNodeWhereOptPoseisAvailable() const;

    // latest node where image is available
    const ARDataNode* latestNodeWhereImageisAvailable() const;


private:
    mutable std::mutex data_map_mutex;
    std::map< ros::Time, ARDataNode* > data_map;

    std::map<ros::Time,ARDataNode*>::iterator findClosestKey( const ros::Time& key)  ;
    std::map<ros::Time,ARDataNode*>::iterator findClosestKeyApprox( const ros::Time& key) ;

    // run thread
    atomic<bool> run_thread_flag;

    // monitor thread
    atomic<bool> monitor_thread_flag;

    // renderer
    SceneRenderer * renderer = NULL;

    // image publisher
    image_transport::Publisher pub_ARimage;
    bool isPubARImageset = false;

    // marker publisher
    ros::Publisher pub_marker;
    bool isPubMarkerset = false;

    // Ground Plane
    atomic<bool> m_ground_plane_estimated;
    VectorXd groundplane_coeff;
    Matrix4d groundplane_wTp;




    //////// cam imu extrinsic (this is published by vins estimator node)
public:
    void extrinsic_cam_imu_callback( const nav_msgs::Odometry::ConstPtr msg );
    Matrix4d get_imu_T_cam() const;
    void get_imu_T_cam( Matrix4d& res, ros::Time& _t ) const;
    bool is_imu_T_cam_available() const;

private:
    mutable mutex imu_cam_mx;
    bool imu_T_cam_available = false;
    Matrix4d imu_T_cam;
    ros::Time imu_T_cam_stamp;

};
