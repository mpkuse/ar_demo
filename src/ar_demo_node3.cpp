/*
    This node is written in an organized way, learning from the mistakes of ar_demo_node.cpp
    and ar_demo_node2.cpp

        Author : Manohar Kuse <mpkuse@connect.ust.hk>
        Created: 8th Jan, 2019
*/
// STD headers
#include <vector>
#include <queue>

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

// camodocal
#include "camodocal/camera_models/Camera.h"
#include "camodocal/camera_models/CameraFactory.h"
#include "camodocal/camera_models/CataCamera.h"
#include "camodocal/camera_models/EquidistantCamera.h"

// My Headers
// #include "PinholeCamera.h"
#include "ARDataManager.h"
#include "MeshObject.h"
#include "SceneRenderer.h"

// My Utility Headers
#include "utils/TermColor.h"

using namespace std;
using namespace Eigen;

template<typename Out>
void split(const std::string &s, char delim, Out result) {
    std::stringstream ss(s);
    std::string item;
    while (std::getline(ss, item, delim)) {
        *(result++) = item;
    }
}

std::vector<std::string> split(const std::string &s, char delim) {
    std::vector<std::string> elems;
    split(s, delim, std::back_inserter(elems));
    return elems;
}


int main(int argc, char ** argv )
{
    ////////////////////////////////////////////
    // ROS INIT
    ///////////////////////////////////////////
    ros::init(argc, argv, "ar_demo_3");
    ros::NodeHandle n("~");

    ROS_INFO( "Started Node (ar_demo_3)");




    ///////////////////////////////////////////
    // Load Camera and MeshObject
    ///////////////////////////////////////////
    //    a) Load Camera Params (from yaml file)
    //    b) Load mesh(es) (list of obj from roslaunch params)

    string calib_file;
    // calib_file = string("/home/mpkuse/catkin_ws/src/VINS_testbed/config/black_box4/blackbox4.yaml");
    n.getParam("calib_file", calib_file); //TODO
    ROS_INFO("reading paramerter of camera %s", calib_file.c_str());
    camodocal::CameraPtr m_camera;
    m_camera = camodocal::CameraFactory::instance()->generateCameraFromYamlFile(calib_file);
    if( !m_camera ) {
        cout << TermColor::RED() << "[ar_demo_node3]camodocal cannot load calib file...quitting" << TermColor::RED() << endl;;
        ROS_ERROR( "[ar_demo_node3]camodocal cannot load calib file...quitting");
        exit(1);
    }
    // Inbuild function for priniting
    std::cout << m_camera->parametersToString() << std::endl;



    // Will read the mesh object (.obj) from this package's resources folder.


    // Read all the mesh objects from ros params
    string obj_list, mesh_scaling_list, mesh_initial_positions_list;
    n.getParam("obj_list", obj_list);
    vector<string> objs = split( obj_list, ';' );

    // now attempt to reach scaling for each param
    n.getParam("mesh_scaling_list", mesh_scaling_list);
    vector<string> mesh_scaling_ = split( mesh_scaling_list, ';' );

    n.getParam("mesh_initial_positions_list", mesh_initial_positions_list); // this is like "1,2,3;5,1,2"
    vector<string> mesh_initial_positions_ = split( mesh_initial_positions_list, ';' );


    cout << "# of meshes               : " << objs.size() << endl;
    cout << "# of mesh_scaling_list    : " << mesh_scaling_.size() << endl;
    cout << "# of init positions       : " << mesh_initial_positions_.size() << endl;

    // Ensure number of obj specified is equal to number of mesh and controls scaling values
    // if( (objs.size() == mesh_scaling_.size()) && (objs.size() == controls_scaling_.size()) && (mesh_scaling_.size()==controls_scaling_.size())  )
    if( (objs.size()==mesh_scaling_.size())  &&
      (objs.size()==mesh_initial_positions_.size())  )
    {
    // OK!
    ;
    }
    else
    {
    ROS_ERROR( "Number of of meshes specified not equal to scaling parameters. Quit!");
    return 0;
    }

    cout << "----- All Objects ------\n";
    for( int i=0 ; i<(int)objs.size() ; i++ )
    {
    cout << i << "  mesh="<<objs[i] << "; ";
    cout << "mesh_scaling="<< std::stod(mesh_scaling_[i]) << "; ";

    vector<string> __t = split( mesh_initial_positions_[i], ',' );
    cout << "mesh_initial_positions="<< std::stod(__t[0]) << "," << std::stod(__t[1]) << "," << std::stod(__t[2]) << "; ";
    cout << endl;
    }
    cout << "------------------------\n";




    //////////////////////////////////////////////
    // Renderer
    //////////////////////////////////////////////
    SceneRenderer * renderer = new SceneRenderer();
    renderer->setCamera( m_camera );

    for( int i=0 ; i<(int)objs.size() ; i++ ) // read all obj files
    {
      MeshObject *m1 = new MeshObject( objs[i], std::stod(mesh_scaling_[i]) );

      vector<string> __t = split( mesh_initial_positions_[i], ',' );
      Matrix4d init_w_T_obj;
      init_w_T_obj << 1.0, 0.0, 0.0, std::stod(__t[0]),
                      0.0, 1.0, 0.0, std::stod(__t[1]),
                      0.0, 0.0, 1.0, std::stod(__t[2]),
                      0.0, 0.0, 0.0, 1.0;
      m1->setObjectWorldPose(init_w_T_obj);
      renderer->addMesh( m1 );
    }



    ////////////////////////////////////////////
    // ARDataManager
    ///////////////////////////////////////////
    ARDataManager * manager = new ARDataManager();
    manager->setRenderer(renderer);

    ////////////////////////////////////
    //     Subscribers
    ////////////////////////////////////
    //   a) Camera Odometry and Corrected Odometry (from pose graph optimization node)
    //   b) Raw Images
    //   c) Updates to pose (optional.)
    // string odometry_topic_name = "/vins_estimator/camera_pose"; //< this can also be the corrected pose.
    string odometry_topic_name = "/keyframe_pose_graph_slam_node/opt_odometry"; //< this can also be the corrected pose.
    ROS_INFO( "Subscribe to odometry_topic_name: %s", odometry_topic_name.c_str() );
    ros::Subscriber sub_odometry_topic = n.subscribe(odometry_topic_name.c_str(), 100, &ARDataManager::odom_pose_callback, manager);


    string raw_image_topic = "/mynteye/left/image_raw";
    ROS_INFO( "Subscribe to Raw Image: %s", raw_image_topic.c_str() );
    ros::Subscriber sub_img = n.subscribe(raw_image_topic.c_str(), 100, &ARDataManager::raw_image_callback, manager );


    string imupose_topic_name = "/vins_estimator/imu_propagate"; //< this can also be the corrected pose.
    ROS_INFO( "Subscribe to imupose_topic_name: %s", imupose_topic_name.c_str() );
    ros::Subscriber sub_imuodometry_topic = n.subscribe(imupose_topic_name.c_str(), 100, &ARDataManager::imuodom_pose_callback, manager);


    string marker_pose_topic_name = "/interactive_marker_server/object_mesh_pose";
    ROS_INFO( "Subscribe to Interactive Marker Pose (updates): %s", marker_pose_topic_name.c_str() );
    ros::Subscriber sub_mesh_pose = n.subscribe( marker_pose_topic_name, 1000, &ARDataManager::mesh_pose_callback, manager );

    /////////////////////////////////////
    // Publishers
    /////////////////////////////////////
    //   a) Augmented Image (Raw Image + object overlayed on it)
    image_transport::ImageTransport it(n);
    string ar_image_topic = "AR_image";
    ROS_INFO( "Publish ar_image_topic : %s", ar_image_topic.c_str() );
    image_transport::Publisher pub_ARimage = it.advertise(ar_image_topic, 100);
    manager->setARImagePublisher( pub_ARimage );


    //////////////////////////////////
    // Threads
    //////////////////////////////////

    // this thread monitors data_map and removes old items, and optionally prints status
    manager->monitor_thread_enable();
    std::thread t_monitor( &ARDataManager::monitor_thread, manager, 1 , false);

    // this thread monitors the data_map periodically and renders if it sees something new
    manager->run_thread_enable();
    std::thread t_render( &ARDataManager::run_thread, manager, 30 );


    ROS_INFO( "spin()");
    ros::spin();

    manager->run_thread_disable();
    manager->monitor_thread_disable();

    t_render.join();
    t_monitor.join();

    // manager->~ARDataManager();

}
