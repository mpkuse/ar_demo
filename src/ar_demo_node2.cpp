/*/////////////////////////////////////////////////////////////
  Rendering Node

  This node only does rendering of the mesh objects on the image.
  Subscribes :
  a) Camera Path (from pose graph optimization node)
  b) Raw Images
  c) Updates to pose (optional.)

  Publishes :
  a) Augmented Image (Raw Image + object overlayed on it)

//////////////////////////////////////////////////////////////*/

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

// My Headers
#include "PinholeCamera.h"
#include "MeshObject.h"
#include "SceneRenderer.h"


using namespace std;
using namespace Eigen;

// Global Variables
queue<sensor_msgs::ImageConstPtr> img_buf;
bool pose_init = false;
int img_cnt = 0;


image_transport::Publisher pub_ARimage;

MeshObject m1;
SceneRenderer renderer;


///////////////////////////////////////////////////////////
//////.///////////////// Utils ////////////////////////////

void rosmsg_2_matrix4d( const geometry_msgs::Pose& pose, Matrix4d& frame_pose )
{
  Vector3d pose_p;
  Quaterniond pose_q;
  pose_p = Vector3d(pose.position.x, pose.position.y,pose.position.z );
  pose_q = Quaterniond( pose.orientation.w, pose.orientation.x,pose.orientation.y,pose.orientation.z );


  // pose_p, pose_q --> Matrix4d
  frame_pose = Matrix4d::Zero();
  frame_pose.col(3) << pose_p, 1.0;
  // Matrix3d R = e_q.toRotationMatrix();
  frame_pose.topLeftCorner<3,3>() = pose_q.toRotationMatrix();

}


void matrix4d_2_rosmsg( const Matrix4d& frame_pose, geometry_msgs::Pose& pose )
{
  Matrix3d R = frame_pose.topLeftCorner<3,3>();
  Quaterniond qua = Quaterniond(R);
  pose.position.x = frame_pose(0,3);
  pose.position.y = frame_pose(1,3);
  pose.position.z = frame_pose(2,3);

  pose.orientation.w = qua.w();
  pose.orientation.x = qua.x();
  pose.orientation.y = qua.y();
  pose.orientation.z = qua.z();

}
/////////////////////////////END UTILS //////////////////////////////////



///////////////////////////////////////////////////////////
//////////////// Synced Callback //////////////////////////
// In this callback, the image and pose correspond
void callback(const sensor_msgs::ImageConstPtr& img_msg, const nav_msgs::Odometry::ConstPtr pose_msg)
{

  // ROS_INFO("sync callback!");
  // cout << "sync callback: " << img_msg->header.stamp << ":" << pose_msg->header.stamp << endl;
  //throw the first few unstable pose
  if(img_cnt < 50)
  {
      img_cnt ++;
      return;
  }

  // Retrive pose w_T_{c}
  Vector3d camera_p(pose_msg->pose.pose.position.x,
                    pose_msg->pose.pose.position.y,
                    pose_msg->pose.pose.position.z);
  Quaterniond camera_q(pose_msg->pose.pose.orientation.w,
                       pose_msg->pose.pose.orientation.x,
                       pose_msg->pose.pose.orientation.y,
                       pose_msg->pose.pose.orientation.z);
  Matrix4d w_T_cam = Matrix4d::Zero();
  w_T_cam.col(3) << camera_p, 1.0;
  w_T_cam.topLeftCorner<3,3>() = camera_q.toRotationMatrix();

  // Retrive Image
  // cv_bridge::CvImagePtr bridge_ptr = cv_bridge::toCvCopy(img_msg, sensor_msgs::image_encodings::MONO8);
  cv_bridge::CvImagePtr bridge_ptr = cv_bridge::toCvCopy(img_msg);
  cv::Mat AR_image;
  // AR_image = bridge_ptr->image.clone();
  AR_image = bridge_ptr->image;//.clone();
  // cv::imshow( "image", AR_image);

  // cout << camera_p << endl;
  //  cv::cvtColor(AR_image, AR_image, cv::COLOR_GRAY2RGB);


  // Render Object
  cv::Mat out;
  renderer.renderIn( AR_image, w_T_cam, out );
  // cv::imshow( "out", out);



  // Publish Augmented Image (use global pub_ARimage)
  sensor_msgs::ImagePtr AR_msg = cv_bridge::CvImage(img_msg->header, "bgr8", out).toImageMsg();
  pub_ARimage.publish(AR_msg);

  cv::waitKey(10);
}



///////////////////////////////////////////////////////////
//////////////// Basic Callbacks //////////////////////////
void img_callback(const sensor_msgs::ImageConstPtr& img_msg)
{
  // ROS_INFO_STREAM( "img_callback" << img_msg->header.stamp );
    if(pose_init)
    {
        img_buf.push(img_msg);
    }
    else
        return;
}

void pose_callback(const nav_msgs::Odometry::ConstPtr &pose_msg)
{
    // ROS_INFO_STREAM( "pose_callback" << pose_msg->header.stamp );
    if(!pose_init)
    {
        pose_init = true;
        return;
    }

    if (img_buf.empty())
    {
        ROS_WARN("image coming late");
        return;
    }

    while (img_buf.front()->header.stamp < pose_msg->header.stamp && !img_buf.empty())
    {
        img_buf.pop();
    }

    if (!img_buf.empty())
    {
        callback(img_buf.front(), pose_msg);
        img_buf.pop();
    }
    else
       ROS_WARN("image coming late");
}

// added by mpkuse
void path_callback( const nav_msgs::Path::ConstPtr& path_msg )
{
  // ROS_INFO_STREAM( "Received path msg: " <<  path_msg->header.stamp );
  // Get the latest pose.
  int len = path_msg->poses.size() ;
  if( len < 1 )
    return ;

  geometry_msgs::PoseStamped p = path_msg->poses[len-1] ;


  // Convert the pose to nav_msgs::Odometry
  // nav_msgs::Odometry pose_msg;
  // nav_msgs::Odometry::Ptr pose_msg();
  boost::shared_ptr<nav_msgs::Odometry> pose_msg(boost::make_shared<nav_msgs::Odometry>());

  // c(boost::make_shared<MyClass>());
  pose_msg->header = path_msg->poses[len-1].header;
  // pose_msg->pose.pose = path_msg->poses[len-1].pose;
  pose_msg->pose.pose.position.x = path_msg->poses[len-1].pose.position.x;
  pose_msg->pose.pose.position.y = path_msg->poses[len-1].pose.position.y;
  pose_msg->pose.pose.position.z = path_msg->poses[len-1].pose.position.z;

  pose_msg->pose.pose.orientation.x = path_msg->poses[len-1].pose.orientation.x;
  pose_msg->pose.pose.orientation.y = path_msg->poses[len-1].pose.orientation.y;
  pose_msg->pose.pose.orientation.z = path_msg->poses[len-1].pose.orientation.z;
  pose_msg->pose.pose.orientation.w = path_msg->poses[len-1].pose.orientation.w;


  // call pose_callback
  pose_callback( pose_msg );

}


/// This will update the mesh-pose upon receiving the message from `interactive_marker_server`
void mesh_pose_callback( const geometry_msgs::PoseStamped& msg )
{
  ROS_INFO_STREAM( "+        XXXXX mesh_pose_callback() for mesh "<< msg.header.frame_id  );
  string frame_id = msg.header.frame_id ;
  if( frame_id == "control_marker" )
  {

    return;
  }

  // msg->pose --> pose_p, pose_q
  Matrix4d w_T_ob;
  rosmsg_2_matrix4d( msg.pose, w_T_ob );
  cout << "Recvd Pose (w_T_{" << frame_id << "}):\n" << w_T_ob << endl;


  // search this mesh
  for( int i=0 ; i<renderer.getMeshCount() ; i++ )
  {
    if( frame_id ==  renderer.getMeshName(i) )
    {
      // ROS_INFO_STREAM( "            Found :}" << "set w_T_obj=" << frame_pose );
      cout << "Found "<< frame_id << "; Setting w_T_obj" << endl;
      (renderer.getMeshObject(i))->setObjectWorldPose( w_T_ob );
      return;
    }
  }
  ROS_INFO( "mesh not Found :{");
  return;


}

///////////////////////////////////////////////////////////
//////////////// END Callbacks ////////////////////////////





int main( int argc, char ** argv )
{
  ros::init(argc, argv, "ar_demo_2");
  ros::NodeHandle n("~");

  ROS_INFO( "Started Node (ar_demo_2)");


  // Set up renderer
  //    a) Load Camera Params (from yaml file)
  //    b) Load mesh(es)

  string calib_file;
  calib_file = string("/home/mpkuse/catkin_ws/src/VINS_testbed/config/black_box4/blackbox4.yaml");
  // n.getParam("calib_file", calib_file); //TODO
  ROS_INFO("reading paramerter of camera %s", calib_file.c_str());
  PinholeCamera camera = PinholeCamera( calib_file );


  // Will read the mesh object (.obj) from this package's resources folder.
  m1 = MeshObject( "cube.obj");
    // Set a default object world pose
  Matrix4d init_w_T_obj;
  init_w_T_obj << 1.0, 0.0, 0.0, 8.0,
                  0.0, 1.0, 0.0, -1.,
                  0.0, 0.0, 1.0, -1.2,
                  0.0, 0.0, 0.0, 1.0;
  m1.setObjectWorldPose(init_w_T_obj);


  renderer = SceneRenderer();
  renderer.setCamera( &camera );
  renderer.addMesh( &m1 );



  // Subscribers
  //   a) Camera Path (from pose graph optimization node)
  //   b) Raw Images
  //   c) Updates to pose (optional.)
  ROS_INFO( "Subscribe to Camera Path: ~image_raw" );
  ros::Subscriber path_of_img = n.subscribe("camera_path", 100, path_callback);

  ROS_INFO( "Subscribe to Raw Image: ~image_raw" );
  ros::Subscriber sub_img = n.subscribe("image_raw", 100, img_callback);

  // ROS_INFO( "Subscribe to Interactive Marker Pose (updates): ~s" )
  ros::Subscriber sub_mesh_pose = n.subscribe( "/object_mesh_pose", 1000, mesh_pose_callback );


  // Publishers
  //   a) Augmented Image (Raw Image + object overlayed on it)
  image_transport::ImageTransport it(n);
  pub_ARimage = it.advertise("AR_image", 1000);





  ROS_INFO( "spin()");

  ros::spin();
}
