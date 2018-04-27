/*/////////////////////////////////////////////////////////////
  InteractiveMarkerServer

  This node loads up mesh files (.obj) publishes as interactive markers.
  These markers can be visualized with rviz. This node also subscribes to
  a feedback message (on marker pose update by user).


//////////////////////////////////////////////////////////////*/


#include <ros/ros.h>
#include <visualization_msgs/Marker.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseStamped.h>
#include <interactive_markers/interactive_marker_server.h>
#include <interactive_markers/menu_handler.h>

#include <tf/tf.h>
#include <tf/transform_broadcaster.h>
#include <string>
#include <vector>
#include <iterator>
#include <sstream>
using namespace std;
using namespace visualization_msgs;

boost::shared_ptr<interactive_markers::InteractiveMarkerServer> server;
interactive_markers::MenuHandler menu_handler;
ros::Publisher pub_obj_pose;


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

void publish_marker_pose( const string& marker_name, const geometry_msgs::Pose& pose )
{
  geometry_msgs::PoseStamped msg_to_send;
  // msg_to_send.header.seq = 0;
  // msg_to_send.header.stamp.secs = 0;
  // msg_to_send.header.stamp.nsecs = 0;

  msg_to_send.header.frame_id = marker_name;
  msg_to_send.pose = pose;
  ROS_WARN( "interactive_marker_server: Publish geometry_msgs::PoseStamped");
  pub_obj_pose.publish( msg_to_send );

}

void processFeedback(
    const visualization_msgs::InteractiveMarkerFeedbackConstPtr &feedback )
{
  // ROS_INFO_STREAM( feedback->marker_name << " is now at "
      // << feedback->pose.position.x << ", " << feedback->pose.position.y
      // << ", " << feedback->pose.position.z );

      std::ostringstream s;
      s << "Feedback from marker '" << feedback->marker_name << "' "
          << " / control '" << feedback->control_name << "'";



      std::ostringstream mouse_point_ss;
      if( feedback->mouse_point_valid )
      {
        mouse_point_ss << " at " << feedback->mouse_point.x
                       << ", " << feedback->mouse_point.y
                       << ", " << feedback->mouse_point.z
                       << " in frame " << feedback->header.frame_id;
      }




    switch ( feedback->event_type )
     {
       case visualization_msgs::InteractiveMarkerFeedback::BUTTON_CLICK:
         ROS_INFO_STREAM( s.str() << ": button click" << mouse_point_ss.str() << "." );
         break;

       case visualization_msgs::InteractiveMarkerFeedback::MENU_SELECT:
         ROS_INFO_STREAM( s.str() << ": menu item " << feedback->menu_entry_id << " clicked" << mouse_point_ss.str() << "." );
         break;

       case visualization_msgs::InteractiveMarkerFeedback::POSE_UPDATE:
         ROS_INFO_STREAM( s.str() << ": pose changed"
             << "\nposition = "
             << feedback->pose.position.x
             << ", " << feedback->pose.position.y
             << ", " << feedback->pose.position.z
             << "\norientation = "
             << feedback->pose.orientation.w
             << ", " << feedback->pose.orientation.x
             << ", " << feedback->pose.orientation.y
             << ", " << feedback->pose.orientation.z
             << "\nframe: " << feedback->header.frame_id
             << " time: " << feedback->header.stamp.sec << "sec, "
             << feedback->header.stamp.nsec << " nsec" );
         break;

       case visualization_msgs::InteractiveMarkerFeedback::MOUSE_DOWN:
         ROS_INFO_STREAM( s.str() << ": mouse down" << mouse_point_ss.str() << "." );
         break;

       case visualization_msgs::InteractiveMarkerFeedback::MOUSE_UP:
         ROS_INFO_STREAM( s.str() << ": mouse up" << mouse_point_ss.str() << "." );

         // publish updated pose
         if( (feedback->marker_name.c_str())[0] != 'X' )
           publish_marker_pose( feedback->marker_name, feedback->pose );
          else
            publish_marker_pose( "control_marker", feedback->pose );


         break;
     }
}

Marker makeBox( InteractiveMarker &msg )
{
  Marker marker;

  marker.type = Marker::CUBE;
  // msg.scale *= 10.;
  marker.scale.x = msg.scale * 0.45;
  marker.scale.y = msg.scale * 0.45;
  marker.scale.z = msg.scale * 0.45;
  marker.color.r = 0.5;
  marker.color.g = 0.0;
  marker.color.b = 0.0;
  marker.color.a = .5;


  // marker.type = visualization_msgs::Marker::MESH_RESOURCE;
  // marker.mesh_resource = "package://nap/resources/1.obj";

  return marker;
}

Marker makeMeshBox( InteractiveMarker &msg, string mesh_fname )
{
  Marker marker;

  // marker.type = Marker::CUBE;
  msg.scale *= 10.;
  marker.scale.x = 1.0; //msg.scale * 0.45;
  marker.scale.y = 1.0; //msg.scale * 0.45;
  marker.scale.z = 1.0; //msg.scale * 0.45;
  marker.color.r = 0.5;
  marker.color.g = 0.5;
  marker.color.b = 0.5;
  marker.color.a = .5;


  marker.type = visualization_msgs::Marker::MESH_RESOURCE;
  // marker.mesh_resource = "package://nap/resources/1.obj";
  ROS_INFO_STREAM( "Load Mesh : " << string("package://ar_demo/resources/") + mesh_fname );
  marker.mesh_resource = string("package://ar_demo/resources/") + mesh_fname;

  return marker;
}

InteractiveMarkerControl& makeBoxControl( InteractiveMarker &msg, bool useMesh )
{
  InteractiveMarkerControl control;
  control.always_visible = true;
  if( useMesh ) {
    control.markers.push_back( makeMeshBox(msg, msg.name) );
  }
  else {
    control.markers.push_back( makeBox(msg) );
  }
  msg.controls.push_back( control );

  return msg.controls.back();
}

void make6DofMarker( const tf::Vector3& position, string name, bool useMesh )
{
  bool fixed = true;
  unsigned int interaction_mode = visualization_msgs::InteractiveMarkerControl::NONE;
  bool show_6dof = true;

  InteractiveMarker int_marker;
  int_marker.header.frame_id = "world";
  tf::pointTFToMsg(position, int_marker.pose.position);
  int_marker.scale = 1;

  int_marker.name = name.c_str(); //"simple_6dof";
  int_marker.description = name.c_str();//"Simple 6-DOF Control";

  // insert a box
  makeBoxControl(int_marker, useMesh );
  int_marker.controls[0].interaction_mode = interaction_mode;

  InteractiveMarkerControl control;

  if ( fixed )
  {
    // int_marker.name += "_fixed";
    int_marker.description += "\n(click object to publish pose)";
    control.orientation_mode = InteractiveMarkerControl::FIXED;
  }

  if (interaction_mode != visualization_msgs::InteractiveMarkerControl::NONE)
  {
      std::string mode_text;
      if( interaction_mode == visualization_msgs::InteractiveMarkerControl::MOVE_3D )         mode_text = "MOVE_3D";
      if( interaction_mode == visualization_msgs::InteractiveMarkerControl::ROTATE_3D )       mode_text = "ROTATE_3D";
      if( interaction_mode == visualization_msgs::InteractiveMarkerControl::MOVE_ROTATE_3D )  mode_text = "MOVE_ROTATE_3D";
      // int_marker.name += "_" + mode_text;
      int_marker.description = std::string("3D Control") + (show_6dof ? " + 6-DOF controls" : "") + "\n" + mode_text;
  }

  if(show_6dof)
  {
    control.orientation.w = 1;
    control.orientation.x = 1;
    control.orientation.y = 0;
    control.orientation.z = 0;
    control.name = "rotate_x";
    control.interaction_mode = InteractiveMarkerControl::ROTATE_AXIS;
    int_marker.controls.push_back(control);
    control.name = "move_x";
    control.interaction_mode = InteractiveMarkerControl::MOVE_AXIS;
    int_marker.controls.push_back(control);

    control.orientation.w = 1;
    control.orientation.x = 0;
    control.orientation.y = 1;
    control.orientation.z = 0;
    control.name = "rotate_z";
    control.interaction_mode = InteractiveMarkerControl::ROTATE_AXIS;
    int_marker.controls.push_back(control);
    control.name = "move_z";
    control.interaction_mode = InteractiveMarkerControl::MOVE_AXIS;
    int_marker.controls.push_back(control);

    control.orientation.w = 1;
    control.orientation.x = 0;
    control.orientation.y = 0;
    control.orientation.z = 1;
    control.name = "rotate_y";
    control.interaction_mode = InteractiveMarkerControl::ROTATE_AXIS;
    int_marker.controls.push_back(control);
    control.name = "move_y";
    control.interaction_mode = InteractiveMarkerControl::MOVE_AXIS;
    int_marker.controls.push_back(control);
  }

  server->insert(int_marker);
  server->setCallback(int_marker.name, &processFeedback);

  if (interaction_mode != visualization_msgs::InteractiveMarkerControl::NONE)
    menu_handler.apply( *server, int_marker.name );
}

int main(int argc, char** argv)
{
  ros::init( argc, argv, "interactive_marker_server" );
  ros::NodeHandle n;

  ROS_INFO( "Started Interactive Marker Server" );
  server.reset( new interactive_markers::InteractiveMarkerServer("interactive_marker_server", "", false) );




    // A Publisher
    pub_obj_pose = n.advertise<geometry_msgs::PoseStamped>( "object_mesh_pose", 1000 );




  // string all_obj = string( "1.obj;chair.obj;simple_car.obj");
  string all_obj = string( "cube.obj");
  vector<string> objs = split( all_obj, ';' );
  cout << "size: " << objs.size() << endl;
  for( int i=0 ; i<objs.size() ; i++ )
    cout << objs[i] << endl;


  tf::Vector3 position;
  string resource_fname;

  for( int i=0 ; i<objs.size() ; i++ )
  {

  resource_fname = objs[i]; //"chair.obj"; //this should exist in nap/resources/
  ROS_INFO_STREAM( "Load Mesh: " << resource_fname );

  // TODO Read init pose file if available. Else put default poses.
  position = tf::Vector3( -3,3, 0 );
  ROS_INFO_STREAM( "Set Initial Pose: "<< position.getX() <<", " << position.getY() << ", " << position.getZ() );
  make6DofMarker( position,  resource_fname, true );

  //TODO: Only make marker if that resouce exists.
  }


  position = tf::Vector3( -10,3, 0 );
  make6DofMarker( position,  "X", false );


  server->applyChanges();





  ros::spin();

  server.reset();
}
