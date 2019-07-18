#include <iostream>
#include <chrono>
#include <random>

#include <ros/ros.h>
#include <string>
#include <geometry_msgs/PoseStamped.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl/filters/statistical_outlier_removal.h>

#include <pcl/sample_consensus/sac_model_plane.h>
#include <pcl/sample_consensus/ransac.h>
#include <pcl/features/normal_3d.h>
#include <pcl/sample_consensus/sac_model_cylinder.h>
#include <pcl/sample_consensus/sac_model_sphere.h>


#include <pcl/filters/extract_indices.h>

#include "../utils/PoseManipUtils.h"
#include "../utils/RosMarkerUtils.h"


using namespace std;

void init_plane_marker_from_eq_coeff( visualization_msgs::Marker& marker, Eigen::VectorXf coeff )
{
    assert( coeff.rows() == 4 );
    float a = coeff[0];
    float b = coeff[1];
    float c = coeff[2];
    float d = coeff[3];

    marker.header.frame_id = "world";
    marker.header.stamp = ros::Time::now();
    marker.action = visualization_msgs::Marker::ADD;
    marker.color.a = .8; // Don't forget to set the alpha!
    marker.type = visualization_msgs::Marker::TRIANGLE_LIST;

    marker.scale.x = 1.02;
    marker.scale.y = 1.02;
    marker.scale.z = 1.02;



    //// Done . no need to edit firther
    marker.pose.position.x = 0.;
    marker.pose.position.y = 0.;
    marker.pose.position.z = 0.;
    marker.pose.orientation.x = 0.;
    marker.pose.orientation.y = 0.;
    marker.pose.orientation.z = 0.;
    marker.pose.orientation.w = 1.;
    // marker.id = i;
    // marker.ns = "camerapose_visual";
    marker.color.r = 0.2;marker.color.b = 0.;marker.color.g = 0.;



    marker.points.clear();
    //just assuming neither of a,b,c are zeros
    geometry_msgs::Point pt;
    pt.x = 0.0; pt.y = 0.0; pt.z = -d/c;
    marker.points.push_back( pt );
    pt.x = 0.0; pt.y = -d/b ; pt.z = 0.0;
    marker.points.push_back( pt );
    pt.x = -d/a; pt.y = 0.0; pt.z = 0.0;
    marker.points.push_back( pt );

    std_msgs::ColorRGBA vertex_color;
    vertex_color.r = 1.0; vertex_color.r = 1.0; vertex_color.r = 1.0; vertex_color.a = .6;
    marker.colors.push_back( vertex_color );
    marker.colors.push_back( vertex_color );
    marker.colors.push_back( vertex_color );
}


void init_sphere_marker( visualization_msgs::Marker& marker, float cx, float cy, float cz, float radius )
{
    marker.header.frame_id = "world";
    marker.header.stamp = ros::Time();
    // marker.ns = "sphere";
    // marker.id = 0;
    marker.type = visualization_msgs::Marker::SPHERE;
    marker.action = visualization_msgs::Marker::ADD;
    marker.pose.position.x = cx;
    marker.pose.position.y = cy;
    marker.pose.position.z = cz;
    marker.pose.orientation.x = 0.0;
    marker.pose.orientation.y = 0.0;
    marker.pose.orientation.z = 0.0;
    marker.pose.orientation.w = 1.0;
    marker.scale.x = radius;
    marker.scale.y = radius;
    marker.scale.z = radius;
    marker.color.a = 0.6; // Don't forget to set the alpha!
    marker.color.r = 0.0;
    marker.color.g = 1.0;
    marker.color.b = 0.0;
}


/*! Estimates the equation of ground plane from an input pointcloud

    @param cloud: Input pointcloud
    @param int_inliers : indices of the inliers as per RANSAC
    @param ransac_distance_threshold: Ransac distance threshold. The point is considered inlier if distance to the fitted model is less than this value.
    @param do_optimizeModelCoefficients: Setting this to true will perform (and return) coeff after non linear optimization.
*/
#define __estimate_ground_plane_from_pointcloud__( msg ) msg;
// #define __estimate_ground_plane_from_pointcloud__( msg ) ;

Eigen::VectorXf
estimate_ground_plane_from_pointcloud(
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud,
    std::vector<int>& int_inliers, const float ransac_distance_threshold=0.05,
    const bool do_optimizeModelCoefficients = true
)
{
    __estimate_ground_plane_from_pointcloud__(
    cout << "====" << "Estimate Ground Plane, with do_optimizeModelCoefficients=" << do_optimizeModelCoefficients << "\n" << "====" << endl;
    cout << "Input cloud has " << cloud->width * cloud->height << "data points" << endl;
    )

    pcl::SampleConsensusModelPlane<pcl::PointXYZ>::Ptr model (new pcl::SampleConsensusModelPlane<pcl::PointXYZ> (cloud));
    pcl::RandomSampleConsensus<pcl::PointXYZ> sac (model, ransac_distance_threshold);
    bool result = sac.computeModel();
    assert( result );


    int_inliers.clear();
    sac.getInliers (int_inliers);
    __estimate_ground_plane_from_pointcloud__(
    cout << "ground plane estimate #inliers=" << int_inliers.size() << endl;
    // for( int i=0 ; i<int_inliers.size() ; i++ ) cout << int_inliers[i] << " ";
    // cout << endl;
    )

    Eigen::VectorXf coeff;
    sac.getModelCoefficients (coeff);
    __estimate_ground_plane_from_pointcloud__(
    cout << "coeff  [normal_x normal_y normal_z d]" << coeff.transpose() << endl;
    )

    if( do_optimizeModelCoefficients ) {
        Eigen::VectorXf coeff_refined;
        model->optimizeModelCoefficients (int_inliers, coeff, coeff_refined);
        __estimate_ground_plane_from_pointcloud__(
        cout << "coeff_refined: " << coeff_refined.transpose() << endl;
        )
        return coeff_refined;
    }
    else {
        return coeff;
    }

}

/*! Given the equation of the plane, returns a random point on the plane

    To generate a random point on the plane in 3d, we have 2 free variables.
    The 3rd is determined from the plane's analytic equation.

    @param coeff :  [normal_x normal_y normal_z d]
    @param fv1_min, fv1_max: Free variable 1 limits.
    @param fv2_min, fv2_max: Free variable 2 limits.
    @param[return] : a random point
*/
#define __random__pt_on_plane__(msg) msg;
// #define __random__pt_on_plane__(msg) ;
Vector3d random_pt_on_plane( const VectorXd coeff,
    float fv1_min=-10.0, float fv1_max=10.0,
    float fv2_min=-10.0, float fv2_max=10.0
  )
{

    assert( coeff.rows() == 4 );
    double a = coeff[0];
    double b = coeff[1];
    double c = coeff[2];
    double d = coeff[3];
    __random__pt_on_plane__(
    cout << "Input Eq: "<< a << "x + " << b << "y + " << c << "z + " << d << " = 0";)


    // assuming min is less than max
    unsigned seed = std::chrono::system_clock::now().time_since_epoch().count();
    std::default_random_engine generator(seed);
    std::uniform_real_distribution<double> fv1(  min(fv1_min,fv1_max),  max(fv1_min,fv1_max)  );
    std::uniform_real_distribution<double> fv2(  min(fv2_min,fv2_max),  max(fv2_min,fv2_max)  );



    // TODO: this treats a,b as free variable, this wont work when c is zero,
    if( abs(c) > 1e-3 ) {
        __random__pt_on_plane__(
        cout << "[random_pt_on_plane]since coeff.c is non-zero we can have x_ and y_ as free variables\n";)
        double x_ = fv1(generator); //drand48() * 10;
        double y_ = fv2(generator); // drand48() * 10;
        double z_ = -( a*x_ + b*y_ + d ) / c; //this assumes c is non zero
        Vector3d plane_a( x_, y_, z_ ); //!< point A on the plane
        __random__pt_on_plane__(cout << "[random_pt_on_plane]return generated point on plane : " << plane_a.transpose() << endl;)
        return plane_a;
    } else { __random__pt_on_plane__(cout << "[random_pt_on_plane]coeff.c is zero, so try coeff.b\n";) }

    if( abs(b) > 1e-3 ) {
        __random__pt_on_plane__(
        cout << "[random_pt_on_plane]since coeff.b is non-zero we can have x_ and z_ as free variables\n";)
        double x_ = fv1(generator); // drand48() * 10;
        double z_ = fv2(generator); //drand48() * 10;
        double y_ = -( a*x_ + c*z_ + d ) / b; //this assumes c is non zero
        Vector3d plane_a( x_, y_, z_ ); //!< point A on the plane
        __random__pt_on_plane__(cout << "[random_pt_on_plane]return generated point on plane : " << plane_a.transpose() << endl;)
        return plane_a;
    } else { __random__pt_on_plane__(cout << "[random_pt_on_plane]coeff.b is zero, so try coeff.a\n";) }

    if( abs(a) > 1e-3 ) {
        __random__pt_on_plane__(
        cout << "[random_pt_on_plane]since coeff.b is non-zero we can have y_ and z_ as free variables\n";)
        double y_ = fv1(generator); //drand48() * 10;
        double z_ = fv2(generator); //drand48() * 10;
        double x_ = -( b*y_ + c*z_ + d ) / a; //this assumes c is non zero
        Vector3d plane_a( x_, y_, z_ ); //!< point A on the plane
        __random__pt_on_plane__(cout << "[random_pt_on_plane]return generated point on plane : " << plane_a.transpose() << endl;)
        return plane_a;
    } else { __random__pt_on_plane__(cout << "[random_pt_on_plane]coeff.a is zero, this means all of a,b,c were zero, this looks like an invalid plane equation to me. \n";) }
    exit(2);
}

/*! Given the equation of the plane's coefficients, return a orientation of the plane wrt to world

    @param coeff :  [normal_x normal_y normal_z d]
*/
#define __estimate_plane_orientation_wRp__( msg ) msg;
// #define __estimate_plane_orientation_wRp__( msg ) ;
Matrix3d estimate_plane_orientation_wRp( VectorXd coeff )
{
    __estimate_plane_orientation_wRp__(
    cout << "====__estimate_plane_orientation_wRp__====\n";)

    assert( coeff.rows() == 4 );
    // double a = coeff[0];
    // double b = coeff[1];
    // double c = coeff[2];
    // double d = coeff[3];
    __estimate_plane_orientation_wRp__( cout << "coeff: " << coeff.transpose() << endl; )



    // a) Get a unit vector which is normal to the plane from equation of the plane (the plane's z axis)
    Vector3d plane_z( coeff.topRows(3) ); //!< Plane's z-axis
    if( coeff[2] < 0 )
        plane_z = -plane_z;
    plane_z.normalize();

    // b) Get 2 points on the plane (to make a vector) on the plane (the planes's x axis)
    Vector3d plane_a = random_pt_on_plane( coeff );
    __estimate_plane_orientation_wRp__(
    cout << "PointA (on the plane): " << plane_a.transpose() << endl;)


    Vector3d plane_b = random_pt_on_plane( coeff );
    __estimate_plane_orientation_wRp__(
    cout << "PointB (on the plane): " << plane_b.transpose() << endl; )

    Vector3d plane_x = plane_b - plane_a;
    plane_x.normalize();

    // c) cross product x and (-z) to get the plane's y axis
    Vector3d plane_y = -PoseManipUtils::vec_to_cross_matrix(plane_x) * plane_z;
    __estimate_plane_orientation_wRp__(
    cout << "plane_x: " << plane_x.transpose() << endl;
    cout << "plane_y: " << plane_y.transpose() << endl;
    cout << "plane_z: " << plane_z.transpose() << endl; )

    Matrix3d wRp;
    wRp.col(0) = plane_x;
    wRp.col(1) = plane_y;
    wRp.col(2) = plane_z;
    __estimate_plane_orientation_wRp__(
    cout << "==== DONE __estimate_plane_orientation_wRp__ ====\n";)
    return wRp;
}


#if 0
int main(int argc, char ** argv )
{

    //--- init rosnode
    ros::init(argc, argv, "unit_test_estimate_ground_plane");
    ros::NodeHandle nh("~");



    //--- setup publisher for marker_topic
    ros::Publisher pub_model = nh.advertise<visualization_msgs::Marker>("models", 1000);
    ros::Publisher pub_pcl = nh.advertise<sensor_msgs::PointCloud2>( "pt_cld", 100 );



    //--- Load .pcd file
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>);
    if (pcl::io::loadPCDFile<pcl::PointXYZ>("/tmp/estimate.pcd", *cloud) == -1) //* load the file
    {
        PCL_ERROR ("Couldn't read file test_pcd.pcd \n");
        return (-1);
    }
    std::cout << "Loaded "
            << cloud->width * cloud->height
            << " data points from test_pcd.pcd with the following fields: "
            << std::endl;


    //--- estimate ground plane
        // Adopted from https://github.com/PointCloudLibrary/pcl/blob/master/test/sample_consensus/test_sample_consensus_plane_models.cpp
    Eigen::VectorXf coeff; // Equation of plane [normal_x normal_y normal_z d]
    std::vector<int> int_inliers;
    coeff = estimate_ground_plane_from_pointcloud( cloud, int_inliers, 0.05, true );
    assert( coeff.rows() == 4 );

    //--- publish ground plane
    #if 0
    visualization_msgs::Marker pmarker;
    init_plane_marker_from_eq_coeff( pmarker, coeff );
    pmarker.ns = "ground_plane";
    pmarker.id = 0;
    #endif

    #if 0
    cout << "====" << "Estimate Ground Plane\n" << "====" << endl;
    pcl::SampleConsensusModelPlane<pcl::PointXYZ>::Ptr model (new pcl::SampleConsensusModelPlane<pcl::PointXYZ> (cloud));
    pcl::RandomSampleConsensus<pcl::PointXYZ> sac (model, 0.05);
    bool result = sac.computeModel();
    assert( result );

    // std::vector<int> sample;
    // sac.getModel (sample);

    std::vector<int> int_inliers;
    sac.getInliers (int_inliers);
    cout << "ground plane estimate #inliers=" << int_inliers.size() << endl;
    // for( int i=0 ; i<int_inliers.size() ; i++ ) cout << int_inliers[i] << " ";
    // cout << endl;

    Eigen::VectorXf coeff;
    sac.getModelCoefficients (coeff);
    cout << "coeff  [normal_x normal_y normal_z d]" << coeff.transpose() << endl;

    Eigen::VectorXf coeff_refined;
    model->optimizeModelCoefficients (int_inliers, coeff, coeff_refined);
    cout << "coeff_refined: " << coeff_refined.transpose() << endl;


    //--- publish ground plane
    visualization_msgs::Marker pmarker;
    init_plane_marker_from_eq_coeff( pmarker, coeff );
    pmarker.ns = "ground_plane";
    pmarker.id = 0;
    cout << "=== DONE ====\n";
    #endif


    //--- from the original point cloud remove the ground plane 3d points
    #if 1
    cout << "====" << "Remove 3d points of ground plane from the point cloud" << "====" << endl;
    pcl::PointIndices::Ptr inliers(new pcl::PointIndices());
    for( int i=0 ; i<int_inliers.size() ; i++ )
        inliers->indices.push_back(int_inliers[i]);

    pcl::ExtractIndices<pcl::PointXYZ> extract;
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_p (new pcl::PointCloud<pcl::PointXYZ>);
    extract.setInputCloud(cloud);
    extract.setIndices(inliers);
    extract.setNegative(true);
    extract.filter(*cloud_p);
    cout << "ExtractIndices, cloud_p has " << cloud_p->width * cloud_p->height << endl;
    cout << "=== DONE ===\n";
    #endif



    //--- wTp pose to set an object on ground plane
    Matrix3d wRp = estimate_plane_orientation_wRp( coeff.cast<double>() );
    cout << "wRp:\n" << wRp << endl;

    Matrix4d wTp = Matrix4d::Identity();
    wTp.topLeftCorner(3,3) = wRp;
    wTp.col(3).topRows(3) = random_pt_on_plane( coeff.cast<double>(), 2, 3, -.5, .5 );
    cout << "wTp:\n" << wTp << endl;

    //---- mark center (of the plane) with translation as a random point satisfying equation of plane
    visualization_msgs::Marker axis_marker;
    RosMarkerUtils::init_XYZ_axis_marker( axis_marker );
    axis_marker.ns = "on_plane_axis"; axis_marker.id = 0;
    RosMarkerUtils::setpose_to_marker( wTp, axis_marker );


    //--- Plot a ground plane
    visualization_msgs::Marker gplane;
    RosMarkerUtils::init_plane_marker(  gplane, 20, 10 );
    gplane.ns = "gplane";
    gplane.id = 0;
    RosMarkerUtils::setpose_to_marker( wTp, gplane );

    //--- spin
    ros::Rate rate(10);
    while( ros::ok() ) {
        ros::spinOnce();

        // pub_model.publish( pmarker );

        sensor_msgs::PointCloud2 cloud_msg;
        // pcl::toROSMsg( *cloud, cloud_msg );
        pcl::toROSMsg( *cloud_p, cloud_msg );
        cloud_msg.header.frame_id = "world";
        pub_pcl.publish( cloud_msg );

        // pub_model.publish( sp_marker );
        pub_model.publish( axis_marker );
        pub_model.publish( gplane );

        rate.sleep();

    }


}

#endif


#include "../utils/EstimationFromPointClouds.h"

int main(int argc, char ** argv )
{
    //--- init rosnode
    ros::init(argc, argv, "unit_test_estimate_ground_plane");
    ros::NodeHandle nh("~");



    //--- setup publisher for marker_topic
    ros::Publisher pub_model = nh.advertise<visualization_msgs::Marker>("models", 1000);
    ros::Publisher pub_pcl = nh.advertise<sensor_msgs::PointCloud2>( "pt_cld", 100 );



    //--- Load .pcd file
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>);
    if (pcl::io::loadPCDFile<pcl::PointXYZ>("/tmp/estimate.pcd", *cloud) == -1) //* load the file
    {
        PCL_ERROR ("Couldn't read file test_pcd.pcd \n");
        return (-1);
    }
    std::cout << "Loaded "
            << cloud->width * cloud->height
            << " data points from test_pcd.pcd with the following fields: "
            << std::endl;



    //--- Estimate ground plane
    vector<int> int_inliers;
    VectorXf coeff = EstimationFromPointClouds::estimate_ground_plane_from_pointcloud( cloud, int_inliers, 0.05, true );
    assert( coeff.rows() == 4 );
    cout << "[main] coeff of ground plane: " << coeff.transpose() << endl;


    //--- wTp pose to set an object on ground plane (orientation of ground plane)
    Matrix3d wRp = EstimationFromPointClouds::estimate_plane_orientation_wRp( coeff.cast<double>() );
    cout << "[main]wRp:\n" << wRp << endl;

    Matrix4d wTp = Matrix4d::Identity();
    wTp.topLeftCorner(3,3) = wRp;
    wTp.col(3).topRows(3) = EstimationFromPointClouds::random_pt_on_plane( coeff.cast<double>(), 2, 3, -.5, .5 );
    cout << "[main]wTp:\n" << wTp << endl;


    //--- Plot Axis and Ground Plane with wTp as the pose on rviz
    visualization_msgs::Marker gplane;
    RosMarkerUtils::init_plane_marker(  gplane, 20, 10 );
    gplane.ns = "ground_plane";
    gplane.id = 0;
    RosMarkerUtils::setpose_to_marker( wTp, gplane );

    visualization_msgs::Marker axis_marker;
    RosMarkerUtils::init_XYZ_axis_marker( axis_marker );
    axis_marker.ns = "on_plane_axis";
    axis_marker.id = 0;
    RosMarkerUtils::setpose_to_marker( wTp, axis_marker );


    //--spin
    ros::Rate rate(10);
    while( ros::ok() ) {
        ros::spinOnce();
        rate.sleep();

        sensor_msgs::PointCloud2 cloud_msg;
        pcl::toROSMsg( *cloud, cloud_msg );
        // pcl::toROSMsg( *cloud_p, cloud_msg );
        cloud_msg.header.frame_id = "world";
        pub_pcl.publish( cloud_msg );


        pub_model.publish( axis_marker );
        pub_model.publish( gplane );
    }

}
