#include "EstimationFromPointClouds.h"


#define __EstimationFromPointClouds__estimate_ground_plane_from_pointcloud__( msg ) msg;
// #define __EstimationFromPointClouds__estimate_ground_plane_from_pointcloud__( msg ) ;
Eigen::VectorXf EstimationFromPointClouds::estimate_ground_plane_from_pointcloud(
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud,
    std::vector<int>& int_inliers, const float ransac_distance_threshold,
    const bool do_optimizeModelCoefficients
)
{
    __EstimationFromPointClouds__estimate_ground_plane_from_pointcloud__(
    cout << "====" << "Estimate Ground Plane, with do_optimizeModelCoefficients=" << do_optimizeModelCoefficients  << "====" << endl;
    cout << "Input cloud has " << cloud->width * cloud->height << "data points" << endl;
    )

    pcl::SampleConsensusModelPlane<pcl::PointXYZ>::Ptr model (new pcl::SampleConsensusModelPlane<pcl::PointXYZ> (cloud));
    pcl::RandomSampleConsensus<pcl::PointXYZ> sac (model, ransac_distance_threshold);
    bool result = sac.computeModel();
    assert( result );


    int_inliers.clear();
    sac.getInliers (int_inliers);
    __EstimationFromPointClouds__estimate_ground_plane_from_pointcloud__(
    cout << "ground plane estimate #inliers=" << int_inliers.size() << endl;
    // for( int i=0 ; i<int_inliers.size() ; i++ ) cout << int_inliers[i] << " ";
    // cout << endl;
    )

    Eigen::VectorXf coeff;
    sac.getModelCoefficients (coeff);
    __EstimationFromPointClouds__estimate_ground_plane_from_pointcloud__(
    cout << "coeff  [normal_x normal_y normal_z d]: " << coeff.transpose() << endl;
    )

    if( do_optimizeModelCoefficients ) {
        Eigen::VectorXf coeff_refined;
        model->optimizeModelCoefficients (int_inliers, coeff, coeff_refined);
        __EstimationFromPointClouds__estimate_ground_plane_from_pointcloud__(
        cout << "coeff_refined: " << coeff_refined.transpose() << endl;
        cout << "==== DONE __EstimationFromPointClouds__estimate_ground_plane_from_pointcloud__====\n";
        )
        return coeff_refined;
    }
    else {
        __EstimationFromPointClouds__estimate_ground_plane_from_pointcloud__(
        cout << "==== DONE __EstimationFromPointClouds__estimate_ground_plane_from_pointcloud__====\n";)
        return coeff;
    }

}


#define __EstimationFromPointClouds__random__pt_on_plane__(msg) msg;
// #define __EstimationFromPointClouds__random__pt_on_plane__(msg) ;
Vector3d EstimationFromPointClouds::random_pt_on_plane( const VectorXd coeff,
    float fv1_min, float fv1_max,
    float fv2_min, float fv2_max
  )
{

    assert( coeff.rows() == 4 );
    double a = coeff[0];
    double b = coeff[1];
    double c = coeff[2];
    double d = coeff[3];
    __EstimationFromPointClouds__random__pt_on_plane__(
    cout << "Input Eq: "<< a << "x + " << b << "y + " << c << "z + " << d << " = 0\n";)


    // assuming min is less than max
    unsigned seed = std::chrono::system_clock::now().time_since_epoch().count();
    std::default_random_engine generator(seed);
    std::uniform_real_distribution<double> fv1(  min(fv1_min,fv1_max),  max(fv1_min,fv1_max)  );
    std::uniform_real_distribution<double> fv2(  min(fv2_min,fv2_max),  max(fv2_min,fv2_max)  );



    // TODO: this treats a,b as free variable, this wont work when c is zero,
    if( abs(c) > 1e-3 ) {
        __EstimationFromPointClouds__random__pt_on_plane__(
        cout << "[random_pt_on_plane]since coeff.c is non-zero we can have x_ and y_ as free variables\n";)
        double x_ = fv1(generator); //drand48() * 10;
        double y_ = fv2(generator); // drand48() * 10;
        double z_ = -( a*x_ + b*y_ + d ) / c; //this assumes c is non zero
        Vector3d plane_a( x_, y_, z_ ); //!< point A on the plane
        __EstimationFromPointClouds__random__pt_on_plane__(cout << "[random_pt_on_plane]return generated point on plane : " << plane_a.transpose() << endl;)
        return plane_a;
    } else { __EstimationFromPointClouds__random__pt_on_plane__(cout << "[random_pt_on_plane]coeff.c is zero, so try coeff.b\n";) }

    if( abs(b) > 1e-3 ) {
        __EstimationFromPointClouds__random__pt_on_plane__(
        cout << "[random_pt_on_plane]since coeff.b is non-zero we can have x_ and z_ as free variables\n";)
        double x_ = fv1(generator); // drand48() * 10;
        double z_ = fv2(generator); //drand48() * 10;
        double y_ = -( a*x_ + c*z_ + d ) / b; //this assumes c is non zero
        Vector3d plane_a( x_, y_, z_ ); //!< point A on the plane
        __EstimationFromPointClouds__random__pt_on_plane__(cout << "[random_pt_on_plane]return generated point on plane : " << plane_a.transpose() << endl;)
        return plane_a;
    } else { __EstimationFromPointClouds__random__pt_on_plane__(cout << "[random_pt_on_plane]coeff.b is zero, so try coeff.a\n";) }

    if( abs(a) > 1e-3 ) {
        __EstimationFromPointClouds__random__pt_on_plane__(
        cout << "[random_pt_on_plane]since coeff.b is non-zero we can have y_ and z_ as free variables\n";)
        double y_ = fv1(generator); //drand48() * 10;
        double z_ = fv2(generator); //drand48() * 10;
        double x_ = -( b*y_ + c*z_ + d ) / a; //this assumes c is non zero
        Vector3d plane_a( x_, y_, z_ ); //!< point A on the plane
        __EstimationFromPointClouds__random__pt_on_plane__(cout << "[random_pt_on_plane]return generated point on plane : " << plane_a.transpose() << endl;)
        return plane_a;
    } else { __EstimationFromPointClouds__random__pt_on_plane__(cout << "[random_pt_on_plane]coeff.a is zero, this means all of a,b,c were zero, this looks like an invalid plane equation to me. \n";) }
    exit(2);
}



#define __EstimationFromPointClouds__estimate_plane_orientation_wRp__( msg ) msg;
// #define __EstimationFromPointClouds__estimate_plane_orientation_wRp__( msg ) ;
Matrix3d EstimationFromPointClouds::estimate_plane_orientation_wRp( VectorXd coeff )
{
    __EstimationFromPointClouds__estimate_plane_orientation_wRp__(
    cout << "====__EstimationFromPointClouds__estimate_plane_orientation_wRp__====\n";)

    assert( coeff.rows() == 4 );
    // double a = coeff[0];
    // double b = coeff[1];
    // double c = coeff[2];
    // double d = coeff[3];
    __EstimationFromPointClouds__estimate_plane_orientation_wRp__( cout << "coeff: " << coeff.transpose() << endl; )



    // a) Get a unit vector which is normal to the plane from equation of the plane (the plane's z axis)
    Vector3d plane_z( coeff.topRows(3) ); //!< Plane's z-axis
    if( coeff[2] < 0 )
        plane_z = -plane_z;
    plane_z.normalize();

    // b) Get 2 points on the plane (to make a vector) on the plane (the planes's x axis)
    Vector3d plane_a = EstimationFromPointClouds::random_pt_on_plane( coeff );
    __EstimationFromPointClouds__estimate_plane_orientation_wRp__(
    cout << "PointA (on the plane): " << plane_a.transpose() << endl;)


    Vector3d plane_b = EstimationFromPointClouds::random_pt_on_plane( coeff );
    __EstimationFromPointClouds__estimate_plane_orientation_wRp__(
    cout << "PointB (on the plane): " << plane_b.transpose() << endl; )

    Vector3d plane_x = plane_b - plane_a;
    plane_x.normalize();

    // c) cross product x and (-z) to get the plane's y axis
    Vector3d plane_y = -PoseManipUtils::vec_to_cross_matrix(plane_x) * plane_z;
    __EstimationFromPointClouds__estimate_plane_orientation_wRp__(
    cout << "plane_x: " << plane_x.transpose() << endl;
    cout << "plane_y: " << plane_y.transpose() << endl;
    cout << "plane_z: " << plane_z.transpose() << endl; )

    Matrix3d wRp;
    wRp.col(0) = plane_x;
    wRp.col(1) = plane_y;
    wRp.col(2) = plane_z;
    __EstimationFromPointClouds__estimate_plane_orientation_wRp__(
    cout << "==== DONE __EstimationFromPointClouds__estimate_plane_orientation_wRp__ ====\n";)
    return wRp;
}
