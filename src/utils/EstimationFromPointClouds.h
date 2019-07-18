#pragma once

#include <iostream>
#include <chrono>
#include <random>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/sample_consensus/sac_model_plane.h>
#include <pcl/sample_consensus/ransac.h>
#include <pcl/filters/extract_indices.h>

#include "PoseManipUtils.h"



/*! @class Collection of functions which act on point clouds or other utils on 3d
*/
class EstimationFromPointClouds
{
public:
    /*! Estimates the equation of ground plane from an input pointcloud

        @param cloud: Input pointcloud
        @param int_inliers : indices of the inliers as per RANSAC
        @param ransac_distance_threshold: Ransac distance threshold. The point is considered inlier if distance to the fitted model is less than this value.
        @param do_optimizeModelCoefficients: Setting this to true will perform (and return) coeff after non linear optimization.
    */
    static Eigen::VectorXf estimate_ground_plane_from_pointcloud(
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud,
        std::vector<int>& int_inliers, const float ransac_distance_threshold=0.05,
        const bool do_optimizeModelCoefficients = true
    );



    /*! Given the equation of the plane, returns a random point on the plane

        To generate a random point on the plane in 3d, we have 2 free variables.
        The 3rd is determined from the plane's analytic equation.

        @param coeff :  [normal_x normal_y normal_z d]
        @param fv1_min, fv1_max: Free variable 1 limits.
        @param fv2_min, fv2_max: Free variable 2 limits.
        @param[return] : a random point
    */
    static Vector3d random_pt_on_plane( const VectorXd coeff,
        float fv1_min=-10.0, float fv1_max=10.0,
        float fv2_min=-10.0, float fv2_max=10.0
    );


    /*! Given the equation of the plane's coefficients, return a orientation of the plane wrt to world

        @param coeff :  [normal_x normal_y normal_z d]
    */
    static Matrix3d estimate_plane_orientation_wRp( VectorXd coeff );


};
