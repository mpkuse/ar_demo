#pragma once


/** Class to handle camera intrinsics

      Author  : Manohar Kuse <mpkuse@connect.ust.hk>
      Created : 3rd Oct, 2017
*/


#include <iostream>
#include <string>
#include <vector>
#include <fstream>
#include <queue>
#include <ostream>


#include <thread>
#include <mutex>
#include <atomic>

//opencv
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/calib3d.hpp>
#include <cv_bridge/cv_bridge.h>


#include <ros/ros.h>
#include <ros/package.h>



#include <Eigen/Dense>
#include <Eigen/Geometry>
using namespace Eigen;
#include <opencv2/core/eigen.hpp>

using namespace std;



class PinholeCamera {

public:
  PinholeCamera() { mValid = false; }
  PinholeCamera( string config_file );

  cv::Mat m_K; //3x3
  cv::Mat m_D; //4x1

  Matrix3d e_K;
  Vector4d e_D;
  // Caution, keeping these open. But rememebr opencv stores matrices as row major, while eigen stores them as col-major by default

  bool isValid() { return mValid; }
  double fx() { return _fx; }
  double fy() { return _fy; }
  double cx() { return _cx; }
  double cy() { return _cy; }
  double k1() { return _k1; }
  double k2() { return _k2; }
  double p1() { return _p1; }
  double p2() { return _p2; }

  string getModelType() { return config_model_type; }
  string getCameraName() { return config_camera_name; }
  string getConfigFileName() { return config_file_name; }
  int getImageWidth() { return config_image_width; }
  int getImageHeight() { return config_image_height; }

  int getImageRows() { return this->getImageHeight(); }
  int getImageCols() { return this->getImageWidth(); }


  // Given N 3D points,  3xN matrix, or 4xN matrix (homogeneous)
  // project these points using this camera.
  // TODO: Extend this function to include extrinsics (given as arguments)
  void perspectiveProject3DPoints( cv::Mat& _3dpts, cv::Mat& out_pts );
  void perspectiveProject3DPoints( const MatrixXd& _3dpts, MatrixXd& out_pts );
  void perspectiveProject3DPoints( cv::Mat& _3dpts, Matrix4f& T, cv::Mat& out_pts ); //< T * _3dpts

  // Given an input pointset 2xN 1-channel matrix, returns a 2xN 1-channel matrix
  // containing undistorted points on input
  // [Input]
  //    point set in observed image space
  // [Output]
  //    point set in undistorted image space
  void undistortPointSet( const cv::Mat& pts_observed_image_space, cv::Mat& pts_undistorted_image_space );

  // Given a) the pose of 2 cameras (to compute fundamental matrix), b) Point matches in observed space.
  // This function undisorts these point sets and correct matches to be consistent with the fundamental matrix.
  // See Optimal Triangulation in Hartley-Zizzerman book.
  // [Input]
  // F : Fundamental Matrix F_c_cm. 3x3
  // curr_pts_in_observed_image_space : 2xN
  // currm_pts_in_observed_image_space : 2xN
  // [Output]
  // curr_pts_undistorted : 2xN in undistorted image space
  // currm_pts_undistorted : 2xN in undistorted image space
  void jointUndistortPointSets( const cv::Mat& F,
                const cv::Mat& curr_pts_in_observed_image_space, const cv::Mat& currm_pts_in_observed_image_space,
               cv::Mat& curr_pts_undistorted, cv::Mat& currm_pts_undistorted);


   // Returns points in normalized_cords. Input being observed points in distorted image space.
   void getUndistortedNormalizedCords( const cv::Mat& pts_observed_image_space, cv::Mat& pts_undist_normed );




   // Given a point2d set in homogeneous co-ords (3xN eigen matrix)
   // in normalized image co-ordinates. This function shall doDistortion() followed
   // by scaling with camera intrinsic to give point these points in distorted_image_cords
   void normalizedImCords_2_imageCords( const MatrixXd& in_, MatrixXd& out_ );


private:
  string config_model_type, config_camera_name;
  string config_file_name;
  int config_image_width, config_image_height;

  double _fx, _fy, _cx, _cy;
  double _k1, _k2, _p1, _p2;

  bool mValid;


  // Utilities

  // given a 2xN input cv::Mat converts to 1xN 2-channel output. also assumes CV_32F type
  void _1channel_to_2channel( const cv::Mat& input, cv::Mat& output );

  // given a 1xN 2-channel input cv::Mat converts to 2xN,also assumes CV_32F type
  void _2channel_to_1channel( const cv::Mat& input, cv::Mat& output );

  void print_cvmat_info( string msg, const cv::Mat& A );
  string type2str( int );



  // Other Geometric Debugging
  // x is either 1ch 2xN or 2ch 1xN
  double verify_epipolar_constraint( const cv::Mat& x, const cv::Mat& F, const cv::Mat& xd );

  // Computes Gradient and Hessian of 5d state. Used in the optimal triangulation method
  void grad_and_hessian( const Matrix<double,5,1>& S,
                         const Matrix3d& F, const Vector3d& xl, const Vector3d& xr,
                         Matrix<double,5,1>& GRAD, Matrix<double,5,5>& HESS
                        );

};
