#include "PinholeCamera.h"


PinholeCamera::PinholeCamera( string config_file )
{

  cv::FileStorage fs( config_file, cv::FileStorage::READ );
  if( !fs.isOpened() )
  {
    ROS_ERROR( "Cannot open config file : %s", config_file.c_str() );
    ROS_ERROR( "Quit");
    mValid = false;
    exit(1);
  }
  this->config_file_name = string(config_file);

  cout << "---Camera Config---\n";
  fs["model_type"] >> config_model_type;     cout << "config_model_type:"<< config_model_type << endl;
  fs["camera_name"] >> config_camera_name;   cout << "config_camera_name:" << config_camera_name << endl;
  fs["image_width"] >> config_image_width;   cout << "config_image_width:" << config_image_width << endl;
  fs["image_height"] >> config_image_height; cout << "config_image_height:" << config_image_height << endl;

  if( config_model_type != string("PINHOLE") )
  {
      cout << "[PinholeCamera::PinholeCamera] I can only process PinholeCamera. this is a deprecated class. Use camodocal's abstract camera\n";
      exit(10);
  }

  fs["projection_parameters"]["fx"] >> _fx;
  fs["projection_parameters"]["fy"] >> _fy;
  fs["projection_parameters"]["cx"] >> _cx;
  fs["projection_parameters"]["cy"] >> _cy;
  // Scaling to adjust for keyframe image resize. TODO: the scaling factors need to be infered from config_image_width and config_image_height compared to (320,240)
  cout << "Scaling camera intrinsics to adjust for image resizing\n";
  double x_scale = config_image_width / 320.0; //3.0
  double y_scale = config_image_height / 240.0; //2.5
  x_scale=1.0;
  y_scale=1.0;
  _fx /= x_scale;
  _cx /= x_scale;
  _fy /= y_scale;
  _cy /= y_scale;
  cout << "projection_parameters :: " << _fx << " " << _fy << " " << _cx << " " << _cy << " " << endl;

  fs["distortion_parameters"]["k1"] >> _k1;
  fs["distortion_parameters"]["k2"] >> _k2;
  fs["distortion_parameters"]["p1"] >> _p1;
  fs["distortion_parameters"]["p2"] >> _p2;
  cout << "distortion_parameters :: " << _k1 << " " << _k2 << " " << _p1 << " " << _p2 << " " << endl;
  cout << "---    ---\n";

  // Define the 3x3 Projection matrix eigen and/or cv::Mat.
  m_K = cv::Mat::zeros( 3,3,CV_32F );
  m_K.at<float>(0,0) = _fx;
  m_K.at<float>(1,1) = _fy;
  m_K.at<float>(0,2) = _cx;
  m_K.at<float>(1,2) = _cy;
  m_K.at<float>(2,2) = 1.0;

  m_D = cv::Mat::zeros( 4, 1, CV_32F );
  m_D.at<float>(0,0) = _k1;
  m_D.at<float>(1,0) = _k2;
  m_D.at<float>(2,0) = _p1;
  m_D.at<float>(3,0) = _p2;
  cout << "m_K" << m_K << endl;
  cout << "m_D" << m_D << endl;

  // Define 4x1 vector of distortion params eigen and/or cv::Mat.
  e_K << _fx, 0.0, _cx,
        0.0,  _fy, _cy,
        0.0, 0.0, 1.0;

  e_D << _k1 , _k2, _p1 , _p2;
  cout << "e_K" << m_K << endl;
  cout << "e_D" << m_D << endl;
  mValid = true;

}



void PinholeCamera::_1channel_to_2channel( const cv::Mat& input, cv::Mat& output )
{
  assert( input.rows == 2 && input.channels()==1 );
  output = cv::Mat( 1, input.cols, CV_32FC2 );
  for( int l=0 ; l<input.cols ; l++ )
  {
    output.at<cv::Vec2f>(0,l)[0] = input.at<float>(0,l);
    output.at<cv::Vec2f>(0,l)[1] = input.at<float>(1,l);
  }

}

void PinholeCamera::_2channel_to_1channel( const cv::Mat& input, cv::Mat& output )
{
  assert( input.rows == 1 && input.channels()==2 );
  output = cv::Mat( 2, input.cols, CV_32F );
  for( int l=0 ; l<input.cols ; l++ )
  {
    output.at<float>(0,l) = input.at<cv::Vec2f>(0,l)[0];
    output.at<float>(1,l) = input.at<cv::Vec2f>(0,l)[1];
  }
}



//////////////////////////////////////////
void PinholeCamera::perspectiveProject3DPoints( cv::Mat& _3dpts, Matrix4f& T,
                                  cv::Mat& out_pts  )
{
  MatrixXf c_X;
  cv::cv2eigen( _3dpts, c_X ); //3xN

  MatrixXf cm_X;
  cm_X = T * c_X;

  cv::Mat _3dpts_cm;
  cv::eigen2cv( cm_X, _3dpts_cm );

  perspectiveProject3DPoints( _3dpts_cm, out_pts );
}

// Input 3d points in homogeneous co-ordinates 4xN matrix. Eigen I/O
void PinholeCamera::perspectiveProject3DPoints( const MatrixXd& c_X,
                              MatrixXd& out_pts )
{

    // DIY - Do It Yourself Projection
    // c_X.row(0).array() /= c_X.row(3).array();
    // c_X.row(1).array() /= c_X.row(3).array();
    // c_X.row(2).array() /= c_X.row(3).array();
    // c_X.row(3).array() /= c_X.row(3).array();



    // K [ I | 0 ]
    MatrixXd I_0;
    I_0 = Matrix4d::Identity().topLeftCorner<3,4>();
    // MatrixXf P1;
    // P1 = cam_intrin * I_0; //3x4

    // Project and Perspective Divide
    MatrixXd im_pts;
    im_pts = I_0 * c_X; //in normalized image co-ordinate. Beware that distortion need to be applied in normalized co-ordinates
    im_pts.row(0).array() /= im_pts.row(2).array();
    im_pts.row(1).array() /= im_pts.row(2).array();
    im_pts.row(2).array() /= im_pts.row(2).array();

    // Apply Distortion
    MatrixXd Xdd = MatrixXd( im_pts.rows(), im_pts.cols() );
    for( int i=0 ; i<im_pts.cols() ; i++)
    {
      double r2 = im_pts(0,i)*im_pts(0,i) + im_pts(1,i)*im_pts(1,i);
      double c = 1.0f + (double)k1()*r2 + (double)k2()*r2*r2;
      Xdd(0,i) = im_pts(0,i) * c + 2.0f*(double)p1()*im_pts(0,i)*im_pts(1,i) + (double)p2()*(r2 + 2.0*im_pts(0,i)*im_pts(0,i));
      Xdd(1,i) = im_pts(1,i) * c + 2.0f*(double)p2()*im_pts(0,i)*im_pts(1,i) + (double)p1()*(r2 + 2.0*im_pts(1,i)*im_pts(1,i));
      Xdd(2,i) = 1.0f;
    }

    out_pts = e_K * Xdd;


}

void PinholeCamera::normalizedImCords_2_imageCords( const MatrixXd& im_pts, MatrixXd& out_pts )
{
  assert( im_pts.rows() == 3 );

  // Apply Distortion
  MatrixXd Xdd = MatrixXd( im_pts.rows(), im_pts.cols() );
  for( int i=0 ; i<im_pts.cols() ; i++)
  {
    double r2 = im_pts(0,i)*im_pts(0,i) + im_pts(1,i)*im_pts(1,i);
    double c = 1.0f + (double)k1()*r2 + (double)k2()*r2*r2;
    Xdd(0,i) = im_pts(0,i) * c + 2.0f*(double)p1()*im_pts(0,i)*im_pts(1,i) + (double)p2()*(r2 + 2.0*im_pts(0,i)*im_pts(0,i));
    Xdd(1,i) = im_pts(1,i) * c + 2.0f*(double)p2()*im_pts(0,i)*im_pts(1,i) + (double)p1()*(r2 + 2.0*im_pts(1,i)*im_pts(1,i));
    Xdd(2,i) = 1.0f;
  }

  out_pts = e_K * Xdd;
}


// Input 3d points in homogeneous co-ordinates 4xN matrix.
void PinholeCamera::perspectiveProject3DPoints( cv::Mat& _3dpts,
                                  cv::Mat& out_pts )
{

    // DIY - Do It Yourself Projection
    MatrixXf c_X;
    cv::cv2eigen( _3dpts, c_X ); //4xN
    // c_X.row(0).array() /= c_X.row(3).array();
    // c_X.row(1).array() /= c_X.row(3).array();
    // c_X.row(2).array() /= c_X.row(3).array();
    // c_X.row(3).array() /= c_X.row(3).array();

    Matrix3f cam_intrin;
    cv::cv2eigen( m_K, cam_intrin );

    Vector4f cam_dist;
    cv::cv2eigen( m_D, cam_dist );


    // K [ I | 0 ]
    MatrixXf I_0;
    I_0 = Matrix4f::Identity().topLeftCorner<3,4>();
    // MatrixXf P1;
    // P1 = cam_intrin * I_0; //3x4

    // Project and Perspective Divide
    MatrixXf im_pts;
    im_pts = I_0 * c_X; //in normalized image co-ordinate. Beware that distortion need to be applied in normalized co-ordinates
    im_pts.row(0).array() /= im_pts.row(2).array();
    im_pts.row(1).array() /= im_pts.row(2).array();
    im_pts.row(2).array() /= im_pts.row(2).array();

    // Apply Distortion
    MatrixXf Xdd = MatrixXf( im_pts.rows(), im_pts.cols() );
    for( int i=0 ; i<im_pts.cols() ; i++)
    {
      float r2 = im_pts(0,i)*im_pts(0,i) + im_pts(1,i)*im_pts(1,i);
      float c = 1.0f + (float)k1()*r2 + (float)k2()*r2*r2;
      Xdd(0,i) = im_pts(0,i) * c + 2.0f*(float)p1()*im_pts(0,i)*im_pts(1,i) + (float)p2()*(r2 + 2.0*im_pts(0,i)*im_pts(0,i));
      Xdd(1,i) = im_pts(1,i) * c + 2.0f*(float)p2()*im_pts(0,i)*im_pts(1,i) + (float)p1()*(r2 + 2.0*im_pts(1,i)*im_pts(1,i));
      Xdd(2,i) = 1.0f;
    }

    MatrixXf out = cam_intrin * Xdd;


    // cv::eigen2cv( im_pts, out_pts );
    cv::eigen2cv( out, out_pts );

}



void PinholeCamera::undistortPointSet( const cv::Mat& pts_observed_image_space, cv::Mat& pts_undistorted_image_space )
{
  // cout << "Not Implemented undistort_points\n";

  cv::Mat _in, _out;
  _1channel_to_2channel( pts_observed_image_space, _in );

  // this is because, undistort function takes in a 1xN 2channel input
  //call opencv's undistortPoints()
  cv::undistortPoints( _in, _out, m_K,  m_D, cv::Mat::eye(3,3, CV_32F), m_K );
  // If you do not set m_K the returned points will be in normalized co-ordinate system.


  _2channel_to_1channel( _out, pts_undistorted_image_space );

}


void PinholeCamera::getUndistortedNormalizedCords( const cv::Mat& pts_observed_image_space, cv::Mat& pts_undist_normed )
{
  // cout << "Not Implemented undistort_points\n";

  cv::Mat _in, _out;
  _1channel_to_2channel( pts_observed_image_space, _in );

  // this is because, undistort function takes in a 1xN 2channel input
  //call opencv's undistortPoints()
  cv::undistortPoints( _in, _out, m_K,  m_D, cv::Mat::eye(3,3, CV_32F) );
  // If you do not set m_K the returned points will be in normalized co-ordinate system.


  _2channel_to_1channel( _out, pts_undist_normed );


}


#define print_msg(x) //cout << x << endl;
#define print_mat_info( msg, x ) //print_cvmat_info( msg, x );
void PinholeCamera::jointUndistortPointSets( const cv::Mat& F,
              const cv::Mat& curr_pts_in_observed_image_space, const cv::Mat& currm_pts_in_observed_image_space,
             cv::Mat& curr_pts_undistorted, cv::Mat& currm_pts_undistorted )
{
  // camA : [I|0] ; camB : [R|t]
  // Fundamental Matrix F := [t]_x  R

  // Points in Normalized Image Cordinates
  print_msg( "getUndistortedNormalizedCords()" );
  cv::Mat undist_normed_curr, undist_normed_curr_m; // Should be 2xN
  getUndistortedNormalizedCords( curr_pts_in_observed_image_space,  undist_normed_curr );
  getUndistortedNormalizedCords( currm_pts_in_observed_image_space,  undist_normed_curr_m );
  print_mat_info( "undist_normed_curr", undist_normed_curr);
  print_mat_info( "undist_normed_curr_m", undist_normed_curr_m);


  // Allocate output matrix
  int N=undist_normed_curr.cols;
  curr_pts_undistorted = cv::Mat( 2, N, CV_32FC1 );
  currm_pts_undistorted = cv::Mat( 2, N, CV_32FC1 );


  // Independently Optimize Each Keypoint now
  Matrix3d _F;
  cv::cv2eigen(F, _F);
  cout << "F_opencv:\n" << F << endl;
  cout << "F_eigen :\n" << _F << endl;
  for( int i=0 ; i<N ; i++ )
  {
    Vector3d xl, xr;
    xl << (double) undist_normed_curr.at<float>(0,i), (double) undist_normed_curr.at<float>(1,i), 1.0;
    xr << (double) undist_normed_curr_m.at<float>(0,i), (double) undist_normed_curr_m.at<float>(1,i), 1.0;

    cout << "Iniital Epipolar Residue" <<  xl.transpose() * _F * xr << endl;

    Matrix<double,5,1> S, S_new; //state [u,v,u',v',lambda]
    S << xl(0), xl(1) , xr(0) , xr(1), 0.1 ;

    Matrix<double,5,1> GRAD;
    Matrix<double,5,5> HESS;
    grad_and_hessian( S, _F, xl, xr, GRAD, HESS );

    // Just 1 iteration is enough, since the cost function is quadratic
    S_new = S - HESS.inverse() * GRAD;

    Vector3d xl_opt, xr_opt;
    xl_opt << S_new(0), S_new(1), 1.0;
    xr_opt << S_new(2), S_new(3), 1.0;


    cout << "Finall Epipolar Residue" <<  xl_opt.transpose() * _F * xr_opt << endl;
    curr_pts_undistorted.at<float>(0,i) = (float)  (S_new(0)*fx() + cx());
    curr_pts_undistorted.at<float>(1,i) = (float)  (S_new(1)*fy() + cy());
    currm_pts_undistorted.at<float>(0,i) = (float) (S_new(2)*fx() + cx());
    currm_pts_undistorted.at<float>(1,i) = (float) (S_new(3)*fy() + cy());
  }

}

void PinholeCamera::print_cvmat_info( string msg, const cv::Mat& A )
{
  cout << msg << ":" << "rows=" << A.rows << ", cols=" << A.cols << ", ch=" << A.channels() << ", type=" << type2str( A.type() ) << endl;
}

string PinholeCamera::type2str(int type) {
  string r;

  uchar depth = type & CV_MAT_DEPTH_MASK;
  uchar chans = 1 + (type >> CV_CN_SHIFT);

  switch ( depth ) {
    case CV_8U:  r = "8U"; break;
    case CV_8S:  r = "8S"; break;
    case CV_16U: r = "16U"; break;
    case CV_16S: r = "16S"; break;
    case CV_32S: r = "32S"; break;
    case CV_32F: r = "32F"; break;
    case CV_64F: r = "64F"; break;
    default:     r = "User"; break;
  }

  r += "C";
  r += (chans+'0');

  return r;
}


double PinholeCamera::verify_epipolar_constraint( const cv::Mat& x, const cv::Mat& F, const cv::Mat& xd )
{
  int N = x.cols;
  int ch = x.channels();
  double sum = 0.0;
  for( int i=0 ; i<N ; i++ )
  {
    cv::Mat _x = cv::Mat( 3, 1, CV_64FC1 );
    cv::Mat _xd = cv::Mat( 3, 1, CV_64FC1 );

    if( ch == 2 )
    {
      _x.at<double>(0,0) = (double) x.at<cv::Vec2f>(0,i)[0];
      _x.at<double>(1,0) = (double) x.at<cv::Vec2f>(0,i)[1];
      _x.at<double>(2,0) = 1.0;

      _xd.at<double>(0,0) = (double) xd.at<cv::Vec2f>(0,i)[0];
      _xd.at<double>(1,0) = (double) xd.at<cv::Vec2f>(0,i)[1];
      _xd.at<double>(2,0) = 1.0;
    }
    else
    {
      _x.at<double>(0,0) = (double) x.at<float>(0,i);
      _x.at<double>(1,0) = (double) x.at<float>(1,i);
      _x.at<double>(2,0) = 1.0;

      _xd.at<double>(0,0) = (double) xd.at<float>(0,i);
      _xd.at<double>(1,0) = (double) xd.at<float>(1,i);
      _xd.at<double>(2,0) = 1.0;
    }

    cv::Mat _x_t;
    cv::transpose( _x, _x_t );

    cv::Mat outcome;
    outcome = (_x_t * F) * _xd;

    // cout << "_x_t" << _x_t << endl;
    // cout << "F" << F << endl;
    // cout << "xd" << _xd;
    // cout << i << "x^T F x' = "<< outcome.at<double>(0,0) << endl;
    sum += outcome.at<double>(0,0);

  }
  return sum/N;
}



// Computes Gradient and Hessian of 5d state. Used in the optimal triangulation method
void PinholeCamera::grad_and_hessian( const Matrix<double,5,1>& S,
                       const Matrix3d& F, const Vector3d& xl, const Vector3d& xr,
                       Matrix<double,5,1>& GRAD, Matrix<double,5,5>& HESS
                      )
{
    double u = S(0);
    double v = S(1);
    double ud = S(2);
    double vd = S(3);
    double lam = S(4);

    double f1=F(0,0);
    double f2=F(0,1);
    double f3=F(0,2);
    double f4=F(1,0);
    double f5=F(1,1);
    double f6=F(1,2);
    double f7=F(2,0);
    double f8=F(2,1);
    double f9=F(2,2);

    double x = xl(0);
    double y = xl(1);
    double xd = xr(0);
    double yd = xr(1);

    GRAD = Matrix<double,5,1>::Zero(); //np.zeros( 5 )
    GRAD(0) = 2*(u-x) + lam*(f1*ud + f2*vd + f3);
    GRAD(1) = 2*(v-y) + lam*(f4*ud + f5*vd + f6);
    GRAD(2) = 2*(ud-xd) + lam*(f1*u + f4*v + f7);
    GRAD(3) = 2*(ud-xd) + lam*(f2*u + f5*v + f8);
    GRAD(4) = u*(f1*ud+f2*vd+f3) + v*(f4*ud+f5*vd+f6) + (f7*ud+f8*vd+f9);

    HESS = Matrix<double,5,5>::Zero(); //np.zeros( (5,5) )
    HESS(0,0) = 2;
    HESS(0,1) = 0;
    HESS(0,2) = lam*f1;
    HESS(0,3) = lam*f2;
    HESS(0,4) = f1*ud+f2*vd+f3;

    HESS(1,0) = HESS(0,1);
    HESS(1,1) = 2;
    HESS(1,2) = lam*f4;
    HESS(1,3) = lam*f5;
    HESS(1,4) = f4*ud+f5*vd+f6;

    HESS(2,0) = HESS(0,2);
    HESS(2,1) = HESS(1,2);
    HESS(2,2) = 2;
    HESS(2,3) = 0;
    HESS(2,4) = f1*u+f4*v+f7;

    HESS(3,0) = HESS(0,3);
    HESS(3,1) = HESS(1,3);
    HESS(3,2) = HESS(2,3);
    HESS(3,3) = 2;
    HESS(3,4) = f2*u+f5*v+f8;

    HESS(4,0) = HESS(0,4);
    HESS(4,1) = HESS(1,4);
    HESS(4,2) = HESS(2,4);
    HESS(4,3) = HESS(3,4);
    HESS(4,4) = 0;

}
