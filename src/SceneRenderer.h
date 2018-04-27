#pragma once

/**
  This class will have a scene graph and a camera. Will implement a render()
  function. Given a canvas and pose of this canvas w_T_c, will render the meshes on it.

  Version-1: Current version does a dumb wireframe rendering DIY. No OpenGL.

  Author  : Manohar Kuse <mpkuse@connect.ust.hk>
  Created : 4th Jan, 2017
**/


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





#include <Eigen/Dense>
#include <Eigen/Geometry>
using namespace Eigen;
#include <opencv2/core/eigen.hpp>


using namespace std;

#include "MeshObject.h"
#include "PinholeCamera.h"


class SceneRenderer
{
public:
  SceneRenderer();

  void setCamera(  PinholeCamera* cam );
  void addMesh(  MeshObject* mesh );
  int getMeshCount();
  const string& getMeshName( int i );
  MeshObject* getMeshObject( int i );

  void render( const cv::Mat& canvas, const Matrix4d& w_T_c );
  void renderIn( const cv::Mat& canvas, const Matrix4d& w_T_c, cv::Mat& buf );


private:
  vector<MeshObject*> objGraph;
  PinholeCamera * cam;


};
