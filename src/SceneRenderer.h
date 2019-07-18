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



// camodocal
#include "camodocal/camera_models/Camera.h"
#include "camodocal/camera_models/CameraFactory.h"
#include "camodocal/camera_models/CataCamera.h"
#include "camodocal/camera_models/EquidistantCamera.h"


#include <Eigen/Dense>
#include <Eigen/Geometry>
using namespace Eigen;
#include <opencv2/core/eigen.hpp>


using namespace std;

#include "MeshObject.h"
// #include "PinholeCamera.h"

#include "utils/PoseManipUtils.h"
#include "utils/TermColor.h"

class SceneRenderer
{
public:
    SceneRenderer();

    // void setCamera(  PinholeCamera* cam );
    void setCamera(  camodocal::CameraPtr cam );
    bool addMesh(  MeshObject* mesh );

    // std::shared_ptr<MeshObject> getMesh( string name )
    MeshObject* getMesh( string name )
    {
        if( meshmap.count(name) == 0 ) {
            cout << "[SceneRenderer::getMesh] you requested to get mesh with name=" << name << " such a mesh does not exist\n";
            cout << "Following are available, n=" << meshmap.size() << ":\n";
            for( auto it=meshmap.begin() ; it!=meshmap.end() ; it++ )
            {
                cout << it->first;
            }
            exit(2);
        }

        // return std::make_shared( meshmap.at( name ) );
        return meshmap.at( name ) ;
    }


    // void render( const cv::Mat& canvas, const Matrix4d& w_T_c );
    void renderIn( const cv::Mat& canvas, const Matrix4d& w_T_c, cv::Mat& buf );

    void getMeshList( vector<string>& mesh_keys );

    bool setWorldPoseOfMesh( string mesh_id, const Matrix4d& w_T_obj );


private:
    mutable std::mutex mesh_mutex;
    std::map<string, MeshObject*> meshmap;

    bool isCamSet = false;
    camodocal::CameraPtr m_camera;

    // given 3d points to spaceToPlane() of camodocal's abstract camera.
    void perspectiveProject3DPoints( const MatrixXd& c_V, MatrixXd& c_v );

};
