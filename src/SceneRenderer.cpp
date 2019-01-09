#include "SceneRenderer.h"

SceneRenderer::SceneRenderer()
{
}


void SceneRenderer::setCamera( PinholeCamera* cam )
{ this->cam = cam; }

bool SceneRenderer::addMesh( MeshObject* mesh )
{
    std::lock_guard<std::mutex> lk(mesh_mutex);
    // this->objGraph.push_back( mesh );
    if( meshmap.count(mesh->getMeshObjectName()) == 0 ) {
        meshmap[ mesh->getMeshObjectName() ] = mesh;
        return true;
    }
    else {
        cout << "[SceneRenderer::addMesh] You requyested to add a mesh which already exists. So not adding again\n";
        return false;
    }
}

/* //todo removal
void SceneRenderer::render( const cv::Mat& canvas, const Matrix4d& w_T_c )
{
  // MeshObject *m = objGraph[0];
  cv::Mat buf = canvas.clone();

  for( int ob_i=0 ; ob_i <getMeshCount() ; ob_i++ )
  {
    MeshObject *m = objGraph[ob_i];

    // Get Vertices
    // assert( m->isMeshLoaded() );
    if( !m->isMeshLoaded() ) {
      ROS_WARN_STREAM( "isMeshLoaded: Ignore mesh#" << ob_i );
      continue;
    }
    MatrixXd ob_V = m->getVertices(); //ob_V


    // Prepare c_T_ob and mesh points in camere-frame-of-ref
    // assert( m->isWorldPoseAvailable() );
    if( !m->isWorldPoseAvailable() ) {
      ROS_WARN_STREAM( "isWorldPoseAvailable: Ignore mesh#" << ob_i );
      continue;
    }
    Matrix4d w_T_ob = m->getObjectWorldPose();

    Matrix4d c_T_ob;
    c_T_ob = w_T_c.inverse() * w_T_ob; // c_T_ob


    MatrixXd c_V = c_T_ob * ob_V; // vertices in camera-frame
    // cout << "ob_V:\n" << ob_V << endl;
    // cout << "c_T_ob:\n" << c_T_ob << endl;
    // cout << "c_V:\n" << c_V << endl;

    // View Frustrum Clipping
    vector<bool> cliped_keep;
    double near_clip = 0.001;
    double top_clip  = 3.;
    double right_clip = 3.;
    for( int i=0 ; i<c_V.cols() ; i++ )
    {
      bool stati = true;
      if( c_V(2,i) < near_clip )
        stati = false;

      double cos_thetaz = c_V(2,i) / sqrt( c_V(0,i)*c_V(0,i) + c_V(1,i)*c_V(1,i) +c_V(2,i)*c_V(2,i)  );
      if( cos_thetaz > 0.66 )
        stati = true;
      else
        stati = false;

      cliped_keep.push_back( stati );
    }

    // Perspective Projection
    MatrixXd c_v;
    cam->perspectiveProject3DPoints( c_V, c_v );
    // cout << "c_v(Projected):\n" << c_v << endl;


    // plot these points on canvas
    int u_ob_i = ob_i * 15705 + 25740;
    uchar _blue = u_ob_i & 255;
    uchar _green = (u_ob_i>>8) & 255;
    uchar _red = (u_ob_i>>16) & 255;

    cv::Scalar color = cv::Scalar(_blue,_green,_red);
    // Plot points
    for( int i=0 ; i<c_v.cols() ; i++ ) //loop on all the points
    {
      // cout << i << ": "<< c_v(0,i) << ", " << c_v(1,i) << endl;
      if( !cliped_keep[i] ) //only plot point in front of camera
        continue;

      cv::circle( buf, cv::Point(c_v(0,i), c_v(1,i)), 2, color, 1, 8 );
    }

    // Plot triangles
    MatrixXi faces = m->getFaces();
    // cout << "nFaces : "<< faces.rows() << "x" << faces.cols() << endl;
    for( int f=0 ; f<faces.cols() ; f++ )
    {
      // Vector3i face = faces[f];
      int f1 = faces(0,f)-1; //face-indexing starts with 1 in .obj
      int f2 = faces(1,f)-1;
      int f3 = faces(2,f)-1;
      // cout << f << ":f  " << f1 << ", " << f2 << ", " << f3 << endl;
      cv::Point p1 = cv::Point( c_v(0,f1), c_v(1,f1) );
      cv::Point p2 = cv::Point( c_v(0,f2), c_v(1,f2) );
      cv::Point p3 = cv::Point( c_v(0,f3), c_v(1,f3) );

      if( cliped_keep[f1]==false || cliped_keep[f2]==false || cliped_keep[f3]==false    ) //only plot point in front of camera
        continue;


      cv::line( buf, p1, p2, color, 1, 8 );
      cv::line( buf, p1, p3, color, 1, 8 );
      cv::line( buf, p3, p2, color, 1, 8 );
    }
  }

  cv::imshow( "buf", buf);

}
*/

void SceneRenderer::renderIn( const cv::Mat& canvas, const Matrix4d& w_T_c, cv::Mat& buf )
{
    if( canvas.channels() == 3 )
      buf = canvas.clone();
    else
        cv::cvtColor(canvas, buf, cv::COLOR_GRAY2BGR);


  vector<string> mesh_keys;
  getMeshList( mesh_keys );
  for( int ob_i=0 ; ob_i < mesh_keys.size() ; ob_i++ )
  {
    MeshObject *m = meshmap[  mesh_keys[ob_i]  ];

    // Get Vertices
    // assert( m->isMeshLoaded() );
    if( !m->isMeshLoaded() ) {
      ROS_WARN_STREAM( "isMeshLoaded: Ignore mesh#" << ob_i );
      continue;
    }
    MatrixXd ob_V = m->getVertices(); //ob_V


    // Prepare c_T_ob and mesh points in camere-frame-of-ref
    // assert( m->isWorldPoseAvailable() );
    if( !m->isWorldPoseAvailable() ) {
      ROS_WARN_STREAM( "isWorldPoseAvailable: Ignore mesh#" << ob_i );
      continue;
    }
    Matrix4d w_T_ob = m->getObjectWorldPose();

    Matrix4d c_T_ob;
    c_T_ob = w_T_c.inverse() * w_T_ob; // c_T_ob


    MatrixXd c_V = c_T_ob * ob_V; // vertices in camera-frame
    // cout << "ob_V:\n" << ob_V << endl;
    // cout << "c_T_ob:\n" << c_T_ob << endl;
    // cout << "c_V:\n" << c_V << endl;

    // View Frustrum Clipping
    vector<bool> cliped_keep;
    double near_clip = 0.001;
    double top_clip  = 5.;
    double right_clip = 5.;
    for( int i=0 ; i<c_V.cols() ; i++ )
    {
      bool stati = true;
      if( c_V(2,i) < near_clip )
        stati = false;

      double cos_thetaz = c_V(2,i) / sqrt( c_V(0,i)*c_V(0,i) + c_V(1,i)*c_V(1,i) +c_V(2,i)*c_V(2,i)  );
      if( cos_thetaz > 0.66 )
        stati = true;
      else
        stati = false;

      cliped_keep.push_back( stati );
    }

    // Perspective Projection
    MatrixXd c_v;
    cam->perspectiveProject3DPoints( c_V, c_v );
    // cout << "c_v(Projected):\n" << c_v << endl;


    // plot these points on canvas
    int u_ob_i = ob_i * 15705 + 25740;
    uchar _blue = 0;u_ob_i & 255;
    uchar _green = 0;(u_ob_i>>8) & 255;
    uchar _red = 255;(u_ob_i>>16) & 255;

    cv::Scalar color = cv::Scalar(_blue,_green,_red);
    // Plot points
    for( int i=0 ; i<c_v.cols() ; i++ ) //loop on all the points
    {
      // cout << i << ": "<< c_v(0,i) << ", " << c_v(1,i) << endl;
      if( !cliped_keep[i] ) //only plot point in front of camera
        continue;

      cv::circle( buf, cv::Point(c_v(0,i), c_v(1,i)), 2, color, 1, 8 );
    }

    // Plot triangles
    MatrixXi faces = m->getFaces();
    // cout << "nFaces : "<< faces.rows() << "x" << faces.cols() << endl;
    for( int f=0 ; f<faces.cols() ; f++ )
    {
      // Vector3i face = faces[f];
      int f1 = faces(0,f)-1; //face-indexing starts with 1 in .obj
      int f2 = faces(1,f)-1;
      int f3 = faces(2,f)-1;
      // cout << f << ":f  " << f1 << ", " << f2 << ", " << f3 << endl;
      cv::Point p1 = cv::Point( c_v(0,f1), c_v(1,f1) );
      cv::Point p2 = cv::Point( c_v(0,f2), c_v(1,f2) );
      cv::Point p3 = cv::Point( c_v(0,f3), c_v(1,f3) );

      if( cliped_keep[f1]==false || cliped_keep[f2]==false || cliped_keep[f3]==false    ) //only plot point in front of camera
        continue;


      cv::line( buf, p1, p2, color, 2, cv::LINE_AA );
      cv::line( buf, p1, p3, color, 2, cv::LINE_AA );
      cv::line( buf, p3, p2, color, 2, cv::LINE_AA );
    }
  }

  // cv::imshow( "buf", buf);

}


// TODO removal
/*
const int SceneRenderer::getMeshCount() const
{
    std::lock_guard<std::mutex> lk(mesh_mutex);
    return meshmap.size();
}


const string SceneRenderer::getMeshName( int i ) const
{
    std::lock_guard<std::mutex> lk(mesh_mutex);
    assert( i>=0 && i<getMeshCount() );
    return objGraph[i]->getMeshObjectName();
}

MeshObject* SceneRenderer::getMeshObject( int i )
{
    std::lock_guard<std::mutex> lk(mesh_mutex);
    assert( i>=0 && i<getMeshCount() );
    return objGraph[i];
}

MeshObject * SceneRenderer::getMeshObject( string obj_name )
{
    std::lock_guard<std::mutex> lk(mesh_mutex);
    if( meshmap.count(obj_name) > 0 )
        return meshmap[obj_name];
    else
        return NULL;
}
*/

void SceneRenderer::getMeshList( vector<string>& mesh_keys )
{
    std::lock_guard<std::mutex> lk(mesh_mutex);
    mesh_keys.clear();
    for( auto it=meshmap.begin() ; it!=meshmap.end() ; it++ )
        mesh_keys.push_back( it->first );
}


bool SceneRenderer::setWorldPoseOfMesh( string mesh_id, const Matrix4d& w_T_obj )
{
    std::lock_guard<std::mutex> lk(mesh_mutex);
    if( meshmap.count(mesh_id) > 0 )
    {
        meshmap[mesh_id]->setObjectWorldPose( w_T_obj );
        cout << "[SceneRenderer::setWorldPoseOfMesh] successfully set for mesh_id=" << mesh_id << " pose=" << PoseManipUtils::prettyprintMatrix4d( w_T_obj ) << endl;
        return true;
    }
    else {
        cout << TermColor::RED() << "[SceneRenderer::setWorldPoseOfMesh] cannot find mesh=" << mesh_id << "in my list of "<< meshmap.size() << " mesh objects\n" << TermColor::RESET() ;
        return false;
    }
}
