#include "MeshObject.h"

MeshObject::MeshObject()
{
  m_loaded = false;
  m_world_pose_available = false;
  obj_name = string( "N/A" );
}

MeshObject::MeshObject(const string obj_name, double scaling )
{
  // when the vertices are read, a scaling is applied on the co-ordinates

  cout << "Constructor MeshObject\n";
  m_loaded = false;
  m_world_pose_available = false;
  this->obj_name = string(obj_name);

  string path = ros::package::getPath("ar_demo") + "/resources/";
  string obj_file_nme = path + obj_name;

  // cout << "Resource Path : " << path << endl;
  cout << "Open File     : " << obj_file_nme << endl;
  cout << "Name          : " << obj_name; 

  // Loading mesh here
  load_obj( obj_file_nme , scaling);


  m_world_pose_available = false;
  m_loaded = true;

}


void MeshObject::setObjectWorldPose( Matrix4d w_T_ob )
{
  cout << "[MeshObject::setObjectWorldPose::" << obj_name << "]w_T_ob\n" << w_T_ob << endl;
  this->w_T_ob = Matrix4d( w_T_ob );
  m_world_pose_available = true;
}


bool MeshObject::getObjectWorldPose( Matrix4d& w_T_ob )
{
  w_T_ob = Matrix4d( this->w_T_ob );
  return m_world_pose_available;
}


bool MeshObject::load_obj( string fname, double scaling )
{

  cout << "MeshObject::load_obj()\n";

  ifstream myfile( fname.c_str() );

  if( !myfile.is_open() )
  {
    ROS_ERROR_STREAM( "Fail to open file: "<< fname );
    return false;
  }


  // line-by-line reading
  vector<string> vec_of_items;
  int nvertex=0, nfaces = 0;


  for( string line; getline( myfile, line ) ; )
  {
    // cout << "l:" << line << endl;
    split( line, ' ', vec_of_items );
    if( vec_of_items.size() <= 0 )
      continue;

    if( vec_of_items[0] == "v" )
    {
      nvertex++;
      Vector3d vv;
      vv << stod( vec_of_items[1] ), stod( vec_of_items[2] ), stod( vec_of_items[3] ) ;
      vertices.push_back( vv );
    }


    if( vec_of_items[0] == "f" )
    {
      nfaces++;
      Vector3i vv;
      vv << stoi( vec_of_items[1] ), stod( vec_of_items[2] ), stod( vec_of_items[3] ) ;
      faces.push_back( vv );
    }


  }


  cout << "Vertex: "<< nvertex << "  Faces: " << nfaces << endl;
  o_X = MatrixXd( 4, nvertex );
  for( int i=0 ; i<nvertex ; i++ )
  {
    o_X.col(i) << scaling * vertices[i], 1.0 ;
  }

  eigen_faces = MatrixXi( 3, nfaces );
  for( int i=0 ; i<nfaces ; i++ )
  {
    eigen_faces.col(i) = faces[i];
  }

  cout << "end MeshObject::load_obj()\n";
  return true;

}


void MeshObject::split(const std::string &s, char delim, vector<string>& vec_of_items)
{
    std::stringstream ss(s);
    std::string item;
    // vector<string> vec_of_items;
    vec_of_items.clear();
    while (std::getline(ss, item, delim)) {
        // *(result++) = item;
        vec_of_items.push_back( item );
    }
}


bool MeshObject::writeMeshWorldPose()
{
  string interactive_finalpose_filename = ros::package::getPath("ar_demo") + "/resources/" + obj_name + ".worldpose";
  if( !isWorldPoseAvailable() )
  {
    cout << "WorldPose not available not writing. " << interactive_finalpose_filename << endl;
    return false;
  }


  cout << "Write File " << interactive_finalpose_filename << endl;
  ofstream myfile( interactive_finalpose_filename.c_str() );

  if( !myfile.is_open() )
  {
    ROS_ERROR_STREAM( "MeshObject::writeMeshWorldPose(): Cannot write file :"<< interactive_finalpose_filename );
    return false;
  }

  myfile << "#,x,y,z;q.w,q.x,q.y,q.z\n";
  Vector3d t;
  t << w_T_ob(0,3), w_T_ob(1,3), w_T_ob(2,3);

  Quaterniond qt( w_T_ob.topLeftCorner<3,3>() );
  myfile << t.x() << "," << t.y() << "," << t.z() << "\n";
  myfile << qt.w() << "," << qt.x() << "," << qt.y() << "," << qt.z() ;//<< "\n";
  return true;

}
/*
bool MeshObject::load_debug_xml( const  string& fname )
{
  cout << "-----Open file : " << fname << "------" << endl;
  cv::FileStorage fs( fname, cv::FileStorage::READ );
  if( fs.isOpened() == false )
  {
    ROS_ERROR_STREAM( "in Node::load_debug_xml, Cannot open file " << fname );
    return false;
  }

  fs["obj_name"] >> this->obj_name;
  cout << "obj_name: " << this->obj_name << endl;

  cv::Mat o_X_mat;
  fs["o_X"] >> o_X_mat;
  if( o_X_mat.empty() ) {
    cout << "`o_X` is empty\n";
  }
  else {
    cout << "Mesh vertices loaded: " << o_X_mat.rows << ", " << o_X_mat.cols << endl;
    MatrixXd tmp_o_X;
    cv::cv2eigen( o_X_mat, tmp_o_X );
    o_X = MatrixXd( tmp_o_X );
    // TODO: Also fill the corresponding vector for redundancy, or else completely get rid of the vectors. Just keep a vertex matrix and a face matrix
    m_loaded = true;
  }

  cv::Mat faces_mat;
  fs["faces"] >> faces_mat;
  if( faces_mat.empty() ) {
    cout << "`faces` is empty\n";
    m_loaded = m_loaded && false;
  }
  else {
    cout << "Mesh faces loaded" << faces_mat.rows << ", " << faces_mat.cols << endl;
    MatrixXi tmp_faces;
    cv::cv2eigen( faces_mat, tmp_faces );
    eigen_faces = MatrixXi( tmp_faces );
    m_loaded = m_loaded && true;
  }

  cv::Mat w_T_ob_mat;
  fs["w_T_ob"] >>  w_T_ob_mat;
  if( w_T_ob_mat.empty() ) {
    cout << "`w_T_ob` is empty\n";
  }
  else {
    cout << "Mesh w_T_{ob} loaded\n";
    MatrixXd tmp_w_T_ob;
    cv::cv2eigen( w_T_ob_mat, tmp_w_T_ob );
    setObjectWorldPose( tmp_w_T_ob );
  }
  cout << "--- End loading Mesh ---\n";
  fs.release();

}

void MeshObject::write_debug_xml( const char * fname )
{
  cv::FileStorage fs( fname, cv::FileStorage::WRITE );
  if( fs.isOpened() == false )
  {
    ROS_ERROR_STREAM( "in MeshObject::write_debug_xml, Cannot open file " << fname );
    return false;
  }

  fs << "obj_name" << obj_name ;
  if( isWorldPoseAvailable() )
  {
    cv::Mat w_T_ob_mat;
    cv::eigen2cv( w_T_ob, w_T_ob_mat );
    fs << "w_T_ob" <<w_T_ob_mat;
  }

  if( isMeshLoaded() )
  {
    // Write Vertices
    cv::Mat o_X_mat;
    cv::eigen2cv( o_X, o_X_mat );
    fs << "o_X" <<o_X_mat;

    // Write Faces
    cv::Mat eigen_faces_mat;
    cv::eigen2cv( eigen_faces, eigen_faces_mat );
    fs << "faces" << eigen_faces_mat;
  }

  fs.release();

}
*/
