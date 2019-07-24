#include "ARDataManager.h"




std::map<ros::Time,ARDataNode*>::iterator
ARDataManager::findClosestKey( const ros::Time& key)
{
    return data_map.find(key);
}

std::map<ros::Time,ARDataNode*>::iterator
ARDataManager::findClosestKeyApprox( const ros::Time& key)
{
    for( auto it=data_map.begin() ; it!=data_map.end() ; it++ )
    {
        if( abs( int64_t(it->first.toNSec()) - int64_t(key.toNSec())) < (int64_t) 1000000L )
            return it;
    }

    return data_map.end();

}

//=============================================================================//
//--------------------------- Image Raw ---------------------------------------//

// this will only print timestamp when callback comes.
// #define __ARDataManager__callback( msg ) msg;
#define __ARDataManager__callback( msg ) ;



// #define __ARDataManager__raw_image_callback( msg ) msg;
#define __ARDataManager__raw_image_callback( msg ) ;

void ARDataManager::raw_image_callback( const sensor_msgs::ImageConstPtr msg )
{
    std::lock_guard<std::mutex> lk(data_map_mutex);

    __ARDataManager__raw_image_callback(
        std::cout << "[raw_image_callback] msg.t=" << msg->header.stamp << "\t";
    )
    __ARDataManager__callback(
        std::cout << "[raw_image_callback] msg.t=" << msg->header.stamp << "\n";
    )
    ARDataNode * node = NULL;

    if( data_map.size() > 0  ) {
        if( data_map.rbegin()->first > msg->header.stamp ) {
            cout << "[ARDataManager::raw_image_callback] Image seem to be coming out of order. This is not normal. I am expecting the images to come sequentially in time. ideally this should not be happening, perhaps something wrong with the bag or timestaamps on the images\n";
            exit(1);
        }
    }

    auto it = findClosestKey( msg->header.stamp );
    if( it==data_map.end() ) {//not found
        __ARDataManager__raw_image_callback( cout << "--Not Found so allocate new--"; )
        node = new ARDataNode(); //so allocate new
        data_map[ msg->header.stamp ] = node;
    } else{
        __ARDataManager__raw_image_callback( cout << "--Found--"; )
         node = it->second;
     }

    __ARDataManager__raw_image_callback(
        cout << "setImageFromMsg\n";
    )
    node->setImageFromMsg( msg );
}

//=============================================================================//
//--------------------- Pose Info from vins_estimator -------------------------//
// #define __ARDataManager__odom_pose_callback(msg) msg;
#define __ARDataManager__odom_pose_callback(msg) ;
void ARDataManager::odom_pose_callback( const nav_msgs::Odometry::ConstPtr msg ) ///< w_T_c. pose of camera in the world-cordinate system. All the cameras. only a subset of this will be keyframes
{
    std::lock_guard<std::mutex> lk(data_map_mutex);

    __ARDataManager__odom_pose_callback(
    std::cout << "[odom_pose_callback] t=" << msg->header.stamp << "\t";
    )
    __ARDataManager__callback( std::cout << "[odom_pose_callback] t=" << msg->header.stamp << "\n";)
    ARDataNode * node = NULL;


    auto it = findClosestKeyApprox( msg->header.stamp );
    if( it==data_map.end() ) {//not found
        __ARDataManager__odom_pose_callback(
        cout << TermColor::RED() << "not found\t" << TermColor::RESET(); )

    } else{
        __ARDataManager__odom_pose_callback(
        cout << TermColor::GREEN() << "found at it_=" << it->first << "\t" << TermColor::RESET();)
        node = it->second;
    }

    __ARDataManager__odom_pose_callback(
    cout << "setPoseFromMsg\n";
    )
    node->setPoseFromMsg( msg );
}



//=============================================================================//
//-------------------- Pose From Pose Graph Solver ----------------------------//
// #define __ARDataManager__detailed_path_callback__( msg ) msg;
#define __ARDataManager__detailed_path_callback__( msg ) ;
void ARDataManager::detailed_path_callback( const nav_msgs::Path::ConstPtr msg )
{

    std::lock_guard<std::mutex> lk(data_map_mutex);

    __ARDataManager__detailed_path_callback__(
    cout << TermColor::iGREEN() << "[detailed_path_callback] len=" << msg->poses.size() << endl << TermColor::RESET();
    )

    __ARDataManager__detailed_path_callback__(
    cout << "items in data_map:\n";
    for( auto it=data_map.begin() ; it!=data_map.end() ; it++ )
    {
        cout << it->first << "\t";
    }
    cout << endl;
    )

    for( auto it=msg->poses.begin() ; it!=msg->poses.end() ; it++ ) //loop over all poses in path
    {
        __ARDataManager__detailed_path_callback__(
        // --- print
        cout << "i=" << it-msg->poses.begin() << "\t";
        cout << "t=" << it->header.stamp << "\t";
        // cout << "frame_id=" << it->header.frame_id << "\t";
        )

        // --- basic info
        int this__i = it-msg->poses.begin();
        ros::Time this__t = it->header.stamp;
        string this__frameid = it->header.frame_id;

        vector<string> after_split = MiscUtils::split(this__frameid, ':' );
        if( after_split.size() != 4 ) {
            cout << "[ARDataManager::detailed_path_callback]ERROR: I am expecting frame_id to have 4 fields separated by `:`. worldID:<num>:setID_of_worldID:<num>\n";
            exit(1);
        }
        int this__worldID = std::stoi( after_split[1] );
        int this__setID_of_worldID = std::stoi( after_split[3] );
        __ARDataManager__detailed_path_callback__(
        cout << "worldID=" << this__worldID << "\t";
        cout << "setID_of_worldID=" << this__setID_of_worldID << "\t";)


        // --- ws_T_cam
        #if 1
        // I assume, the poses in the path message are in reference of imu.
        Matrix4d ws_T_imu = Matrix4d::Identity();
        PoseManipUtils::geometry_msgs_Pose_to_eigenmat( it->pose, ws_T_imu );
        assert( is_imu_T_cam_available() );
        Matrix4d ws_T_cam = ws_T_imu * this->get_imu_T_cam();
        #else
        // I assume the poses in path message in reference of camera
        Matrix4d ws_T_cam = Matrix4d::Identity();
        PoseManipUtils::geometry_msgs_Pose_to_eigenmat( it->pose, ws_T_cam );
        #endif

        __ARDataManager__detailed_path_callback__(
        cout << "ws_T_cam: " << PoseManipUtils::prettyprintMatrix4d( ws_T_cam );)


        //--- lookup
        // auto it_f = data_map.find( this__t );
        auto it_f = findClosestKeyApprox( this__t );
        if( it_f != data_map.end() ) {
            __ARDataManager__detailed_path_callback__(
            cout << TermColor::GREEN() << "Found it_f=" << it_f->first << "\t" << TermColor::RESET(); )
            it_f->second->setOptCamPose( this__t,  ws_T_cam, this__worldID, this__setID_of_worldID );
        }
        else {
            __ARDataManager__detailed_path_callback__(
            cout << TermColor::RED() << "NOT Found\t" << TermColor::RESET();
            )
        }


        __ARDataManager__detailed_path_callback__(
        cout << endl;)
    }
}

void ARDataManager::hz200_imu_callback( const geometry_msgs::PoseStamped::ConstPtr msg )
{
    __ARDataManager__callback(
    cout << "hz200_imu_callback" << msg->header.stamp << "\n";)

    std::lock_guard<std::mutex> lk(imuprop_msgs_mutex);
    imuprop_msgs.push( *msg );
}

//=============================================================================//
//------------------------ Poses of Virtual Objects ---------------------------//
#define __ARDataManager__meshposecallback( msg ) msg;
// #define __ARDataManager__meshposecallback( msg ) ;
/// This will update the mesh-pose upon receiving the message from `interactive_marker_server`
void ARDataManager::mesh_pose_callback( const geometry_msgs::PoseStamped::ConstPtr msg )
{
    __ARDataManager__meshposecallback(
    cout << TermColor::BLUE();
    cout << "+        XXXXX mesh_pose_callback() for mesh "<< msg->header.frame_id << endl ;
    )


    string frame_id = msg->header.frame_id ;
    if( frame_id == "control_marker" )
    {
        __ARDataManager__meshposecallback( cout << TermColor::RESET(); )
        return;
    }

    // msg->pose --> pose_p, pose_q
    Matrix4d w_T_ob;
    PoseManipUtils::geometry_msgs_Pose_to_eigenmat( msg->pose, w_T_ob );
    __ARDataManager__meshposecallback(
    cout << "Recvd Pose (w_T_{" << frame_id << "}):" << PoseManipUtils::prettyprintMatrix4d(w_T_ob) << endl;
    )


    bool status = renderer->setWorldPoseOfMesh( frame_id, w_T_ob );
    if( status == false ) {
        cout << TermColor::RED() << "[ARDataManager::mesh_pose_callback] received a pose for mesh which I do not recognize\n" << TermColor::RESET();
    }

    __ARDataManager__meshposecallback( cout << TermColor::RESET(); )

}

// #define __ARDataManager__surfelmap_callback( msg ) msg;
#define __ARDataManager__surfelmap_callback( msg ) ;
void ARDataManager::surfelmap_callback(const sensor_msgs::PointCloud2::ConstPtr pointcloud_map)
{

    if( m_ground_plane_estimated )
    {
        // cout << TermColor::iYELLOW() << "[ARDataManager::surfelmap_callback] ignore, since ground plane was already estimated" << TermColor::RESET() << endl;
        return;
    }

    __ARDataManager__surfelmap_callback(
    cout << TermColor::iYELLOW() << "[ARDataManager::surfelmap_callback]" << TermColor::RESET() << endl;
    cout << "\theight=" << pointcloud_map->height << "\twidth=" << pointcloud_map->width << endl;
    )

    // sensor_msgs::PointCloud2 --> pcl::PointCloud<pcl::PointXYZ>::Ptr
    pcl::PointCloud<pcl::PointXYZ>::Ptr raw_cloud (new pcl::PointCloud<pcl::PointXYZ>);
    pcl::fromROSMsg(*pointcloud_map, *raw_cloud);
	ros::Time map_time = pointcloud_map->header.stamp;


    // if #pts above say 5000 try to estimate ground plane.
    if( raw_cloud->points.size() > 20000 )
    {

            // Filtering
            ElapsedTime _eta_;
            _eta_.tic();
            pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered;
            if(raw_cloud->points.size() == 0) return;
        	cloud_filtered.reset( new pcl::PointCloud<pcl::PointXYZ>);
            cloud_filtered->points.clear();
            pcl::StatisticalOutlierRemoval<pcl::PointXYZ> sor;
            sor.setInputCloud (raw_cloud);
            sor.setMeanK (30);
            sor.setStddevMulThresh (1.0);
            sor.filter (*cloud_filtered);
            __ARDataManager__surfelmap_callback(
            cout << "\tPointCloud Filtering Elapsed (ms): " << _eta_.toc_milli() << endl;
            cout << "\tcloud_filtered->points.size()=" << cloud_filtered->points.size() << endl;
            )



        #if 0
        cout << "Saving point cloud\n";
        pcl::io::savePCDFile("/tmp/estimate.pcd", *cloud_filtered);
        cout << "Saving point cloud done\n";
        #endif

        // Estimate Ground Plane
        _eta_.tic();
        vector<int> int_inliers;
        VectorXf coeff = EstimationFromPointClouds::estimate_ground_plane_from_pointcloud( cloud_filtered, int_inliers, 0.13, true );
        assert( coeff.rows() == 4 );
        __ARDataManager__surfelmap_callback(
        cout << "[ARDataManager::surfelmap_callback] coeff of ground plane: " << coeff.transpose() << endl;
        cotut << "Ground PLane Estimation took (ms): " << _eta_.toc_milli() << endl;
        )


        // Estimate Plane Orientation
        Matrix3d wRp = EstimationFromPointClouds::estimate_plane_orientation_wRp( coeff.cast<double>() );
        __ARDataManager__surfelmap_callback(
        cout << "[ARDataManager::surfelmap_callback]wRp:\n" << wRp << endl; )

        Matrix4d wTp = Matrix4d::Identity();
        wTp.topLeftCorner(3,3) = wRp;

        {
            std::lock_guard<std::mutex> lk(data_map_mutex);

            // Where is the current camera
            const ARDataNode* node =  latestNodeWhereOptPoseisAvailable();


            // get the chair's random point here:
            assert( node != NULL );
            Matrix4d w0_T_node = node->getOptCamPose();
            cout << "The current camera is at: " << PoseManipUtils::prettyprintMatrix4d(w0_T_node);
            double w0_x_node = w0_T_node(0,3);
            double w0_y_node = w0_T_node(1,3);
            double w0_z_node = w0_T_node(2,3);
            wTp.col(3).topRows(3) = EstimationFromPointClouds::random_pt_on_plane( coeff.cast<double>(),
                                w0_x_node+3, w0_x_node+5,
                                w0_y_node-0.5, w0_y_node+0.5 );

        }
        // wTp.col(3).topRows(3) = EstimationFromPointClouds::random_pt_on_plane( coeff.cast<double>(), 4, 8, -4., 0. );
        __ARDataManager__surfelmap_callback(
        cout << "[ARDataManager::surfelmap_callback]wTp:\n" << wTp << endl;)


        // note this down in class variables.
        this->groundplane_coeff = coeff.cast<double>();
        this->groundplane_wTp = wTp;
        m_ground_plane_estimated = true;


        // publish ground plane markers
        if( isPubMarkerset )
        {
            #if 0
            visualization_msgs::Marker gplane;
            RosMarkerUtils::init_plane_marker(  gplane, 20, 10, 0.82, 0.7, 0.54, 0.6 );
            gplane.ns = "ground_plane";
            gplane.id = 0;
            RosMarkerUtils::setpose_to_marker( this->groundplane_wTp, gplane );
            pub_marker.publish( gplane );
            #endif

            visualization_msgs::Marker axis_marker;
            RosMarkerUtils::init_XYZ_axis_marker( axis_marker );
            axis_marker.ns = "on_plane_axis";
            axis_marker.id = 0;
            RosMarkerUtils::setpose_to_marker( this->groundplane_wTp, axis_marker );
            pub_marker.publish( axis_marker );



            // mesh_pose_callback(  );
            {
                string mesh__id = "chair.obj";
                bool status = renderer->setWorldPoseOfMesh( mesh__id, this->groundplane_wTp );
                if( status == false ) {
                    cout << "[ARDataManager::surfelmap_callback]renderer->setWorldPoseOfMesh failed\n";
                    exit(1);
                }

                // publish mesh with wTp
                visualization_msgs::Marker mesh_marker;
                RosMarkerUtils::init_mesh_marker( mesh_marker );
                mesh_marker.ns = "mesh_"+mesh__id;
                mesh_marker.id = 0;
                auto mesh_obj = renderer->getMesh( mesh__id );
                RosMarkerUtils::setscaling_to_marker( mesh_obj->getScalingFactor(), mesh_marker );
                RosMarkerUtils::setpose_to_marker( this->groundplane_wTp, mesh_marker );
                mesh_marker.mesh_resource = "package://ar_demo/resources/"+mesh_obj->getMeshObjectName();
                pub_marker.publish( mesh_marker );

            }



            {
                string mesh__id = "ground_plane.obj";
                bool status = renderer->setWorldPoseOfMesh( mesh__id, this->groundplane_wTp );
                if( status == false ) {
                    cout << "[ARDataManager::surfelmap_callback]renderer->setWorldPoseOfMesh failed, mesh__id=" << mesh__id << "\n";
                    exit(1);
                }


                // publish mesh with wTp
                visualization_msgs::Marker mesh_marker;
                RosMarkerUtils::init_mesh_marker( mesh_marker );
                mesh_marker.ns = "mesh_"+mesh__id;
                mesh_marker.id = 0;
                auto mesh_obj = renderer->getMesh( mesh__id );
                RosMarkerUtils::setscaling_to_marker( mesh_obj->getScalingFactor(), mesh_marker );
                RosMarkerUtils::setpose_to_marker( this->groundplane_wTp, mesh_marker );
                mesh_marker.mesh_resource = "package://ar_demo/resources/"+mesh_obj->getMeshObjectName();
                pub_marker.publish( mesh_marker );

            }


        }

        cout << TermColor::iRED() << "system cmd=pkill surfel_fusion \n"<< TermColor::RESET();
        system( "pkill surfel_fusion");

    } //if( npts > 20000)

    // if #pts above say 5000 try to estimate box.

}



///////////////////////////////////////////////////////////////////////////////
void ARDataManager::monitor_thread( int hz, bool printing )
{
    cout << TermColor::GREEN() << "starting thread `monitor_thread`. will have an inf-loop @" << hz << "\n" << TermColor::RESET() << endl;
    ros::Rate rate(hz);

    ofstream myfile;
    if( printing ) {
        myfile.open("/dev/pts/3");
        if (! myfile.is_open() )
        {
            cout << "[]Cannt open ffile\n";
            exit(2);
        }
    }


    while( monitor_thread_flag )
    {
        {
            std::lock_guard<std::mutex> lk(data_map_mutex);

            if( printing )
            {
                // cout << TermColor::BLUE() << "run_thread\n" << TermColor::RESET() << endl;
                myfile << "---monitor_thread, map_size: " << data_map.size();
                if( data_map.size() > 0 ) {
                myfile << "  start.t=" << std::setprecision(20) << data_map.begin()->first << "  ";
                myfile << "end.t=" << std::setprecision(20) << data_map.rbegin()->first << "  ";
                }
                myfile << endl;

                // color-coded show
                for( auto it = data_map.begin(); it != data_map.end() ; it++ ) {
                    // auto node = it->second;
                    bool status1 = it->second->isImageAvailable();
                    bool status2 = it->second->isPoseAvailable();
                    bool status3 = it->second->isOptPoseAvailable();
                    if( !status3 ) {
                        if( status1&& status2 ) myfile << TermColor::GREEN() << "|" << TermColor::RESET();
                        if( status1&& !status2 ) myfile << TermColor::YELLOW() << "|" << TermColor::RESET();
                        if( !status1&& status2 ) myfile << TermColor::BLUE() << "|" << TermColor::RESET();
                        if( !status1&& !status2 ) myfile << TermColor::RED() << "|" << TermColor::RESET();
                    }
                    else {
                        if( status1&& status2 ) myfile << TermColor::iGREEN() << "|" << TermColor::RESET();
                        if( status1&& !status2 ) myfile << TermColor::iYELLOW() << "|" << TermColor::RESET();
                        if( !status1&& status2 ) myfile << TermColor::iBLUE() << "|" << TermColor::RESET();
                        if( !status1&& !status2 ) myfile << TermColor::iRED() << "|" << TermColor::RESET();
                    }
                    // myfile << it->first << endl;
                }
                myfile << endl;
            }


            if( data_map.size() > 0 ) {
                // older than 10sec from current are removed
                ros::Time latest_t = data_map.rbegin()->first;
                for( auto it = data_map.begin(); it != data_map.end() ; it++ ) {
                    ros::Duration diff = latest_t - it->first;
                    if( diff > ros::Duration(1) ) {
                        //remove this
                        it->second->~ARDataNode();
                        data_map.erase( it );

                    }
                }
            }
        }


        rate.sleep();
    }
    cout << TermColor::GREEN() << "finished thread `monitor_thread`\n" << TermColor::RESET() << endl;
}

#if 0
// #define __ARDataManager__run_thread( msg ) msg;
#define __ARDataManager__run_thread( msg ) ;
void ARDataManager::run_thread( int hz )
{
    cout << TermColor::GREEN() << "starting thread `run_thread`. will have an inf-loop @" << hz << "\n" << TermColor::RESET() << endl;
    ros::Rate rate(hz);

    if( renderer == NULL ) {
        cout << TermColor::RED() << "you need to set the renderer (with ARDataManager::setRenderer()) before you can run this thread\n" << TermColor::RESET();
        run_thread_flag = false;
    }

    if( isPubARImageset==false ) {
        cout << TermColor::RED() << "you need to set AR image publisher before running this thread\n" << TermColor::RESET() ;
        run_thread_flag = false;
    }

    ros::Time prev_rendered_stamp;
    while( run_thread_flag )
    {
        rate.sleep();
        __ARDataManager__run_thread( cout << "---run thread\n"; )


        const ARDataNode* node = latestNodeWherePoseisAvailable();
        const ARDataNode* nodeX = latestNodeWhereIMUPoseisAvailable();
        if( node == NULL || nodeX == NULL ) {
            __ARDataManager__run_thread( cout << "node is NULL"; )
            continue;
        }

        __ARDataManager__run_thread(
        cout << "render pose.t="<< node->getPoseTimestamp() << "  image.t="<< node->getImageTimestamp() << endl;
        cout << "renderX pose.t="<< nodeX->getIMUPoseTimestamp() << "  image.t="<< nodeX->getImageTimestamp() << endl;
        )

        if( prev_rendered_stamp >= node->getPoseTimestamp() ) {
            __ARDataManager__run_thread( cout << "this was already rendered before, so ignore it\n"; )
            continue;
        }



        #if 0
            cout << "---run_thread, map_size: " << data_map.size();
            if( data_map.size() > 0 ) {
            cout << "  start.t=" << std::setprecision(20) << data_map.begin()->first << "  ";
            cout << "end.t=" << std::setprecision(20) << data_map.rbegin()->first << "  ";
            }
            cout << endl;

            // color-coded show
            for( auto it = data_map.begin(); it != data_map.end() ; it++ ) {
                // auto node = it->second;
                bool status1 = it->second->isImageAvailable();
                bool status2 = it->second->isPoseAvailable();
                bool status3 = it->second->isIMUPoseAvailable();
                string ch = "|";
                if( it->second == node ) {
                    ch = "r";
                }
                if( it->second == nodeX ) {
                    ch = "X";
                }
                if( !status3 ) {
                    if( status1&& status2 ) cout << TermColor::GREEN() << ch << TermColor::RESET();
                    if( status1&& !status2 ) cout << TermColor::YELLOW() << ch << TermColor::RESET();
                    if( !status1&& status2 ) cout << TermColor::BLUE() << ch << TermColor::RESET();
                    if( !status1&& !status2 ) cout << TermColor::RED() << ch << TermColor::RESET();
                }
                else {
                    if( status1&& status2 ) cout << TermColor::iGREEN() << ch << TermColor::RESET();
                    if( status1&& !status2 ) cout << TermColor::iYELLOW() << ch << TermColor::RESET();
                    if( !status1&& status2 ) cout << TermColor::iBLUE() << ch << TermColor::RESET();
                    if( !status1&& !status2 ) cout << TermColor::iRED() << ch << TermColor::RESET();
                }
            }
            cout << endl;
        #endif


        cv::Mat buffer;
        #if 1
        //      ---simple render ..works!
        // render on last pose where camera pose was available.
        renderer->renderIn( node->getImage(), node->getPose(), buffer );
        prev_rendered_stamp = node->getImageTimestamp();
        #else
        //--> this bit doesnt work..:/
        if( node->isIMUPoseAvailable() && nodeX->isIMUPoseAvailable() ) {
        // render with imu prop.
        // w_T_X <== w_T_r + ( r_T_X ). but r_T_X can currently only be computed with imu propagation
        Matrix4d w_T_X = node->getPose() * (  node->getIMUPose().inverse() * nodeX->getIMUPose()  );
        renderer->renderIn( nodeX->getImage(), w_T_X, buffer );
        prev_rendered_stamp = nodeX->getImageTimestamp();
        }
        else {
            renderer->renderIn( node->getImage(), node->getPose(), buffer );
            prev_rendered_stamp = node->getImageTimestamp();



        #endif

        #if 1
        cv_bridge::CvImage cv_image;
        cv_image.image = buffer;
        cv_image.encoding = "bgr8";
        sensor_msgs::Image ros_image_msg;
        cv_image.toImageMsg(ros_image_msg);
        pub_ARimage.publish( ros_image_msg );
        #else
        cv::imshow( "buffer", buffer );
        cv::waitKey(10);
        #endif

    }
    cout << TermColor::GREEN() << "finished thread `run_thread`\n" << TermColor::RESET() << endl;

}
#endif


#define __ARDataManager__run_thread_duo( msg ) msg;
// #define __ARDataManager__run_thread_duo( msg ) ;
void ARDataManager::run_thread_duo( int hz) //newer
{
    cout << TermColor::GREEN() << "starting thread `run_thread`. will have an inf-loop @" << hz << "\n" << TermColor::RESET() << endl;
    ros::Rate rate(hz);

    if( renderer == NULL ) {
        cout << TermColor::RED() << "you need to set the renderer (with ARDataManager::setRenderer()) before you can run this thread\n" << TermColor::RESET();
        run_thread_flag = false;
    }

    if( isPubARImageset==false ) {
        cout << TermColor::RED() << "you need to set AR image publisher before running this thread\n" << TermColor::RESET() ;
        run_thread_flag = false;
    }

    ros::Time prev_rendered_stamp = ros::Time();
    while( run_thread_flag )
    {
        rate.sleep();
        // __ARDataManager__run_thread_duo( cout << "---run thread_duo\n" );
        {
            std::lock_guard<std::mutex> lk(data_map_mutex);


            const ARDataNode* node =  latestNodeWhereOptPoseisAvailable();
            const ARDataNode* nodeX = latestNodeWhereImageisAvailable();


            if( node == NULL || nodeX==NULL ) {
                __ARDataManager__run_thread_duo( cout << "either of node or nodeX is null. This means data_map is empty or data_map doesnot contain pose or image data\n" );
                continue;
            }

            ros::Time t_node = node->getOptPoseTimestamp();
            // ros::Time t_node = node->getPoseTimestamp();
            ros::Time t_nodeX = nodeX->getImageTimestamp();
            int64_t diff_ms =  ( int64_t(t_nodeX.toNSec()) - int64_t(t_node.toNSec()) ) / 1000000L;

            if( prev_rendered_stamp >= t_nodeX ) {
                __ARDataManager__run_thread_duo( cout << "this was already rendered before, so ignore it\n"; )
                continue;
            }

            __ARDataManager__run_thread_duo(
            cout << "latestNodeWhereOptPoseisAvailable=" << t_node << "\t";
            cout << "latestNodeWhereImageisAvailable=" << t_nodeX << "\t";
            cout << TermColor::RED() << "diff_ms=" << diff_ms  << TermColor::RESET() << "\t";
            cout << TermColor::iYELLOW() << "WorldID,setID_of_worldID=" << node->getOptPose_worldID() << ":" << node->getOptPose_setID_of_worldID() << TermColor::RESET() << "\t";
            cout << endl;
            )


            // do rendering.
            // TODO Although using slightly old pose, the correct way is to
            // use imu pose of the current using 200hz message



            Matrix4d w0_T_node = node->getOptCamPose(); //camera pose (in world co-ordinates) at t=node
            Matrix4d w0_T_nodeximu = Matrix4d::Identity();//TODO imu pose (in world co-ordinates) at t=nodeX
            {
                // get a imu pose from the queue.
                std::lock_guard<std::mutex> lk(imuprop_msgs_mutex);
                __ARDataManager__run_thread_duo(
                cout << "\tlen of queue = " << imuprop_msgs.size() << endl;)
                geometry_msgs::PoseStamped currimumsg;
                while( !imuprop_msgs.empty() ) {
                    currimumsg = imuprop_msgs.front();
                    int64_t imu_t__m__image_t = int64_t(currimumsg.header.stamp.toNSec()) - int64_t(t_nodeX.toNSec());
                    if( abs(imu_t__m__image_t) < 1000000L*5L )
                        break;
                    else{
                        // __ARDataManager__run_thread_duo(
                        // cout << "\tpop t=" << currimumsg.header.stamp ;)
                        imuprop_msgs.pop();
                    }
                }
                __ARDataManager__run_thread_duo(
                cout << "\n\tuse imu message at t=" << currimumsg.header.stamp << "\t";
                cout << "still have " << imuprop_msgs.size() << " imu msg in queue" << "\t";
                cout << endl;
                )

                PoseManipUtils::geometry_msgs_Pose_to_eigenmat( currimumsg.pose, w0_T_nodeximu );
            }
            // TODO is_imu_T_cam_available?
            assert( is_imu_T_cam_available() && "[ARDataManager::run_thread_duo] imu_T_cam doesnt seem to be available. I need it to " );
            Matrix4d w0_T_nodexcam = w0_T_nodeximu * get_imu_T_cam();


            cv::Mat buffer;
            // render only if current setID_of_worldID is zero(ie. poses are in world-0)
            if( node->getOptPose_setID_of_worldID() == 0  ) {
                // renderer->renderIn( nodeX->getImage(), w0_T_node, buffer );
                renderer->renderIn( nodeX->getImage(), w0_T_nodexcam, buffer );
                cout << "renderrIn on image at t=" << nodeX->getImageTimestamp() << "\t";
                cout << " w0_T_c=" << PoseManipUtils::prettyprintMatrix4d( w0_T_nodexcam  );
                cout << endl;
            }
            else {
                cv::Mat __tmpo = nodeX->getImage();
                if( __tmpo.channels() == 3 )
                    buffer = __tmpo.clone();
                else
                    cv::cvtColor(__tmpo, buffer, cv::COLOR_GRAY2BGR);

                cout << "clone t="<< nodeX->getImageTimestamp() << "\n";
            }

            prev_rendered_stamp = t_nodeX;

            #if 1
            cv_bridge::CvImage cv_image;
            cv_image.image = buffer;
            cv_image.encoding = "bgr8";
            sensor_msgs::Image ros_image_msg;
            cv_image.toImageMsg(ros_image_msg);
            pub_ARimage.publish( ros_image_msg );
            #else
            cv::imshow( "buffer", buffer );
            cv::waitKey(10);
            #endif
        }

    }

}

//////////////////////////////////////////////////////////////////////
/////////////////////// HELPERS///////////////////////////////////////
//////////////////////////////////////////////////////////////////////
const ARDataNode* ARDataManager::latestNodeWherePoseisAvailable() const
{
    // std::lock_guard<std::mutex> lk(data_map_mutex);
    for( auto it=data_map.rbegin() ; it != data_map.rend() ; it++ )  //loop in reverse
    {
        bool status1 = it->second->isImageAvailable();
        bool status2 = it->second->isPoseAvailable();
        if( status1 && status2 )
            return it->second;
    }
    return NULL;
}

const ARDataNode* ARDataManager::latestNodeWhereOptPoseisAvailable() const
{
    // std::lock_guard<std::mutex> lk(data_map_mutex);
    for( auto it=data_map.rbegin() ; it != data_map.rend() ; it++ )  //loop in reverse
    {
        bool status1 = it->second->isImageAvailable();
        bool status2 = it->second->isOptPoseAvailable();
        if( status1 && status2 )
            return it->second;
    }
    return NULL;
}


const ARDataNode* ARDataManager::latestNodeWhereImageisAvailable() const
{
    // std::lock_guard<std::mutex> lk(data_map_mutex);
    for( auto it=data_map.rbegin() ; it != data_map.rend() ; it++ )  //loop in reverse
    {
        bool status1 = it->second->isImageAvailable();
        if( status1 )
            return it->second;
    }
    return NULL;
}

//////////////////////////////////////////////////////////////////////////
// IMU_T_CAM /////////
//////////////////////////////////////////////////////////////////////////

// #define __ARDataManager__extrinsic_cam_imu_callback( msg ) msg;
#define __ARDataManager__extrinsic_cam_imu_callback( msg ) ;

void ARDataManager::extrinsic_cam_imu_callback( const nav_msgs::Odometry::ConstPtr msg )
{
    // __NODEDATAMANAGER_CALLBACKS( cout << TermColor::GREEN() << "[NodeDataManager::extrinsic_cam_imu_callback]" << msg->header.stamp  << TermColor::RESET() << endl; )

    // Acquire lock
    //      update imu_T_cam,
    //      update last got timestamp
    //      set is_imu_cam_extrinsic_available to true
    {
        std::lock_guard<std::mutex> lk(imu_cam_mx);
        PoseManipUtils::geometry_msgs_Pose_to_eigenmat( msg->pose.pose, this->imu_T_cam );
        this->imu_T_cam_stamp = msg->header.stamp;
        this->imu_T_cam_available = true;

    }


    __ARDataManager__extrinsic_cam_imu_callback(
    cout << TermColor::GREEN() << "[ARDataManager::extrinsic_cam_imu_callback]" << msg->header.stamp  << TermColor::RESET();
    cout << " imu_T_cam = " << PoseManipUtils::prettyprintMatrix4d(this->imu_T_cam);
    cout << endl;
    )

}


Matrix4d ARDataManager::get_imu_T_cam() const
{
    std::lock_guard<std::mutex> lk(imu_cam_mx);
    // Remove this if, once i am confident everythinbg is ok!
    if( imu_T_cam_available == false )
    {
        ROS_ERROR( "[ARDataManager::get_imu_T_cam] posegraph solver, you requested imu_T_cam aka imu-cam extrinsic calib, but currently it is not available. FATAL ERROR.\n");
        exit(1);
    }
    assert( imu_T_cam_available );
    return imu_T_cam;
}


void ARDataManager::get_imu_T_cam( Matrix4d& res, ros::Time& _t ) const
{
    std::lock_guard<std::mutex> lk(imu_cam_mx);
    // Remove this if, once i am confident everythinbg is ok!
    if( imu_T_cam_available == false )
    {
        ROS_ERROR( "[ARDataManager::get_imu_T_cam] posegraph solver, you requested imu_T_cam aka imu-cam extrinsic calib, but currently it is not available. FATAL ERROR.\n");
        exit(1);
    }

    assert( imu_T_cam_available );
    res = imu_T_cam;
    _t = imu_T_cam_stamp;
    return;
}

bool ARDataManager::is_imu_T_cam_available() const
{
    std::lock_guard<std::mutex> lk(imu_cam_mx);
    return imu_T_cam_available;
}



//////////////////////////////////////////////////////////////////////////
// DONE IMU_T_CAM /////////
//////////////////////////////////////////////////////////////////////////
