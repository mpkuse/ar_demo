#include "ARDataManager.h"


// #define __ARDataManager__callback( msg ) msg;
#define __ARDataManager__callback( msg ) ;

std::map<ros::Time,ARDataNode*>::iterator
ARDataManager::findClosestKey( const ros::Time& key)
{
    if( data_map.size() == 0 ) {
        __ARDataManager__callback( cout << "not found coz data_map was empty" );
        return data_map.end();
    }

    for( auto it=data_map.begin() ; it!=data_map.end() ; it++ )
    {
        ros::Time it_key = it->first;

        // compare key and it_key
        ros::Duration diff = it_key - key;
        if( (diff.sec == 0  &&  abs(diff.nsec) < 1000000) || (diff.sec == -1  &&  diff.nsec > (1000000000-1000000) )  ) {
            // cout << "NodeDataManager::find_indexof_node " << i << " "<< diff.sec << " " << diff.nsec << endl;
            __ARDataManager__callback(
            cout << TermColor::GREEN() << std::setprecision(20) << "found key="<< key.toSec() << " it_key=" << it_key.toSec() << TermColor::RESET();
            )
            return it;
        }
    }

    __ARDataManager__callback(
    cout << TermColor::RED() << "Notfound t=" << std::setprecision(20) << key.toSec() << TermColor::RESET();
    )
    return data_map.end();
}



void ARDataManager::raw_image_callback( const sensor_msgs::ImageConstPtr msg )
{
    std::lock_guard<std::mutex> lk(data_map_mutex);

    __ARDataManager__callback(
        std::cout << "[raw_image_callback] msg.t=" << msg->header.stamp << "\t";
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
        node = new ARDataNode(); //so allocate new
        data_map[ msg->header.stamp ] = node;
    } else{
         node = it->second;
     }

    __ARDataManager__callback(
        cout << "setImageFromMsg\n";
    )
    node->setImageFromMsg( msg );
}


void ARDataManager::odom_pose_callback( const nav_msgs::Odometry::ConstPtr msg ) ///< w_T_c. pose of camera in the world-cordinate system. All the cameras. only a subset of this will be keyframes
{
    std::lock_guard<std::mutex> lk(data_map_mutex);

    __ARDataManager__callback(
    std::cout << "[odom_pose_callback] t=" << msg->header.stamp << "\t";
    )
    ARDataNode * node = NULL;


    auto it = findClosestKey( msg->header.stamp );
    if( it==data_map.end() ) {//not found
        node = new ARDataNode(); //so allocate new
        data_map[ msg->header.stamp ] = node;
    } else{
         node = it->second;
    }

    __ARDataManager__callback(
    cout << "setPoseFromMsg\n";
    )
    node->setPoseFromMsg( msg );
}

void ARDataManager::imuodom_pose_callback( const nav_msgs::Odometry::ConstPtr msg ) ///< w_T_c. pose of camera in the world-cordinate system. All the cameras. only a subset of this will be keyframes
{
    std::lock_guard<std::mutex> lk(data_map_mutex);

    __ARDataManager__callback(
    std::cout << "[imuodom_pose_callback] t=" << msg->header.stamp << "\t";
    )
    ARDataNode * node = NULL;


    auto it = findClosestKey( msg->header.stamp );
    if( it==data_map.end() ) {//not found
        __ARDataManager__callback( cout << "ignore imu msg" << endl; )
        return;
        // node = new ARDataNode(); //so allocate new
        // data_map[ msg->header.stamp ] = node;
    } else{
         node = it->second;
    }

    __ARDataManager__callback(
    cout << "setIMUPoseFromMsg\n";
    )
    node->setIMUPoseFromMsg( msg );
}



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

///////////////////////////////////////////////////////////////////////////////
void ARDataManager::monitor_thread( int hz, bool printing )
{
    cout << TermColor::GREEN() << "starting thread `monitor_thread`. will have an inf-loop @" << hz << "\n" << TermColor::RESET() << endl;
    ros::Rate rate(hz);
    while( monitor_thread_flag )
    {
        {
            std::lock_guard<std::mutex> lk(data_map_mutex);

            if( printing )
            {
                // cout << TermColor::BLUE() << "run_thread\n" << TermColor::RESET() << endl;
                cout << "---monitor_thread, map_size: " << data_map.size();
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
                    if( !status3 ) {
                        if( status1&& status2 ) cout << TermColor::GREEN() << "|" << TermColor::RESET();
                        if( status1&& !status2 ) cout << TermColor::YELLOW() << "|" << TermColor::RESET();
                        if( !status1&& status2 ) cout << TermColor::BLUE() << "|" << TermColor::RESET();
                        if( !status1&& !status2 ) cout << TermColor::RED() << "|" << TermColor::RESET();
                    }
                    else {
                        if( status1&& status2 ) cout << TermColor::iGREEN() << "|" << TermColor::RESET();
                        if( status1&& !status2 ) cout << TermColor::iYELLOW() << "|" << TermColor::RESET();
                        if( !status1&& status2 ) cout << TermColor::iBLUE() << "|" << TermColor::RESET();
                        if( !status1&& !status2 ) cout << TermColor::iRED() << "|" << TermColor::RESET();
                    }
                }
                cout << endl;
            }


            if( data_map.size() > 0 ) {
                // older than 10sec from current are removed
                ros::Time latest_t = data_map.rbegin()->first;
                for( auto it = data_map.begin(); it != data_map.end() ; it++ ) {
                    ros::Duration diff = latest_t - it->first;
                    if( diff > ros::Duration(10.) ) {
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

        }

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


const ARDataNode* ARDataManager::latestNodeWherePoseisAvailable() const
{
    std::lock_guard<std::mutex> lk(data_map_mutex);
    for( auto it=data_map.rbegin() ; it != data_map.rend() ; it++ )  //loop in reverse
    {
        bool status1 = it->second->isImageAvailable();
        bool status2 = it->second->isPoseAvailable();
        if( status1 && status2 )
            return it->second;
    }
    return NULL;
}


const ARDataNode* ARDataManager::latestNodeWhereIMUPoseisAvailable() const
{
    std::lock_guard<std::mutex> lk(data_map_mutex);
    for( auto it=data_map.rbegin() ; it != data_map.rend() ; it++ )  //loop in reverse
    {
        bool status1 = it->second->isImageAvailable();
        bool status2 = it->second->isIMUPoseAvailable();
        if( status1 && status2 )
            return it->second;
    }
    return NULL;
}
