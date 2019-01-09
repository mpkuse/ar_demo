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

///////////////////////////////////////////////////////////////////////////////


void ARDataManager::run_thread( int hz )
{
    cout << TermColor::GREEN() << "starting thread `run_thread`. will have an inf-loop @" << hz << "\n" << TermColor::RESET() << endl;

    ros::Rate rate(hz);
    while( run_thread_flag )
    {
        {
            std::lock_guard<std::mutex> lk(data_map_mutex);

            // cout << TermColor::BLUE() << "run_thread\n" << TermColor::RESET() << endl;
            cout << "---\nrun_thread, map_size: " << data_map.size();
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