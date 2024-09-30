#include "../include/tf_nav.h"
#include <tf/transform_broadcaster.h> //HomeWork 4.c

void TF_NAV::arucoPoseCallback(const geometry_msgs::PoseStamped & msg){ //HomeWork 4 ---------------------------------------------------------

    tf::Vector3 arucoPosition(msg.pose.position.x, msg.pose.position.y, msg.pose.position.z);
    tf::Quaternion arucoOrientation(msg.pose.orientation.x, msg.pose.orientation.y, msg.pose.orientation.z, msg.pose.orientation.w);                  
    tf::Transform tfCameraAruco(arucoOrientation, arucoPosition);  
    // aruco wrt world frame
    _tfAruco = _tfBase * _tfBaseCamera * tfCameraAruco;
    //Broadcast TF ---- HomeWork 4.c
    static tf::TransformBroadcaster br;
    br.sendTransform(tf::StampedTransform(_tfAruco, ros::Time::now(), "map", "aruco_frame"));

}

TF_NAV::TF_NAV() {

    _position_pub = _nh.advertise<geometry_msgs::PoseStamped>( "/fra2mo/pose", 1 );
    int goalNumber;
    _nh.getParam("goalNumber", goalNumber);

    _cur_pos << 0.0, 0.0, 0.0;
    _cur_or << 0.0, 0.0, 0.0, 1.0;

    //HomeWork 2.b -----------------------------------------------------------------------
    _goal_pos.resize(goalNumber);
    _goal_or.resize(goalNumber);
    for (int i = 0; i < goalNumber; i++) {
        _goal_pos.at(i) << 0.0, 0.0, 0.0;
        _goal_or.at(i) << 0.0, 0.0, 0.0, 1.0;
    }
    //------------------------------------------------------------------------------------
    
    _home_pos << -3.0, 5.0, 0.0; //HomeWork 1.a
    _home_or << 0, 0, -0.7068, 0.7073; 

    // aruco wrt world frame ---- HomeWork 4 -------------------------------------------------------------------------------------------
    if(goalNumber == 1){
        ros::Rate r( 5 );
        tf::TransformListener listener;
        tf::StampedTransform tfBaseCamera;
        _aruco_pose_sub = _nh.subscribe("/aruco_single/pose", 1, &TF_NAV::arucoPoseCallback, this);
        

        try {
            listener.waitForTransform( "base_footprint", "camera_depth_optical_frame", ros::Time(0), ros::Duration(10.0) );
            listener.lookupTransform( "base_footprint", "camera_depth_optical_frame", ros::Time(0), tfBaseCamera );
        } catch ( tf::TransformException &ex ) {
            ROS_ERROR("%s", ex.what());
            r.sleep();
            return;
        }
        _tfBaseCamera = tfBaseCamera;
    }
    // -----------------------------------------------------------------------------------------------------------------------------------
}

void TF_NAV::tf_listener_fun() {
    ros::Rate r( 5 );
    tf::TransformListener listener;
    tf::StampedTransform transform;
    
    while ( ros::ok() )
    {
        try {
            listener.waitForTransform( "map", "base_footprint", ros::Time(0), ros::Duration(10.0) );
            listener.lookupTransform( "map", "base_footprint", ros::Time(0), transform );

        }
        catch( tf::TransformException &ex ) {
            ROS_ERROR("%s", ex.what());
            r.sleep();
            continue;
        }
        
        _tfBase = transform; //HomeWork 4
       
        _cur_pos << transform.getOrigin().x(), transform.getOrigin().y(), transform.getOrigin().z();
        _cur_or << transform.getRotation().w(),  transform.getRotation().x(), transform.getRotation().y(), transform.getRotation().z();
        position_pub();
        r.sleep();
    }

}

void TF_NAV::position_pub() {

    geometry_msgs::PoseStamped pose;

    pose.header.stamp = ros::Time::now();
    pose.header.frame_id = "map";

    pose.pose.position.x = _cur_pos[0];
    pose.pose.position.y = _cur_pos[1];
    pose.pose.position.z = _cur_pos[2];

    pose.pose.orientation.w = _cur_or[0];
    pose.pose.orientation.x = _cur_or[1];
    pose.pose.orientation.y = _cur_or[2];
    pose.pose.orientation.z = _cur_or[3];

    _position_pub.publish(pose);
}

void TF_NAV::goal_listener() { //HomeWork 2.b -------------------------------------------------------------------------------------------
    ros::Rate r( 1 );
    tf::TransformListener listener;
    tf::StampedTransform transform;
    
    int goalNumber;
    _nh.getParam("goalNumber", goalNumber);
    static std::vector<bool> logged(goalNumber, false);

    while ( ros::ok() )
    {
        for (int i = 0; i < goalNumber; i++) {
            std::string goal_frame = "goal" + std::to_string(i + 1);

            try
            {
                listener.waitForTransform( "map", goal_frame, ros::Time( 0 ), ros::Duration( 10.0 ) );
                listener.lookupTransform( "map", goal_frame, ros::Time( 0 ), transform );
                if (!logged[i]) {
                    ROS_INFO("Goal_[%d]: \n pos (x:%f, y:%f, z:%f) \n rot (x:%f, y:%f, z:%f, w:%f)\n", (i+1), 
                                                        transform.getOrigin().x(), transform.getOrigin().y(), transform.getOrigin().z(),
                                                        transform.getRotation().w(),  transform.getRotation().x(), transform.getRotation().y(), transform.getRotation().z());
                    logged[i]=true;
                }
            }
            catch( tf::TransformException &ex )
            {
                ROS_ERROR("%s", ex.what());
                r.sleep();
                continue;
            }

            _goal_pos.at(i) << transform.getOrigin().x(), transform.getOrigin().y(), transform.getOrigin().z();
            _goal_or.at(i) << transform.getRotation().w(),  transform.getRotation().x(), transform.getRotation().y(), transform.getRotation().z();
            
        }
        r.sleep();
    }    
}

void TF_NAV::send_goal() {
    ros::Rate r( 5 );
    int cmd;
    move_base_msgs::MoveBaseGoal goal;

    int goalNumber;
    _nh.getParam("goalNumber", goalNumber);

    while ( ros::ok() )
    {
        if(goalNumber == 4){
            std::cout<<"\nInsert 1 to send goals from TF: 2.c "<<std::endl;
            std::cout<<"Insert 2 to send home position goal "<<std::endl;
        }

        if(goalNumber == 6){
            std::cout<<"Insert 2 to send home position goal "<<std::endl;
            std::cout<<"Insert 3 to explore the map: 3.a "<<std::endl;
        }

        if(goalNumber == 1){
            std::cout<<"Insert 2 to send home position goal "<<std::endl;
            std::cout<<"Insert 4 to find aruco marker: 4.b "<<std::endl;
        }
        
        std::cout<<"Inser your choice"<<std::endl;
        std::cin>>cmd;

        if ( (cmd == 1) && (goalNumber == 4))  { //HomeWork 2.c -------------------------------------------------------------------------
            MoveBaseClient ac("move_base", true);
            while(!ac.waitForServer(ros::Duration(5.0))){
            ROS_INFO("Waiting for the move_base action server to come up");
            }

            Eigen::Vector4d goalOrder;
            goalOrder << 3, 4, 2, 1;

                for (int goal_index = 0; goal_index < goalNumber; goal_index++) {
                    goal.target_pose.header.frame_id = "map";
                    goal.target_pose.header.stamp = ros::Time::now();

                    goal.target_pose.pose.position.x = _goal_pos.at(goalOrder(goal_index) - 1)[0];
                    goal.target_pose.pose.position.y = _goal_pos.at(goalOrder(goal_index) - 1)[1];
                    goal.target_pose.pose.position.z = _goal_pos.at(goalOrder(goal_index) - 1)[2];

                    goal.target_pose.pose.orientation.w = _goal_or.at(goalOrder(goal_index) - 1)[0];
                    goal.target_pose.pose.orientation.x = _goal_or.at(goalOrder(goal_index) - 1)[1];
                    goal.target_pose.pose.orientation.y = _goal_or.at(goalOrder(goal_index) - 1)[2];
                    goal.target_pose.pose.orientation.z = _goal_or.at(goalOrder(goal_index) - 1)[3];

                    ROS_INFO("Sending goal %f", goalOrder(goal_index));

                    ros::Duration(0.1).sleep();
                    ac.sendGoal(goal);
                    ac.waitForResult();

                    if (ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
                        ROS_INFO("The mobile robot arrived in the TF goal number %f",  goalOrder(goal_index));
                    else 
                        ROS_INFO("The base failed to move for some reason");
                }
        }
        else if ( cmd == 2 ) { //HomeWork 1.a -------------------------------------------------------------------------------------
            MoveBaseClient ac("move_base", true);
            while(!ac.waitForServer(ros::Duration(5.0))){
            ROS_INFO("Waiting for the move_base action server to come up");
            }
            goal.target_pose.header.frame_id = "map";
            goal.target_pose.header.stamp = ros::Time::now();
            
            goal.target_pose.pose.position.x = _home_pos[0];
            goal.target_pose.pose.position.y = _home_pos[1];
            goal.target_pose.pose.position.z = _home_pos[2];

            goal.target_pose.pose.orientation.w = _home_or[0];
            goal.target_pose.pose.orientation.x = _home_or[1];
            goal.target_pose.pose.orientation.y = _home_or[2];
            goal.target_pose.pose.orientation.z = _home_or[3];

            ROS_INFO("Sending HOME position as goal");
            ac.sendGoal(goal);

            ac.waitForResult();

            if(ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
                ROS_INFO("The mobile robot arrived in the HOME position");
            else
                ROS_INFO("The base failed to move for some reason");
        }
        else if ( (cmd == 3) && (goalNumber == 6) ) { //HomeWork 3.a ------------------------------------------------------------------
            MoveBaseClient ac("move_base", true);
            while(!ac.waitForServer(ros::Duration(5.0))){
            ROS_INFO("Waiting for the move_base action server to come up");
            }
            for (int goal_index = 0; goal_index < goalNumber; goal_index++) {
                goal.target_pose.header.frame_id = "map";
                goal.target_pose.header.stamp = ros::Time::now();

                goal.target_pose.pose.position.x = _goal_pos.at(goal_index)[0];
                goal.target_pose.pose.position.y = _goal_pos.at(goal_index)[1];
                goal.target_pose.pose.position.z = _goal_pos.at(goal_index)[2];

                goal.target_pose.pose.orientation.w = _goal_or.at(goal_index)[0];
                goal.target_pose.pose.orientation.x = _goal_or.at(goal_index)[1];
                goal.target_pose.pose.orientation.y = _goal_or.at(goal_index)[2];
                goal.target_pose.pose.orientation.z = _goal_or.at(goal_index)[3];

                ROS_INFO("Sending goal %i", goal_index);

                ros::Duration(0.1).sleep();
                ac.sendGoal(goal);
                ac.waitForResult();

                if (ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
                    ROS_INFO("The mobile robot arrived in the TF goal number %i",  goal_index);
                else 
                    ROS_INFO("The base failed to move for some reason");
            }
        }
        else if ( (cmd == 4) && (goalNumber == 1) ) { //HomeWork 4.b ---------------------------------------------------------------------
            MoveBaseClient ac("move_base", true);
            while(!ac.waitForServer(ros::Duration(5.0))){
            ROS_INFO("Waiting for the move_base action server to come up");
            }
            goal.target_pose.header.frame_id = "map";
            goal.target_pose.header.stamp = ros::Time::now();
            
            goal.target_pose.pose.position.x = _goal_pos.at(0)[0];
            goal.target_pose.pose.position.y = _goal_pos.at(0)[1];
            goal.target_pose.pose.position.z = _goal_pos.at(0)[2];

            goal.target_pose.pose.orientation.w = _goal_or.at(0)[0];
            goal.target_pose.pose.orientation.x = _goal_or.at(0)[1];
            goal.target_pose.pose.orientation.y = _goal_or.at(0)[2];
            goal.target_pose.pose.orientation.z = _goal_or.at(0)[3];

            ROS_INFO("Sending Goal to find Aruco Marker");
            ac.sendGoal(goal);

            ac.waitForResult();

            if(ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
                ROS_INFO("The mobile robot has found the Aruco Marker");
            else
                ROS_INFO("The base failed to move for some reason");
        
            //Vision Task
            while(!ac.waitForServer(ros::Duration(5.0))){
            ROS_INFO("Waiting for the move_base action server to come up");
            }
            goal.target_pose.header.frame_id = "map";
            goal.target_pose.header.stamp = ros::Time::now();
            
            goal.target_pose.pose.position.x = _tfAruco.getOrigin().x()+1;
            goal.target_pose.pose.position.y = _tfAruco.getOrigin().y();
            goal.target_pose.pose.position.z = 0;

            goal.target_pose.pose.orientation.w = _cur_or[0];
            goal.target_pose.pose.orientation.x = _cur_or[1];
            goal.target_pose.pose.orientation.y = _cur_or[2];
            goal.target_pose.pose.orientation.z = _cur_or[3];

            ROS_INFO("Sending Goal to move toward the Aruco Marker");
            ac.sendGoal(goal);

            ac.waitForResult();

            if(ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
                ROS_INFO("The mobile robot arrived in the DESIRED position");
            else
                ROS_INFO("The base failed to move for some reason");
        
        }
         else { // fine --------------------------------------------------------------------------------------------------------------
            ROS_INFO("Wrong input!");
        }
        r.sleep();
    }
    
}

void TF_NAV::run() {
    boost::thread tf_listener_fun_t( &TF_NAV::tf_listener_fun, this );
    boost::thread tf_listener_goal_t( &TF_NAV::goal_listener, this );
    boost::thread send_goal_t( &TF_NAV::send_goal, this );
    ros::spin();
}



int main( int argc, char** argv ) {
    ros::init(argc, argv, "tf_navigation");   
    TF_NAV tfnav;
    tfnav.run();

    return 0;
}