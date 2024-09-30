#include <ros/ros.h>
#include <tf/transform_listener.h>
#include <geometry_msgs/PoseStamped.h>
#include "boost/thread.hpp"
#include "Eigen/Dense"
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>

class TF_NAV {

    public:
        TF_NAV();
        void run();
        void tf_listener_fun();
        void position_pub();
        void goal_listener();
        void send_goal();

        void arucoPoseCallback(const geometry_msgs::PoseStamped & msg); //HomeWork 4

    private:

        ros::NodeHandle _nh;

        ros::Publisher _position_pub;

        Eigen::Vector3d _home_pos;
        Eigen::Vector4d _home_or;


        Eigen::Vector3d _cur_pos;
        Eigen::Vector4d _cur_or;

        //HomeWork 4
        ros::Subscriber _aruco_pose_sub;
        tf::Transform _tfBaseCamera;    // transformation matrix from camera to base
        tf::Transform _tfAruco;         // pose of the Aruco Marker in base frame
        tf::Transform _tfBase;          // current pose of the base footprint

        std::vector<Eigen::Vector3d> _goal_pos;
        std::vector<Eigen::Vector4d> _goal_or;
        
        typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;

};