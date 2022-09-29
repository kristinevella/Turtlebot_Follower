// Declaration file 

#pragma once //designed to include the current source file only once in a single compilation.

#ifndef TURTLEBOT_FOLLOWER //used for conditional compiling.
#define TURTLEBOT_FOLLOWER

#include <ros/ros.h> // including the ros header file

#include <sensor_msgs/LaserScan.h>
#include <geometry_msgs/Twist.h>
#include <nav_msgs/Odometry.h>

#define DEG2RAD (M_PI / 180.0)
#define RAD2DEG (180.0 / M_PI)

/* defining the class */
class TurtlebotFollower
{
    public:
        TurtlebotFollower(ros::NodeHandle &nh, ros::NodeHandle &pnh); //constructor method
        ~TurtlebotFollower(); // distructor method
        void runOnce(); // runOnce method to control the flow of program
        bool init();
        bool controlLoop();
    private:
        // ROS NodeHandle
        ros::NodeHandle nh_; // Defining the ros NodeHandle variable for registrating the same with the master

        // ROS Topic Publishers
        ros::Publisher cmd_vel_pub_;

        // ROS Topic Subscribers
        ros::Subscriber laser_scan_sub_;
        ros::Subscriber odom_sub_;

        // Variables
        double escape_range_;
        double check_forward_dist_;
        double check_side_dist_;

        double scan_data_[3] = {0.0, 0.0, 0.0};

        double tb3_pose_;
        double prev_tb3_pose_;

        // Functions
        void odomMsgCallBack(const nav_msgs::Odometry::ConstPtr &msg);
        void laserScanMsgCallBack(const sensor_msgs::LaserScan::ConstPtr &msg);
        void updatecommandVelocity(double linear, double angular);
};
#endif  