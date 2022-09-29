// Declaration file 

#pragma once //designed to include the current source file only once in a single compilation.

#ifndef MARKER_ATTACHER //used for conditional compiling.
#define MARKER_ATTACHER

#include <ros/ros.h> // including the ros header file

#include <sensor_msgs/LaserScan.h>
#include <geometry_msgs/Twist.h>
#include <nav_msgs/Odometry.h>
#include <gazebo_msgs/ModelState.h>
#include <gazebo_msgs/SetModelState.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

#define DEG2RAD (M_PI / 180.0)
#define RAD2DEG (180.0 / M_PI)

/* defining the class */
class MarkerAttacher
{
    public:
        MarkerAttacher(ros::NodeHandle &nh, ros::NodeHandle &pnh); //constructor method
        ~MarkerAttacher(); // distructor method
        void runOnce(); // runOnce method to control the flow of program
        bool init();
        bool controlLoop(ros::NodeHandle &nh);
    private:
        // ROS NodeHandle
        ros::NodeHandle nh_; // Defining the ros NodeHandle variable for registrating the same with the master

        // ROS Topic Publishers
        ros::Publisher cmd_vel_pub_;

        // ROS Topic Subscribers
        ros::Subscriber laser_scan_sub_;
        ros::Subscriber odom_sub_;

        // Variables
        geometry_msgs::Pose pose_;
        geometry_msgs::Twist twist_;

        // Functions
        void odomMsgCallBack(const nav_msgs::Odometry::ConstPtr &msg);
};
#endif  