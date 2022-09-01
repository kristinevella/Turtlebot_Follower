// Declaration file 

#pragma once //designed to include the current source file only once in a single compilation.
#ifndef TURTLEBOT_FOLLOWER //used for conditional compiling.
#define TURTLEBOT_FOLLOWER
#include <ros/ros.h> // including the ros header file

/* defining the class */
class TurtlebotFollower
{
    public:
        TurtlebotFollower(ros::NodeHandle &nh, ros::NodeHandle &pnh); //constructor method
        ~TurtlebotFollower(); // distructor method
        void runOnce(); // runOnce method to control the flow of program
    private:
        ros::NodeHandle nh_; // Defining the ros NodeHandle variable for registrating the same with the master
};
#endif  