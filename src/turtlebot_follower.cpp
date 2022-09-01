#include "turtlebot_follower.hpp"

//constructor method
TurtlebotFollower::TurtlebotFollower(ros::NodeHandle &nh, ros::NodeHandle &pnh)
{
    nh_ = nh;
    std::cout<<"from Constructor \n";
    // storing the values in the member variable
    // get the parameters or configurations and store them in member variables
    // initialize the publisher and subscribers
}

// Distructor method
TurtlebotFollower::~TurtlebotFollower()
{
    std::cout<<"from Distructor \n";
    // Free up the memory assigned from heap
}

void TurtlebotFollower::runOnce()
{
    std::cout<<"from Runonce \n";
}