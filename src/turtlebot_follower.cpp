#include "turtlebot_follower.hpp"

//constructor method
TurtlebotFollower::TurtlebotFollower(ros::NodeHandle &nh, ros::NodeHandle &pnh)
{
    nh_ = nh;
    std::cout<<"from Constructor \n";
    ROS_INFO("TurtleBot Follower Node Init");
    auto ret = init(); //Init gazebo ros turtlebot3 node
    ROS_ASSERT(ret);
    // storing the values in the member variable
    // get the parameters or configurations and store them in member variables
    // initialize the publisher and subscribers
}

// Distructor method
TurtlebotFollower::~TurtlebotFollower()
{
    std::cout<<"from Distructor \n";
    updatecommandVelocity(0.0, 0.0);
    ros::shutdown();
    // Free up the memory assigned from heap
}

void TurtlebotFollower::runOnce()
{
    std::cout<<"from Runonce \n";
}

/*******************************************************************************
* Init function
*******************************************************************************/
bool TurtlebotFollower::init()
{
  // initialize variables
  escape_range_       = 30.0 * DEG2RAD;
  check_forward_dist_ = 0.7;
  check_side_dist_    = 0.6;

  tb3_pose_ = 0.0;
  prev_tb3_pose_ = 0.0;

  // initialize publishers
  cmd_vel_pub_   = nh_.advertise<geometry_msgs::Twist>("robot1/cmd_vel", 10);

  // initialize subscribers
  laser_scan_sub_  = nh_.subscribe("robot1/scan", 10, &TurtlebotFollower::laserScanMsgCallBack, this);
  odom_sub_ = nh_.subscribe("robot1/odom", 10, &TurtlebotFollower::odomMsgCallBack, this);

  return true;
}

void TurtlebotFollower::odomMsgCallBack(const nav_msgs::Odometry::ConstPtr &msg)
{
  double siny = 2.0 * (msg->pose.pose.orientation.w * msg->pose.pose.orientation.z + msg->pose.pose.orientation.x * msg->pose.pose.orientation.y);
	double cosy = 1.0 - 2.0 * (msg->pose.pose.orientation.y * msg->pose.pose.orientation.y + msg->pose.pose.orientation.z * msg->pose.pose.orientation.z);  

	tb3_pose_ = atan2(siny, cosy);
}

void TurtlebotFollower::laserScanMsgCallBack(const sensor_msgs::LaserScan::ConstPtr &msg)
{
  uint16_t scan_angle[3] = {0, 30, 330};

  for (int num = 0; num < 3; num++)
  {
    if (std::isinf(msg->ranges.at(scan_angle[num])))
    {
      scan_data_[num] = msg->range_max;
    }
    else
    {
      scan_data_[num] = msg->ranges.at(scan_angle[num]);
    }
  }
}

void TurtlebotFollower::updatecommandVelocity(double linear, double angular)
{
  geometry_msgs::Twist cmd_vel;

  cmd_vel.linear.x  = linear;
  cmd_vel.angular.z = angular;

  cmd_vel_pub_.publish(cmd_vel);
}

/*******************************************************************************
* Control Loop function
*******************************************************************************/
bool TurtlebotFollower::controlLoop()
{
    ROS_INFO("From controlLoop");

    updatecommandVelocity(0.3, 1.5);

}