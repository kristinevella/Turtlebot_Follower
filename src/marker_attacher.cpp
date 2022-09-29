#include "marker_attacher.hpp"

//constructor method
MarkerAttacher::MarkerAttacher(ros::NodeHandle &nh, ros::NodeHandle &pnh)
{
    nh_ = nh;
    std::cout<<"from Constructor \n";
    ROS_INFO("Marker Attacher Node Init");
    auto ret = init(); //Init gazebo ros turtlebot3 node
    ROS_ASSERT(ret);
    // storing the values in the member variable
    // get the parameters or configurations and store them in member variables
    // initialize the publisher and subscribers
}

// Distructor method
MarkerAttacher::~MarkerAttacher()
{
    std::cout<<"from Distructor \n";
    ros::shutdown();
    // Free up the memory assigned from heap
}

void MarkerAttacher::runOnce()
{
    std::cout<<"from Runonce \n";
}

/*******************************************************************************
* Init function
*******************************************************************************/
bool MarkerAttacher::init()
{
    // initialize subscribers
    odom_sub_ = nh_.subscribe("robot1/odom", 10, &MarkerAttacher::odomMsgCallBack, this);

    return true;
}

void MarkerAttacher::odomMsgCallBack(const nav_msgs::Odometry::ConstPtr &msg)
{
    pose_ = msg->pose.pose;
    twist_ = msg->twist.twist;
}

/*******************************************************************************
* Control Loop function
*******************************************************************************/
bool MarkerAttacher::controlLoop(ros::NodeHandle &nh)
{
    ROS_INFO("From controlLoop");
    
    tf2::Quaternion q_orig, q_rot, q_new;

    // Get the original orientation of 'commanded_pose'
    tf2::convert(pose_.orientation, q_orig);

    double r=0, p=90*DEG2RAD, y=0;  // Rotate the previous pose by 90* about Y
    q_rot.setRPY(r, p, y);

    q_new = q_rot*q_orig;  // Calculate the new orientation
    q_new.normalize();

    // Stuff the new rotation back into the pose. This requires conversion into a msg type
    tf2::convert(q_new, pose_.orientation);

    geometry_msgs::Pose pose;
    pose.position.x = pose_.position.x-0.3;
    pose.position.y = pose_.position.y-0.3;
    pose.position.z = pose_.position.z;
    pose.orientation.x = pose_.orientation.x;
    pose.orientation.y = pose_.orientation.y;
    pose.orientation.z = pose_.orientation.z;
    pose.orientation.w = pose_.orientation.w;

    geometry_msgs::Twist twist;
    twist.linear.x = 0.0;
    twist.linear.y = 0.0;
    twist.linear.z = 0.0;
    twist.angular.x = 0.0;
    twist.angular.y = 0.0;
    twist.angular.z = 0.0;

    gazebo_msgs::ModelState modelstate;
    modelstate.model_name = (std::string) "Marker";
    modelstate.reference_frame = (std::string) "world";
    modelstate.pose = pose;
    modelstate.twist = twist;

    ros::ServiceClient client = nh.serviceClient<gazebo_msgs::SetModelState>("/gazebo/set_model_state");
    gazebo_msgs::SetModelState setmodelstate;
    setmodelstate.request.model_state = modelstate;
    client.call(setmodelstate);
}