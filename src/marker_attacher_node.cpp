// Node file to create object and initialising the ROS node
#include "marker_attacher.hpp" 

int main(int argc, char** argv)
{
    /* initialising the ROS node creating node handle
    for regestring it to the master and then private node handle to
    handle the parameters */
    ros::init(argc, argv, "marker_attacher"); 
    ros::NodeHandle nh;
    ros::NodeHandle pnh("~"); 
    
    MarkerAttacher node(nh,pnh); // Creating the object

    ros::Rate rate(100); // Defing the looping rate

    /* Looking for any interupt else it will continue looping */
    while (ros::ok())
    {   
        // node.runOnce();
        node.controlLoop(nh);
        ros::spinOnce();
        rate.sleep();
    }
    return 0;
}