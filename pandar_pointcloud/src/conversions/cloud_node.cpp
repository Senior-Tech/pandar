//
// Created by gld on 2020/5/4.
//


#include <ros/ros.h>
#include "convert.h"

/** Main node entry point. */
int main(int argc, char **argv)
{
    ros::init(argc, argv, "cloud_node");
    ros::NodeHandle node;
    ros::NodeHandle priv_nh("~");

    // create conversion class, which subscribes to raw data
    pandar_pointcloud::Convert conv(node, priv_nh);

    // handle callbacks until shut down
    ros::spin();

    return 0;
}
