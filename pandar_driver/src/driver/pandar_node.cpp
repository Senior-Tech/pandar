//
// Created by gld on 2020/5/4.
//

#include <ros/ros.h>
#include "driver.h"

int main(int argc, char** argv){
    ros::init(argc, argv, "pandar_node");
    ros::NodeHandle node;
    ros::NodeHandle private_nh("~");

    /// start the driver
    pandar_driver::PandarDriver dvr(node, private_nh);

    /// loop until shut down or end of file
    while(ros::ok()){
        /// poll device until end of file
        bool polled = dvr.poll();
        if(!polled)
            ROS_ERROR_THROTTLE(1.0, "pandar - Failed to poll device");

        ros::spinOnce();
    }
    return 0;
}

