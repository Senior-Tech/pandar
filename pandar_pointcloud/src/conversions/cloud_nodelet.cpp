//
// Created by gld on 2020/5/4.
//

#include <ros/ros.h>
#include <pluginlib/class_list_macros.h>
#include <nodelet/nodelet.h>

#include "convert.h"

namespace pandar_pointcloud
{
    class CloudNodelet: public nodelet::Nodelet
    {
    public:

        CloudNodelet() {}
        ~CloudNodelet() {}

    private:

        virtual void onInit();
        boost::shared_ptr<Convert> conv_;
    };

    /** @brief Nodelet initialization. */
    void CloudNodelet::onInit()
    {
        conv_.reset(new Convert(getNodeHandle(), getPrivateNodeHandle(), getName()));
    }

} // namespace pandar_pointcloud


// Register this plugin with pluginlib.  Names must match nodelets.xml.
//
// parameters: class type, base class type
PLUGINLIB_EXPORT_CLASS(pandar_pointcloud::CloudNodelet, nodelet::Nodelet)

