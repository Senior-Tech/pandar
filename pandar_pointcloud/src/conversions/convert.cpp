//
// Created by gld on 2020/5/4.
//

#include "convert.h"
#include <pointcloudXYZIR.h>
#include <ros/package.h>

namespace pandar_pointcloud{

    Convert::Convert(ros::NodeHandle node, ros::NodeHandle private_nh, const std::string &node_name)
    :data_(new pandar_rawdata::RawData()), first_rcfg_call(true), diagnostics_(node, private_nh, node_name){

        if (!private_nh.getParam("calibration", config_.calibrationfile))
        {
            ROS_ERROR_STREAM("No calibration angles specified! Using test values!");

            // have to use something: grab unit test version as a default
            std::string pkgPath = ros::package::getPath("pandar_pointcloud");
//            config_.calibrationfile = pkgPath + "/params/64e_utexas.yaml";
            config_.calibrationfile = pkgPath + "/params/64e_utexas.yaml";
        }

        double max_range;
        double min_range;
        if(!private_nh.getParam("max_range", max_range)){
            max_range = 200.0;
        }
        if(!private_nh.getParam("min_range", min_range)){
            min_range = 1.0;
        }

        data_->setupOffline(config_.calibrationfile, max_range, min_range);

        container_ptr_ = boost::shared_ptr<PointcloudXYZIR>(
                new PointcloudXYZIR(config_.max_range,
                        config_.min_range,
                        config_.target_frame,
                        config_.fixed_frame,
                        data_->scansPerPacket()));

        output_ = node.advertise<sensor_msgs::PointCloud2>("pandar_points", 10);
        srv_ = boost::make_shared<dynamic_reconfigure::Server<pandar_pointcloud::CloudNodeConfig>> (private_nh);
        dynamic_reconfigure::Server<pandar_pointcloud::CloudNodeConfig>::CallbackType f;
        f = boost::bind(&Convert::callback, this, _1, _2);
        srv_->setCallback(f);

        ROS_INFO("pandar_packets_sub");
        pandar_scan_ = node.subscribe("pandar_packets", 10, &Convert::processScan,
                                        (Convert *)this, ros::TransportHints().tcpNoDelay(true));

        diag_min_freq_ = 2.0;
        diag_max_freq_ = 20.0;
        using namespace diagnostic_updater;
        diag_topic_.reset(new TopicDiagnostic("pandar_points", diagnostics_,
                FrequencyStatusParam(&diag_min_freq_, &diag_max_freq_, 0.1, 10),
                TimeStampStatusParam()));
    }

    void Convert::callback(pandar_pointcloud::CloudNodeConfig &config, uint32_t level) {
        ROS_INFO("Reconfigure Request");
        data_->setParameters(config.min_range, config.max_range, config.view_direction,
                             config.view_width);
        config_.fixed_frame = config.fixed_frame;
        config_.target_frame = config.target_frame;
        config_.min_range = config.min_range;
        config_.max_range = config.max_range;

        if(first_rcfg_call || config.organize_cloud != config_.organize_cloud){
            first_rcfg_call = false;
            config_.organize_cloud = config.organize_cloud;

            container_ptr_ = boost::shared_ptr<PointcloudXYZIR>(
                        new PointcloudXYZIR(config_.max_range, config_.min_range,
                                            config_.target_frame, config_.fixed_frame,
                                            data_->scansPerPacket()));

        }

        container_ptr_->configure(config_.max_range, config_.min_range, config_.fixed_frame, config_.target_frame);
    }

    void Convert::processScan(const pandar_msgs::PandarScan::ConstPtr &scanMsg) {
        if (output_.getNumSubscribers() == 0)         // no one listening?
            return;                                     // avoid much work
        boost::lock_guard<boost::mutex> guard(reconfigure_mtx_);
        // allocate a point cloud with same time and frame ID as raw data
        container_ptr_->setup(scanMsg);

        // process each packet provided by the driver
        for (size_t i = 0; i < scanMsg->packets.size(); ++i) {
//            container_ptr_->computeTransformation(scanMsg->packets[i].stamp);
            data_->unpack(scanMsg->packets[i], *container_ptr_, scanMsg->header.stamp);
        }

        // publish the accumulated cloud message
        diag_topic_->tick(scanMsg->header.stamp);
        diagnostics_.update();
        output_.publish(container_ptr_->finishCloud());
    }
}





