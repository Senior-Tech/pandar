//
// Created by gld on 2020/5/4.
//

#include <string>
#include <cmath>
#include <ros/ros.h>
#include <tf/transform_listener.h>
#include <pandar_msgs/Pandar40pPacket.h>
#include <pandar_msgs/PandarScan.h>
#include <driver.h>

#include "driver.h"

namespace pandar_driver{
    PandarDriver::PandarDriver(ros::NodeHandle node,
            ros::NodeHandle private_nh,
            const std::string &node_name)
            : diagnostics_(node, private_nh, node_name){
        /// use private_node handle to get parameters
        private_nh.param("frame_id", config_.frame_id, std::string("pandar"));
        std::string tf_prefix = tf::getPrefixParam(private_nh);
        ROS_DEBUG_STREAM("tf_prefix" << tf_prefix);
        config_.frame_id = tf::resolve(tf_prefix, config_.frame_id);

        /// get model name. validate string, determine packet rate
        private_nh.param("model", config_.model, std::string("pandar40p"));
        double packet_rate;
        std::string model_full_name;
        if(config_.model == "pandar40p"){
            packet_rate = 1800;    // 单次回波，每秒720000个点， 一个package里有 10 * ((124 - 4) /3) = 400个点，  多5个包
            model_full_name = "pandar40p";
        }
        else if(config_.model == "pandar40m"){
            packet_rate = 1800;    // 单次回波，每秒720000个点， 一个package里有 10 * ((124 - 4) /3) = 400个点，  多5个包
            model_full_name = "pandar40m";
        }
        else{
            ROS_ERROR_STREAM("unknown pandar lidar_model: " << config_.model);
            packet_rate = 1800.0;
        }

        std::string deviceName(model_full_name);
        private_nh.param("rpm", config_.rpm, 600.0);
        ROS_INFO_STREAM(deviceName << "rotating at " << config_.rpm << " RPM");
        double frequency = (config_.rpm / 60.0);

        //// default number of packets for each scan is a single revolution
        /// (fractions rounded up) 分数四舍五入
        config_.npackets = (int)ceil(packet_rate / frequency);
        private_nh.getParam("npackets", config_.npackets);
        ROS_INFO_STREAM("publishing " << config_.npackets << " packets per scan");

        /// if we are timestamping based on the first or last packet in the scan
        private_nh.param("timestamp_first_packet", config_.timestamp_first_packet, false);
        if(config_.timestamp_first_packet)
            ROS_INFO("Setting pandar scan start time to timestamp of first packet");

        std::string dump_file;
        private_nh.param("pcap", dump_file, std::string(""));
        double cut_angle;
        private_nh.param("cut_angle", cut_angle, -0.01);
        if(cut_angle < 0.0){
            ROS_INFO_STREAM("Cut at specific angle feature deactivated.");
        }else if(cut_angle < (2 * M_PI)){
            ROS_INFO_STREAM("Cut at specific angle feature activated."
                            "cutting pandar points always at " << cut_angle << " rad.");
        }else{
            ROS_ERROR_STREAM("cut_angle parameter is out of range. Allowed range is "
            << "bewteen 0.0 and 2*PI or negative values to deactivate this feature.");
            cut_angle = -0.01;
        }

        // Convert cut_angle from radian to one-hundredth degree,
        // which is used in Pandar packets
        config_.cut_angle = int((cut_angle*360/(2*M_PI))*100);

        int udp_port;
        private_nh.param("port", udp_port, (int)DATA_PORT_NUMBER);

        /// Initialize dynamic reconfigure
        srv_ = boost::make_shared<dynamic_reconfigure::Server<pandar_driver::PandarNodeConfig>> (private_nh);
        dynamic_reconfigure::Server<pandar_driver::PandarNodeConfig>::CallbackType f;
        f = boost::bind(&PandarDriver::callback, this, _1, _2);
        srv_->setCallback(f);

        /// initialize diagnostics
        diagnostics_.setHardwareID(deviceName);
        const double diag_freq = packet_rate / config_.npackets;
        diag_max_freq_ = diag_freq;
        diag_min_freq_ = diag_freq;
        ROS_INFO("excepted frequency: %.3f (Hz)", diag_freq);

        using namespace diagnostic_updater;
        diag_topic_.reset(new TopicDiagnostic("pandar_packets", diagnostics_,
                FrequencyStatusParam(&diag_min_freq_, &diag_max_freq_, 0.1, 10),
                TimeStampStatusParam()));
        diag_timer_ = private_nh.createTimer(ros::Duration(0.2), &PandarDriver::diagTimerCallback, this);

        config_.enabled = true;
        if(dump_file != ""){
            input_.reset(new pandar_driver::InputPCAP(private_nh, udp_port, packet_rate, dump_file));
        }
        else{
            input_.reset(new pandar_driver::InputSocket(private_nh, udp_port));
        }

        /// raw packet output topic
        output_ = node.advertise<pandar_msgs::PandarScan>("pandar_packets", 10);
        last_azimuth_ = -1;
    }

    /* poll the device
     * */
    bool PandarDriver::poll(void){
        if(!config_.enabled){
            /// If we are not enabled exit once a second to let the caller handle
            /// anything it might need to, such as if it needs to exit.
            ros::Duration(1).sleep();
            return true;
        }
        /// Allocate a new shared pointer for zero-copy sharing with other nodelets.
        pandar_msgs::PandarScanPtr scan(new pandar_msgs::PandarScan);
        if(config_.cut_angle >= 0){    /// cut at specific angle feature enabled

        }else{     /// standard behaviour
            /// since the pandar delivers data at a very high rate, keep reading and publishing scans as fast as possible
            scan->packets.resize(config_.npackets);
            for(int i = 0; i < config_.npackets; ++i){
                while(true){
                    /// keep reading until full packet received
                    int rc = input_->getPacket(&scan->packets[i], config_.time_offset);
                    if(rc == 0) break;    /// got a full packet?
                    if(rc < 0) return false;    /// end of file reached
                }
            }
        }

        /// publish message using time of last packet read
        ROS_DEBUG("Publishing a fulll Pandar scan.");

        if(config_.timestamp_first_packet){
            scan->header.stamp = scan->packets.front().stamp;
        }
        else{
            scan->header.stamp = scan->packets.back().stamp;
        }

        // ROS_INFO_STREAM("scan size: " << scan->packets.size());

        scan->header.frame_id = config_.frame_id;
        output_.publish(scan);

        /// notify diagnostics that a message has benn published, updating its status
        diag_topic_->tick(scan->header.stamp);
        diagnostics_.update();

        return true;
    }

    void PandarDriver::callback(pandar_driver::PandarNodeConfig &config, uint32_t level) {
        ROS_INFO("Reconfigure Requeset");
        if(level & 1){
            config_.time_offset = config.time_offset;
        }
        if(level & 2){
            config_.enabled = config.enabled;
        }
    }

    void PandarDriver::diagTimerCallback(const ros::TimerEvent &event) {
        (void)event;
        diagnostics_.update();
    }
}




