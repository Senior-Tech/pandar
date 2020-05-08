//
// Created by gld on 2020/5/4.
//

#ifndef PANDAR40P_PACKETS_DRIVER_H
#define PANDAR40P_PACKETS_DRIVER_H

#include <string>
#include <ros/ros.h>
#include <diagnostic_updater/diagnostic_updater.h>
#include <diagnostic_updater/publisher.h>
#include <dynamic_reconfigure/server.h>

#include <input.h>
#include <pandar_driver/PandarNodeConfig.h>

namespace pandar_driver{

    class PandarDriver{
    public:
        PandarDriver(ros::NodeHandle node,
                     ros::NodeHandle private_nh,
                     std::string const &node_name = ros::this_node::getName());

        ~PandarDriver() {}

        bool poll(void);

    private:
        /// callback for dynamic reconfigure
        void callback(pandar_driver::PandarNodeConfig &config, uint32_t level);
        /// callback for diagnostics update for lost communication with pandar lidar
        void diagTimerCallback(const ros::TimerEvent &event);
        /// pointer to dynamic reconfigure service srv_
        boost::shared_ptr<dynamic_reconfigure::Server<pandar_driver::PandarNodeConfig>> srv_;

        /// configuration parameters
        struct{
            std::string frame_id;   /// tf frame ID
            std::string model;      /// device model name
            int npackets;           /// number of packets to collect
            double rpm;             /// device rotation rate (RPMS)
            int cut_angle;          /// cutting angle in 1 / 100度
            double time_offset;     /// time in seconds added to each pandar time stamp
            bool enabled;           /// polling is enabled
            bool timestamp_first_packet;
        }config_;

        boost::shared_ptr<Input> input_;
        ros::Publisher output_;
        int last_azimuth_;

        /// diagnostics updater
        ros::Timer diag_timer_;
        diagnostic_updater::Updater diagnostics_;
        double diag_min_freq_;
        double diag_max_freq_;
        boost::shared_ptr<diagnostic_updater::TopicDiagnostic> diag_topic_;
    };
}
#endif //PANDAR40P_PACKETS_DRIVER_H
