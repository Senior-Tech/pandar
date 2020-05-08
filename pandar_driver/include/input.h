//
// Created by gld on 2020/5/4.
//

#ifndef PANDAR40P_PACKETS_INPUT_H
#define PANDAR40P_PACKETS_INPUT_H

#include <unistd.h>
#include <stdio.h>
#include <pcap.h>
#include <netinet/in.h>
#include <string>

#include <ros/ros.h>
#include <pandar_msgs/Pandar40pPacket.h>

namespace pandar_driver{
    static uint16_t  DATA_PORT_NUMBER = 2368;
    static uint16_t POSITION_PORT_NUMBER = 8308;

    class Input{
    public:
        Input(ros::NodeHandle private_nh, uint16_t port);
        virtual  ~Input(){}

        virtual int getPacket(pandar_msgs::Pandar40pPacket *pkt,
                              const double time_offset) = 0;

    protected:
        ros::NodeHandle private_nh_;
        uint16_t  port_;
        std::string devip_str_;
        bool gps_time_;
    };


    class InputSocket : public Input{
    public:
        InputSocket(ros::NodeHandle private_n, uint16_t port = DATA_PORT_NUMBER);
        virtual ~InputSocket();
        virtual int getPacket(pandar_msgs::Pandar40pPacket *pkt,
                              const double time_offset);
        void setDeviceIP(const std::string& ip);

    private:
        int sockfd_;
        in_addr devip_;
    };


    class InputPCAP : public Input{
    public:
        InputPCAP(ros::NodeHandle private_nh,
                  uint16_t port = DATA_PORT_NUMBER,
                  double packet_rate = 0.0,
                  std::string filename = "",
                  bool read_once = false,
                  bool read_fast = false,
                  double repeat_delay = 0.0);
        virtual ~InputPCAP();
        virtual int getPacket(pandar_msgs::Pandar40pPacket* pkt,
                              const double time_offset);
        void setDeviceIP(const std::string& ip);

    private:
        ros::Rate packet_rate_;
        std::string filename_;
        pcap_t *pcap_;
        bpf_program pcap_packet_filter_;
        char errbuf_[PCAP_ERRBUF_SIZE];
        bool empty_;
        bool read_once_;
        bool read_fast_;
        double repeat_delay_;
    };
}

#endif //PANDAR40P_PACKETS_INPUT_H
