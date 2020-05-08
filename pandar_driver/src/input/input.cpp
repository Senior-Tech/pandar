//
// Created by gld on 2020/5/4.
//

#include <unistd.h>
#include <string>
#include <sstream>
#include <sys/socket.h>
#include <arpa/inet.h>
#include <poll.h>
#include <errno.h>
#include <fcntl.h>
#include <sys/file.h>
#include <input.h>
#include <time_conversion.h>

namespace pandar_driver{

    static const size_t packet_size = sizeof(pandar_msgs::Pandar40pPacket().data);

    Input::Input(ros::NodeHandle private_nh, uint16_t port) : private_nh_(private_nh), port_(port){
        private_nh.param("device_ip", devip_str_, std::string(""));
        private_nh.param("gps_time", gps_time_, false);
        if(!devip_str_.empty())
            ROS_INFO_STREAM("Only accepting packets from IP address" << devip_str_);
    }

    InputSocket::InputSocket(ros::NodeHandle private_nh, uint16_t port) : Input(private_nh, port){
        sockfd_ = -1;
        if(!devip_str_.empty()){
            inet_aton(devip_str_.c_str(), &devip_);
        }

        /// connect to Pandar UDP port
        ROS_INFO_STREAM("Opening UDP socket: port " << port);
        sockfd_ = socket(PF_INET, SOCK_DGRAM, 0);
        if(sockfd_ == -1){
            perror("socket");
            return;
        }
        sockaddr_in my_addr;
        memset(&my_addr, 0, sizeof(my_addr));
        my_addr.sin_family = AF_INET;
        my_addr.sin_port = htons(port);
        my_addr.sin_addr.s_addr = INADDR_ANY;

        if(bind(sockfd_, (sockaddr *)&my_addr, sizeof(sockaddr)) == -1){
            perror("bind");
            return;
        }
        if(fcntl(sockfd_, F_SETFL, O_NONBLOCK | FASYNC) < 0){
            perror("non-block");
            return;
        }
        ROS_DEBUG("Pandar socket fd is %d\n", sockfd_);
    }

    InputSocket::~InputSocket(void) {
        (void) close(sockfd_);
    }

    int InputSocket::getPacket(pandar_msgs::Pandar40pPacket *pkt, const double time_offset) {
        double time1 = ros::Time::now().toSec();

        struct pollfd fds[1];
        fds[0].fd = sockfd_;
        fds[0].events = POLLIN;
        static const int POLL_TIMEOUT = 1000; /// one second (in msec)

        sockaddr_in sender_address;
        socklen_t  sender_address_len = sizeof(sender_address);

        while(true){
            do{
                int retval = poll(fds, 1, POLL_TIMEOUT);
                if(retval < 0){
                    if(errno != EINTR)
                        ROS_ERROR("poll() error: %s", strerror(errno));
                    return -1;
                }
                if(retval == 0){
                    ROS_WARN("pandar poll() timeout");
                    return -1;
                }
                if((fds[0].revents & POLLERR)
                   || (fds[0].revents & POLLHUP)
                   || (fds[0].revents & POLLNVAL))
                {
                    ROS_ERROR("poll() reports pandar error");
                    return -1;
                }
            }while((fds[0].revents & POLLIN) == 0);

            ssize_t  nbytes = recvfrom(sockfd_, &pkt->data[0],
                                       packet_size, 0,
                                       (sockaddr*)&sender_address,
                                       &sender_address_len);



            if(nbytes < 0){
                if(errno != EWOULDBLOCK){
                    perror("recvfail");
                    ROS_INFO("recvfail");
                    return -1;
                }
            }
            else if((size_t) nbytes == packet_size){
                /// read successful,
                /// if packet is not from the lidar scanner we selected by IP,
                /// continue otherwise we are done
                if(devip_str_ != ""
                   && sender_address.sin_addr.s_addr != devip_.s_addr )
                    continue;
                else
                    break;    /// done
            }
            ROS_DEBUG_STREAM("incomplete pandar packet read: " << nbytes << " bytes");
        }
        if(!gps_time_){
            double time2 = ros::Time::now().toSec();
            pkt->stamp = ros::Time((time2 +time1) / 2.0 + time_offset);
        }else{
            /// time for each packet is a 4 bytes uint located starting at offset 1250 in the data packet
            pkt->stamp = rosTimeFromGpsTimestamp(&(pkt->data[1250]));
        }
        return 0;
    }

    InputPCAP::InputPCAP(ros::NodeHandle private_nh, uint16_t port, double packet_rate, std::string filename,
                         bool read_once, bool read_fast, double repeat_delay) :
                         Input(private_nh, port),
                         packet_rate_(packet_rate),
                         filename_(filename){
        pcap_ = NULL;
        empty_ = true;

        /// get parameters using private node handle
        private_nh.param("read_once", read_once_, false);
        private_nh.param("read_fast", read_fast_, false);
        private_nh.param("repeat_delay", repeat_delay_, 0.0);

        if(read_once_)
            ROS_INFO("Read input file only once");
        if(read_fast_)
            ROS_INFO("Read input filse as quickly as possible.");
        if(repeat_delay_ > 0.0)
            ROS_INFO("Delay %.3f seconds before repeating input file.", repeat_delay_);

        /// Open the PCAP dump file
        ROS_INFO("Opening PCAP file \"%s\"", filename_.c_str());
        if ((pcap_ = pcap_open_offline(filename_.c_str(), errbuf_) ) == NULL){
            ROS_FATAL("Error opening socket dump file.");
            return;
        }

        std::stringstream filter;
        if(devip_str_ != ""){
            filter << "src host " << devip_str_ << " && ";
        }
        filter << "udp dst port " << port;
        pcap_compile(pcap_, &pcap_packet_filter_, filter.str().c_str(), 1, PCAP_NETMASK_UNKNOWN);
    }

    InputPCAP::~InputPCAP(void) {
        pcap_close(pcap_);
    }

    int InputPCAP::getPacket(pandar_msgs::Pandar40pPacket *pkt, const double time_offset) {
        struct pcap_pkthdr *header;
        const u_char *pkt_data;

        while (true) {
            int res;
            if ((res = pcap_next_ex(pcap_, &header, &pkt_data)) >= 0) {
                if (0 == pcap_offline_filter(&pcap_packet_filter_, header, pkt_data))
                    continue;

                /// keep the reader from blowing through the file.
                if (read_fast_ == false)
                    packet_rate_.sleep();
                memcpy(&pkt->data[0], pkt_data + 42, packet_size);
                pkt->stamp = ros::Time::now();  /// time_offset not considered here, as no synchronization required
                empty_ = false;
                return 0;
            }
            if (empty_) {
                ROS_WARN("Error %d reading pandar packet: %s", res, pcap_geterr(pcap_));
                return -1;
            }
            if (read_once_) {
                ROS_INFO("end_of file reached -- done reading.");
                return -1;
            }
            if (repeat_delay_ > 0.0) {
                ROS_INFO("end of file reached -- delaying %.3f seconds.", repeat_delay_);
                usleep(rint(repeat_delay_ * 1000000.0));
            }
            ROS_DEBUG("repeating pandar dump file");

            pcap_close(pcap_);
            pcap_ = pcap_open_offline(filename_.c_str(), errbuf_);
            empty_ = true;
        }
    }
}
