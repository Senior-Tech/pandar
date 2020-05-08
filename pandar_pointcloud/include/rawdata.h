//
// Created by gld on 2020/5/4.
//

#ifndef PANDAR_POINTCLOUD_RAWDATA_H
#define PANDAR_POINTCLOUD_RAWDATA_H

#include <errno.h>
#include <stdint.h>
#include <string>
#include <boost/format.hpp>
#include <math.h>

#include <ros/ros.h>
#include <pandar_msgs/PandarScan.h>
#include <calibration.h>
#include <datacontainerbase.h>

namespace pandar_rawdata{

    static const float RANGE_RESOLUTION_M = 0.004;

    static const float ROTATION_RESOLUTION = 0.01f;
    static const uint16_t ROTATION_MAX_UINTS = 36000u;

    static const int LASER_NUMBER = 40;
    static const int RAW_SCAN_SIZE = 3;
    static const int SCANS_PER_BLOCK = 40;
    static const int BLOCK_DATA_SIZE = (SCANS_PER_BLOCK * RAW_SCAN_SIZE);   /// 120

    static const uint16_t UPPER_BANK = 0xeeff;

    static const int PACKET_SIZE = 1266;
    static const int BLOCKS_PER_PACKET = 10;
//    static const int PACKET_STATUS_SIZE = 4 +2; ???
    static const int SCANS_PER_PACKET = (SCANS_PER_BLOCK * BLOCKS_PER_PACKET); /// 400

    union two_bytes{
        uint16_t uint;
        uint8_t bytes[2];
    };

    typedef struct raw_block{
        uint16_t header;
        uint16_t rotation;
        uint8_t data[BLOCK_DATA_SIZE];            // 120
    }raw_block_t;

    typedef struct raw_packet{
        raw_block_t blocks[BLOCKS_PER_PACKET];    // 10
        uint8_t reserved[5];
        uint8_t temperature;
        uint16_t reserved2;
        uint16_t speed;
        uint32_t gps_time;
        uint8_t reflector;
        uint8_t Manufacturer;
        uint8_t utc[6];
//        uint16_t revolution;
//        uint8_t status[4];
    }raw_packet_t;


    class RawData{
    public:
        RawData();
        ~RawData(){

        }

        boost::optional<pandar_pointcloud::Calibration> setup(ros::NodeHandle private_nh);

        int setupOffline(std::string calibration_file, double max_range_, double min_range_);

        void unpack(const pandar_msgs::Pandar40pPacket& pkt, DataContainerBase& data, const ros::Time& scan_start_time);

        void setParameters(double min_range, double max_range, double view_directoin, double view_width);

        int scansPerPacket() const;

    private:
        typedef struct{
            std::string calibrationFile;
            double max_range;
            double min_range;
            double max_angle;
            double min_angle;
        }Config;
        Config config_;

        /// calibration file
        pandar_pointcloud::Calibration calibration_;
        float sin_rot_table_[ROTATION_MAX_UINTS];
        float cos_rot_table_[ROTATION_MAX_UINTS];

        void unpack_pandar40p(const pandar_msgs::Pandar40pPacket& pkt, DataContainerBase& data, const ros::Time& scan_start_time);
    };





    ///// pandar40p_sdk
    struct Pandar40PUnit_s {
        uint8_t intensity;
        double distance;
    };
    typedef struct Pandar40PUnit_s Pandar40PUnit;

    struct Pandar40PBlock_s {
        uint16_t azimuth;
        uint16_t sob;
        Pandar40PUnit units[LASER_NUMBER];
    };
    typedef struct Pandar40PBlock_s Pandar40PBlock;

    struct Pandar40PPacket_s {
        Pandar40PBlock blocks[BLOCKS_PER_PACKET];
        struct tm t;
        uint32_t usec;
        int echo;
    };
    typedef struct Pandar40PPacket_s Pandar40PPacket;




}



#endif //PANDAR_POINTCLOUD_RAWDATA_H
