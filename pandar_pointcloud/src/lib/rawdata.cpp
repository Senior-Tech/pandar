//
// Created by gld on 2020/5/4.
//
#include "rawdata.h"

#include <fstream>
#include <math.h>
#include <ros/ros.h>
#include <ros/package.h>
#include <angles/angles.h>


namespace pandar_rawdata{

    inline float SQR(float val) {return val*val; }

    inline double degreeToRadian(double degree) { return degree * M_PI / 180; }

    RawData::RawData() {
    }

    void RawData::setParameters(double min_range, double max_range, double view_directoin, double view_width) {
        config_.min_range = min_range;
        config_.max_range = max_range;

        // TODO: min_angle and max_angle
        config_.min_range = 0.0;
        config_.max_range = 36000;
    }


    int RawData::scansPerPacket() const {
        if(calibration_.num_lasers == 40){
            return BLOCKS_PER_PACKET * SCANS_PER_BLOCK;
        }
    }

    /// setup for online operation
    boost::optional<pandar_pointcloud::Calibration> RawData::setup(ros::NodeHandle private_nh) {

    }


    /// setup for offline operation
    int RawData::setupOffline(std::string calibration_file, double max_range_, double min_range_) {
        config_.max_range = max_range_;
        config_.min_range = min_range_;

        config_.calibrationFile = calibration_file;

        calibration_.initParams(calibration_file);

        for(uint16_t rot_index = 0; rot_index < ROTATION_MAX_UINTS; ++rot_index){
            float rotation = angles::from_degrees(ROTATION_RESOLUTION * rot_index);
            cos_rot_table_[rot_index] = cosf(rotation);
            sin_rot_table_[rot_index] = sinf(rotation);
        }
        return 0;
    }

    void RawData::unpack(const pandar_msgs::Pandar40pPacket &pkt, pandar_rawdata::DataContainerBase &data, const ros::Time& scan_start_time) {

        if(calibration_.num_lasers == 40){
            unpack_pandar40p(pkt, data, scan_start_time);
            return;
        }

    }

    void RawData::unpack_pandar40p(const pandar_msgs::Pandar40pPacket &pkt, pandar_rawdata::DataContainerBase &data, const ros::Time& scan_start_time) {

        const raw_packet_t *raw = (const raw_packet_t*) &pkt.data[0];

//        double time_diff_start_to_this_packet = (pkt.stamp - scan_start_time).toSec();

        for(int i = 0; i < BLOCKS_PER_PACKET; ++i){      /// 10
            for(int j = 0, k = 0; j < SCANS_PER_BLOCK; ++j, k += RAW_SCAN_SIZE){
                float x, y, z;
                float intensity;
                const uint8_t laser_number = j;

                /// correction
                const raw_block_t &block = raw->blocks[i];
                union two_bytes tmp;
                tmp.bytes[0] = block.data[k];
                tmp.bytes[1] = block.data[k+1];

                if(tmp.bytes[0] == 0 && tmp.bytes[1] == 0){    /// no laser beam return
                    continue;
                }

                double distance = tmp.uint * RANGE_RESOLUTION_M;
                intensity = (raw->blocks[i].data[k + 2] & 0xff);

                double angle_azimuth = (static_cast<double>(block.rotation) / 100.0) + calibration_.horizatal_azimuth_offset_map_[j];

                double distance_xy = distance * cosf(degreeToRadian(calibration_.elev_angle_map_[laser_number]));
                x = static_cast<float>(distance_xy * sinf(degreeToRadian(angle_azimuth)));
                y = static_cast<float>(distance_xy * cosf(degreeToRadian(angle_azimuth)));
                z = static_cast<float>(distance * sinf(degreeToRadian(calibration_.elev_angle_map_[laser_number])));

                // TODO intensity 校准
                if((distance == 0x010101 && intensity == 0x0101) ||
                    distance > (200 * 1000 / 2)){
                    distance = 0;
                    intensity = 0;
                }

                data.addPoint(x, y, z, 39-laser_number, raw->blocks[i].rotation, distance, intensity, 0.0);
            }
            data.newLine();
        }
    }
}








