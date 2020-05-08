//
// Created by gld on 2020/5/4.
//

#ifndef PANDAR_POINTCLOUD_POINTCLOUDXYZIR_H
#define PANDAR_POINTCLOUD_POINTCLOUDXYZIR_H

#include <datacontainerbase.h>
#include <string>

namespace pandar_pointcloud
{
    class PointcloudXYZIR : public pandar_rawdata::DataContainerBase
    {
    public:
        PointcloudXYZIR(const double max_range, const double min_range, const std::string& target_frame,
                        const std::string& fixed_frame, const unsigned int scans_per_block,
                        boost::shared_ptr<tf::TransformListener> tf_ptr = boost::shared_ptr<tf::TransformListener>());

        virtual void newLine();

        virtual void setup(const pandar_msgs::PandarScan::ConstPtr& scan_msg);

        virtual void addPoint(float x, float y, float z, uint16_t ring, uint16_t azimuth, float distance, float intensity, float time);

        sensor_msgs::PointCloud2Iterator<float> iter_x, iter_y, iter_z, iter_intensity, iter_distance, iter_time;
        sensor_msgs::PointCloud2Iterator<uint16_t> iter_ring, iter_azimuth;
    };
}  // namespace pandar_pointcloud


#endif //PANDAR_POINTCLOUD_POINTCLOUDXYZIR_H
