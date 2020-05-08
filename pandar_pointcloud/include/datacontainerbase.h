//
// Created by gld on 2020/5/4.
//

#ifndef PANDAR_POINTCLOUD_DATACONTAINERBASE_H
#define PANDAR_POINTCLOUD_DATACONTAINERBASE_H

#include <tf/transform_listener.h>
#include <pandar_msgs/PandarScan.h>
#include <sensor_msgs/point_cloud2_iterator.h>
#include <Eigen/Dense>
#include <string>
#include <algorithm>
#include <cstdarg>

namespace pandar_rawdata{

    class DataContainerBase{
    public:
        DataContainerBase(const double max_range, const double min_range, const std::string& target_frame,
                          const std::string& fixed_frame, const unsigned int init_width, const unsigned int init_height,
                          const bool is_dense, const unsigned int scans_per_packet,
                          boost::shared_ptr<tf::TransformListener>& tf_ptr, int fields, ...) :
                          config_(max_range, min_range, target_frame, fixed_frame, init_width, init_height, is_dense, scans_per_packet), tf_ptr(tf_ptr){
            va_list vl;
            cloud.fields.clear();
            cloud.fields.reserve(fields);
            va_start(vl, fields);
            int offset = 0;
            for(int i = 0; i < fields; ++i){
                std::string name(va_arg(vl, char*));
                int count(va_arg(vl, int));
                int datatype(va_arg(vl, int));
                offset = addPointField(cloud, name, count, datatype, offset);
            }
            va_end(vl);
            cloud.point_step = offset;
            cloud.row_step = init_width * cloud.point_step;
            if(config_.transform && !tf_ptr){
                tf_ptr = boost::shared_ptr<tf::TransformListener>(new tf::TransformListener);
            }
        }

        struct Config{
            double max_range;
            double min_range;
            std::string target_frame;
            std::string fixed_frame;
            unsigned int init_width;
            unsigned int init_height;
            bool is_dense;
            unsigned int scans_per_packet;
            bool transform;

            Config(double max_range, double min_range, std::string target_frame, std::string fixed_frame,
                   unsigned int init_width, unsigned int init_height, bool is_dense, unsigned int scans_per_packet)
                   : max_range(max_range)
                   , min_range(min_range)
                   , target_frame(target_frame)
                   , fixed_frame(fixed_frame)
                   , transform(fixed_frame != target_frame)
                   , init_width(init_width)
                   , init_height(init_height)
                   , is_dense(is_dense)
                   , scans_per_packet(scans_per_packet){
                ROS_INFO_STREAM("Initialized container with "
                                << "min_range: " << min_range << ", max_range: " << max_range
                                << ", target_frame: " << target_frame << ", fixed_frame: " << fixed_frame
                                << ", init_width: " << init_width << ", init_height: " << init_height
                                << ", is_dense: " << is_dense << ", scans_per_packet: " << scans_per_packet);
            }
        };

        virtual void setup(const pandar_msgs::PandarScan::ConstPtr& scan_msg){
            cloud.header = scan_msg->header;
            cloud.data.resize(scan_msg->packets.size() * config_.scans_per_packet * cloud.point_step);
            cloud.width = config_.init_width;
            cloud.height = config_.init_height;
            cloud.is_dense = static_cast<uint8_t>(config_.is_dense);
        }

        virtual void addPoint(float x, float y, float z, const uint16_t ring,
                const uint16_t azimuth, const float distance, const float intensity, const float time) = 0;

        virtual void newLine() = 0;

        const sensor_msgs::PointCloud2& finishCloud(){
            cloud.data.resize(cloud.point_step * cloud.width * cloud.height);
            if(!config_.target_frame.empty()){
                cloud.header.frame_id = config_.target_frame;
            }

//            ROS_ERROR_STREAM("Prepared cloud width " << cloud.height * cloud.width
//                              << "Pandar points, time: " << cloud.header.stamp);

            return cloud;
        }

        void configure(const double max_range, const double min_range,
                const std::string fixed_frame, const std::string target_frame){
            config_.max_range = max_range;
            config_.min_range = min_range;
            config_.fixed_frame = fixed_frame;
            config_.target_frame = target_frame;

            config_.transform = fixed_frame.compare(target_frame) != 0;
            if(config_.transform && !tf_ptr){
                tf_ptr = boost::shared_ptr<tf::TransformListener>(new tf::TransformListener);
            }
        }

        sensor_msgs::PointCloud2 cloud;

        inline void vectorTfToEigen(tf::Vector3& tf_vec, Eigen::Vector3f& eigen_vec){
            eigen_vec(0) = tf_vec[0];
            eigen_vec(1) = tf_vec[1];
            eigen_vec(2) = tf_vec[2];
        }

        inline bool computeTransformation(const ros::Time& time){
            tf::StampedTransform transform;
            try {
                tf_ptr->lookupTransform(config_.target_frame, cloud.header.frame_id, time, transform);
            }
            catch(tf::LookupException& e){
                ROS_ERROR("%s", e.what());
                return false;
            }
            catch (tf::ExtrapolationException& e){
                ROS_ERROR("%s", e.what());
                return false;
            }
            tf::Quaternion quaternion = transform.getRotation();
            Eigen::Quaternionf rotation(quaternion.w(), quaternion.x(), quaternion.y(    ), quaternion.z());

            Eigen::Vector3f eigen_origin;
            vectorTfToEigen(transform.getOrigin(), eigen_origin);
            Eigen::Translation3f translation(eigen_origin);
            transformation = translation * rotation;
            return true;
        }

        inline void transformPoint(float& x, float& y, float& z){
            Eigen::Vector3f p = transformation * Eigen::Vector3f(x, y, z);
            x = p.x();
            y = p.y();
            z = p.z();
        }

        inline bool pointInRange(float range){
            return (range >= config_.min_range && range <= config_.max_range);
        }

    protected:
        Config config_;
        boost::shared_ptr<tf::TransformListener> tf_ptr;
        Eigen::Affine3f transformation;
    };


}





#endif //PANDAR_POINTCLOUD_DATACONTAINERBASE_H
