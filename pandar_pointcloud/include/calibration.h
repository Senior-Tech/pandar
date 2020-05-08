//
// Created by gld on 2020/5/4.
//

#ifndef PANDAR_POINTCLOUD_CALIBRATION_H
#define PANDAR_POINTCLOUD_CALIBRATION_H

#include <map>
#include <vector>
#include <string>


namespace pandar_pointcloud{

// elevation angle of each line for HS Line 40 Lidar, Line 1 - Line 40
//    static const float pandar40p_elev_angle_map[] = {
//            6.96,   5.976,  4.988,   3.996,   2.999,   2.001,  1.667,   1.333,
//            1.001,  0.667,  0.333,   0,       -0.334,  -0.667, -1.001,  -1.334,
//            -1.667, -2.001, -2.331,  -2.667,  -3,      -3.327, -3.663,  -3.996,
//            -4.321, -4.657, -4.986,  -5.311,  -5.647,  -5.974, -6.957,  -7.934,
//            -8.908, -9.871, -10.826, -11.772, -12.705, -13.63, -14.543, -15.444};
//
//// Line 40 Lidar azimuth Horizatal offset ,  Line 1 - Line 40
//    static const float pandar40p_horizatal_azimuth_offset_map[] = {
//            0.005,  0.006,  0.006,  0.006,  -2.479, -2.479, 2.491,  -4.953,
//            -2.479, 2.492,  -4.953, -2.479, 2.492,  -4.953, 0.007,  2.491,
//            -4.953, 0.006,  4.961,  -2.479, 0.006,  4.96,   -2.478, 0.006,
//            4.958,  -2.478, 2.488,  4.956,  -2.477, 2.487,  2.485,  2.483,
//            0.004,  0.004,  0.003,  0.003,  -2.466, -2.463, -2.46,  -2.457};

    static const float pandar40p_elev_angle_map[] = {
            15.00, 11.00, 8.00, 5.00, 3.00, 2.00, 1.67, 1.33, 1.00, 0.67,
            0.33, 0.00, -0.33, -0.67, -1.00, -1.33, -1.67, -2.00, -2.33, -2.67,
            -3.00, -3.33, -3.67, -4.00, -4.33, -4.67, -5.00, -5.33, -5.67, -6.00,
            -7.00, -8.00, -9.00, -10.00, -11.00, -12.00, -13.00, -14.00, -19.00, -25.00
    };

    static const float pandar40p_horizatal_azimuth_offset_map[] = {
            -1.042, -1.042, -1.042, -1.042, -1.042, -1.042, 3.125, -5.208, -1.042, 3.125,
            -5.208, -1.042, 3.125, -5.208, -1.042, 3.125, -5.208, -1.042, 3.125, -5.208,
            -1.042, 3.125, -5.208, -1.042, 3.125, -5.208, -1.042, 3.125, -5.208, -1.042,
            -1.042, -1.042, -1.042, -1.042, -1.042, -1.042, -1.042, -1.042, -1.042, -1.042
    };


    struct LaserCorrection
    {
        /** parameters in db.xml */
        float rot_correction;
        float vert_correction;
        float dist_correction;
        bool two_pt_correction_available;
        float dist_correction_x;
        float dist_correction_y;
        float vert_offset_correction;
        float horiz_offset_correction;
        int max_intensity;
        int min_intensity;
        float focal_distance;
        float focal_slope;

        /** cached values calculated when the calibration file is read */
        float cos_rot_correction;              ///< cosine of rot_correction
        float sin_rot_correction;              ///< sine of rot_correction
        float cos_vert_correction;             ///< cosine of vert_correction
        float sin_vert_correction;             ///< sine of vert_correction

        int laser_ring;                        ///< ring number for this laser
    };

/** \brief Calibration information for the entire device. */
    class Calibration
    {
    public:
        float distance_resolution_m;
        std::map<int, LaserCorrection> laser_corrections_map;
        std::vector<LaserCorrection> laser_corrections;
        int num_lasers;
        bool initialized;
        bool ros_info;


        //// pandar
        float elev_angle_map_[40];
        float horizatal_azimuth_offset_map_[40];
        float blockOffset_[10];
        float laserOffset_[40];

    public:
//        Calibration();
        Calibration(){}

        int initParams(const std::string& calibration_file);

        int read(const std::string& calibration_file);
//        void write(const std::string& calibration_file);
    };
}
#endif //PANDAR_POINTCLOUD_CALIBRATION_H
