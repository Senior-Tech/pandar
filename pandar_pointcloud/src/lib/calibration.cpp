//
// Created by gld on 2020/5/4.
//

#include "calibration.h"
#include <sstream>
#include <fstream>
#include <iostream>
#include <ros/ros.h>

namespace pandar_pointcloud{

#define LASER_COUNT 40

    int Calibration::initParams(const std::string &calibration_file) {

        num_lasers  = 40;

        std::ifstream in(calibration_file, std::ifstream::in);
        if(in.is_open()){
            in.close();
            read(calibration_file);
        }else{
            for (int i = 0; i < LASER_COUNT; ++i) {
                /* for all the laser offset */
                elev_angle_map_[i] = pandar40p_elev_angle_map[i];
                horizatal_azimuth_offset_map_[i] =
                        pandar40p_horizatal_azimuth_offset_map[i];
            }

            // init the block time offset, us
            blockOffset_[9] = 55.1f * 0.0 + 45.18f;
            blockOffset_[8] = 55.1f * 1.0 + 45.18f;
            blockOffset_[7] = 55.1f * 2.0 + 45.18f;
            blockOffset_[6] = 55.1f * 3.0 + 45.18f;
            blockOffset_[5] = 55.1f * 4.0 + 45.18f;
            blockOffset_[4] = 55.1f * 5.0 + 45.18f;
            blockOffset_[3] = 55.1f * 6.0 + 45.18f;
            blockOffset_[2] = 55.1f * 7.0 + 45.18f;
            blockOffset_[1] = 55.1f * 8.0 + 45.18f;
            blockOffset_[0] = 55.1f * 9.0 + 45.18f;

            // init the laser shot time offset, us
            laserOffset_[3] = 0.93f * 1.0f;
            laserOffset_[35] = 0.93f * 2.0f;
            laserOffset_[39] = 0.93f * 3.0f;
            laserOffset_[23] = 0.93f * 3.0f + 1.6f * 1.0f;
            laserOffset_[16] = 0.93f * 3.0f + 1.6f * 2.0f;
            laserOffset_[27] = 0.93f * 4.0f + 1.6f * 2.0f;
            laserOffset_[11] = 0.93f * 4.0f + 1.6f * 3.0f;
            laserOffset_[31] = 0.93f * 5.0f + 1.6f * 3.0f;
            laserOffset_[28] = 0.93f * 6.0f + 1.6f * 3.0f;
            laserOffset_[15] = 0.93f * 6.0f + 1.6f * 4.0f;
            laserOffset_[2] = 0.93f * 7.0f + 1.6f * 4.0f;
            laserOffset_[34] = 0.93f * 8.0f + 1.6f * 4.0f;
            laserOffset_[38] = 0.93f * 9.0f + 1.6f * 4.0f;
            laserOffset_[20] = 0.93f * 9.0f + 1.6f * 5.0f;
            laserOffset_[13] = 0.93f * 9.0f + 1.6f * 6.0f;
            laserOffset_[24] = 0.93f * 9.0f + 1.6f * 7.0f;
            laserOffset_[8] = 0.93f * 9.0f + 1.6f * 8.0f;
            laserOffset_[30] = 0.93f * 10.0f + 1.6f * 8.0f;
            laserOffset_[25] = 0.93f * 11.0f + 1.6f * 8.0f;
            laserOffset_[12] = 0.93f * 11.0f + 1.6f * 9.0f;
            laserOffset_[1] = 0.93f * 12.0f + 1.6f * 9.0f;
            laserOffset_[33] = 0.93f * 13.0f + 1.6f * 9.0f;
            laserOffset_[37] = 0.93f * 14.0f + 1.6f * 9.0f;
            laserOffset_[17] = 0.93f * 14.0f + 1.6f * 10.0f;
            laserOffset_[10] = 0.93f * 14.0f + 1.6f * 11.0f;
            laserOffset_[21] = 0.93f * 14.0f + 1.6f * 12.0f;
            laserOffset_[5] = 0.93f * 14.0f + 1.6f * 13.0f;
            laserOffset_[29] = 0.93f * 15.0f + 1.6f * 13.0f;
            laserOffset_[22] = 0.93f * 15.0f + 1.6f * 14.0f;
            laserOffset_[9] = 0.93f * 15.0f + 1.6f * 15.0f;
            laserOffset_[0] = 0.93f * 16.0f + 1.6f * 15.0f;
            laserOffset_[32] = 0.93f * 17.0f + 1.6f * 15.0f;
            laserOffset_[36] = 0.93f * 18.0f + 1.6f * 15.0f;
            laserOffset_[14] = 0.93f * 18.0f + 1.6f * 16.0f;
            laserOffset_[7] = 0.93f * 18.0f + 1.6f * 17.0f;
            laserOffset_[18] = 0.93f * 18.0f + 1.6f * 18.0f;
            laserOffset_[4] = 0.93f * 19.0f + 1.6f * 18.0f;
            laserOffset_[26] = 0.93f * 20.0f + 1.6f * 18.0f;
            laserOffset_[19] = 0.93f * 20.0f + 1.6f * 19.0f;
            laserOffset_[6] = 0.93f * 20.0f + 1.6f * 20.0f;
        }
    }

    int Calibration::read(const std::string &calibration_file) {

        std::istringstream ifs(calibration_file);

        std::string line;
        if (std::getline(ifs, line)) {  // first line "Laser id,Elevation,Azimuth"
//            std::cout << "Parse Lidar Correction..." << std::endl;
        }

        double azimuthOffset[LASER_COUNT];
        double elev_angle[LASER_COUNT];

        int lineCounter = 0;
        while (std::getline(ifs, line)) {
            if (lineCounter++ >= LASER_COUNT) break;

            int lineId = 0;
            double elev, azimuth;

            std::stringstream ss(line);
            std::string subline;
            std::getline(ss, subline, ',');
            std::stringstream(subline) >> lineId;
            std::getline(ss, subline, ',');
            std::stringstream(subline) >> elev;
            std::getline(ss, subline, ',');
            std::stringstream(subline) >> azimuth;

            if (lineId != lineCounter) {
                return -1;
            }

            elev_angle[lineId - 1] = elev;
            azimuthOffset[lineId - 1] = azimuth;
        }

        for (int i = 0; i < LASER_COUNT; ++i) {
            /* for all the laser offset */
            elev_angle_map_[i] = elev_angle[i];
            horizatal_azimuth_offset_map_[i] = azimuthOffset[i];
        }

        return 0;
    }

}
