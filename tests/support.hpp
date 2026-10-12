#pragma once

#include "odomcal/diff_drive.hpp"
#include "odomcal/simulator.hpp"

#include <vector>

inline odomcal::DiffDriveParams test_nominal() {
    return odomcal::nominal_params(0.05, 0.30);
}

inline odomcal::SimConfig test_square(odomcal::DiffDriveParams plant, odomcal::Direction direction) {
    odomcal::SimConfig config;
    config.plant = plant;
    config.odometry = test_nominal();
    config.command_radius_m = 0.05;
    config.command_baseline_m = 0.30;
    config.side_m = 4.0;
    config.steps_per_segment = 8;
    config.noise_std_rad = 0.0;
    config.seed = 1;
    config.direction = direction;
    return config;
}

inline std::vector<odomcal::Pose> repeat_pose(const odomcal::Pose& pose, int copies) {
    return std::vector<odomcal::Pose>(static_cast<std::size_t>(copies), pose);
}
