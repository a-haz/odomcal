#pragma once

#include "odomcal/diff_drive.hpp"

#include <vector>

namespace odomcal {

// Minimum bidirectional square runs required by the UMBmark procedure.
constexpr int kMinRunsPerDirection = 5;

// Result of the Borenstein and Feng (1996) bidirectional-square calibration.
// Return-position errors are (actual pose - odometry pose) at the end of each square.
// alpha is the Type A orientation error per nominal 90 degree turn (wheel baseline).
// beta is the Type B orientation error accumulated along one straight leg (wheel diameters).
struct UmbmarkEstimate {
    double x_cg_cw = 0.0;
    double y_cg_cw = 0.0;
    double x_cg_ccw = 0.0;
    double y_cg_ccw = 0.0;
    double r_cg_cw = 0.0;
    double r_cg_ccw = 0.0;
    double e_max_syst = 0.0;
    double alpha_rad = 0.0;
    double beta_rad = 0.0;
    double ed = 1.0;
    double eb = 1.0;
    double c_l = 1.0;
    double c_r = 1.0;
    // Mean absolute heading component of the return-position errors, radians.
    double heading_err_rad = 0.0;
};

// Centroids of the two clouds, then the published separation of Type A and Type B:
//   alpha = (x_cg_cw + x_cg_ccw) / (-4 L)
//   beta  = (x_cg_cw - x_cg_ccw) / (-4 L)
//   Eb    = 90 deg / (90 deg - alpha)
//   R     = (L / 2) / sin(beta / 2)
//   Ed    = (R + b/2) / (R - b/2)
// Both input sequences must contain at least kMinRunsPerDirection poses.
UmbmarkEstimate calibrate_umbmark(const std::vector<Pose>& errors_cw,
                                  const std::vector<Pose>& errors_ccw,
                                  double side_m,
                                  double b_nominal);

// Odometry parameters that compensate a measured (Ed, Eb). The correction
// factors keep the average wheel diameter fixed:
//   c_L = 2 / (Ed + 1),  c_R = 2 / ((1/Ed) + 1),  b <- Eb * b_nominal.
DiffDriveParams apply_umbmark_correction(double ed,
                                         double eb,
                                         double r_nominal,
                                         double b_nominal);

double mean_abs_heading(const std::vector<Pose>& errors);

}  // namespace odomcal
