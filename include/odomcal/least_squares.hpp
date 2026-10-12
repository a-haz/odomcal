#pragma once

#include "odomcal/diff_drive.hpp"

#include <vector>

namespace odomcal {

// One logged unicycle increment. ds and dth are external-pose measurements of
// the step; dphi_* are the encoder rotations over the same step.
struct WheelSample {
    double dphi_l = 0.0;
    double dphi_r = 0.0;
    double ds = 0.0;
    double dth = 0.0;
};

struct LsEstimate {
    DiffDriveParams params;
    double ed = 1.0;
    double eb = 1.0;
    double residual_rms_m = 0.0;
};

// Rebuild the arc length of one integrator step from its endpoint poses.
// The chord is converted to arc length so a finite turn is not biased short.
WheelSample wheel_sample(const Pose& before,
                         const Pose& after,
                         double dphi_l,
                         double dphi_r);

// Linear least squares for (r_L, r_R, b). Each sample contributes
//   r_L dphi_L + r_R dphi_R           = 2 ds
//  -r_L dphi_L + r_R dphi_R - b dth   = 0
// which is the unicycle map written so every unknown is linear.
// Solved with a rank-revealing QR factorization (Eigen). The path must excite
// both translation and rotation or the solve is rejected.
// Eb is reported relative to b_nominal: Eb = b / b_nominal. Ed = r_R / r_L.
LsEstimate calibrate_least_squares(const std::vector<WheelSample>& samples, double b_nominal);

}  // namespace odomcal
