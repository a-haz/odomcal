#pragma once

#include <cstdint>
#include <string>
#include <vector>

namespace odomcal {

constexpr double kPi = 3.14159265358979323846;

// Wrap an angle in radians into (-pi, pi].
double wrap_angle(double theta_rad);

struct Pose {
    double x = 0.0;
    double y = 0.0;
    double theta = 0.0;
};

// Left/right wheel radii and the track width (wheel baseline), all in metres.
struct DiffDriveParams {
    double r_l = 0.0;
    double r_r = 0.0;
    double b = 0.0;
};

struct WheelDelta {
    double dphi_l = 0.0;
    double dphi_r = 0.0;
};

enum class Direction { Ccw, Cw };

std::string to_string(Direction direction);

// Plant parameters for a diameter ratio Ed = r_R / r_L and a baseline ratio
// Eb = b / b_nominal. The arithmetic mean of the two radii stays r_nom, which
// is the scaling-error convention used by Borenstein and Feng (Ed is a ratio,
// not an average-diameter error).
DiffDriveParams params_from_ratios(double r_nom, double b_nom, double ed, double eb);

DiffDriveParams nominal_params(double r_nom, double b_nom);

// Exact constant-curvature step of a differential-drive unicycle.
// dphi_* are wheel rotations in radians. Position is updated in the plane;
// theta is stored wrapped to (-pi, pi].
Pose integrate(const Pose& pose, double dphi_l, double dphi_r, const DiffDriveParams& params);

// Open-loop wheel rotations that trace a square of side `side_m` when executed
// with equal wheel radius r_nom and baseline b_nom. Each straight leg and each
// in-place 90 degree turn is split into `steps_per_segment` equal increments.
// CCW turns are positive yaw. The fourth turn is included, so a perfect run
// finishes at the origin with the starting heading.
std::vector<WheelDelta> square_commands(Direction direction,
                                        double r_nom,
                                        double b_nom,
                                        double side_m,
                                        int steps_per_segment);

}  // namespace odomcal
