#include "odomcal/diff_drive.hpp"

#include <cmath>
#include <stdexcept>

namespace odomcal {

double wrap_angle(double theta_rad) {
    // std::remainder yields (-pi, pi], with the pi/-pi tie kept at +pi.
    return std::remainder(theta_rad, 2.0 * kPi);
}

std::string to_string(Direction direction) {
    return direction == Direction::Ccw ? "ccw" : "cw";
}

DiffDriveParams params_from_ratios(double r_nom, double b_nom, double ed, double eb) {
    if (!(r_nom > 0.0) || !(b_nom > 0.0)) {
        throw std::invalid_argument("nominal radius and baseline must be positive");
    }
    if (!(ed > 0.0) || !(eb > 0.0)) {
        throw std::invalid_argument("Ed and Eb must be positive");
    }
    DiffDriveParams params;
    params.r_l = 2.0 * r_nom / (ed + 1.0);
    params.r_r = 2.0 * r_nom * ed / (ed + 1.0);
    params.b = eb * b_nom;
    return params;
}

DiffDriveParams nominal_params(double r_nom, double b_nom) {
    return params_from_ratios(r_nom, b_nom, 1.0, 1.0);
}

Pose integrate(const Pose& pose, double dphi_l, double dphi_r, const DiffDriveParams& params) {
    if (!(params.r_l > 0.0) || !(params.r_r > 0.0) || !(params.b > 0.0)) {
        throw std::invalid_argument("wheel radii and baseline must be positive");
    }
    const double s_l = params.r_l * dphi_l;
    const double s_r = params.r_r * dphi_r;
    const double ds = 0.5 * (s_r + s_l);
    const double dth = (s_r - s_l) / params.b;

    Pose next;
    if (std::abs(dth) < 1e-12) {
        next.x = pose.x + ds * std::cos(pose.theta);
        next.y = pose.y + ds * std::sin(pose.theta);
    } else {
        const double radius = ds / dth;
        next.x = pose.x + radius * (std::sin(pose.theta + dth) - std::sin(pose.theta));
        next.y = pose.y - radius * (std::cos(pose.theta + dth) - std::cos(pose.theta));
    }
    next.theta = wrap_angle(pose.theta + dth);
    return next;
}

std::vector<WheelDelta> square_commands(Direction direction,
                                        double r_nom,
                                        double b_nom,
                                        double side_m,
                                        int steps_per_segment) {
    if (!(r_nom > 0.0) || !(b_nom > 0.0) || !(side_m > 0.0)) {
        throw std::invalid_argument("square command geometry must be positive");
    }
    if (steps_per_segment < 1) {
        throw std::invalid_argument("steps_per_segment must be at least 1");
    }
    const double sign = direction == Direction::Ccw ? 1.0 : -1.0;
    const double yaw = sign * (kPi / 2.0);
    const double steps = static_cast<double>(steps_per_segment);
    const double dphi_straight = (side_m / steps) / r_nom;
    const double dphi_r_turn = (yaw * b_nom / 2.0) / steps / r_nom;
    const double dphi_l_turn = -dphi_r_turn;

    std::vector<WheelDelta> commands;
    commands.reserve(static_cast<std::size_t>(8 * steps_per_segment));
    for (int leg = 0; leg < 4; ++leg) {
        for (int step = 0; step < steps_per_segment; ++step) {
            commands.push_back(WheelDelta{dphi_straight, dphi_straight});
        }
        for (int step = 0; step < steps_per_segment; ++step) {
            commands.push_back(WheelDelta{dphi_l_turn, dphi_r_turn});
        }
    }
    return commands;
}

}  // namespace odomcal
