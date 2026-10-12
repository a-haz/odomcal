#include "odomcal/umbmark.hpp"

#include <cmath>
#include <stdexcept>

namespace odomcal {
namespace {

struct Centroid {
    double x = 0.0;
    double y = 0.0;
};

Centroid centroid_xy(const std::vector<Pose>& errors) {
    Centroid cog;
    for (const Pose& error : errors) {
        cog.x += error.x;
        cog.y += error.y;
    }
    const double n = static_cast<double>(errors.size());
    cog.x /= n;
    cog.y /= n;
    return cog;
}

}  // namespace

double mean_abs_heading(const std::vector<Pose>& errors) {
    if (errors.empty()) {
        throw std::invalid_argument("heading error requires at least one pose");
    }
    double sum = 0.0;
    for (const Pose& error : errors) {
        sum += std::abs(error.theta);
    }
    return sum / static_cast<double>(errors.size());
}

UmbmarkEstimate calibrate_umbmark(const std::vector<Pose>& errors_cw,
                                  const std::vector<Pose>& errors_ccw,
                                  double side_m,
                                  double b_nominal) {
    if (static_cast<int>(errors_cw.size()) < kMinRunsPerDirection ||
        static_cast<int>(errors_ccw.size()) < kMinRunsPerDirection) {
        throw std::invalid_argument(
            "UMBmark requires at least 5 clockwise and 5 counter-clockwise runs");
    }
    if (!(side_m > 0.0) || !(b_nominal > 0.0)) {
        throw std::invalid_argument("square side and nominal baseline must be positive");
    }

    const Centroid cw = centroid_xy(errors_cw);
    const Centroid ccw = centroid_xy(errors_ccw);

    UmbmarkEstimate estimate;
    estimate.x_cg_cw = cw.x;
    estimate.y_cg_cw = cw.y;
    estimate.x_cg_ccw = ccw.x;
    estimate.y_cg_ccw = ccw.y;
    estimate.r_cg_cw = std::hypot(cw.x, cw.y);
    estimate.r_cg_ccw = std::hypot(ccw.x, ccw.y);
    estimate.e_max_syst = std::max(estimate.r_cg_cw, estimate.r_cg_ccw);

    // Borenstein and Feng, Eqs. for Type A (alpha) and Type B (beta), radians.
    estimate.alpha_rad = (cw.x + ccw.x) / (-4.0 * side_m);
    estimate.beta_rad = (cw.x - ccw.x) / (-4.0 * side_m);

    const double alpha_deg = estimate.alpha_rad * (180.0 / kPi);
    const double turn_denom_deg = 90.0 - alpha_deg;
    if (std::abs(turn_denom_deg) < 1e-9) {
        throw std::runtime_error("UMBmark baseline solution is singular");
    }
    estimate.eb = 90.0 / turn_denom_deg;

    const double half_beta = 0.5 * estimate.beta_rad;
    if (std::abs(std::sin(half_beta)) < 1e-12) {
        estimate.ed = 1.0;
    } else {
        const double radius = (side_m / 2.0) / std::sin(half_beta);
        const double half_b = 0.5 * b_nominal;
        if (std::abs(radius - half_b) < 1e-12) {
            throw std::runtime_error("UMBmark diameter solution is singular");
        }
        estimate.ed = (radius + half_b) / (radius - half_b);
    }
    if (!(estimate.ed > 0.0) || !(estimate.eb > 0.0)) {
        throw std::runtime_error("UMBmark produced a non-positive Ed or Eb");
    }

    estimate.c_l = 2.0 / (estimate.ed + 1.0);
    estimate.c_r = 2.0 / ((1.0 / estimate.ed) + 1.0);

    std::vector<Pose> both = errors_cw;
    both.insert(both.end(), errors_ccw.begin(), errors_ccw.end());
    estimate.heading_err_rad = mean_abs_heading(both);
    return estimate;
}

DiffDriveParams apply_umbmark_correction(double ed,
                                         double eb,
                                         double r_nominal,
                                         double b_nominal) {
    if (!(ed > 0.0) || !(eb > 0.0)) {
        throw std::invalid_argument("Ed and Eb must be positive");
    }
    if (!(r_nominal > 0.0) || !(b_nominal > 0.0)) {
        throw std::invalid_argument("nominal radius and baseline must be positive");
    }
    DiffDriveParams corrected;
    corrected.r_l = r_nominal * (2.0 / (ed + 1.0));
    corrected.r_r = r_nominal * (2.0 / ((1.0 / ed) + 1.0));
    corrected.b = eb * b_nominal;
    return corrected;
}

}  // namespace odomcal
