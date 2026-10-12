#include "odomcal/least_squares.hpp"

#include <Eigen/Dense>

#include <cmath>
#include <stdexcept>

namespace odomcal {

WheelSample wheel_sample(const Pose& before, const Pose& after, double dphi_l, double dphi_r) {
    const double dx = after.x - before.x;
    const double dy = after.y - before.y;
    const double dth = wrap_angle(after.theta - before.theta);
    const double chord = std::hypot(dx, dy);
    double ds = 0.0;
    if (std::abs(dth) < 1e-9) {
        ds = dx * std::cos(before.theta) + dy * std::sin(before.theta);
    } else {
        const double arc = chord * (dth / 2.0) / std::sin(dth / 2.0);
        const double forward = dx * std::cos(before.theta) + dy * std::sin(before.theta);
        ds = std::copysign(std::abs(arc), forward);
    }
    WheelSample sample;
    sample.dphi_l = dphi_l;
    sample.dphi_r = dphi_r;
    sample.ds = ds;
    sample.dth = dth;
    return sample;
}

LsEstimate calibrate_least_squares(const std::vector<WheelSample>& samples, double b_nominal) {
    if (samples.size() < 2) {
        throw std::invalid_argument("least squares requires at least two motion samples");
    }
    if (!(b_nominal > 0.0)) {
        throw std::invalid_argument("nominal baseline must be positive");
    }

    const Eigen::Index rows = static_cast<Eigen::Index>(samples.size() * 2);
    Eigen::MatrixXd design(rows, 3);
    Eigen::VectorXd measurement(rows);
    for (std::size_t i = 0; i < samples.size(); ++i) {
        const WheelSample& sample = samples[i];
        const Eigen::Index row = static_cast<Eigen::Index>(i * 2);
        design(row, 0) = sample.dphi_l;
        design(row, 1) = sample.dphi_r;
        design(row, 2) = 0.0;
        measurement(row) = 2.0 * sample.ds;

        design(row + 1, 0) = -sample.dphi_l;
        design(row + 1, 1) = sample.dphi_r;
        design(row + 1, 2) = -sample.dth;
        measurement(row + 1) = 0.0;
    }

    Eigen::ColPivHouseholderQR<Eigen::MatrixXd> factorization(design);
    factorization.setThreshold(1e-10);
    if (factorization.rank() < 3) {
        throw std::runtime_error(
            "least squares: path does not excite both wheel radii and the baseline");
    }
    const Eigen::Vector3d solution = factorization.solve(measurement);
    if (!(solution(0) > 0.0) || !(solution(1) > 0.0) || !(solution(2) > 0.0)) {
        throw std::runtime_error("least squares produced a non-positive radius or baseline");
    }

    const Eigen::VectorXd residual = design * solution - measurement;
    LsEstimate estimate;
    estimate.params.r_l = solution(0);
    estimate.params.r_r = solution(1);
    estimate.params.b = solution(2);
    estimate.ed = estimate.params.r_r / estimate.params.r_l;
    estimate.eb = estimate.params.b / b_nominal;
    estimate.residual_rms_m = std::sqrt(residual.squaredNorm() / static_cast<double>(residual.size()));
    return estimate;
}

}  // namespace odomcal
