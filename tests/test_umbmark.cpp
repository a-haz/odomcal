#include "odomcal/simulator.hpp"
#include "odomcal/umbmark.hpp"
#include "support.hpp"

#include <gtest/gtest.h>

#include <cmath>
#include <vector>

namespace {

odomcal::TrialSet zero_noise_trials(double ed, double eb) {
    const auto plant = odomcal::params_from_ratios(0.05, 0.30, ed, eb);
    const auto nominal = test_nominal();
    return odomcal::run_square_trials(plant, nominal, 0.05, 0.30, 5, 1u, 0.0, 4.0, 8);
}

std::vector<odomcal::Pose> errors_of(const std::vector<odomcal::SimRun>& runs) {
    std::vector<odomcal::Pose> errors;
    for (const odomcal::SimRun& run : runs) {
        errors.push_back(run.return_error());
    }
    return errors;
}

odomcal::UmbmarkEstimate solve(double ed, double eb) {
    const odomcal::TrialSet trials = zero_noise_trials(ed, eb);
    return odomcal::calibrate_umbmark(errors_of(trials.cw), errors_of(trials.ccw), 4.0, 0.30);
}

double replay_emax(const odomcal::TrialSet& trials, const odomcal::DiffDriveParams& odometry) {
    double x_cw = 0.0;
    double y_cw = 0.0;
    double x_ccw = 0.0;
    double y_ccw = 0.0;
    for (const odomcal::SimRun& run : trials.cw) {
        const odomcal::Pose error = odomcal::replay_odometry(run, odometry).return_error();
        x_cw += error.x;
        y_cw += error.y;
    }
    for (const odomcal::SimRun& run : trials.ccw) {
        const odomcal::Pose error = odomcal::replay_odometry(run, odometry).return_error();
        x_ccw += error.x;
        y_ccw += error.y;
    }
    const double n = static_cast<double>(trials.cw.size());
    const double r_cw = std::hypot(x_cw / n, y_cw / n);
    const double r_ccw = std::hypot(x_ccw / n, y_ccw / n);
    return std::max(r_cw, r_ccw);
}

}  // namespace

TEST(Umbmark, IdentityEstimatesAreUnity) {
    const odomcal::UmbmarkEstimate estimate = solve(1.0, 1.0);
    EXPECT_NEAR(estimate.ed, 1.0, 1e-9);
    EXPECT_NEAR(estimate.eb, 1.0, 1e-9);
    EXPECT_NEAR(estimate.e_max_syst, 0.0, 1e-9);
    EXPECT_NEAR(estimate.heading_err_rad, 0.0, 1e-9);
}

TEST(Umbmark, RecoversDiameterRatio) {
    const odomcal::UmbmarkEstimate estimate = solve(1.002, 1.0);
    EXPECT_NEAR(estimate.ed, 1.002, 2e-5);
    EXPECT_NEAR(estimate.eb, 1.0, 1e-3);
}

TEST(Umbmark, RecoversBaselineRatio) {
    const odomcal::UmbmarkEstimate estimate = solve(1.0, 1.01);
    EXPECT_NEAR(estimate.eb, 1.01, 3e-4);
    EXPECT_NEAR(estimate.ed, 1.0, 1e-6);
}

TEST(Umbmark, RecoversBothParameters) {
    const odomcal::UmbmarkEstimate estimate = solve(1.002, 1.01);
    EXPECT_NEAR(estimate.ed, 1.002, 2e-4);
    EXPECT_NEAR(estimate.eb, 1.01, 1.5e-3);
}

TEST(Umbmark, RejectsFewerThanFiveRuns) {
    const odomcal::Pose error{0.1, -0.2, 0.01};
    EXPECT_THROW(odomcal::calibrate_umbmark(repeat_pose(error, 4), repeat_pose(error, 5), 4.0, 0.30),
                 std::invalid_argument);
    EXPECT_THROW(odomcal::calibrate_umbmark(repeat_pose(error, 5), repeat_pose(error, 4), 4.0, 0.30),
                 std::invalid_argument);
}

TEST(Umbmark, CorrectionFactorsPreserveMeanRadiusAndRatio) {
    const odomcal::DiffDriveParams corrected = odomcal::apply_umbmark_correction(1.004, 1.01, 0.05, 0.30);
    EXPECT_NEAR(0.5 * (corrected.r_l + corrected.r_r), 0.05, 1e-12);
    EXPECT_NEAR(corrected.r_r / corrected.r_l, 1.004, 1e-12);
    EXPECT_NEAR(corrected.b, 1.01 * 0.30, 1e-12);
    const double c_l = corrected.r_l / 0.05;
    const double c_r = corrected.r_r / 0.05;
    EXPECT_NEAR(c_l, 2.0 / (1.004 + 1.0), 1e-12);
    EXPECT_NEAR(c_r, 2.0 / ((1.0 / 1.004) + 1.0), 1e-12);
}

TEST(Umbmark, CorrectionShrinksReturnError) {
    const odomcal::TrialSet trials = zero_noise_trials(1.002, 1.01);
    const odomcal::UmbmarkEstimate estimate =
        odomcal::calibrate_umbmark(errors_of(trials.cw), errors_of(trials.ccw), 4.0, 0.30);
    const double before = estimate.e_max_syst;
    const odomcal::DiffDriveParams corrected =
        odomcal::apply_umbmark_correction(estimate.ed, estimate.eb, 0.05, 0.30);
    const double after = replay_emax(trials, corrected);
    EXPECT_GT(before, 0.2);
    EXPECT_LT(after, before / 5.0);
}

TEST(Umbmark, EmaxIsTheLargerCentroid) {
    const odomcal::UmbmarkEstimate estimate = solve(1.002, 1.01);
    EXPECT_NEAR(estimate.e_max_syst, std::max(estimate.r_cg_cw, estimate.r_cg_ccw), 1e-15);
    EXPECT_GT(estimate.e_max_syst, 0.0);
}

TEST(Umbmark, PureBaselineSplitsLateralErrorByDirection) {
    const odomcal::TrialSet trials = zero_noise_trials(1.0, 1.01);
    const odomcal::Pose cw = trials.cw.front().return_error();
    const odomcal::Pose ccw = trials.ccw.front().return_error();
    EXPECT_NEAR(cw.x, ccw.x, 1e-9);
    EXPECT_NEAR(cw.y, -ccw.y, 1e-9);
    EXPECT_NEAR(cw.theta, -ccw.theta, 1e-9);
    EXPECT_GT(std::abs(cw.y), 0.05);
}
