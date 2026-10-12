#include "odomcal/least_squares.hpp"
#include "odomcal/simulator.hpp"
#include "odomcal/umbmark.hpp"
#include "support.hpp"

#include <gtest/gtest.h>

#include <cmath>
#include <vector>

namespace {

std::vector<odomcal::WheelSample> samples_of(const odomcal::TrialSet& trials) {
    std::vector<odomcal::WheelSample> samples;
    const auto append = [&samples](const std::vector<odomcal::SimRun>& runs) {
        for (const odomcal::SimRun& run : runs) {
            for (std::size_t i = 0; i < run.encoders.size(); ++i) {
                samples.push_back(odomcal::wheel_sample(run.actual[i],
                                                        run.actual[i + 1],
                                                        run.encoders[i].dphi_l,
                                                        run.encoders[i].dphi_r));
            }
        }
    };
    append(trials.cw);
    append(trials.ccw);
    return samples;
}

odomcal::TrialSet logged_square(double ed, double eb) {
    const auto plant = odomcal::params_from_ratios(0.05, 0.30, ed, eb);
    return odomcal::run_square_trials(plant, test_nominal(), 0.05, 0.30, 5, 3u, 0.0, 4.0, 8);
}

}  // namespace

TEST(LeastSquares, RecoversWheelParameters) {
    const auto plant = odomcal::params_from_ratios(0.05, 0.30, 1.002, 1.01);
    const odomcal::LsEstimate estimate =
        odomcal::calibrate_least_squares(samples_of(logged_square(1.002, 1.01)), 0.30);
    EXPECT_NEAR(estimate.params.r_l, plant.r_l, 1e-8);
    EXPECT_NEAR(estimate.params.r_r, plant.r_r, 1e-8);
    EXPECT_NEAR(estimate.params.b, plant.b, 1e-8);
    EXPECT_LT(estimate.residual_rms_m, 1e-8);
}

TEST(LeastSquares, DerivedRatiosMatchTruth) {
    const odomcal::LsEstimate estimate =
        odomcal::calibrate_least_squares(samples_of(logged_square(0.998, 1.005)), 0.30);
    EXPECT_NEAR(estimate.ed, 0.998, 1e-8);
    EXPECT_NEAR(estimate.eb, 1.005, 1e-8);
}

TEST(LeastSquares, AgreesWithUmbmarkOnMildCase) {
    const odomcal::TrialSet trials = logged_square(1.002, 1.01);
    std::vector<odomcal::Pose> cw;
    std::vector<odomcal::Pose> ccw;
    for (const odomcal::SimRun& run : trials.cw) {
        cw.push_back(run.return_error());
    }
    for (const odomcal::SimRun& run : trials.ccw) {
        ccw.push_back(run.return_error());
    }
    const odomcal::UmbmarkEstimate umbmark = odomcal::calibrate_umbmark(cw, ccw, 4.0, 0.30);
    const odomcal::LsEstimate least_squares =
        odomcal::calibrate_least_squares(samples_of(trials), 0.30);
    EXPECT_NEAR(least_squares.ed, umbmark.ed, 2e-3);
    EXPECT_NEAR(least_squares.eb, umbmark.eb, 2e-3);
}

TEST(LeastSquares, StraightPathIsRankDeficient) {
    const odomcal::DiffDriveParams params = test_nominal();
    odomcal::Pose pose;
    std::vector<odomcal::WheelSample> samples;
    for (int step = 0; step < 20; ++step) {
        const odomcal::Pose next = odomcal::integrate(pose, 0.2, 0.2, params);
        samples.push_back(odomcal::wheel_sample(pose, next, 0.2, 0.2));
        pose = next;
    }
    EXPECT_THROW(odomcal::calibrate_least_squares(samples, 0.30), std::runtime_error);
}

TEST(LeastSquares, CorrectedModelShrinksPoseError) {
    const odomcal::TrialSet trials = logged_square(1.005, 0.99);
    const odomcal::LsEstimate estimate =
        odomcal::calibrate_least_squares(samples_of(trials), 0.30);
    const odomcal::Pose before = trials.ccw.front().return_error();
    const odomcal::Pose after =
        odomcal::replay_odometry(trials.ccw.front(), estimate.params).return_error();
    EXPECT_LT(std::hypot(after.x, after.y), std::hypot(before.x, before.y) / 20.0);
    EXPECT_LT(std::abs(after.theta), std::abs(before.theta) / 20.0);
}
