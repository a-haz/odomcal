#include "odomcal/diff_drive.hpp"
#include "odomcal/simulator.hpp"
#include "support.hpp"

#include <gtest/gtest.h>

#include <cmath>

namespace {

odomcal::Pose straight(const odomcal::DiffDriveParams& plant, double distance_m) {
    const double dphi = distance_m / 0.05;
    return odomcal::integrate(odomcal::Pose{}, dphi, dphi, plant);
}

}  // namespace

TEST(Errors, LargerRightWheelCurvesLeft) {
    const auto plant = odomcal::params_from_ratios(0.05, 0.30, 1.01, 1.0);
    const odomcal::Pose end = straight(plant, 2.0);
    EXPECT_GT(end.theta, 0.0);
    EXPECT_GT(end.y, 0.0);
    EXPECT_LT(end.x, 2.0);
}

TEST(Errors, LargerLeftWheelCurvesRight) {
    const auto plant = odomcal::params_from_ratios(0.05, 0.30, 0.99, 1.0);
    const odomcal::Pose end = straight(plant, 2.0);
    EXPECT_LT(end.theta, 0.0);
    EXPECT_LT(end.y, 0.0);
}

TEST(Errors, EqualWheelsHoldHeading) {
    const odomcal::Pose end = straight(test_nominal(), 3.0);
    EXPECT_NEAR(end.theta, 0.0, 1e-12);
    EXPECT_NEAR(end.y, 0.0, 1e-12);
    EXPECT_NEAR(end.x, 3.0, 1e-12);
}

TEST(Errors, LargerBaselineReducesTurn) {
    const double yaw = odomcal::kPi / 2.0;
    const double dphi_r = (yaw * 0.30 / 2.0) / 0.05;
    const odomcal::DiffDriveParams plant{0.05, 0.05, 0.30 * 1.02};
    const odomcal::Pose end = odomcal::integrate(odomcal::Pose{}, -dphi_r, dphi_r, plant);
    EXPECT_NEAR(end.theta, yaw / 1.02, 1e-12);
}

TEST(Errors, SmallerBaselineIncreasesTurn) {
    const double yaw = odomcal::kPi / 2.0;
    const double dphi_r = (yaw * 0.30 / 2.0) / 0.05;
    const odomcal::DiffDriveParams plant{0.05, 0.05, 0.30 * 0.97};
    const odomcal::Pose end = odomcal::integrate(odomcal::Pose{}, -dphi_r, dphi_r, plant);
    EXPECT_NEAR(end.theta, yaw / 0.97, 1e-12);
}

TEST(Errors, SlipIsDeterministicForASeed) {
    odomcal::SimConfig config = test_square(odomcal::params_from_ratios(0.05, 0.30, 1.002, 1.01),
                                            odomcal::Direction::Ccw);
    config.noise_std_rad = 1e-3;
    config.seed = 42;
    const auto first = odomcal::simulate(config);
    const auto second = odomcal::simulate(config);
    ASSERT_EQ(first.actual.size(), second.actual.size());
    for (std::size_t i = 0; i < first.actual.size(); ++i) {
        EXPECT_DOUBLE_EQ(first.actual[i].x, second.actual[i].x);
        EXPECT_DOUBLE_EQ(first.actual[i].y, second.actual[i].y);
        EXPECT_DOUBLE_EQ(first.actual[i].theta, second.actual[i].theta);
    }
}

TEST(Errors, DistinctSeedsDiverge) {
    odomcal::SimConfig config = test_square(test_nominal(), odomcal::Direction::Cw);
    config.noise_std_rad = 2e-3;
    config.seed = 1;
    const auto first = odomcal::simulate(config);
    config.seed = 2;
    const auto second = odomcal::simulate(config);
    const double gap = std::hypot(first.actual.back().x - second.actual.back().x,
                                  first.actual.back().y - second.actual.back().y);
    EXPECT_GT(gap, 1e-6);
}

TEST(Errors, ZeroNoiseIgnoresSeed) {
    odomcal::SimConfig config = test_square(odomcal::params_from_ratios(0.05, 0.30, 1.003, 0.995),
                                            odomcal::Direction::Cw);
    config.noise_std_rad = 0.0;
    config.seed = 7;
    const auto first = odomcal::simulate(config);
    config.seed = 99;
    const auto second = odomcal::simulate(config);
    EXPECT_DOUBLE_EQ(first.actual.back().x, second.actual.back().x);
    EXPECT_DOUBLE_EQ(first.actual.back().y, second.actual.back().y);
    EXPECT_DOUBLE_EQ(first.actual.back().theta, second.actual.back().theta);
}

TEST(Errors, ReturnErrorIsActualMinusOdometry) {
    odomcal::SimConfig config = test_square(odomcal::params_from_ratios(0.05, 0.30, 1.002, 1.0),
                                            odomcal::Direction::Ccw);
    const auto run = odomcal::simulate(config);
    const odomcal::Pose error = run.return_error();
    EXPECT_NEAR(error.x, run.actual.back().x - run.odom.back().x, 1e-15);
    EXPECT_NEAR(error.y, run.actual.back().y - run.odom.back().y, 1e-15);
    EXPECT_GT(std::hypot(error.x, error.y), 0.05);
    EXPECT_NEAR(run.odom.back().x, 0.0, 1e-9);
    EXPECT_NEAR(run.odom.back().y, 0.0, 1e-9);
}
