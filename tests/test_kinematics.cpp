#include "odomcal/diff_drive.hpp"
#include "odomcal/simulator.hpp"
#include "support.hpp"

#include <gtest/gtest.h>

#include <cmath>

namespace {

using odomcal::DiffDriveParams;
using odomcal::Direction;
using odomcal::Pose;
using odomcal::integrate;
using odomcal::kPi;
using odomcal::wrap_angle;

Pose drive(const Pose& start, double dphi_l, double dphi_r, const DiffDriveParams& params) {
    return integrate(start, dphi_l, dphi_r, params);
}

}  // namespace

TEST(Kinematics, StraightLineOneMetre) {
    const DiffDriveParams params = test_nominal();
    const double dphi = 1.0 / params.r_l;
    const Pose end = drive(Pose{}, dphi, dphi, params);
    EXPECT_NEAR(end.x, 1.0, 1e-12);
    EXPECT_NEAR(end.y, 0.0, 1e-12);
    EXPECT_NEAR(end.theta, 0.0, 1e-12);
}

TEST(Kinematics, ReverseOneMetre) {
    const DiffDriveParams params = test_nominal();
    const double dphi = -1.0 / params.r_l;
    const Pose end = drive(Pose{}, dphi, dphi, params);
    EXPECT_NEAR(end.x, -1.0, 1e-12);
    EXPECT_NEAR(end.y, 0.0, 1e-12);
    EXPECT_NEAR(end.theta, 0.0, 1e-12);
}

TEST(Kinematics, PureRotationCcwQuarterTurn) {
    const DiffDriveParams params = test_nominal();
    const double yaw = kPi / 2.0;
    const double dphi_r = (yaw * params.b / 2.0) / params.r_r;
    const double dphi_l = -dphi_r;
    const Pose end = drive(Pose{}, dphi_l, dphi_r, params);
    EXPECT_NEAR(end.x, 0.0, 1e-12);
    EXPECT_NEAR(end.y, 0.0, 1e-12);
    EXPECT_NEAR(end.theta, yaw, 1e-12);
}

TEST(Kinematics, PureRotationCwHalfTurn) {
    const DiffDriveParams params = test_nominal();
    const double yaw = -kPi;
    const double dphi_r = (yaw * params.b / 2.0) / params.r_r;
    const double dphi_l = -dphi_r;
    const Pose end = drive(Pose{}, dphi_l, dphi_r, params);
    EXPECT_NEAR(end.x, 0.0, 1e-12);
    EXPECT_NEAR(end.y, 0.0, 1e-12);
    EXPECT_NEAR(std::abs(end.theta), kPi, 1e-12);
}

TEST(Kinematics, QuarterCircleArc) {
    const DiffDriveParams params{0.05, 0.05, 0.30};
    const double ds = kPi / 2.0;
    const double dth = kPi / 2.0;
    const double s_r = ds + dth * params.b / 2.0;
    const double s_l = ds - dth * params.b / 2.0;
    const Pose end = drive(Pose{}, s_l / params.r_l, s_r / params.r_r, params);
    EXPECT_NEAR(end.x, 1.0, 1e-12);
    EXPECT_NEAR(end.y, 1.0, 1e-12);
    EXPECT_NEAR(end.theta, kPi / 2.0, 1e-12);
}

TEST(Kinematics, FourQuarterCirclesClose) {
    const DiffDriveParams params{0.05, 0.05, 0.30};
    const double ds = kPi / 2.0;
    const double dth = kPi / 2.0;
    const double s_r = ds + dth * params.b / 2.0;
    const double s_l = ds - dth * params.b / 2.0;
    Pose pose;
    for (int turn = 0; turn < 4; ++turn) {
        pose = drive(pose, s_l / params.r_l, s_r / params.r_r, params);
    }
    EXPECT_NEAR(pose.x, 0.0, 1e-12);
    EXPECT_NEAR(pose.y, 0.0, 1e-12);
    EXPECT_NEAR(pose.theta, 0.0, 1e-12);
}

TEST(Kinematics, SquareClosesBothDirections) {
    const DiffDriveParams params = test_nominal();
    for (const Direction direction : {Direction::Ccw, Direction::Cw}) {
        const auto run = odomcal::simulate(test_square(params, direction));
        const Pose& end = run.actual.back();
        EXPECT_NEAR(end.x, 0.0, 1e-9);
        EXPECT_NEAR(end.y, 0.0, 1e-9);
        EXPECT_NEAR(end.theta, 0.0, 1e-9);
    }
}

TEST(Kinematics, SubdividedStraightMatchesSingleStep) {
    const DiffDriveParams params = test_nominal();
    const double dphi = 1.6 / params.r_l;
    const Pose single = drive(Pose{}, dphi, dphi, params);
    Pose split;
    for (int step = 0; step < 16; ++step) {
        split = drive(split, dphi / 16.0, dphi / 16.0, params);
    }
    EXPECT_NEAR(split.x, single.x, 1e-12);
    EXPECT_NEAR(split.y, single.y, 1e-12);
    EXPECT_NEAR(split.theta, single.theta, 1e-12);
}

TEST(Kinematics, SubdividedArcMatchesSingleStep) {
    const DiffDriveParams params = test_nominal();
    const double ds = 1.2;
    const double dth = 0.4;
    const double s_r = ds + dth * params.b / 2.0;
    const double s_l = ds - dth * params.b / 2.0;
    const double dphi_l = s_l / params.r_l;
    const double dphi_r = s_r / params.r_r;
    const Pose single = drive(Pose{}, dphi_l, dphi_r, params);
    Pose split;
    constexpr int kSteps = 20;
    for (int step = 0; step < kSteps; ++step) {
        split = drive(split, dphi_l / kSteps, dphi_r / kSteps, params);
    }
    EXPECT_NEAR(split.x, single.x, 1e-12);
    EXPECT_NEAR(split.y, single.y, 1e-12);
    EXPECT_NEAR(split.theta, single.theta, 1e-12);
}

TEST(Kinematics, TurnAngleScalesInverselyWithBaseline) {
    const double r = 0.05;
    const double b_nom = 0.30;
    const double yaw = kPi / 2.0;
    const double dphi_r = (yaw * b_nom / 2.0) / r;
    const DiffDriveParams plant{r, r, 1.25 * b_nom};
    const Pose end = drive(Pose{}, -dphi_r, dphi_r, plant);
    EXPECT_NEAR(end.theta, yaw / 1.25, 1e-12);
    EXPECT_NEAR(end.x, 0.0, 1e-12);
    EXPECT_NEAR(end.y, 0.0, 1e-12);
}

TEST(Kinematics, WrapAngleStaysInRange) {
    EXPECT_NEAR(wrap_angle(0.0), 0.0, 1e-15);
    EXPECT_NEAR(wrap_angle(3.0 * kPi / 2.0), -kPi / 2.0, 1e-12);
    EXPECT_NEAR(wrap_angle(-3.0 * kPi / 2.0), kPi / 2.0, 1e-12);
    EXPECT_LE(std::abs(wrap_angle(8.0)), kPi);
}

TEST(Kinematics, RejectsNonPositiveGeometry) {
    DiffDriveParams params = test_nominal();
    params.b = 0.0;
    EXPECT_THROW(drive(Pose{}, 0.1, 0.1, params), std::invalid_argument);
    EXPECT_THROW(odomcal::params_from_ratios(0.05, 0.30, 0.0, 1.0), std::invalid_argument);
    EXPECT_THROW(odomcal::square_commands(Direction::Ccw, 0.05, 0.30, 4.0, 0), std::invalid_argument);
}

TEST(Kinematics, RatioBuilderKeepsMeanRadius) {
    const auto params = odomcal::params_from_ratios(0.05, 0.30, 1.004, 0.992);
    EXPECT_NEAR(0.5 * (params.r_l + params.r_r), 0.05, 1e-12);
    EXPECT_NEAR(params.r_r / params.r_l, 1.004, 1e-12);
    EXPECT_NEAR(params.b, 0.992 * 0.30, 1e-12);
}
