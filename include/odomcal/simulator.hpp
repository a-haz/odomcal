#pragma once

#include "odomcal/diff_drive.hpp"

#include <cstdint>
#include <vector>

namespace odomcal {

// One square-path execution.
// `actual` is the plant pose (wheel rotations plus slip, true radii and baseline).
// `odom` integrates the encoder rotations with the odometry parameters.
// `encoders` are the rotations the encoders report; slip is not observed.
// All pose sequences include the origin as element 0, so size == encoders.size() + 1.
struct SimRun {
    std::vector<Pose> actual;
    std::vector<Pose> odom;
    std::vector<WheelDelta> encoders;
    Direction direction = Direction::Ccw;

    Pose return_error() const;
};

struct SimConfig {
    DiffDriveParams plant;
    DiffDriveParams odometry;
    // Geometry the motion program uses to turn a square into encoder counts.
    // Calibration changes `odometry` only; the commanded counts stay on this model,
    // matching a dead-reckoning correction rather than a change of the plant.
    double command_radius_m = 0.05;
    double command_baseline_m = 0.30;
    double side_m = 4.0;
    int steps_per_segment = 16;
    // Standard deviation of zero-mean Gaussian slip on each wheel rotation step,
    // in radians. Encoders do not observe this slip.
    double noise_std_rad = 0.0;
    std::uint32_t seed = 1;
    Direction direction = Direction::Ccw;
};

// Deterministic stream mixer so each run has its own RNG seed.
std::uint32_t mix_seed(std::uint32_t seed, std::uint32_t stream);

SimRun simulate(const SimConfig& config);

// Re-integrate encoder readings with a different odometry model.
// The plant trajectory and the encoder log are unchanged.
SimRun replay_odometry(const SimRun& run, const DiffDriveParams& odometry);

struct TrialSet {
    std::vector<SimRun> cw;
    std::vector<SimRun> ccw;
};

// n_runs clockwise and n_runs counter-clockwise squares. Run i in each
// direction consumes an independent seed derived from `seed`.
TrialSet run_square_trials(const DiffDriveParams& plant,
                           const DiffDriveParams& odometry,
                           double command_radius_m,
                           double command_baseline_m,
                           int n_runs,
                           std::uint32_t seed,
                           double noise_std_rad,
                           double side_m,
                           int steps_per_segment);

}  // namespace odomcal
