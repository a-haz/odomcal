#include "odomcal/simulator.hpp"

#include <random>
#include <stdexcept>

namespace odomcal {

Pose SimRun::return_error() const {
    if (actual.empty() || odom.empty()) {
        throw std::logic_error("simulated run has no poses");
    }
    const Pose& ground_truth = actual.back();
    const Pose& estimated = odom.back();
    Pose error;
    error.x = ground_truth.x - estimated.x;
    error.y = ground_truth.y - estimated.y;
    error.theta = wrap_angle(ground_truth.theta - estimated.theta);
    return error;
}

std::uint32_t mix_seed(std::uint32_t seed, std::uint32_t stream) {
    std::uint32_t x = seed + 0x9e3779b9u * (stream + 1u);
    x ^= x >> 16;
    x *= 0x7feb352du;
    x ^= x >> 15;
    x *= 0x846ca68bu;
    x ^= x >> 16;
    return x;
}

SimRun simulate(const SimConfig& config) {
    if (config.noise_std_rad < 0.0) {
        throw std::invalid_argument("noise standard deviation must be non-negative");
    }
    const std::vector<WheelDelta> commands = square_commands(config.direction,
                                                             config.command_radius_m,
                                                             config.command_baseline_m,
                                                             config.side_m,
                                                             config.steps_per_segment);

    SimRun run;
    run.direction = config.direction;
    run.actual.push_back(Pose{});
    run.odom.push_back(Pose{});
    run.encoders.reserve(commands.size());

    std::mt19937 rng(config.seed);
    std::normal_distribution<double> slip(0.0, config.noise_std_rad > 0.0 ? config.noise_std_rad : 1.0);

    Pose actual;
    Pose odom;
    for (const WheelDelta& command : commands) {
        double slip_l = 0.0;
        double slip_r = 0.0;
        if (config.noise_std_rad > 0.0) {
            slip_l = slip(rng);
            slip_r = slip(rng);
        }
        actual = integrate(actual, command.dphi_l + slip_l, command.dphi_r + slip_r, config.plant);
        odom = integrate(odom, command.dphi_l, command.dphi_r, config.odometry);
        run.actual.push_back(actual);
        run.odom.push_back(odom);
        run.encoders.push_back(command);
    }
    return run;
}

SimRun replay_odometry(const SimRun& run, const DiffDriveParams& odometry) {
    if (run.encoders.size() + 1 != run.actual.size()) {
        throw std::invalid_argument("encoder log and plant trajectory have different lengths");
    }
    SimRun replayed = run;
    replayed.odom.clear();
    replayed.odom.push_back(Pose{});
    Pose odom;
    for (const WheelDelta& command : run.encoders) {
        odom = integrate(odom, command.dphi_l, command.dphi_r, odometry);
        replayed.odom.push_back(odom);
    }
    return replayed;
}

TrialSet run_square_trials(const DiffDriveParams& plant,
                           const DiffDriveParams& odometry,
                           double command_radius_m,
                           double command_baseline_m,
                           int n_runs,
                           std::uint32_t seed,
                           double noise_std_rad,
                           double side_m,
                           int steps_per_segment) {
    if (n_runs < 1) {
        throw std::invalid_argument("n_runs must be at least 1");
    }
    TrialSet trials;
    trials.cw.reserve(static_cast<std::size_t>(n_runs));
    trials.ccw.reserve(static_cast<std::size_t>(n_runs));
    for (int index = 0; index < n_runs; ++index) {
        SimConfig config;
        config.plant = plant;
        config.odometry = odometry;
        config.command_radius_m = command_radius_m;
        config.command_baseline_m = command_baseline_m;
        config.side_m = side_m;
        config.steps_per_segment = steps_per_segment;
        config.noise_std_rad = noise_std_rad;

        config.direction = Direction::Cw;
        config.seed = mix_seed(seed, static_cast<std::uint32_t>(index));
        trials.cw.push_back(simulate(config));

        config.direction = Direction::Ccw;
        config.seed = mix_seed(seed, static_cast<std::uint32_t>(n_runs + index));
        trials.ccw.push_back(simulate(config));
    }
    return trials;
}

}  // namespace odomcal
