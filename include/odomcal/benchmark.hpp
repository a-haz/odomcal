#pragma once

#include "odomcal/diff_drive.hpp"
#include "odomcal/least_squares.hpp"
#include "odomcal/simulator.hpp"
#include "odomcal/umbmark.hpp"

#include <string>
#include <vector>

namespace odomcal {

struct BenchSettings {
    double r_nom_m = 0.05;
    double b_nom_m = 0.30;
    double side_m = 4.0;
    double noise_std_rad = 5.0e-4;
    int n_runs = kMinRunsPerDirection;
    int n_seeds = 8;
    int steps_per_segment = 16;
    // Representative square drawn in the trajectory file.
    double figure_ed = 1.002;
    double figure_eb = 1.010;
    int figure_seed = 0;
};

struct ErrorLevel {
    double ed;
    double eb;
};

struct BenchRow {
    std::string row_type;  // "trial" or "mean"
    int seed = 0;
    double ed_true = 1.0;
    double eb_true = 1.0;
    double e_max_before_m = 0.0;
    double heading_before_deg = 0.0;
    double e_max_after_umb_m = 0.0;
    double heading_after_umb_deg = 0.0;
    double e_max_after_ls_m = 0.0;
    double heading_after_ls_deg = 0.0;
    double ed_umb = 1.0;
    double eb_umb = 1.0;
    double ed_ls = 1.0;
    double eb_ls = 1.0;
    double r_l_m = 0.0;
    double r_r_m = 0.0;
    double b_m = 0.0;
    double r_l_true_m = 0.0;
    double r_r_true_m = 0.0;
    double b_true_m = 0.0;
    double side_m = 4.0;
    double r_nom_m = 0.05;
    double b_nom_m = 0.30;
    double noise_std_rad = 0.0;
    int n_runs = 5;
    int n_seeds = 1;
    int steps_per_segment = 16;
};

struct TrajectorySample {
    int seed = 0;
    double ed_true = 1.0;
    double eb_true = 1.0;
    double side_m = 4.0;
    Direction direction = Direction::Ccw;
    std::string trace;
    int index = 0;
    double x_m = 0.0;
    double y_m = 0.0;
    double theta_rad = 0.0;
};

const std::vector<ErrorLevel>& benchmark_levels();

// One seeded case. Calibration runs and the reported before/after runs are
// different seeds, so the scored squares are held out from the fit.
BenchRow evaluate_case(const ErrorLevel& level, std::uint32_t seed, const BenchSettings& settings);

// Trial rows for every level and seed, followed by one mean row per level.
std::vector<BenchRow> run_benchmark(const BenchSettings& settings = {});

// Held-out square used for the before/after figure, seed and level from settings.
std::vector<TrajectorySample> representative_trajectories(const BenchSettings& settings = {});

std::string benchmark_csv(const std::vector<BenchRow>& rows);
std::string trajectory_csv(const std::vector<TrajectorySample>& samples);

void write_text_file(const std::string& path, const std::string& contents);

}  // namespace odomcal
