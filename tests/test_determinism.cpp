#include "odomcal/benchmark.hpp"
#include "odomcal/simulator.hpp"
#include "support.hpp"

#include <gtest/gtest.h>

#include <cmath>

TEST(Determinism, SameSeedSameTrajectory) {
    const auto plant = odomcal::params_from_ratios(0.05, 0.30, 1.002, 1.01);
    const auto first = odomcal::run_square_trials(plant, test_nominal(), 0.05, 0.30, 5, 11u, 5e-4, 4.0, 8);
    const auto second = odomcal::run_square_trials(plant, test_nominal(), 0.05, 0.30, 5, 11u, 5e-4, 4.0, 8);
    ASSERT_EQ(first.cw.size(), second.cw.size());
    for (std::size_t i = 0; i < first.cw.size(); ++i) {
        EXPECT_DOUBLE_EQ(first.cw[i].actual.back().x, second.cw[i].actual.back().x);
        EXPECT_DOUBLE_EQ(first.ccw[i].actual.back().y, second.ccw[i].actual.back().y);
        EXPECT_DOUBLE_EQ(first.cw[i].odom.back().theta, second.cw[i].odom.back().theta);
    }
}

TEST(Determinism, SameSeedSameCalibrationRow) {
    odomcal::BenchSettings settings;
    settings.n_seeds = 2;
    settings.noise_std_rad = 5e-4;
    const odomcal::ErrorLevel level{1.002, 1.010};
    const odomcal::BenchRow first = odomcal::evaluate_case(level, 4u, settings);
    const odomcal::BenchRow second = odomcal::evaluate_case(level, 4u, settings);
    EXPECT_DOUBLE_EQ(first.e_max_before_m, second.e_max_before_m);
    EXPECT_DOUBLE_EQ(first.e_max_after_umb_m, second.e_max_after_umb_m);
    EXPECT_DOUBLE_EQ(first.e_max_after_ls_m, second.e_max_after_ls_m);
    EXPECT_DOUBLE_EQ(first.ed_umb, second.ed_umb);
    EXPECT_DOUBLE_EQ(first.eb_umb, second.eb_umb);
    EXPECT_DOUBLE_EQ(first.ed_ls, second.ed_ls);
    EXPECT_DOUBLE_EQ(first.r_l_m, second.r_l_m);
    EXPECT_DOUBLE_EQ(first.b_m, second.b_m);
}

TEST(Determinism, BiasedLevelsImproveOnHeldOutSquares) {
    odomcal::BenchSettings settings;
    settings.n_seeds = 2;
    settings.noise_std_rad = 5e-4;
    const std::vector<odomcal::BenchRow> rows = odomcal::run_benchmark(settings);
    int mean_rows = 0;
    for (const odomcal::BenchRow& row : rows) {
        if (row.row_type != "mean") {
            continue;
        }
        ++mean_rows;
        const bool biased = std::abs(row.ed_true - 1.0) > 1e-12 || std::abs(row.eb_true - 1.0) > 1e-12;
        if (!biased) {
            continue;
        }
        EXPECT_GT(row.e_max_before_m, 0.05);
        EXPECT_LT(row.e_max_after_umb_m, row.e_max_before_m / 3.0);
        EXPECT_LT(row.e_max_after_ls_m, row.e_max_before_m / 3.0);
        EXPECT_LT(row.heading_after_ls_deg, row.heading_before_deg / 3.0);
    }
    EXPECT_EQ(mean_rows, static_cast<int>(odomcal::benchmark_levels().size()));
}

TEST(Determinism, CsvContainsTrialAndMeanRows) {
    odomcal::BenchSettings settings;
    settings.n_seeds = 1;
    settings.noise_std_rad = 0.0;
    const std::string csv = odomcal::benchmark_csv(odomcal::run_benchmark(settings));
    EXPECT_NE(csv.find("row_type,seed,ed_true"), std::string::npos);
    EXPECT_NE(csv.find("\ntrial,0,"), std::string::npos);
    EXPECT_NE(csv.find("\nmean,-1,"), std::string::npos);
    EXPECT_NE(csv.find("4.00000000"), std::string::npos);
    EXPECT_NE(csv.find("0.05000000"), std::string::npos);
    EXPECT_NE(csv.find("0.30000000"), std::string::npos);
}
