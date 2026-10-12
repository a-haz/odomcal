#include "odomcal/benchmark.hpp"

#include <iostream>
#include <string>

int main(int argc, char** argv) {
    const std::string csv_path = argc >= 2 ? argv[1] : "docs/benchmark.csv";
    const std::string trajectory_path = argc >= 3 ? argv[2] : "docs/trajectories.csv";

    const odomcal::BenchSettings settings;
    const std::vector<odomcal::BenchRow> rows = odomcal::run_benchmark(settings);
    const std::vector<odomcal::TrajectorySample> trajectories =
        odomcal::representative_trajectories(settings);
    odomcal::write_text_file(csv_path, odomcal::benchmark_csv(rows));
    odomcal::write_text_file(trajectory_path, odomcal::trajectory_csv(trajectories));

    std::cout << "wrote " << rows.size() << " rows to " << csv_path << '\n';
    std::cout << "wrote " << trajectories.size() << " samples to " << trajectory_path << '\n';
    for (const odomcal::BenchRow& row : rows) {
        if (row.row_type != "mean") {
            continue;
        }
        std::cout << "Ed " << row.ed_true << " Eb " << row.eb_true
                  << "  Emax before " << row.e_max_before_m << "  after UMBmark "
                  << row.e_max_after_umb_m << "  after least squares " << row.e_max_after_ls_m
                  << '\n';
    }
    return 0;
}
