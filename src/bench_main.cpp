#include "odomcal/benchmark.hpp"

#include <exception>
#include <iostream>
#include <string>

namespace {

void print_usage(std::ostream& out) {
    out << "Usage: odomcal_bench [benchmark.csv [trajectories.csv]]\n"
        << "Runs the simulated UMBmark and least-squares benchmark and writes the summary\n"
        << "(default docs/benchmark.csv) and the example squares (default docs/trajectories.csv).\n";
}

}  // namespace

int main(int argc, char** argv) {
    if (argc >= 2 && (std::string(argv[1]) == "--help" || std::string(argv[1]) == "-h")) {
        print_usage(std::cout);
        return 0;
    }
    // Paths are positional, so anything that looks like an option is a mistake.
    for (int i = 1; i < argc; ++i) {
        if (argv[i][0] == '-' || i > 2) {
            std::cerr << "odomcal_bench: unexpected argument " << argv[i] << '\n';
            print_usage(std::cerr);
            return 1;
        }
    }

    try {
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
    } catch (const std::exception& error) {
        std::cerr << "odomcal_bench: " << error.what() << '\n';
        return 1;
    }
}
