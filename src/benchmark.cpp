#include "odomcal/benchmark.hpp"

#include <cmath>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <sstream>
#include <stdexcept>
#include <utility>

namespace odomcal {
namespace {

struct Score {
    double e_max_syst = 0.0;
    double heading_deg = 0.0;
};

std::vector<Pose> return_errors(const std::vector<SimRun>& runs) {
    std::vector<Pose> errors;
    errors.reserve(runs.size());
    for (const SimRun& run : runs) {
        errors.push_back(run.return_error());
    }
    return errors;
}

double centroid_radius(const std::vector<SimRun>& runs) {
    double x = 0.0;
    double y = 0.0;
    for (const SimRun& run : runs) {
        const Pose error = run.return_error();
        x += error.x;
        y += error.y;
    }
    const double n = static_cast<double>(runs.size());
    return std::hypot(x / n, y / n);
}

Score score_trials(const TrialSet& trials) {
    if (trials.cw.empty() || trials.ccw.empty()) {
        throw std::invalid_argument("scoring requires both directions");
    }
    double heading_sum = 0.0;
    int count = 0;
    for (const SimRun& run : trials.cw) {
        heading_sum += std::abs(run.return_error().theta);
        ++count;
    }
    for (const SimRun& run : trials.ccw) {
        heading_sum += std::abs(run.return_error().theta);
        ++count;
    }
    Score score;
    score.e_max_syst = std::max(centroid_radius(trials.cw), centroid_radius(trials.ccw));
    score.heading_deg = (heading_sum / static_cast<double>(count)) * (180.0 / kPi);
    return score;
}

TrialSet replay_trials(const TrialSet& trials, const DiffDriveParams& odometry) {
    TrialSet replayed;
    replayed.cw.reserve(trials.cw.size());
    replayed.ccw.reserve(trials.ccw.size());
    for (const SimRun& run : trials.cw) {
        replayed.cw.push_back(replay_odometry(run, odometry));
    }
    for (const SimRun& run : trials.ccw) {
        replayed.ccw.push_back(replay_odometry(run, odometry));
    }
    return replayed;
}

std::vector<WheelSample> samples_from_trials(const TrialSet& trials) {
    std::vector<WheelSample> samples;
    const auto append = [&samples](const std::vector<SimRun>& runs) {
        for (const SimRun& run : runs) {
            for (std::size_t i = 0; i < run.encoders.size(); ++i) {
                samples.push_back(wheel_sample(run.actual[i],
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

double mean_of(const std::vector<BenchRow>& rows, double BenchRow::* field) {
    double sum = 0.0;
    for (const BenchRow& row : rows) {
        sum += row.*field;
    }
    return sum / static_cast<double>(rows.size());
}

BenchRow mean_row(const std::vector<BenchRow>& rows) {
    if (rows.empty()) {
        throw std::logic_error("cannot average an empty benchmark group");
    }
    BenchRow mean = rows.front();
    mean.row_type = "mean";
    mean.seed = -1;
    mean.e_max_before_m = mean_of(rows, &BenchRow::e_max_before_m);
    mean.heading_before_deg = mean_of(rows, &BenchRow::heading_before_deg);
    mean.e_max_after_umb_m = mean_of(rows, &BenchRow::e_max_after_umb_m);
    mean.heading_after_umb_deg = mean_of(rows, &BenchRow::heading_after_umb_deg);
    mean.e_max_after_ls_m = mean_of(rows, &BenchRow::e_max_after_ls_m);
    mean.heading_after_ls_deg = mean_of(rows, &BenchRow::heading_after_ls_deg);
    mean.ed_umb = mean_of(rows, &BenchRow::ed_umb);
    mean.eb_umb = mean_of(rows, &BenchRow::eb_umb);
    mean.ed_ls = mean_of(rows, &BenchRow::ed_ls);
    mean.eb_ls = mean_of(rows, &BenchRow::eb_ls);
    mean.r_l_m = mean_of(rows, &BenchRow::r_l_m);
    mean.r_r_m = mean_of(rows, &BenchRow::r_r_m);
    mean.b_m = mean_of(rows, &BenchRow::b_m);
    mean.n_seeds = static_cast<int>(rows.size());
    return mean;
}

std::string format_double(double value) {
    if (std::abs(value) < 5e-9) {
        value = 0.0;
    }
    std::ostringstream stream;
    stream << std::fixed << std::setprecision(8) << value;
    return stream.str();
}

void append_trace(std::vector<TrajectorySample>& samples,
                  const std::vector<Pose>& poses,
                  const SimRun& run,
                  const std::string& trace,
                  const BenchSettings& settings) {
    for (std::size_t i = 0; i < poses.size(); ++i) {
        TrajectorySample sample;
        sample.seed = settings.figure_seed;
        sample.ed_true = settings.figure_ed;
        sample.eb_true = settings.figure_eb;
        sample.side_m = settings.side_m;
        sample.direction = run.direction;
        sample.trace = trace;
        sample.index = static_cast<int>(i);
        sample.x_m = poses[i].x;
        sample.y_m = poses[i].y;
        sample.theta_rad = poses[i].theta;
        samples.push_back(sample);
    }
}

struct CaseDetail {
    BenchRow row;
    TrialSet evaluation;
    DiffDriveParams umb_odom;
    DiffDriveParams ls_odom;
};

CaseDetail evaluate_detail(const ErrorLevel& level, std::uint32_t seed, const BenchSettings& settings) {
    if (settings.n_runs < kMinRunsPerDirection) {
        throw std::invalid_argument("benchmark uses at least 5 runs in each direction");
    }
    const DiffDriveParams plant =
        params_from_ratios(settings.r_nom_m, settings.b_nom_m, level.ed, level.eb);
    const DiffDriveParams nominal = nominal_params(settings.r_nom_m, settings.b_nom_m);
    const std::uint32_t calibration_seed = mix_seed(seed, 1u);
    const std::uint32_t evaluation_seed = mix_seed(seed, 2u);

    const TrialSet calibration = run_square_trials(plant,
                                                   nominal,
                                                   settings.r_nom_m,
                                                   settings.b_nom_m,
                                                   settings.n_runs,
                                                   calibration_seed,
                                                   settings.noise_std_rad,
                                                   settings.side_m,
                                                   settings.steps_per_segment);
    CaseDetail detail;
    detail.evaluation = run_square_trials(plant,
                                          nominal,
                                          settings.r_nom_m,
                                          settings.b_nom_m,
                                          settings.n_runs,
                                          evaluation_seed,
                                          settings.noise_std_rad,
                                          settings.side_m,
                                          settings.steps_per_segment);

    const UmbmarkEstimate umbmark = calibrate_umbmark(return_errors(calibration.cw),
                                                      return_errors(calibration.ccw),
                                                      settings.side_m,
                                                      settings.b_nom_m);
    detail.umb_odom =
        apply_umbmark_correction(umbmark.ed, umbmark.eb, settings.r_nom_m, settings.b_nom_m);
    const LsEstimate least_squares =
        calibrate_least_squares(samples_from_trials(calibration), settings.b_nom_m);
    detail.ls_odom = least_squares.params;

    const Score before = score_trials(detail.evaluation);
    const Score after_umb = score_trials(replay_trials(detail.evaluation, detail.umb_odom));
    const Score after_ls = score_trials(replay_trials(detail.evaluation, detail.ls_odom));

    BenchRow& row = detail.row;
    row.row_type = "trial";
    row.seed = static_cast<int>(seed);
    row.ed_true = level.ed;
    row.eb_true = level.eb;
    row.e_max_before_m = before.e_max_syst;
    row.heading_before_deg = before.heading_deg;
    row.e_max_after_umb_m = after_umb.e_max_syst;
    row.heading_after_umb_deg = after_umb.heading_deg;
    row.e_max_after_ls_m = after_ls.e_max_syst;
    row.heading_after_ls_deg = after_ls.heading_deg;
    row.ed_umb = umbmark.ed;
    row.eb_umb = umbmark.eb;
    row.ed_ls = least_squares.ed;
    row.eb_ls = least_squares.eb;
    row.r_l_m = least_squares.params.r_l;
    row.r_r_m = least_squares.params.r_r;
    row.b_m = least_squares.params.b;
    row.r_l_true_m = plant.r_l;
    row.r_r_true_m = plant.r_r;
    row.b_true_m = plant.b;
    row.side_m = settings.side_m;
    row.r_nom_m = settings.r_nom_m;
    row.b_nom_m = settings.b_nom_m;
    row.noise_std_rad = settings.noise_std_rad;
    row.n_runs = settings.n_runs;
    row.n_seeds = settings.n_seeds;
    row.steps_per_segment = settings.steps_per_segment;
    return detail;
}

}  // namespace

const std::vector<ErrorLevel>& benchmark_levels() {
    static const std::vector<ErrorLevel> levels = {
        {1.000, 1.000},
        {1.001, 1.000},
        {1.000, 1.010},
        {1.002, 1.010},
        {0.998, 1.005},
        {1.005, 0.990},
    };
    return levels;
}

BenchRow evaluate_case(const ErrorLevel& level, std::uint32_t seed, const BenchSettings& settings) {
    return evaluate_detail(level, seed, settings).row;
}

std::vector<BenchRow> run_benchmark(const BenchSettings& settings) {
    std::vector<BenchRow> rows;
    for (const ErrorLevel& level : benchmark_levels()) {
        std::vector<BenchRow> group;
        group.reserve(static_cast<std::size_t>(settings.n_seeds));
        for (int seed = 0; seed < settings.n_seeds; ++seed) {
            group.push_back(evaluate_case(level, static_cast<std::uint32_t>(seed), settings));
        }
        rows.insert(rows.end(), group.begin(), group.end());
        rows.push_back(mean_row(group));
    }
    return rows;
}

std::vector<TrajectorySample> representative_trajectories(const BenchSettings& settings) {
    const CaseDetail detail =
        evaluate_detail(ErrorLevel{settings.figure_ed, settings.figure_eb},
                        static_cast<std::uint32_t>(settings.figure_seed),
                        settings);
    std::vector<TrajectorySample> samples;
    const auto record_direction = [&](const SimRun& run) {
        const SimRun umb = replay_odometry(run, detail.umb_odom);
        const SimRun least_squares = replay_odometry(run, detail.ls_odom);
        append_trace(samples, run.actual, run, "actual", settings);
        append_trace(samples, run.odom, run, "odom_before", settings);
        append_trace(samples, umb.odom, run, "odom_umbmark", settings);
        append_trace(samples, least_squares.odom, run, "odom_least_squares", settings);
    };
    record_direction(detail.evaluation.cw.front());
    record_direction(detail.evaluation.ccw.front());
    return samples;
}

std::string benchmark_csv(const std::vector<BenchRow>& rows) {
    std::ostringstream out;
    out << "row_type,seed,ed_true,eb_true,e_max_before_m,heading_before_deg,"
           "e_max_after_umb_m,heading_after_umb_deg,e_max_after_ls_m,heading_after_ls_deg,"
           "ed_umb,eb_umb,ed_ls,eb_ls,r_l_m,r_r_m,b_m,r_l_true_m,r_r_true_m,b_true_m,"
           "side_m,r_nom_m,b_nom_m,noise_std_rad,n_runs,n_seeds,steps_per_segment\n";
    for (const BenchRow& row : rows) {
        out << row.row_type << ',' << row.seed << ',' << format_double(row.ed_true) << ','
            << format_double(row.eb_true) << ',' << format_double(row.e_max_before_m) << ','
            << format_double(row.heading_before_deg) << ',' << format_double(row.e_max_after_umb_m)
            << ',' << format_double(row.heading_after_umb_deg) << ','
            << format_double(row.e_max_after_ls_m) << ',' << format_double(row.heading_after_ls_deg)
            << ',' << format_double(row.ed_umb) << ',' << format_double(row.eb_umb) << ','
            << format_double(row.ed_ls) << ',' << format_double(row.eb_ls) << ','
            << format_double(row.r_l_m) << ',' << format_double(row.r_r_m) << ','
            << format_double(row.b_m) << ',' << format_double(row.r_l_true_m) << ','
            << format_double(row.r_r_true_m) << ',' << format_double(row.b_true_m) << ','
            << format_double(row.side_m) << ',' << format_double(row.r_nom_m) << ','
            << format_double(row.b_nom_m) << ',' << format_double(row.noise_std_rad) << ','
            << row.n_runs << ',' << row.n_seeds << ',' << row.steps_per_segment << '\n';
    }
    return out.str();
}

std::string trajectory_csv(const std::vector<TrajectorySample>& samples) {
    std::ostringstream out;
    out << "seed,ed_true,eb_true,side_m,direction,trace,index,x_m,y_m,theta_rad\n";
    for (const TrajectorySample& sample : samples) {
        out << sample.seed << ',' << format_double(sample.ed_true) << ','
            << format_double(sample.eb_true) << ',' << format_double(sample.side_m) << ','
            << to_string(sample.direction) << ',' << sample.trace << ',' << sample.index << ','
            << format_double(sample.x_m) << ',' << format_double(sample.y_m) << ','
            << format_double(sample.theta_rad) << '\n';
    }
    return out.str();
}

void write_text_file(const std::string& path, const std::string& contents) {
    const std::filesystem::path file_path(path);
    const std::filesystem::path parent = file_path.parent_path();
    if (!parent.empty()) {
        std::filesystem::create_directories(parent);
    }
    std::ofstream output(file_path);
    if (!output) {
        throw std::runtime_error("failed to open " + path);
    }
    output << contents;
}

}  // namespace odomcal
