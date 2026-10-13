#include "odomcal_ros/odom_calibrator.hpp"

#include "odomcal/diff_drive.hpp"
#include "odomcal/simulator.hpp"

#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <sensor_msgs/msg/joint_state.hpp>

#include <charconv>
#include <chrono>
#include <cmath>
#include <cstdlib>
#include <filesystem>
#include <fstream>
#include <functional>
#include <iostream>
#include <stdexcept>
#include <string>
#include <vector>

namespace {

constexpr double kSideM = 4.0;
constexpr double kNominalRadiusM = 0.05;
constexpr double kNominalTrackM = 0.30;
constexpr double kEd = 1.002;
constexpr double kEb = 1.010;
constexpr int kStepsPerSegment = 16;
constexpr double kNoiseStdRad = 0.0;

std::string format_double(double value) {
    char buffer[128];
    const auto result = std::to_chars(buffer,
                                      buffer + sizeof(buffer),
                                      value,
                                      std::chars_format::general);
    if (result.ec != std::errc()) {
        throw std::runtime_error("could not format a floating-point value");
    }
    return std::string(buffer, result.ptr);
}

void write_field(std::ostream& out, const std::string& key, const std::string& value) {
    out << key << ',' << value << '\n';
}

void write_field(std::ostream& out, const std::string& key, double value) {
    write_field(out, key, format_double(value));
}

void write_field(std::ostream& out, const std::string& key, int value) {
    write_field(out, key, std::to_string(value));
}

bool spin_until(rclcpp::executors::SingleThreadedExecutor& executor,
                const std::function<bool()>& done,
                std::chrono::milliseconds timeout) {
    const auto deadline = std::chrono::steady_clock::now() + timeout;
    while (std::chrono::steady_clock::now() < deadline) {
        executor.spin_some(std::chrono::milliseconds(20));
        if (done()) {
            return true;
        }
    }
    executor.spin_some(std::chrono::milliseconds(20));
    return done();
}

double yaw_from_quaternion(const geometry_msgs::msg::Quaternion& quaternion) {
    const double sin_yaw = 2.0 * (quaternion.w * quaternion.z + quaternion.x * quaternion.y);
    const double cos_yaw = 1.0 - 2.0 * (quaternion.y * quaternion.y + quaternion.z * quaternion.z);
    return std::atan2(sin_yaw, cos_yaw);
}

builtin_interfaces::msg::Time stamp_from_index(int index) {
    const int64_t nanos = static_cast<int64_t>(index) * 50000000LL;
    builtin_interfaces::msg::Time stamp;
    stamp.sec = static_cast<int32_t>(nanos / 1000000000LL);
    stamp.nanosec = static_cast<uint32_t>(nanos % 1000000000LL);
    return stamp;
}

std::string parse_output_path(int argc, char** argv, std::vector<std::string>* ros_args) {
    std::string output = "square_replay.csv";
    for (int index = 1; index < argc; ++index) {
        const std::string arg = argv[index];
        if (arg == "--output") {
            if (index + 1 >= argc) {
                throw std::invalid_argument("--output requires a path");
            }
            output = argv[++index];
        } else if (arg.rfind("--output=", 0) == 0) {
            output = arg.substr(std::string("--output=").size());
        } else {
            ros_args->push_back(arg);
        }
    }
    return output;
}

void require_close(const std::string& label, double actual, double expected) {
    if (std::abs(actual - expected) > 1e-8) {
        throw std::runtime_error(label + " diverged from the library integrator: got " + format_double(actual) +
                                 " expected " + format_double(expected));
    }
}

}  // namespace

int main(int argc, char** argv) {
    std::string output_path;
    std::vector<std::string> ros_args;
    try {
        output_path = parse_output_path(argc, argv, &ros_args);
    } catch (const std::exception& ex) {
        std::cerr << ex.what() << '\n';
        return 1;
    }

    std::vector<char*> ros_argv;
    ros_argv.push_back(argv[0]);
    for (std::string& arg : ros_args) {
        ros_argv.push_back(arg.data());
    }
    int ros_argc = static_cast<int>(ros_argv.size());
    rclcpp::init(ros_argc, ros_argv.data());

    int exit_code = 0;
    try {
        const odomcal::DiffDriveParams nominal = odomcal::nominal_params(kNominalRadiusM, kNominalTrackM);
        const odomcal::DiffDriveParams plant =
            odomcal::params_from_ratios(kNominalRadiusM, kNominalTrackM, kEd, kEb);

        odomcal::SimConfig config;
        config.plant = plant;
        config.odometry = nominal;
        config.command_radius_m = kNominalRadiusM;
        config.command_baseline_m = kNominalTrackM;
        config.side_m = kSideM;
        config.steps_per_segment = kStepsPerSegment;
        config.noise_std_rad = kNoiseStdRad;
        config.seed = 1;
        config.direction = odomcal::Direction::Ccw;
        const odomcal::SimRun run = odomcal::simulate(config);

        const std::filesystem::path calibration_path =
            std::filesystem::temp_directory_path() / "odomcal_square_replay.yaml";
        {
            std::ofstream calibration(calibration_path);
            if (!calibration) {
                throw std::runtime_error("could not write " + calibration_path.string());
            }
            calibration << "r_l: " << format_double(plant.r_l) << '\n';
            calibration << "r_r: " << format_double(plant.r_r) << '\n';
            calibration << "track_width: " << format_double(plant.b) << '\n';
        }

        rclcpp::NodeOptions raw_options;
        raw_options.append_parameter_override("joint_states_topic", std::string("/square_replay/joint_states"));
        raw_options.append_parameter_override("odom_topic", std::string("/square_replay/odom_raw"));
        raw_options.append_parameter_override("calibration_file", std::string(""));
        raw_options.append_parameter_override("r_l", nominal.r_l);
        raw_options.append_parameter_override("r_r", nominal.r_r);
        raw_options.append_parameter_override("track_width", nominal.b);
        raw_options.append_parameter_override("use_velocities", false);
        raw_options.append_parameter_override("publish_tf", false);
        raw_options.append_parameter_override("odom_frame", std::string("odom"));
        raw_options.append_parameter_override("base_frame", std::string("base_link"));

        rclcpp::NodeOptions corrected_options;
        corrected_options.append_parameter_override("joint_states_topic",
                                                    std::string("/square_replay/joint_states"));
        corrected_options.append_parameter_override("odom_topic", std::string("/square_replay/odom_corrected"));
        corrected_options.append_parameter_override("calibration_file", calibration_path.string());
        corrected_options.append_parameter_override("use_velocities", false);
        corrected_options.append_parameter_override("publish_tf", true);
        corrected_options.append_parameter_override("odom_frame", std::string("odom"));
        corrected_options.append_parameter_override("base_frame", std::string("base_link"));

        auto raw_node = std::make_shared<odomcal_ros::OdomCalibrator>(raw_options, "odom_raw");
        auto corrected_node = std::make_shared<odomcal_ros::OdomCalibrator>(corrected_options, "odom_corrected");
        auto replay = std::make_shared<rclcpp::Node>("square_replay");

        std::vector<nav_msgs::msg::Odometry> raw_messages;
        std::vector<nav_msgs::msg::Odometry> corrected_messages;
        const auto qos = rclcpp::QoS(rclcpp::KeepLast(10)).reliable();
        auto joint_pub = replay->create_publisher<sensor_msgs::msg::JointState>("/square_replay/joint_states", qos);
        auto raw_sub = replay->create_subscription<nav_msgs::msg::Odometry>(
            "/square_replay/odom_raw",
            qos,
            [&raw_messages](const nav_msgs::msg::Odometry::SharedPtr msg) { raw_messages.push_back(*msg); });
        auto corrected_sub = replay->create_subscription<nav_msgs::msg::Odometry>(
            "/square_replay/odom_corrected",
            qos,
            [&corrected_messages](const nav_msgs::msg::Odometry::SharedPtr msg) {
                corrected_messages.push_back(*msg);
            });

        rclcpp::executors::SingleThreadedExecutor executor;
        executor.add_node(raw_node);
        executor.add_node(corrected_node);
        executor.add_node(replay);

        if (!spin_until(
                executor,
                [&] { return joint_pub->get_subscription_count() >= 2 && raw_sub->get_publisher_count() >= 1 &&
                             corrected_sub->get_publisher_count() >= 1; },
                std::chrono::seconds(20))) {
            throw std::runtime_error("timed out waiting for odometry publishers and joint subscribers");
        }

        double phi_l = 0.0;
        double phi_r = 0.0;
        int stamp_index = 0;
        auto publish_positions = [&](double left, double right) {
            sensor_msgs::msg::JointState joints;
            joints.header.stamp = stamp_from_index(stamp_index);
            ++stamp_index;
            joints.name = {"left_wheel_joint", "right_wheel_joint"};
            joints.position = {left, right};
            const std::size_t expected = raw_messages.size() + 1;
            joint_pub->publish(joints);
            if (!spin_until(
                    executor,
                    [&] { return raw_messages.size() >= expected && corrected_messages.size() >= expected; },
                    std::chrono::seconds(5))) {
                throw std::runtime_error("timed out waiting for odometry");
            }
        };

        publish_positions(0.0, 0.0);
        for (const odomcal::WheelDelta& command : run.encoders) {
            phi_l += command.dphi_l;
            phi_r += command.dphi_r;
            publish_positions(phi_l, phi_r);
        }

        if (raw_messages.size() != run.encoders.size() + 1 || corrected_messages.size() != raw_messages.size()) {
            throw std::runtime_error("odometry message count does not match the encoder log");
        }

        const nav_msgs::msg::Odometry& raw_end = raw_messages.back();
        const nav_msgs::msg::Odometry& corrected_end = corrected_messages.back();
        const odomcal::Pose& ground_truth = run.actual.back();
        const odomcal::Pose& library_raw = run.odom.back();
        const double raw_yaw = yaw_from_quaternion(raw_end.pose.pose.orientation);
        const double corrected_yaw = yaw_from_quaternion(corrected_end.pose.pose.orientation);

        require_close("raw x", raw_end.pose.pose.position.x, library_raw.x);
        require_close("raw y", raw_end.pose.pose.position.y, library_raw.y);
        require_close("raw yaw", raw_yaw, library_raw.theta);
        require_close("corrected x", corrected_end.pose.pose.position.x, ground_truth.x);
        require_close("corrected y", corrected_end.pose.pose.position.y, ground_truth.y);
        require_close("corrected yaw", corrected_yaw, ground_truth.theta);

        const double raw_endpoint_error = std::hypot(ground_truth.x - raw_end.pose.pose.position.x,
                                                     ground_truth.y - raw_end.pose.pose.position.y);
        const double corrected_endpoint_error = std::hypot(ground_truth.x - corrected_end.pose.pose.position.x,
                                                           ground_truth.y - corrected_end.pose.pose.position.y);
        const double raw_heading_error = std::abs(odomcal::wrap_angle(ground_truth.theta - raw_yaw));
        const double corrected_heading_error = std::abs(odomcal::wrap_angle(ground_truth.theta - corrected_yaw));
        const double raw_vs_corrected = std::hypot(raw_end.pose.pose.position.x - corrected_end.pose.pose.position.x,
                                                   raw_end.pose.pose.position.y - corrected_end.pose.pose.position.y);
        const double rad_to_deg = 180.0 / odomcal::kPi;

        std::ofstream output(output_path);
        if (!output) {
            throw std::runtime_error("could not write " + output_path);
        }
        output << "# simulated replay, not a physical robot\n";
        output << "field,value\n";
        write_field(output, "kind", "simulated_replay");
        write_field(output, "node", "odomcal_node");
        write_field(output, "motion", "ccw_square");
        write_field(output, "side_m", kSideM);
        write_field(output, "steps_per_segment", kStepsPerSegment);
        write_field(output, "noise_std_rad", kNoiseStdRad);
        write_field(output, "ed", kEd);
        write_field(output, "eb", kEb);
        write_field(output, "nominal_r_l_m", nominal.r_l);
        write_field(output, "nominal_r_r_m", nominal.r_r);
        write_field(output, "nominal_track_width_m", nominal.b);
        write_field(output, "plant_r_l_m", plant.r_l);
        write_field(output, "plant_r_r_m", plant.r_r);
        write_field(output, "plant_track_width_m", plant.b);
        write_field(output, "raw_parameter_r_l_m", raw_node->get_parameter("r_l").as_double());
        write_field(output, "raw_parameter_r_r_m", raw_node->get_parameter("r_r").as_double());
        write_field(output, "raw_parameter_track_width_m", raw_node->get_parameter("track_width").as_double());
        write_field(output, "corrected_parameter_r_l_m", corrected_node->get_parameter("r_l").as_double());
        write_field(output, "corrected_parameter_r_r_m", corrected_node->get_parameter("r_r").as_double());
        write_field(output, "corrected_parameter_track_width_m",
                    corrected_node->get_parameter("track_width").as_double());
        write_field(output, "joint_messages", stamp_index);
        write_field(output, "raw_odom_messages", static_cast<int>(raw_messages.size()));
        write_field(output, "corrected_odom_messages", static_cast<int>(corrected_messages.size()));
        write_field(output, "ground_truth_x_m", ground_truth.x);
        write_field(output, "ground_truth_y_m", ground_truth.y);
        write_field(output, "ground_truth_theta_rad", ground_truth.theta);
        write_field(output, "raw_x_m", raw_end.pose.pose.position.x);
        write_field(output, "raw_y_m", raw_end.pose.pose.position.y);
        write_field(output, "raw_theta_rad", raw_yaw);
        write_field(output, "corrected_x_m", corrected_end.pose.pose.position.x);
        write_field(output, "corrected_y_m", corrected_end.pose.pose.position.y);
        write_field(output, "corrected_theta_rad", corrected_yaw);
        write_field(output, "raw_endpoint_error_m", raw_endpoint_error);
        write_field(output, "corrected_endpoint_error_m", corrected_endpoint_error);
        write_field(output, "raw_heading_error_rad", raw_heading_error);
        write_field(output, "corrected_heading_error_rad", corrected_heading_error);
        write_field(output, "raw_heading_error_deg", raw_heading_error * rad_to_deg);
        write_field(output, "corrected_heading_error_deg", corrected_heading_error * rad_to_deg);
        write_field(output, "raw_vs_corrected_endpoint_m", raw_vs_corrected);
        output.flush();
        if (!output) {
            throw std::runtime_error("failed while writing " + output_path);
        }
        std::cout << "wrote " << output_path << '\n';
        std::filesystem::remove(calibration_path);
    } catch (const std::exception& ex) {
        std::cerr << ex.what() << '\n';
        exit_code = 1;
    }

    rclcpp::shutdown();
    return exit_code;
}
