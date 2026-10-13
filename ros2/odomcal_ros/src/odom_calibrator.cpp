#include "odomcal_ros/odom_calibrator.hpp"

#include "odomcal_ros/srv/set_calibration.hpp"

#include <std_srvs/srv/trigger.hpp>

#include <algorithm>
#include <cctype>
#include <cmath>
#include <cstdlib>
#include <fstream>
#include <iterator>
#include <optional>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

namespace odomcal_ros {
namespace {

struct Calibration {
    double r_l = 0.0;
    double r_r = 0.0;
    double track_width = 0.0;
};

std::string trim(std::string text) {
    auto not_space = [](unsigned char c) { return std::isspace(c) == 0; };
    text.erase(text.begin(), std::find_if(text.begin(), text.end(), not_space));
    text.erase(std::find_if(text.rbegin(), text.rend(), not_space).base(), text.end());
    return text;
}

Calibration read_calibration_file(const std::string& path) {
    std::ifstream input(path);
    if (!input) {
        throw std::runtime_error("could not open calibration file: " + path);
    }

    Calibration calibration;
    bool have_l = false;
    bool have_r = false;
    bool have_b = false;
    std::string line;
    int line_number = 0;
    while (std::getline(input, line)) {
        ++line_number;
        const auto comment = line.find('#');
        if (comment != std::string::npos) {
            line.erase(comment);
        }
        line = trim(std::move(line));
        if (line.empty()) {
            continue;
        }
        const auto colon = line.find(':');
        if (colon == std::string::npos) {
            throw std::runtime_error("calibration file " + path + " line " + std::to_string(line_number) +
                                     " is not a key: value pair");
        }
        const std::string key = trim(line.substr(0, colon));
        const std::string value_text = trim(line.substr(colon + 1));
        char* end = nullptr;
        const double value = std::strtod(value_text.c_str(), &end);
        if (end == value_text.c_str() || *end != '\0') {
            throw std::runtime_error("calibration file " + path + " has a non-numeric value for " + key);
        }
        if (key == "r_l") {
            calibration.r_l = value;
            have_l = true;
        } else if (key == "r_r") {
            calibration.r_r = value;
            have_r = true;
        } else if (key == "track_width" || key == "b") {
            calibration.track_width = value;
            have_b = true;
        } else {
            throw std::runtime_error("calibration file " + path + " has unknown key " + key);
        }
    }
    if (!have_l || !have_r || !have_b) {
        throw std::runtime_error("calibration file " + path + " must set r_l, r_r, and track_width");
    }
    if (!(calibration.r_l > 0.0) || !(calibration.r_r > 0.0) || !(calibration.track_width > 0.0)) {
        throw std::runtime_error("calibration file " + path + " radii and track width must be positive");
    }
    return calibration;
}

std::optional<std::size_t> find_joint(const std::vector<std::string>& names, const std::string& wanted) {
    const auto found = std::find(names.begin(), names.end(), wanted);
    if (found == names.end()) {
        return std::nullopt;
    }
    return static_cast<std::size_t>(std::distance(names.begin(), found));
}

geometry_msgs::msg::Quaternion yaw_quaternion(double yaw) {
    geometry_msgs::msg::Quaternion quaternion;
    quaternion.x = 0.0;
    quaternion.y = 0.0;
    quaternion.z = std::sin(yaw * 0.5);
    quaternion.w = std::cos(yaw * 0.5);
    return quaternion;
}

void copy_result(const rcl_interfaces::msg::SetParametersResult& result, bool* success, std::string* message) {
    *success = result.successful;
    if (result.successful) {
        *message = "applied calibration";
    } else {
        *message = result.reason.empty() ? "rejected calibration" : result.reason;
    }
}

}  // namespace

OdomCalibrator::OdomCalibrator(const rclcpp::NodeOptions& options, const std::string& node_name)
    : rclcpp::Node(node_name, options) {
    declare_parameter<std::string>("joint_states_topic", "joint_states");
    declare_parameter<std::string>("left_wheel_joint", "left_wheel_joint");
    declare_parameter<std::string>("right_wheel_joint", "right_wheel_joint");
    declare_parameter<bool>("use_velocities", false);
    declare_parameter<std::string>("odom_topic", "odom");
    declare_parameter<std::string>("odom_frame", "odom");
    declare_parameter<std::string>("base_frame", "base_link");
    declare_parameter<bool>("publish_tf", true);
    declare_parameter<std::string>("calibration_file", "");
    declare_parameter<double>("r_l", 0.05);
    declare_parameter<double>("r_r", 0.05);
    declare_parameter<double>("track_width", 0.30);

    left_joint_ = get_parameter("left_wheel_joint").as_string();
    right_joint_ = get_parameter("right_wheel_joint").as_string();
    use_velocities_ = get_parameter("use_velocities").as_bool();
    odom_frame_ = get_parameter("odom_frame").as_string();
    base_frame_ = get_parameter("base_frame").as_string();
    publish_tf_ = get_parameter("publish_tf").as_bool();
    r_l_ = get_parameter("r_l").as_double();
    r_r_ = get_parameter("r_r").as_double();
    b_ = get_parameter("track_width").as_double();

    const std::string calibration_file = get_parameter("calibration_file").as_string();
    if (!calibration_file.empty()) {
        const Calibration loaded = read_calibration_file(calibration_file);
        r_l_ = loaded.r_l;
        r_r_ = loaded.r_r;
        b_ = loaded.track_width;
        set_parameters_atomically({
            rclcpp::Parameter("r_l", r_l_),
            rclcpp::Parameter("r_r", r_r_),
            rclcpp::Parameter("track_width", b_),
        });
    }
    if (!(r_l_ > 0.0) || !(r_r_ > 0.0) || !(b_ > 0.0)) {
        throw std::invalid_argument("wheel radii and track width must be positive");
    }

    parameter_handle_ = add_on_set_parameters_callback(
        [this](const std::vector<rclcpp::Parameter>& parameters) { return on_parameters(parameters); });

    const auto qos = rclcpp::QoS(rclcpp::KeepLast(50)).reliable();
    joint_sub_ = create_subscription<sensor_msgs::msg::JointState>(
        get_parameter("joint_states_topic").as_string(),
        qos,
        [this](const sensor_msgs::msg::JointState::SharedPtr msg) { on_joint_state(msg); });
    odom_pub_ = create_publisher<nav_msgs::msg::Odometry>(get_parameter("odom_topic").as_string(), qos);
    tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);

    set_calibration_srv_ = create_service<odomcal_ros::srv::SetCalibration>(
        "~/set_calibration",
        [this](const std::shared_ptr<odomcal_ros::srv::SetCalibration::Request> request,
               std::shared_ptr<odomcal_ros::srv::SetCalibration::Response> response) {
            const auto result = set_parameters_atomically({
                rclcpp::Parameter("r_l", request->r_l),
                rclcpp::Parameter("r_r", request->r_r),
                rclcpp::Parameter("track_width", request->track_width),
            });
            copy_result(result, &response->success, &response->message);
        });
    reload_calibration_srv_ = create_service<std_srvs::srv::Trigger>(
        "~/reload_calibration",
        [this](const std::shared_ptr<std_srvs::srv::Trigger::Request> /*request*/,
               std::shared_ptr<std_srvs::srv::Trigger::Response> response) {
            const std::string path = get_parameter("calibration_file").as_string();
            if (path.empty()) {
                response->success = false;
                response->message = "calibration_file is empty";
                return;
            }
            Calibration loaded;
            try {
                loaded = read_calibration_file(path);
            } catch (const std::exception& ex) {
                response->success = false;
                response->message = ex.what();
                return;
            }
            const auto result = set_parameters_atomically({
                rclcpp::Parameter("r_l", loaded.r_l),
                rclcpp::Parameter("r_r", loaded.r_r),
                rclcpp::Parameter("track_width", loaded.track_width),
            });
            copy_result(result, &response->success, &response->message);
        });

    RCLCPP_INFO(get_logger(),
                "wheel radii r_l=%.8f r_r=%.8f track_width=%.8f joints=%s,%s use_velocities=%s",
                r_l_,
                r_r_,
                b_,
                left_joint_.c_str(),
                right_joint_.c_str(),
                use_velocities_ ? "true" : "false");
}

rcl_interfaces::msg::SetParametersResult OdomCalibrator::on_parameters(
    const std::vector<rclcpp::Parameter>& parameters) {
    std::lock_guard<std::mutex> lock(mutex_);
    double r_l = r_l_;
    double r_r = r_r_;
    double track_width = b_;
    rcl_interfaces::msg::SetParametersResult result;
    result.successful = true;
    try {
        for (const rclcpp::Parameter& parameter : parameters) {
            if (parameter.get_name() == "r_l") {
                r_l = parameter.as_double();
            } else if (parameter.get_name() == "r_r") {
                r_r = parameter.as_double();
            } else if (parameter.get_name() == "track_width") {
                track_width = parameter.as_double();
            }
        }
    } catch (const rclcpp::ParameterTypeException& ex) {
        result.successful = false;
        result.reason = ex.what();
        return result;
    }
    if (!(r_l > 0.0) || !(r_r > 0.0) || !(track_width > 0.0)) {
        result.successful = false;
        result.reason = "wheel radii and track width must be positive";
        return result;
    }
    const bool changed = r_l != r_l_ || r_r != r_r_ || track_width != b_;
    r_l_ = r_l;
    r_r_ = r_r;
    b_ = track_width;
    if (changed) {
        RCLCPP_INFO(get_logger(),
                    "applied calibration r_l=%.8f r_r=%.8f track_width=%.8f",
                    r_l_,
                    r_r_,
                    b_);
    }
    result.reason = "applied calibration";
    return result;
}

void OdomCalibrator::on_joint_state(const sensor_msgs::msg::JointState::SharedPtr msg) {
    const auto left_index = find_joint(msg->name, left_joint_);
    const auto right_index = find_joint(msg->name, right_joint_);
    if (!left_index || !right_index) {
        RCLCPP_WARN_THROTTLE(get_logger(),
                             *get_clock(),
                             5000,
                             "joint state is missing '%s' or '%s'",
                             left_joint_.c_str(),
                             right_joint_.c_str());
        return;
    }

    const rclcpp::Time stamp(msg->header.stamp, RCL_ROS_TIME);
    std::lock_guard<std::mutex> lock(mutex_);

    double dphi_l = 0.0;
    double dphi_r = 0.0;
    bool have_delta = false;
    if (use_velocities_) {
        if (*left_index >= msg->velocity.size() || *right_index >= msg->velocity.size()) {
            RCLCPP_WARN_THROTTLE(get_logger(),
                                 *get_clock(),
                                 5000,
                                 "joint state has no velocity for '%s' or '%s'",
                                 left_joint_.c_str(),
                                 right_joint_.c_str());
            return;
        }
        if (have_sample_) {
            const double dt = (stamp - last_stamp_).seconds();
            if (dt > 0.0) {
                dphi_l = msg->velocity[*left_index] * dt;
                dphi_r = msg->velocity[*right_index] * dt;
                have_delta = true;
            }
        }
    } else {
        if (*left_index >= msg->position.size() || *right_index >= msg->position.size()) {
            RCLCPP_WARN_THROTTLE(get_logger(),
                                 *get_clock(),
                                 5000,
                                 "joint state has no position for '%s' or '%s'",
                                 left_joint_.c_str(),
                                 right_joint_.c_str());
            return;
        }
        const double left = msg->position[*left_index];
        const double right = msg->position[*right_index];
        if (have_sample_) {
            dphi_l = left - prev_left_;
            dphi_r = right - prev_right_;
            have_delta = true;
        }
        prev_left_ = left;
        prev_right_ = right;
    }

    double dt = 0.0;
    if (have_sample_) {
        dt = (stamp - last_stamp_).seconds();
    }
    last_stamp_ = stamp;
    have_sample_ = true;

    if (!have_delta) {
        publish(stamp, 0.0, 0.0, 0.0);
        return;
    }

    const odomcal::DiffDriveParams params{r_l_, r_r_, b_};
    try {
        pose_ = odomcal::integrate(pose_, dphi_l, dphi_r, params);
    } catch (const std::exception& ex) {
        RCLCPP_ERROR(get_logger(), "odometry step rejected: %s", ex.what());
        return;
    }
    const double ds = 0.5 * (params.r_r * dphi_r + params.r_l * dphi_l);
    const double dth = (params.r_r * dphi_r - params.r_l * dphi_l) / params.b;
    publish(stamp, dt, ds, dth);
}

void OdomCalibrator::publish(const rclcpp::Time& stamp, double dt, double ds, double dth) {
    const double vx = dt > 0.0 ? ds / dt : 0.0;
    const double wz = dt > 0.0 ? dth / dt : 0.0;
    const geometry_msgs::msg::Quaternion quaternion = yaw_quaternion(pose_.theta);

    nav_msgs::msg::Odometry odometry;
    odometry.header.stamp = stamp;
    odometry.header.frame_id = odom_frame_;
    odometry.child_frame_id = base_frame_;
    odometry.pose.pose.position.x = pose_.x;
    odometry.pose.pose.position.y = pose_.y;
    odometry.pose.pose.position.z = 0.0;
    odometry.pose.pose.orientation = quaternion;
    odometry.twist.twist.linear.x = vx;
    odometry.twist.twist.linear.y = 0.0;
    odometry.twist.twist.linear.z = 0.0;
    odometry.twist.twist.angular.x = 0.0;
    odometry.twist.twist.angular.y = 0.0;
    odometry.twist.twist.angular.z = wz;
    odom_pub_->publish(odometry);

    if (!publish_tf_) {
        return;
    }
    geometry_msgs::msg::TransformStamped transform;
    transform.header = odometry.header;
    transform.child_frame_id = base_frame_;
    transform.transform.translation.x = pose_.x;
    transform.transform.translation.y = pose_.y;
    transform.transform.translation.z = 0.0;
    transform.transform.rotation = quaternion;
    tf_broadcaster_->sendTransform(transform);
}

}  // namespace odomcal_ros
