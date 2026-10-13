#include "odomcal_ros/odom_calibrator.hpp"

#include "odomcal/diff_drive.hpp"
#include "odomcal/simulator.hpp"
#include "odomcal_ros/srv/set_calibration.hpp"

#include <gtest/gtest.h>
#include <nav_msgs/msg/odometry.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <std_srvs/srv/trigger.hpp>
#include <tf2_msgs/msg/tf_message.hpp>
#include <tf2_ros/qos.hpp>

#include <chrono>
#include <cmath>
#include <fstream>
#include <functional>
#include <memory>
#include <sstream>
#include <string>
#include <vector>

namespace {

using namespace std::chrono_literals;

double yaw_from_quaternion(const geometry_msgs::msg::Quaternion& quaternion) {
    const double sin_yaw = 2.0 * (quaternion.w * quaternion.z + quaternion.x * quaternion.y);
    const double cos_yaw = 1.0 - 2.0 * (quaternion.y * quaternion.y + quaternion.z * quaternion.z);
    return std::atan2(sin_yaw, cos_yaw);
}

builtin_interfaces::msg::Time stamp_at(int64_t nanoseconds) {
    builtin_interfaces::msg::Time stamp;
    stamp.sec = static_cast<int32_t>(nanoseconds / 1000000000LL);
    stamp.nanosec = static_cast<uint32_t>(nanoseconds % 1000000000LL);
    return stamp;
}

class Harness {
public:
    explicit Harness(const std::string& name, const rclcpp::NodeOptions& calibrator_options)
        : calibrator_(std::make_shared<odomcal_ros::OdomCalibrator>(calibrator_options, name)),
          helper_(std::make_shared<rclcpp::Node>(name + "_helper")) {
        const auto qos = rclcpp::QoS(rclcpp::KeepLast(20)).reliable();
        publisher_ = helper_->create_publisher<sensor_msgs::msg::JointState>(
            calibrator_->get_parameter("joint_states_topic").as_string(),
            qos);
        odom_sub_ = helper_->create_subscription<nav_msgs::msg::Odometry>(
            calibrator_->get_parameter("odom_topic").as_string(),
            qos,
            [this](const nav_msgs::msg::Odometry::SharedPtr msg) { odometry_.push_back(*msg); });
        tf_sub_ = helper_->create_subscription<tf2_msgs::msg::TFMessage>(
            "/tf",
            tf2_ros::DynamicListenerQoS(),
            [this](const tf2_msgs::msg::TFMessage::SharedPtr msg) {
                for (const auto& transform : msg->transforms) {
                    transforms_.push_back(transform);
                }
            });
        executor_.add_node(calibrator_);
        executor_.add_node(helper_);
    }

    bool wait_for(const std::function<bool()>& done, std::chrono::milliseconds timeout = 10s) {
        const auto deadline = std::chrono::steady_clock::now() + timeout;
        while (std::chrono::steady_clock::now() < deadline) {
            executor_.spin_some(50ms);
            if (done()) {
                return true;
            }
        }
        executor_.spin_some(50ms);
        return done();
    }

    void publish(const sensor_msgs::msg::JointState& message) {
        const std::size_t expected = odometry_.size() + 1;
        publisher_->publish(message);
        ASSERT_TRUE(wait_for([&] { return odometry_.size() >= expected; }));
    }

    std::shared_ptr<odomcal_ros::OdomCalibrator> calibrator_;
    std::shared_ptr<rclcpp::Node> helper_;
    rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr publisher_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
    rclcpp::Subscription<tf2_msgs::msg::TFMessage>::SharedPtr tf_sub_;
    rclcpp::executors::SingleThreadedExecutor executor_;
    std::vector<nav_msgs::msg::Odometry> odometry_;
    std::vector<geometry_msgs::msg::TransformStamped> transforms_;
};

rclcpp::NodeOptions geometry_options(const std::string& name, bool use_velocities, double radius) {
    rclcpp::NodeOptions options;
    options.append_parameter_override("joint_states_topic", "/joint_states_" + name);
    options.append_parameter_override("odom_topic", "/odom_" + name);
    options.append_parameter_override("left_wheel_joint", std::string("left_wheel_joint"));
    options.append_parameter_override("right_wheel_joint", std::string("right_wheel_joint"));
    options.append_parameter_override("use_velocities", use_velocities);
    options.append_parameter_override("publish_tf", true);
    options.append_parameter_override("calibration_file", std::string(""));
    options.append_parameter_override("r_l", radius);
    options.append_parameter_override("r_r", radius);
    options.append_parameter_override("track_width", 0.30);
    options.append_parameter_override("odom_frame", std::string("odom"));
    options.append_parameter_override("base_frame", std::string("base_link"));
    return options;
}

sensor_msgs::msg::JointState positions_at(int64_t nanoseconds, double left, double right, bool swap_names) {
    sensor_msgs::msg::JointState message;
    message.header.stamp = stamp_at(nanoseconds);
    if (swap_names) {
        message.name = {"right_wheel_joint", "left_wheel_joint"};
        message.position = {right, left};
    } else {
        message.name = {"left_wheel_joint", "right_wheel_joint"};
        message.position = {left, right};
    }
    return message;
}

std::string service_name(const rclcpp::Node& node, const std::string& suffix) {
    return std::string(node.get_fully_qualified_name()) + "/" + suffix;
}

std::string service_list(const rclcpp::Node& node) {
    std::ostringstream listed;
    for (const auto& item : node.get_service_names_and_types()) {
        listed << item.first << " ";
    }
    return listed.str();
}

sensor_msgs::msg::JointState velocities_at(int64_t nanoseconds, double left, double right) {
    sensor_msgs::msg::JointState message;
    message.header.stamp = stamp_at(nanoseconds);
    message.name = {"left_wheel_joint", "right_wheel_joint"};
    message.velocity = {left, right};
    return message;
}

}  // namespace

class OdomCalibratorTest : public ::testing::Test {
protected:
    static void SetUpTestSuite() {
        if (!rclcpp::ok()) {
            rclcpp::init(0, nullptr);
        }
    }

    static void TearDownTestSuite() {
        if (rclcpp::ok()) {
            rclcpp::shutdown();
        }
    }
};

TEST_F(OdomCalibratorTest, StraightLineFromJointPositions) {
    Harness harness("straight_positions", geometry_options("straight_positions", false, 0.05));
    ASSERT_TRUE(harness.wait_for([&] { return harness.publisher_->get_subscription_count() >= 1; }));

    harness.publish(positions_at(0, 0.0, 0.0, false));
    harness.publish(positions_at(1000000000LL, 20.0, 20.0, false));

    ASSERT_GE(harness.odometry_.size(), 2u);
    const auto& end = harness.odometry_.back();
    EXPECT_EQ(end.header.frame_id, "odom");
    EXPECT_EQ(end.child_frame_id, "base_link");
    EXPECT_NEAR(end.pose.pose.position.x, 1.0, 1e-9);
    EXPECT_NEAR(end.pose.pose.position.y, 0.0, 1e-9);
    EXPECT_NEAR(yaw_from_quaternion(end.pose.pose.orientation), 0.0, 1e-9);
    EXPECT_NEAR(end.twist.twist.linear.x, 1.0, 1e-9);
    EXPECT_NEAR(end.twist.twist.angular.z, 0.0, 1e-9);

    ASSERT_TRUE(harness.wait_for([&] { return !harness.transforms_.empty(); }));
    const auto& transform = harness.transforms_.back();
    EXPECT_EQ(transform.header.frame_id, "odom");
    EXPECT_EQ(transform.child_frame_id, "base_link");
    EXPECT_NEAR(transform.transform.translation.x, 1.0, 1e-9);
    EXPECT_NEAR(transform.transform.translation.y, 0.0, 1e-9);
}

TEST_F(OdomCalibratorTest, JointOrderDoesNotMatter) {
    Harness harness("swapped_joints", geometry_options("swapped_joints", false, 0.05));
    ASSERT_TRUE(harness.wait_for([&] { return harness.publisher_->get_subscription_count() >= 1; }));
    harness.publish(positions_at(0, 0.0, 0.0, true));
    harness.publish(positions_at(1000000000LL, 20.0, 20.0, true));
    const auto& end = harness.odometry_.back();
    EXPECT_NEAR(end.pose.pose.position.x, 1.0, 1e-9);
    EXPECT_NEAR(end.pose.pose.position.y, 0.0, 1e-9);
}

TEST_F(OdomCalibratorTest, StraightLineFromJointVelocities) {
    Harness harness("straight_velocities", geometry_options("straight_velocities", true, 0.05));
    ASSERT_TRUE(harness.wait_for([&] { return harness.publisher_->get_subscription_count() >= 1; }));
    harness.publish(velocities_at(0, 20.0, 20.0));
    harness.publish(velocities_at(1000000000LL, 20.0, 20.0));
    const auto& end = harness.odometry_.back();
    EXPECT_NEAR(end.pose.pose.position.x, 1.0, 1e-9);
    EXPECT_NEAR(end.pose.pose.position.y, 0.0, 1e-9);
    EXPECT_NEAR(end.twist.twist.linear.x, 1.0, 1e-9);
}

TEST_F(OdomCalibratorTest, QuarterTurnInPlace) {
    Harness harness("quarter_turn", geometry_options("quarter_turn", false, 0.05));
    ASSERT_TRUE(harness.wait_for([&] { return harness.publisher_->get_subscription_count() >= 1; }));
    const double yaw = odomcal::kPi / 2.0;
    const double dphi_r = (yaw * 0.30 / 2.0) / 0.05;
    harness.publish(positions_at(0, 0.0, 0.0, false));
    harness.publish(positions_at(500000000LL, -dphi_r, dphi_r, false));
    const auto& end = harness.odometry_.back();
    EXPECT_NEAR(end.pose.pose.position.x, 0.0, 1e-9);
    EXPECT_NEAR(end.pose.pose.position.y, 0.0, 1e-9);
    EXPECT_NEAR(yaw_from_quaternion(end.pose.pose.orientation), yaw, 1e-9);
    EXPECT_NEAR(end.twist.twist.angular.z, yaw / 0.5, 1e-9);
}

TEST_F(OdomCalibratorTest, SetCalibrationScalesTheNextStep) {
    Harness harness("set_calibration", geometry_options("set_calibration", false, 0.05));
    ASSERT_TRUE(harness.wait_for([&] { return harness.publisher_->get_subscription_count() >= 1; }));
    harness.publish(positions_at(0, 0.0, 0.0, false));
    harness.publish(positions_at(1000000000LL, 20.0, 20.0, false));
    EXPECT_NEAR(harness.odometry_.back().pose.pose.position.x, 1.0, 1e-9);

    auto client = harness.helper_->create_client<odomcal_ros::srv::SetCalibration>(
        service_name(*harness.calibrator_, "set_calibration"));
    auto request = std::make_shared<odomcal_ros::srv::SetCalibration::Request>();
    request->r_l = 0.10;
    request->r_r = 0.10;
    request->track_width = 0.30;
    auto future = client->async_send_request(request);
    ASSERT_TRUE(harness.wait_for([&] { return future.wait_for(0s) == std::future_status::ready; }))
        << service_list(*harness.calibrator_);
    EXPECT_TRUE(future.get()->success);

    harness.publish(positions_at(2000000000LL, 40.0, 40.0, false));
    EXPECT_NEAR(harness.odometry_.back().pose.pose.position.x, 3.0, 1e-9);
    EXPECT_NEAR(harness.calibrator_->get_parameter("r_l").as_double(), 0.10, 1e-12);
}

TEST_F(OdomCalibratorTest, RejectsNonPositiveCalibration) {
    Harness harness("reject_calibration", geometry_options("reject_calibration", false, 0.05));
    auto client = harness.helper_->create_client<odomcal_ros::srv::SetCalibration>(
        service_name(*harness.calibrator_, "set_calibration"));
    auto request = std::make_shared<odomcal_ros::srv::SetCalibration::Request>();
    request->r_l = -0.05;
    request->r_r = 0.05;
    request->track_width = 0.30;
    auto future = client->async_send_request(request);
    ASSERT_TRUE(harness.wait_for([&] { return future.wait_for(0s) == std::future_status::ready; }))
        << service_list(*harness.calibrator_);
    EXPECT_FALSE(future.get()->success);
    EXPECT_NEAR(harness.calibrator_->get_parameter("r_l").as_double(), 0.05, 1e-12);
}

TEST_F(OdomCalibratorTest, ReloadCalibrationFile) {
    const std::string path = "/tmp/odomcal_reload_calibration.yaml";
    {
        std::ofstream file(path);
        ASSERT_TRUE(file);
        file << "r_l: 0.05\nr_r: 0.05\ntrack_width: 0.30\n";
    }
    rclcpp::NodeOptions options = geometry_options("reload_calibration", false, 0.05);
    options.append_parameter_override("calibration_file", path);
    Harness harness("reload_calibration", options);
    EXPECT_NEAR(harness.calibrator_->get_parameter("r_l").as_double(), 0.05, 1e-12);

    {
        std::ofstream file(path);
        ASSERT_TRUE(file);
        file << "# updated radii\nr_l: 0.10\nr_r: 0.10\nb: 0.30\n";
    }
    auto client =
        harness.helper_->create_client<std_srvs::srv::Trigger>(service_name(*harness.calibrator_, "reload_calibration"));
    auto future = client->async_send_request(std::make_shared<std_srvs::srv::Trigger::Request>());
    ASSERT_TRUE(harness.wait_for([&] { return future.wait_for(0s) == std::future_status::ready; }))
        << service_list(*harness.calibrator_);
    EXPECT_TRUE(future.get()->success);
    EXPECT_NEAR(harness.calibrator_->get_parameter("r_l").as_double(), 0.10, 1e-12);
    EXPECT_NEAR(harness.calibrator_->get_parameter("track_width").as_double(), 0.30, 1e-12);

    ASSERT_TRUE(harness.wait_for([&] { return harness.publisher_->get_subscription_count() >= 1; }));
    harness.publish(positions_at(0, 0.0, 0.0, false));
    harness.publish(positions_at(1000000000LL, 10.0, 10.0, false));
    EXPECT_NEAR(harness.odometry_.back().pose.pose.position.x, 1.0, 1e-9);
}

TEST_F(OdomCalibratorTest, CalibratedSquareMatchesPlant) {
    const odomcal::DiffDriveParams nominal = odomcal::nominal_params(0.05, 0.30);
    const odomcal::DiffDriveParams plant = odomcal::params_from_ratios(0.05, 0.30, 1.002, 1.010);
    odomcal::SimConfig config;
    config.plant = plant;
    config.odometry = nominal;
    config.command_radius_m = 0.05;
    config.command_baseline_m = 0.30;
    config.side_m = 4.0;
    config.steps_per_segment = 4;
    config.noise_std_rad = 0.0;
    config.seed = 1;
    config.direction = odomcal::Direction::Ccw;
    const odomcal::SimRun run = odomcal::simulate(config);

    rclcpp::NodeOptions raw_options = geometry_options("square_raw", false, nominal.r_l);
    raw_options.append_parameter_override("joint_states_topic", std::string("/square_case/joints"));
    raw_options.append_parameter_override("odom_topic", std::string("/square_case/raw"));
    raw_options.append_parameter_override("publish_tf", false);
    rclcpp::NodeOptions corrected_options = geometry_options("square_corrected", false, plant.r_l);
    corrected_options.append_parameter_override("joint_states_topic", std::string("/square_case/joints"));
    corrected_options.append_parameter_override("odom_topic", std::string("/square_case/corrected"));
    corrected_options.append_parameter_override("r_r", plant.r_r);
    corrected_options.append_parameter_override("track_width", plant.b);
    corrected_options.append_parameter_override("publish_tf", false);

    auto raw = std::make_shared<odomcal_ros::OdomCalibrator>(raw_options, "square_raw");
    auto corrected = std::make_shared<odomcal_ros::OdomCalibrator>(corrected_options, "square_corrected");
    auto helper = std::make_shared<rclcpp::Node>("square_case_helper");
    std::vector<nav_msgs::msg::Odometry> raw_messages;
    std::vector<nav_msgs::msg::Odometry> corrected_messages;
    const auto qos = rclcpp::QoS(rclcpp::KeepLast(10)).reliable();
    auto publisher = helper->create_publisher<sensor_msgs::msg::JointState>("/square_case/joints", qos);
    auto raw_sub = helper->create_subscription<nav_msgs::msg::Odometry>(
        "/square_case/raw",
        qos,
        [&raw_messages](const nav_msgs::msg::Odometry::SharedPtr msg) { raw_messages.push_back(*msg); });
    auto corrected_sub = helper->create_subscription<nav_msgs::msg::Odometry>(
        "/square_case/corrected",
        qos,
        [&corrected_messages](const nav_msgs::msg::Odometry::SharedPtr msg) { corrected_messages.push_back(*msg); });

    rclcpp::executors::SingleThreadedExecutor executor;
    executor.add_node(raw);
    executor.add_node(corrected);
    executor.add_node(helper);
    const auto wait = [&](const std::function<bool()>& done) {
        const auto deadline = std::chrono::steady_clock::now() + 10s;
        while (std::chrono::steady_clock::now() < deadline) {
            executor.spin_some(20ms);
            if (done()) {
                return true;
            }
        }
        return done();
    };
    ASSERT_TRUE(wait([&] {
        return publisher->get_subscription_count() >= 2 && raw_sub->get_publisher_count() >= 1 &&
               corrected_sub->get_publisher_count() >= 1;
    }));

    double phi_l = 0.0;
    double phi_r = 0.0;
    int step = 0;
    auto publish_positions = [&](double left, double right) {
        sensor_msgs::msg::JointState message;
        message.header.stamp = stamp_at(static_cast<int64_t>(step) * 50000000LL);
        ++step;
        message.name = {"left_wheel_joint", "right_wheel_joint"};
        message.position = {left, right};
        const std::size_t expected = raw_messages.size() + 1;
        publisher->publish(message);
        ASSERT_TRUE(wait([&] { return raw_messages.size() >= expected && corrected_messages.size() >= expected; }));
    };
    publish_positions(0.0, 0.0);
    for (const odomcal::WheelDelta& command : run.encoders) {
        phi_l += command.dphi_l;
        phi_r += command.dphi_r;
        publish_positions(phi_l, phi_r);
    }

    ASSERT_FALSE(raw_messages.empty());
    const auto& raw_end = raw_messages.back();
    const auto& corrected_end = corrected_messages.back();
    EXPECT_NEAR(raw_end.pose.pose.position.x, run.odom.back().x, 1e-8);
    EXPECT_NEAR(raw_end.pose.pose.position.y, run.odom.back().y, 1e-8);
    EXPECT_NEAR(corrected_end.pose.pose.position.x, run.actual.back().x, 1e-8);
    EXPECT_NEAR(corrected_end.pose.pose.position.y, run.actual.back().y, 1e-8);
    const double raw_error = std::hypot(run.actual.back().x - raw_end.pose.pose.position.x,
                                         run.actual.back().y - raw_end.pose.pose.position.y);
    const double corrected_error = std::hypot(run.actual.back().x - corrected_end.pose.pose.position.x,
                                               run.actual.back().y - corrected_end.pose.pose.position.y);
    EXPECT_GT(raw_error, 0.1);
    EXPECT_LT(corrected_error, 1e-8);
}

TEST_F(OdomCalibratorTest, IgnoresUnrelatedJoints) {
    Harness harness("missing_joint", geometry_options("missing_joint", false, 0.05));
    ASSERT_TRUE(harness.wait_for([&] { return harness.publisher_->get_subscription_count() >= 1; }));
    sensor_msgs::msg::JointState message;
    message.header.stamp = stamp_at(0);
    message.name = {"caster"};
    message.position = {1.0};
    harness.publisher_->publish(message);
    EXPECT_FALSE(harness.wait_for([&] { return !harness.odometry_.empty(); }, 300ms));
}
