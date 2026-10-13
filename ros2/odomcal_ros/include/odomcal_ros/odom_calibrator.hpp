#pragma once

#include "odomcal/diff_drive.hpp"

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <tf2_ros/transform_broadcaster.hpp>

#include <memory>
#include <mutex>
#include <string>

namespace odomcal_ros {

class OdomCalibrator : public rclcpp::Node {
public:
    explicit OdomCalibrator(const rclcpp::NodeOptions& options = rclcpp::NodeOptions(),
                            const std::string& node_name = "odom_calibrator");

private:
    void on_joint_state(const sensor_msgs::msg::JointState::SharedPtr msg);
    rcl_interfaces::msg::SetParametersResult on_parameters(const std::vector<rclcpp::Parameter>& parameters);
    void publish(const rclcpp::Time& stamp, double dt, double ds, double dth);

    rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_sub_;
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;
    std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
    rclcpp::ServiceBase::SharedPtr set_calibration_srv_;
    rclcpp::ServiceBase::SharedPtr reload_calibration_srv_;
    OnSetParametersCallbackHandle::SharedPtr parameter_handle_;

    std::mutex mutex_;
    double r_l_ = 0.0;
    double r_r_ = 0.0;
    double b_ = 0.0;
    std::string left_joint_;
    std::string right_joint_;
    bool use_velocities_ = false;
    std::string odom_frame_;
    std::string base_frame_;
    bool publish_tf_ = true;

    bool have_sample_ = false;
    double prev_left_ = 0.0;
    double prev_right_ = 0.0;
    rclcpp::Time last_stamp_{0, 0, RCL_ROS_TIME};
    odomcal::Pose pose_{};
};

}  // namespace odomcal_ros
