#include "odomcal_ros/odom_calibrator.hpp"

#include <rclcpp/rclcpp.hpp>

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<odomcal_ros::OdomCalibrator>());
    rclcpp::shutdown();
    return 0;
}
