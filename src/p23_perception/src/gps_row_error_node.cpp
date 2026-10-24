#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <p23_perception/msg/row_error.hpp>
#include <vector>
#include <cmath>
#include <limits>

class GpsRowErrorNode : public rclcpp::Node
{
public:
  GpsRowErrorNode()
  : Node("gps_row_error_node"), latest_odom_(nullptr)
  {
    declare_parameter<int>("row_count", 10);
    declare_parameter<double>("row_spacing", 1.0);
    declare_parameter<double>("row_length", 10.0);
    declare_parameter<double>("row_start_offset", 0.0);
    declare_parameter<double>("row_direction", 0.0);

    int row_count        = get_parameter("row_count").as_int();
    row_spacing_         = get_parameter("row_spacing").as_double();
    row_length_          = get_parameter("row_length").as_double();
    row_start_offset_    = get_parameter("row_start_offset").as_double();
    row_direction_       = get_parameter("row_direction").as_double();

    for (int i = 0; i < row_count; ++i) {
      row_centerlines_.push_back(row_start_offset_ + i * row_spacing_);
    }

    RCLCPP_INFO(get_logger(), "Row centerlines (Y):");
    for (size_t i = 0; i < row_centerlines_.size(); ++i) {
      RCLCPP_INFO(get_logger(), "  Row %zu: Y = %.4f", i, row_centerlines_[i]);
    }

    publisher_ = create_publisher<p23_perception::msg::RowError>("/row_error/gps", 10);

    subscription_ = create_subscription<nav_msgs::msg::Odometry>(
      "/odometry/filtered", rclcpp::QoS(10),
      [this](nav_msgs::msg::Odometry::SharedPtr msg) { latest_odom_ = msg; });

    // 50 Hz timer
    timer_ = create_wall_timer(
      std::chrono::milliseconds(20),
      std::bind(&GpsRowErrorNode::compute, this));
  }

private:
  void compute()
  {
    if (!latest_odom_) {
      return;
    }

    const double robot_x = latest_odom_->pose.pose.position.x;
    const double robot_y = latest_odom_->pose.pose.position.y;

    tf2::Quaternion q(
      latest_odom_->pose.pose.orientation.x,
      latest_odom_->pose.pose.orientation.y,
      latest_odom_->pose.pose.orientation.z,
      latest_odom_->pose.pose.orientation.w);
    double roll, pitch, yaw;
    tf2::Matrix3x3(q).getRPY(roll, pitch, yaw);

    // Row assignment: minimum |robot_Y - centerline_Y|
    size_t best_row = 0;
    double min_diff = std::numeric_limits<double>::max();
    for (size_t i = 0; i < row_centerlines_.size(); ++i) {
      double diff = std::abs(robot_y - row_centerlines_[i]);
      if (diff < min_diff) {
        min_diff = diff;
        best_row = i;
      }
    }

    const double lateral_error = robot_y - row_centerlines_[best_row];
    const double heading_error = yaw - row_direction_;

    const double y_min = row_start_offset_ - row_spacing_ / 2.0;
    const double y_max = row_start_offset_ +
      (static_cast<double>(row_centerlines_.size()) - 1) * row_spacing_ +
      row_spacing_ / 2.0;
    const bool row_valid =
      (robot_x >= -0.5 && robot_x <= row_length_) &&
      (robot_y >= y_min  && robot_y <= y_max);

    p23_perception::msg::RowError out;
    out.lateral_error = lateral_error;
    out.heading_error = heading_error;
    out.row_valid     = row_valid;
    out.confidence    = 1.0;
    publisher_->publish(out);
  }

  std::vector<double> row_centerlines_;
  double row_spacing_{1.0};
  double row_length_{10.0};
  double row_start_offset_{0.0};
  double row_direction_{0.0};

  nav_msgs::msg::Odometry::SharedPtr latest_odom_;
  rclcpp::Publisher<p23_perception::msg::RowError>::SharedPtr publisher_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr subscription_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<GpsRowErrorNode>());
  rclcpp::shutdown();
  return 0;
}
