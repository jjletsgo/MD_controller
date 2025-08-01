#ifndef CMD_VEL_TO_RPM_HPP
#define CMD_VEL_TO_RPM_HPP

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <std_msgs/msg/int32.hpp>

class CmdVelToRpm : public rclcpp::Node
{
public:
  CmdVelToRpm();

private:
  void cmdVelCallback(const geometry_msgs::msg::Twist::SharedPtr msg);

  // ROS 인터페이스
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_sub_;
  rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr rpm_left_pub_;
  rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr rpm_right_pub_;

  // 파라미터
  double wheel_base_;   // 바퀴 간 거리 (m)
  double wheel_radius_; // 바퀴 반지름 (m)
};

#endif // CMD_VEL_TO_RPM_HPP
