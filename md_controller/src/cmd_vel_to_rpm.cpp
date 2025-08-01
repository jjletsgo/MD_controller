#include "md_controller/cmd_vel_to_rpm.hpp"
#include <cmath>

CmdVelToRpm::CmdVelToRpm()
: Node("cmd_vel_to_rpm")
{
  // 1) 파라미터 선언 & 초기값
  this->declare_parameter<double>("wheel_base", 0.3);
  this->declare_parameter<double>("wheel_radius", 0.05);
  this->get_parameter("wheel_base", wheel_base_);
  this->get_parameter("wheel_radius", wheel_radius_);

  // 2) /cmd_vel 구독
  cmd_vel_sub_ = this->create_subscription<geometry_msgs::msg::Twist>(
    "/cmd_vel", 10,
    std::bind(&CmdVelToRpm::cmdVelCallback, this, std::placeholders::_1));

  // 3) 좌우 RPM 퍼블리셔
  rpm_left_pub_  = this->create_publisher<std_msgs::msg::Int32>("/cmd_rpm_left",  10);
  rpm_right_pub_ = this->create_publisher<std_msgs::msg::Int32>("/cmd_rpm_right", 10);
}

void CmdVelToRpm::cmdVelCallback(const geometry_msgs::msg::Twist::SharedPtr msg)
{
  // a) /cmd_vel에서 속도 추출
  double v = msg->linear.x;    // 전진 속도 (m/s)
  double w = msg->angular.z;   // 회전 속도 (rad/s)

  // b) 차동구동 공식으로 좌/우 휠 선속도 계산
  double v_l = v - (w * wheel_base_ / 2.0);
  double v_r = v + (w * wheel_base_ / 2.0);

  // c) 선속도 → 각속도(rad/s)
  double omega_l = v_l / wheel_radius_;
  double omega_r = v_r / wheel_radius_;

  // d) 각속도 → RPM
  double rpm_l = omega_l * 60.0 / (2.0 * M_PI);
  double rpm_r = omega_r * 60.0 / (2.0 * M_PI);

  // e) 정수로 반올림
  std_msgs::msg::Int32 l_msg, r_msg;
  l_msg.data = static_cast<int>(std::round(rpm_l));
  r_msg.data = static_cast<int>(std::round(rpm_r));

  // f) 퍼블리시
  rpm_left_pub_->publish(l_msg);
  rpm_right_pub_->publish(r_msg);

  RCLCPP_DEBUG(this->get_logger(),
    "v=%.2f w=%.2f → rpm_l=%d, rpm_r=%d", v, w, l_msg.data, r_msg.data);
}
