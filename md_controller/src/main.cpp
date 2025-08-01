#include <rclcpp/rclcpp.hpp>
#include "md_controller/cmd_vel_to_rpm.hpp"

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<CmdVelToRpm>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
