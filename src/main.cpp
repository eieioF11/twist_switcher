#include <rclcpp/rclcpp.hpp>
#include "twist_switcher/twist_switcher_component.hpp"

int main(int argc, char * argv[]){
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<TwistSwitcher>());
  rclcpp::shutdown();
  return 0;
}