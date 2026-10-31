#include <memory>

#include "control/waypoint_follower.hpp"
#include "rclcpp/rclcpp.hpp"

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<control::WaypointFollower>());
  rclcpp::shutdown();
  return 0;
}
