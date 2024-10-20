#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

class WaypointNavNode : public rclcpp::Node
{
  public:
    WaypointNavNode()
    : Node("waypoint_nav_node")
    {
      this->declare_parameter<std::string>("example_param", "default_value");
      std::string example_param = this->get_parameter("example_param").as_string();
      RCLCPP_INFO(this->get_logger(), "Declared parameter 'example_param'. Value: %s", example_param.c_str());

      RCLCPP_INFO(this->get_logger(), "Hello world from the C++ node %s", "waypoint_nav_node");
    }
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<WaypointNavNode>());
  rclcpp::shutdown();
  return 0;
}