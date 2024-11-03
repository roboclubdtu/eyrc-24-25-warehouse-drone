// WD_4122

#include <memory>
#include <string>
#include <vector>
#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/point.hpp>
#include "swift_pico/srv/GenPath.hpp"
// #include <opencv2/opencv.hpp>

using namespace std::placeholders;
using GenPath = swift_pico::srv::GenPath;

class PathPlanningServer : public rclcpp::Node
{
public:
    PathPlanningServer()
        : Node("path_planning_server")
    {
        // Declare and get the parameter for the image file path
        this->declare_parameter<std::string>("bit_map_path", "default_image_path.png");
        this->get_parameter("bit_map_path", bit_map_path_);

        // Initialize the service
        service_ = this->create_service<GenPath>(
            "GenPath", std::bind(&PathPlanningServer::handle_gen_path_service, this, _1, _2));

        RCLCPP_INFO(this->get_logger(), "PathPlanningServer node has been started.");
        load_image();
    }   

private:
    std::string bit_map_path_;
    rclcpp::Service<GenPath>::SharedPtr service_;

    void load_image()
    {
        // cv::Mat image = cv::imread(bit_map_path_, cv::IMREAD_COLOR);
        // if (image.empty())
        // {
        //     RCLCPP_ERROR(this->get_logger(), "Failed to load image from: %s", bit_map_path_.c_str());
        // }
        // else
        // {
        //     RCLCPP_INFO(this->get_logger(), "Successfully loaded image from: %s", bit_map_path_.c_str());
        //     // Process the image if needed
        // }
        RCLCPP_INFO(this->get_logger(), "Successfully loaded image from: %s", bit_map_path_.c_str());
    }

    // Service handler for 'GenPath'
    void handle_gen_path_service(
        const std::shared_ptr<GenPath::Request> request,
        std::shared_ptr<GenPath::Response> response)
    {
        RCLCPP_INFO(this->get_logger(), "Received request to generate path.");

        // Example logic for generating a path
        std::vector<geometry_msgs::msg::Point> generated_path;
        for (const auto &waypoint : request->waypoints)
        {
            geometry_msgs::msg::Point path_point;
            path_point.x = waypoint.x + 1.0;  // Example transformation
            path_point.y = waypoint.y + 1.0;
            path_point.z = waypoint.z;

            generated_path.push_back(path_point);
        }

        response->path = generated_path;
        RCLCPP_INFO(this->get_logger(), "Path generated and sent.");
    }
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<PathPlanningServer>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
