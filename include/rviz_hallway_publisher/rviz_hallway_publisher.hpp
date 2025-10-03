#ifndef ROSFLIGHT_SIM_STANDALONE_SIM_HPP
#define ROSFLIGHT_SIM_STANDALONE_SIM_HPP

#include <vector>
#include <Eigen/Geometry>

#include <ament_index_cpp/get_package_share_directory.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <rclcpp/rclcpp.hpp>
#include <visualization_msgs/msg/marker.hpp>

namespace onboarding_project
{

class RvizPublisher : public rclcpp::Node
{
public:
  RvizPublisher();

private:
  rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr hallway_pub_;
  Eigen::Vector2d last_second_point_right_{0.0, 0.0};
  Eigen::Vector2d last_second_point_left_{0.0, 0.0};
  rclcpp::QoS qos_transient_local_20_;

  // Set up parameter handling
  OnSetParametersCallbackHandle::SharedPtr parameter_callback_handle_;
  rcl_interfaces::msg::SetParametersResult
  parameters_callback(const std::vector<rclcpp::Parameter> & parameters);
  void declare_parameters();

  void create_and_publish_hallway();
  visualization_msgs::msg::Marker create_default_hallway();
  void add_back_wall();
  void publish_model();
  Eigen::Vector2d move_second_up_or_down(Eigen::Vector2d line_dir, Eigen::Vector2d normal, double width);
};

} // namespace onboarding_project

#endif
