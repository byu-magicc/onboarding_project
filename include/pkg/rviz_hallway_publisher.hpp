#ifndef ROSFLIGHT_SIM_STANDALONE_SIM_HPP
#define ROSFLIGHT_SIM_STANDALONE_SIM_HPP

#include <ament_index_cpp/get_package_share_directory.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <rclcpp/rclcpp.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_ros/transform_broadcaster.h>
#include <visualization_msgs/msg/marker.hpp>

namespace onboarding_project
{

class RvizPublisher : public rclcpp::Node
{
public:
  RvizPublisher();

private:
  rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr rviz_wp_pub_;
  rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr rviz_mesh_pub_;
  rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr rviz_aircraft_path_pub_;
  rclcpp::Subscription<rosflight_msgs::msg::SimState>::SharedPtr vehicle_state_sub_;

  std::unique_ptr<tf2_ros::TransformBroadcaster> aircraft_tf2_broadcaster_;

  // Set up parameter handling
  OnSetParametersCallbackHandle::SharedPtr parameter_callback_handle_;
  rcl_interfaces::msg::SetParametersResult
  parameters_callback(const std::vector<rclcpp::Parameter> & parameters);
  void declare_parameters();

  void state_update_callback(const rosflight_msgs::msg::SimState & state);
  void update_list();
  void update_mesh();
  void update_aircraft_history();

  rosflight_msgs::msg::SimState vehicle_state_;

  // Persistent rviz markers
  visualization_msgs::msg::Marker aircraft_;
  visualization_msgs::msg::Marker aircraft_history_;
  std::vector<geometry_msgs::msg::Point> aircraft_history_points_;

  int i_;
};

} // namespace onboarding_project

#endif
