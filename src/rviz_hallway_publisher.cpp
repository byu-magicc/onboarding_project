#include "rviz_hallway_publisher.hpp"

namespace onboarding_project {

RvizPublisher::RvizPublisher() : Node("rviz_hallway_publisher") {
  declare_parameters();
  parameter_callback_handle_ = this->add_on_set_parameters_callback(std::bind(
      &RvizPublisher::parameters_callback, this, std::placeholders::_1));

  rclcpp::QoS qos_transient_local_20_(20);
  qos_transient_local_20_.transient_local();
  hallway_pub_ = this->create_publisher<visualization_msgs::msg::Marker>(
      "rviz/hallway", qos_transient_local_20_);
  cosmo_pub_ =
      this->create_publisher<visualization_msgs::msg::Marker>("rviz/mesh", 5);

  create_and_publish_hallway();
}

void RvizPublisher::declare_parameters() {
  this->declare_parameter("cosmo_file", "resource/cosmo.dae");
  this->declare_parameter("hallway_color", std::vector<double>{0.0, 1.0, 0.0, 1.0});
  this->declare_parameter("hallway_width", 5.0);
  this->declare_parameter("hallway_height", 10.0);
  this->declare_parameter("wall_width", 5.0);
  this->declare_parameter("hallway_waypoints_x", std::vector<double>{2.0, 20.0, 20.0, 80.0, 80.0, 60.0});
  this->declare_parameter("hallway_waypoints_y", std::vector<double>{0.0, 0.0, -20.0, -20.0, -60.0, -60.0});
  this->declare_parameter("hallway_waypoints_z", std::vector<double>{0.0, 0.0, 0.0, 0.0, 0.0, 0.0});
}

rcl_interfaces::msg::SetParametersResult RvizPublisher::parameters_callback(
    const std::vector<rclcpp::Parameter> &parameters) {
  rcl_interfaces::msg::SetParametersResult result;
  result.successful = true;
  result.reason = "success";

  // TODO: Create and publish hallway
  // TODO: Clear all waypoint

  return result;
}

void RvizPublisher::create_and_publish_hallway() {
  std::vector<double> xs = this->get_parameter("hallway_waypoints_x").as_double_array();
  std::vector<double> ys = this->get_parameter("hallway_waypoints_y").as_double_array();
  std::vector<double> zs = this->get_parameter("hallway_waypoints_z").as_double_array();
  std::vector<double> color = this->get_parameter("hallway_color").as_double_array();
  double wall_width = this->get_parameter("wall_width").as_double();
  double hallway_width = this->get_parameter("hallway_width").as_double();
  double hallway_height = this->get_parameter("hallway_height").as_double();

  if (xs.size() != ys.size() || xs.size() != zs.size()) {
    RCLCPP_ERROR(this->get_logger(), "Hallway waypoints are not the same length! Cannot create hallway");
  }

  for (std::size_t i=0; i<xs.size()-1; ++i) {
    Eigen::Vector2d curr{xs[i], ys[i]};
    Eigen::Vector2d next{xs[i+1], ys[i+1]};

    Eigen::Vector2d line_dir{next[0] - curr[0], next[1] - curr[1]};
    Eigen::Vector2d normal{-line_dir[1], line_dir[0]};

    line_dir.normalize();
    normal.normalize();

    double move_width = (hallway_width + wall_width) / 2;
    Eigen::Vector2d first_point = i > 0 ? last_second_point_left_ : curr + move_width * normal;
    Eigen::Vector2d second_point = next + move_width * normal;

    Eigen::Vector2d first_point_right = i > 0 ? last_second_point_right_ : curr - move_width * normal;
    Eigen::Vector2d second_point_right = next - move_width * normal;

    if (i < xs.size() - 2) {
      Eigen::Vector2d next_next{xs[i+2], ys[i+2]};

      Eigen::Vector2d line_dir_next{next_next[0] - next[0], next_next[1] - next[1]};
      Eigen::Vector2d normal_next{-line_dir_next[1], line_dir_next[0]};

      line_dir_next.normalize();
      normal_next.normalize();

      second_point += move_second_up_or_down(line_dir, normal_next, move_width);
      second_point_right += move_second_up_or_down(line_dir, -1 * normal_next, move_width);

      last_second_point_left_ = second_point;
      last_second_point_right_ = second_point_right;

      second_point += move_second_up_or_down(line_dir, line_dir, wall_width / 2);
      second_point_right += move_second_up_or_down(line_dir, line_dir, wall_width / 2);
    }

    Eigen::Vector2d left_wall_center = (first_point + second_point) / 2;
    Eigen::Vector2d right_wall_center = (first_point_right + second_point_right) / 2;

    visualization_msgs::msg::Marker hallway;
    hallway.header.frame_id = "world"; // TODO: Check this
    hallway.ns = "hallway";
    hallway.id = i;
    hallway.type = visualization_msgs::msg::Marker::CUBE;
    hallway.action = visualization_msgs::msg::Marker::ADD;
    hallway.pose.position.x = left_wall_center[0];
    hallway.pose.position.y = left_wall_center[1];
    hallway.pose.position.z = (zs[i] + zs[i+1] + hallway_height) / 2.0;
    hallway.pose.orientation.x = 0.0;
    hallway.pose.orientation.y = 0.0;
    hallway.pose.orientation.z = 0.0;
    hallway.pose.orientation.w = 1.0;
    hallway.scale.x = std::max(std::abs(second_point[0] - first_point[0]), wall_width);
    hallway.scale.y = std::max(std::abs(second_point[1] - first_point[1]), wall_width);
    hallway.scale.z = hallway_height;
    hallway.color.r = static_cast<float>(color[0]);
    hallway.color.g = static_cast<float>(color[1]);
    hallway.color.b = static_cast<float>(color[2]);
    hallway.color.a = static_cast<float>(color[3]);
    hallway_pub_->publish(hallway);

    visualization_msgs::msg::Marker hallway_right = hallway;
    hallway_right.id = i + xs.size();
    hallway_right.pose.position.x = right_wall_center[0];
    hallway_right.pose.position.y = right_wall_center[1];
    hallway_right.scale.x = std::max(std::abs(second_point_right[0] - first_point_right[0]), wall_width);
    hallway_right.scale.y = std::max(std::abs(second_point_right[1] - first_point_right[1]), wall_width);
    hallway_pub_->publish(hallway_right);
  }
}

Eigen::Vector2d RvizPublisher::move_second_up_or_down(Eigen::Vector2d line_dir, Eigen::Vector2d normal, double width) {
  Eigen::Vector2d result = line_dir.dot(normal) * line_dir * width;
  return result;
}

} // namespace onboarding_project

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);

  auto node = std::make_shared<onboarding_project::RvizPublisher>();

  rclcpp::spin(node);

  return 0;
}
