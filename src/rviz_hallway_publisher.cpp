#include "rviz_hallway_publisher.hpp"

namespace onboarding_project {

RvizPublisher::RvizPublisher()
  : Node("rviz_hallway_publisher")
  , qos_transient_local_20_(20)
{
  declare_parameters();
  parameter_callback_handle_ = this->add_on_set_parameters_callback(std::bind(
      &RvizPublisher::parameters_callback, this, std::placeholders::_1));

  qos_transient_local_20_.transient_local();
  hallway_pub_ = this->create_publisher<visualization_msgs::msg::Marker>(
      "rviz/hallway", qos_transient_local_20_);

  create_and_publish_hallway();
  publish_model();
  // publish_finish_area(); // This is an optional addition to the map, which shows the target area
}

void RvizPublisher::declare_parameters() {
  this->declare_parameter("cosmo_file", "resource/cosmo.dae");
  this->declare_parameter("hallway_color", std::vector<double>{0.0, 1.0, 0.0, 0.4});
  this->declare_parameter("hallway_width", 10.0);
  this->declare_parameter("hallway_height", 10.0);
  this->declare_parameter("wall_width", 2.0);
  this->declare_parameter("hallway_waypoints_x", std::vector<double>{-1.0, 20.0, 20.0, 80.0, 80.0, 60.0});
  this->declare_parameter("hallway_waypoints_y", std::vector<double>{0.0, 0.0, -20.0, -20.0, -60.0, -60.0});
  this->declare_parameter("hallway_waypoints_z", std::vector<double>{0.0, 0.0, 0.0, 0.0, 0.0, 0.0});
  this->declare_parameter("model_file", "resource/maeserstatue_small.stl");
  this->declare_parameter("model_scale", 0.15);
  this->declare_parameter("model_z_offset", 2.5);
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
    } else {
      second_point += move_second_up_or_down(line_dir, line_dir, wall_width * 1.5);
      second_point_right += move_second_up_or_down(line_dir, line_dir, wall_width * 1.5);
      last_second_point_left_ = second_point;
      last_second_point_right_ = second_point_right;
    }

    Eigen::Vector2d left_wall_center = (first_point + second_point) / 2;
    Eigen::Vector2d right_wall_center = (first_point_right + second_point_right) / 2;

    visualization_msgs::msg::Marker hallway = create_default_hallway();
    hallway.id = i;
    hallway.pose.position.x = left_wall_center[0];
    hallway.pose.position.y = left_wall_center[1];
    hallway.pose.position.z = (zs[i] + zs[i+1] + hallway_height) / 2.0;
    hallway.scale.x = std::max(std::abs(second_point[0] - first_point[0]), wall_width);
    hallway.scale.y = std::max(std::abs(second_point[1] - first_point[1]), wall_width);
    hallway_pub_->publish(hallway);

    visualization_msgs::msg::Marker hallway_right = hallway;
    hallway_right.id = i + xs.size();
    hallway_right.pose.position.x = right_wall_center[0];
    hallway_right.pose.position.y = right_wall_center[1];
    hallway_right.scale.x = std::max(std::abs(second_point_right[0] - first_point_right[0]), wall_width);
    hallway_right.scale.y = std::max(std::abs(second_point_right[1] - first_point_right[1]), wall_width);
    hallway_pub_->publish(hallway_right);
  }

  add_back_wall();
}

visualization_msgs::msg::Marker RvizPublisher::create_default_hallway()
{
  std::vector<double> color = this->get_parameter("hallway_color").as_double_array();
  double hallway_height = this->get_parameter("hallway_height").as_double();

  visualization_msgs::msg::Marker hallway;
  hallway.header.frame_id = "world"; // TODO: Check this
  hallway.ns = "hallway";
  hallway.type = visualization_msgs::msg::Marker::CUBE;
  hallway.action = visualization_msgs::msg::Marker::ADD;
  hallway.pose.orientation.x = 0.0;
  hallway.pose.orientation.y = 0.0;
  hallway.pose.orientation.z = 0.0;
  hallway.pose.orientation.w = 1.0;
  hallway.color.r = static_cast<float>(color[0]);
  hallway.color.g = static_cast<float>(color[1]);
  hallway.color.b = static_cast<float>(color[2]);
  hallway.color.a = static_cast<float>(color[3]);
  hallway.scale.z = hallway_height;

  return hallway;
}

void RvizPublisher::add_back_wall()
{
  std::vector<double> xs = this->get_parameter("hallway_waypoints_x").as_double_array();
  std::vector<double> zs = this->get_parameter("hallway_waypoints_z").as_double_array();
  double wall_width = this->get_parameter("wall_width").as_double();
  double hallway_width = this->get_parameter("hallway_width").as_double();
  double hallway_height = this->get_parameter("hallway_height").as_double();

  Eigen::Vector2d hall_point = (last_second_point_right_ + last_second_point_left_) / 2;

  visualization_msgs::msg::Marker hallway = create_default_hallway();
  hallway.id = 3*xs.size();
  hallway.pose.position.x = hall_point[0];
  hallway.pose.position.y = hall_point[1];
  hallway.pose.position.z = (zs.back() + hallway_height) / 2.0;
  if (last_second_point_right_[0] - last_second_point_left_[0] < 0.1) {
    hallway.scale.x = wall_width;
    hallway.scale.y = hallway_width + wall_width * 2;
  } else {
    hallway.scale.x = hallway_width + wall_width * 2;
    hallway.scale.y = wall_width;
  }

  hallway.color.a = 0.4;
  hallway_pub_->publish(hallway);
}

Eigen::Vector2d RvizPublisher::move_second_up_or_down(Eigen::Vector2d line_dir, Eigen::Vector2d normal, double width) {
  Eigen::Vector2d result = line_dir.dot(normal) * line_dir * width;
  return result;
}

void RvizPublisher::publish_model() {
  std::vector<double> xs = this->get_parameter("hallway_waypoints_x").as_double_array();
  std::vector<double> ys = this->get_parameter("hallway_waypoints_y").as_double_array();
  std::vector<double> zs = this->get_parameter("hallway_waypoints_z").as_double_array();

  visualization_msgs::msg::Marker model;
  model.header.frame_id = "world";
  model.ns = "stl";
  model.id = 0;
  model.type = visualization_msgs::msg::Marker::MESH_RESOURCE;
  model.mesh_resource = "package://onboarding_project/" + this->get_parameter("model_file").as_string();
  model.mesh_use_embedded_materials = false;
  model.action = visualization_msgs::msg::Marker::ADD;
  model.pose.position.x = xs.back();
  model.pose.position.y = ys.back();
  model.pose.position.z = zs.back() + this->get_parameter("model_z_offset").as_double();
  model.pose.orientation.x = 0.0;
  model.pose.orientation.y = 0.0;
  model.pose.orientation.z = 0.0;
  model.pose.orientation.w = 1.0;
  model.scale.x = this->get_parameter("model_scale").as_double();
  model.scale.y = this->get_parameter("model_scale").as_double();
  model.scale.z = this->get_parameter("model_scale").as_double();
  model.color.r = 0.67f;
  model.color.g = 0.67f;
  model.color.b = 0.67f;
  model.color.a = 1.0;
  hallway_pub_->publish(model);
}

void RvizPublisher::publish_finish_area() {
  std::vector<double> xs = this->get_parameter("hallway_waypoints_x").as_double_array();
  std::vector<double> ys = this->get_parameter("hallway_waypoints_y").as_double_array();
  std::vector<double> zs = this->get_parameter("hallway_waypoints_z").as_double_array();
  visualization_msgs::msg::Marker area;
  area.header.frame_id = "world";
  area.id = 100;
  area.ns = "area";
  area.type = visualization_msgs::msg::Marker::CUBE;
  area.action = visualization_msgs::msg::Marker::ADD;
  area.pose.position.x = xs.back() + 2.5;
  area.pose.position.y = ys.back();
  area.pose.position.z = zs.back() + 1.0;
  area.pose.orientation.x = 0.0;
  area.pose.orientation.y = 0.0;
  area.pose.orientation.z = 0.0;
  area.pose.orientation.w = 1.0;
  area.color.r = 1.0;
  area.color.g = 0.0;
  area.color.b = 0.0;
  area.color.a = 0.4;
  area.scale.x = 10.0;
  area.scale.y = 10.0;
  area.scale.z = 2.0;
  hallway_pub_->publish(area);
}

} // namespace onboarding_project

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);

  auto node = std::make_shared<onboarding_project::RvizPublisher>();

  rclcpp::spin(node);

  return 0;
}
