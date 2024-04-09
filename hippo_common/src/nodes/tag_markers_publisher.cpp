
// Copyright (C) 2024 Thies Lennart Alff
//
// This program is free software; you can redistribute it and/or
// modify it under the terms of the GNU General Public License
// as published by the Free Software Foundation; either version 2
// of the License, or (at your option) any later version.
//
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
// GNU General Public License for more details.
//
// You should have received a copy of the GNU General Public License
// along with this program; if not, write to the Free Software
// Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA  02110-1301,
// USA

#include <yaml-cpp/yaml.h>

#include <geometry_msgs/msg/pose_stamped.hpp>
#include <hippo_common/convert.hpp>
#include <hippo_common/param_utils.hpp>
#include <hippo_common/yaml.hpp>
#include <rclcpp/rclcpp.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

namespace hippo_common {
class TagMarkersPublisher : public rclcpp::Node {
 public:
  struct Params {
    std::string tag_poses_file;
  };
  TagMarkersPublisher() : Node("tag_markers_publisher") {
    HIPPO_COMMON_DECLARE_PARAM_READONLY(tag_poses_file);
    publisher_ = create_publisher<visualization_msgs::msg::Marker>("tags", 10);
    timer_ = create_timer(std::chrono::milliseconds(500), [this]() {
      if (publisher_) {
        publisher_->publish(lines_);
      }
    });
    YAML::Node node;
    try {
      node = YAML::LoadFile(params_.tag_poses_file);

    } catch (const YAML::BadFile &) {
      RCLCPP_FATAL(get_logger(), "Could not load tag_poses file [%s]",
                   params_.tag_poses_file.c_str());
      return;
    }

    auto tag_poses = node["tag_poses"].as<hippo_common::yaml::TagPoses>();
    geometry_msgs::msg::Pose pose;
    lines_.type = visualization_msgs::msg::Marker::LINE_LIST;
    lines_.action = visualization_msgs::msg::Marker::ADD;
    lines_.ns = "frames";
    lines_.id = 0;
    lines_.scale.x = 0.01;
    lines_.points.clear();
    lines_.color.a = 1.0;
    for (const auto &tag_pose : tag_poses) {
      add_frame_points(tag_pose);
      lines_.header.frame_id = tag_pose.frame_id;
    }
    lines_.header.stamp = now();
  }
  void add_frame_points(const hippo_common::yaml::TagPose &pose) {
    std::vector<Eigen::Vector3d> tips;
    tips.push_back(pose.position +
                   pose.orientation * Eigen::Vector3d::UnitX() * 0.1);
    tips.push_back(pose.position +
                   pose.orientation * Eigen::Vector3d::UnitY() * 0.1);
    tips.push_back(pose.position +
                   pose.orientation * Eigen::Vector3d::UnitZ() * 0.1);
    geometry_msgs::msg::Point tmp_point;
    int i = 0;
    std::vector<std_msgs::msg::ColorRGBA> rgba;
    std_msgs::msg::ColorRGBA color;
    color.a = 1.0;
    color.r = 1.0;
    color.g = 0.0;
    color.b = 0.0;
    rgba.push_back(color);
    color.a = 1.0;
    color.r = 0.0;
    color.g = 1.0;
    color.b = 0.0;
    rgba.push_back(color);
    color.a = 1.0;
    color.r = 0.0;
    color.g = 0.0;
    color.b = 1.0;
    rgba.push_back(color);
    for (const auto &tip : tips) {
      hippo_common::convert::EigenToRos(pose.position, tmp_point);
      lines_.points.push_back(tmp_point);
      lines_.colors.push_back(rgba.at(i));
      lines_.colors.push_back(rgba.at(i));
      hippo_common::convert::EigenToRos(tip, tmp_point);
      lines_.points.push_back(tmp_point);
      ++i;
    }
  }

 private:
  Params params_;
  rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr publisher_;
  rclcpp::TimerBase::SharedPtr timer_;
  visualization_msgs::msg::Marker lines_;
};
}  // namespace hippo_common
   //
int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<hippo_common::TagMarkersPublisher>();
  rclcpp::spin(node);
  return 0;
}
