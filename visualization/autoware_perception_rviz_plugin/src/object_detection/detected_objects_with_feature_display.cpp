// Copyright 2021 Apex.AI, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.
//
// Co-developed by Tier IV, Inc. and Apex.AI, Inc.
//
// Modified due to: SDP-4469

#include "autoware_perception_rviz_plugin/object_detection/detected_objects_with_feature_display.hpp"

#include <memory>

namespace autoware
{
namespace rviz_plugins
{
namespace object_detection
{
DetectedObjectsWithFeatureDisplay::DetectedObjectsWithFeatureDisplay()
: ObjectPolygonDisplayBase("detected_objects_with_feature"),
  m_display_point_cloud_property{
    "Display Point Cloud", true, "Enable/disable point cloud visualization", this},
  m_point_size_property{"Point Size", 0.07, "Set the size of points from the point cloud.", this},
  m_polygon_display_mode_property{
    "Polygon display mode", "Marker", "How do draw 3d polygon (Marker / Ogre).", this}
{
  m_polygon_display_mode_property.addOption("Marker", static_cast<int>(PolygonDisplayMode::Marker));
  m_polygon_display_mode_property.addOption("Ogre", static_cast<int>(PolygonDisplayMode::Ogre));
}

void DetectedObjectsWithFeatureDisplay::processMessage(
  DetectedObjectsWithFeature::ConstSharedPtr msg)
{
  clear_markers();

  // show shape
  auto display_mode =
    static_cast<PolygonDisplayMode>(m_polygon_display_mode_property.getOptionInt());
  switch (display_mode) {
    case PolygonDisplayMode::Marker:
      show_shapes_marker(msg);
      break;
    case PolygonDisplayMode::Ogre:
      show_shapes_ogre(msg);
      break;
    default:
      report_error(
        "Display", fmt::format(
                     "No implementation for polygon display mode '{}'",
                     m_polygon_display_mode_property.getOptionInt()));
      break;
  }

  // show other markers
  int id = 0;
  for (const auto & feature_object : msg->feature_objects) {
    const auto & object = feature_object.object;

    // Get marker for label
    auto label_marker = get_label_marker_ptr(
      object.kinematics.pose_with_covariance.pose.position,
      object.kinematics.pose_with_covariance.pose.orientation, object.classification);
    if (label_marker) {
      auto label_marker_ptr = label_marker.value();
      label_marker_ptr->header = msg->header;
      label_marker_ptr->id = id++;
      add_marker(label_marker_ptr);
    }

    // Get marker for pose covariance
    auto pose_with_covariance_marker =
      get_pose_covariance_marker_ptr(object.kinematics.pose_with_covariance);
    if (pose_with_covariance_marker) {
      auto marker_ptr = pose_with_covariance_marker.value();
      marker_ptr->header = msg->header;
      marker_ptr->id = id++;
      add_marker(marker_ptr);
    }

    // Get marker for yaw covariance
    auto yaw_covariance_marker = get_yaw_covariance_marker_ptr(
      object.kinematics.pose_with_covariance, object.shape.dimensions.x * 0.65,
      get_line_width() * 0.5);
    if (yaw_covariance_marker) {
      auto marker_ptr = yaw_covariance_marker.value();
      marker_ptr->header = msg->header;
      marker_ptr->id = id++;
      add_marker(marker_ptr);
    }

    // Get marker for existence probability
    geometry_msgs::msg::Point existence_probability_position;
    existence_probability_position.x = object.kinematics.pose_with_covariance.pose.position.x + 0.5;
    existence_probability_position.y = object.kinematics.pose_with_covariance.pose.position.y;
    existence_probability_position.z = object.kinematics.pose_with_covariance.pose.position.z + 0.5;
    const float existence_probability = object.existence_probability;
    auto existence_prob_marker = get_existence_probability_marker_ptr(
      existence_probability_position, object.kinematics.pose_with_covariance.pose.orientation,
      existence_probability, object.classification);
    if (existence_prob_marker) {
      auto existence_prob_marker_ptr = existence_prob_marker.value();
      existence_prob_marker_ptr->header = msg->header;
      existence_prob_marker_ptr->id = id++;
      add_marker(existence_prob_marker_ptr);
    }

    // Get marker for velocity text
    geometry_msgs::msg::Point vel_vis_position;
    vel_vis_position.x = object.kinematics.pose_with_covariance.pose.position.x - 0.5;
    vel_vis_position.y = object.kinematics.pose_with_covariance.pose.position.y;
    vel_vis_position.z = object.kinematics.pose_with_covariance.pose.position.z - 0.5;
    auto velocity_text_marker = get_velocity_text_marker_ptr(
      object.kinematics.twist_with_covariance.twist, vel_vis_position, object.classification);
    if (velocity_text_marker) {
      auto velocity_text_marker_ptr = velocity_text_marker.value();
      velocity_text_marker_ptr->header = msg->header;
      velocity_text_marker_ptr->id = id++;
      add_marker(velocity_text_marker_ptr);
    }

    // Get marker for twist
    auto twist_marker = get_twist_marker_ptr(
      object.kinematics.pose_with_covariance, object.kinematics.twist_with_covariance,
      get_line_width());
    if (twist_marker) {
      auto twist_marker_ptr = twist_marker.value();
      twist_marker_ptr->header = msg->header;
      twist_marker_ptr->id = id++;
      add_marker(twist_marker_ptr);
    }

    // Get marker for twist covariance
    auto twist_covariance_marker = get_twist_covariance_marker_ptr(
      object.kinematics.pose_with_covariance, object.kinematics.twist_with_covariance);
    if (twist_covariance_marker) {
      auto marker_ptr = twist_covariance_marker.value();
      marker_ptr->header = msg->header;
      marker_ptr->id = id++;
      add_marker(marker_ptr);
    }

    // Get marker for yaw rate
    auto yaw_rate_marker = get_yaw_rate_marker_ptr(
      object.kinematics.pose_with_covariance, object.kinematics.twist_with_covariance,
      get_line_width() * 0.4);
    if (yaw_rate_marker) {
      auto marker_ptr = yaw_rate_marker.value();
      marker_ptr->header = msg->header;
      marker_ptr->id = id++;
      add_marker(marker_ptr);
    }

    // Get marker for yaw rate covariance
    auto yaw_rate_covariance_marker = get_yaw_rate_covariance_marker_ptr(
      object.kinematics.pose_with_covariance, object.kinematics.twist_with_covariance,
      get_line_width() * 0.3);
    if (yaw_rate_covariance_marker) {
      auto marker_ptr = yaw_rate_covariance_marker.value();
      marker_ptr->header = msg->header;
      marker_ptr->id = id++;
      add_marker(marker_ptr);
    }
    // Get marker for cluster
    auto point_cloud_marker = get_point_cloud_marker_ptr(
      feature_object.feature.cluster, object.classification, get_point_size());
    if (point_cloud_marker) {
      auto point_cloud_marker_ptr = point_cloud_marker.value();
      point_cloud_marker_ptr->header = msg->header;
      point_cloud_marker_ptr->id = id++;
      add_marker(point_cloud_marker_ptr);
    }
  }
}

void DetectedObjectsWithFeatureDisplay::show_shapes_marker(
  DetectedObjectsWithFeature::ConstSharedPtr msg)
{
  ogre_edges_.resize(0);

  int id = 0;
  for (const auto & feature_object : msg->feature_objects) {
    const auto & object = feature_object.object;
    auto shape_marker = get_shape_marker_ptr(
      object.shape, object.kinematics.pose_with_covariance.pose.position,
      object.kinematics.pose_with_covariance.pose.orientation, object.classification,
      get_line_width(),
      object.kinematics.orientation_availability ==
        autoware_perception_msgs::msg::DetectedObjectKinematics::AVAILABLE);
    if (shape_marker) {
      auto shape_marker_ptr = shape_marker.value();
      shape_marker_ptr->header = msg->header;
      shape_marker_ptr->id = id++;
      add_marker(shape_marker_ptr);
    }
  }
}

void DetectedObjectsWithFeatureDisplay::show_shapes_ogre(
  DetectedObjectsWithFeature::ConstSharedPtr msg)
{
  // allocates memory for given number of billboard lines
  auto allocate_edges = [this](size_t count) {
    if (count > ogre_edges_.size()) {
      for (size_t i = ogre_edges_.size(); i < count; i++) {
        auto line = std::make_shared<rviz_rendering::BillboardLine>(
          this->context_->getSceneManager(), this->scene_node_);
        ogre_edges_.push_back(line);
      }
    } else if (count < ogre_edges_.size()) {
      ogre_edges_.resize(count);
    }
  };

  // adds point to given edge
  auto add_point =
    [](std::shared_ptr<rviz_rendering::BillboardLine> edge, double x, double y, double z) {
      Ogre::Vector3 edge_point;
      edge_point[0] = x;
      edge_point[1] = y;
      edge_point[2] = z;
      edge->addPoint(edge_point);
    };

  // creates shape using billboard lines, draws rectangle on every box facing
  auto draw_shape = [add_point](
                      std::shared_ptr<rviz_rendering::BillboardLine> edge,
                      const autoware_perception_msgs::msg::Shape & shape) {
    if (shape.footprint.points.size() < 3) {
      return;
    }

    edge->setMaxPointsPerLine(5);
    edge->setNumLines(shape.footprint.points.size());
    auto z = shape.dimensions.z;

    auto & first_point = shape.footprint.points[0];
    auto & last_point = shape.footprint.points[shape.footprint.points.size() - 1];

    for (size_t i = 0; i < shape.footprint.points.size() - 1; i++) {
      auto & a = shape.footprint.points[i];
      auto & b = shape.footprint.points[i + 1];
      add_point(edge, a.x, a.y, z * +0.5);
      add_point(edge, a.x, a.y, z * -0.5);
      add_point(edge, b.x, b.y, z * -0.5);
      add_point(edge, b.x, b.y, z * +0.5);
      add_point(edge, a.x, a.y, z * +0.5);
      edge->finishLine();
    }

    add_point(edge, last_point.x, last_point.y, z * +0.5);
    add_point(edge, last_point.x, last_point.y, z * -0.5);
    add_point(edge, first_point.x, first_point.y, z * -0.5);
    add_point(edge, first_point.x, first_point.y, z * +0.5);
    add_point(edge, last_point.x, last_point.y, z * +0.5);
    edge->finishLine();
  };

  // tries to transform object position
  auto try_transform = [this](
                         const std_msgs::msg::Header & header,
                         const tier4_perception_msgs::msg::DetectedObjectWithFeature & object) {
    Ogre::Vector3 position;
    Ogre::Quaternion orientation;
    const auto pose = object.object.kinematics.pose_with_covariance.pose;
    const auto manager = context_->getFrameManager();
    const auto success =
      manager != nullptr && manager->transform(header, pose, position, orientation);
    return std::tuple{success, position, orientation};
  };

  auto line_width = get_line_width();
  allocate_edges(msg->feature_objects.size());

  size_t count = 0;
  for (auto & object : msg->feature_objects) {
    std::shared_ptr<rviz_rendering::BillboardLine> edge = ogre_edges_[count++];
    edge->clear();

    auto [transform_ok, position, orientation] = try_transform(msg->header, object);
    if (!transform_ok) {
      report_error(
        "Transform", fmt::format(
                       "Error transforming pose from frame '{}' to frame '{}'",
                       msg->header.frame_id, qPrintable(fixed_frame_)));
      break;
    }

    edge->setPosition(position);
    edge->setOrientation(orientation);
    edge->setLineWidth(line_width);
    auto color = get_color_rgba(object.object.classification);
    edge->setColor(color.r, color.g, color.b, color.a);

    draw_shape(edge, object.object.shape);
  }
}

}  // namespace object_detection
}  // namespace rviz_plugins
}  // namespace autoware

// Export the plugin
#include <pluginlib/class_list_macros.hpp>  // NOLINT
PLUGINLIB_EXPORT_CLASS(
  autoware::rviz_plugins::object_detection::DetectedObjectsWithFeatureDisplay, rviz_common::Display)
