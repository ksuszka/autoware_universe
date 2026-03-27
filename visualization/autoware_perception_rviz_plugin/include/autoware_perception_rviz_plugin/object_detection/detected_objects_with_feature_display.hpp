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
#ifndef AUTOWARE_PERCEPTION_RVIZ_PLUGIN__OBJECT_DETECTION__DETECTED_OBJECTS_WITH_FEATURE_DISPLAY_HPP_
#define AUTOWARE_PERCEPTION_RVIZ_PLUGIN__OBJECT_DETECTION__DETECTED_OBJECTS_WITH_FEATURE_DISPLAY_HPP_

#include "autoware_perception_rviz_plugin/object_detection/object_polygon_display_base.hpp"

#include <rviz_rendering/objects/billboard_line.hpp>

#include <tier4_perception_msgs/msg/detected_objects_with_feature.hpp>

#include <fmt/core.h>

#include <stdexcept>

namespace autoware
{
namespace rviz_plugins
{
namespace object_detection
{

enum class PolygonDisplayMode { Marker = 1, Ogre = 2 };

/// \brief Class defining rviz plugin to visualize DetectedObjectsWithFeature
class AUTOWARE_PERCEPTION_RVIZ_PLUGIN_PUBLIC DetectedObjectsWithFeatureDisplay
: public ObjectPolygonDisplayBase<tier4_perception_msgs::msg::DetectedObjectsWithFeature>
{
  Q_OBJECT

public:
  using DetectedObjectsWithFeature = tier4_perception_msgs::msg::DetectedObjectsWithFeature;

  DetectedObjectsWithFeatureDisplay();

private:
  std::vector<std::shared_ptr<rviz_rendering::BillboardLine>> ogre_edges_;

  void show_shapes_marker(DetectedObjectsWithFeature::ConstSharedPtr msg);
  void show_shapes_ogre(DetectedObjectsWithFeature::ConstSharedPtr msg);

  template <typename ClassificationContainerT>
  std::optional<Marker::SharedPtr> get_point_cloud_marker_ptr(
    const sensor_msgs::msg::PointCloud2 & point_cloud, const ClassificationContainerT & labels,
    const double & point_size)
  {
    const std_msgs::msg::ColorRGBA color_rgba = get_color_rgba(labels);
    if (m_display_point_cloud_property.getBool()) {
      auto marker = detail::get_point_cloud_marker_ptr(point_cloud, color_rgba, point_size);
      if (marker != nullptr) {
        return marker;
      } else {
        report_error("Point cloud", fmt::format("Failed to generate point cloud"));
      }
    }
    return std::nullopt;
  }

  double get_point_size() { return m_point_size_property.getFloat(); }
  void processMessage(DetectedObjectsWithFeature::ConstSharedPtr msg) override;

  // Properties to enable/disable and modify point cloud visualization
  rviz_common::properties::BoolProperty m_display_point_cloud_property;
  rviz_common::properties::FloatProperty m_point_size_property;

  // Property to choose bounding polygon display type: Marker / Ogre
  rviz_common::properties::EnumProperty m_polygon_display_mode_property;
};

}  // namespace object_detection
}  // namespace rviz_plugins
}  // namespace autoware

#endif  // AUTOWARE_PERCEPTION_RVIZ_PLUGIN__OBJECT_DETECTION__DETECTED_OBJECTS_WITH_FEATURE_DISPLAY_HPP_
