// Copyright 2025 TIER IV, Inc.
// Copyright (c) 2025 Autonomous Systems Sp. z o.o.
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

#include "utils.hpp"

namespace autoware::detection_by_tracker
{
namespace utils
{

void setClusterInObjectWithFeature(
  const std_msgs::msg::Header & header, const pcl::PointCloud<pcl::PointXYZ> & cluster,
  tier4_perception_msgs::msg::DetectedObjectWithFeature & feature_object)
{
  sensor_msgs::msg::PointCloud2 ros_pointcloud;
  pcl::toROSMsg(cluster, ros_pointcloud);
  ros_pointcloud.header = header;
  feature_object.feature.cluster = ros_pointcloud;
}

autoware_perception_msgs::msg::Shape extendShape(
  const autoware_perception_msgs::msg::Shape & shape, const float scale)
{
  autoware_perception_msgs::msg::Shape output = shape;
  output.dimensions.x *= scale;
  output.dimensions.y *= scale;
  output.dimensions.z *= scale;
  for (auto & point : output.footprint.points) {
    point.x *= scale;
    point.y *= scale;
    point.z *= scale;
  }
  return output;
}

std::vector<autoware_utils::Polygon2d> bufferPolygon2d(
  const autoware_utils::Polygon2d & polygon, const double buffer_distance)
{
  bg::strategy::buffer::distance_symmetric<double> distance_strategy(buffer_distance);
  bg::strategy::buffer::join_miter join_strategy;
  bg::strategy::buffer::end_flat end_strategy;
  bg::strategy::buffer::side_straight side_strategy;
  bg::strategy::buffer::point_square point_strategy;
  std::vector<autoware_utils::Polygon2d> buffed_polygons;
  bg::buffer(
    polygon, buffed_polygons, distance_strategy, side_strategy, join_strategy, end_strategy,
    point_strategy);
  if (buffed_polygons.empty()) {
    buffed_polygons.push_back(autoware_utils::Polygon2d{});
  }
  return buffed_polygons;
}

boost::optional<autoware::shape_estimation::ReferenceYawInfo> getReferenceYawInfo(
  const uint8_t label, const float yaw)
{
  const bool is_vehicle =
    Label::CAR == label || Label::TRUCK == label || Label::BUS == label || Label::TRAILER == label;
  if (is_vehicle) {
    return autoware::shape_estimation::ReferenceYawInfo{yaw, autoware_utils::deg2rad(30)};
  } else {
    return boost::none;
  }
}

boost::optional<autoware::shape_estimation::ReferenceShapeSizeInfo> getReferenceShapeSizeInfo(
  const uint8_t label, const autoware_perception_msgs::msg::Shape & shape)
{
  const bool is_vehicle =
    Label::CAR == label || Label::TRUCK == label || Label::BUS == label || Label::TRAILER == label;
  if (is_vehicle) {
    return autoware::shape_estimation::ReferenceShapeSizeInfo{
      shape, autoware::shape_estimation::ReferenceShapeSizeInfo::Mode::Min};
  } else {
    return boost::none;
  }
}

std::tuple<pcl::PointCloud<pcl::PointXYZ>, pcl::PointCloud<pcl::PointXYZ>> splitPointsInsidePolygon(
  const autoware_utils::Polygon2d & polygon, const pcl::PointCloud<pcl::PointXYZ> & input)
{
  pcl::PointCloud<pcl::PointXYZ> inside_output;
  pcl::PointCloud<pcl::PointXYZ> outside_output;

  inside_output.points.reserve(input.points.size());
  outside_output.points.reserve(input.points.size());

  constexpr double eps = 1.0e-9;

  for (const auto & pt : input.points) {
    autoware_utils::Point2d p(pt.x, pt.y);
    const bool point_is_inside = boost::geometry::distance(p, polygon) < eps;

    (point_is_inside ? inside_output.points : outside_output.points).push_back(pt);
  }

  // Update width and height after all points are processed
  inside_output.width = inside_output.points.size();
  inside_output.height = 1;
  outside_output.width = outside_output.points.size();
  outside_output.height = 1;

  return std::make_tuple(inside_output, outside_output);
}

}  // namespace utils
}  // namespace autoware::detection_by_tracker
