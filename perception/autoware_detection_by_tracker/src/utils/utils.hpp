// Copyright 2023 TIER IV, Inc.
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

#ifndef UTILS__UTILS_HPP_
#define UTILS__UTILS_HPP_

#include <autoware/object_recognition_utils/object_recognition_utils.hpp>
#include <autoware/shape_estimation/shape_estimator.hpp>
#include <autoware_utils/geometry/boost_geometry.hpp>
#include <autoware_utils/math/unit_conversion.hpp>

#include <autoware_perception_msgs/msg/object_classification.hpp>
#include <autoware_perception_msgs/msg/shape.hpp>
#include <tier4_perception_msgs/msg/detected_objects_with_feature.hpp>

#include <boost/optional/optional.hpp>

#include <cstdint>

namespace autoware::detection_by_tracker
{
namespace utils
{
using Label = autoware_perception_msgs::msg::ObjectClassification;
namespace bg = boost::geometry;

struct TrackerIgnoreLabel
{
  bool UNKNOWN;
  bool CAR;
  bool TRUCK;
  bool BUS;
  bool TRAILER;
  bool MOTORCYCLE;
  bool BICYCLE;
  bool PEDESTRIAN;
  bool isIgnore(const uint8_t label) const
  {
    return (label == Label::UNKNOWN && UNKNOWN) || (label == Label::CAR && CAR) ||
           (label == Label::TRUCK && TRUCK) || (label == Label::BUS && BUS) ||
           (label == Label::TRAILER && TRAILER) || (label == Label::MOTORCYCLE && MOTORCYCLE) ||
           (label == Label::BICYCLE && BICYCLE) || (label == Label::PEDESTRIAN && PEDESTRIAN);
  }
};

void setClusterInObjectWithFeature(
  const std_msgs::msg::Header & header, const pcl::PointCloud<pcl::PointXYZ> & cluster,
  tier4_perception_msgs::msg::DetectedObjectWithFeature & feature_object);

autoware_perception_msgs::msg::Shape extendShape(
  const autoware_perception_msgs::msg::Shape & shape, const float scale);

std::vector<autoware_utils::Polygon2d> bufferPolygon2d(
  const autoware_utils::Polygon2d & polygon, const double buffer_distance);

boost::optional<autoware::shape_estimation::ReferenceYawInfo> getReferenceYawInfo(
  const uint8_t label, const float yaw);

boost::optional<autoware::shape_estimation::ReferenceShapeSizeInfo> getReferenceShapeSizeInfo(
  const uint8_t label, const autoware_perception_msgs::msg::Shape & shape);

std::tuple<pcl::PointCloud<pcl::PointXYZ>, pcl::PointCloud<pcl::PointXYZ>> splitPointsInsidePolygon(
  const autoware_utils::Polygon2d & polygon, const pcl::PointCloud<pcl::PointXYZ> & input);

}  // namespace utils
}  // namespace autoware::detection_by_tracker

#endif  // UTILS__UTILS_HPP_
