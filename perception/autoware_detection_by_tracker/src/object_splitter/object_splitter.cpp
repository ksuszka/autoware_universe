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

#define EIGEN_MPL2_ONLY

#include "object_splitter.hpp"

#include <autoware_utils/geometry/alt_geometry.hpp>
#include <autoware_utils/geometry/boost_geometry.hpp>
#include <pcl/impl/point_types.hpp>
#include <rclcpp/logging.hpp>

#include <autoware_perception_msgs/msg/object_classification.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/point_cloud2_iterator.hpp>
#include <tier4_perception_msgs/msg/detected_object_with_feature.hpp>
#include <tier4_perception_msgs/msg/feature.hpp>

#include <boost/geometry/algorithms/correct.hpp>
#include <boost/geometry/algorithms/detail/intersects/interface.hpp>
#include <boost/geometry/index/predicates.hpp>

#include <pcl/point_cloud.h>
#include <pcl_conversions/pcl_conversions.h>

#include <algorithm>

#include "autoware_utils/geometry/geometry.hpp"

namespace autoware
{
namespace detection_by_tracker
{
namespace bg = boost::geometry;

ObjectSplitter::ObjectSplitter(
  const std::map<uint8_t, int> & max_search_distance_map,
  const detection_by_tracker::utils::TrackerIgnoreLabel & tracker_ignore,
  autoware::shape_estimation::ShapeEstimator shape_estimator,
  const std::shared_ptr<autoware::euclidean_cluster::EuclideanClusterInterface> & cluster,
  const double extend_scale, const double buffer_distance,
  const double existence_probability_threshold, const double existence_probability_modifier,
  const rclcpp::Logger & logger)
: max_search_distance_map_(max_search_distance_map),
  tracker_ignore_(tracker_ignore),
  shape_estimator_(std::move(shape_estimator)),
  cluster_(cluster),
  extend_scale_(extend_scale),
  buffer_distance_(buffer_distance),
  existence_probability_threshold_(existence_probability_threshold),
  existence_probability_modifier_(existence_probability_modifier),
  logger_(logger)
{
}

ClusterWithPose ObjectSplitter::objectWithFeatureToClusterWithPose(
  const tier4_perception_msgs::msg::DetectedObjectWithFeature & object_with_feature) const
{
  const auto & cluster_pose = object_with_feature.object.kinematics.pose_with_covariance.pose;

  pcl::PointCloud<pcl::PointXYZ> pcl_cluster;
  pcl::fromROSMsg(object_with_feature.feature.cluster, pcl_cluster);

  const auto & polygon = autoware_utils::to_polygon2d(object_with_feature.object);
  const auto cluster_size = pcl_cluster.points.size();

  if (polygon.outer().empty()) {
    RCLCPP_DEBUG(logger_, "Skipping cluster with empty polygon");
  }

  return ClusterWithPose{std::move(pcl_cluster), cluster_size, std::move(polygon), cluster_pose};
}

ClustersWithPose ObjectSplitter::getRelevantClusters(
  const DetectedObjects & tracked_objects,
  const DetectedObjectsWithFeature & input_cluster_objects) const
{
  ClustersWithPose relevant_clusters;
  relevant_clusters.reserve(input_cluster_objects.feature_objects.size());

  for (const auto & cluster_object : input_cluster_objects.feature_objects) {
    if (cluster_object.feature.cluster.data.empty()) {
      continue;
    }

    const auto & cluster_pose = cluster_object.object.kinematics.pose_with_covariance.pose;

    // Try to match with tracked objects
    for (const auto & tracked_object : tracked_objects.objects) {
      const auto label = getLabel(tracked_object.classification);

      if (tracker_ignore_.isIgnore(label)) {
        continue;
      }

      const auto max_search_range = getMaxSearchDistance(tracked_object.classification);
      const auto distance = autoware_utils::calc_distance2d(
        tracked_object.kinematics.pose_with_covariance.pose, cluster_pose);

      // Only consider clusters within search range
      if (distance > max_search_range) {
        continue;
      }

      relevant_clusters.push_back(objectWithFeatureToClusterWithPose(cluster_object));

      break;  // Only match with one tracked object
    }
  }
  return relevant_clusters;
}

Polygon2d ObjectSplitter::getExtendedPolygon(const DetectedObject & object_to_expand) const
{
  const auto label = getLabel(object_to_expand.classification);

  // For smaller object types, use buffer approach for better representation
  if (label == Label::PEDESTRIAN || label == Label::BICYCLE || label == Label::MOTORCYCLE) {
    const auto tracked_polygon = autoware_utils::to_polygon2d(object_to_expand);
    return utils::bufferPolygon2d(tracked_polygon, buffer_distance_).front();
  }

  // For large vehicles, use shape scaling for more accurate dimension expansion
  auto expanded_shape = utils::extendShape(object_to_expand.shape, extend_scale_);
  return autoware_utils::to_polygon2d(
    object_to_expand.kinematics.pose_with_covariance.pose, expanded_shape);
}

bool ObjectSplitter::shouldProcessTrackedObject(const DetectedObject & tracked_object) const
{
  const auto label = getLabel(tracked_object.classification);
  // Skip low-probability objects
  // Only update large objects (vehicle types)
  return tracked_object.existence_probability > existence_probability_threshold_ &&
         (label == Label::CAR || label == Label::TRUCK || label == Label::BUS ||
          label == Label::TRAILER);
}

bool ObjectSplitter::updateTrackedObject(
  DetectedObject & tracked_object, const pcl::PointCloud<pcl::PointXYZ> & pcl_cluster)
{
  const auto label = getLabel(tracked_object.classification);

  // Get orientation info for shape estimation
  const auto yaw_info = utils::getReferenceYawInfo(
    label, tf2::getYaw(tracked_object.kinematics.pose_with_covariance.pose.orientation));

  // Get shape size constraints
  const auto shape_size_info = utils::getReferenceShapeSizeInfo(label, tracked_object.shape);

  // Try to estimate shape and pose from point cloud
  if (!shape_estimator_.estimateShapeAndPose(
        label, pcl_cluster, yaw_info, shape_size_info, boost::none, tracked_object.shape,
        tracked_object.kinematics.pose_with_covariance.pose)) {
    RCLCPP_DEBUG(logger_, "Failed to estimate shape for object with label: %d", label);
    return false;
  }

  // Update probability after successful shape estimation
  updateExistenceProbability(tracked_object);
  return true;
}

void ObjectSplitter::updateExistenceProbability(DetectedObject & tracked_object)
{
  tracked_object.existence_probability =
    std::clamp(tracked_object.existence_probability * existence_probability_modifier_, 0.0, 1.0);
}

void ObjectSplitter::splitClustersWithPolygon(
  const ExtendedTrackedObject & extended_tracked_object, ClustersWithPose & cluster_objects,
  pcl::PointCloud<pcl::PointXYZ> & points_inside_tracked_polygon)
{
  for (auto & cluster_obj : cluster_objects) {
    const auto max_search_range =
      getMaxSearchDistance(extended_tracked_object.object->classification);
    // Skip distant clusters
    const auto distance = autoware_utils::calc_distance2d(
      extended_tracked_object.object->kinematics.pose_with_covariance.pose, cluster_obj.pose);
    if (distance > max_search_range) {
      continue;
    }

    auto & cluster = cluster_obj.cluster;
    // Skip malformed clusters
    if (cluster_obj.polygon.outer().size() < 3 || cluster.points.empty()) {
      RCLCPP_DEBUG(logger_, "Skipping empty cluster");
      continue;
    }

    // Check if the entire cluster is within the tracked polygon
    // TODO update after resolving
    // https://github.com/autowarefoundation/autoware.universe/issues/8128 bool is_cluster_within =
    // bg::within(cluster_obj.polygon, extended_tracked_polygon);
    const auto & cluster_polygon =
      autoware_utils::alt::ConvexPolygon2d::create(cluster_obj.polygon);
    const auto & extended_tracked_polygon =
      autoware_utils::alt::ConvexPolygon2d::create(extended_tracked_object.polygon);
    if (!cluster_polygon || !extended_tracked_polygon) {
      RCLCPP_DEBUG(logger_, "Failed to create convex polygon");
      continue;
    }

    if (autoware_utils::within(cluster_polygon.value(), extended_tracked_polygon.value())) {
      // If cluster is entirely within, move or append all its points
      if (points_inside_tracked_polygon.points.empty()) {
        points_inside_tracked_polygon = std::move(cluster);
      } else {
        points_inside_tracked_polygon += cluster;
        cluster.clear();
      }
      continue;
    }

    // Skip non-intersecting clusters
    if (!bg::intersects(extended_tracked_object.polygon, cluster_obj.polygon)) {
      continue;
    }

    // For partially overlapping clusters, split points
    auto [points_within_tracked_polygon, points_outside_tracked_polygon] =
      utils::splitPointsInsidePolygon(extended_tracked_object.polygon, cluster);

    // Update tracked cluster with points inside the polygon
    if (!points_within_tracked_polygon.empty()) {
      points_inside_tracked_polygon += points_within_tracked_polygon;
    }

    // Update cluster with remaining points outside the polygon
    cluster = std::move(points_outside_tracked_polygon);
  }

  // Remove empty clusters after processing
  const auto new_end = std::remove_if(
    cluster_objects.begin(), cluster_objects.end(),
    [](const auto & cluster_obj) { return cluster_obj.cluster.points.empty(); });

  cluster_objects.erase(new_end, cluster_objects.end());
}

DetectedObjectsWithFeature ObjectSplitter::clusterDividedClusters(
  const ClustersWithPose & under_segmented_clusters)
{
  // Re-cluster points that may contain multiple objects
  DetectedObjectsWithFeature divided_objects;

  for (const auto & under_segmented_cluster : under_segmented_clusters) {
    if (under_segmented_cluster.cluster.empty()) {
      RCLCPP_DEBUG(logger_, "Skipping empty under-segmented cluster");
      continue;
    }

    // Convert PCL to ROS message
    sensor_msgs::msg::PointCloud2 under_segmented_cluster_msg;
    pcl::toROSMsg(under_segmented_cluster.cluster, under_segmented_cluster_msg);
    // Apply clustering algorithm to segment points
    cluster_->cluster(
      std::make_shared<sensor_msgs::msg::PointCloud2>(std::move(under_segmented_cluster_msg)),
      divided_objects);
  }

  return divided_objects;
}

DetectedObjects ObjectSplitter::clusterObjectsToDetectedObjects(
  const DetectedObjectsWithFeature & cluster_objects)
{
  DetectedObjects out_objects;
  out_objects.objects.reserve(cluster_objects.feature_objects.size());

  for (const auto & cluster_object : cluster_objects.feature_objects) {
    // Skip empty clusters
    if (cluster_object.feature.cluster.data.empty()) {
      continue;
    }

    // Convert cluster to point cloud
    pcl::PointCloud<pcl::PointXYZ> divided_cluster;
    pcl::fromROSMsg(cluster_object.feature.cluster, divided_cluster);

    DetectedObject out_object;

    // Estimate shape from points
    if (!shape_estimator_.estimateShapeAndPose(
          Label::UNKNOWN, divided_cluster, boost::none, boost::none, boost::none, out_object.shape,
          out_object.kinematics.pose_with_covariance.pose)) {
      RCLCPP_DEBUG(logger_, "Failed to estimate shape for UNKNOWN cluster");
      continue;
    }

    // Only add objects with valid footprints
    if (!out_object.shape.footprint.points.empty()) {
      // Set object properties
      out_object.existence_probability = 1.0;

      // Set classification as UNKNOWN
      auto unknown_label = Label{};
      unknown_label.label = Label::UNKNOWN;
      unknown_label.probability = 1.0;

      out_object.classification.clear();
      out_object.classification.push_back(unknown_label);

      out_objects.objects.push_back(std::move(out_object));
    }
  }

  return out_objects;
}

void ObjectSplitter::processObjects(
  DetectedObjects & tracked_objects, ClustersWithPose & clusters_with_pose,
  DetectedObjects & out_objects)
{
  out_objects.objects.reserve(tracked_objects.objects.size());

  // Process each tracked object
  for (auto & tracked_object : tracked_objects.objects) {
    const auto label = getLabel(tracked_object.classification);
    // Skip ignored labels
    if (tracker_ignore_.isIgnore(label)) {
      continue;
    }

    ExtendedTrackedObject extended_tracked_object{
      &tracked_object, getExtendedPolygon(tracked_object)};

    if (extended_tracked_object.polygon.outer().size() < 3) {
      RCLCPP_DEBUG(
        logger_, "Skipping object with invalid polygon, label: %d",
        tracked_object.classification.front().label);
      continue;
    }

    pcl::PointCloud<pcl::PointXYZ> points_inside_tracked_polygon;

    // Split clusters that intersect with tracked object
    splitClustersWithPolygon(
      extended_tracked_object, clusters_with_pose, points_inside_tracked_polygon);

    // Update tracked object with points inside the polygon
    if (
      !points_inside_tracked_polygon.points.empty() && shouldProcessTrackedObject(tracked_object)) {
      if (!updateTrackedObject(tracked_object, points_inside_tracked_polygon)) {
        RCLCPP_DEBUG(logger_, "Tracked object is not updated! Label: %d", label);
        continue;
      }

      out_objects.objects.push_back(std::move(tracked_object));
    }
  }
}

void ObjectSplitter::processClusters(
  ClustersWithPose & clusters_with_pose, DetectedObjects & out_objects)
{
  // Keep only modified clusters
  const auto new_end = std::remove_if(
    clusters_with_pose.begin(), clusters_with_pose.end(), [](const auto & cluster_obj) {
      return cluster_obj.cluster.points.size() == cluster_obj.cluster_size;
    });

  clusters_with_pose.erase(new_end, clusters_with_pose.end());

  // Process remaining clusters that may contain other objects
  if (!clusters_with_pose.empty()) {
    const auto cluster_objects = clusterDividedClusters(clusters_with_pose);
    const auto detected_objects = clusterObjectsToDetectedObjects(cluster_objects);
    out_objects.objects.insert(
      out_objects.objects.end(), detected_objects.objects.begin(), detected_objects.objects.end());
  }
}

void ObjectSplitter::process(
  const DetectedObjectsWithFeature & cluster_objects_msg, DetectedObjects & tracked_objects,
  DetectedObjects & out_objects)
{
  out_objects.header = tracked_objects.header;

  // Get clusters relevant to tracked objects
  auto relevant_cluster_objects = getRelevantClusters(tracked_objects, cluster_objects_msg);

  if (relevant_cluster_objects.empty() && tracked_objects.objects.empty()) {
    RCLCPP_DEBUG(logger_, "No relevant clusters or tracked objects found");
    return;
  }

  // Process objects and clusters
  processObjects(tracked_objects, relevant_cluster_objects, out_objects);
  processClusters(relevant_cluster_objects, out_objects);
}

}  // namespace detection_by_tracker
}  // namespace autoware