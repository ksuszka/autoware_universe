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

#ifndef OBJECT_SPLITTER_HPP_
#define OBJECT_SPLITTER_HPP_

#include "utils/utils.hpp"

#include <autoware/euclidean_cluster/euclidean_cluster.hpp>
#include <autoware/shape_estimation/shape_estimator.hpp>
#include <autoware_utils/geometry/alt_geometry.hpp>
#include <autoware_utils/geometry/boost_geometry.hpp>
#include <autoware_utils/geometry/boost_polygon_utils.hpp>
#include <autoware_utils/geometry/geometry.hpp>

#include <autoware_perception_msgs/msg/detected_object.hpp>
#include <autoware_perception_msgs/msg/detected_objects.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <tier4_perception_msgs/msg/detected_objects_with_feature.hpp>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include <map>
#include <memory>
#include <vector>

namespace autoware
{
namespace detection_by_tracker
{

using autoware_utils::Point2d;
using autoware_utils::Polygon2d;
using DetectedObjects = autoware_perception_msgs::msg::DetectedObjects;
using DetectedObject = autoware_perception_msgs::msg::DetectedObject;
using DetectedObjectsWithFeature = tier4_perception_msgs::msg::DetectedObjectsWithFeature;
using Label = autoware_perception_msgs::msg::ObjectClassification;

struct ClusterWithPose
{
  pcl::PointCloud<pcl::PointXYZ> cluster;
  std::size_t cluster_size;
  Polygon2d polygon;
  geometry_msgs::msg::Pose pose;
};

using ClustersWithPose = std::vector<ClusterWithPose>;

struct ExtendedTrackedObject
{
  DetectedObject * object;
  Polygon2d polygon;
};

/**
 * @brief Class responsible for splitting detected objects based on tracking information
 *
 * This class processes tracked objects and clusters to improve detection accuracy
 * by splitting under-segmented clusters and updating tracked object.
 */
class ObjectSplitter
{
public:
  /**
   * @brief Construct a new Object Splitter
   *
   * @param max_search_distance_map Map of label to maximum search distance
   * @param tracker_ignore Object with labels to ignore during tracking
   * @param shape_estimator Shape estimation component
   * @param cluster Clustering component
   * @param extend_scale Scale factor to extend large object polygons (car/truck/bus/trailer)
   * @param buffer_distance Buffer distance for small object polygons
   * (pedestrian/bicycle/motorcycle)
   * @param existence_probability_threshold Minimum probability threshold for processing objects
   * @param existence_probability_modifier Modifier for adjusting existence probability
   * @param logger ROS logger // TODO maybe not needed
   */
  explicit ObjectSplitter(
    const std::map<uint8_t, int> & max_search_distance_map,
    const detection_by_tracker::utils::TrackerIgnoreLabel & tracker_ignore,
    autoware::shape_estimation::ShapeEstimator shape_estimator,
    const std::shared_ptr<autoware::euclidean_cluster::EuclideanClusterInterface> & cluster,
    const double extend_scale, const double buffer_distance,
    const double existence_probability_threshold, const double existence_probability_modifier,
    const bool fast_spawn_unknown_clusters,
    const rclcpp::Logger & logger);

  /**
   * @brief Main processing function
   *
   * Processes cluster objects and tracked objects to produce updated objects
   *
   * @param cluster_objects_msg Input cluster objects with features
   * @param tracked_objects Input tracked objects to be processed
   * @param out_objects Output detected objects after processing
   */
  void process(
    const DetectedObjectsWithFeature & cluster_objects_msg, DetectedObjects & tracked_objects,
    DetectedObjects & out_objects);

private:
  /**
   * @brief Get clusters that are relevant to tracked objects
   *
   * This function matches clusters near tracked objects and prepares them for processing
   *
   * @param tracked_objects Tracked objects to match with clusters
   * @param input_cluster_objects Input cluster objects
   * @return ClustersWithPose Converted pcl clusters with initial size, polygon and pose information
   */
  ClustersWithPose getRelevantClusters(
    const DetectedObjects & tracked_objects,
    const DetectedObjectsWithFeature & input_cluster_objects) const;

  /**
   * @brief Convert DetectedObjectWithFeature to ClusterWithPose
   *
   * @param object_with_feature Detected object with feature to convert
   * @return ClusterWithPose Converted cluster with pose information
   */

  ClusterWithPose objectWithFeatureToClusterWithPose(
    const tier4_perception_msgs::msg::DetectedObjectWithFeature & object_with_feature) const;

  /**
   * @brief Split clusters that intersect with a tracked object's polygon
   *
   * @param extended_tracked_object Tracked object with extended polygon
   * @param cluster_objects Clusters to process
   * @param points_inside_tracked_polygon Output points inside the tracked polygon
   */
  void splitClustersWithPolygon(
    const ExtendedTrackedObject & extended_tracked_object, ClustersWithPose & cluster_objects,
    pcl::PointCloud<pcl::PointXYZ> & points_inside_tracked_polygon);

  /**
   * @brief Split objects using tracked information and clusters
   *
   * @param tracked_objects Tracked objects to process
   * @param clusters_with_pose Clusters with pose information
   * @param out_objects Output detected objects
   */
  void processObjects(
    DetectedObjects & tracked_objects, ClustersWithPose & clusters_with_pose,
    DetectedObjects & out_objects);

  /**
   * @brief Process clusters that may contain multiple objects
   *
   * This function filters out not modified clusters, re-cluster under-segmented clusters and adds
   * them to output
   *
   * @param clusters_with_pose Clusters that may contain multiple objects
   * @param out_objects Output detected objects
   */
  void processClusters(ClustersWithPose & clusters_with_pose, DetectedObjects & out_objects);

  /**
   * @brief Get extended polygon representation of an object
   *
   * @param object_to_expand Object to create extended polygon from
   * @return Polygon2d Extended polygon
   */
  Polygon2d getExtendedPolygon(const DetectedObject & object_to_expand) const;

  /**
   * @brief Check if a tracker should be processed
   *
   * @param tracked_object Tracked object to check
   * @return true If the tracker should be processed
   * @return false If the tracker should be ignored
   */
  bool shouldProcessTrackedObject(const DetectedObject & tracked_object) const;

  /**
   * @brief Update tracked object shape and position with point cloud
   *
   * @param tracked_object Tracked object to update
   * @param pcl_cluster Point cloud for shape estimation
   * @return true If update was successful
   * @return false If update failed
   */
  bool updateTrackedObject(
    DetectedObject & tracked_object, const pcl::PointCloud<pcl::PointXYZ> & pcl_cluster);

  /**
   * @brief Update object's existence probability
   *
   * @param tracked_object Object to update
   */
  void updateExistenceProbability(DetectedObject & tracked_object);

  /**
   * @brief Re-cluster under-segmented clusters
   *
   * @param under_segmented_clusters Clusters that may contain multiple objects
   * @return DetectedObjectsWithFeature Re-clustered objects
   */
  DetectedObjectsWithFeature clusterDividedClusters(
    const ClustersWithPose & under_segmented_clusters);

  /**
   * @brief Convert cluster objects to DetectedObjects format
   *
   * @param cluster_objects Cluster objects to convert
   * @param out_objects Output detected objects
   */
  DetectedObjects clusterObjectsToDetectedObjects(
    const DetectedObjectsWithFeature & cluster_objects);

  /**
   * @brief Get maximum search distance for a specific object classification
   *
   * @param classification The classification array of the object
   * @return Maximum search distance for the object's primary label
   * @throw std::out_of_range if classification is empty or label not in map
   */
  int getMaxSearchDistance(
    const autoware_perception_msgs::msg::DetectedObject::_classification_type & classification)
    const
  {
    auto label = getLabel(classification);
    if (max_search_distance_map_.find(label) == max_search_distance_map_.end()) {
      throw std::out_of_range("Label not found in max search distance map");
    }
    return max_search_distance_map_.at(label);
  }

  /**
   * @brief Safely get primary label from object classification
   *
   * @param classification Object classification array
   * @param default_label Label to return if classification is empty
   * @return Label value from classification or default
   * @throw std::out_of_range if classification is empty and no default provided
   */
  Label::_label_type getLabel(
    const autoware_perception_msgs::msg::DetectedObject::_classification_type & classification)
    const
  {
    if (classification.empty()) {
      throw std::out_of_range("No classification for tracked object");
    }
    return classification.front().label;
  }

  // Member variables
  std::map<object_recognition_utils::ObjectClassification::_label_type, int>
    max_search_distance_map_;
  detection_by_tracker::utils::TrackerIgnoreLabel tracker_ignore_;
  autoware::shape_estimation::ShapeEstimator shape_estimator_;
  std::shared_ptr<autoware::euclidean_cluster::EuclideanClusterInterface> cluster_;
  double extend_scale_;
  double buffer_distance_;
  double existence_probability_threshold_;
  double existence_probability_modifier_;
  bool fast_spawn_unknown_clusters_;
  rclcpp::Logger logger_;
};

}  // namespace detection_by_tracker
}  // namespace autoware

#endif  // OBJECT_SPLITTER_HPP_