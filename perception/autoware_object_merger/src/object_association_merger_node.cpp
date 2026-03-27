// Copyright 2020 Tier IV, Inc.
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

#include <type_traits>
#define EIGEN_MPL2_ONLY

#include "autoware/object_merger/object_association_merger_node.hpp"
#include "autoware/object_recognition_utils/object_recognition_utils.hpp"
#include <autoware_utils/geometry/alt_geometry.hpp>

#include <Eigen/Core>
#include <Eigen/Geometry>

#include <boost/optional.hpp>

#include <chrono>
#include <map>
#include <memory>
#include <string>
#include <unordered_map>
#include <utility>
#include <vector>

#include <boost/geometry/algorithms/buffer.hpp>
#include <cmath>
#include <autoware_utils/geometry/alt_geometry.hpp>

using Label = autoware_perception_msgs::msg::ObjectClassification;

namespace
{
namespace bg = boost::geometry;

bool isUnknownObjectOverlapped(
  const autoware_perception_msgs::msg::DetectedObject & unknown_object,
  const autoware_perception_msgs::msg::DetectedObject & known_object,
  const double precision_threshold, const double recall_threshold,
  const std::map<int, double> & distance_threshold_map,
  const std::map<int, double> & generalized_iou_threshold_map)
{
  const double generalized_iou_threshold = generalized_iou_threshold_map.at(
    autoware::object_recognition_utils::getHighestProbLabel(known_object.classification));
  const double distance_threshold = distance_threshold_map.at(
    autoware::object_recognition_utils::getHighestProbLabel(known_object.classification));
  const double sq_distance_threshold = std::pow(distance_threshold, 2.0);
  const double sq_distance = autoware_utils::calc_squared_distance2d(
    unknown_object.kinematics.pose_with_covariance.pose,
    known_object.kinematics.pose_with_covariance.pose);
  if (sq_distance_threshold < sq_distance) return false;
  const auto precision =
    autoware::object_recognition_utils::get2dPrecision(unknown_object, known_object);
  const auto recall = autoware::object_recognition_utils::get2dRecall(unknown_object, known_object);
  const auto generalized_iou =
    autoware::object_recognition_utils::get2dGeneralizedIoU(unknown_object, known_object);
  return precision > precision_threshold || recall > recall_threshold ||
         generalized_iou > generalized_iou_threshold;
}

geometry_msgs::msg::Point calculatePolygonPosition(
  const autoware_utils::Polygon2d & polygon)
{
  double min_x = std::numeric_limits<double>::max();
  double max_x = std::numeric_limits<double>::lowest();
  double min_y = std::numeric_limits<double>::max();
  double max_y = std::numeric_limits<double>::lowest();

  // Calculate min and max coordinates of the polygon
  for (const auto & point : polygon.outer()) {
    min_x = std::min(min_x, point.x());
    max_x = std::max(max_x, point.x());
    min_y = std::min(min_y, point.y());
    max_y = std::max(max_y, point.y());
  }

  const auto length = max_x - min_x;
  const auto width = max_y - min_y;

  geometry_msgs::msg::Point position;
  position.x = min_x + length / 2.0;
  position.y = min_y + width / 2.0;

  return position;
}

geometry_msgs::msg::Polygon polygon2dToFootprint(
  const autoware_utils::Polygon2d & polygon, geometry_msgs::msg::Point position)
{
  geometry_msgs::msg::Polygon footprint;
  footprint.points.resize(polygon.outer().size());
  std::transform(
    polygon.outer().begin(), polygon.outer().end(), footprint.points.begin(),
    [&position](auto & polygon_point) {
      geometry_msgs::msg::Point32 point32;
      // Update the object's footprint to be relative to the object's position
      point32.x = polygon_point.x() - position.x;
      point32.y = polygon_point.y() - position.y;
      return point32;
    });
  return footprint;
}

// Creates a extended / shrunk polygons by buffer_distance
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
  return buffed_polygons;
}

// Filters out too small and thin parts of the polygon by eroding and dilating it
std::vector<autoware_utils::Polygon2d> filterPolygonThinParts(
  const autoware_utils::Polygon2d & polygon, double buffer_distance, double min_area)
{
  std::vector<autoware_utils::Polygon2d> filtered_polygons;
  // Erode and dilate the polygon to filter out thin parts
  auto eroded_polygons = bufferPolygon2d(polygon, -buffer_distance);
  for (auto & eroded_polygon : eroded_polygons) {
    if (eroded_polygon.outer().empty()) {
      continue;
    }
    auto opened_polygons = bufferPolygon2d(eroded_polygon, buffer_distance);
    for (auto & opened_polygon : opened_polygons) {
      auto polygon_area = std::abs(bg::area(opened_polygon));
      if (polygon_area < min_area) {
        continue;
      }
      // Check if the polygon is clockwise, if not, inverse it since Autoware Universe uses
      // counter-clockwise
      if (autoware_utils::is_clockwise(opened_polygon)) {
        opened_polygon = autoware_utils::inverse_clockwise(opened_polygon);
      }
      filtered_polygons.push_back(opened_polygon);
    }
  }
  return filtered_polygons;
}

/**
 * @brief Subtracts the known object from the unknown object and returns the resulting objects.
 *
 * This function takes two detected objects (unknown and known), converts them to 2D polygons,
 * and subtracts the known polygon from the unknown polygon. Result polygons are eroded and dilated
 * to filter out extremely thin parts of polygons. It returns the resulting polygons as detected
 * objects, filtering out any small polygons below a specified minimum area.
 *
 * @param unknown_object The detected object that label is unknown.
 * @param known_object The detected object that label is known.
 * @param buffer_distance The distance to morphologically open the known polygon to reduce the
 * number of excessively thin polygon parts.
 * @param min_area The minimum area threshold for the resulting polygons to be considered valid.
 * @return A vector of detected objects representing the subtracted polygons.
 */
std::vector<autoware_perception_msgs::msg::DetectedObject> getSubtractedObjects2D(
  const autoware_perception_msgs::msg::DetectedObject & unknown_object,
  const autoware_perception_msgs::msg::DetectedObject & known_object, const double buffer_distance,
  const double min_area)
{
  using autoware_utils::alt::ConvexPolygon2d;
  // Convert DetectedObject to Polygon2d
  const auto unknown_polygon = autoware_utils::to_polygon2d(unknown_object);
  const auto known_polygon = autoware_utils::to_polygon2d(known_object);

  // Check if unknown object is within known object
  bool is_unknown_within_known = autoware_utils::within(
    ConvexPolygon2d::create(unknown_polygon).value(),
    ConvexPolygon2d::create(known_polygon).value());

  // Additional check in case of boost geometry error. Should be fixed by upgrading boost version
  // to >1.79 or by replacing boost geometry difference with self-implemented functions
  // https://github.com/autowarefoundation/autoware.universe/issues/8128
  if (is_unknown_within_known) {
    return {};
  }

  // Subtract unknown polygon from known polygon
  std::vector<autoware_utils::Polygon2d> not_overlapping_unknown_polygons;
  bg::difference(unknown_polygon, known_polygon, not_overlapping_unknown_polygons);

  // If unknown polygon is not overlapping with known polygon, return unknown object as it is
  if (not_overlapping_unknown_polygons.empty()) {
    return {unknown_object};
  }

  std::vector<autoware_perception_msgs::msg::DetectedObject> not_overlapping_unknown_objects;
  for (auto & polygon : not_overlapping_unknown_polygons) {
    auto opened_polygons = filterPolygonThinParts(polygon, buffer_distance, min_area);
    for (auto & opened_polygon : opened_polygons) {
      autoware_perception_msgs::msg::DetectedObject detected_object = unknown_object;
      detected_object.kinematics.pose_with_covariance.pose.position =
        calculatePolygonPosition(opened_polygon);
      detected_object.kinematics.pose_with_covariance.pose.position.z =
        unknown_object.kinematics.pose_with_covariance.pose.position.z;
      detected_object.shape.footprint = polygon2dToFootprint(
        opened_polygon, detected_object.kinematics.pose_with_covariance.pose.position);
      not_overlapping_unknown_objects.push_back(detected_object);
    }
  }
  return not_overlapping_unknown_objects;
}
}  // namespace

namespace
{
std::map<int, double> convertListToClassMap(const std::vector<double> & distance_threshold_list)
{
  std::map<int /*class label*/, double /*distance_threshold*/> distance_threshold_map;
  int class_label = 0;
  for (const auto & distance_threshold : distance_threshold_list) {
    distance_threshold_map.insert(std::make_pair(class_label, distance_threshold));
    class_label++;
  }
  return distance_threshold_map;
}
}  // namespace

namespace autoware::object_merger
{
ObjectAssociationMergerNode::ObjectAssociationMergerNode(const rclcpp::NodeOptions & node_options)
: rclcpp::Node("object_association_merger_node", node_options),
  tf_buffer_(get_clock()),
  tf_listener_(tf_buffer_),
  object0_sub_(this, "input/object0", rclcpp::QoS{1}.get_rmw_qos_profile()),
  object1_sub_(this, "input/object1", rclcpp::QoS{1}.get_rmw_qos_profile())
{
  // Parameters
  base_link_frame_id_ = declare_parameter<std::string>("base_link_frame_id");
  priority_mode_ = static_cast<PriorityMode>(declare_parameter<int>("priority_mode"));
  sync_queue_size_ = declare_parameter<int>("sync_queue_size");
  remove_overlapped_unknown_objects_ = declare_parameter<bool>("remove_overlapped_unknown_objects");
  separate_unknown_objects_from_known_ =
    declare_parameter<bool>("separate_unknown_objects_from_known");
  separated_opening_distance_ = declare_parameter<double>("separated_opening_distance");
  separated_min_area_ = declare_parameter<double>("separated_min_area");
  overlapped_judge_param_.precision_threshold =
    declare_parameter<double>("precision_threshold_to_judge_overlapped");
  overlapped_judge_param_.recall_threshold =
    declare_parameter<double>("recall_threshold_to_judge_overlapped");
  overlapped_judge_param_.generalized_iou_threshold =
    convertListToClassMap(declare_parameter<std::vector<double>>("generalized_iou_threshold"));

  // get distance_threshold_map from distance_threshold_list
  /** TODO(Shin-kyoto):
   *  this implementation assumes index of vector shows class_label.
   *  if param supports map, refactor this code.
   */
  overlapped_judge_param_.distance_threshold_map =
    convertListToClassMap(declare_parameter<std::vector<double>>("distance_threshold_list"));

  const auto tmp = this->declare_parameter<std::vector<int64_t>>("can_assign_matrix");
  const std::vector<int> can_assign_matrix(tmp.begin(), tmp.end());
  const auto max_dist_matrix = this->declare_parameter<std::vector<double>>("max_dist_matrix");
  const auto max_rad_matrix = this->declare_parameter<std::vector<double>>("max_rad_matrix");
  const auto min_iou_matrix = this->declare_parameter<std::vector<double>>("min_iou_matrix");
  data_association_ = std::make_unique<autoware::object_merger::DataAssociation>(
    can_assign_matrix, max_dist_matrix, max_rad_matrix, min_iou_matrix);

  // Create publishers and subscribers
  using std::placeholders::_1;
  using std::placeholders::_2;
  sync_ptr_ = std::make_shared<Sync>(SyncPolicy(sync_queue_size_), object0_sub_, object1_sub_);
  sync_ptr_->registerCallback(
    std::bind(&ObjectAssociationMergerNode::objectsCallback, this, _1, _2));

  merged_object_pub_ = create_publisher<autoware_perception_msgs::msg::DetectedObjects>(
    "output/object", rclcpp::QoS{1});

  // Debug publisher
  processing_time_publisher_ =
    std::make_unique<autoware_utils::DebugPublisher>(this, "object_association_merger");
  stop_watch_ptr_ = std::make_unique<autoware_utils::StopWatch<std::chrono::milliseconds>>();
  stop_watch_ptr_->tic("cyclic_time");
  stop_watch_ptr_->tic("processing_time");
  published_time_publisher_ = std::make_unique<autoware_utils::PublishedTimePublisher>(this);
  // Timeout process initialization
  message_timeout_sec_ = this->declare_parameter<double>("message_timeout_sec");
  initialization_timeout_sec_ = this->declare_parameter<double>("initialization_timeout_sec");
  last_sync_time_ = std::nullopt;
  message_interval_ = std::nullopt;
  timeout_timer_ = this->create_wall_timer(
    std::chrono::duration<double>(message_timeout_sec_ / 2),
    std::bind(&ObjectAssociationMergerNode::diagCallback, this));
  diagnostics_interface_ptr_ =
    std::make_unique<autoware_utils::DiagnosticsInterface>(this, "object_association_merger");
}

void ObjectAssociationMergerNode::objectsCallback(
  const autoware_perception_msgs::msg::DetectedObjects::ConstSharedPtr & input_objects0_msg,
  const autoware_perception_msgs::msg::DetectedObjects::ConstSharedPtr & input_objects1_msg)
{
  // Guard
  if (merged_object_pub_->get_subscription_count() < 1) {
    return;
  }
  stop_watch_ptr_->toc("processing_time", true);

  /* transform to base_link coordinate */
  autoware_perception_msgs::msg::DetectedObjects transformed_objects0, transformed_objects1;
  if (
    !autoware::object_recognition_utils::transformObjects(
      *input_objects0_msg, base_link_frame_id_, tf_buffer_, transformed_objects0) ||
    !autoware::object_recognition_utils::transformObjects(
      *input_objects1_msg, base_link_frame_id_, tf_buffer_, transformed_objects1)) {
    return;
  }

  // build output msg
  autoware_perception_msgs::msg::DetectedObjects output_msg;
  output_msg.header = input_objects0_msg->header;
  output_msg.header.frame_id = base_link_frame_id_;

  /* global nearest neighbor */
  std::unordered_map<int, int> direct_assignment, reverse_assignment;
  const auto & objects0 = transformed_objects0.objects;
  const auto & objects1 = transformed_objects1.objects;
  Eigen::MatrixXd score_matrix =
    data_association_->calcScoreMatrix(transformed_objects1, transformed_objects0);
  data_association_->assign(score_matrix, direct_assignment, reverse_assignment);

  for (size_t object0_idx = 0; object0_idx < objects0.size(); ++object0_idx) {
    const auto & object0 = objects0.at(object0_idx);
    if (direct_assignment.find(object0_idx) != direct_assignment.end()) {  // found and merge
      const auto & object1 = objects1.at(direct_assignment.at(object0_idx));
      switch (priority_mode_) {
        case PriorityMode::Object0:
          output_msg.objects.push_back(object0);
          break;
        case PriorityMode::Object1:
          output_msg.objects.push_back(object1);
          break;
        case PriorityMode::Confidence:
          if (object1.existence_probability <= object0.existence_probability)
            output_msg.objects.push_back(object0);
          else
            output_msg.objects.push_back(object1);
          break;
      }
    } else {  // not found
      output_msg.objects.push_back(object0);
    }
  }
  for (size_t object1_idx = 0; object1_idx < objects1.size(); ++object1_idx) {
    const auto & object1 = objects1.at(object1_idx);
    if (reverse_assignment.find(object1_idx) != reverse_assignment.end()) {  // found
    } else {                                                                 // not found
      output_msg.objects.push_back(object1);
    }
  }

  // Remove overlapped unknown object
  if (remove_overlapped_unknown_objects_) {
    std::vector<autoware_perception_msgs::msg::DetectedObject> unknown_objects, known_objects;
    unknown_objects.reserve(output_msg.objects.size());
    known_objects.reserve(output_msg.objects.size());
    for (const auto & object : output_msg.objects) {
      if (
        autoware::object_recognition_utils::getHighestProbLabel(object.classification) ==
        Label::UNKNOWN) {
        unknown_objects.push_back(object);
      } else {
        known_objects.push_back(object);
      }
    }
    output_msg.objects.clear();
    output_msg.objects = known_objects;
    for (const auto & unknown_object : unknown_objects) {
      bool is_overlapped = false;
      for (const auto & known_object : known_objects) {
        if (isUnknownObjectOverlapped(
              unknown_object, known_object, overlapped_judge_param_.precision_threshold,
              overlapped_judge_param_.recall_threshold,
              overlapped_judge_param_.distance_threshold_map,
              overlapped_judge_param_.generalized_iou_threshold)) {
          is_overlapped = true;
          if (separate_unknown_objects_from_known_) {
            auto not_overlapped_objects = getSubtractedObjects2D(
              unknown_object, known_object, separated_opening_distance_, separated_min_area_);
            if (not_overlapped_objects.empty()) {
              break;  // unknown object is within known object
            }
            output_msg.objects.reserve(output_msg.objects.size() + not_overlapped_objects.size());
            std::move(
              not_overlapped_objects.begin(), not_overlapped_objects.end(),
              std::back_inserter(output_msg.objects));
          }
          break;
        }
      }
      if (!is_overlapped) {
        output_msg.objects.push_back(unknown_object);
      }
    }
  }

  // Diagnostics part
  rclcpp::Time now = this->now();
  // Calculate the interval since the last sync,
  // or set to 0.0 if this is the first sync
  if (message_interval_.has_value()) {
    message_interval_ = (now - last_sync_time_.value()).seconds();
  } else {
    // initialize message interval
    message_interval_ = 0.0;
  }
  // Update the last sync time to now
  last_sync_time_ = now;

  // publish output msg
  merged_object_pub_->publish(output_msg);
  published_time_publisher_->publish_if_subscribed(merged_object_pub_, output_msg.header.stamp);
  // publish processing time
  processing_time_publisher_->publish<autoware_internal_debug_msgs::msg::Float64Stamped>(
    "debug/cyclic_time_ms", stop_watch_ptr_->toc("cyclic_time", true));
  processing_time_publisher_->publish<autoware_internal_debug_msgs::msg::Float64Stamped>(
    "debug/processing_time_ms", stop_watch_ptr_->toc("processing_time", true));
}

void ObjectAssociationMergerNode::diagCallback()
{
  rclcpp::Time now = this->now();
  // If the time source is not initialized, return early
  if (now.nanoseconds() == 0) {
    return;
  }
  // Initialize the time source if it hasn't been initialized yet
  if (!last_sync_time_.has_value()) {
    last_sync_time_ = now;
    return;
  }

  const double time_since_last_sync = (now - last_sync_time_.value()).seconds();
  const double message_interval_value = message_interval_.value_or(0.0);
  const double timeout = message_interval_ ? message_timeout_sec_ : initialization_timeout_sec_;
  const bool interval_exceeded = message_interval_value >= message_timeout_sec_;
  const bool elapsed_exceeded = time_since_last_sync >= timeout;
  const bool timeout_occurred = elapsed_exceeded || interval_exceeded;
  diagnostics_interface_ptr_->clear();
  diagnostics_interface_ptr_->add_key_value("timeout_occurred", timeout_occurred);
  diagnostics_interface_ptr_->add_key_value("elapsed_time_since_sync", time_since_last_sync);
  diagnostics_interface_ptr_->add_key_value("messages_interval", message_interval_value);
  std::string message;
  if (elapsed_exceeded) {
    const std::string prefix = message_interval_
                                 ? "No recent messages received or synchronized"
                                 : "No synchronized messages received since startup";
    message = "[WARN] " + prefix + " - Elapsed time " + std::to_string(time_since_last_sync) +
              "s exceeded timeout threshold of " + std::to_string(timeout) + "s.";
  } else if (interval_exceeded) {
    message = "[WARN] Message interval " + std::to_string(message_interval_value) +
              "s exceeded allowed interval of " + std::to_string(message_timeout_sec_) + "s.";
  } else {
    message = "[OK] Status is normal.";
  }
  diagnostics_interface_ptr_->update_level_and_message(
    timeout_occurred ? diagnostic_msgs::msg::DiagnosticStatus::WARN
                     : diagnostic_msgs::msg::DiagnosticStatus::OK,
    message);
  diagnostics_interface_ptr_->publish(now);
}

}  // namespace autoware::object_merger

#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(autoware::object_merger::ObjectAssociationMergerNode)
