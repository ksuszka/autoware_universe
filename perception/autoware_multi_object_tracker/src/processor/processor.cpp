// Copyright 2024 TIER IV, Inc.
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

#include "processor.hpp"

#include "autoware/multi_object_tracker/object_model/object_model.hpp"
#include "autoware/multi_object_tracker/object_model/shapes.hpp"
#include "autoware/multi_object_tracker/object_model/types.hpp"
#include "autoware/multi_object_tracker/tracker/tracker.hpp"

#include <autoware/object_recognition_utils/object_recognition_utils.hpp>
#include <autoware_utils/geometry/boost_geometry.hpp>

#include <autoware_perception_msgs/msg/tracked_objects.hpp>

#include <boost/geometry/algorithms/detail/intersects/interface.hpp>

#include <iterator>
#include <map>
#include <memory>
#include <string>
#include <unordered_map>
#include <utility>
#include <vector>

namespace autoware::multi_object_tracker
{
using autoware_utils::ScopedTimeTrack;
using Label = autoware_perception_msgs::msg::ObjectClassification;
using LabelType = autoware_perception_msgs::msg::ObjectClassification::_label_type;

namespace bg = boost::geometry;
using Box = bg::model::box<autoware_utils::Point2d>;

TrackerProcessor::TrackerProcessor(
  const TrackerProcessorConfig & config, const AssociatorConfig & associator_config,
  const std::vector<types::InputChannel> & channels_config)
: config_(config), channels_config_(channels_config)
{
  association_ = std::make_unique<DataAssociation>(associator_config);
}

void TrackerProcessor::predict(const rclcpp::Time & time)
{
  std::unique_ptr<ScopedTimeTrack> st_ptr;
  if (time_keeper_) st_ptr = std::make_unique<ScopedTimeTrack>(__func__, *time_keeper_);

  for (auto itr = list_tracker_.begin(); itr != list_tracker_.end(); ++itr) {
    (*itr)->predict(time);
  }
}

void TrackerProcessor::associate(
  const types::DynamicObjectList & detected_objects,
  std::unordered_map<int, int> & direct_assignment,
  std::unordered_map<int, int> & reverse_assignment) const
{
  std::unique_ptr<ScopedTimeTrack> st_ptr;
  if (time_keeper_) st_ptr = std::make_unique<ScopedTimeTrack>(__func__, *time_keeper_);

  const auto & tracker_list = list_tracker_;
  // global nearest neighbor
  Eigen::MatrixXd score_matrix = association_->calcScoreMatrix(
    detected_objects, tracker_list);  // row : tracker, col : measurement
  association_->assign(score_matrix, direct_assignment, reverse_assignment);
}

void TrackerProcessor::update(
  const types::DynamicObjectList & detected_objects,
  const std::unordered_map<int, int> & direct_assignment)
{
  std::unique_ptr<ScopedTimeTrack> st_ptr;
  if (time_keeper_) st_ptr = std::make_unique<ScopedTimeTrack>(__func__, *time_keeper_);

  int tracker_idx = 0;
  const auto & time = detected_objects.header.stamp;
  for (auto tracker_itr = list_tracker_.begin(); tracker_itr != list_tracker_.end();
       ++tracker_itr, ++tracker_idx) {
    if (direct_assignment.find(tracker_idx) != direct_assignment.end()) {
      // found
      const auto & associated_object =
        detected_objects.objects.at(direct_assignment.find(tracker_idx)->second);
      const types::InputChannel channel_info = channels_config_[associated_object.channel_index];
      (*(tracker_itr))->updateWithMeasurement(associated_object, time, channel_info);

    } else {
      // not found
      (*(tracker_itr))->updateWithoutMeasurement(time);
    }
  }
}

static std::vector<autoware_utils::Polygon2d> bufferPolygon2d(
  const autoware_utils::Polygon2d & polygon, const double buffer_distance)
{
  bg::strategy::buffer::distance_symmetric<double> distance_strategy(buffer_distance);
  bg::strategy::buffer::join_miter join_strategy;
  bg::strategy::buffer::end_flat end_strategy;
  bg::strategy::buffer::side_straight side_strategy;
  bg::strategy::buffer::point_square point_strategy;
  std::vector<autoware_utils::Polygon2d> buffered_polygons;
  bg::buffer(
    polygon, buffered_polygons, distance_strategy, side_strategy, join_strategy, end_strategy,
    point_strategy);
  return buffered_polygons;
}

bool TrackerProcessor::isLargerThanChild(
  const types::DynamicObject & object)
{
  // Dimensions check
  const auto & shape = object.shape;
  if (shape.dimensions.z < config_.min_child_height) {
    return false;
  }

  if (shape.type == autoware_perception_msgs::msg::Shape::BOUNDING_BOX) {
    if (
      shape.dimensions.x < config_.min_box_size_xy || shape.dimensions.y < config_.min_box_size_xy)
      return false;

  } else if (shape.type == autoware_perception_msgs::msg::Shape::CYLINDER) {
    if (shape.dimensions.x < config_.min_cylinder_radius) {
      return false;
    }

  } else if (shape.type == autoware_perception_msgs::msg::Shape::POLYGON) {
    const auto polygon = autoware_utils::to_polygon2d(object.pose, shape);
    if (bg::area(polygon) < config_.min_polygon_area) {
      const auto shrunk_polygons = bufferPolygon2d(polygon, -config_.polygon_shrink_buffer);
      if (shrunk_polygons.size() == 0) {
        return false;
      }
    }
  }
  return true;
}

void TrackerProcessor::spawn(
  const types::DynamicObjectList & detected_objects,
  const std::unordered_map<int, int> & reverse_assignment)
{
  std::unique_ptr<ScopedTimeTrack> st_ptr;
  if (time_keeper_) st_ptr = std::make_unique<ScopedTimeTrack>(__func__, *time_keeper_);

  const auto channel_config = channels_config_[detected_objects.channel_index];
  // If spawn is disabled, return
  if (!channel_config.is_spawn_enabled) {
    return;
  }

  // Spawn new trackers for the objects that are not associated
  const auto & time = detected_objects.header.stamp;
  for (size_t i = 0; i < detected_objects.objects.size(); ++i) {
    if (reverse_assignment.find(i) != reverse_assignment.end()) {  // found
      continue;
    }
    const auto & new_object = detected_objects.objects.at(i);
    std::shared_ptr<Tracker> tracker = createNewTracker(new_object, time);

    // Initialize existence probabilities
    if (channel_config.trust_existence_probability) {
      tracker->initializeExistenceProbabilities(
        new_object.channel_index, new_object.existence_probability);
    } else {
      tracker->initializeExistenceProbabilities(new_object.channel_index, 0.5);
    }

    if (
      new_object.existence_probability >= config_.min_unknown_object_add_existence_prob ||
      (object_recognition_utils::getHighestProbLabel(new_object.classification) ==
          Label::UNKNOWN &&
        isLargerThanChild(new_object))) {
      // Add object immediately to the trackers
      tracker->setTotalMeasurementCount(config_.confident_count_threshold.at(Label::UNKNOWN));
    }

    // Update the tracker with the new object
    list_tracker_.push_back(tracker);
  }
}

std::shared_ptr<Tracker> TrackerProcessor::createNewTracker(
  const types::DynamicObject & object, const rclcpp::Time & time) const
{
  const LabelType label =
    autoware::object_recognition_utils::getHighestProbLabel(object.classification);
  if (config_.tracker_map.count(label) != 0) {
    const auto tracker = config_.tracker_map.at(label);
    if (tracker == "bicycle_tracker")
      return std::make_shared<VehicleTracker>(object_model::bicycle, time, object);
    if (tracker == "big_vehicle_tracker")
      return std::make_shared<VehicleTracker>(object_model::big_vehicle, time, object);
    if (tracker == "multi_vehicle_tracker")
      return std::make_shared<MultipleVehicleTracker>(time, object);
    if (tracker == "normal_vehicle_tracker")
      return std::make_shared<VehicleTracker>(object_model::normal_vehicle, time, object);
    if (tracker == "pass_through_tracker")
      return std::make_shared<PassThroughTracker>(time, object);
    if (tracker == "pedestrian_and_bicycle_tracker")
      return std::make_shared<PedestrianAndBicycleTracker>(time, object);
    if (tracker == "pedestrian_tracker") return std::make_shared<PedestrianTracker>(time, object);
  }
  return std::make_shared<UnknownTracker>(time, object);
}

void TrackerProcessor::prune(const rclcpp::Time & time)
{
  std::unique_ptr<ScopedTimeTrack> st_ptr;
  if (time_keeper_) st_ptr = std::make_unique<ScopedTimeTrack>(__func__, *time_keeper_);

  // Check tracker lifetime: if the tracker is old, delete it
  removeOldTracker(time);
  // Check tracker overlap
  removeOverlappedTracker(time);
}

void TrackerProcessor::removeOldTracker(const rclcpp::Time & time)
{
  std::unique_ptr<ScopedTimeTrack> st_ptr;
  if (time_keeper_) st_ptr = std::make_unique<ScopedTimeTrack>(__func__, *time_keeper_);

  // Check elapsed time from last update
  for (auto itr = list_tracker_.begin(); itr != list_tracker_.end(); ++itr) {
    bool is_old = false;
    auto lifetime = config_.tracker_lifetime;
    if ((*itr)->getHighestProbLabel() == Label::UNKNOWN) {
      // Limit the lifetime of UNKNOWN trackers to reduce the number of empty UNKNOWN trackers,
      // as they are spawned faster with the isLargerThanChild check.
      lifetime = config_.unknown_lifetime;
    }
    is_old = lifetime < (*itr)->getElapsedTimeFromLastUpdate(time);
    // If the tracker is old, delete it
    if (is_old) {
      auto erase_itr = itr;
      --itr;
      list_tracker_.erase(erase_itr);
    }
  }
}

struct TrackerData
{
  types::DynamicObject object;
  autoware_utils::Polygon2d polygon;
  Box bbox;
};

static std::unordered_map<std::shared_ptr<Tracker>, TrackerData> calculateTrackerCache(
  std::vector<std::shared_ptr<Tracker>> & trackers, const rclcpp::Time & time)
{
  std::unordered_map<std::shared_ptr<Tracker>, TrackerData> tracker_cache;
  for (auto itr = trackers.begin(); itr != trackers.end();) {
    types::DynamicObject obj;
    if ((*itr)->getTrackedObject(time, obj)) {
      const auto polygon = autoware_utils::to_polygon2d(obj.pose, obj.shape);
      Box boost_bbox;
      bg::envelope(polygon, boost_bbox);
      tracker_cache[*itr] = {obj, polygon, boost_bbox};
      ++itr;
    } else {
      itr = trackers.erase(itr);
    }
  }
  return tracker_cache;
}

static bool overlappedObjects(const TrackerData & tracker1, const TrackerData & tracker2, double distance_threshold_sq)
{
  const auto & obj1 = tracker1.object;
  const auto & obj2 = tracker2.object;
  const double dx = obj1.pose.position.x - obj2.pose.position.x;
  const double dy = obj1.pose.position.y - obj2.pose.position.y;
  const double distance_sq = dx * dx + dy * dy;

  return distance_sq < distance_threshold_sq ||
         bg::intersects(tracker1.bbox, tracker2.bbox) || bg::within(tracker1.bbox, tracker2.bbox) ||
         bg::within(tracker2.bbox, tracker1.bbox);
}

static double getIoU(double intersection_area, double union_area) {
  return union_area < 1E-2 ? 0.0 : std::min(1.0, intersection_area / union_area);
}

void TrackerProcessor::removeOverlappedTracker(const rclcpp::Time & time)
{
  std::unique_ptr<ScopedTimeTrack> st_ptr;
  if (time_keeper_) st_ptr = std::make_unique<ScopedTimeTrack>(__func__, *time_keeper_);

  // Create sorted list with non-UNKNOWN objects first, then by measurement count
  std::vector<std::shared_ptr<Tracker>> sorted_list_tracker(
    list_tracker_.begin(), list_tracker_.end());
  std::sort(
    sorted_list_tracker.begin(), sorted_list_tracker.end(),
    [&time](const std::shared_ptr<Tracker> & a, const std::shared_ptr<Tracker> & b) {
      bool a_unknown = (a->getHighestProbLabel() == Label::UNKNOWN);
      bool b_unknown = (b->getHighestProbLabel() == Label::UNKNOWN);
      if (a_unknown != b_unknown) {
        return b_unknown;  // Put non-UNKNOWN objects first
      }
      if (a->getTotalMeasurementCount() != b->getTotalMeasurementCount()) {
        return a->getTotalMeasurementCount() >
               b->getTotalMeasurementCount();  // Then sort by measurement count
      }
      return a->getElapsedTimeFromLastUpdate(time) <
             b->getElapsedTimeFromLastUpdate(time);  // Finally sort by elapsed time (smaller first)
    });

  /* Iterate through the list of trackers */
  auto tracker_cache = calculateTrackerCache(sorted_list_tracker, time);
  for (size_t i = 0; i < sorted_list_tracker.size(); ++i) {
    const auto & cache_it1 = tracker_cache.find(sorted_list_tracker[i]);
    if (cache_it1 == tracker_cache.end()) {
      continue;
    }
    const auto & tracker_data1 = cache_it1->second;
    const auto & object1_polygon = tracker_data1.polygon;

    // Compare the current tracker with the remaining trackers
    for (size_t j = i + 1; j < sorted_list_tracker.size(); ++j) {
      const auto & cache_it2 = tracker_cache.find(sorted_list_tracker[j]);
      if (cache_it2 == tracker_cache.end()) {
        continue;
      }
      const auto & tracker_data2 = cache_it2->second;
      const auto & object2_polygon = tracker_data2.polygon;

      if (!overlappedObjects(tracker_data1, tracker_data2, config_.distance_threshold_sq)) {
        continue;
      }

      // Check the Intersection over Union (IoU) between the two objects
      const double object1_area = bg::area(object1_polygon);
      if (object1_area < 1E-6) {
        continue;
      }
      const double object2_area = bg::area(object2_polygon);
      if (object2_area < 1E-6) {
        continue;
      }
      const double intersection_area =
        object_recognition_utils::getIntersectionArea(object1_polygon, object2_polygon);
      if (intersection_area < 1E-6) {
        continue;
      }
      const double union_area = object_recognition_utils::getUnionArea(object1_polygon, object2_polygon);
      const double iou = getIoU(intersection_area, union_area);

      const auto & label1 = sorted_list_tracker[i]->getHighestProbLabel();
      const auto & label2 = sorted_list_tracker[j]->getHighestProbLabel();

      const double object1_overlap_ratio = intersection_area / object1_area;
      const double object2_overlap_ratio = intersection_area / object2_area;
      bool exceeded_overlap_ratio = object1_overlap_ratio > config_.min_object_removal_overlap ||
                                    object2_overlap_ratio > config_.min_object_removal_overlap;

      bool should_delete_tracker1 = false;
      bool should_delete_tracker2 = false;
      // Case 1: At least one tracker is UNKNOWN
      if (label1 == Label::UNKNOWN || label2 == Label::UNKNOWN) {
        // Case 1.1: Both trackers are UNKNOWN
        if (label1 == Label::UNKNOWN && label2 == Label::UNKNOWN) {
          if (exceeded_overlap_ratio) {
            if (object1_overlap_ratio > object2_overlap_ratio) {
              should_delete_tracker1 = true;
            } else {
              should_delete_tracker2 = true;
            }
          } else if (iou > config_.min_unknown_object_removal_iou_with_unknown) {
            if (sorted_list_tracker[i]->getTotalMeasurementCount() < sorted_list_tracker[j]->getTotalMeasurementCount()) {
              should_delete_tracker1 = true;
            } else {
              should_delete_tracker2 = true;
            }
          }
          // Case 1.2: Only tracker1 is UNKNOWN
        } else if (label1 == Label::UNKNOWN) {
          if (
            iou > config_.min_unknown_object_removal_iou_with_known ||
            object1_overlap_ratio > config_.min_object_removal_overlap) {
            should_delete_tracker1 = true;
          }
          // Case 1.3: Only tracker2 is UNKNOWN
        } else if (label2 == Label::UNKNOWN) {
          if (
            iou > config_.min_unknown_object_removal_iou_with_known ||
            object2_overlap_ratio > config_.min_object_removal_overlap) {
            should_delete_tracker2 = true;
          }
        }
        // Case 2: Both trackers are KNOWN
      } else if (exceeded_overlap_ratio) {
        if (object1_overlap_ratio > object2_overlap_ratio) {
          should_delete_tracker1 = true;
        } else {
          should_delete_tracker2 = true;
        }
      } else if (iou > config_.min_known_object_removal_iou) {
        if (sorted_list_tracker[i]->getTotalMeasurementCount() < sorted_list_tracker[j]->getTotalMeasurementCount()) {
          should_delete_tracker1 = true;
        } else {
          should_delete_tracker2 = true;
        }
      }

      auto remove = [&sorted_list_tracker, this](size_t & index) {
          // Remove from original list_tracker
          list_tracker_.remove(sorted_list_tracker[index]);
          // Remove from sorted list
          sorted_list_tracker.erase(sorted_list_tracker.begin() + index);
          --index;
      };

      // Delete the tracker
      if (
        should_delete_tracker1 && isConfidentTracker(sorted_list_tracker[j]) &&
        (label1 != Label::UNKNOWN || (sorted_list_tracker[i]->getTotalExistenceProbability() <
                                      config_.min_unknown_object_add_existence_prob))) {
        remove(i);
        break;
      }
      if (
        should_delete_tracker2 && isConfidentTracker(sorted_list_tracker[i]) &&
        (label2 != Label::UNKNOWN || (sorted_list_tracker[j]->getTotalExistenceProbability() <
                                      config_.min_unknown_object_add_existence_prob))) {
        remove(j);
      }
    }
  }
}

bool TrackerProcessor::isConfidentTracker(const std::shared_ptr<Tracker> & tracker) const
{
  // Confidence is determined by counting the number of measurements.
  // If the number of measurements is equal to or greater than the threshold, the tracker is
  // considered confident.
  auto label = tracker->getHighestProbLabel();
  return (
    tracker->getTotalMeasurementCount() >= config_.confident_count_threshold.at(label) ||
    tracker->getTotalExistenceProbability() > config_.min_unknown_object_add_existence_prob);
}

void TrackerProcessor::getTrackedObjects(
  const rclcpp::Time & time, autoware_perception_msgs::msg::TrackedObjects & tracked_objects) const
{
  std::unique_ptr<ScopedTimeTrack> st_ptr;
  if (time_keeper_) st_ptr = std::make_unique<ScopedTimeTrack>(__func__, *time_keeper_);

  tracked_objects.header.stamp = time;
  types::DynamicObject tracked_object;
  for (const auto & tracker : list_tracker_) {
    // Skip if the tracker is not confident
    if (!isConfidentTracker(tracker)) {
      continue;
    }
    // Get the tracked object, extrapolated to the given time
    if (tracker->getTrackedObject(time, tracked_object)) {
      tracked_objects.objects.push_back(toTrackedObjectMsg(tracked_object));
    }
  }
}

void TrackerProcessor::getTentativeObjects(
  const rclcpp::Time & time,
  autoware_perception_msgs::msg::TrackedObjects & tentative_objects) const
{
  std::unique_ptr<ScopedTimeTrack> st_ptr;
  if (time_keeper_) st_ptr = std::make_unique<ScopedTimeTrack>(__func__, *time_keeper_);

  tentative_objects.header.stamp = time;
  types::DynamicObject tracked_object;
  for (const auto & tracker : list_tracker_) {
    if (!isConfidentTracker(tracker)) {
      if (tracker->getTrackedObject(time, tracked_object)) {
        tentative_objects.objects.push_back(toTrackedObjectMsg(tracked_object));
      }
    }
  }
}

void TrackerProcessor::setTimeKeeper(std::shared_ptr<autoware_utils::TimeKeeper> time_keeper_ptr)
{
  time_keeper_ = std::move(time_keeper_ptr);
}

}  // namespace autoware::multi_object_tracker
