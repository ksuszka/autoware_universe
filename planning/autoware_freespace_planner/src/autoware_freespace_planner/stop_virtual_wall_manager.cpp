// Copyright (c) 2026 Autonomous Systems Sp. z o.o.
// All rights reserved. Proprietary and confidential.
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

#include "autoware/freespace_planner/stop_virtual_wall_manager.hpp"

#include <autoware/motion_utils/marker/marker_helper.hpp>

namespace autoware::freespace_planner
{
namespace
{
constexpr double MIN_WALL_VISIBLE_DURATION_SEC = 1.0;
}  // namespace

StopVirtualWallManager::StopVirtualWallManager(rclcpp::Node & node, double base_link2front)
: node_(node),
  publisher_(node.create_publisher<visualization_msgs::msg::MarkerArray>("~/virtual_wall", 1)),
  base_link2front_(base_link2front)
{
}

void StopVirtualWallManager::startPlanning(const geometry_msgs::msg::Pose & pose)
{
  vehicle_pose_ = pose;
  collision_pose_ = {};
  if (!planning_failed_) {
    collision_label_.clear();
  }
  clear_pending_ = false;
  publish(Reason::PlanningInProgress);
}

void StopVirtualWallManager::onPlanSuccess()
{
  planning_failed_ = false;
  clear_pending_ = false;
  last_visible_publish_time_ = rclcpp::Time{0, 0, RCL_ROS_TIME};
  publish(Reason::None);
}

void StopVirtualWallManager::onPlanFailed()
{
  planning_failed_ = true;
  publish(Reason::NoPathToGoal);
}

void StopVirtualWallManager::setCollisionContext(
  const geometry_msgs::msg::Pose & pose, const std::string & label)
{
  collision_pose_ = pose;
  collision_label_ = label;
  reason_ = Reason::NoPathToGoal;
}

void StopVirtualWallManager::onObstacle(const geometry_msgs::msg::Pose & pose)
{
  vehicle_pose_ = pose;
  collision_pose_ = {};
  if (reason_ != Reason::ObstacleOnTrajectory) {
    publish(Reason::ObstacleOnTrajectory);
  }
}

void StopVirtualWallManager::onObstacleCleared()
{
  if (reason_ == Reason::ObstacleOnTrajectory) {
    publish(Reason::None);
  }
}

void StopVirtualWallManager::onInactive()
{
  if (reason_ != Reason::None) {
    clear_pending_ = false;
    last_visible_publish_time_ = rclcpp::Time{0, 0, RCL_ROS_TIME};
    publish(Reason::None);
  }
}

void StopVirtualWallManager::republishIfActive()
{
  if (clear_pending_) {
    const auto now = node_.get_clock()->now();
    if ((now - last_visible_publish_time_).seconds() >= MIN_WALL_VISIBLE_DURATION_SEC) {
      publish(Reason::None);
    }
    return;
  }

  if (reason_ != Reason::None && (vehicle_pose_ || collision_pose_)) {
    publish(reason_);
  }
}

void StopVirtualWallManager::publish(Reason reason)
{
  const auto now = node_.get_clock()->now();
  visualization_msgs::msg::MarkerArray markers;

  const auto append_markers = [&markers](const visualization_msgs::msg::MarkerArray & input) {
    markers.markers.insert(markers.markers.end(), input.markers.begin(), input.markers.end());
  };

  if (reason == Reason::None) {
    if (
      reason_ != Reason::None &&
      (now - last_visible_publish_time_).seconds() < MIN_WALL_VISIBLE_DURATION_SEC) {
      clear_pending_ = true;
      return;
    }

    append_markers(autoware::motion_utils::createDeletedStopVirtualWallMarker(now, 0));
    append_markers(autoware::motion_utils::createDeletedStopVirtualWallMarker(now, 1));
    vehicle_pose_ = {};
    collision_pose_ = {};
    collision_label_.clear();
    clear_pending_ = false;
  } else {
    if (!vehicle_pose_ && !collision_pose_) {
      reason_ = reason;
      return;
    }

    std::string primary_text;
    const auto primary_pose = (reason == Reason::NoPathToGoal && collision_pose_)
                                ? collision_pose_
                                : (vehicle_pose_ ? vehicle_pose_ : collision_pose_);

    if (reason == Reason::ObstacleOnTrajectory) {
      primary_text = "REPLAN";
    } else if (reason == Reason::PlanningInProgress) {
      primary_text = "PLANNING";
    } else if (collision_label_ == "start") {
      primary_text = "START BLOCKED";
    } else if (collision_label_ == "goal") {
      primary_text = "GOAL BLOCKED";
    } else {
      primary_text = "BLOCKED";
    }

    if (primary_pose) {
      append_markers(
        autoware::motion_utils::createStopVirtualWallMarker(
          *primary_pose, primary_text, now, 0, base_link2front_));
    }

    // Override default 0.5 s lifetime so the wall survives the blocking makePlan() call
    for (auto & m : markers.markers) {
      m.lifetime = rclcpp::Duration::from_seconds(0.0);  // infinite until explicitly deleted
    }

    clear_pending_ = false;
    last_visible_publish_time_ = now;
  }
  publisher_->publish(markers);
  reason_ = reason;
}

}  // namespace autoware::freespace_planner
