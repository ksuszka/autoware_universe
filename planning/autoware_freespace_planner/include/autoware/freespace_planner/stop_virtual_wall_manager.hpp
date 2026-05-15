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

#ifndef AUTOWARE__FREESPACE_PLANNER__STOP_VIRTUAL_WALL_MANAGER_HPP_
#define AUTOWARE__FREESPACE_PLANNER__STOP_VIRTUAL_WALL_MANAGER_HPP_

#include <rclcpp/rclcpp.hpp>

#include <geometry_msgs/msg/pose.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

#include <optional>
#include <string>

namespace autoware::freespace_planner
{

/// @brief Manages the stop virtual wall marker: owns the publisher, all state
///        (reason, pose, collision label, planning_failed flag), and all publish logic.
class StopVirtualWallManager
{
public:
  enum class Reason { None, ObstacleOnTrajectory, PlanningInProgress, NoPathToGoal };

  StopVirtualWallManager(rclcpp::Node & node, double base_link2front);

  /// Called before makePlan() starts — shows "Preparing the path…" wall.
  void startPlanning(const geometry_msgs::msg::Pose & pose);

  /// Called when planning succeeds — clears the wall.
  void onPlanSuccess();

  /// Called when planning fails — shows "failed to find path" wall.
  void onPlanFailed();

  /// Called by AstarSearch collision observer during makePlan() — caches the precise
  /// collision pose/label so the subsequent publish uses it instead of current_pose.
  void setCollisionContext(const geometry_msgs::msg::Pose & pose, const std::string & label);

  /// Called in isPlanRequired() when a trajectory-blocking obstacle is detected.
  void onObstacle(const geometry_msgs::msg::Pose & pose);

  /// Called in isPlanRequired() when the blocking obstacle is no longer present.
  void onObstacleCleared();

  /// Called in onTimer() when the scenario becomes inactive — clears the wall.
  void onInactive();

  /// Called in onTimer() every tick to keep the marker alive past its default lifetime.
  void republishIfActive();

private:
  rclcpp::Node & node_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr publisher_;
  double base_link2front_;

  Reason reason_{Reason::None};
  std::optional<geometry_msgs::msg::Pose> vehicle_pose_;
  std::optional<geometry_msgs::msg::Pose> collision_pose_;
  std::string collision_label_;
  bool planning_failed_{false};
  bool clear_pending_{false};
  rclcpp::Time last_visible_publish_time_{0, 0, RCL_ROS_TIME};

  void publish(Reason reason);
};

}  // namespace autoware::freespace_planner

#endif  // AUTOWARE__FREESPACE_PLANNER__STOP_VIRTUAL_WALL_MANAGER_HPP_
