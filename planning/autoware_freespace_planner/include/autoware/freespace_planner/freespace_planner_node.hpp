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

/*
 * Copyright 2018-2019 Autoware Foundation. All rights reserved.
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#ifndef AUTOWARE__FREESPACE_PLANNER__FREESPACE_PLANNER_NODE_HPP_
#define AUTOWARE__FREESPACE_PLANNER__FREESPACE_PLANNER_NODE_HPP_

#include "autoware/freespace_planner/planning_stats.hpp"
#include "autoware/freespace_planner/stop_virtual_wall_manager.hpp"
#include "autoware_utils/ros/logger_level_configure.hpp"

#include <autoware/freespace_planning_algorithms/astar_search.hpp>
#include <autoware/freespace_planning_algorithms/rrtstar.hpp>
#include <autoware_utils/ros/polling_subscriber.hpp>
#include <autoware_vehicle_info_utils/vehicle_info_utils.hpp>
#include <diagnostic_updater/diagnostic_updater.hpp>
#include <rclcpp/rclcpp.hpp>

#include <autoware_internal_debug_msgs/msg/float64_stamped.hpp>
#include <autoware_internal_planning_msgs/msg/scenario.hpp>
#include <autoware_planning_msgs/msg/lanelet_route.hpp>
#include <autoware_planning_msgs/msg/trajectory.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <std_msgs/msg/bool.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

#ifdef ROS_DISTRO_GALACTIC
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#else
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#endif

#include <autoware/route_handler/route_handler.hpp>

#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

#include <deque>
#include <memory>
#include <string>
#include <vector>

class TestFreespacePlanner;

namespace autoware::freespace_planner
{
using autoware::freespace_planning_algorithms::AbstractPlanningAlgorithm;
using autoware::freespace_planning_algorithms::AstarParam;
using autoware::freespace_planning_algorithms::AstarSearch;
using autoware::freespace_planning_algorithms::PlannerCommonParam;
using autoware::freespace_planning_algorithms::RRTStar;
using autoware::freespace_planning_algorithms::RRTStarParam;
using autoware::freespace_planning_algorithms::VehicleShape;
using autoware_internal_planning_msgs::msg::Scenario;
using autoware_planning_msgs::msg::LaneletRoute;
using autoware_planning_msgs::msg::Trajectory;
using geometry_msgs::msg::PoseArray;
using geometry_msgs::msg::PoseStamped;
using geometry_msgs::msg::TransformStamped;
using geometry_msgs::msg::Twist;
using nav_msgs::msg::OccupancyGrid;
using nav_msgs::msg::Odometry;

struct NodeParam
{
  std::string planning_algorithm;
  double waypoints_velocity;  // constant velocity on planned waypoints [km/h]
  double update_rate;         // replanning and publishing rate [Hz]
  double th_arrived_distance_m;
  double th_stopped_time_sec;
  double th_stopped_velocity_mps;
  double th_course_out_distance_m;  // collision margin [m]
  double th_obstacle_time_sec;
  double vehicle_shape_margin_m;
  bool replan_when_obstacle_found;
  bool replan_when_course_out;
  double parking_accuracy_tolerance;
  int max_replan_count;
  // Statistics tracking parameters
  bool stats_enabled;
  bool stats_report_on_session_end;
  bool stats_periodic_enabled;
  int stats_periodic_cycles;
};

struct ParkingSessionStatsContext
{
  bool is_active = false;
  rclcpp::Time start_time;
  uint64_t attempt_count = 0;
  uint64_t last_periodic_report_count = 0;
  PlanningStatsCollector success_stats;
  PlanningStatsCollector failure_stats;

  void resetStats() noexcept
  {
    attempt_count = 0;
    last_periodic_report_count = 0;
    success_stats.reset();
    failure_stats.reset();
  }
};

struct OverallPlanningStatsContext
{
  rclcpp::Time start_time;
  uint64_t attempt_count = 0;
  PlanningStatsCollector success_stats;
  PlanningStatsCollector failure_stats;
};

class FreespaceStatusUpdater
{
public:
  explicit FreespaceStatusUpdater(rclcpp::Node & node) : updater_(&node)
  {
    updater_.setHardwareID("freespace_planner");
    updater_.add("freespace_planner_status", this, &FreespaceStatusUpdater::check);
  }

  void setScenarioAvailable(bool available) noexcept { has_scenario_ = available; }
  void setActive(bool active) noexcept { is_active_ = active; }
  void onPlanResult(bool success) noexcept { last_plan_failed_ = !success; }
  void forceUpdate() { updater_.force_update(); }

private:
  diagnostic_updater::Updater updater_;
  bool has_scenario_{false};
  bool is_active_{false};
  bool last_plan_failed_{false};

  void check(diagnostic_updater::DiagnosticStatusWrapper & stat)
  {
    if (!has_scenario_) {
      stat.summary(diagnostic_msgs::msg::DiagnosticStatus::OK, "Waiting for scenario");
      return;
    }

    if (is_active_) {
      if (last_plan_failed_) {
        stat.summary(
          diagnostic_msgs::msg::DiagnosticStatus::ERROR, "Freespace Planner failed to find a path");
      } else {
        stat.summary(diagnostic_msgs::msg::DiagnosticStatus::OK, "Freespace Planner is active");
      }
    } else {
      stat.summary(diagnostic_msgs::msg::DiagnosticStatus::OK, "Freespace Planner is inactive");
    }
  }
};

class FreespacePlannerNode : public rclcpp::Node
{
public:
  explicit FreespacePlannerNode(const rclcpp::NodeOptions & node_options);
  ~FreespacePlannerNode() noexcept override;

private:
  // ros
  rclcpp::Publisher<Trajectory>::SharedPtr trajectory_pub_;
  rclcpp::Publisher<PoseArray>::SharedPtr debug_pose_array_pub_;
  rclcpp::Publisher<PoseArray>::SharedPtr debug_partial_pose_array_pub_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr parking_state_pub_;
  rclcpp::Publisher<autoware_internal_debug_msgs::msg::Float64Stamped>::SharedPtr
    processing_time_pub_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr debug_marker_pub_;

  autoware_utils::InterProcessPollingSubscriber<
    LaneletRoute, autoware_utils::polling_policy::Newest>
    route_sub_{this, "~/input/route", rclcpp::QoS{1}.transient_local()};
  autoware_utils::InterProcessPollingSubscriber<OccupancyGrid> occupancy_grid_sub_{
    this, "~/input/occupancy_grid"};
  autoware_utils::InterProcessPollingSubscriber<Scenario> scenario_sub_{this, "~/input/scenario"};
  autoware_utils::InterProcessPollingSubscriber<Odometry, autoware_utils::polling_policy::All>
    odom_sub_{this, "~/input/odometry", rclcpp::QoS{100}};

  rclcpp::TimerBase::SharedPtr timer_;

  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

  // params
  NodeParam node_param_;
  VehicleShape vehicle_shape_;
  VehicleShape collision_vehicle_shape_;

  // variables
  std::unique_ptr<AbstractPlanningAlgorithm> algo_;
  PoseStamped current_pose_;
  PoseStamped goal_pose_;

  Trajectory trajectory_;
  Trajectory partial_trajectory_;
  std::vector<size_t> reversing_indices_;
  size_t prev_target_index_;
  size_t target_index_;
  int replan_count_;
  bool is_completed_ = false;
  bool reset_in_progress_ = false;
  bool is_new_parking_cycle_ = true;
  std::optional<rclcpp::Time> obs_found_time_;
  std::optional<geometry_msgs::msg::Pose> obstacle_pose_;

  LaneletRoute::ConstSharedPtr route_;
  OccupancyGrid::ConstSharedPtr occupancy_grid_;
  Scenario::ConstSharedPtr scenario_;
  Odometry::ConstSharedPtr odom_;
  std::shared_ptr<autoware::route_handler::RouteHandler> route_handler_;

  std::deque<Odometry::ConstSharedPtr> odom_buffer_;

  // diag
  FreespaceStatusUpdater diag_status_;

  // virtual wall
  std::unique_ptr<StopVirtualWallManager> wall_manager_;

  // statistics tracking for parking planning session
  ParkingSessionStatsContext session_stats_;
  OverallPlanningStatsContext overall_stats_;

  // Helper function to report session statistics
  void reportSessionStats(const std::string & session_type);
  void reportOverallStats(const std::string & trigger_type);
  void updateOverallPlanningStats(
    const bool result, const PlanningStatsCollector::MillisecondsF & duration_ms);
  void updateSessionPlanningStats(
    const bool result, const PlanningStatsCollector::MillisecondsF & duration_ms);

  // functions used in the constructor
  PlannerCommonParam getPlannerCommonParam();

  // functions, callback
  void onRoute(const LaneletRoute::ConstSharedPtr msg);
  void onOdometry(const Odometry::ConstSharedPtr msg);

  void onTimer();
  void handleSessionTransition(const bool is_active_now);
  void updatePlanningStats(
    const bool result, const PlanningStatsCollector::MillisecondsF & duration_ms);
  void updateData();
  void reset();
  void planTrajectory();
  void initializePlanningAlgorithm();
  bool isDataReady();

  /**
   * @brief Checks if a new trajectory planning is required.
   * @details A new trajectory planning is required if:
   *           - Current trajectory points are empty, or
   *           - Current trajectory collides with an object, or
   *           - Ego deviates from current trajectory
   * @return true if any of the conditions are met.
   */
  bool isPlanRequired();

  /**
   * @brief Sets the target index along the current trajectory points
   * @details if Ego is stopped AND is near the current target index along the trajectory,
   *          then will get the next target index along the trajectory.
   *          If the new target index is the same as the current target index, then
   *          is_complete_ is set to true, and will publish is_completed_msg.
   *          Otherwise will update prev_target_index_ and target_index_, to continue
   *          following the trajectory.
   */
  void updateTargetIndex();

  /**
   * @brief Checks if current trajectory is colliding with an object.
   * @details Will check if an obstacle exists along the current trajectory,
   *          if there is no obstacle along the current trajectory, will reset obs_found_time_.
   *          If an obstacle exists and the variable obs_found_time_ is not initialized,
   *          will initialize with the current time.
   * @return true if there is an obstacle along current trajectory, AND duration since
   *         obs_found_time_ exceeds the parameter th_obstacle_time_sec
   */
  bool checkCurrentTrajectoryCollision();

  void publishCollisionFootprintMarker(
    const geometry_msgs::msg::Pose & pose_local, const std::string & label);
  void publishCriticalPlannerObstacleMarker(
    const geometry_msgs::msg::Pose & pose_global, const std::string & short_label, bool is_failure);
  void clearCriticalPlannerObstacleMarker();

  TransformStamped getTransform(const std::string & from, const std::string & to);

  std::unique_ptr<autoware_utils::LoggerLevelConfigure> logger_configure_;

  friend class ::TestFreespacePlanner;
};
}  // namespace autoware::freespace_planner

#endif  // AUTOWARE__FREESPACE_PLANNER__FREESPACE_PLANNER_NODE_HPP_
