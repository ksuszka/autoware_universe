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
 * Copyright 2015-2019 Autoware Foundation. All rights reserved.
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

#include "autoware/freespace_planner/freespace_planner_node.hpp"

#include "autoware/freespace_planner/stop_virtual_wall_manager.hpp"
#include "autoware/freespace_planner/utils.hpp"

#include <autoware/freespace_planning_algorithms/abstract_algorithm.hpp>
#include <autoware/freespace_planning_algorithms/pose_fmt.hpp>
#include <autoware/motion_utils/marker/marker_helper.hpp>
#include <autoware/motion_utils/trajectory/trajectory.hpp>
#include <autoware_utils/geometry/geometry.hpp>
#include <autoware_utils/geometry/pose_deviation.hpp>
#include <autoware_utils/system/stop_watch.hpp>
#include <rclcpp/logging.hpp>

#include <algorithm>
#include <array>
#include <chrono>
#include <cinttypes>
#include <cmath>
#include <deque>
#include <memory>
#include <optional>
#include <stdexcept>
#include <string>
#include <system_error>
#include <thread>
#include <utility>
#include <vector>

namespace autoware::freespace_planner
{
namespace
{
// Visualization constants for collision markers
constexpr double COLLISION_MARKER_LIFETIME_SEC = 10.0;
constexpr double COLLISION_TEXT_HEIGHT_OFFSET = 1.0;  // meters above pose
constexpr double COLLISION_TEXT_SCALE = 0.3;          // meters
constexpr double COLLISION_FOOTPRINT_WIDTH = 0.05;    // line width in meters
constexpr double CRITICAL_OBSTACLE_MARKER_SCALE = 0.5;
constexpr double CRITICAL_OBSTACLE_TEXT_SCALE = 0.35;
constexpr size_t OBSTACLE_WALL_MARGIN_INDEX = 2;

/// @brief Create standard marker header (pure function)
std_msgs::msg::Header create_marker_header(const std::string & frame_id, const rclcpp::Time & stamp)
{
  std_msgs::msg::Header header;
  header.frame_id = frame_id;
  header.stamp = stamp;
  return header;
}

/// @brief Create base marker with common properties (pure function)
visualization_msgs::msg::Marker create_base_marker(
  const std_msgs::msg::Header & header, const std::string & ns, int id, uint8_t type)
{
  visualization_msgs::msg::Marker marker;
  marker.header = header;
  marker.ns = ns;
  marker.id = id;
  marker.type = type;
  marker.action = visualization_msgs::msg::Marker::ADD;
  return marker;
}

std::optional<geometry_msgs::msg::Pose> calc_obstacle_wall_pose_on_partial_trajectory(
  const Trajectory & partial_trajectory, const geometry_msgs::msg::Pose & current_pose,
  const geometry_msgs::msg::Pose & obstacle_pose)
{
  if (partial_trajectory.points.empty()) {
    return {};
  }

  const size_t ego_index =
    autoware::motion_utils::findNearestIndex(partial_trajectory.points, current_pose.position);
  const size_t obstacle_index =
    autoware::motion_utils::findNearestIndex(partial_trajectory.points, obstacle_pose.position);
  const size_t clamped_obstacle_index =
    std::min(obstacle_index, partial_trajectory.points.size() - 1);

  const size_t margin_index = (clamped_obstacle_index > OBSTACLE_WALL_MARGIN_INDEX)
                                ? (clamped_obstacle_index - OBSTACLE_WALL_MARGIN_INDEX)
                                : 0U;
  const size_t stop_index = std::max(ego_index, margin_index);

  if (stop_index >= partial_trajectory.points.size()) {
    return {};
  }

  return partial_trajectory.points.at(stop_index).pose;
}
}  // namespace

FreespacePlannerNode::FreespacePlannerNode(const rclcpp::NodeOptions & node_options)
: Node("freespace_planner", node_options), diag_status_{*this}
{
  using std::placeholders::_1;

  // NodeParam
  {
    auto & p = node_param_;
    p.planning_algorithm = declare_parameter<std::string>("planning_algorithm");
    p.waypoints_velocity = declare_parameter<double>("waypoints_velocity");
    p.update_rate = declare_parameter<double>("update_rate");
    p.th_arrived_distance_m = declare_parameter<double>("th_arrived_distance_m");
    p.th_stopped_time_sec = declare_parameter<double>("th_stopped_time_sec");
    p.th_stopped_velocity_mps = declare_parameter<double>("th_stopped_velocity_mps");
    p.th_course_out_distance_m = declare_parameter<double>("th_course_out_distance_m");
    p.th_obstacle_time_sec = declare_parameter<double>("th_obstacle_time_sec");
    p.vehicle_shape_margin_m = declare_parameter<double>("vehicle_shape_margin_m");
    p.replan_when_obstacle_found = declare_parameter<bool>("replan_when_obstacle_found");
    p.replan_when_course_out = declare_parameter<bool>("replan_when_course_out");
    p.parking_accuracy_tolerance = declare_parameter<double>("parking_accuracy_tolerance");
    p.max_replan_count = declare_parameter<int>("max_replan_count");
    p.stats_enabled = declare_parameter<bool>("stats_enabled");
    p.stats_report_on_session_end = declare_parameter<bool>("stats_report_on_session_end");
    p.stats_periodic_enabled = declare_parameter<bool>("stats_periodic_enabled");
    p.stats_periodic_cycles = declare_parameter<int>("stats_periodic_cycles");

    if (p.stats_periodic_cycles < 1) {
      throw std::invalid_argument(
        "stats_periodic_cycles must be >= 1. configured value: " +
        std::to_string(p.stats_periodic_cycles));
    }
  }

  // set vehicle_info
  {
    const auto vehicle_info =
      autoware::vehicle_info_utils::VehicleInfoUtils(*this).getVehicleInfo();
    vehicle_shape_.length = vehicle_info.vehicle_length_m;
    vehicle_shape_.width = vehicle_info.vehicle_width_m;
    vehicle_shape_.base_length = vehicle_info.wheel_base_m;
    vehicle_shape_.max_steering = vehicle_info.max_steer_angle_rad;
    vehicle_shape_.base2back = vehicle_info.rear_overhang_m;
    wall_manager_ =
      std::make_unique<StopVirtualWallManager>(*this, vehicle_info.max_longitudinal_offset_m);
  }

  {
    rclcpp::QoS qos{1};
    qos.transient_local();  // latch
    trajectory_pub_ = create_publisher<Trajectory>("~/output/trajectory", qos);
    debug_pose_array_pub_ = create_publisher<PoseArray>("~/debug/pose_array", qos);
    debug_partial_pose_array_pub_ = create_publisher<PoseArray>("~/debug/partial_pose_array", qos);
    parking_state_pub_ = create_publisher<std_msgs::msg::Bool>("is_completed", qos);
    processing_time_pub_ = create_publisher<autoware_internal_debug_msgs::msg::Float64Stamped>(
      "~/debug/processing_time_ms", 1);
    debug_marker_pub_ =
      create_publisher<visualization_msgs::msg::MarkerArray>("~/debug/astar_search_tree", qos);
  }

  // Planning (after publishers are created)
  initializePlanningAlgorithm();
  replan_count_ = 0;

  // TF
  {
    tf_buffer_ = std::make_shared<tf2_ros::Buffer>(get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
  }

  // Timer
  {
    const auto period_ns = rclcpp::Rate(node_param_.update_rate).period();
    timer_ = rclcpp::create_timer(
      this, get_clock(), period_ns, std::bind(&FreespacePlannerNode::onTimer, this));
  }

  logger_configure_ = std::make_unique<autoware_utils::LoggerLevelConfigure>(this);

  overall_stats_.start_time = get_clock()->now();
}

FreespacePlannerNode::~FreespacePlannerNode() noexcept
{
  try {
    joinPlanningThread();
    reportOverallStats("shutdown");
  } catch (...) {
    // Destructor must not throw.
  }
}

PlannerCommonParam FreespacePlannerNode::getPlannerCommonParam()
{
  PlannerCommonParam p;

  // search configs
  p.time_limit = declare_parameter<double>("time_limit");

  p.theta_size = declare_parameter<int>("theta_size");
  p.angle_goal_range = declare_parameter<double>("angle_goal_range");
  p.curve_weight = declare_parameter<double>("curve_weight");
  p.reverse_weight = declare_parameter<double>("reverse_weight");
  p.direction_change_weight = declare_parameter<double>("direction_change_weight");
  p.lateral_goal_range = declare_parameter<double>("lateral_goal_range");
  p.longitudinal_goal_range = declare_parameter<double>("longitudinal_goal_range");
  p.max_turning_ratio = declare_parameter<double>("max_turning_ratio");
  p.turning_steps = declare_parameter<int>("turning_steps");

  // costmap configs
  p.obstacle_threshold = declare_parameter<int>("obstacle_threshold");

  return p;
}

AstarParam FreespacePlannerNode::getAstarParam()
{
  return AstarParam{
    declare_parameter<std::string>("astar.search_method"),
    declare_parameter<bool>("astar.only_behind_solutions"),
    declare_parameter<bool>("astar.use_back"),
    declare_parameter<bool>("astar.adapt_expansion_distance"),
    declare_parameter<double>("astar.expansion_distance"),
    declare_parameter<double>("astar.near_goal_distance"),
    declare_parameter<double>("astar.distance_heuristic_weight"),
    declare_parameter<double>("astar.smoothness_weight"),
    declare_parameter<double>("astar.obstacle_distance_weight"),
    declare_parameter<double>("astar.goal_lat_distance_weight"),
    declare_parameter<double>("astar.final_segment_threshold"),
    declare_parameter<double>("astar.extra_steering_penalty_factor"),
    declare_parameter<double>("astar.yaw_weight"),
    declare_parameter<double>("astar.distance_to_goal_extension_weight"),
    declare_parameter<double>("astar.reparking_forward_first_weight"),
    declare_parameter<double>("astar.reparking_deviation_penalty"),
    declare_parameter<double>("astar.reparking_alignment_weight"),
    declare_parameter<double>("astar.reparking_distance"),
    declare_parameter<bool>("astar.enable_debug_markers")};
}

RRTStarParam FreespacePlannerNode::getRRTStarParam()
{
  return RRTStarParam{
    declare_parameter<bool>("rrtstar.enable_update"),
    declare_parameter<bool>("rrtstar.use_informed_sampling"),
    declare_parameter<double>("rrtstar.max_planning_time"),
    declare_parameter<double>("rrtstar.neighbor_radius"),
    declare_parameter<double>("rrtstar.margin")};
}

bool FreespacePlannerNode::isPlanRequired()
{
  if (trajectory_.points.empty()) {
    clearCriticalPlannerObstacleMarker();
    return true;
  }

  if (node_param_.replan_when_obstacle_found && checkCurrentTrajectoryCollision()) {
    RCLCPP_INFO(get_logger(), "New obstacle found on trajectory. Initiating replanning.");
    const auto wall_pose = calc_obstacle_wall_pose_on_partial_trajectory(
      partial_trajectory_, current_pose_.pose, *obstacle_pose_);
    const auto critical_pose = wall_pose ? *wall_pose : *obstacle_pose_;
    wall_manager_->onObstacle(critical_pose);
    publishCriticalPlannerObstacleMarker(critical_pose, "REPLAN", false);
    return true;
  }

  wall_manager_->onObstacleCleared();
  clearCriticalPlannerObstacleMarker();

  if (node_param_.replan_when_course_out) {
    const bool is_course_out = utils::calc_distance_2d(trajectory_, current_pose_.pose) >
                               node_param_.th_course_out_distance_m;
    if (is_course_out) {
      RCLCPP_INFO(get_logger(), "Course out");
      return true;
    }
  }

  return false;
}

bool FreespacePlannerNode::checkCurrentTrajectoryCollision()
{
  if (!occupancy_grid_ || partial_trajectory_.points.empty()) {
    obs_found_time_ = {};
    obstacle_pose_ = {};
    return false;
  }

  algo_->setMap(*occupancy_grid_);

  const size_t nearest_index_partial = autoware::motion_utils::findNearestIndex(
    partial_trajectory_.points, current_pose_.pose.position);
  const size_t end_index_partial = partial_trajectory_.points.size() - 1;
  const auto forward_trajectory = utils::get_partial_trajectory(
    partial_trajectory_, nearest_index_partial, end_index_partial, get_clock());

  const auto collision_pose =
    algo_->getFirstCollisionPose(utils::trajectory_to_pose_array(forward_trajectory));

  if (!collision_pose) {
    obs_found_time_ = {};
    obstacle_pose_ = {};
    return false;
  }

  obstacle_pose_ = *collision_pose;

  if (!obs_found_time_) obs_found_time_ = get_clock()->now();

  return (get_clock()->now() - *obs_found_time_).seconds() > node_param_.th_obstacle_time_sec;
}

void FreespacePlannerNode::updateTargetIndex()
{
  if (!utils::is_stopped(odom_buffer_, node_param_.th_stopped_velocity_mps)) {
    return;
  }

  const auto is_near_target = utils::is_near_target(
    trajectory_.points.at(target_index_).pose, current_pose_.pose,
    node_param_.th_arrived_distance_m);

  if (!is_near_target) return;

  const auto new_target_index =
    utils::get_next_target_index(trajectory_.points.size(), reversing_indices_, target_index_);

  if (new_target_index == target_index_) {
    double yaw_error =
      autoware_utils_geometry::calc_yaw_deviation(goal_pose_.pose, current_pose_.pose);

    RCLCPP_INFO_STREAM(
      get_logger(), " Angle difference (goal pose vs current pose): " << yaw_error << " degrees");
    RCLCPP_INFO_STREAM(
      get_logger(), " Final deviation from goal - X: "
                      << current_pose_.pose.position.x - goal_pose_.pose.position.x
                      << " Y: " << current_pose_.pose.position.y - goal_pose_.pose.position.y);
    // activate reparking function
    if (std::fabs(yaw_error) >= node_param_.parking_accuracy_tolerance) {
      if (replan_count_ < node_param_.max_replan_count) {
        replan_count_++;
        is_reparking_ = true;
        algo_->setReparking(true);
        reset();
        RCLCPP_INFO(
          get_logger(), "Reparking enabled (replan count: %d) due to yaw error: %f", replan_count_,
          yaw_error);
        return;
      } else {
        is_completed_ = true;
        RCLCPP_INFO_THROTTLE(
          get_logger(), *get_clock(), 1000, " Reparking has reached the limit counts.");

        std_msgs::msg::Bool is_completed_msg;
        is_completed_msg.data = is_completed_;
        parking_state_pub_->publish(is_completed_msg);
        return;
      }
    } else {
      replan_count_ = 0;
      is_completed_ = true;
      RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 1000, " Freespace planning completed.");

      std_msgs::msg::Bool is_completed_msg;
      is_completed_msg.data = is_completed_;
      parking_state_pub_->publish(is_completed_msg);
    }
  } else {
    // Switch to next partial trajectory
    prev_target_index_ = target_index_;
    target_index_ = new_target_index;
  }
}

void FreespacePlannerNode::onRoute(const LaneletRoute::ConstSharedPtr msg)
{
  route_ = msg;

  if (msg->segments.empty()) {
    has_valid_route_ = false;
    is_reparking_ = false;
    algo_->setReparking(false);
    is_new_parking_cycle_ = true;
    replan_count_ = 0;
    ++planning_generation_;
    reset_in_progress_ = false;
    wall_manager_->onInactive();
    reset();

    RCLCPP_INFO(get_logger(), "Route cleared. Canceling freespace planning.");
    return;
  }

  has_valid_route_ = true;

  goal_pose_.header = msg->header;
  goal_pose_.pose = msg->goal_pose;

  is_reparking_ = false;
  algo_->setReparking(false);
  is_new_parking_cycle_ = true;
  replan_count_ = 0;
  ++planning_generation_;
  diag_status_.onPlanResult(true);
  wall_manager_->onPlanSuccess();
  reset();

  RCLCPP_INFO(
    get_logger(), "%s", fmt::format("New route received. Goal pose - {}", goal_pose_.pose).c_str());
}

void FreespacePlannerNode::onOdometry(const Odometry::ConstSharedPtr msg)
{
  odom_ = msg;

  odom_buffer_.push_back(msg);

  // Delete old data in buffer
  while (true) {
    const auto time_diff =
      rclcpp::Time(msg->header.stamp) - rclcpp::Time(odom_buffer_.front()->header.stamp);

    if (time_diff.seconds() < node_param_.th_stopped_time_sec) {
      break;
    }

    odom_buffer_.pop_front();
  }
}

void FreespacePlannerNode::updateData()
{
  {
    auto route_msg = route_sub_.take_data();
    if (route_msg) {
      onRoute(route_msg);
    }
  }

  occupancy_grid_ = occupancy_grid_sub_.take_data();

  {
    auto msgs = odom_sub_.take_data();
    for (const auto & msg : msgs) {
      onOdometry(msg);
    }
  }
}

bool FreespacePlannerNode::isDataReady()
{
  bool is_ready = true;

  if (!has_valid_route_) {
    RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 5000, "Waiting for route data.");
    is_ready = false;
  }

  if (!occupancy_grid_) {
    RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 5000, "Waiting for occupancy grid.");
    is_ready = false;
  }

  if (!odom_) {
    RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 5000, "Waiting for odometry.");
    is_ready = false;
  }

  return is_ready;
}

void FreespacePlannerNode::handleSessionTransition(const bool is_active_now)
{
  const bool was_active = session_stats_.is_active;
  const bool session_transition_to_inactive = was_active && !is_active_now;
  const bool session_transition_to_active = !was_active && is_active_now;

  if (session_transition_to_inactive && node_param_.stats_enabled) {
    if (node_param_.stats_report_on_session_end) {
      reportSessionStats("end");
    } else {
      reportOverallStats("end");
    }
    session_stats_.resetStats();
  }

  if (session_transition_to_active && node_param_.stats_enabled) {
    session_stats_.start_time = get_clock()->now();
    session_stats_.resetStats();
  }

  session_stats_.is_active = is_active_now;
}

void FreespacePlannerNode::reportSessionStats(const std::string & session_type)
{
  if (!node_param_.stats_enabled) {
    return;
  }

  const auto success_summary = session_stats_.success_stats.formatSummary();
  const auto failure_summary = session_stats_.failure_stats.formatSummary();
  const double session_duration_sec = (get_clock()->now() - session_stats_.start_time).seconds();

  RCLCPP_INFO_STREAM(
    get_logger(), "Freespace parking session ["
                    << session_type << "]"
                    << " duration=" << session_duration_sec << "s"
                    << " - Success: " << success_summary << " | Failure: " << failure_summary
                    << " | Total attempts: " << session_stats_.attempt_count);

  reportOverallStats(session_type);
}

void FreespacePlannerNode::reportOverallStats(const std::string & trigger_type)
{
  if (!node_param_.stats_enabled) {
    return;
  }

  const auto success_summary = overall_stats_.success_stats.formatSummary();
  const auto failure_summary = overall_stats_.failure_stats.formatSummary();
  const double uptime_sec = (get_clock()->now() - overall_stats_.start_time).seconds();

  RCLCPP_INFO_STREAM(
    get_logger(), "Freespace parking overall stats ["
                    << trigger_type << "]"
                    << " uptime=" << uptime_sec << "s"
                    << " - Success: " << success_summary << " | Failure: " << failure_summary
                    << " | Total attempts: " << overall_stats_.attempt_count);
}

void FreespacePlannerNode::updatePlanningStats(
  const bool result, const PlanningStatsCollector::MillisecondsF & duration_ms)
{
  if (!node_param_.stats_enabled) {
    return;
  }

  updateOverallPlanningStats(result, duration_ms);
  updateSessionPlanningStats(result, duration_ms);
}

void FreespacePlannerNode::updateOverallPlanningStats(
  const bool result, const PlanningStatsCollector::MillisecondsF & duration_ms)
{
  ++overall_stats_.attempt_count;
  if (result) {
    overall_stats_.success_stats.recordSample(duration_ms);
  } else {
    overall_stats_.failure_stats.recordSample(duration_ms);
  }
}

void FreespacePlannerNode::updateSessionPlanningStats(
  const bool result, const PlanningStatsCollector::MillisecondsF & duration_ms)
{
  if (!session_stats_.is_active) {
    return;
  }

  ++session_stats_.attempt_count;

  if (result) {
    session_stats_.success_stats.recordSample(duration_ms);
  } else {
    session_stats_.failure_stats.recordSample(duration_ms);
  }

  const uint64_t periodic_cycles = static_cast<uint64_t>(node_param_.stats_periodic_cycles);
  if (
    node_param_.stats_periodic_enabled &&
    (session_stats_.attempt_count - session_stats_.last_periodic_report_count) >= periodic_cycles) {
    reportSessionStats("periodic");
    session_stats_.last_periodic_report_count = session_stats_.attempt_count;
  }
}

void FreespacePlannerNode::onTimer()
{
  autoware_utils::StopWatch<std::chrono::milliseconds> stop_watch;

  scenario_ = scenario_sub_.take_data();
  diag_status_.setScenarioAvailable(static_cast<bool>(scenario_));
  const bool is_active_now = utils::is_active(scenario_);
  diag_status_.setActive(is_active_now);
  diag_status_.forceUpdate();

  handleSessionTransition(is_active_now);

  if (!is_active_now) {
    ++planning_generation_;
    reset();
    wall_manager_->onInactive();
    return;
  }

  updateData();

  if (!has_valid_route_) {
    if (odom_) {
      current_pose_.pose = odom_->pose.pose;
      current_pose_.header = odom_->header;

      const auto stop_trajectory = partial_trajectory_.points.empty()
                                     ? utils::create_stop_trajectory(current_pose_, get_clock())
                                     : utils::create_stop_trajectory(partial_trajectory_);
      trajectory_pub_->publish(stop_trajectory);
    }
    return;
  }

  if (!isDataReady()) {
    return;
  }

  if (is_completed_) {
    partial_trajectory_.header = odom_->header;
    const auto stop_trajectory = utils::create_stop_trajectory(partial_trajectory_);
    trajectory_pub_->publish(stop_trajectory);
    return;
  }

  // Get current pose
  current_pose_.pose = odom_->pose.pose;
  current_pose_.header = odom_->header;

  if (current_pose_.header.frame_id == "") {
    return;
  }

  // Ingest result from the planning worker if it finished since the last tick.
  consumePlanningResult();
  publishPendingCollisionEvents();

  // Must stop before replanning any new trajectory
  const bool is_reset_required = !reset_in_progress_ && !is_planning_ && isPlanRequired();
  if (is_reset_required) {
    // Stop before planning new trajectory, except in a new parking cycle as the vehicle already
    // stops.
    if (!is_new_parking_cycle_) {
      const auto stop_trajectory = partial_trajectory_.points.empty()
                                     ? utils::create_stop_trajectory(current_pose_, get_clock())
                                     : utils::create_stop_trajectory(partial_trajectory_);
      trajectory_pub_->publish(stop_trajectory);
      debug_pose_array_pub_->publish(utils::trajectory_to_pose_array(stop_trajectory));
      debug_partial_pose_array_pub_->publish(utils::trajectory_to_pose_array(stop_trajectory));
    }

    reset();
    reset_in_progress_ = true;
  }

  if (reset_in_progress_) {
    const auto is_ego_stopped =
      utils::is_stopped(odom_buffer_, node_param_.th_stopped_velocity_mps);
    if (is_ego_stopped) {
      if (!is_planning_) {
        // Launch async planning — returns immediately; onTimer keeps firing while A* runs.
        tryStartPlanning();
        reset_in_progress_ = false;
      }
      // While is_planning_ is true we fall through and publish a stop trajectory below.
    } else {
      // Will keep current stop trajectory until the vehicle has stopped.
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 1000,
        "Waiting for the vehicle to stop before generating a new trajectory.");
    }
  }

  // Keep the virtual wall visible by re-publishing on every tick
  wall_manager_->republishIfActive();

  // While planning is in progress (or no valid trajectory exists yet) keep publishing a stop
  // trajectory so that the topic_monitor never times out and MRM is not triggered spuriously.
  if (is_planning_ || trajectory_.points.size() <= 1) {
    const auto stop_trajectory = partial_trajectory_.points.empty()
                                   ? utils::create_stop_trajectory(current_pose_, get_clock())
                                   : utils::create_stop_trajectory(partial_trajectory_);
    trajectory_pub_->publish(stop_trajectory);
    is_new_parking_cycle_ = false;

    // Publish ProcessingTime
    autoware_internal_debug_msgs::msg::Float64Stamped processing_time_msg;
    processing_time_msg.stamp = get_clock()->now();
    processing_time_msg.data = stop_watch.toc();
    processing_time_pub_->publish(processing_time_msg);
    return;
  }

  // Update partial trajectory
  updateTargetIndex();
  partial_trajectory_ =
    utils::get_partial_trajectory(trajectory_, prev_target_index_, target_index_, get_clock());

  // Publish messages
  trajectory_pub_->publish(partial_trajectory_);
  debug_pose_array_pub_->publish(utils::trajectory_to_pose_array(trajectory_));
  debug_partial_pose_array_pub_->publish(utils::trajectory_to_pose_array(partial_trajectory_));

  is_new_parking_cycle_ = false;

  // Publish ProcessingTime
  autoware_internal_debug_msgs::msg::Float64Stamped processing_time_msg;
  processing_time_msg.stamp = get_clock()->now();
  processing_time_msg.data = stop_watch.toc();
  processing_time_pub_->publish(processing_time_msg);
}

void FreespacePlannerNode::tryStartPlanning()
{
  if (is_planning_) {
    return;
  }
  if (!occupancy_grid_) {
    return;
  }

  // Join any previously finished thread and consume its result before
  // starting a new one, so that a completed result is never dropped.
  joinPlanningThread();
  consumePlanningResult();

  wall_manager_->startPlanning(current_pose_.pose);

  // Snapshot inputs on the timer thread; the worker only reads its own copies.
  PlanningRequest request;
  request.occupancy_grid = *occupancy_grid_;
  request.current_pose = current_pose_;
  request.goal_pose = goal_pose_;
  request.is_reparking = is_reparking_;
  request.generation = planning_generation_;

  is_planning_ = true;
  planning_result_ready_ = false;

  try {
    planning_thread_ =
      std::thread(&FreespacePlannerNode::runPlanningWorker, this, std::move(request));
  } catch (const std::system_error & e) {
    is_planning_ = false;
    planning_result_ready_ = false;
    RCLCPP_ERROR(get_logger(), "Failed to start planning thread: %s", e.what());
  }
}

void FreespacePlannerNode::runPlanningWorker(PlanningRequest request)
{
  const rclcpp::Time start = get_clock()->now();
  std::string error_msg;
  bool result = false;
  Trajectory pending_traj;
  try {
    if (!request.occupancy_grid) {
      throw std::runtime_error("Cannot plan without occupancy grid.");
    }

    auto local_algo = initializePlanningAlgorithmInstance();
    if (auto * astar = dynamic_cast<AstarSearch *>(local_algo.get())) {
      astar->setCollisionObserver([this, generation = request.generation](
                                    const geometry_msgs::msg::Pose & pose_local,
                                    const std::string & label,
                                    const AstarSearch::CollisionStatus status) {
        if (status != AstarSearch::CollisionStatus::Collision) {
          return;
        }
        auto pending_collision_events = pending_collision_events_.synchronize();
        pending_collision_events->push_back(CollisionEvent{pose_local, label, generation});
      });
    }
    local_algo->setReparking(request.is_reparking);
    local_algo->setMap(*request.occupancy_grid);

    // tf2_ros::Buffer serializes access internally; lookupTransform is safe from this worker.
    const auto current_pose_in_costmap_frame = utils::transform_pose(
      request.current_pose.pose,
      getTransform(request.occupancy_grid->header.frame_id, request.current_pose.header.frame_id));

    const auto goal_pose_in_costmap_frame = utils::transform_pose(
      request.goal_pose.pose,
      getTransform(request.occupancy_grid->header.frame_id, request.goal_pose.header.frame_id));

    result = local_algo->makePlan(current_pose_in_costmap_frame, goal_pose_in_costmap_frame);
    if (result) {
      pending_traj = utils::create_trajectory(
        request.current_pose, local_algo->getWaypoints(), node_param_.waypoints_velocity);
    }
  } catch (const std::exception & e) {
    error_msg = e.what();
  } catch (...) {
    error_msg = "Unknown exception in planning worker.";
  }
  const rclcpp::Time end = get_clock()->now();

  const auto duration_sec = std::chrono::duration<double>{(end - start).seconds()};
  const auto duration_ms =
    std::chrono::duration_cast<PlanningStatsCollector::MillisecondsF>(duration_sec);

  {
    auto pending_result = pending_result_.synchronize();
    pending_result->success = result;
    pending_result->trajectory = std::move(pending_traj);
    pending_result->duration_ms = duration_ms;
    pending_result->error_msg = std::move(error_msg);
    pending_result->generation = request.generation;
  }
  planning_result_ready_ = true;
  is_planning_ = false;
}

void FreespacePlannerNode::consumePlanningResult()
{
  if (!planning_result_ready_) {
    return;
  }
  planning_result_ready_ = false;

  auto trajectory = Trajectory(rosidl_runtime_cpp::MessageInitialization::ALL);
  bool result{false};
  PlanningStatsCollector::MillisecondsF duration_ms{};
  std::string error_msg;
  uint64_t result_generation{0};
  {
    auto pending_result = pending_result_.synchronize();
    trajectory = std::move(pending_result->trajectory);
    result = pending_result->success;
    duration_ms = pending_result->duration_ms;
    error_msg = std::move(pending_result->error_msg);
    result_generation = pending_result->generation;
    pending_result->trajectory = Trajectory(rosidl_runtime_cpp::MessageInitialization::ALL);
    pending_result->success = false;
    pending_result->duration_ms = PlanningStatsCollector::MillisecondsF{};
    pending_result->error_msg.clear();
    pending_result->generation = 0;
  }

  // Discard results from a previous route or session.
  if (result_generation != planning_generation_) {
    publishPendingCollisionEvents();
    RCLCPP_DEBUG(
      get_logger(), "Discarding stale planning result (gen %" PRIu64 " vs current %" PRIu64 ").",
      result_generation, planning_generation_);
    return;
  }

  RCLCPP_DEBUG(
    get_logger(), "Freespace planning: %f [s]",
    std::chrono::duration<double>(duration_ms).count());

  updatePlanningStats(result, duration_ms);
  publishPendingCollisionEvents();

  if (result) {
    RCLCPP_DEBUG(get_logger(), "Found goal!");
    diag_status_.onPlanResult(true);
    wall_manager_->onPlanSuccess();
    clearCriticalPlannerObstacleMarker();
    is_reparking_ = false;
    algo_->setReparking(false);
    trajectory_ = std::move(trajectory);

    reversing_indices_ = utils::get_reversing_indices(trajectory_);
    prev_target_index_ = 0;
    target_index_ = utils::get_next_target_index(
      trajectory_.points.size(), reversing_indices_, prev_target_index_);
  } else {
    RCLCPP_WARN_THROTTLE(
      get_logger(), *get_clock(), 30000, "Failed to find path: %s",
      error_msg.c_str());
    diag_status_.onPlanResult(false);
    wall_manager_->onPlanFailed();
    reset();
  }
}

void FreespacePlannerNode::publishPendingCollisionEvents()
{
  std::vector<CollisionEvent> collision_events;
  {
    auto pending_collision_events = pending_collision_events_.synchronize();
    collision_events.swap(*pending_collision_events);
  }

  if (!occupancy_grid_) {
    return;
  }

  for (const auto & event : collision_events) {
    if (event.generation != planning_generation_) {
      continue;
    }

    publishCollisionFootprintMarker(event.pose_local, event.label);
    const auto global_pose =
      autoware::freespace_planning_algorithms::local2global(*occupancy_grid_, event.pose_local);
    publishCriticalPlannerObstacleMarker(
      global_pose, (event.label == "start") ? "START" : "GOAL", true);
    wall_manager_->setCollisionContext(global_pose, event.label);
  }
}

void FreespacePlannerNode::joinPlanningThread()
{
  if (planning_thread_.joinable()) {
    planning_thread_.join();
  }
}

void FreespacePlannerNode::reset()
{
  trajectory_ = Trajectory();
  partial_trajectory_ = Trajectory();
  is_completed_ = false;
  std_msgs::msg::Bool is_completed_msg;
  is_completed_msg.data = is_completed_;
  parking_state_pub_->publish(is_completed_msg);
  obs_found_time_ = {};
  obstacle_pose_ = {};
  clearCriticalPlannerObstacleMarker();
}

void FreespacePlannerNode::publishCollisionFootprintMarker(
  const geometry_msgs::msg::Pose & pose_local, const std::string & label)
{
  if (!debug_marker_pub_ || !occupancy_grid_) {
    return;
  }

  visualization_msgs::msg::MarkerArray marker_array;
  const auto header = create_marker_header(occupancy_grid_->header.frame_id, get_clock()->now());

  auto footprint_marker = create_base_marker(
    header, "collision_footprint", (label == "start") ? 0 : 1,
    visualization_msgs::msg::Marker::LINE_STRIP);
  footprint_marker.scale.x = COLLISION_FOOTPRINT_WIDTH;
  footprint_marker.color.r = 1.0f;
  footprint_marker.color.g = 0.0f;
  footprint_marker.color.b = 0.0f;
  footprint_marker.color.a = 0.9f;
  footprint_marker.lifetime = rclcpp::Duration::from_seconds(COLLISION_MARKER_LIFETIME_SEC);

  const auto footprint_local = autoware::freespace_planning_algorithms::createFootprintPoints(
    pose_local, collision_vehicle_shape_);

  const auto to_global_point = [this](const auto & point) {
    geometry_msgs::msg::Pose corner_pose;
    corner_pose.position = point;
    corner_pose.orientation.w = 1.0;
    return autoware::freespace_planning_algorithms::local2global(*occupancy_grid_, corner_pose)
      .position;
  };

  std::transform(
    footprint_local.begin(), footprint_local.end(), std::back_inserter(footprint_marker.points),
    to_global_point);

  if (!footprint_marker.points.empty()) {
    footprint_marker.points.push_back(footprint_marker.points.front());
  }

  auto text_marker = create_base_marker(
    header, "collision_text", (label == "start") ? 0 : 1,
    visualization_msgs::msg::Marker::TEXT_VIEW_FACING);
  const auto global_pose =
    autoware::freespace_planning_algorithms::local2global(*occupancy_grid_, pose_local);
  text_marker.pose.position = global_pose.position;
  text_marker.pose.position.z += COLLISION_TEXT_HEIGHT_OFFSET;
  text_marker.pose.orientation.w = 1.0;
  text_marker.text = label + " (COLLISION)";
  text_marker.scale.z = COLLISION_TEXT_SCALE;
  text_marker.color.r = 1.0f;
  text_marker.color.g = 1.0f;
  text_marker.color.b = 1.0f;
  text_marker.color.a = 1.0f;
  text_marker.lifetime = rclcpp::Duration::from_seconds(COLLISION_MARKER_LIFETIME_SEC);

  marker_array.markers.push_back(footprint_marker);
  marker_array.markers.push_back(text_marker);
  debug_marker_pub_->publish(marker_array);
}

void FreespacePlannerNode::publishCriticalPlannerObstacleMarker(
  const geometry_msgs::msg::Pose & pose_global, const std::string & short_label,
  const bool is_failure)
{
  if (!debug_marker_pub_ || !occupancy_grid_) {
    return;
  }

  visualization_msgs::msg::MarkerArray marker_array;
  const auto header = create_marker_header(occupancy_grid_->header.frame_id, get_clock()->now());

  auto critical_marker = create_base_marker(
    header, "critical_planner_obstacle", 0, visualization_msgs::msg::Marker::SPHERE);
  critical_marker.pose = pose_global;
  critical_marker.scale.x = CRITICAL_OBSTACLE_MARKER_SCALE;
  critical_marker.scale.y = CRITICAL_OBSTACLE_MARKER_SCALE;
  critical_marker.scale.z = CRITICAL_OBSTACLE_MARKER_SCALE;
  critical_marker.color.r = 1.0f;
  critical_marker.color.g = is_failure ? 0.0f : 0.7f;
  critical_marker.color.b = 0.0f;
  critical_marker.color.a = 0.95f;

  auto text_marker = create_base_marker(
    header, "critical_planner_obstacle", 1, visualization_msgs::msg::Marker::TEXT_VIEW_FACING);
  text_marker.pose = pose_global;
  text_marker.pose.position.z += COLLISION_TEXT_HEIGHT_OFFSET;
  text_marker.scale.z = CRITICAL_OBSTACLE_TEXT_SCALE;
  text_marker.color.r = 1.0f;
  text_marker.color.g = 1.0f;
  text_marker.color.b = 1.0f;
  text_marker.color.a = 1.0f;
  text_marker.text = short_label;

  marker_array.markers.push_back(critical_marker);
  marker_array.markers.push_back(text_marker);
  debug_marker_pub_->publish(marker_array);
}

void FreespacePlannerNode::clearCriticalPlannerObstacleMarker()
{
  if (!debug_marker_pub_ || !occupancy_grid_) {
    return;
  }

  visualization_msgs::msg::MarkerArray marker_array;
  const auto header = create_marker_header(occupancy_grid_->header.frame_id, get_clock()->now());

  auto clear_sphere = create_base_marker(
    header, "critical_planner_obstacle", 0, visualization_msgs::msg::Marker::SPHERE);
  clear_sphere.action = visualization_msgs::msg::Marker::DELETE;

  auto clear_text = create_base_marker(
    header, "critical_planner_obstacle", 1, visualization_msgs::msg::Marker::TEXT_VIEW_FACING);
  clear_text.action = visualization_msgs::msg::Marker::DELETE;

  marker_array.markers.push_back(clear_sphere);
  marker_array.markers.push_back(clear_text);
  debug_marker_pub_->publish(marker_array);
}

TransformStamped FreespacePlannerNode::getTransform(
  const std::string & from, const std::string & to)
{
  TransformStamped tf;
  try {
    tf = tf_buffer_->lookupTransform(from, to, tf2::TimePointZero, tf2::durationFromSec(1.0));
  } catch (const tf2::TransformException & ex) {
    RCLCPP_ERROR(get_logger(), "%s", ex.what());
  }
  return tf;
}

void FreespacePlannerNode::initializePlanningAlgorithm()
{
  collision_vehicle_shape_ = vehicle_shape_;
  const double margin = node_param_.vehicle_shape_margin_m;
  collision_vehicle_shape_.length += margin;
  collision_vehicle_shape_.width += margin;
  collision_vehicle_shape_.base2back += margin / 2;
  collision_vehicle_shape_.setMinMaxDimension();

  planner_common_param_ = getPlannerCommonParam();
  const auto algo_name = node_param_.planning_algorithm;
  if (algo_name == "astar") {
    astar_param_ = getAstarParam();
  } else if (algo_name == "rrtstar") {
    rrtstar_param_ = getRRTStarParam();
  }

  // Initialize the shared algo_ used by checkCurrentTrajectoryCollision().
  algo_ = initializePlanningAlgorithmInstance();

  // Register the collision observer on the shared algo_ only (it captures
  // this node's members so it must only run on the timer thread).
  if (node_param_.planning_algorithm == "astar") {
    auto * astar = dynamic_cast<AstarSearch *>(algo_.get());
    if (astar) {
      astar->setCollisionObserver([this](
                                    const geometry_msgs::msg::Pose & pose_local,
                                    const std::string & label,
                                    const AstarSearch::CollisionStatus status) {
        if (status != AstarSearch::CollisionStatus::Collision || !occupancy_grid_) {
          return;
        }
        publishCollisionFootprintMarker(pose_local, label);
        const auto global_pose =
          autoware::freespace_planning_algorithms::local2global(*occupancy_grid_, pose_local);
        publishCriticalPlannerObstacleMarker(
          global_pose, (label == "start") ? "START" : "GOAL", true);
        wall_manager_->setCollisionContext(global_pose, label);
      });
    }
  }

  RCLCPP_INFO_STREAM(
    get_logger(), "initialize planning algorithm: " << node_param_.planning_algorithm);
}

std::unique_ptr<AbstractPlanningAlgorithm>
FreespacePlannerNode::initializePlanningAlgorithmInstance()
{
  if (planning_algorithm_factory_) {
    return planning_algorithm_factory_();
  }

  const auto algo_name = node_param_.planning_algorithm;

  if (algo_name == "astar") {
    return std::make_unique<AstarSearch>(
      planner_common_param_, collision_vehicle_shape_, astar_param_, get_clock(),
      debug_marker_pub_);
  } else if (algo_name == "rrtstar") {
    return std::make_unique<RRTStar>(
      planner_common_param_, collision_vehicle_shape_, rrtstar_param_, get_clock());
  } else {
    throw std::runtime_error("No such algorithm named " + algo_name + " exists.");
  }
}

}  // namespace autoware::freespace_planner

#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(autoware::freespace_planner::FreespacePlannerNode)
