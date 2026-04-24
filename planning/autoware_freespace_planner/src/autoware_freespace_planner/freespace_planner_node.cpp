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
#include "autoware/freespace_planning_algorithms/abstract_algorithm.hpp"
#include "autoware/freespace_planning_algorithms/pose_fmt.hpp"

#include <autoware/motion_utils/marker/marker_helper.hpp>
#include <autoware/motion_utils/trajectory/trajectory.hpp>
#include <autoware_utils/geometry/geometry.hpp>
#include <autoware_utils/geometry/pose_deviation.hpp>
#include <autoware_utils/system/stop_watch.hpp>
#include <rclcpp/logging.hpp>

#include <algorithm>
#include <array>
#include <deque>
#include <memory>
#include <string>
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

boost::optional<geometry_msgs::msg::Pose> calc_obstacle_wall_pose_on_partial_trajectory(
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

bool FreespacePlannerNode::isPlanRequired()
{
  if (trajectory_.points.empty()) {
    return true;
  }

  if (node_param_.replan_when_obstacle_found && checkCurrentTrajectoryCollision()) {
    RCLCPP_INFO(get_logger(), "New obstacle found on trajectory. Initiating replanning.");
    const auto wall_pose = calc_obstacle_wall_pose_on_partial_trajectory(
      partial_trajectory_, current_pose_.pose, *obstacle_pose_);
    wall_manager_->onObstacle(wall_pose ? *wall_pose : *obstacle_pose_);
    return true;
  }

  wall_manager_->onObstacleCleared();

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

  return (get_clock()->now() - obs_found_time_.get()).seconds() > node_param_.th_obstacle_time_sec;
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

  goal_pose_.header = msg->header;
  goal_pose_.pose = msg->goal_pose;

  algo_->setReparking(false);
  is_new_parking_cycle_ = true;
  replan_count_ = 0;
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

  if (!route_) {
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

void FreespacePlannerNode::onTimer()
{
  autoware_utils::StopWatch<std::chrono::milliseconds> stop_watch;

  scenario_ = scenario_sub_.take_data();
  diag_status_.setScenarioAvailable(static_cast<bool>(scenario_));
  diag_status_.setActive(utils::is_active(scenario_));
  diag_status_.forceUpdate();

  if (!utils::is_active(scenario_)) {
    reset();
    wall_manager_->onInactive();
    return;
  }

  updateData();

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

  // Must stop before replanning any new trajectory
  const bool is_reset_required = !reset_in_progress_ && isPlanRequired();
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
      // Plan new trajectory
      const rclcpp::Time start_time = get_clock()->now();

      // Plan new trajectory
      planTrajectory();

      const rclcpp::Time end_time = get_clock()->now();
      const double duration = (end_time - start_time).seconds();

      RCLCPP_DEBUG(get_logger(), " execution time: %f seconds", duration);

      reset_in_progress_ = false;
    } else {
      // Will keep current stop trajectory
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 1000,
        "Waiting for the vehicle to stop before generating a new trajectory.");
    }
  }

  // Keep the virtual wall visible by re-publishing on every tick
  wall_manager_->republishIfActive();

  // StopTrajectory
  if (trajectory_.points.size() <= 1) {
    is_new_parking_cycle_ = false;
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

void FreespacePlannerNode::planTrajectory()
{
  if (occupancy_grid_ == nullptr) {
    return;
  }

  wall_manager_->startPlanning(current_pose_.pose);

  // Provide robot shape and map for the planner
  algo_->setMap(*occupancy_grid_);

  // Calculate poses in costmap frame
  const auto current_pose_in_costmap_frame = utils::transform_pose(
    current_pose_.pose,
    getTransform(occupancy_grid_->header.frame_id, current_pose_.header.frame_id));

  const auto goal_pose_in_costmap_frame = utils::transform_pose(
    goal_pose_.pose, getTransform(occupancy_grid_->header.frame_id, goal_pose_.header.frame_id));

  // execute planning
  const rclcpp::Time start = get_clock()->now();
  std::string error_msg;
  bool result = false;
  try {
    result = algo_->makePlan(current_pose_in_costmap_frame, goal_pose_in_costmap_frame);
  } catch (const std::exception & e) {
    error_msg = e.what();
  }
  const rclcpp::Time end = get_clock()->now();

  RCLCPP_DEBUG(get_logger(), "Freespace planning: %f [s]", (end - start).seconds());

  if (result) {
    RCLCPP_DEBUG(get_logger(), "Found goal!");
    diag_status_.onPlanResult(true);
    wall_manager_->onPlanSuccess();
    trajectory_ = utils::create_trajectory(
      current_pose_, algo_->getWaypoints(), node_param_.waypoints_velocity);
    reversing_indices_ = utils::get_reversing_indices(trajectory_);
    prev_target_index_ = 0;
    target_index_ = utils::get_next_target_index(
      trajectory_.points.size(), reversing_indices_, prev_target_index_);
  } else {
    RCLCPP_WARN_THROTTLE(
      get_logger(), *get_clock(), 30000, "Failed to find path: %s", error_msg.c_str());
    diag_status_.onPlanResult(false);
    wall_manager_->onPlanFailed();
    reset();
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
  // Extend robot shape
  collision_vehicle_shape_ = vehicle_shape_;
  const double margin = node_param_.vehicle_shape_margin_m;
  collision_vehicle_shape_.length += margin;
  collision_vehicle_shape_.width += margin;
  collision_vehicle_shape_.base2back += margin / 2;
  collision_vehicle_shape_.setMinMaxDimension();

  const auto planner_common_param = getPlannerCommonParam();

  const auto algo_name = node_param_.planning_algorithm;

  // initialize specified algorithm
  if (algo_name == "astar") {
    auto astar_algo = std::make_unique<AstarSearch>(
      planner_common_param, collision_vehicle_shape_, *this, debug_marker_pub_);

    astar_algo->setCollisionObserver([this](
                                       const geometry_msgs::msg::Pose & pose_local,
                                       const std::string & label,
                                       const AstarSearch::CollisionStatus status) {
      if (status != AstarSearch::CollisionStatus::Collision || !occupancy_grid_) {
        return;
      }
      publishCollisionFootprintMarker(pose_local, label);
      const auto global_pose =
        autoware::freespace_planning_algorithms::local2global(*occupancy_grid_, pose_local);
      wall_manager_->setCollisionContext(global_pose, label);
    });
    algo_ = std::move(astar_algo);
  } else if (algo_name == "rrtstar") {
    algo_ = std::make_unique<RRTStar>(planner_common_param, collision_vehicle_shape_, *this);
  } else {
    throw std::runtime_error("No such algorithm named " + algo_name + " exists.");
  }
  RCLCPP_INFO_STREAM(get_logger(), "initialize planning algorithm: " << algo_name);
}

}  // namespace autoware::freespace_planner

#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(autoware::freespace_planner::FreespacePlannerNode)
