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

#include "autoware/freespace_planner/freespace_planner_node.hpp"
#include "autoware/freespace_planner/utils.hpp"

#include <ament_index_cpp/get_package_share_directory.hpp>
#include <autoware_test_utils/autoware_test_utils.hpp>
#include <rclcpp/rclcpp.hpp>

#include <autoware_planning_msgs/msg/trajectory.hpp>
#include <diagnostic_msgs/msg/diagnostic_array.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>

#include <geometry_msgs/msg/pose.h>
#include <gtest/gtest.h>

#include <atomic>
#include <chrono>
#include <limits>
#include <memory>
#include <optional>
#include <string>
#include <thread>
#include <utility>
#include <vector>

using autoware::freespace_planner::FreespacePlannerNode;
using autoware_planning_msgs::msg::Trajectory;
using autoware_planning_msgs::msg::TrajectoryPoint;
using geometry_msgs::msg::Pose;
using nav_msgs::msg::OccupancyGrid;
using nav_msgs::msg::Odometry;

namespace
{
struct FakePlanningAlgorithmState
{
  std::atomic<bool> block{false};
  std::atomic<bool> release{false};
  std::atomic<bool> started{false};
  std::atomic<bool> last_reparking{false};
  std::atomic<int> make_plan_calls{0};
};

class FakePlanningAlgorithm
: public autoware::freespace_planning_algorithms::AbstractPlanningAlgorithm
{
public:
  FakePlanningAlgorithm(
    const autoware::freespace_planning_algorithms::PlannerCommonParam & planner_common_param,
    const autoware::freespace_planning_algorithms::VehicleShape & vehicle_shape,
    const rclcpp::Clock::SharedPtr & clock, std::shared_ptr<FakePlanningAlgorithmState> state)
  : AbstractPlanningAlgorithm(planner_common_param, clock, vehicle_shape), state_(std::move(state))
  {
  }

  bool makePlan(const Pose & start_pose, const Pose & goal_pose) override
  {
    state_->started = true;
    ++state_->make_plan_calls;
    while (state_->block && !state_->release) {
      std::this_thread::sleep_for(std::chrono::milliseconds(1));
    }

    waypoints_.header.stamp = clock_->now();
    waypoints_.header.frame_id = costmap_.header.frame_id;
    waypoints_.waypoints.clear();

    autoware::freespace_planning_algorithms::PlannerWaypoint start_waypoint;
    start_waypoint.pose.header = waypoints_.header;
    start_waypoint.pose.pose = start_pose;
    start_waypoint.is_back = false;
    waypoints_.waypoints.push_back(start_waypoint);

    autoware::freespace_planning_algorithms::PlannerWaypoint goal_waypoint;
    goal_waypoint.pose.header = waypoints_.header;
    goal_waypoint.pose.pose = goal_pose;
    goal_waypoint.is_back = false;
    waypoints_.waypoints.push_back(goal_waypoint);

    return true;
  }

  bool makePlan(const Pose & start_pose, const std::vector<Pose> & goal_candidates) override
  {
    if (goal_candidates.empty()) {
      return false;
    }
    return makePlan(start_pose, goal_candidates.front());
  }

  void setReparking(const bool is_reparking) override { state_->last_reparking = is_reparking; }

private:
  std::shared_ptr<FakePlanningAlgorithmState> state_;
};
}  // namespace

class TestFreespacePlanner : public ::testing::Test
{
public:
  void SetUp() override
  {
    rclcpp::init(0, nullptr);
    set_up_node();
    set_up_costmap();
    set_up_trajectory();
  }

  void set_up_node()
  {
    auto node_options = rclcpp::NodeOptions{};
    const auto autoware_test_utils_dir =
      ament_index_cpp::get_package_share_directory("autoware_test_utils");
    const auto freespace_planner_dir =
      ament_index_cpp::get_package_share_directory("autoware_freespace_planner");
    node_options.arguments(
      {"--ros-args", "--params-file",
       autoware_test_utils_dir + "/config/test_vehicle_info.param.yaml", "--params-file",
       freespace_planner_dir + "/config/freespace_planner.param.yaml"});
    freespace_planner_ = std::make_shared<FreespacePlannerNode>(node_options);
  }

  void set_up_costmap()
  {
    const size_t width = 80;
    const size_t height = 80;
    const double resolution = 0.5;
    costmap_ = autoware::test_utils::makeCostMapMsg(width, height, resolution);
  }

  void set_up_trajectory()
  {
    trajectory_.points.clear();

    const double y = 0.5 * costmap_.info.height * costmap_.info.resolution;

    // forward trajectory for 20.0 m
    double x = 5.0;
    for (; x < 25.0 + std::numeric_limits<double>::epsilon(); x += costmap_.info.resolution) {
      TrajectoryPoint point;
      point.pose.position.x = x;
      point.pose.position.y = y;
      point.longitudinal_velocity_mps = 1.0;
      trajectory_.points.push_back(point);
    }

    // backward trajectory for 10.0 m
    x = trajectory_.points.back().pose.position.x - costmap_.info.resolution;
    for (; x > 15.0 - std::numeric_limits<double>::epsilon(); x -= costmap_.info.resolution) {
      TrajectoryPoint point;
      point.pose.position.x = x;
      point.pose.position.y = y;
      point.longitudinal_velocity_mps = -1.0;
      trajectory_.points.push_back(point);
    }

    // forward trajectory for 20.0 m
    x = trajectory_.points.back().pose.position.x + costmap_.info.resolution;
    for (; x < 35.0 + std::numeric_limits<double>::epsilon(); x += costmap_.info.resolution) {
      TrajectoryPoint point;
      point.pose.position.x = x;
      point.pose.position.y = y;
      point.longitudinal_velocity_mps = 1.0;
      trajectory_.points.push_back(point);
    }

    reversing_indices = autoware::freespace_planner::utils::get_reversing_indices(trajectory_);
  }

  [[nodiscard]] OccupancyGrid get_blocked_costmap() const
  {
    auto costmap = costmap_;
    const auto block_y = 0.5 * costmap.info.height * costmap.info.resolution;
    const auto block_x = 20.0;
    const int index_x = std::round(block_x / costmap.info.resolution);
    const int index_y = std::round(block_y / costmap.info.resolution);
    const auto id = index_y * costmap.info.width + index_x;
    costmap.data.at(id) = 100;
    return costmap;
  }

  bool test_is_plan_required(
    const bool empty_traj = false, const bool colliding = false, const bool out_of_course = false)
  {
    freespace_planner_->trajectory_ = Trajectory();
    freespace_planner_->partial_trajectory_ = Trajectory();
    freespace_planner_->reversing_indices_.clear();
    freespace_planner_->obs_found_time_ = {};

    freespace_planner_->occupancy_grid_ = std::make_shared<OccupancyGrid>(costmap_);

    if (empty_traj) {
      return freespace_planner_->isPlanRequired();
    }

    freespace_planner_->trajectory_ = trajectory_;
    freespace_planner_->reversing_indices_ = reversing_indices;
    freespace_planner_->partial_trajectory_ =
      autoware::freespace_planner::utils::get_partial_trajectory(
        trajectory_, 0, reversing_indices.front(),
        std::make_shared<rclcpp::Clock>(RCL_SYSTEM_TIME));
    freespace_planner_->current_pose_.pose = trajectory_.points.front().pose;

    if (colliding) {
      freespace_planner_->obs_found_time_ = freespace_planner_->get_clock()->now();
      freespace_planner_->occupancy_grid_ = std::make_shared<OccupancyGrid>(get_blocked_costmap());
      std::this_thread::sleep_for(std::chrono::seconds(1));
    }

    if (out_of_course) {
      freespace_planner_->current_pose_.pose.position.y +=
        (freespace_planner_->node_param_.th_course_out_distance_m + 0.1);
    }

    return freespace_planner_->isPlanRequired();
  }

  std::pair<size_t, size_t> test_update_target_index(
    const bool stopped = false, const bool near_target = false)
  {
    freespace_planner_->trajectory_ = trajectory_;
    freespace_planner_->reversing_indices_ = reversing_indices;
    freespace_planner_->prev_target_index_ = 0;
    freespace_planner_->target_index_ = autoware::freespace_planner::utils::get_next_target_index(
      trajectory_.points.size(), reversing_indices, freespace_planner_->prev_target_index_);
    freespace_planner_->partial_trajectory_ =
      autoware::freespace_planner::utils::get_partial_trajectory(
        trajectory_, freespace_planner_->prev_target_index_, freespace_planner_->target_index_,
        std::make_shared<rclcpp::Clock>(RCL_SYSTEM_TIME));

    Odometry odom;
    odom.pose.pose = trajectory_.points.front().pose;
    odom.twist.twist.linear.x = 1.0;

    if (stopped) odom.twist.twist.linear.x = 0.0;
    if (near_target) odom.pose.pose = trajectory_.points.at(freespace_planner_->target_index_).pose;

    freespace_planner_->odom_buffer_.clear();
    freespace_planner_->odom_ = std::make_shared<Odometry>(odom);
    freespace_planner_->odom_buffer_.push_back(freespace_planner_->odom_);
    freespace_planner_->current_pose_.pose = odom.pose.pose;

    freespace_planner_->updateTargetIndex();

    return {freespace_planner_->prev_target_index_, freespace_planner_->target_index_};
  }

  std::optional<diagnostic_msgs::msg::DiagnosticStatus> force_diagnostic_update(
    const bool scenario_available, const bool is_active)
  {
    auto test_node = std::make_shared<rclcpp::Node>("freespace_planner_diagnostic_test_node");
    std::optional<diagnostic_msgs::msg::DiagnosticStatus> received_status;

    const auto subscription = test_node->create_subscription<diagnostic_msgs::msg::DiagnosticArray>(
      "/diagnostics", rclcpp::QoS{10},
      [&received_status](const diagnostic_msgs::msg::DiagnosticArray::SharedPtr msg) {
        for (const auto & status : msg->status) {
          if (status.hardware_id == "freespace_planner") {
            received_status = status;
            break;
          }
        }
      });

    rclcpp::executors::SingleThreadedExecutor executor;
    executor.add_node(test_node->get_node_base_interface());
    executor.add_node(freespace_planner_->get_node_base_interface());

    const auto deadline = std::chrono::steady_clock::now() + std::chrono::seconds(1);
    while (!received_status && std::chrono::steady_clock::now() < deadline) {
      freespace_planner_->diag_status_.setScenarioAvailable(scenario_available);
      freespace_planner_->diag_status_.setActive(is_active);
      freespace_planner_->diag_status_.forceUpdate();
      executor.spin_some();
      std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }

    executor.remove_node(freespace_planner_->get_node_base_interface());
    executor.remove_node(test_node->get_node_base_interface());
    return received_status;
  }

  void set_up_planning_context()
  {
    costmap_.header.frame_id = "map";
    freespace_planner_->occupancy_grid_ = std::make_shared<OccupancyGrid>(costmap_);

    freespace_planner_->current_pose_.header.frame_id = costmap_.header.frame_id;
    freespace_planner_->current_pose_.header.stamp = freespace_planner_->get_clock()->now();
    freespace_planner_->current_pose_.pose = trajectory_.points.front().pose;

    freespace_planner_->goal_pose_.header.frame_id = costmap_.header.frame_id;
    freespace_planner_->goal_pose_.header.stamp = freespace_planner_->get_clock()->now();
    freespace_planner_->goal_pose_.pose = trajectory_.points.back().pose;

    freespace_planner_->planning_generation_ = 0;
  }

  void use_fake_planning_algorithm(const std::shared_ptr<FakePlanningAlgorithmState> & state)
  {
    freespace_planner_->planning_algorithm_factory_ = [this, state]() {
      return std::make_unique<FakePlanningAlgorithm>(
        freespace_planner_->planner_common_param_, freespace_planner_->collision_vehicle_shape_,
        freespace_planner_->get_clock(), state);
    };
  }

  void wait_until_fake_planning_started(const std::shared_ptr<FakePlanningAlgorithmState> & state)
  {
    const auto deadline = std::chrono::steady_clock::now() + std::chrono::seconds(1);
    while (!state->started.load() && std::chrono::steady_clock::now() < deadline) {
      std::this_thread::sleep_for(std::chrono::milliseconds(1));
    }
    ASSERT_TRUE(state->started.load());
  }

  void test_async_planning_can_run_repeatedly_without_parameter_redeclaration()
  {
    set_up_planning_context();

    EXPECT_NO_THROW(freespace_planner_->tryStartPlanning());
    freespace_planner_->joinPlanningThread();
    freespace_planner_->consumePlanningResult();
    EXPECT_FALSE(freespace_planner_->is_planning_.load());
    EXPECT_FALSE(freespace_planner_->planning_result_ready_.load());

    EXPECT_NO_THROW(freespace_planner_->tryStartPlanning());
    freespace_planner_->joinPlanningThread();
    freespace_planner_->consumePlanningResult();
    EXPECT_FALSE(freespace_planner_->is_planning_.load());
    EXPECT_FALSE(freespace_planner_->planning_result_ready_.load());
  }

  void test_async_worker_propagates_reparking_snapshot()
  {
    set_up_planning_context();
    const auto state = std::make_shared<FakePlanningAlgorithmState>();
    state->block = true;
    use_fake_planning_algorithm(state);

    freespace_planner_->is_reparking_ = true;
    freespace_planner_->tryStartPlanning();
    wait_until_fake_planning_started(state);

    EXPECT_TRUE(freespace_planner_->is_planning_.load());
    EXPECT_FALSE(freespace_planner_->planning_result_ready_.load());
    EXPECT_TRUE(state->last_reparking.load());

    state->release = true;
    freespace_planner_->joinPlanningThread();

    EXPECT_FALSE(freespace_planner_->is_planning_.load());
    EXPECT_TRUE(freespace_planner_->planning_result_ready_.load());
    freespace_planner_->consumePlanningResult();
    EXPECT_GE(freespace_planner_->trajectory_.points.size(), 2UL);
    EXPECT_EQ(state->make_plan_calls.load(), 1);
  }

  void test_successful_plan_consumption_clears_reparking_mode()
  {
    set_up_planning_context();
    const auto state = std::make_shared<FakePlanningAlgorithmState>();
    use_fake_planning_algorithm(state);

    freespace_planner_->is_reparking_ = true;
    freespace_planner_->tryStartPlanning();
    freespace_planner_->joinPlanningThread();
    freespace_planner_->consumePlanningResult();

    EXPECT_FALSE(freespace_planner_->is_reparking_);
    EXPECT_GE(freespace_planner_->trajectory_.points.size(), 2UL);
  }

  void test_worker_reports_missing_costmap_as_failed_result()
  {
    set_up_planning_context();
    freespace_planner_->is_planning_ = true;
    freespace_planner_->planning_result_ready_ = false;

    FreespacePlannerNode::PlanningRequest request;
    // occupancy_grid left empty
    request.current_pose = freespace_planner_->current_pose_;
    request.goal_pose = freespace_planner_->goal_pose_;
    request.is_reparking = false;
    request.generation = freespace_planner_->planning_generation_;

    freespace_planner_->runPlanningWorker(std::move(request));

    EXPECT_FALSE(freespace_planner_->is_planning_.load());
    EXPECT_TRUE(freespace_planner_->planning_result_ready_.load());
    auto pending_result = freespace_planner_->pending_result_.synchronize();
    EXPECT_FALSE(pending_result->success);
    EXPECT_NE(pending_result->error_msg.find("occupancy grid"), std::string::npos);
  }

  void test_is_plan_required_with_empty_partial_trajectory_does_not_throw()
  {
    freespace_planner_->trajectory_ = trajectory_;
    freespace_planner_->partial_trajectory_ = Trajectory();
    freespace_planner_->reversing_indices_ = reversing_indices;
    freespace_planner_->current_pose_.pose = trajectory_.points.front().pose;
    freespace_planner_->occupancy_grid_ = std::make_shared<OccupancyGrid>(costmap_);

    EXPECT_NO_THROW({
      const bool required = freespace_planner_->isPlanRequired();
      (void)required;
    });
  }

  void test_stale_generation_result_is_discarded()
  {
    set_up_planning_context();
    const auto state = std::make_shared<FakePlanningAlgorithmState>();
    state->block = true;
    use_fake_planning_algorithm(state);

    freespace_planner_->tryStartPlanning();
    wait_until_fake_planning_started(state);

    // Simulate a route change while the worker is still running.
    ++freespace_planner_->planning_generation_;

    state->release = true;
    freespace_planner_->joinPlanningThread();

    // The worker wrote its result, but the generation is now stale.
    EXPECT_TRUE(freespace_planner_->planning_result_ready_.load());

    freespace_planner_->consumePlanningResult();

    // Result should have been discarded — trajectory must remain empty.
    EXPECT_TRUE(freespace_planner_->trajectory_.points.empty());
  }

  void TearDown() override
  {
    if (freespace_planner_) {
      freespace_planner_->joinPlanningThread();
    }
    freespace_planner_ = nullptr;
    trajectory_ = Trajectory();
    costmap_ = OccupancyGrid();
    rclcpp::shutdown();
  }

  std::shared_ptr<FreespacePlannerNode> freespace_planner_;
  OccupancyGrid costmap_;
  Trajectory trajectory_;
  std::vector<size_t> reversing_indices;
};

TEST_F(TestFreespacePlanner, testIsPlanRequired)
{
  EXPECT_FALSE(test_is_plan_required());
  // test with empty current trajectory
  EXPECT_TRUE(test_is_plan_required(true));
  // test with blocked trajectory
  EXPECT_TRUE(test_is_plan_required(false, true));
  // test with deviation from trajectory
  EXPECT_TRUE(test_is_plan_required(false, false, true));
}

TEST_F(TestFreespacePlanner, testIsPlanRequiredWithEmptyPartialTrajectoryDoesNotThrow)
{
  test_is_plan_required_with_empty_partial_trajectory_does_not_throw();
}

TEST_F(TestFreespacePlanner, testUpdateTargetIndex)
{
  size_t prev_target_index, target_index;
  std::tie(prev_target_index, target_index) = test_update_target_index();
  EXPECT_EQ(prev_target_index, 0ul);
  EXPECT_EQ(target_index, reversing_indices.front());

  std::tie(prev_target_index, target_index) = test_update_target_index(true);
  EXPECT_EQ(prev_target_index, 0ul);
  EXPECT_EQ(target_index, reversing_indices.front());

  std::tie(prev_target_index, target_index) = test_update_target_index(false, true);
  EXPECT_EQ(prev_target_index, 0ul);
  EXPECT_EQ(target_index, reversing_indices.front());

  std::tie(prev_target_index, target_index) = test_update_target_index(true, true);
  EXPECT_EQ(prev_target_index, reversing_indices.front());
  EXPECT_EQ(target_index, *std::next(reversing_indices.begin()));
}

TEST_F(TestFreespacePlanner, testInactiveDiagnosticStatusIsOk)
{
  const auto diagnostic_status = force_diagnostic_update(true, false);

  ASSERT_TRUE(diagnostic_status.has_value());
  EXPECT_EQ(diagnostic_status->level, diagnostic_msgs::msg::DiagnosticStatus::OK);
  EXPECT_EQ(diagnostic_status->message, "Freespace Planner is inactive");
}

TEST_F(TestFreespacePlanner, testMissingScenarioDiagnosticStatusIsOk)
{
  const auto diagnostic_status = force_diagnostic_update(false, false);

  ASSERT_TRUE(diagnostic_status.has_value());
  EXPECT_EQ(diagnostic_status->level, diagnostic_msgs::msg::DiagnosticStatus::OK);
  EXPECT_EQ(diagnostic_status->message, "Waiting for scenario");
}

TEST_F(TestFreespacePlanner, testAsyncPlanningCanRunRepeatedlyWithoutParameterRedeclaration)
{
  test_async_planning_can_run_repeatedly_without_parameter_redeclaration();
}

TEST_F(TestFreespacePlanner, testAsyncWorkerPropagatesReparkingSnapshot)
{
  test_async_worker_propagates_reparking_snapshot();
}

TEST_F(TestFreespacePlanner, testSuccessfulPlanConsumptionClearsReparkingMode)
{
  test_successful_plan_consumption_clears_reparking_mode();
}

TEST_F(TestFreespacePlanner, testWorkerReportsMissingCostmapAsFailedResult)
{
  test_worker_reports_missing_costmap_as_failed_result();
}

TEST_F(TestFreespacePlanner, testStaleGenerationResultIsDiscarded)
{
  test_stale_generation_result_is_discarded();
}
