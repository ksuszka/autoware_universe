// Copyright 2021 The Autoware Foundation
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

#include "autoware/mpc_lateral_controller/mpc_trajectory.hpp"
#include "autoware/mpc_lateral_controller/mpc_utils.hpp"
#include "gtest/gtest.h"

#include "autoware_planning_msgs/msg/trajectory.hpp"
#include "autoware_planning_msgs/msg/trajectory_point.hpp"

#include <memory>
#include <vector>

namespace autoware::motion::control::mpc_lateral_controller
{
namespace
{
namespace MPCUtils = autoware::motion::control::mpc_lateral_controller::MPCUtils;
using autoware_planning_msgs::msg::Trajectory;
using autoware_planning_msgs::msg::TrajectoryPoint;

TrajectoryPoint makePoint(const double x, const double y, const float vx)
{
  TrajectoryPoint p;
  p.pose.position.x = x;
  p.pose.position.y = y;
  p.longitudinal_velocity_mps = vx;
  return p;
}

/* cppcheck-suppress syntaxError */
TEST(TestMPC, CalcStopDistance)
{
  constexpr float MOVE = 1.0f;
  constexpr float STOP = 0.0f;

  Trajectory trajectory_msg;
  trajectory_msg.points.push_back(makePoint(0.0, 0.0, MOVE));
  trajectory_msg.points.push_back(makePoint(1.0, 0.0, MOVE));
  trajectory_msg.points.push_back(makePoint(2.0, 0.0, STOP));  // STOP
  trajectory_msg.points.push_back(makePoint(3.0, 0.0, MOVE));
  trajectory_msg.points.push_back(makePoint(4.0, 0.0, MOVE));
  trajectory_msg.points.push_back(makePoint(5.0, 0.0, MOVE));
  trajectory_msg.points.push_back(makePoint(6.0, 0.0, STOP));  // STOP
  trajectory_msg.points.push_back(makePoint(7.0, 0.0, STOP));  // STOP

  EXPECT_EQ(MPCUtils::calcStopDistance(trajectory_msg, 0), 2.0);
  EXPECT_EQ(MPCUtils::calcStopDistance(trajectory_msg, 1), 1.0);
  EXPECT_EQ(MPCUtils::calcStopDistance(trajectory_msg, 2), 0.0);
  EXPECT_EQ(MPCUtils::calcStopDistance(trajectory_msg, 3), 3.0);
  EXPECT_EQ(MPCUtils::calcStopDistance(trajectory_msg, 4), 2.0);
  EXPECT_EQ(MPCUtils::calcStopDistance(trajectory_msg, 5), 1.0);
  EXPECT_EQ(MPCUtils::calcStopDistance(trajectory_msg, 6), 0.0);
  EXPECT_EQ(MPCUtils::calcStopDistance(trajectory_msg, 7), -1.0);
}

/* cppcheck-suppress syntaxError */
TEST(TestMPC, resampleMPCTrajectoryByDistance)
{
  auto mpc_trajectory_from_vec = [](const std::vector<double> & x_vec) {
    MPCTrajectory trajectory;
    for (size_t i = 0; i < x_vec.size(); ++i) {
      trajectory.push_back(x_vec.at(i), 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0);
    }
    return trajectory;
  };

  MPCTrajectory input = mpc_trajectory_from_vec({0.0, 1.0, 2.0, 3.0, 4.0, 5.0});
  std::vector<double> expected_x{0.1, 0.6, 1.1, 1.6, 2.1, 2.6, 3.1, 3.6, 4.1, 4.6};
  double resample_interval_dist = 0.5;
  size_t nearest_seg_idx = 2;
  double ego_offset_to_segment = 0.1;

  auto [success, output, base_link_segment_idx] = MPCUtils::resampleMPCTrajectoryByDistance(
    input, resample_interval_dist, nearest_seg_idx, ego_offset_to_segment);

  ASSERT_TRUE(success) << "resampling failed";
  ASSERT_EQ(output.size(), 10) << "output trajectory size mismatch";
  for (size_t i = 0; i < output.size(); ++i) {
    EXPECT_NEAR(output.x.at(i), expected_x.at(i), 1e-6) << "x value mismatch at index " << i;
    EXPECT_NEAR(output.y.at(i), 0.0, 1e-6) << "y value mismatch at index " << i;
  }
  EXPECT_EQ(base_link_segment_idx, 4)
    << "aligned base_link index should be 4 (2.1 is the first point after the nearest segment "
       "index 2 with the offset of 0.1)";
  EXPECT_NEAR(output.x.at(base_link_segment_idx), 2.1, 1e-6)
    << "aligned base_link should be at position (2.1, 0.0) on the trajectory";
}

/* cppcheck-suppress syntaxError */
TEST(TestMPC, resampleMPCTrajectoryByDistance_empty_input)
{
  MPCTrajectory input;
  double resample_interval_dist = 0.5;
  size_t nearest_seg_idx = 0;
  double ego_offset_to_segment = 0.1;

  auto [success, output, base_link_segment_idx] = MPCUtils::resampleMPCTrajectoryByDistance(
    input, resample_interval_dist, nearest_seg_idx, ego_offset_to_segment);

  ASSERT_FALSE(success) << "resampling failed for empty input";
  ASSERT_TRUE(output.empty()) << "output should be empty for empty input";
  EXPECT_EQ(base_link_segment_idx, 0) << "base_link_segment_idx should be 0 for empty input";
}

/* cppcheck-suppress syntaxError */
TEST(TestMPC, resampleMPCTrajectoryByDistance_too_short_after_resampling)
{
  MPCTrajectory input;
  input.push_back(0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0);
  input.push_back(0.1, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0);
  input.push_back(0.2, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0);

  const double resample_interval_dist = 1.0;
  const size_t nearest_seg_idx = 1;
  const double ego_offset_to_segment = 0.0;

  auto [success, output, base_link_segment_idx] = MPCUtils::resampleMPCTrajectoryByDistance(
    input, resample_interval_dist, nearest_seg_idx, ego_offset_to_segment);

  ASSERT_FALSE(success) << "resampling should fail when the output trajectory has fewer than 3 points";
  ASSERT_TRUE(output.empty()) << "output should stay empty when resampling result is too short";
  EXPECT_EQ(base_link_segment_idx, 0);
}

/* cppcheck-suppress syntaxError */
TEST(TestMPC, dynamicSmoothingVelocity)
{
  auto mpc_trajectory_from_vec = [](const std::vector<double> & vx_vec) {
    MPCTrajectory trajectory;
    for (size_t i = 0; i < vx_vec.size(); ++i) {
      trajectory.push_back(static_cast<double>(i), 0.0, 0.0, 0.0, vx_vec.at(i), 0.0, 0.0, 0.0);
    }
    return trajectory;
  };

  MPCTrajectory trajectory = mpc_trajectory_from_vec({0.0, 0.0, 0.0, 5.0, 5.0, 5.0, 5.0, 5.0});
  size_t start_seg_idx = 2;
  double start_vel = 1.0;
  double acc_limit = 1.0;
  double tau = 1.0;

  // expected trajectory velocity:
  // before segment index: do not change (0.0, 0.0)
  // at segment index: set to start_vel (1.0)
  // after segment index: smoothly change with time constant tau and acceleration limit acc_limit
  // (2.0, 2.5, 2.9, 3.24, 3.55)
  std::vector<double> expected_vx{0.0, 0.0, 1.0, 2.0, 2.5, 2.9, 3.24, 3.55};

  MPCUtils::dynamicSmoothingVelocity(start_seg_idx, start_vel, acc_limit, tau, trajectory);
  for (size_t i = 0; i < trajectory.size(); ++i) {
    EXPECT_NEAR(trajectory.vx.at(i), expected_vx.at(i), 1e-2) << "velocity mismatch at index " << i;
  }
}

/* cppcheck-suppress syntaxError */
TEST(TestMPC, calcNearestPoseInterp_false_if_nearest_index_of_out_bounds)
{
  MPCTrajectory trajectory;
  geometry_msgs::msg::Pose current_pose;
  geometry_msgs::msg::Pose nearest_pose;
  double nearest_time;

  size_t nearest_idx = 0;  // out of bounds for empty trajectory

  auto result = MPCUtils::calcNearestPoseInterp(
    trajectory, current_pose, &nearest_pose, &nearest_idx, &nearest_time);
  ASSERT_FALSE(result) << "Should return false for out of bounds nearest index";
}

/* cppcheck-suppress syntaxError */
TEST(TestMPC, calcNearestPoseInterp)
{
  auto mpc_trajectory_from_vec =
    [](
      const std::vector<double> & x_vec, const std::vector<double> & y_vec,
      const std::vector<double> & yaw_vec, const std::vector<double> & vx_vec,
      const std::vector<double> & relative_time_vec) {
      MPCTrajectory trajectory;
      for (size_t i = 0; i < vx_vec.size(); ++i) {
        trajectory.push_back(
          x_vec.at(i), y_vec.at(i), 0.0, yaw_vec.at(i), vx_vec.at(i), 0.0, 0.0,
          relative_time_vec.at(i));
      }
      return trajectory;
    };

  auto create_pose = [](const double x, const double y, const double yaw) {
    geometry_msgs::msg::Pose pose;
    pose.position.x = x;
    pose.position.y = y;
    pose.orientation = autoware_utils::create_quaternion_from_yaw(yaw);
    return pose;
  };

  MPCTrajectory trajectory = mpc_trajectory_from_vec(
    {0.0, 1.0, 2.0, 3.0, 4.0, 5.0},        // x (0..5)
    {10.0, 11.0, 12.0, 13.0, 14.0, 15.0},  // y (10..15)
    {0.0, 0.1, 0.2, 0.3, 0.4, 0.5},        // yaw (0.0..0.5)
    {30.0, 31.0, 32.0, 33.0, 34.0, 35.0},  // vx (30..35)
    {40.0, 41.0, 42.0, 43.0, 44.0, 45.0}   // relative_time (40..45)
  );
  geometry_msgs::msg::Pose current_pose = create_pose(1.1, 11.1, 0.11);
  geometry_msgs::msg::Pose nearest_pose;
  ;
  double nearest_time;

  size_t nearest_idx = 1;

  // expected results should be interpolated to the nearest pose nad index
  auto result = MPCUtils::calcNearestPoseInterp(
    trajectory, current_pose, &nearest_pose, &nearest_idx, &nearest_time);
  ASSERT_TRUE(result) << "Should return true for valid nearest index";
  ASSERT_NEAR(nearest_pose.position.x, 1.1, 1e-6)
    << "interpolated position should be close to current pose";
  ASSERT_NEAR(nearest_pose.position.y, 11.1, 1e-6)
    << "interpolated position should be close to current pose";
  ASSERT_NEAR(autoware_utils::get_rpy(nearest_pose.orientation).z, 0.11, 1e-6)
    << "interpolated yaw should be close to current pose";
  ASSERT_NEAR(nearest_time, 41.1, 1e-6) << "interpolated time should be close to current pose";
}

/* cppcheck-suppress syntaxError */
TEST(TestMPC, calcTrajectoryCurvature_duplicate_points)
{
  MPCTrajectory trajectory;
  trajectory.push_back(1.0, 1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0);
  trajectory.push_back(1.0, 1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0);
  trajectory.push_back(1.0, 1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0);

  const auto curvature = MPCUtils::calcTrajectoryCurvature(1, trajectory);

  ASSERT_EQ(curvature.size(), trajectory.size());
  for (const auto value : curvature) {
    EXPECT_DOUBLE_EQ(value, 0.0);
  }
}

}  // namespace
}  // namespace autoware::motion::control::mpc_lateral_controller
