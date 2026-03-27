// Copyright (c) 2025 Autonomous Systems Sp. z o.o.
// All rights reserved. Proprietary and confidential.

#include "autoware/mpc_lateral_controller/mpc_trajectory.hpp"
#include "autoware/mpc_lateral_controller/steering_corrector.hpp"
#include "autoware/mpc_lateral_controller/vehicle_model/vehicle_model_bicycle_kinematics.hpp"
#include "gtest/gtest.h"

#include <Eigen/Core>
#include <autoware_utils/math/unit_conversion.hpp>
#include <autoware_utils_math/unit_conversion.hpp>

#include <Eigen/src/Core/Matrix.h>

#include <memory>

namespace autoware::motion::control::mpc_lateral_controller
{
struct PointWithVelocity
{
  double x;
  double y;
  double vx;
  explicit PointWithVelocity(double x_val, double y_val, double vx_val)
  : x(x_val), y(y_val), vx(vx_val)
  {
  }
};

class SteeringCorrectorTest : public ::testing::Test
{
public:
  auto updateState(
    const Eigen::Vector3d & state, const double & input, const double dt, const double velocity)
  {
    return corrector_->updateState(state, input, dt, velocity);
  }

  auto calculateInputAngleCorrection(
    const Eigen::Vector3d & state, size_t current_idx, const Eigen::MatrixXd & Uex, const double dt,
    const MPCTrajectory & reference_trajectory, const MPCTrajectory & frenet_trajectory,
    bool is_forward_shift)
  {
    return corrector_->calculateInputAngleCorrection(
      state, current_idx, Uex, dt, reference_trajectory, frenet_trajectory, is_forward_shift);
  }

protected:
  void SetUp() override
  {
    vehicle_model_ = std::make_shared<KinematicsBicycleModel>(WHEELBASE, STEER_LIMIT, STEER_TAU);

    SteeringCorrectorParams params;
    params.steer_limit = STEER_LIMIT;
    params.correction_limit = CORRECTION_LIMIT;
    params.min_distance = MIN_DISTANCE;
    params.max_heading_diff = MAX_HEADING_DIFF;
    corrector_ = std::make_shared<SteeringCorrector>(vehicle_model_, params);

    velocity_ = autoware_utils::kmph2mps(10.0);
    // Reference trajectory, only velocity component is relevant for correction
    reference_trajectory_ = generate_trajectory({
      PointWithVelocity(0, 0, velocity_),
      PointWithVelocity(0, 0, velocity_),
      PointWithVelocity(0, 0, velocity_),
      PointWithVelocity(0, 0, velocity_),
      PointWithVelocity(0, 0, velocity_),
    });
  }

  void TearDown() override {}

  Eigen::Vector3d generate_state(double x, double y, double z)
  {
    Eigen::Vector3d state = Eigen::Vector3d::Zero();
    state.x() = x;
    state.y() = y;
    state.z() = z;
    return state;
  }

  Eigen::MatrixXd generate_uex(std::vector<double> steering_commands)
  {
    const auto DIM_U = vehicle_model_->getDimU();
    Eigen::MatrixXd uex = Eigen::VectorXd::Zero(steering_commands.size() * DIM_U, 1);
    for (size_t i = 0; i < steering_commands.size(); ++i) {
      uex(i * DIM_U, 0) = steering_commands.at(i);
    }
    return uex;
  }

  MPCTrajectory generate_trajectory(std::vector<PointWithVelocity> points)
  {
    MPCTrajectory trajectory;
    for (const auto & point : points) {
      trajectory.push_back(point.x, point.y, 0.0, 0.0, point.vx, 0.0, 0.0, 0.0);
    }
    return trajectory;
  }

  static constexpr const auto WHEELBASE = 2.7;
  static constexpr const auto STEER_LIMIT = autoware_utils::deg2rad(60.0);
  static constexpr const auto STEER_TAU = 0.3;
  static constexpr const auto CORRECTION_LIMIT = autoware_utils::deg2rad(5.0);
  static constexpr const auto MIN_DISTANCE = 0.1;
  static constexpr const auto MAX_HEADING_DIFF = autoware_utils::deg2rad(90.0);
  static constexpr const auto DT = 0.1;

  std::shared_ptr<VehicleModelInterface> vehicle_model_;
  std::shared_ptr<SteeringCorrector> corrector_;
  double velocity_;
  MPCTrajectory reference_trajectory_;
};

/* cppcheck-suppress syntaxError */
TEST_F(SteeringCorrectorTest, updateState)
{
  Eigen::Vector3d state = generate_state(1.23, 3.45, autoware_utils::deg2rad(30.0));
  auto input = autoware_utils::deg2rad(10.0);

  // Calculate expected state as rotation over Z axis in DT time using ackermann model
  // v = 10 km/h = 2.777... m/s
  // angle = tan(input) / WHEELBASE = tan(10 deg) / 2.7 m = 0.065 rad/m
  // 30 deg = 0.523 rad
  // cos(30 deg) = 0.866, sin(30 deg) = 0.500
  // x` = x + v * cos(z) * dt = 1.23 + 2.777 * 0.866 * 0.1 = 1.23 + 0.2405 = 1.4705
  // y` = y + v * sin(z) * dt = 3.45 + 2.777 * 0.500 * 0.1 = 3.45 + 0.1389 = 3.5889
  // z` = z + v * angle * dt = 0.523 + 2.777 * 0.065 * 0.1 = 0.523 + 0.018 = 0.541
  Eigen::Vector3d expected = generate_state(1.4705, 3.5889, 0.541);
  expected.y() = 3.5889;
  expected.z() = 0.541;

  auto result = updateState(state, input, DT, velocity_);
  EXPECT_NEAR(expected.x(), result.x(), 1e-3);
  EXPECT_NEAR(expected.y(), result.y(), 1e-3);
  EXPECT_NEAR(expected.z(), result.z(), 1e-3);
}

/* cppcheck-suppress syntaxError */
TEST_F(SteeringCorrectorTest, calculateInputAngleCorrection_slight_left_correction)
{
  // Current state: position [0, 0], heading straight
  Eigen::Vector3d state = generate_state(0.0, 0.0, autoware_utils::deg2rad(0.0));
  // Optimization result - driving straight forward
  auto uex = generate_uex({0.0, 0.0, 0.0, 0.0, 0.0});

  auto current_idx = 1;

  // Frenet trajectory slightly curved to the left
  auto frenet_trajectory = generate_trajectory({
    PointWithVelocity(0.0, 0.00, velocity_),
    PointWithVelocity(0.5, 0.01, velocity_),
    PointWithVelocity(1.0, 0.02, velocity_),
    PointWithVelocity(1.5, 0.04, velocity_),
    PointWithVelocity(2.0, 0.08, velocity_),
  });

  auto result = calculateInputAngleCorrection(
    state, current_idx, uex, DT, reference_trajectory_, frenet_trajectory, true);

  // Vehicle oriented straight forward, but trajectory is slightly to the left, so correction to the
  // left expected.
  // Expected correction: idx=1, 3 point ahead, differences: x=2.0, y=0.08 ->
  // atan2(0.08,2.0)=0.04 rad
  auto expected_correction = std::atan2(0.08, 2.0);
  EXPECT_NEAR(expected_correction, result, 1e-3);
}

/* cppcheck-suppress syntaxError */
TEST_F(SteeringCorrectorTest, calculateInputAngleCorrection_slight_right_correction)
{
  // Current state: position [0, 0], heading straight
  Eigen::Vector3d state = generate_state(0.0, 0.0, autoware_utils::deg2rad(0.0));
  // Optimization result - driving straight forward
  auto uex = generate_uex({0.0, 0.0, 0.0, 0.0, 0.0});

  auto current_idx = 1;

  // Frenet trajectory slightly curved to the left
  auto frenet_trajectory = generate_trajectory({
    PointWithVelocity(0.0, 0.00, velocity_),
    PointWithVelocity(0.5, -0.01, velocity_),
    PointWithVelocity(1.0, -0.02, velocity_),
    PointWithVelocity(1.5, -0.04, velocity_),
    PointWithVelocity(2.0, -0.08, velocity_),
  });

  auto result = calculateInputAngleCorrection(
    state, current_idx, uex, DT, reference_trajectory_, frenet_trajectory, true);

  // Vehicle oriented straight forward, but trajectory is slightly to the left, so correction to the
  // left expected.
  // Expected correction: idx=1, 3 point ahead, differences: x=2.0, y=0.08 ->
  // atan2(0.08,2.0)=0.04 rad
  auto expected_correction = -std::atan2(0.08, 2.0);
  EXPECT_NEAR(expected_correction, result, 1e-3);
}

/* cppcheck-suppress syntaxError */
TEST_F(
  SteeringCorrectorTest,
  calculateInputAngleCorrection_slight_right_correction_due_to_current_state_steering_wheel_offset)
{
  // Current state: position [0, 0], heading 5 deg to he left
  Eigen::Vector3d state = generate_state(0.0, 0.0, autoware_utils::deg2rad(5.0));
  // Optimization result - driving straight forward
  auto uex = generate_uex({0.0, 0.0, 0.0, 0.0, 0.0});

  auto current_idx = 1;

  // Frenet trajectory slightly curved to the left
  auto frenet_trajectory = generate_trajectory({
    PointWithVelocity(0.0, 0.00, velocity_),
    PointWithVelocity(0.5, 0.01, velocity_),
    PointWithVelocity(1.0, 0.02, velocity_),
    PointWithVelocity(1.5, 0.04, velocity_),
    PointWithVelocity(2.0, 0.08, velocity_),
  });

  auto result = calculateInputAngleCorrection(
    state, current_idx, uex, DT, reference_trajectory_, frenet_trajectory, true);

  // Vehicle oriented 10 deg to the left, but trajectory is slightly to the left, so correction to
  // the right expected.
  // Frenet vector (trajectory[current_idx+3]) = [2.0, 0.08]
  // State vector (cos(5°)*3*DT*velocity, sin(5°)*3*DT*velocity) = [0.830, 0.073]
  // expected correction, angle between state and frenet vector: -0.0473
  auto expected_correction = -0.0473;
  EXPECT_NEAR(expected_correction, result, 1e-3);
}

/* cppcheck-suppress syntaxError */
TEST_F(SteeringCorrectorTest, calculateInputAngleCorrection_slight_right_correction_backward)
{
  // Current state: position [0, 0], heading straight
  Eigen::Vector3d state = generate_state(0.0, 0.0, autoware_utils::deg2rad(0.0));
  // Optimization result - driving straight forward
  auto uex = generate_uex({0.0, 0.0, 0.0, 0.0, 0.0});

  auto current_idx = 1;

  // Frenet trajectory slightly curved to the left
  auto frenet_trajectory = generate_trajectory({
    PointWithVelocity(0.0, 0.00, velocity_),
    PointWithVelocity(0.5, 0.01, velocity_),
    PointWithVelocity(1.0, 0.02, velocity_),
    PointWithVelocity(1.5, 0.04, velocity_),
    PointWithVelocity(2.0, 0.08, velocity_),
  });

  auto result = calculateInputAngleCorrection(
    state, current_idx, uex, DT, reference_trajectory_, frenet_trajectory, false);

  // Vehicle oriented straight forward, but trajectory is slightly to the left.
  // Vehicle is driving backward, so correction to the right expected.
  // Expected correction: idx=1, 3 point ahead, differences: x=2.0, y=0.08 ->
  // atan2(0.08,2.0)=0.04 rad
  auto expected_correction = -std::atan2(0.08, 2.0);
  EXPECT_NEAR(expected_correction, result, 1e-3);
}

/* cppcheck-suppress syntaxError */
TEST_F(
  SteeringCorrectorTest,
  calculateInputAngleCorrection_no_correction_required_due_to_trajectory_aligned_with_current_state)
{
  // Current state: position [0, 0], heading forward
  Eigen::Vector3d state = generate_state(0.0, 0.0, autoware_utils::deg2rad(45.0));
  // Optimization result - driving straight forward
  auto uex = generate_uex({0.0, 0.0, 0.0, 0.0, 0.0});
  auto current_idx = 1;

  // Frenet trajectory is straight forward in the direction of 45 deg, so no correction expected
  auto frenet_trajectory = generate_trajectory({
    PointWithVelocity(0.0, 0.0, velocity_),
    PointWithVelocity(0.2, 0.2, velocity_),
    PointWithVelocity(0.4, 0.4, velocity_),
    PointWithVelocity(0.6, 0.6, velocity_),
    PointWithVelocity(0.8, 0.8, velocity_),
  });

  auto result = calculateInputAngleCorrection(
    state, current_idx, uex, DT, reference_trajectory_, frenet_trajectory, true);

  // both trajectories are the same, initial state has correct heading, no correction expected
  auto expected_correction = 0.0;
  EXPECT_NEAR(expected_correction, result, 1e-3);
}

/* cppcheck-suppress syntaxError */
TEST_F(SteeringCorrectorTest, calculateInputAngleCorrection_no_correction_for_first_index)
{
  // Current state: position [0, 0], heading forward
  Eigen::Vector3d state = generate_state(0.0, 0.0, autoware_utils::deg2rad(0.0));
  // Optimization result - driving straight forward
  auto uex = generate_uex({0.0, 0.0, 0.0, 0.0, 0.0});
  auto current_idx = 0;

  // Frenet trajectory slightly curved to the right
  auto frenet_trajectory = generate_trajectory({
    PointWithVelocity(0.0, 0.0, velocity_),
    PointWithVelocity(0.2, 0.2, velocity_),
    PointWithVelocity(0.4, 0.4, velocity_),
    PointWithVelocity(0.6, 0.6, velocity_),
    PointWithVelocity(0.8, 0.8, velocity_),
  });

  auto result = calculateInputAngleCorrection(
    state, current_idx, uex, DT, reference_trajectory_, frenet_trajectory, true);

  // no correction expected for the first index
  auto expected_correction = 0.0;
  EXPECT_NEAR(expected_correction, result, 1e-3);
}

/* cppcheck-suppress syntaxError */
TEST_F(SteeringCorrectorTest, calculateInputAngleCorrection_limit_correction)
{
  // Current state: position [0, 0], heading forward
  Eigen::Vector3d state = generate_state(0.0, 0.0, autoware_utils::deg2rad(0.0));

  // Optimization result - steering straight forward
  auto uex = generate_uex({0.0, 0.0, 0.0, 0.0, 0.0});
  auto current_idx = 1;

  // Frenet trajectory slightly curved to the right
  auto frenet_trajectory = generate_trajectory({
    PointWithVelocity(0.0, -0.0, velocity_),
    PointWithVelocity(0.2, -0.2, velocity_),
    PointWithVelocity(0.4, -0.4, velocity_),
    PointWithVelocity(0.6, -0.6, velocity_),
    PointWithVelocity(0.8, -0.8, velocity_),
  });

  auto result = calculateInputAngleCorrection(
    state, current_idx, uex, DT, reference_trajectory_, frenet_trajectory, true);

  // Frenet trajectory goes 45° to the right, correction is limited to 5 deg, so expected correction
  // is -5 deg.
  auto expected_correction = -CORRECTION_LIMIT;
  EXPECT_NEAR(expected_correction, result, 1e-3);
}

}  // namespace autoware::motion::control::mpc_lateral_controller
