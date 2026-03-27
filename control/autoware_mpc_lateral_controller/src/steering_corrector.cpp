// Copyright (c) 2025 Autonomous Systems Sp. z o.o.
// All rights reserved. Proprietary and confidential.

#include "autoware/mpc_lateral_controller/steering_corrector.hpp"

#include "autoware/mpc_lateral_controller/mpc.hpp"
#include "autoware/mpc_lateral_controller/mpc_trajectory.hpp"

#include <Eigen/Core>

namespace autoware::motion::control::mpc_lateral_controller
{
SteeringCorrector::SteeringCorrector(
  std::shared_ptr<VehicleModelInterface> vehicle_model_ptr, const SteeringCorrectorParams & params)
: m_vehicle_model_ptr(vehicle_model_ptr),
  m_steer_limit(params.steer_limit),
  m_correction_limit(params.correction_limit),
  m_min_distance_squared(params.min_distance * params.min_distance),
  m_max_heading_diff(params.max_heading_diff)
{
}

const Eigen::VectorXd SteeringCorrector::calculate(
  const MPCTrajectory & reference_trajectory, MPCMatrix mpc_matrix, Eigen::VectorXd initial_state,
  const Eigen::MatrixXd & Uex, const double dt)
{
  const auto DIM_U = m_vehicle_model_ptr->getDimU();

  // Trajectory to which the reference trajectory will be matched
  auto frenet_trajectory = m_vehicle_model_ptr->calculatePredictedTrajectoryInFrenetCoordinate(
    mpc_matrix.Aex, mpc_matrix.Bex, mpc_matrix.Cex, mpc_matrix.Wex, initial_state, Uex,
    reference_trajectory, dt);

  std::vector<double> new_steering;
  new_steering.reserve(reference_trajectory.size());

  // create initial state in the world coordinate
  // calculating trajectory in world coordinates starts with the same pose as in frenet trajectory
  Eigen::Vector3d state_w = Eigen::Vector3d::Zero();
  state_w.x() = frenet_trajectory.x.at(0);
  state_w.y() = frenet_trajectory.y.at(0);
  state_w.z() = frenet_trajectory.yaw.at(0);

  for (size_t i = 0; i < reference_trajectory.size(); ++i) {
    auto input = Uex(i * DIM_U, 0);
    auto input_d =
      calculateInputAngleCorrection(state_w, i, Uex, dt, reference_trajectory, frenet_trajectory);
    auto new_input = std::clamp(input + input_d, -m_steer_limit, m_steer_limit);
    new_steering.push_back(new_input);

    state_w = updateState(state_w, new_input, dt, reference_trajectory.vx.at(i));
  }

  return Eigen::Map<Eigen::VectorXd, Eigen::Unaligned>(new_steering.data(), new_steering.size());
}

const Eigen::Vector3d SteeringCorrector::updateState(
  const Eigen::Vector3d & state, const double & input, const double dt, const double velocity)
{
  const auto yaw = state(2);
  const auto WHEELBASE = m_vehicle_model_ptr->getWheelbase();

  Eigen::Vector3d dstate;
  dstate.x() = velocity * std::cos(yaw);
  dstate.y() = velocity * std::sin(yaw);
  dstate.z() = velocity * std::tan(input) / WHEELBASE;

  // Note: don't do "return state_w + dstate * dt", which does not work due to the lazy evaluation
  // in Eigen.
  const Eigen::Vector3d next_state = state + dstate * dt;
  return next_state;
};

double SteeringCorrector::calculateInputAngleCorrection(
  const Eigen::Vector3d & state, size_t current_idx, const Eigen::MatrixXd & Uex, const double dt,
  const MPCTrajectory & reference_trajectory, const MPCTrajectory & frenet_trajectory)
{
  const auto DIM_U = m_vehicle_model_ptr->getDimU();

  auto w_x0 = state(0);
  auto w_y0 = state(1);
  auto state_wk = state;

  if (current_idx <= 0) {
    return 0.0;
  }

  // Find index where distance to current trajectory point is large enough.
  // Minimum 2 points are required, current control affects the next section.
  for (size_t j = 0, k = current_idx; k < reference_trajectory.size() - 1;) {
    auto input_k = Uex(k * DIM_U, 0);
    state_wk = updateState(state_wk, input_k, dt, reference_trajectory.vx.at(k));
    ++j;
    ++k;
    auto wk_x = state_wk(0);
    auto wk_y = state_wk(1);
    auto ft_x = frenet_trajectory.x.at(k);
    auto ft_y = frenet_trajectory.y.at(k);
    auto wd_x = wk_x - w_x0;
    auto wd_y = wk_y - w_y0;
    auto fd_x = ft_x - w_x0;
    auto fd_y = ft_y - w_y0;
    auto wd_d = wd_x * wd_x + wd_y * wd_y;
    auto fd_d = fd_x * fd_x + fd_y * fd_y;

    if (j > 2 && wd_d > m_min_distance_squared && fd_d > m_min_distance_squared) {
      auto correction = std::atan2(wd_x * fd_y - wd_y * fd_x, wd_x * fd_x + wd_y * fd_y) * 1.0;
      return std::abs(correction) > m_max_heading_diff
               ? 0.0
               : std::clamp(correction, -m_correction_limit, m_correction_limit);
    }
  }
  return 0.0;
};

}  // namespace autoware::motion::control::mpc_lateral_controller
