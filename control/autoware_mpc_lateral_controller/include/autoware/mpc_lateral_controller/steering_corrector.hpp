// Copyright (c) 2025 Autonomous Systems Sp. z o.o.
// All rights reserved. Proprietary and confidential.

#ifndef AS_AUTOWARE__MPC_LATERAL_CONTROLLER__STEERING_CORRECTOR_HPP_
#define AS_AUTOWARE__MPC_LATERAL_CONTROLLER__STEERING_CORRECTOR_HPP_

#include "autoware/mpc_lateral_controller/mpc_trajectory.hpp"
#include "autoware/mpc_lateral_controller/vehicle_model/vehicle_model_interface.hpp"

#include <Eigen/Core>

#include <memory>

namespace autoware::motion::control::mpc_lateral_controller
{
struct MPCMatrix;  // forward declaration from mpc.hpp

struct SteeringCorrectorParams
{
  /**
   * @brief Maximum steering angle of the vehicle (rover-model wheel).
   */
  double steer_limit;

  /**
   * @brief Maximum correction that can be applied to single point.
   */
  double correction_limit;

  /**
   * @brief Minimum distance between points in world coordinates to calculate correction.
   */
  double min_distance;

  /**
   * @brief Maximum heading difference. If calculated correction exceeds this value, no correction
   * is applied.
   */
  double max_heading_diff;
};

/**
 * @brief Corrects MPC trajectory to better fit in world coordinates.
 * MPC optimization is generated on frenet coordinates, after conversion to world coords, due to
 * numeric error propagation, points may be distant from the trajectory that vehicle really follows.
 * Check:
 * https://autonomous-systems.atlassian.net/wiki/spaces/SDP/pages/3527540741/MPC+-+testy+z+korekt+trajektorii
 */
class SteeringCorrector
{
public:
  /**
   * Corrector constructor.
   */
  explicit SteeringCorrector(
    std::shared_ptr<VehicleModelInterface> vehicle_model_ptr,
    const SteeringCorrectorParams & params);

  /**
   * @brief Generates trajectory with corrected steering angle that better fits in world
   coordinates.
   * @param reference_trajectory Planned trajectory.
     @param frenet_trajectory Trajectory in frenet coordinates from MPC optimization.
     @param Uex MPC optimization matrix.
     @param dt Prediction time step.
     @return MPC optimization result with corrected steering.
   */
  const Eigen::VectorXd calculate(
    const MPCTrajectory & reference_trajectory, MPCMatrix mpc_matrix, Eigen::VectorXd initial_state,
    const Eigen::MatrixXd & Uex, const double dt);

private:
  const std::shared_ptr<VehicleModelInterface> m_vehicle_model_ptr;
  const double m_steer_limit;
  const double m_correction_limit;
  const double m_min_distance_squared;
  const double m_max_heading_diff;

  /**
   * @brief Updates state in the world coordinate.
   *        Taken from: KinematicsBicycleModel::calculatePredictedTrajectoryInWorldCoordinate.
   */
  const Eigen::Vector3d updateState(
    const Eigen::Vector3d & state, const double & input, const double dt, const double velocity);

  /**
   * @brief Calculates steering correction for single step.
   */
  double calculateInputAngleCorrection(
    const Eigen::Vector3d & state, size_t current_idx, const Eigen::MatrixXd & Uex, const double dt,
    const MPCTrajectory & reference_trajectory, const MPCTrajectory & frenet_trajectory);

  friend class SteeringCorrectorTest;
};
}  // namespace autoware::motion::control::mpc_lateral_controller

#endif  // AS_AUTOWARE__MPC_LATERAL_CONTROLLER__STEERING_CORRECTOR_HPP_
