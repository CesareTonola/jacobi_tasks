/**
 * @file ik_solver_jacobi.hpp
 * @brief Inverse kinematics solver using Pinocchio.
 *
 */

#ifndef IK_SOLVER_JACOBI_HPP
#define IK_SOLVER_JACOBI_HPP

#include <iostream>

#include <pinocchio/parsers/urdf.hpp>
#include <pinocchio/algorithm/kinematics.hpp>
#include <pinocchio/algorithm/jacobian.hpp>
#include <pinocchio/algorithm/joint-configuration.hpp>
#include <pinocchio/algorithm/frames.hpp>

namespace jacobi
{

/**
 * @struct ik_solver_options
 * @brief Options for configuring the inverse kinematics solver.
 */
struct ik_solver_options
{
  /// Minimum norm of the twist error to consider convergence.
  double min_error = 1e-4;

  /// Maximum number of iterations allowed.
  int max_iterations = 1000;

  /// Integration step size for the configuration update.
  double DT = 1e-1;

  /// Damping factor added to the pseudo-inverse of the Jacobian to improve numerical stability.
  double damp = 1e-6;
};

/**
 * @class IkSolver
 * @brief Inverse kinematics solver based on Pinocchio.
 *
 * This class allows computing inverse kinematics solutions to reach a given
 * target pose for a specified frame of a robot defined by a URDF model.
 */
class IkSolver
{
protected:

  /// Path to the URDF file used to build the robot model.
  std::string urdf_file_;

  /// The Pinocchio kinematic model.
  pinocchio::Model model_;

public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  /// Verbosity of the algorithm
  bool verbose_ = false;

  /**
    * @brief Constructor that loads the robot model from a URDF file.
    * @param urdf_file The path to the robot's URDF file.
    */
  IkSolver(const std::string& urdf_file);

  /**
   * @brief Returns a read-only reference to the Pinocchio kinematic model of the robot.
   * @return A const reference to the internal pinocchio::Model.
   */
  const pinocchio::Model& get_model() const;

  /**
   * @brief Returns a copy of the Pinocchio kinematic model of the robot.
   * @return A copy of the internal pinocchio::Model.
   */
  pinocchio::Model get_model_copy() const;

  /**
   * @brief Generates a random reachable pose for a given frame of the robot.
   *
   * @param frame_name The name of the frame for which to compute the pose.
   * @param pose_target Output target pose in the world frame.
   */
  void get_random_pose_target(const std::string& frame_name, pinocchio::SE3& pose_target);

  /**
    * @brief Computes inverse kinematics to reach a target pose.
    *
    * @param frame_name Name of the frame to control.
    * @param pose_target Desired pose (in world coordinates).
    * @param options Parameters controlling convergence and damping.
    * @param seed Initial configuration to start the solver from.
    *  If not provided, a random configuration is considered.
    * @param ik Output joint configuration achieving the target.
    * @return True if IK converged successfully, false otherwise.
    */
  bool compute_ik(const std::string &frame_name,
                  const pinocchio::SE3& pose_target,
                  const ik_solver_options &options,
                  const Eigen::VectorXd &seed,
                  Eigen::VectorXd& ik);

  bool compute_ik(const std::string &frame_name,
                  const pinocchio::SE3& pose_target,
                  const ik_solver_options &options,
                  Eigen::VectorXd& ik);

};

}

#endif // IK_SOLVER_JACOBI_HPP
