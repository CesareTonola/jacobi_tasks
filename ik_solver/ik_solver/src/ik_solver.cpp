#include "ik_solver/ik_solver.hpp"

namespace jacobi
{
IkSolver::IkSolver(const std::string& urdf_file)
{
  // Init seed
  std::srand(std::time(nullptr));

  //Load the model from URDF file
  urdf_file_ = urdf_file;
  pinocchio::urdf::buildModel(urdf_file, model_);

  if(verbose_)
  {
    std::cout << model_.name << " has these joints: " <<std::endl;
    for (int i = 0; i < model_.njoints; ++i)
      std::cout << i << ": " << model_.names[i] << std::endl;

    std::cout << model_.name << " has these frames: " <<std::endl;
    for (int i = 0; i < model_.nframes; ++i)
      std::cout << i << ": " << model_.frames[i].name << std::endl;
  }
}

const pinocchio::Model& IkSolver::get_model() const
{
  return model_;
}

pinocchio::Model IkSolver::get_model_copy() const
{
  return model_;
}

void IkSolver::get_random_pose_target(const std::string& frame_name, pinocchio::SE3& pose_target)
{
  pinocchio::Data data(model_);
  Eigen::VectorXd q = pinocchio::randomConfiguration(model_);
  pinocchio::forwardKinematics(model_, data, q);
  pinocchio::updateFramePlacements(model_, data);

  const int frame_id = model_.getFrameId(frame_name);
  pose_target = data.oMf[frame_id];

  return;
}

bool IkSolver::compute_ik(const std::string& frame_name,
                          const pinocchio::SE3& pose_target,
                          const ik_solver_options& options,
                          Eigen::VectorXd& ik)
{
  Eigen::VectorXd seed = pinocchio::randomConfiguration(model_);
  return this->compute_ik(frame_name,pose_target,options,seed,ik);
}

bool IkSolver::compute_ik(const std::string& frame_name,
                          const pinocchio::SE3& pose_target,
                          const ik_solver_options& options,
                          const Eigen::VectorXd& seed,
                          Eigen::VectorXd& ik)
{
  //Get the frame ID in the model
  const int frame_id = model_.getFrameId(frame_name);

  pinocchio::Data data(model_);
  Eigen::VectorXd dq(model_.nv);
  Eigen::Matrix<double, 6, 1> error_twist;
  pinocchio::Data::Matrix6x J(6, model_.nv);
  J.setZero();

  ik = seed;

  for (int i = 0; i < options.max_iterations; i++)
  {
    pinocchio::forwardKinematics(model_, data, ik);
    pinocchio::updateFramePlacements(model_, data);
    const pinocchio::SE3& current_pose = data.oMf[frame_id]; // espressed in 'world'

    // current_pose⁻¹ * pose_target -> pose_target espressed in end-effector frame
    const pinocchio::SE3 target_in_ee = current_pose.actInv(pose_target);

    error_twist = pinocchio::log6(target_in_ee).toVector(); // espressed in 'end-effector'

    if (error_twist.norm() < options.min_error)
    {
      if(verbose_)
        std::cout<<"ik found!"<<std::endl;

      return true;
    }

    pinocchio::computeFrameJacobian(model_, data, ik, frame_id, J); // jacobian referred to the end-effector

    // Jlog = d(log6(current_pose⁻¹ * target)) / d(current_pose), evaluated at target_in_ee⁻¹
    pinocchio::Data::Matrix6 Jlog;
    pinocchio::Jlog6(target_in_ee.inverse(), Jlog); //jacobian correction
    J = -Jlog * J;

    pinocchio::Data::Matrix6 JJt;
    JJt.noalias() = J * J.transpose();
    JJt.diagonal().array() += options.damp;

    // δq = - Jᵗ · (J·Jᵗ + λ·I)⁻¹ · error_twist
    dq.noalias() = -J.transpose() * JJt.ldlt().solve(error_twist); // JJt.ldlt().solve(error_twist) == JJt⁻¹ · error_twist
    ik = pinocchio::integrate(model_, ik, dq * options.DT); // qₖ₊₁ = qₖ + δq· Δt

    // force joint bounds
    for (int j = 0; j < model_.nv; ++j)
    {
      if (ik[j] < model_.lowerPositionLimit[j])
        ik[j] = model_.lowerPositionLimit[j];
      else if (ik[j] > model_.upperPositionLimit[j])
        ik[j] = model_.upperPositionLimit[j];
    }

    if ((i % 10)==0 && verbose_)
      std::cout << i << ": error = " << error_twist.transpose() << std::endl;
  }

  if(verbose_)
    std::cout<<"ik NOT found!"<<std::endl;

  return false;
}


}
