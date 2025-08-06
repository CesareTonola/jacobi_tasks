#include <chrono>
#include "ik_solver/ik_solver.hpp"

int main(int argc, char **argv)
{
  std::string urdf_file = std::string(IK_SOLVER_DIR)+"/../robot-urdfs/jacobi_robot.urdf";
  jacobi::IkSolver solver(urdf_file);
  std::string frame_name = "abb_irb6700_flange";

  std::cout <<"Computing IK for '"<<frame_name<<"' frame"<<std::endl;

  jacobi::ik_solver_options options; //default options

  solver.verbose_ = false;

  while(true)
  {
    // generate random pose target
    pinocchio::SE3 pose_target;
    solver.get_random_pose_target(frame_name,pose_target);

    bool success;
    unsigned int iter = 0;
    unsigned int max_iter = 50;

    auto t_start = std::chrono::high_resolution_clock::now();
    while(iter<max_iter)
    {
      // compute ik
      Eigen::VectorXd ik;
      success = solver.compute_ik(frame_name,pose_target,options,ik);

      if(success)
      {
        auto t_end = std::chrono::high_resolution_clock::now();
        std::chrono::duration<double> elapsed = t_end - t_start;

        std::cout<<"Solution found in "<<iter+1<<" iterations"<<std::endl;
        std::cout << "Time to find a solution: " << elapsed.count() * 1000.0 << " ms" << std::endl;
        std::cout<<"IK: "<<ik.transpose()<<std::endl;

        std::cout<<"Checking the result:"<<std::endl;
        pinocchio::Model model = solver.get_model_copy();

        // compare the computed pose with the target pose
        pinocchio::Data data(model);
        pinocchio::forwardKinematics(model, data, ik);
        pinocchio::updateFramePlacements(model, data);

        const int frame_id = model.getFrameId(frame_name);
        const pinocchio::SE3& computed_pose = data.oMf[frame_id];
        const pinocchio::SE3 err_pose = computed_pose.inverse() * pose_target;
        const Eigen::Matrix<double,6,1> final_twist_error = pinocchio::log6(err_pose).toVector();

        std::cout << "Target pose viewed from computed ee pose:" << std::endl;
        std::cout << "Rotation:\n" << err_pose.rotation() << std::endl;
        std::cout << "Translation: " << err_pose.translation().transpose() << std::endl;

        std::cout << "Final 6D error (twist): " << final_twist_error.transpose() << std::endl;
        std::cout << "Final error norm: " << final_twist_error.norm() << std::endl;

        assert([& ]() ->bool{
          for (int j = 0; j < model.nv; ++j)
          {
            if (ik[j] < model.lowerPositionLimit[j])
            {
              std::cout<<"ik not respecting lower bounds! LB: "<<model.lowerPositionLimit.transpose()<<std::endl;
              return false;
            }
            else if (ik[j] > model.upperPositionLimit[j])
            {
              std::cout<<"ik not respecting upper bounds! LB: "<<model.upperPositionLimit.transpose()<<std::endl;
              return false;
            }
          }
          std::cout<<"Bounds check passed"<<std::endl;
          return true;
        }());

        break; // exit and start again with a new pose
      }

      iter++;
    }

    if(!success)
      std::cout<<"Solution not found!"<<std::endl;

    std::cout << "Press Enter to generate another pose target...";
    std::cin.ignore(std::numeric_limits<std::streamsize>::max(), '\n');
    std::cout<<"------------------------------------------------------"<<std::endl;
  }

  return 0;
}
