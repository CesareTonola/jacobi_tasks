#include <pybind11/pybind11.h>
#include <pybind11/eigen.h>
#include <pinocchio/fwd.hpp>
#include "ik_solver/ik_solver.hpp"

namespace py = pybind11;
using namespace jacobi;

PYBIND11_MODULE(ik_solver_py, m)
{
  py::class_<ik_solver_options>(m, "IkSolverOptions")
      .def(py::init<>())
      .def_readwrite("min_error", &ik_solver_options::min_error)
      .def_readwrite("max_iterations", &ik_solver_options::max_iterations)
      .def_readwrite("DT", &ik_solver_options::DT)
      .def_readwrite("damp", &ik_solver_options::damp);

  py::class_<IkSolver>(m, "IkSolver")
      .def(py::init<const std::string &>())

      .def("get_random_pose_target", [](IkSolver &self, const std::string &frame_name)
  {
    pinocchio::SE3 pose_target;
    self.get_random_pose_target(frame_name, pose_target);
    return pose_target.toHomogeneousMatrix();
  })

  .def("compute_ik",
       [](IkSolver &self,
       const std::string &frame_name,
       const Eigen::Matrix4d &M,
       const ik_solver_options &options)
  {
    pinocchio::SE3 pose_target(M);
    Eigen::VectorXd ik(self.get_model().nq);
    bool success = self.compute_ik(frame_name, pose_target, options, ik);
    return py::make_tuple(success, ik);
  },
  py::arg("frame_name"),
      py::arg("pose_matrix"),
      py::arg("options"))


      .def_readwrite("verbose_", &IkSolver::verbose_);
}
